"""ROS 2 node bridging Gazebo UWB ranging to ArduPilot's external navigation.

The node turns the UWB ranges published by the ``uwb_gz_plugin`` into a
vehicle position and sends it to ArduPilot as a MAVLink ``ODOMETRY`` message
(id 331) over a serial port.

ArduPilot's ``AP_VisualOdom`` backend consumes that message and forwards the
position to EKF3 as an *external navigation* (EXTNAV) measurement, which
makes it the direct replacement for the old Pozyx ``AP_Beacon`` backend.

The pipeline is:

1. Subscribe to the ranging messages published by the ``uwb_gz_plugin``.
2. Solve for the tag position by weighted least squares.
3. Convert the position from the Gazebo ENU frame to the NED frame that
   ArduPilot expects and send it as an ``ODOMETRY`` message.

Only the position is transmitted.  The velocity fields of an ``ODOMETRY``
message are interpreted by ArduPilot in the body FRD frame, and the UWB
ranging data does not provide a velocity estimate, so they are left at zero
and the EKF velocity source stays disabled (``EK3_SRC1_VELXY = 0``).

.. warning::
   The position of an ``ODOMETRY`` message is **not** transformed by
   ArduPilot; it is fused into the EKF as it arrives, and the EKF works in
   NED.  See :mod:`uwb_pozyx_bridge.mavlink_odometry` for the details.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .mavlink_odometry import build_odometry_message, enu_to_ned_m
from .mavlink_sender import MavlinkSender
from .trilateration import Anchor, RangeMeasurement, solve_position
from .uwb_decoding import DEFAULT_ANCHOR_OFFSETS_MM, decode_ranging_pose

#: A position change larger than this between two fixes is treated as a
#: discontinuity and bumps the MAVLink reset counter.  The default is well
#: above the noise of the UWB solution, because a spurious reset makes the
#: EKF drop its velocity estimate.
DEFAULT_RESET_JUMP_THRESHOLD_M = 2.0


class OdometryBridgeNode(Node):
    """Reads UWB ranging and drives ArduPilot's external navigation."""

    def __init__(self) -> None:
        super().__init__("odometry_node")

        # ----- parameters -----
        self.declare_parameter("input_topic", "/uwb/ranging_ros")
        self.declare_parameter("output_pose_topic", "/uwb/pose")
        self.declare_parameter("serial_host", "127.0.0.1")
        self.declare_parameter("serial_port", 5761)

        self.declare_parameter("anchor_count", 4)
        self.declare_parameter(
            "anchor_offsets_mm",
            [v for triple in DEFAULT_ANCHOR_OFFSETS_MM for v in triple],
        )

        # Noise model, in metres.
        self.declare_parameter("los_std", 0.10)
        self.declare_parameter("nlos_soft_std", 0.30)
        self.declare_parameter("nlos_hard_std", 0.60)

        # Solver.
        self.declare_parameter("min_anchors", 3)
        self.declare_parameter("max_range", 50.0)
        self.declare_parameter("assume_constant_z", True)
        self.declare_parameter("tag_altitude_mm", 0.0)
        self.declare_parameter("initial_guess", [0.0, 0.0, 0.0])

        # Standard deviation of the position estimate, in metres.  ArduPilot
        # derives the measurement noise from the covariance matrix of the
        # ODOMETRY message, so this value ends up as the variance of the
        # position entries.
        self.declare_parameter("position_std_m", 0.05)

        # Quality of the estimate, 0 to 100.  ArduPilot only forwards a
        # message to the EKF when the quality is at least VISO_QUAL_MIN.
        self.declare_parameter("quality", 50)

        # A position change larger than this between two fixes bumps the
        # MAVLink reset counter, which makes the EKF re-align instead of
        # fusing the jump as a large innovation.
        self.declare_parameter(
            "reset_jump_threshold_m", DEFAULT_RESET_JUMP_THRESHOLD_M
        )

        # Output rate.
        self.declare_parameter("send_rate", 20.0)

        # ----- members -----
        self._lock = threading.Lock()
        self._anchors = self._load_anchors()
        self._ranges: dict[int, RangeMeasurement] = {}
        self._last_solution = None
        self._num_solutions = 0
        self._reset_counter = 0
        self._last_sent_position: Optional[tuple[float, float, float]] = None

        # ----- QoS -----
        # The C++ bridge publishes with best-effort reliability so that any
        # subscriber can receive; match it here.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._sub = self.create_subscription(
            Pose,
            self.get_parameter("input_topic").value,
            self._on_ranging_pose,
            qos,
        )

        self._pub_pose = self.create_publisher(
            PoseStamped,
            self.get_parameter("output_pose_topic").value,
            qos,
        )

        # ----- transport -----
        self._sender = MavlinkSender(
            host=self.get_parameter("serial_host").value,
            port=int(self.get_parameter("serial_port").value),
        )
        self._sender.connect(timeout=0.0)  # non-blocking first attempt

        period = 1.0 / float(self.get_parameter("send_rate").value)
        self._timer = self.create_timer(period, self._send_cycle)

        self.get_logger().info(
            f"odometry_node ready: anchors={len(self._anchors)} "
            f"input={self.get_parameter('input_topic').value} "
            f"serial={self.get_parameter('serial_host').value}:"
            f"{self.get_parameter('serial_port').value}"
        )

    # ------------------------------------------------------------ helpers

    def _load_anchors(self) -> list[Anchor]:
        """Build the anchor list from the ``anchor_offsets_mm`` parameter."""
        flat = list(self.get_parameter("anchor_offsets_mm").value)
        count = int(self.get_parameter("anchor_count").value)

        if len(flat) < 3 * count:
            self.get_logger().warn(
                "anchor_offsets_mm is shorter than anchor_count*3; "
                "falling back to the default 10x10 m layout."
            )
            flat = [v for triple in DEFAULT_ANCHOR_OFFSETS_MM for v in triple]
            count = 4

        anchors: list[Anchor] = []
        for i in range(count):
            x_mm, y_mm, z_mm = flat[3 * i], flat[3 * i + 1], flat[3 * i + 2]
            anchors.append(
                Anchor(
                    id=i,
                    # Convert to metres for the trilateration solver.
                    position=(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0),
                )
            )
        return anchors

    # ----------------------------------------------------------- callback

    def _on_ranging_pose(self, msg: Pose) -> None:
        """Decode an incoming ranging message and buffer it."""
        measurement = decode_ranging_pose(
            (
                msg.position.x,
                msg.position.y,
                msg.position.z,
            ),
            msg.orientation.x,
        )
        if measurement is None:
            return
        if measurement.anchor_id >= len(self._anchors):
            return

        with self._lock:
            self._ranges[measurement.anchor_id] = measurement

    # -------------------------------------------------------------- cycle

    def _send_cycle(self) -> None:
        """One output cycle: solve, convert and transmit."""
        with self._lock:
            ranges = list(self._ranges.values())

        if not ranges:
            return

        result = solve_position(
            self._anchors,
            ranges,
            los_std=float(self.get_parameter("los_std").value),
            nlos_soft_std=float(self.get_parameter("nlos_soft_std").value),
            nlos_hard_std=float(self.get_parameter("nlos_hard_std").value),
            min_anchors=int(self.get_parameter("min_anchors").value),
            max_range=float(self.get_parameter("max_range").value),
            initial_guess=list(self.get_parameter("initial_guess").value),
            assume_constant_z=bool(
                self.get_parameter("assume_constant_z").value
            ),
        )

        if result is None:
            return

        self._last_solution = result
        self._num_solutions += 1
        self._publish_pose(result)

        # The trilateration result is in the Gazebo ENU frame; ArduPilot
        # fuses the ODOMETRY position as-is in NED, so the horizontal axes
        # are swapped and the vertical axis is flipped here.
        z_m = (
            float(self.get_parameter("tag_altitude_mm").value) / 1000.0
            if bool(self.get_parameter("assume_constant_z").value)
            else result.position[2]
        )
        north_m, east_m, down_m = enu_to_ned_m(
            result.position[0], result.position[1], z_m
        )

        self._detect_reset(north_m, east_m, down_m)

        message = build_odometry_message(
            north_m,
            east_m,
            down_m,
            time.time_ns() // 1000,
            position_std_m=float(self.get_parameter("position_std_m").value),
            quality=int(self.get_parameter("quality").value),
            reset_counter=self._reset_counter,
        )
        self._sender.send(message)

        # Periodic diagnostics.
        if self._num_solutions % 200 == 1:
            self.get_logger().info(
                f"tx: solutions={self._num_solutions} "
                f"messages_sent={self._sender.messages_sent} "
                f"dropped={self._sender.messages_dropped} "
                f"ned=({north_m:.3f}, {east_m:.3f}, {down_m:.3f}) "
                f"anchors={result.used_anchor_ids}"
            )

    def _detect_reset(
        self, north_m: float, east_m: float, down_m: float
    ) -> None:
        """Bump the reset counter when the position jumps discontinuously.

        ArduPilot uses the counter to know that the external navigation
        estimate was reset, so that it re-aligns instead of fusing the jump
        as a large innovation.
        """
        position = (north_m, east_m, down_m)
        previous = self._last_sent_position
        self._last_sent_position = position

        if previous is None:
            return

        jump = math.dist(position, previous)
        threshold = float(self.get_parameter("reset_jump_threshold_m").value)
        if jump > threshold:
            self._reset_counter = (self._reset_counter + 1) % 256
            self.get_logger().warn(
                f"position jump of {jump:.3f} m detected; "
                f"bumping the reset counter to {self._reset_counter}"
            )

    # -------------------------------------------------------------- output

    def _publish_pose(self, result) -> None:
        """Publish the trilateration result for debugging."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = result.position[0]
        msg.pose.position.y = result.position[1]
        msg.pose.position.z = result.position[2]
        msg.pose.orientation.w = 1.0
        self._pub_pose.publish(msg)

    # ------------------------------------------------------------ shutdown

    def destroy_node(self) -> bool:
        self._sender.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = OdometryBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
