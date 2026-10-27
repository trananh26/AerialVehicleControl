"""ROS 2 node bridging Gazebo UWB ranging to ArduPilot's Pozyx backend.

The node plays the role of the ``IndoorLoiter.ino`` Arduino sketch:

1. Subscribe to the ranging messages published by the ``uwb_gz_plugin``.
2. Solve for the tag position by weighted least squares.
3. Pack the results into the Pozyx binary protocol and send them to the
   ArduPilot SITL serial port over TCP.

Both the beacon configuration and the vehicle position are sent, mirroring the
reference sketch.  The distances are sent **continuously** at the configured
rate, keeping the last known value for an anchor when a fresh ranging is not
available, because the EKF needs a continuous stream of measurements before
it will align to the beacon system.

.. warning::
   The vehicle position accuracy is sent in **metres**, not millimetres.
   See :func:`uwb_pozyx_bridge.pozyx_protocol.encode_vehicle_position` for the
   upstream unit mismatch this works around.
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

from .pozyx_protocol import (
    encode_beacon_config,
    encode_beacon_distance,
    encode_vehicle_position,
)
from .pozyx_sender import PozyxSender
from .trilateration import Anchor, RangeMeasurement, solve_position
from .uwb_decoding import DEFAULT_ANCHOR_OFFSETS_MM, decode_ranging_pose


class PozyxBridgeNode(Node):
    """Reads UWB ranging and drives the ArduPilot Pozyx beacon backend."""

    def __init__(self) -> None:
        super().__init__("pozyx_node")

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

        # Accuracy estimate in metres.  ArduPilot's Pozyx backend forwards
        # this value to the EKF unchanged, and the EKF compares it against a
        # 1.0 m threshold, so it has to be in metres.
        self.declare_parameter("position_accuracy_m", 0.05)

        # Output rate.
        self.declare_parameter("send_rate", 20.0)
        self.declare_parameter("config_interval", 1.0)

        # ----- members -----
        self._lock = threading.Lock()
        self._anchors = self._load_anchors()
        self._ranges: dict[int, RangeMeasurement] = {}
        self._last_config_send = 0.0
        self._last_solution = None
        self._num_solutions = 0

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
            PoseStamped, self.get_parameter("output_pose_topic").value, 10
        )

        # ----- transport -----
        self._sender = PozyxSender(
            host=self.get_parameter("serial_host").value,
            port=int(self.get_parameter("serial_port").value),
        )
        self._sender.connect(timeout=0.0)  # non-blocking first attempt

        period = 1.0 / float(self.get_parameter("send_rate").value)
        self._timer = self.create_timer(period, self._send_cycle)

        self.get_logger().info(
            f"pozyx_node ready: anchors={len(self._anchors)} "
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
        """One output cycle: solve, encode and transmit."""
        now = time.monotonic()

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

        frames = bytearray()

        # The anchor configuration is re-sent periodically so that a restarted
        # ArduPilot process picks it up again.
        config_interval = float(self.get_parameter("config_interval").value)
        if now - self._last_config_send >= config_interval:
            self._last_config_send = now
            for anchor in self._anchors:
                frames += encode_beacon_config(
                    anchor.id,
                    len(self._anchors),
                    int(round(anchor.position[0] * 1000.0)),
                    int(round(anchor.position[1] * 1000.0)),
                    int(round(anchor.position[2] * 1000.0)),
                )

        # Distances, one frame per anchor.
        for anchor in self._anchors:
            measurement = self._latest_range(anchor.id)
            if measurement is None:
                continue
            frames += encode_beacon_distance(
                anchor.id, int(round(measurement.range_m * 1000.0))
            )

        # The vehicle position fix.
        x_mm = int(round(result.position[0] * 1000.0))
        y_mm = int(round(result.position[1] * 1000.0))
        z_mm = int(
            round(
                float(self.get_parameter("tag_altitude_mm").value)
                if bool(self.get_parameter("assume_constant_z").value)
                else result.position[2] * 1000.0
            )
        )
        frames += encode_vehicle_position(
            x_mm,
            y_mm,
            z_mm,
            int(round(float(self.get_parameter("position_accuracy_m").value))),
        )

        self._sender.send(bytes(frames))

        # Periodic diagnostics.
        if self._num_solutions % 200 == 1:
            self.get_logger().info(
                f"tx: solutions={self._num_solutions} "
                f"frames_sent={self._sender.frames_sent} "
                f"dropped={self._sender.frames_dropped} "
                f"pos=({result.position[0]:.3f}, {result.position[1]:.3f}, "
                f"{result.position[2]:.3f}) "
                f"anchors={result.used_anchor_ids}"
            )

    def _latest_range(self, anchor_id: int) -> Optional[RangeMeasurement]:
        """The most recent range measurement for an anchor."""
        with self._lock:
            return self._ranges.get(anchor_id)

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
    node = PozyxBridgeNode()
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
