"""ROS 2 node bridging Gazebo UWB ranging to ArduPilot's external navigation.

The node turns the UWB ranges published by the ``uwb_gz_plugin`` into a
vehicle position and sends it to ArduPilot as a MAVLink ``ODOMETRY`` message
(id 331) over a serial port.

ArduPilot's ``AP_VisualOdom`` backend consumes that message and forwards the
position to EKF3 as an *external navigation* (EXTNAV) measurement, which
makes it the direct replacement for the old Pozyx ``AP_Beacon`` backend.

The pipeline is:

1. Subscribe to the ranging messages published by the ``uwb_gz_plugin``.
2. Solve for the tag position by weighted least squares, in the **anchor
   frame** (the ideal square of the layout, see
   :mod:`uwb_pozyx_bridge.uwb_layout`).
3. Rotate the solution into NED by the layout angle ``alpha`` and send it as
   an ``ODOMETRY`` message.

Only the position is transmitted.  The velocity fields of an ``ODOMETRY``
message are interpreted by ArduPilot in the body FRD frame, and the UWB
ranging data does not provide a velocity estimate, so they are sent as
``NaN`` and the EKF velocity source stays disabled (``EK3_SRC1_VELXY = 0``).

.. warning::
   The position of an ``ODOMETRY`` message is **not** transformed by
   ArduPilot; it is fused into the EKF as it arrives, and the EKF works in
   NED.  See :mod:`uwb_pozyx_bridge.mavlink_odometry` for the details.

.. note::
   Despite the package name, this is **not** a Pozyx/Beacon bridge.  ArduPilot's
   Pozyx Beacon protocol (``BCN_TYPE = 1``, serial protocol 13) and its layout
   conventions (N pattern, lowest ID at the origin, ``BCN_ORIENT_YAW``) do not
   apply here: the position is solved offboard and delivered as a plain
   ``ODOMETRY`` message, so ArduPilot never sees the anchors.  The anchor layout
   is therefore free-form (see :mod:`uwb_pozyx_bridge.uwb_layout`).
   Details: ``docs/NOTES.md``.

.. note::
   The node sends **no attitude**: the quaternion of the ``ODOMETRY`` message
   is all ``NaN``.  The UWB trilateration produces a position, never an
   attitude, and ArduPilot reads the yaw out of that quaternion — with
   ``EK3_SRCx_YAW = EXTNAV`` a finite quaternion would drag the heading of
   the vehicle to whatever it encodes.  See
   :func:`uwb_pozyx_bridge.mavlink_odometry.unknown_attitude_quaternion`.
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

from .mavlink_odometry import (
    MAV_CMD_DO_SET_HOME,
    build_do_set_home_message,
    build_odometry_message,
    build_set_gps_global_origin_message,
    unknown_attitude_quaternion,
)
from .mavlink_sender import MavlinkSender
from .trilateration import Anchor, RangeMeasurement, solve_position
from .uwb_decoding import decode_ranging_pose
from .uwb_layout import AnchorLayout, default_layout

#: A position change larger than this between two fixes is treated as a
#: discontinuity and bumps the MAVLink reset counter.  The default is well
#: above the noise of the UWB solution, because a spurious reset makes the
#: EKF drop its velocity estimate.
DEFAULT_RESET_JUMP_THRESHOLD_M = 2.0

#: How long to wait after the connection is up before the EKF origin is
#: declared, in seconds.  The autopilot needs a moment to finish its own
#: initialisation before it accepts a new origin.
ORIGIN_SET_DELAY_S = 3.0

#: How long to wait before the origin is declared again while the autopilot
#: has not confirmed it.  The origin is not persisted across an autopilot
#: restart, so it has to be re-declared whenever the link comes back up.
ORIGIN_RETRY_S = 5.0

#: How often the EKF origin is refreshed, in seconds.  The navigation link
#: does not carry the autopilot's telemetry back to the companion, so there
#: is no confirmation to wait for; the declaration is simply repeated.
ORIGIN_REFRESH_S = 15.0

#: How often the home position is refreshed, in seconds.
HOME_REFRESH_S = 15.0

#: Minimum time between the origin declaration and the home declaration,
#: in seconds.  ArduPilot refuses to set the home position while the origin
#: is still unknown, so the home command has to follow the origin.
ORIGIN_HOME_GAP_S = 1.0

#: How often a heartbeat is sent to keep the GCS system id associated with
#: the link, in seconds.  ArduPilot only accepts commands from that id.
HEARTBEAT_INTERVAL_S = 1.0


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

        # The layout of the anchors.  ``alpha_deg`` is the bearing of the
        # direction ``a0 -> a1``, measured from North; it is the same
        # quantity the ESP32 tag calls ``config_yaw_offset_deg``.
        #
        # The layout is a rectangle: ``layout_size_x_m`` is the length of
        # the ``a0 -> a1`` side and ``layout_size_y_m`` the length of the
        # ``a0 -> a3`` side.  They are usually different, because the
        # anchors are mounted wherever there is room for them.
        self.declare_parameter("layout_size_x_m", default_layout().size_x_m)
        self.declare_parameter("layout_size_y_m", default_layout().size_y_m)
        self.declare_parameter("layout_alpha_deg", 0.0)
        self.declare_parameter("anchor_plane_height_m", default_layout().plane_height_m)

        # Noise model, in metres.
        self.declare_parameter("los_std", 0.10)
        self.declare_parameter("nlos_soft_std", 0.30)
        self.declare_parameter("nlos_hard_std", 0.60)

        # Solver.
        self.declare_parameter("min_anchors", 3)
        self.declare_parameter("max_range", 50.0)
        # With coplanar anchors the vertical component is ambiguous: the
        # ranges cannot tell which side of the anchor plane the tag is on.
        # The 3D solve resolves this by picking the branch that is closest
        # to the previous solution, which keeps the estimate continuous.
        # The legacy 2D mode (fixed altitude) is still available for
        # comparison, but it biases the horizontal solution by up to 0.2 m.
        self.declare_parameter("assume_constant_z", False)
        self.declare_parameter("tag_altitude_mm", 0.0)
        # The seed of the solver.  When left empty it is derived from the
        # layout: the centre of the rectangle, one metre below the anchor
        # plane.  The vertical component selects the branch of the mirror
        # ambiguity, so it has to be on the side of the anchor plane the
        # tag is expected to be on.
        self.declare_parameter("initial_guess", [])

        # Standard deviation of the position estimate, in metres.  ArduPilot
        # derives the measurement noise from the covariance matrix of the
        # ODOMETRY message, so this value ends up as the variance of the
        # position entries.
        self.declare_parameter("position_std_m", 0.05)

        # The attitude quaternion of the ODOMETRY message.  The UWB
        # trilateration produces a position, never an attitude, so the
        # default is "no attitude estimate", which is encoded as a
        # quaternion whose four components are all NaN.
        #
        # This must stay NaN: ArduPilot extracts the yaw from the
        # quaternion and, with EK3_SRCx_YAW = EXTNAV, aligns and then
        # continuously fuses the EKF yaw to it.  A finite quaternion would
        # therefore claim a heading the system does not know.
        #
        # The parameter exists so that the behaviour can be A/B tested
        # against the old, broken behaviour; "identity" reproduces it.
        self.declare_parameter("attitude_quaternion", "unknown")
        self._attitude_quaternion = self._resolve_attitude_quaternion()
        # The raw value the parameter was last resolved from, so that a
        # change of the parameter at runtime is picked up.
        self._attitude_quaternion_raw = str(
            self.get_parameter("attitude_quaternion").value
        ).strip()

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

        # The global reference of the UWB world.  ArduPilot needs it to set
        # the EKF origin, which it cannot derive from the external
        # navigation data alone.  Without an origin the autopilot rejects
        # MAV_CMD_NAV_TAKEOFF, so the vehicle arms but never leaves the
        # ground.
        self.declare_parameter("origin_latitude_deg", -35.363262)
        self.declare_parameter("origin_longitude_deg", 149.165237)
        self.declare_parameter("origin_altitude_m", 584.0)
        # When false, no origin is declared and the autopilot has to find
        # one by itself, e.g. from a real GPS.
        self.declare_parameter("set_origin", True)

        # ----- members -----
        self._lock = threading.Lock()
        self._layout = AnchorLayout(
            size_x_m=float(self.get_parameter("layout_size_x_m").value),
            size_y_m=float(self.get_parameter("layout_size_y_m").value),
            alpha_deg=float(self.get_parameter("layout_alpha_deg").value),
            plane_height_m=float(
                self.get_parameter("anchor_plane_height_m").value
            ),
        )
        self._anchors = self._load_anchors()
        self._initial_guess = self._resolve_initial_guess()
        self._ranges: dict[int, RangeMeasurement] = {}
        self._last_solution = None
        self._num_solutions = 0
        self._reset_counter = 0
        self._last_sent_position: Optional[tuple[float, float, float]] = None

        # The EKF origin and the home position are declared shortly after the
        # link is up, and re-declared until the autopilot confirms them.
        self._origin_sent = False
        self._origin_sent_at = 0.0
        self._origin_confirmed = False
        self._home_sent = False
        self._home_sent_at = 0.0
        self._home_confirmed = False
        self._connect_time: Optional[float] = None
        self._last_heartbeat_s = 0.0
        self._seen_connections = 0

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
            on_message=self._on_mavlink_message,
        )
        self._sender.connect(timeout=0.0)  # non-blocking first attempt

        period = 1.0 / float(self.get_parameter("send_rate").value)
        self._timer = self.create_timer(period, self._send_cycle)

        self.get_logger().info(
            f"odometry_node ready: anchors={len(self._anchors)} "
            f"layout={self._layout.size_x_m:.1f}x{self._layout.size_y_m:.1f}m "
            f"alpha={self._layout.alpha_deg:.1f}deg "
            f"input={self.get_parameter('input_topic').value} "
            f"serial={self.get_parameter('serial_host').value}:"
            f"{self.get_parameter('serial_port').value}"
        )

    # ------------------------------------------------------------ helpers

    def _load_anchors(self) -> list[Anchor]:
        """Build the anchor list from the layout.

        The solver works in the **anchor frame**: the ideal rectangle, with
        ``a0`` at the origin.  The physical layout is the same rectangle
        rotated by ``alpha``, and the rotation is applied to the solver
        *result* instead — see :meth:`_send_cycle`.  This keeps the
        ranging geometry independent of the mounting angle and makes the
        ESP32 tag and this node behave identically.
        """
        anchors: list[Anchor] = []
        for i, (x_m, y_m, z_m) in enumerate(
            self._layout.anchor_frame_positions()
        ):
            anchors.append(Anchor(id=i, position=(x_m, y_m, z_m)))
        return anchors

    def _resolve_initial_guess(self) -> list[float]:
        """The seed of the solver.

        The parameter overrides the default; when it is empty the seed is
        derived from the layout: the centre of the rectangle, one metre
        below the anchor plane.  The vertical component picks the branch of
        the mirror ambiguity, so it has to be on the side of the anchor
        plane the tag is expected to be on.
        """
        configured = list(self.get_parameter("initial_guess").value)
        if len(configured) >= 3:
            return [float(v) for v in configured[:3]]
        return [
            self._layout.size_x_m / 2.0,
            self._layout.size_y_m / 2.0,
            1.0,
        ]

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
        self._maybe_declare_origin()

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
            initial_guess=self._initial_guess,
            previous_position=(
                self._last_solution.position
                if self._last_solution is not None
                else None
            ),
            assume_constant_z=bool(
                self.get_parameter("assume_constant_z").value
            ),
        )

        if result is None:
            return

        self._last_solution = result
        self._num_solutions += 1
        self._publish_pose(result)

        # The trilateration result is in the anchor frame.  It is converted
        # into the NED frame that the EKF works in: the horizontal axes are
        # rotated by the layout angle and offset by the position of ``a0``,
        # and the vertical axis is converted from "depth below the anchor
        # plane" into "depth below the EKF origin".
        #
        # ArduPilot fuses the ODOMETRY position as-is in NED, so the result
        # of the conversion is what goes on the wire.
        if bool(self.get_parameter("assume_constant_z").value):
            # Legacy 2D mode: the altitude is held at a fixed value.
            down_m = (
                float(self.get_parameter("tag_altitude_mm").value) / 1000.0
                - self._layout.plane_height_m
            )
            north_m, east_m = self._layout.to_ned(
                result.position[0], result.position[1]
            )
        else:
            north_m, east_m, down_m = self._layout.to_ned_3d(
                result.position[0],
                result.position[1],
                result.position[2],
            )

        self._detect_reset(north_m, east_m, down_m)

        # Pick up a runtime change of the attitude quaternion parameter, so
        # that the A/B test does not need a restart of the stack.
        raw = str(self.get_parameter("attitude_quaternion").value).strip()
        if raw != self._attitude_quaternion_raw:
            self._attitude_quaternion_raw = raw
            self._attitude_quaternion = self._resolve_attitude_quaternion()
            self.get_logger().info(
                f"attitude_quaternion is now '{raw}'"
            )

        message = build_odometry_message(
            north_m,
            east_m,
            down_m,
            time.time_ns() // 1000,
            position_std_m=float(self.get_parameter("position_std_m").value),
            quaternion=self._attitude_quaternion,
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

    def _maybe_declare_origin(self) -> None:
        """Declare the global origin and the home position to ArduPilot.

        The UWB ranging data is a purely *local* measurement: it tells the
        autopilot where the vehicle is relative to the anchors, but not
        where the anchors are in the world.  ArduPilot still needs a global
        origin for its EKF, and without one it cannot convert its height
        between the "above origin" and "above home" frames.

        The origin alone is not enough.  ``ModeGuided::do_user_takeoff_start()``
        interprets the requested take-off altitude as *above home* and
        converts it to *above origin* with::

            Location target_loc = copter.current_loc;
            target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);
            if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
                return false;
            }

        ``Location::get_alt_cm()`` needs the home altitude for that
        conversion, so it fails while the home position is unknown and
        ``MAV_CMD_NAV_TAKEOFF`` is answered with ``MAV_RESULT_FAILED``.  That
        is exactly the observed symptom: the vehicle arms but never leaves
        the ground.

        So two things are declared, in this order:

        1. ``SET_GPS_GLOBAL_ORIGIN`` (message id 48), which is documented as
           the way to set an origin "when there is no GPS to provide an
           absolute position".
        2. ``MAV_CMD_DO_SET_HOME`` (command 179), which is the only way to
           establish a home position without a GPS.  ArduPilot refuses it
           while the origin is still unset, so the order matters.

        Both are re-sent periodically, because the origin is lost whenever the
        autopilot restarts (it is not persisted across a reset) and the
        companion node may be running while the autopilot reboots.  The
        navigation link does not carry the autopilot's telemetry back to the
        companion, so the declarations are refreshed on a timer instead of
        being driven by a confirmation message.
        """
        if not bool(self.get_parameter("set_origin").value):
            return
        if not self._sender.connected:
            return

        # A new connection means the autopilot may have restarted, which
        # drops its EKF origin and home position.  Start over in that case.
        if self._sender.connections != self._seen_connections:
            self._seen_connections = self._sender.connections
            self._origin_sent = False
            self._origin_sent_at = 0.0
            self._origin_confirmed = False
            self._home_sent = False
            self._home_sent_at = 0.0
            self._home_confirmed = False
            self._connect_time = None

        now = time.time()
        if self._connect_time is None:
            self._connect_time = now
            # The autopilot associates a system id with a link through the
            # heartbeat, so send one before the command.
            self._sender.send_heartbeat()
            return

        if now - self._connect_time < ORIGIN_SET_DELAY_S:
            return

        # Keep the heartbeat alive: the autopilot only accepts commands from
        # the system id it has seen a heartbeat from.
        if now - self._last_heartbeat_s > HEARTBEAT_INTERVAL_S:
            self._last_heartbeat_s = now
            self._sender.send_heartbeat()

        # The origin goes first: ArduPilot refuses to set the home position
        # while the origin is still unknown.
        if not self._origin_sent or now - self._origin_sent_at >= ORIGIN_REFRESH_S:
            message = build_set_gps_global_origin_message(
                float(self.get_parameter("origin_latitude_deg").value),
                float(self.get_parameter("origin_longitude_deg").value),
                float(self.get_parameter("origin_altitude_m").value),
            )
            if self._sender.send(message):
                if not self._origin_sent:
                    self.get_logger().info(
                        "EKF origin declared: "
                        f"lat={self.get_parameter('origin_latitude_deg').value} "
                        f"lon={self.get_parameter('origin_longitude_deg').value} "
                        f"alt={self.get_parameter('origin_altitude_m').value} m"
                    )
                self._origin_sent = True
                self._origin_sent_at = now
            else:
                self.get_logger().warn(
                    "could not declare the EKF origin; retrying on the next cycle"
                )
            return

        # The origin is in place, so the home position can be set.  It is
        # refreshed as well, because a home that was locked before an
        # autopilot restart is gone with the restart.
        if now - self._home_sent_at < HOME_REFRESH_S:
            return
        # Never send the home before the origin of this cycle went out.
        if now - self._origin_sent_at < ORIGIN_HOME_GAP_S:
            return

        message = build_do_set_home_message(
            float(self.get_parameter("origin_latitude_deg").value),
            float(self.get_parameter("origin_longitude_deg").value),
            float(self.get_parameter("origin_altitude_m").value),
        )
        if self._sender.send(message):
            if not self._home_sent:
                self.get_logger().info("home position declared")
            self._home_sent = True
            self._home_sent_at = now
        else:
            self.get_logger().warn(
                "could not declare the home position; retrying on the next cycle"
            )

    def _on_mavlink_message(self, message) -> None:
        """Handle the messages the autopilot sends back over the nav link.

        The autopilot answers ``COMMAND_LONG`` with a ``COMMAND_ACK``, and
        it sends ``HOME_POSITION`` once the home position is known.  Both
        are used to confirm that the declarations were accepted; without
        that confirmation the node would keep re-sending them forever.
        """
        msg_type = message.get_type()

        if msg_type == "COMMAND_ACK":
            command = getattr(message, "command", None)
            result = getattr(message, "result", None)
            if command == MAV_CMD_DO_SET_HOME:
                if result == 0:  # MAV_RESULT_ACCEPTED
                    self._home_confirmed = True
                    self.get_logger().info("home position accepted by ArduPilot")
                else:
                    self.get_logger().warn(
                        f"home position rejected by ArduPilot (result {result})"
                    )
            return

        if msg_type == "HOME_POSITION":
            self._home_confirmed = True
            self.get_logger().info(
                "ArduPilot reports a home position; "
                f"lat={getattr(message, 'latitude', 0) / 1e7:.6f} "
                f"lon={getattr(message, 'longitude', 0) / 1e7:.6f} "
                f"alt={getattr(message, 'altitude', 0) / 1000.0:.1f} m"
            )
            return

        if msg_type == "ORIGIN":
            self._origin_confirmed = True
            self.get_logger().info("ArduPilot confirms the EKF origin")

    def _resolve_attitude_quaternion(self) -> list[float]:
        """Resolve the ``attitude_quaternion`` parameter.

        The parameter accepts:

        ``unknown``
            The default: no attitude estimate, encoded as a quaternion whose
            four components are all ``NaN``.
        ``identity``
            The identity quaternion ``(1, 0, 0, 0)``.  This reproduces the
            old, broken behaviour and is only useful for A/B testing: it
            decodes to yaw = 0, which ArduPilot would fuse into the EKF as
            "heading North".

        Returns:
            The four components of the quaternion.
        """
        value = str(self.get_parameter("attitude_quaternion").value).strip()

        if value.lower() == "identity":
            return [1.0, 0.0, 0.0, 0.0]
        if value.lower() == "unknown":
            return unknown_attitude_quaternion()

        # An explicit "w,x,y,z" list, for completeness.
        parts = [p.strip() for p in value.split(",")]
        if len(parts) == 4:
            try:
                return [float(p) for p in parts]
            except ValueError:
                pass

        self.get_logger().warning(
            f"attitude_quaternion '{value}' is not understood; "
            "sending 'unknown' (all NaN)"
        )
        return unknown_attitude_quaternion()

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
        """Publish the trilateration result for debugging.

        The pose is published in the **anchor frame**, because that is the
        frame the solver works in.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "uwb_anchor_frame"
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
