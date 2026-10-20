#!/usr/bin/env python3
# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Bridge the UWB trilateration fix into MAVROS ``odometry/in``.

Why ODOMETRY and not VISION_POSITION_ESTIMATE
---------------------------------------------

ArduPilot's EKF3 has no way to fuse a *position-only* external nav
source.  ``VISION_POSITION_ESTIMATE`` carries position but the MAVLink
schema has no velocity fields, so ``AP_VisualOdom`` is left without a
velocity estimate and EKF3 rejects the sample — showing up as a continuous
``EKF3 IMU0 stopped aiding`` / ``is using external nav data`` cycle and
an autopilot that will never arm.

``MAVLINK_MSG_ID_ODOMETRY`` carries **both** position and velocity, and
``GCS_MAVLINK::handle_odometry`` feeds the two into EKF3 as
``AP_VisualOdom::handle_pose_estimate`` and
``AP_VisualOdom::handle_vision_speed_estimate``.  That is exactly what
EKF3 needs, so this node publishes ODOMETRY as the primary path.

Frame conventions
-----------------

``GCS_MAVLINK::handle_odometry`` silently drops the packet unless

* ``frame_id      == MAV_FRAME_LOCAL_FRD``  (x=Forward, y=Right, z=Down)
* ``child_frame_id == MAV_FRAME_BODY_FRD``

The UWB solve is in the Gazebo world frame (x=East, y=North, z=Up), and
the drone spawns facing +x.  This node therefore emits::

    odom.x = +pos.x     (East  -> Forward)
    odom.y = -pos.y     (North -> Right is -North when heading +x)
    odom.z = -pos.z     (Up    -> Down)
    odom.v* = same rotation applied to the differentiated velocity

and fills in a neutral orientation quaternion (the UWB tag carries no
attitude, and EKF3 gets roll/pitch/yaw from the IMU + compass instead).

Two secondary outputs are kept for ROS-side consumers and debugging:

* ``/mavros/vision_pose/pose`` + ``pose_cov`` (MAVROS vision_pose plugin);
* a synthetic ``NavSatFix`` on ``/mavros/gps_input/gps_input`` for tools
  that expect a global fix.  Set ``publish_gps_input`` to ``false`` to
  keep the system strictly GPS-free.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus

# Default origin for the synthetic global fix (matches the UWB runway
# world).  Values are arbitrary — EKF3 only uses relative motion — but a
# plausible WGS-84 point keeps GCS displays sane.  Override at runtime.
DEFAULT_LAT0_DEG = -35.3632621
DEFAULT_LON0_DEG = 149.1652374
DEFAULT_ALT0_M = 584.0
METRES_PER_DEG_LAT = 111_320.0

# Clamp on the finite-difference velocity.  UWB trilateration is noisy at
# the few-cm level; differentiating consecutive fixes can produce large
# spurious spikes when a single sample is an outlier.  Anything beyond
# this is treated as a glitch and clamped (the drone never moves faster
# than a few m/s in this test setup).
MAX_SPEED_MPS = 10.0

# Ignore velocity derived from a gap longer than this — after a pause the
# finite difference is meaningless and would swamp the filter.
MAX_VEL_DT_S = 0.5


class UwbToMavrosVisionNode(Node):
    """Forward the UWB fix to MAVROS as ODOMETRY (position + velocity)."""

    def __init__(self) -> None:
        super().__init__("uwb_to_mavros_vision_node")

        # ----- parameters -----
        self.declare_parameter("input_topic", "/uwb/pose_cov")
        self.declare_parameter("output_topic", "/mavros/odometry/in")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "uwb_tag")
        self.declare_parameter("publish_vision_pose", True)
        self.declare_parameter("publish_gps_input", False)
        self.declare_parameter("gps_lat0", DEFAULT_LAT0_DEG)
        self.declare_parameter("gps_lon0", DEFAULT_LON0_DEG)
        self.declare_parameter("gps_alt0", DEFAULT_ALT0_M)

        # ----- members -----
        self._prev_stamp_s: Optional[float] = None
        self._prev_enu: Optional[tuple[float, float, float]] = None

        # ----- subscriptions / publishers -----
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_topic").get_parameter_value().string_value,
            self._on_pose,
            qos,
        )

        self._parent_frame = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self._child_frame = (
            self.get_parameter("child_frame_id")
            .get_parameter_value()
            .string_value
        )

        # Primary path: ODOMETRY -> MAVROS odometry plugin -> EKF3 (src 6).
        self._pub_odom = self.create_publisher(
            Odometry,
            self.get_parameter("output_topic").get_parameter_value().string_value,
            qos,
        )

        # Secondary, kept for ROS-side consumers.
        self._pub_pose = None
        self._pub_cov = None
        if self.get_parameter("publish_vision_pose").value:
            self._pub_pose = self.create_publisher(
                PoseStamped, "/mavros/vision_pose/pose", qos
            )
            self._pub_cov = self.create_publisher(
                PoseWithCovarianceStamped,
                "/mavros/vision_pose/pose_cov",
                qos,
            )

        self._pub_gps = None
        if self.get_parameter("publish_gps_input").value:
            self._pub_gps = self.create_publisher(
                NavSatFix, "/mavros/gps_input/gps_input", qos
            )

        self.get_logger().info(
            "uwb_to_mavros_vision_node ready: "
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} "
            f"(LOCAL_FRD/BODY_FRD, gps_input="
            f"{self.get_parameter('publish_gps_input').value})"
        )

    # ---------------------------------------------------------------- callback
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Convert one UWB fix into ODOMETRY and (optionally) the extras."""
        pos = msg.pose.pose.position
        enu = (pos.x, pos.y, pos.z)

        # ---- velocity by finite difference on the ENU fix ----
        t_now = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec or msg.header.stamp.nanosec
            else self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        )

        vel_enu = (0.0, 0.0, 0.0)
        if self._prev_enu is not None and self._prev_stamp_s is not None:
            dt = t_now - self._prev_stamp_s
            if 1e-6 < dt <= MAX_VEL_DT_S:
                vel_enu = tuple(
                    max(-MAX_SPEED_MPS, min(MAX_SPEED_MPS, (a - b) / dt))
                    for a, b in zip(enu, self._prev_enu)
                )
        self._prev_enu = enu
        self._prev_stamp_s = t_now

        # ---- ENU -> FRD ----
        # Gazebo world: +x East, +y North, +z Up.
        # MAVLink LOCAL_FRD: +x Forward, +y Right, +z Down.
        # The drone spawns heading +x (East), so Forward=+x, Right=-y, Down=-z.
        frd = (enu[0], -enu[1], -enu[2])
        vel_frd = (vel_enu[0], -vel_enu[1], -vel_enu[2])

        # ---- ODOMETRY ----
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._parent_frame
        odom.child_frame_id = self._child_frame

        odom.pose.pose.position = Point(
            x=frd[0], y=frd[1], z=frd[2]
        )
        # The UWB tag has no attitude.  Send identity so EKF3 takes
        # roll/pitch from the IMU and yaw from the compass, rather than
        # having a bogus orientation claim authority.
        odom.pose.pose.orientation = Quaternion(w=1.0)

        # Carry the trilateration covariance through, rotated into FRD.
        # Sign flips of a coordinate permutation swap the diagonal terms;
        # variances are diagonal-dominant here so a direct swap is fine.
        cov_in = list(msg.pose.covariance)
        if len(cov_in) >= 36 and any(v != 0.0 for v in cov_in):
            cov_out = [0.0] * 36
            # ENU (x, y, z) -> FRD (+x, -y, -z): variances for y and z swap.
            cov_out[0] = cov_in[0]
            cov_out[7] = cov_in[14]
            cov_out[14] = cov_in[7]
            cov_out[21] = cov_in[21] if len(cov_in) > 21 else 0.05
            cov_out[28] = cov_in[28] if len(cov_in) > 28 else 0.05
            cov_out[35] = cov_in[35] if len(cov_in) > 35 else 1.0
        else:
            # Default 2 m horizontal / 4 m vertical 1-sigma.
            cov_out = [0.0] * 36
            cov_out[0] = 4.0
            cov_out[7] = 4.0
            cov_out[14] = 16.0
            cov_out[21] = 0.05
            cov_out[28] = 0.05
            cov_out[35] = 1.0
        odom.pose.covariance = cov_out

        odom.twist.twist.linear = Vector3(
            x=vel_frd[0], y=vel_frd[1], z=vel_frd[2]
        )
        # UWB gives no angular rate; zeros are correct (no rotation claimed).
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        vel_cov = [0.0] * 36
        vel_cov[0] = 0.25
        vel_cov[7] = 0.25
        vel_cov[14] = 0.25
        vel_cov[21] = 0.01
        vel_cov[28] = 0.01
        vel_cov[35] = 0.01
        odom.twist.covariance = vel_cov

        self._pub_odom.publish(odom)

        # ---- secondary: vision_pose ----
        if self._pub_pose is not None:
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = self._parent_frame
            out.pose = msg.pose.pose
            self._pub_pose.publish(out)

            cov_msg = PoseWithCovarianceStamped()
            cov_msg.header = out.header
            cov_msg.pose = PoseWithCovariance()
            cov_msg.pose.pose = out.pose
            cov_msg.pose.covariance = cov_out
            self._pub_cov.publish(cov_msg)

        # ---- secondary: synthetic global fix ----
        if self._pub_gps is not None:
            self._pub_gps.publish(
                self._make_nav_sat_fix(msg.header, enu)
            )

    # ---------------------------------------------------------------- helpers
    def _make_nav_sat_fix(self, header, enu) -> NavSatFix:
        """Map-frame (x, y, z) metres to a WGS-84 ``NavSatFix``."""
        lat0 = math.radians(
            self.get_parameter("gps_lat0").get_parameter_value().double_value
        )
        lon0 = math.radians(
            self.get_parameter("gps_lon0").get_parameter_value().double_value
        )
        alt0 = self.get_parameter("gps_alt0").get_parameter_value().double_value

        x, y, z = enu  # East, North, Up
        lat = lat0 + (y / METRES_PER_DEG_LAT)
        lon = lon0 + (x / (METRES_PER_DEG_LAT * math.cos(lat0)))
        alt = alt0 + z

        fix = NavSatFix()
        fix.header = header
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = math.degrees(lat)
        fix.longitude = math.degrees(lon)
        fix.altitude = alt
        fix.position_covariance = [
            4.0, 0.0, 0.0,
            0.0, 4.0, 0.0,
            0.0, 0.0, 16.0,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return fix


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UwbToMavrosVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()