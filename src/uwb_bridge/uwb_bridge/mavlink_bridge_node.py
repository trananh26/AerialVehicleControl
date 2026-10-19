#!/usr/bin/env python3
# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Forward UWB trilateration results to ArduPilot over MAVLink.

Subscribes to ``geometry_msgs/PoseWithCovarianceStamped`` (produced by
``uwb_trilateration_node``) and sends MAVLink ``VISION_POSITION_ESTIMATE``
messages to a UDP endpoint that the ArduPilot SITL is listening on.

This bypasses MAVROS so that the same code path used in real ESP32
hardware (where the microcontroller crafts the MAVLink message itself)
is also exercised in simulation.

The intended UDP target is the MAVLink instance used by
``ardupilot_sitl`` in this workspace (default ``udp:127.0.0.1:14550``).
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class MavlinkBridgeNode(Node):
    """Republishes ROS pose as MAVLink VISION_POSITION_ESTIMATE."""

    def __init__(self) -> None:
        super().__init__("mavlink_bridge_node")

        # ----- parameters -----
        self.declare_parameter("pose_topic", "uwb_pose_cov")
        self.declare_parameter("mavlink_url", "udpout:127.0.0.1:14550")
        self.declare_parameter("send_rate_hz", 25.0)
        self.declare_parameter("frame_id", "map")  # ECEF or local NED
        self.declare_parameter("use_ros_time", True)
        # VISION_POSITION_ESTIMATE covariance is a 6x6 array in row-major.
        # We scale the position covariance by this factor before sending.
        self.declare_parameter("position_cov_scale", 1.0)

        # ----- members -----
        self._latest: Optional[PoseWithCovarianceStamped] = None
        self._last_send = 0.0

        # ----- MAVLink -----
        url = self.get_parameter("mavlink_url").value
        self.get_logger().info(f"Opening MAVLink: {url}")
        self._mav = mavutil.mavlink_connection(url, source_system=255)
        # Wait a tiny bit so the socket is bound.
        time.sleep(0.2)
        # Heartbeat so the FC knows we're alive (optional).
        try:
            self._mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"heartbeat failed: {exc}")

        # ----- ROS -----
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            qos,
        )

        period = 1.0 / max(self.get_parameter("send_rate_hz").value, 1e-3)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"mavlink_bridge_node ready: topic="
            f"{self.get_parameter('pose_topic').value} -> {url}"
        )

    # ----------------------------------------------------------- callback
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest = msg

    # -------------------------------------------------------------- main
    def _tick(self) -> None:
        if self._latest is None:
            return
        now = time.monotonic()
        period = 1.0 / max(self.get_parameter("send_rate_hz").value, 1e-3)
        if now - self._last_send < period:
            return
        self._last_send = now

        msg = self._latest
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Roll / pitch / yaw from quaternion.
        # We pass the full orientation; if the upstream filter
        # produces a heading, MAVLink will forward it.
        roll, pitch, yaw = self._quat_to_rpy(
            ori.x, ori.y, ori.z, ori.w
        )

        # MAVLink VISION_POSITION_ESTIMATE expects time as a uint32_t
        # of milliseconds since boot. If we have a ROS header we map
        # it to monotonic ms; otherwise just send "0" (FC will stamp).
        usec = 0
        if self.get_parameter("use_ros_time").value:
            try:
                sec = msg.header.stamp.sec
                nsec = msg.header.stamp.nanosec
                usec = int(sec * 1_000_000 + nsec // 1000)
            except Exception:
                usec = 0

        # Pull position covariance from the 6x6.
        cov = msg.pose.covariance
        cov_scale = float(self.get_parameter("position_cov_scale").value)
        p_cov = [
            cov[0]  * cov_scale,  # x
            cov[7]  * cov_scale,  # y
            cov[14] * cov_scale,  # z
        ]

        try:
            self._mav.mav.vision_position_estimate_send(
                usec,
                float(pos.x),
                float(pos.y),
                float(pos.z),
                float(roll),
                float(pitch),
                float(yaw),
                p_cov,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"vision_position_estimate failed: {exc}")

    # -------------------------------------------------------------- utils
    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float):
        """Convert quaternion to (roll, pitch, yaw) in radians."""
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
