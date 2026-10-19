#!/usr/bin/env python3
# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Bridge ``/uwb/pose_cov`` into MAVROS ``vision_pose/pose``.

Subscribes to the ``geometry_msgs/PoseWithCovarianceStamped`` produced by
``uwb_trilateration_node`` and republishes a plain
``geometry_msgs/PoseStamped`` on ``/mavros/vision_pose/pose``.

MAVROS' built-in ``vision_pose_estimate`` plugin then forwards it to the
flight controller as a MAVLink ``VISION_POSITION_ESTIMATE`` message,
which ArduPilot's EKF consumes when ``EK3_SRC1_POSXY = 6`` (ExternalNav).

This indirection keeps the trilateration node agnostic of MAVROS so the
same code can be reused in production (where the ESP32 publishes MAVLink
directly) and in simulation (where MAVROS is the simplest transport).
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class UwbToMavrosVisionNode(Node):
    """Forward ``PoseWithCovarianceStamped`` to MAVROS ``vision_pose``."""

    def __init__(self) -> None:
        super().__init__("uwb_to_mavros_vision_node")

        # ----- parameters -----
        self.declare_parameter("input_topic", "/uwb/pose_cov")
        self.declare_parameter("output_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("frame_id", "map")
        # Set true to also forward a synthetic zero-velocity twist so
        # MAVROS' setpoint_velocity plugin has fresh data; leave false
        # unless you need velocity control.
        self.declare_parameter("reset_stamp_on_each_msg", True)

        # ----- members -----
        self._latest: Optional[PoseWithCovarianceStamped] = None
        self._last_send = self.get_clock().now()

        # ----- subscriptions / publishers -----
        qos_in = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("input_topic").get_parameter_value().string_value,
            self._on_pose,
            qos_in,
        )

        # MAVROS advertises vision_pose/pose as RELIABLE + TRANSIENT_LOCAL
        # so the plugin discovers it even if the relay starts later.
        qos_out = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("output_topic").get_parameter_value().string_value,
            qos_out,
        )

        self.get_logger().info(
            "uwb_to_mavros_vision_node ready: "
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    # ----------------------------------------------------------------- callback
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Buffer the most recent pose and forward immediately."""
        self._latest = msg
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        out.pose = msg.pose.pose
        self._pub.publish(out)


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
