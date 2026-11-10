#!/usr/bin/env python3
"""Fake UWB ranging source for integration testing the Pozyx bridge.

Publishes ``geometry_msgs/Pose`` messages on the ranging topic using the same
custom encoding as the Gazebo ``uwb_gz_plugin``, so the bridge node can be
exercised end to end without starting Gazebo.

The simulated tag orbits (or sits still at) a point inside the anchor
rectangle, and the ranges are computed from the true distances to each
anchor.  The rectangle can be changed with ``--size-x`` / ``--size-y``, so
the tool follows the layout the simulation uses.

Usage::

    python3 fake_ranging_source.py --topic /uwb/ranging_ros
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

#: The default Gazebo world layout: anchors at the corners of a 10 m x 10 m
#: square at z = 2.0 m, and the vehicle spawning at the centre.
DEFAULT_SIZE_X_M = 10.0
DEFAULT_SIZE_Y_M = 10.0
ANCHOR_PLANE_HEIGHT_M = 2.0

#: The tag's true altitude, relative to the ground.
TAG_ALTITUDE = 0.1


def build_anchors(size_x_m: float, size_y_m: float):
    """The anchor positions of the layout, in the world frame.

    Args:
        size_x_m: The length of the ``a0 -> a1`` side, in metres.
        size_y_m: The length of the ``a0 -> a3`` side, in metres.

    Returns:
        A list of ``(anchor_id, (x_m, y_m, z_m))`` pairs, in the anchor
        frame with ``alpha = 0``.
    """
    z = ANCHOR_PLANE_HEIGHT_M
    return [
        (0, (0.0, 0.0, z)),
        (1, (size_x_m, 0.0, z)),
        (2, (size_x_m, size_y_m, z)),
        (3, (0.0, size_y_m, z)),
    ]


def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


class FakeRangingSource(Node):
    """Publishes simulated anchor ranges."""

    def __init__(
        self,
        topic: str = "/uwb/ranging_ros",
        rate: float = 20.0,
        static: bool = False,
        position=(0.0, 0.0),
        orbit_radius: float = 2.0,
        orbit_period: float = 20.0,
        noise_std: float = 0.0,
        size_x_m: float = DEFAULT_SIZE_X_M,
        size_y_m: float = DEFAULT_SIZE_Y_M,
    ) -> None:
        super().__init__("fake_ranging_source")

        self.static = static
        self.position = position
        self.orbit_radius = orbit_radius
        self.orbit_period = orbit_period
        self.noise_std = noise_std
        self.anchors = build_anchors(size_x_m, size_y_m)
        # The tag orbits around the centre of the rectangle.
        self.centre = (size_x_m / 2.0, size_y_m / 2.0)
        self._start = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(Pose, topic, qos)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"fake_ranging_source publishing on {topic} at {rate} Hz"
        )

    def _tag_position(self, t: float) -> tuple[float, float, float]:
        if self.static:
            x, y = self.position
        else:
            phase = 2.0 * math.pi * t / self.orbit_period
            x = self.position[0] + self.orbit_radius * math.cos(phase)
            y = self.position[1] + self.orbit_radius * math.sin(phase)
        return (x + self.centre[0], y + self.centre[1], TAG_ALTITUDE)

    def _publish(self) -> None:
        if self._start is None:
            self._start = self.get_clock().now().nanoseconds / 1e9
        t = self.get_clock().now().nanoseconds / 1e9 - self._start

        tag = self._tag_position(t)

        for anchor_id, anchor_pos in self.anchors:
            rng = distance(tag, anchor_pos)
            if self.noise_std > 0.0:
                rng += self._rng.gauss(0.0, self.noise_std)

            msg = Pose()
            msg.position.x = rng
            msg.position.y = -50.0
            msg.position.z = float(anchor_id)
            msg.orientation.x = 0.0
            msg.orientation.w = 1.0
            self._pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fake UWB ranging source")
    parser.add_argument("--topic", default="/uwb/ranging_ros")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--static", action="store_true")
    parser.add_argument(
        "--position",
        type=float,
        nargs=2,
        default=[0.0, 0.0],
        metavar=("X", "Y"),
        help="tag position relative to the beacon origin, metres",
    )
    parser.add_argument("--orbit-radius", type=float, default=2.0)
    parser.add_argument("--orbit-period", type=float, default=20.0)
    parser.add_argument("--noise-std", type=float, default=0.0)
    parser.add_argument(
        "--size-x",
        type=float,
        default=DEFAULT_SIZE_X_M,
        help="length of the a0 -> a1 side of the anchor rectangle, metres",
    )
    parser.add_argument(
        "--size-y",
        type=float,
        default=DEFAULT_SIZE_Y_M,
        help="length of the a0 -> a3 side of the anchor rectangle, metres",
    )
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = FakeRangingSource(
        topic=args.topic,
        rate=args.rate,
        static=args.static,
        position=tuple(args.position),
        orbit_radius=args.orbit_radius,
        orbit_period=args.orbit_period,
        noise_std=args.noise_std,
        size_x_m=args.size_x,
        size_y_m=args.size_y,
    )
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
