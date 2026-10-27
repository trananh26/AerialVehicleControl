#!/usr/bin/env python3
"""Step 5.1 gate checker: validates the Gazebo -> ROS 2 ranging stream.

Subscribes to ``/uwb/ranging_ros`` and ``/uwb/tag_pose_ros`` and checks that
the stream matches the contract the Pozyx bridge node relies on:

* roughly 20 messages per second
* one message per anchor, with ``position.z`` holding the anchor id
* ``position.x`` holding a plausible range in metres
* ``position.y`` holding the RSS in dBm
* ``orientation.x`` holding the LOS code (0, 1 or 2)
* the tag pose topic is separate and reports the real vehicle position

Exits with a non-zero status when a check fails, so it can be used as a gate.

Usage::

    python3 check_ranging_stream.py --duration 10
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import defaultdict

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

#: The tag sits at the centre of the anchor square, so every range is
#: sqrt(5^2 + 5^2 + 1.9^2) = 7.34 m.
EXPECTED_RANGE_M = 7.34
RANGE_TOLERANCE_M = 0.5


class RangeStreamChecker(Node):
    """Collects ranging messages and checks the stream contract."""

    def __init__(self) -> None:
        super().__init__("range_stream_checker")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.ranges: dict[int, list[float]] = defaultdict(list)
        self.rss: dict[int, list[float]] = defaultdict(list)
        self.los_codes: dict[int, set[int]] = defaultdict(set)
        self.received: list[float] = []
        self.tag_poses: list[tuple[float, float, float]] = []

        self.create_subscription(
            Pose, "/uwb/ranging_ros", self._on_ranging, qos
        )
        self.create_subscription(
            Pose, "/uwb/tag_pose_ros", self._on_tag_pose, qos
        )

    def _on_ranging(self, msg: Pose) -> None:
        anchor_id = int(round(msg.position.z))
        self.ranges[anchor_id].append(msg.position.x)
        self.rss[anchor_id].append(msg.position.y)
        self.los_codes[anchor_id].add(int(round(msg.orientation.x)))
        self.received.append(
            self.get_clock().now().nanoseconds / 1e9
        )

    def _on_tag_pose(self, msg: Pose) -> None:
        self.tag_poses.append(
            (msg.position.x, msg.position.y, msg.position.z)
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--expected-anchors", type=int, default=4,
    )
    parser.add_argument(
        "--expected-rate", type=float, default=20.0,
    )
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = RangeStreamChecker()

    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures: list[str] = []
    notes: list[str] = []

    # --- rate -------------------------------------------------------------
    # The plugin publishes one message per anchor per cycle, so the total
    # message rate is the update rate multiplied by the anchor count.
    n_anchors = max(len(node.ranges), 1)
    if len(node.received) < 2:
        failures.append(
            f"only {len(node.received)} ranging messages received"
        )
        rate = 0.0
    else:
        span = node.received[-1] - node.received[0]
        total_rate = (
            (len(node.received) - 1) / span if span > 0 else 0.0
        )
        rate = total_rate / n_anchors
        notes.append(
            f"total rate = {total_rate:.1f} Hz over {n_anchors} anchors "
            f"= {rate:.1f} Hz per anchor "
            f"(expected {args.expected_rate} Hz)"
        )
        if abs(rate - args.expected_rate) > 5.0:
            failures.append(
                f"per-anchor rate {rate:.1f} Hz is too far from "
                f"{args.expected_rate} Hz"
            )

    # --- anchors ----------------------------------------------------------
    notes.append(f"anchor ids seen: {sorted(node.ranges.keys())}")
    if len(node.ranges) != args.expected_anchors:
        failures.append(
            f"expected {args.expected_anchors} anchors, "
            f"got {sorted(node.ranges.keys())}"
        )

    for anchor_id in sorted(node.ranges):
        values = node.ranges[anchor_id]
        mean = statistics.fmean(values)
        notes.append(
            f"  anchor {anchor_id}: n={len(values)} "
            f"mean range = {mean:.3f} m "
            f"(expected {EXPECTED_RANGE_M} m)"
        )
        if abs(mean - EXPECTED_RANGE_M) > RANGE_TOLERANCE_M:
            failures.append(
                f"anchor {anchor_id} mean range {mean:.3f} m differs from "
                f"the expected {EXPECTED_RANGE_M} m"
            )
        los = node.los_codes[anchor_id]
        notes.append(f"  anchor {anchor_id}: LOS codes {sorted(los)}")
        if not los <= {0, 1, 2}:
            failures.append(
                f"anchor {anchor_id} has invalid LOS codes {sorted(los)}"
            )
        rss = node.rss[anchor_id]
        notes.append(
            f"  anchor {anchor_id}: RSS ~ {statistics.fmean(rss):.1f} dBm"
        )

    # --- tag pose ---------------------------------------------------------
    if not node.tag_poses:
        failures.append("no tag pose received on /uwb/tag_pose_ros")
    else:
        x, y, z = node.tag_poses[-1]
        notes.append(f"tag pose = ({x:.3f}, {y:.3f}, {z:.3f})")
        # The vehicle spawns at the centre of the anchor square.
        if abs(x - 5.0) > 0.3 or abs(y - 5.0) > 0.3:
            failures.append(
                f"tag pose ({x:.3f}, {y:.3f}) is not at the anchor centre"
            )

    # --- no leak ----------------------------------------------------------
    # The tag pose is published as a Pose with a position only, so if it ever
    # leaks into the ranging topic its anchor id would be 0 with a range of
    # about 5 m instead of about 7.3 m.  A large spread between the shortest
    # and longest range would show that up.
    if node.ranges:
        all_values = [v for values in node.ranges.values() for v in values]
        spread = max(all_values) - min(all_values)
        notes.append(f"range spread = {spread:.3f} m")
        if spread > 1.0:
            failures.append(
                f"range spread {spread:.3f} m suggests the tag pose is "
                "leaking into the ranging topic"
            )

    print("--- notes ---")
    for note in notes:
        print(f"  {note}")

    if failures:
        print("--- FAILURES ---")
        for failure in failures:
            print(f"  {failure}")
        print("STEP 5.1 GATE: FAILED")
        rclpy.shutdown()
        return 1

    print("STEP 5.1 GATE: PASSED")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
