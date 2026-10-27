#!/usr/bin/env python3
"""Step 5.2 gate checker: inspects the frame mapping of the UWB pipeline.

Reads the ranging stream and the tag pose, then shows what the Pozyx bridge
node would put on the wire, without sending anything.  This separates the
frame arithmetic from the ArduPilot side, so a mapping mistake shows up
here rather than as a badly-placed EKF estimate.

The chain being checked:

    Gazebo world (ENU, m)
        minus the beacon origin
    beacon-relative (ENU, m)
        times 1000
    Pozyx wire (mm, ENU)
        AP_Beacon_Pozyx flips the sign of z
    ArduPilot (NED, m)

Usage::

    python3 inspect_frames.py --duration 10
"""

from __future__ import annotations

import argparse
import os
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

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "uwb_pozyx_bridge",
    ),
)

from uwb_pozyx_bridge.pozyx_protocol import (  # noqa: E402
    encode_beacon_config,
    encode_beacon_distance,
    encode_vehicle_position,
)
from uwb_pozyx_bridge.trilateration import (  # noqa: E402
    Anchor,
    RangeMeasurement,
    solve_position,
)
from uwb_pozyx_bridge.uwb_decoding import (  # noqa: E402
    DEFAULT_ANCHOR_OFFSETS_MM,
    anchor_offsets_from_world,
    decode_ranging_pose,
)

#: Where the vehicle spawns, in the Gazebo world frame.
BEACON_ORIGIN = (5.0, 5.0, 0.1)

#: The anchors in the Gazebo world frame.
ANCHORS_WORLD = [
    (0.0, 0.0, 2.0),
    (10.0, 0.0, 2.0),
    (10.0, 10.0, 2.0),
    (0.0, 10.0, 2.0),
]


class FrameInspector(Node):
    """Collects ranging data and the tag pose."""

    def __init__(self) -> None:
        super().__init__("frame_inspector")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.ranges: dict[int, RangeMeasurement] = {}
        self.tag_pose = (0.0, 0.0, 0.0)

        self.create_subscription(
            Pose, "/uwb/ranging_ros", self._on_ranging, qos
        )
        self.create_subscription(
            Pose, "/uwb/tag_pose_ros", self._on_tag_pose, qos
        )

    def _on_ranging(self, msg: Pose) -> None:
        measurement = decode_ranging_pose(
            (msg.position.x, msg.position.y, msg.position.z),
            msg.orientation.x,
        )
        if measurement is not None:
            self.ranges[measurement.anchor_id] = measurement

    def _on_tag_pose(self, msg: Pose) -> None:
        self.tag_pose = (msg.position.x, msg.position.y, msg.position.z)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--position-accuracy-m", type=float, default=0.05,
    )
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = FrameInspector()

    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures: list[str] = []

    # --- anchor offsets ---------------------------------------------------
    offsets_m = anchor_offsets_from_world(ANCHORS_WORLD, BEACON_ORIGIN)
    offsets_mm = tuple(
        tuple(v * 1000.0 for v in o) for o in offsets_m
    )

    print("=== anchor offsets (beacon-relative, mm) ===")
    for i, off in enumerate(offsets_mm):
        print(f"  anchor {i}: ({off[0]:+.0f}, {off[1]:+.0f}, {off[2]:+.0f})")

    if offsets_mm != DEFAULT_ANCHOR_OFFSETS_MM:
        failures.append(
            "the computed anchor offsets do not match the node's default "
            f"parameter\n    computed: {offsets_mm}\n"
            f"    default:  {DEFAULT_ANCHOR_OFFSETS_MM}"
        )
    else:
        print("  -> matches the node's default anchor_offsets_mm")

    for off in offsets_mm:
        if off[2] <= 0:
            failures.append(
                f"anchor z offset {off[2]} must be positive: the anchors sit "
                "above the beacon origin"
            )

    # --- trilateration ----------------------------------------------------
    if len(node.ranges) < 3:
        failures.append(
            f"only {len(node.ranges)} anchors received, need at least 3"
        )
        result = None
    else:
        anchors = [
            Anchor(
                id=i,
                position=(
                    offsets_m[i][0],
                    offsets_m[i][1],
                    offsets_m[i][2],
                ),
            )
            for i in range(len(offsets_m))
        ]
        result = solve_position(
            anchors,
            list(node.ranges.values()),
            los_std=0.10,
            nlos_soft_std=0.30,
            nlos_hard_std=0.60,
            min_anchors=3,
            max_range=50.0,
            initial_guess=[0.0, 0.0, 0.0],
            assume_constant_z=True,
        )

    truth = node.tag_pose

    if result is None:
        failures.append("the trilateration solver returned no solution")
    else:
        pos = result.position
        print()
        print("=== tag position ===")
        print(f"  trilateration (beacon-relative, m): "
              f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

        # The beacon origin is the spawn point, so the beacon-relative
        # position of a vehicle sitting at the origin must be zero.
        rel_truth = (
            truth[0] - BEACON_ORIGIN[0],
            truth[1] - BEACON_ORIGIN[1],
        )
        print(f"  Gazebo truth   (beacon-relative, m): "
              f"({rel_truth[0]:.3f}, {rel_truth[1]:.3f})")

        error = (
            (pos[0] - rel_truth[0]) ** 2 + (pos[1] - rel_truth[1]) ** 2
        ) ** 0.5
        print(f"  horizontal error: {error:.3f} m")
        if error > 0.3:
            failures.append(
                f"trilateration error {error:.3f} m exceeds the 0.3 m limit"
            )

        if abs(pos[0]) > 0.3 or abs(pos[1]) > 0.3:
            print()
            print("  note: the vehicle is not exactly at the beacon origin, "
                  "so a non-zero position is expected")

        # --- the frames that would go on the wire -----------------------
        x_mm = int(round(pos[0] * 1000.0))
        y_mm = int(round(pos[1] * 1000.0))
        z_mm = 0
        accuracy = int(round(args.position_accuracy_m))

        print()
        print("=== Pozyx frames that would be sent ===")

        frames = bytearray()
        for anchor in anchors:
            frames += encode_beacon_config(
                anchor.id,
                len(anchors),
                int(round(anchor.position[0] * 1000.0)),
                int(round(anchor.position[1] * 1000.0)),
                int(round(anchor.position[2] * 1000.0)),
            )
        for anchor in anchors:
            measurement = node.ranges.get(anchor.id)
            if measurement is None:
                continue
            frames += encode_beacon_distance(
                anchor.id, int(round(measurement.range_m * 1000.0))
            )
        frames += encode_vehicle_position(x_mm, y_mm, z_mm, accuracy)

        for frame in _split_frames(bytes(frames)):
            msg_id, payload = frame
            print(f"  msg 0x{msg_id:02X}  payload {payload.hex(' ')}")

        print()
        print("=== decode of the vehicle position frame ===")
        print(f"  x = {x_mm} mm, y = {y_mm} mm, z = {z_mm} mm")
        print(f"  position_error = {accuracy}  (metres, workaround R2)")
        if accuracy >= 1:
            failures.append(
                f"position_error {accuracy} is not below the 1.0 threshold "
                "the EKF applies, so the EKF would refuse to align"
            )

        # --- what ArduPilot will make of it ----------------------------
        print()
        print("=== what AP_Beacon_Pozyx hands to the EKF ===")
        print(f"  NED north = {x_mm / 1000.0:+.3f} m")
        print(f"  NED east  = {y_mm / 1000.0:+.3f} m")
        print(f"  NED down  = {-z_mm / 1000.0:+.3f} m")
        print("  (the Pozyx backend converts ENU mm to NED m itself)")

    print()
    if failures:
        print("--- FAILURES ---")
        for failure in failures:
            print(f"  {failure}")
        print("STEP 5.2 GATE: FAILED")
        rclpy.shutdown()
        return 1

    print("STEP 5.2 GATE: PASSED")
    rclpy.shutdown()
    return 0


def _split_frames(data: bytes) -> list[tuple[int, bytes]]:
    """Split a byte stream into (msg_id, payload) pairs."""
    frames = []
    i = 0
    while i < len(data):
        if data[i] != 0x01:
            raise ValueError(f"bad frame header at offset {i}")
        msg_id = data[i + 1]
        length = data[i + 2]
        payload = data[i + 3 : i + 2 + length]
        frames.append((msg_id, payload))
        i += 3 + length
    return frames


if __name__ == "__main__":
    sys.exit(main())
