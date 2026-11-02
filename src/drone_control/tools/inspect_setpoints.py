#!/usr/bin/env python3
"""Log the MAVLink setpoint messages MAVROS sends, plus the positions.

Run alongside circle_mission to see what the FCU actually receives:

    python3 tools/inspect_setpoints.py --duration 60

It reads the MAVProxy UDP output (14550) and the ROS pose topics, so the
setpoint stream and the measured position can be compared side by side.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class PoseCollector(Node):
    def __init__(self) -> None:
        super().__init__("setpoint_inspector")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.latest: tuple[float, float, float] | None = None
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_pose, qos
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.latest = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )


#: MAV_FRAME ids, so the frame can be printed by name.
FRAMES = {
    0: "GLOBAL",
    1: "LOCAL_NED",
    2: "MISSION",
    3: "GLOBAL_RELATIVE_ALT",
    4: "LOCAL_ENU",
    5: "GLOBAL_INT",
    6: "GLOBAL_RELATIVE_ALT_INT",
    7: "LOCAL_OFFSET_NED",
    8: "BODY_NED",
    9: "BODY_OFFSET_NED",
    10: "GLOBAL_TERRAIN_ALT",
    11: "GLOBAL_TERRAIN_ALT_INT",
    12: "BODY_FRD",
    13: "RESERVED_13",
    14: "LOCAL_FRD",
    15: "LOCAL_FLU",
    20: "LOCAL_FRD_OFFSET",
    21: "LOCAL_FLU_OFFSET",
}

#: The type_mask bits, so the payload can be explained.
TYPE_MASK_BITS = (
    (1 << 0, "POS_IGNORE"),
    (1 << 1, "VEL_IGNORE"),
    (1 << 2, "ACC_IGNORE"),
    (1 << 3, "FORCE_SET"),
    (1 << 4, "YAW_IGNORE"),
    (1 << 5, "YAW_RATE_IGNORE"),
)


def describe_mask(mask: int) -> str:
    names = [name for bit, name in TYPE_MASK_BITS if mask & bit]
    return "|".join(names) if names else "(none)"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--udp-port", type=int, default=14550)
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = PoseCollector()

    mav = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.udp_port}",
        source_system=255,
        source_component=0,
    )
    mav.wait_heartbeat(timeout=30)
    print(f"[sp] heartbeat from system {mav.target_system}")

    deadline = time.time() + args.duration
    last_sp = 0.0
    n_sp = 0
    frames_seen: set[int] = set()
    masks_seen: set[int] = set()

    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        msg = mav.recv_match(
            type="SET_POSITION_TARGET_LOCAL_NED", blocking=True, timeout=0.2
        )
        if msg is None:
            continue
        n_sp += 1
        frames_seen.add(msg.coordinate_frame)
        masks_seen.add(msg.type_mask)

        now = time.time()
        if now - last_sp >= 1.0:
            last_sp = now
            pos = node.latest
            pos_txt = (
                f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})"
                if pos
                else "(no pose)"
            )
            print(
                f"[sp] frame={FRAMES.get(msg.coordinate_frame, msg.coordinate_frame)} "
                f"mask={describe_mask(msg.type_mask)} "
                f"v=({msg.vx:+.3f}, {msg.vy:+.3f}, {msg.vz:+.3f}) "
                f"p=({msg.x:+.3f}, {msg.y:+.3f}, {msg.z:+.3f}) "
                f"yaw={msg.yaw:+.3f} "
                f"pos={pos_txt}"
            )

    print()
    print(f"[sp] setpoints received: {n_sp}")
    print(f"[sp] frames seen: {[FRAMES.get(f, f) for f in sorted(frames_seen)]}")
    print(f"[sp] masks seen: {[describe_mask(m) for m in sorted(masks_seen)]}")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
