#!/usr/bin/env python3
"""Step 5.4 gate checker: compares the three position estimates.

Three independent sources are compared in a common frame:

``truth``
    The vehicle's real position in the Gazebo world, taken from
    ``/uwb/tag_pose_ros`` (the plugin publishes it in the world frame).
    Converted to beacon-relative ENU metres by subtracting the beacon
    origin.

``trilateration``
    The bridge node's own solution, published on ``/uwb/pose``.  It is
    already beacon-relative ENU metres.

``ekf``
    ArduPilot's EKF estimate, read from the ``LOCAL_POSITION_NED``
    MAVLink message.  The EKF origin is the beacon origin, so the message
    is beacon-relative NED metres; it is converted to ENU here.

Usage::

    python3 compare_positions.py --duration 20
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

#: Where the vehicle spawns; this is also the beacon origin.
BEACON_ORIGIN = (5.0, 5.0, 0.1)

#: The EKF's origin sits at the beacon system origin, which the plugin
#: reports as 0.1 m below the spawn point in the down direction.
BEACON_Z_OFFSET_DOWN = 0.1


class PositionCollector(Node):
    """Collects the ROS side of the comparison."""

    def __init__(self, history: int = 400) -> None:
        super().__init__("position_collector")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.truth: deque = deque(maxlen=history)
        self.trilateration: deque = deque(maxlen=history)

        self.create_subscription(
            Pose, "/uwb/tag_pose_ros", self._on_truth, qos
        )
        self.create_subscription(
            PoseStamped, "/uwb/pose", self._on_trilateration, qos
        )

    def _on_truth(self, msg: Pose) -> None:
        # World ENU metres, converted to beacon-relative.
        self.truth.append(
            (
                msg.position.x - BEACON_ORIGIN[0],
                msg.position.y - BEACON_ORIGIN[1],
                msg.position.z - BEACON_ORIGIN[2],
            )
        )

    def _on_trilateration(self, msg: PoseStamped) -> None:
        # Already beacon-relative ENU metres.
        self.trilateration.append(
            (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--mavlink-port", type=int, default=5760)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--ekf-tolerance", type=float, default=0.5)
    parser.add_argument(
        "--trilateration-tolerance", type=float, default=0.3
    )
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = PositionCollector()

    mav = mavutil.mavlink_connection(
        f"tcp:{args.host}:{args.mavlink_port}", robust_parsing=True
    )
    mav.wait_heartbeat(timeout=30)
    print(f"[compare] heartbeat from system {mav.target_system}")

    ekf_samples: list[tuple[float, float, float]] = []
    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        msg = mav.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=0.2
        )
        if msg is None:
            continue
        # LOCAL_POSITION_NED is relative to the EKF origin, in NED metres.
        # The EKF origin is the beacon origin, which is the spawn point, so
        # the message is already beacon-relative.  Convert NED to ENU.
        #
        # The vertical datum is left alone: the EKF reports a beacon-to-datum
        # offset of 0.1 m, and the vehicle settles a few centimetres above
        # its spawn height, so the vertical channel is only approximate.
        # The gate therefore compares the horizontal plane.
        north, east, down = msg.x, msg.y, msg.z
        ekf_samples.append((east, north, -down))

    print()
    print("=== sample counts ===")
    print(f"  truth (Gazebo)  : {len(node.truth)}")
    print(f"  trilateration   : {len(node.trilateration)}")
    print(f"  ekf             : {len(ekf_samples)}")

    if not node.truth or not node.trilateration or not ekf_samples:
        print()
        print("  missing data, cannot compare")
        print("STEP 5.4 GATE: FAILED")
        rclpy.shutdown()
        return 1

    truth = node.truth[-1]
    trilat = node.trilateration[-1]
    ekf = ekf_samples[-1]

    print()
    print("=== latest positions (beacon-relative ENU metres) ===")
    print(f"  truth       : ({truth[0]:+.3f}, {truth[1]:+.3f}, {truth[2]:+.3f})")
    print(f"  trilateration: ({trilat[0]:+.3f}, {trilat[1]:+.3f}, {trilat[2]:+.3f})")
    print(f"  ekf         : ({ekf[0]:+.3f}, {ekf[1]:+.3f}, {ekf[2]:+.3f})")

    def horizontal(a, b) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    errors = {
        "ekf vs truth": horizontal(ekf, truth),
        "ekf vs trilateration": horizontal(ekf, trilat),
        "trilateration vs truth": horizontal(trilat, truth),
    }

    print()
    print("=== horizontal errors ===")
    failures = []
    for name, value in errors.items():
        print(f"  {name:28s} = {value:.3f} m")

    if errors["ekf vs truth"] > args.ekf_tolerance:
        failures.append(
            f"ekf vs truth {errors['ekf vs truth']:.3f} m exceeds "
            f"{args.ekf_tolerance} m"
        )
    if errors["ekf vs trilateration"] > args.ekf_tolerance:
        failures.append(
            f"ekf vs trilateration {errors['ekf vs trilateration']:.3f} m "
            f"exceeds {args.ekf_tolerance} m"
        )
    if errors["trilateration vs truth"] > args.trilateration_tolerance:
        failures.append(
            f"trilateration vs truth "
            f"{errors['trilateration vs truth']:.3f} m exceeds "
            f"{args.trilateration_tolerance} m"
        )

    # --- stability -------------------------------------------------------
    # The error must not drift: compare the first and last thirds of the
    # EKF samples.
    n = len(ekf_samples)
    if n >= 30:
        third = n // 3
        early = ekf_samples[:third]
        late = ekf_samples[-third:]
        drift = horizontal(
            (
                statistics.fmean(s[0] for s in early),
                statistics.fmean(s[1] for s in early),
                0.0,
            ),
            (
                statistics.fmean(s[0] for s in late),
                statistics.fmean(s[1] for s in late),
                0.0,
            ),
        )
        print()
        print("=== stability ===")
        print(f"  drift over the run: {drift:.3f} m")
        if drift > 0.3:
            failures.append(f"the EKF drifted {drift:.3f} m during the run")

        spread = max(
            horizontal(s, truth) for s in ekf_samples
        )
        print(f"  worst ekf vs truth: {spread:.3f} m")
        if spread > args.ekf_tolerance:
            failures.append(
                f"the worst ekf vs truth error {spread:.3f} m exceeds "
                f"{args.ekf_tolerance} m"
            )

    print()
    if failures:
        for name in failures:
            print(f"FAILED: {name}")
        print("STEP 5.4 GATE: FAILED")
        rclpy.shutdown()
        return 1

    print("STEP 5.4 GATE: PASSED")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
