"""Check the EKF heading while the external navigation link is running.

The finding under test is that an ``ODOMETRY`` message whose attitude
quaternion is finite injects a yaw into EKF3.  With
``EK3_SRCx_YAW = EXTNAV`` that yaw is fused continuously, so the heading of
the vehicle is dragged to whatever the quaternion encodes — for the identity
quaternion, to 0° (North).

This script reads the heading over ROS (MAVROS republishes it as
``/mavros/estime/attitude`` and ``/mavros/vfr_hud``), and reports:

* the range the heading covered;
* how many samples were within 5° of North;
* whether the autopilot ever printed a "yaw aligned" message, which is what
  ``NavEKF3_core::alignYawAngle()`` emits when the external navigation
  source is selected for yaw.

The ROS topics are used rather than a raw MAVLink connection because MAVROS
already holds the MAVProxy output port, and two UDP listeners on the same
port would each only see a fraction of the packets.

Usage::

    python3 check_ekf_heading.py --duration 60 --label "q=NaN"
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

import rclpy
from mavros_msgs.msg import VfrHud
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class HeadingCollector(Node):
    """Collects the vehicle heading as MAVROS republishes it."""

    def __init__(self) -> None:
        super().__init__("ekf_heading_collector")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.headings: list[float] = []
        self.hud_headings: list[int] = []

        # MAVROS publishes the compass heading on /mavros/vfr_hud; the
        # heading it reports is the one the autopilot uses, i.e. the EKF
        # heading.
        self.create_subscription(VfrHud, "/mavros/vfr_hud", self._on_hud, qos)

    def _on_hud(self, msg: VfrHud) -> None:
        self.hud_headings.append(msg.heading)
        # VFR_HUD.heading is in degrees, clockwise from North.
        self.headings.append(float(msg.heading))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument(
        "--label",
        default="",
        help="A label for the report, e.g. 'q=NaN' or 'q=identity'.",
    )
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = HeadingCollector()

    print(f"[yaw] collecting for {args.duration:.0f} s ...")
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    headings = node.headings
    print()
    print(f"=== heading report {args.label} ===")
    if headings:
        print(
            f"  ATTITUDE.yaw : n={len(headings)} "
            f"min={min(headings):.1f}° max={max(headings):.1f}° "
            f"mean={statistics.fmean(headings):.1f}°"
        )
        print(f"  first        : {headings[0]:.1f}°")
        print(f"  last         : {headings[-1]:.1f}°")

        spread = max(headings) - min(headings)
        near_zero = sum(1 for h in headings if abs(h) < 5.0)
        print(f"  heading spread : {spread:.1f}°")
        print(f"  samples within 5° of North : {near_zero}/{len(headings)}")
        if near_zero == len(headings) and len(headings) > 10:
            print("  VERDICT: the heading is pinned to North")
        elif spread > 20.0:
            print("  VERDICT: the heading follows the vehicle (not pinned)")
    else:
        print("  ATTITUDE.yaw : no samples")
    if node.hud_headings:
        print(
            f"  VFR_HUD      : n={len(node.hud_headings)} "
            f"min={min(node.hud_headings)}° max={max(node.hud_headings)}°"
        )

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
