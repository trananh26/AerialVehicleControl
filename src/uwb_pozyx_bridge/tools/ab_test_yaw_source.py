"""A/B test the external navigation yaw source.

The finding under test is that an ``ODOMETRY`` message with a **finite**
attitude quaternion injects a yaw into EKF3.  With
``EK3_SRCx_YAW = EXTNAV`` (6) that yaw is fused continuously, so the
heading of the vehicle is dragged to whatever the quaternion encodes — for
the identity quaternion, to 0° (North).

This script drives the comparison:

1. it sets ``EK3_SRC1_YAW`` to the requested source;
2. it watches the heading for a while;
3. it reports whether the heading was pinned to North.

The identity quaternion is used as the "bad" case.  It is what the ESP32
tag sent before the fix, and it decodes to yaw = 0.

Usage::

    python3 ab_test_yaw_source.py --yaw-source 6 --duration 40
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

from pymavlink import mavutil

#: ``AP_NavEKF_Source::SourceYaw::COMPASS``.
SOURCE_YAW_COMPASS = 1
#: ``AP_NavEKF_Source::SourceYaw::EXTNAV``.
SOURCE_YAW_EXTNAV = 6

#: The parameter that selects the yaw source of the first EKF3 instance.
YAW_SOURCE_PARAM = "EK3_SRC1_YAW"


class HeadingCollector(Node):
    """Collects the vehicle heading as MAVROS republishes it."""

    def __init__(self) -> None:
        super().__init__("ab_heading_collector")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.headings: list[float] = []
        self.create_subscription(VfrHud, "/mavros/vfr_hud", self._on_hud, qos)

    def _on_hud(self, msg: VfrHud) -> None:
        # VFR_HUD.heading is in degrees, clockwise from North.
        self.headings.append(float(msg.heading))


def set_param(mav, name: str, value: float) -> bool:
    """Set a parameter and wait for the acknowledgement.

    Args:
        mav: The MAVLink connection.
        name: The parameter name.
        value: The value to set.

    Returns:
        Whether the parameter was acknowledged.
    """
    mav.param_set_send(name, value)

    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.param_id == name and abs(msg.param_value - value) < 1e-4:
            return True
    return False


def get_param(mav, name: str) -> float | None:
    """Read a parameter.

    Args:
        mav: The MAVLink connection.
        name: The parameter name.

    Returns:
        The value, or ``None`` if it could not be read.
    """
    mav.param_fetch_one(name)

    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.param_id == name:
            return msg.param_value
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--udp-port", type=int, default=14551)
    parser.add_argument("--yaw-source", type=int, default=SOURCE_YAW_EXTNAV)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--label", default="")
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = HeadingCollector()

    mav = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.udp_port}",
        source_system=255,
        source_component=0,
    )
    mav.wait_heartbeat(timeout=30)
    print(f"[ab] heartbeat from system {mav.target_system}")

    current = get_param(mav, YAW_SOURCE_PARAM)
    print(f"[ab] {YAW_SOURCE_PARAM} is currently {current}")
    if current is not None and abs(current - args.yaw_source) > 1e-6:
        print(f"[ab] setting {YAW_SOURCE_PARAM} = {args.yaw_source}")
        if not set_param(mav, YAW_SOURCE_PARAM, float(args.yaw_source)):
            print(f"[ab] FAILED to set {YAW_SOURCE_PARAM}")
            rclpy.shutdown()
            return 1
        print(f"[ab] {YAW_SOURCE_PARAM} is now {get_param(mav, YAW_SOURCE_PARAM)}")
        # Give the EKF a moment to re-initialise with the new source.
        time.sleep(5.0)

    print(f"[ab] collecting the heading for {args.duration:.0f} s ...")
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    headings = node.headings
    print()
    print(f"=== A/B heading report {args.label} ===")
    if not headings:
        print("  no heading samples")
        rclpy.shutdown()
        return 1

    spread = max(headings) - min(headings)
    near_zero = sum(1 for h in headings if abs(h) < 5.0)
    print(
        f"  heading  : n={len(headings)} "
        f"min={min(headings):.1f}° max={max(headings):.1f}° "
        f"mean={statistics.fmean(headings):.1f}°"
    )
    print(f"  spread   : {spread:.1f}°")
    print(f"  within 5° of North : {near_zero}/{len(headings)}")
    if near_zero == len(headings):
        print("  VERDICT: PINNED TO NORTH")
    elif spread > 20.0:
        print("  VERDICT: the heading follows the vehicle")
    else:
        print("  VERDICT: the heading barely moved")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
