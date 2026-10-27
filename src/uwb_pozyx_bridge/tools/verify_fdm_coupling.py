"""Verify that SITL and Gazebo are actually coupled through the FDM port.

The earlier pipeline ran SITL with its own internal flight model, so the
vehicle never responded to the simulated world.  This script checks the
coupling by commanding a takeoff and watching the vehicle's altitude in
Gazebo.

It reads:

* the Gazebo truth, from the UWB plugin's tag pose topic
* the EKF altitude, from ArduPilot over MAVLink

Both have to move together.  If SITL were still simulating on its own, the
Gazebo truth would stay on the ground while the EKF climbed, or vice versa.

Usage::

    python3 verify_fdm_coupling.py --udp-port 14550
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class TruthCollector(Node):
    """Collects the vehicle's position as Gazebo sees it."""

    def __init__(self) -> None:
        super().__init__("fdm_truth_collector")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.latest: tuple[float, float, float] | None = None
        self.create_subscription(Pose, "/uwb/tag_pose_ros", self._on_truth, qos)

    def _on_truth(self, msg: Pose) -> None:
        self.latest = (
            msg.position.x,
            msg.position.y,
            msg.position.z,
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--udp-port", type=int, default=14550)
    parser.add_argument("--climb", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=12.0)
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = TruthCollector()

    mav = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.udp_port}",
        source_system=255,
        source_component=0,
    )
    mav.wait_heartbeat(timeout=30)
    print(f"[fdm] heartbeat from system {mav.target_system}")

    # The autopilot component is 1 (MAV_COMP_ID_AUTOPILOT1).  The heartbeat
    # is broadcast with component 0, so target_component has to be fixed up
    # explicitly.
    autopilot = 1

    # Arm and take off.  The EKF has to be on the beacon position source
    # and has to have a position estimate before the vehicle will arm, so
    # wait for that first.  The beacon frames start flowing a few seconds
    # after the Pozyx node connects.
    #
    # ArduPilot keeps EKF_UNINITIALIZED (bit 8) set even once it has a
    # position, so the readiness test only looks at the position bits.
    print("[fdm] waiting for the EKF to have a position estimate")
    deadline = time.time() + 60.0
    ekf_ready = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        ekf = mav.recv_match(
            type="EKF_STATUS_REPORT", blocking=True, timeout=1.0
        )
        if ekf is None:
            continue
        flags = ekf.flags
        # EKF_POS_HORIZ_REL (bit 3) or EKF_POS_HORIZ_ABS (bit 4).
        if flags & ((1 << 3) | (1 << 4)):
            ekf_ready = True
            print(f"[fdm] EKF has a position estimate (flags {flags:#x})")
            break
    if not ekf_ready:
        print("[fdm] the EKF never got a position estimate")
        rclpy.shutdown()
        return 1

    # Arming can still be refused for a moment after the EKF reports a
    # position, so retry until the vehicle is armed.
    #
    # The RC override is needed because SITL has no RC input: without it
    # the throttle is zero, and ArduPilot disarms a vehicle whose throttle
    # is at zero.  Only the throttle and the arm switch are overridden, so
    # the mode channel stays free for MAV_CMD_DO_SET_MODE.
    #
    # The throttle sits at its minimum (1000).  A higher value makes
    # ArduPilot refuse the mode change with "throttle too high".
    def rc_override():
        mav.mav.rc_channels_override_send(
            mav.target_system,
            autopilot,
            1000,  # ch1 throttle, at the minimum
            1500,  # ch2 yaw
            1500,  # ch3 pitch
            2000,  # ch4 arm switch
            1500,  # ch5 mode channel
            1500,
            1500,
            1500,
        )

    def set_guided():
        # MAV_CMD_DO_SET_MODE takes the mode as base_mode | custom_mode.
        # GUIDED is 4, and the custom mode of GUIDED is 4 as well.
        mav.mav.command_long_send(
            mav.target_system,
            autopilot,
            176,  # MAV_CMD_DO_SET_MODE
            0,
            1,  # base mode: CUSTOM_MODE_ENABLED
            4,  # custom mode: GUIDED
            0,
            0,
            0,
            0,
            0,
            0,
        )

    # The mode is switched before arming: ArduPilot refuses a mode change
    # on an armed vehicle when the throttle is not at its minimum.
    print("[fdm] switching to GUIDED")
    rc_override()
    set_guided()
    time.sleep(1.5)

    print("[fdm] arming")

    armed = False
    deadline = time.time() + 30.0
    while time.time() < deadline and not armed:
        rc_override()
        mav.mav.command_long_send(
            mav.target_system,
            autopilot,
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            0,
            1,  # arm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        for _ in range(10):
            time.sleep(0.2)
            rc_override()
            # The heartbeat is forwarded by MAVProxy, so it can lag a
            # little behind the actual state; read whatever is available
            # and look at both the heartbeat and the status text.
            for _ in range(3):
                msg = mav.recv_match(
                    type=["HEARTBEAT", "STATUSTEXT"],
                    blocking=True,
                    timeout=0.3,
                )
                if msg is None:
                    continue
                if msg.get_type() == "STATUSTEXT":
                    print(f"[fdm]   {msg.text}")
                    if "armed" in msg.text.lower():
                        armed = True
                elif msg.base_mode & 128:
                    armed = True
                if armed:
                    break
            if armed:
                break
    if not armed:
        print("[fdm] the vehicle did not arm")
        rclpy.shutdown()
        return 1
    print("[fdm] armed")

    print(f"[fdm] taking off to {args.climb} m")
    mav.mav.command_long_send(
        mav.target_system,
        autopilot,
        22,  # MAV_CMD_NAV_TAKEOFF
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        args.climb,
    )

    deadline = time.time() + args.settle
    truth: list[float] = []
    ekf: list[float] = []
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        # Keep the RC override alive, otherwise the throttle drops back to
        # zero and ArduPilot disarms the vehicle.
        rc_override()
        if node.latest is not None:
            truth.append(node.latest[2])
        msg = mav.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=0.2
        )
        if msg is not None:
            # LOCAL_POSITION_NED is relative to the EKF origin, in NED.
            ekf.append(-msg.z)

    print()
    print("=== altitude (metres) ===")
    if truth:
        print(f"  gazebo truth : first {truth[0]:.3f}  last {truth[-1]:.3f}")
    else:
        print("  gazebo truth : no samples")
    if ekf:
        print(f"  ekf          : first {ekf[0]:.3f}  last {ekf[-1]:.3f}")
    else:
        print("  ekf          : no samples")

    failures = []
    if not truth or not ekf:
        failures.append("no position samples from one of the two sources")
    else:
        truth_climb = truth[-1] - truth[0]
        ekf_climb = ekf[-1] - ekf[0]
        print(f"  gazebo climbed {truth_climb:+.3f} m")
        print(f"  ekf    climbed {ekf_climb:+.3f} m")
        if abs(truth_climb) < 0.5:
            failures.append(
                f"the vehicle did not move in Gazebo ({truth_climb:+.3f} m); "
                "SITL is probably not driving the simulation"
            )
        # The EKF lags behind the simulation while the vehicle is
        # accelerating, so the two climb distances are not equal.  What
        # matters is that the EKF ends up in the same place, so the
        # altitude difference is compared instead of the climb distance.
        # The EKF's origin is the beacon origin, which sits 0.1 m below the
        # spawn point, so a small constant offset is expected.
        truth_alt = truth[-1]
        ekf_alt = ekf[-1]
        offset = truth_alt - ekf_alt
        print(f"  altitude difference (gazebo - ekf): {offset:+.3f} m")
        if abs(offset) > 1.5:
            failures.append(
                f"the EKF and Gazebo disagree on the altitude by "
                f"{abs(offset):.3f} m"
            )

    print()
    if failures:
        for name in failures:
            print(f"FAILED: {name}")
        print("FDM COUPLING: FAILED")
        rclpy.shutdown()
        return 1

    print("FDM COUPLING: PASSED")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
