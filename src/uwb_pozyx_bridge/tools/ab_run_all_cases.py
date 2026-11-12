"""Run the four A/B cases of the external navigation yaw finding.

The four cases are:

======  =====================  ============================
case    attitude quaternion     ``EK3_SRC1_YAW``
======  =====================  ============================
A       NaN (no attitude)       6 (EXTNAV)
B       identity (yaw = 0)      6 (EXTNAV)
C       NaN (no attitude)       1 (Compass)
D       identity (yaw = 0)      1 (Compass)
======  =====================  ============================

Case B is the bug: the identity quaternion decodes to yaw = 0, and with
EXTNAV selected for yaw the EKF aligns its yaw to 0 and then fuses it
continuously, so the heading of the vehicle is dragged to North.

Case A is the fix: with a NaN quaternion there is no yaw sample at all, so
the EKF yaw is never aligned to a value the system does not know.

The script drives the two parameters at runtime, so the stack does not have
to be restarted between the cases.

Usage::

    python3 ab_run_all_cases.py --cases A,B,C,D
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

ROOT = "/home/an/ros2_project/AerialVehicleControl"

#: The cases, as ``(label, quaternion, yaw_source)``.
CASES = {
    "A": ("q=NaN, YAW=EXTNAV(6)", "unknown", 6),
    "B": ("q=identity, YAW=EXTNAV(6)", "identity", 6),
    "C": ("q=NaN, YAW=Compass(1)", "unknown", 1),
    "D": ("q=identity, YAW=Compass(1)", "identity", 1),
}


def set_node_param(name: str, value: str) -> None:
    """Set a parameter of the odometry node at runtime."""
    subprocess.run(
        [
            "ros2",
            "param",
            "set",
            "/odometry_node",
            name,
            value,
        ],
        check=True,
        cwd=ROOT,
    )


def run_case(case: str, duration: float) -> int:
    """Run one A/B case.

    Args:
        case: The case key, one of ``A``, ``B``, ``C`` or ``D``.
        duration: How long to watch the heading, in seconds.

    Returns:
        The exit code of the case script.
    """
    label, quaternion, yaw_source = CASES[case]

    print()
    print("#" * 72)
    print(f"# CASE {case}: {label}")
    print("#" * 72)

    print(f"[case {case}] setting attitude_quaternion = {quaternion}")
    set_node_param("attitude_quaternion", quaternion)
    time.sleep(3.0)

    result = subprocess.run(
        [
            "python3",
            "-u",
            f"{ROOT}/src/uwb_pozyx_bridge/tools/ab_test_yaw_source.py",
            "--yaw-source",
            str(yaw_source),
            "--duration",
            str(duration),
            "--label",
            label,
        ],
        cwd=ROOT,
    )
    return result.returncode


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", default="A,B,C,D")
    parser.add_argument("--duration", type=float, default=40.0)
    args = parser.parse_args(argv)

    for case in args.cases.split(","):
        case = case.strip().upper()
        if case not in CASES:
            print(f"unknown case '{case}'")
            return 1
        run_case(case, args.duration)

    return 0


if __name__ == "__main__":
    sys.exit(main())
