"""Minimal MAVLink monitor for verifying the SITL beacon pipeline.

Connects to the SITL MAVLink port, optionally sets the beacon/EKF parameters,
then reports the EKF status so the range-beacon fusion can be verified.

The key signal is ``EKF_STATUS_REPORT`` (MAVLink message 193):

* ``EKF_POS_HORIZ_ABS``  (bit 1)  - horizontal position is absolute
* ``EKF_POS_VERT_ABS``   (bit 2)  - vertical position is absolute
* ``EKF_POS_HORIZ_REL``  (bit 3)  - horizontal position is relative
* ``EKF_UNINITIALIZED``   (bit 8)  - the filter is not initialised yet

Usage::

    python3 sitl_beacon_monitor.py --mavlink-port 5760
"""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


BEACON_PARAMS = {
    "BCN_TYPE": 1,  # Pozyx
    "BCN_LATITUDE": -35.363261,
    "BCN_LONGITUDE": 149.165230,
    "BCN_ALT": 584,
    "BCN_ORIENT_YAW": 0,
    "EK3_ENABLE": 1,
    "EK2_ENABLE": 0,
    "EK3_SRC1_POSXY": 4,  # Beacon
    "EK3_SRC1_POSZ": 1,  # Baro
    "EK3_SRC1_VELXY": 0,  # None
    "EK3_SRC1_VELZ": 0,  # None
    "EK3_SRC1_YAW": 1,  # Compass
    "AHRS_EKF_TYPE": 3,
    "GPS1_TYPE": 0,
    "GPS2_TYPE": 0,
    # Upstream ARMING_SKIPCHK=8 (skip the GPS arming check) is not available in
    # this ArduPilot build; use the ARMING_CHECK bitmask instead, clearing the
    # "GPS lock" bit (bit 3) while keeping every other check enabled.
    "ARMING_CHECK": 1048567,
    "SERIAL1_PROTOCOL": 13,  # SerialProtocol_Beacon
    "SERIAL1_BAUD": 115,
    "BRD_SER1_RTSCTS": 0,
}

# EKF_STATUS_REPORT flags (mavlink common.xml)
EKF_ATTITUDE = 1 << 0
EKF_VELOCITY_HORIZ = 1 << 1
EKF_VELOCITY_VERT = 1 << 2
EKF_POS_HORIZ_REL = 1 << 3
EKF_POS_HORIZ_ABS = 1 << 4
EKF_POS_VERT_ABS = 1 << 5
EKF_POS_VERT_AGL = 1 << 6
EKF_CONST_POS_MODE = 1 << 7
EKF_UNINITIALIZED = 1 << 8
EKF_PRED_POS_HORIZ_REL = 1 << 9
EKF_PRED_POS_HORIZ_ABS = 1 << 10


def describe_ekf_flags(flags: int) -> str:
    """Human readable summary of EKF_STATUS_REPORT flags."""
    names = []
    for bit, label in (
        (EKF_ATTITUDE, "Attitude"),
        (EKF_VELOCITY_HORIZ, "VelH"),
        (EKF_VELOCITY_VERT, "VelV"),
        (EKF_POS_HORIZ_REL, "PosHRel"),
        (EKF_POS_HORIZ_ABS, "PosHAbs"),
        (EKF_POS_VERT_ABS, "PosVAbs"),
        (EKF_CONST_POS_MODE, "ConstPosMode"),
        (EKF_UNINITIALIZED, "UNINITIALIZED"),
        (EKF_PRED_POS_HORIZ_REL, "PredPosHRel"),
        (EKF_PRED_POS_HORIZ_ABS, "PredPosHAbs"),
    ):
        if flags & bit:
            names.append(label)
    return ",".join(names) if names else "(none)"


def set_params(mav, params: dict) -> None:
    """Set parameters one at a time."""
    for name, value in params.items():
        mav.mav.param_set_send(
            mav.target_system,
            mav.target_component,
            name.encode(),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        time.sleep(0.05)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="SITL beacon pipeline monitor")
    parser.add_argument("--mavlink-port", type=int, default=5760)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--set-params", action="store_true")
    args = parser.parse_args(argv)

    print(f"[monitor] connecting to MAVLink on {args.host}:{args.mavlink_port}")
    mav = mavutil.mavlink_connection(
        f"tcp:{args.host}:{args.mavlink_port}", robust_parsing=True
    )

    mav.wait_heartbeat(timeout=30)
    print(f"[monitor] heartbeat from system {mav.target_system}")

    # Ask for the streams we want to observe.  LOCAL_POSITION_NED and
    # EKF_STATUS_REPORT are not sent by default.
    for stream_id in (
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
    ):
        mav.mav.request_data_stream_send(
            mav.target_system,
            mav.target_component,
            stream_id,
            10,  # 10 Hz
            1,  # enabled
        )
    time.sleep(0.5)

    if args.set_params:
        print("[monitor] setting beacon parameters")
        set_params(mav, BEACON_PARAMS)
        time.sleep(2.0)

    print("[monitor] reading back parameters:")
    wanted = {
        "BCN_TYPE",
        "BCN_ORIENT_YAW",
        "EK3_SRC1_POSXY",
        "EK3_SRC1_POSZ",
        "EK3_SRC1_VELXY",
        "EK3_SRC1_VELZ",
        "EK3_SRC1_YAW",
        "ARMING_CHECK",
        "SERIAL1_PROTOCOL",
        "SERIAL1_BAUD",
        "BRD_SER1_RTSCTS",
        "GPS1_TYPE",
    }
    for name in wanted:
        mav.mav.param_request_read_send(
            mav.target_system, mav.target_component, name.encode(), -1
        )
    found = {}
    deadline = time.time() + 5.0
    while time.time() < deadline and len(found) < len(wanted):
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            break
        try:
            pname = msg.param_id
            if isinstance(pname, (bytes, bytearray)):
                pname = pname.rstrip(b"\x00").decode(errors="replace")
            pname = str(pname)
        except Exception:
            continue
        if pname in wanted:
            found[pname] = msg.param_value
    for name in sorted(wanted):
        if name in found:
            print(f"  {name} = {found[name]}")
        else:
            print(f"  {name}: <not received>")

    print(f"[monitor] watching EKF status for {args.duration:.0f}s")

    start = time.time()
    last_report = 0.0
    ekf_flags = 0
    pos = None
    local_pos = None
    counts = {}

    while time.time() - start < args.duration:
        msg = mav.recv_match(
            type=[
                "EKF_STATUS_REPORT",
                "STATUSTEXT",
                "GLOBAL_POSITION_INT",
                "LOCAL_POSITION_NED",
            ],
            blocking=True,
            timeout=2,
        )
        if msg is None:
            continue

        mtype = msg.get_type()
        counts[mtype] = counts.get(mtype, 0) + 1

        if mtype == "EKF_STATUS_REPORT":
            ekf_flags = msg.flags
        elif mtype == "STATUSTEXT":
            print(f"[statustext] {msg.text}")
        elif mtype == "GLOBAL_POSITION_INT":
            pos = (msg.lat, msg.lon, msg.alt, msg.relative_alt)
        elif mtype == "LOCAL_POSITION_NED":
            local_pos = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

        now = time.time()
        if now - last_report >= 2.0:
            last_report = now
            line = f"[ekf] flags=0x{ekf_flags:04x} [{describe_ekf_flags(ekf_flags)}]"
            if pos is not None:
                line += (
                    f" gpos=({pos[0] / 1e7:.6f},{pos[1] / 1e7:.6f})"
                    f" alt={pos[2] / 1e3:.2f} relalt={pos[3] / 1e3:.2f}"
                )
            if local_pos is not None:
                line += (
                    f" ned=({local_pos[0]:.2f},{local_pos[1]:.2f},{local_pos[2]:.2f})"
                    f" vel=({local_pos[3]:.2f},{local_pos[4]:.2f},{local_pos[5]:.2f})"
                )
            print(line)

    print("[monitor] message counts:", counts)
    print("[monitor] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
