"""Diagnostic: why is the EKF3 not leaving AID_NONE when using a beacon source?

Dumps every STATUSTEXT message, the EKF status reports, and a set of
interesting parameters so the blocking readiness condition can be identified.
"""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

PARAMS = [
    "BCN_TYPE",
    "BCN_LATITUDE",
    "BCN_LONGITUDE",
    "BCN_ALT",
    "BCN_ORIENT_YAW",
    "EK3_ENABLE",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "EK3_MAG_CHECK",
    "EK3_COMPASS_CHECK",
    "EK3_RNGBCN_DELAY",
    "EK3_RNGBCN_NOISE",
    "EK3_RNGBCN_HGT_ERR",
    "AHRS_EKF_TYPE",
    "GPS1_TYPE",
    "COMPASS_TYPE",
    "COMPASS_DEV_ID",
    "COMPASS_ORIENT",
    "MAG1_TYPE",
    "MAG1_DEV_ID",
    "MAG2_TYPE",
    "ARMING_CHECK",
    "SERIAL1_PROTOCOL",
    "SERIAL1_BAUD",
]


def decode_flags(flags: int) -> str:
    names = []
    for bit, label in (
        (0, "Attitude"),
        (1, "VelH"),
        (2, "VelV"),
        (3, "PosHRel"),
        (4, "PosHAbs"),
        (5, "PosVAbs"),
        (6, "PosVAGL"),
        (7, "ConstPosMode"),
        (8, "UNINITIALIZED"),
        (9, "PredPosHRel"),
        (10, "PredPosHAbs"),
        (15, "GPSGlitching"),
    ):
        if flags & (1 << bit):
            names.append(label)
    return ",".join(names) if names else "(none)"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink-port", type=int, default=5760)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--duration", type=float, default=45.0)
    args = parser.parse_args(argv)

    mav = mavutil.mavlink_connection(
        f"tcp:{args.host}:{args.mavlink_port}", robust_parsing=True
    )
    mav.wait_heartbeat(timeout=30)
    print(f"[diag] heartbeat from system {mav.target_system}")

    for name in PARAMS:
        mav.mav.param_request_read_send(
            mav.target_system, mav.target_component, name.encode(), -1
        )
    time.sleep(2.0)

    found = {}
    deadline = time.time() + 4.0
    while time.time() < deadline and len(found) < len(PARAMS):
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
        found[pname] = msg.param_value

    print("[diag] parameters:")
    for name in PARAMS:
        if name in found:
            print(f"  {name} = {found[name]}")

    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        5,
        1,
    )

    print(f"[diag] collecting for {args.duration:.0f}s")
    start = time.time()
    last_flags = None
    counts = {}
    while time.time() - start < args.duration:
        msg = mav.recv_match(
            type=["STATUSTEXT", "EKF_STATUS_REPORT", "EKF_STATUS_REPORT2"],
            blocking=True,
            timeout=2,
        )
        if msg is None:
            continue
        mtype = msg.get_type()
        counts[mtype] = counts.get(mtype, 0) + 1
        if mtype == "STATUSTEXT":
            print(f"[text] {msg.text}")
        elif mtype == "EKF_STATUS_REPORT":
            if msg.flags != last_flags:
                last_flags = msg.flags
                print(f"[ekf1] flags=0x{msg.flags:04x} [{decode_flags(msg.flags)}]")
        else:
            print(
                f"[ekf2] flags=0x{msg.flags:04x} vel={msg.vel_horiz:.2f} "
                f"pos_horiz_rel={msg.pos_horiz_rel:.2f} hgt={msg.pos_vert:.2f}"
            )

    print("[diag] counts:", counts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
