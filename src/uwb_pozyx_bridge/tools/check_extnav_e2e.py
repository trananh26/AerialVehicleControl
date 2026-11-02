#!/usr/bin/env python3
"""End-to-end check: send ODOMETRY over SERIAL1 and watch the EKF fuse it."""

import os
import sys
import time

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "uwb_pozyx_bridge",
    ),
)

from pymavlink import mavutil

from uwb_pozyx_bridge.mavlink_odometry import build_odometry_message, enu_to_ned_m

GCS_PORT = 5760
NAV_PORT = 5761


def main() -> int:
    gcs = mavutil.mavlink_connection(f"tcp:127.0.0.1:{GCS_PORT}", robust_parsing=True)
    gcs.wait_heartbeat(timeout=20)
    print(f"[gcs] heartbeat from system {gcs.target_system}")

    # Read back the parameters that matter.
    wanted = [
        "VISO_TYPE",
        "EK3_SRC1_POSXY",
        "EK3_SRC1_VELXY",
        "SERIAL1_PROTOCOL",
        "BCN_TYPE",
    ]
    for name in wanted:
        gcs.mav.param_request_read_send(
            gcs.target_system, gcs.target_component, name.encode(), -1
        )
    deadline = time.time() + 5
    found = {}
    while time.time() < deadline and len(found) < len(wanted):
        msg = gcs.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            break
        pname = msg.param_id
        if isinstance(pname, (bytes, bytearray)):
            pname = pname.rstrip(b"\x00").decode(errors="replace")
        if pname in wanted:
            found[pname] = msg.param_value
    for name in wanted:
        print(f"  {name} = {found.get(name, '<not received>')}")

    # Connect the navigation link.
    nav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{NAV_PORT}", robust_parsing=True)
    print(f"[nav] connected to {NAV_PORT}")

    # Send a fixed position in NED: 3 m north, 4 m east.
    north_m, east_m, down_m = enu_to_ned_m(4.0, 3.0, 0.0)
    print(f"[nav] sending NED north={north_m} east={east_m} down={down_m}")

    t0 = time.time()
    sent = 0
    ekf_flags = None
    local_ned = None
    statustexts = []
    while time.time() - t0 < 20:
        msg = build_odometry_message(
            north_m,
            east_m,
            down_m,
            int(time.time() * 1e6),
            position_std_m=0.05,
            quality=50,
        )
        nav.mav.send(msg)
        sent += 1

        gmsg = gcs.recv_match(
            type=["EKF_STATUS_REPORT", "LOCAL_POSITION_NED", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if gmsg is None:
            continue
        mtype = gmsg.get_type()
        if mtype == "EKF_STATUS_REPORT":
            ekf_flags = gmsg.flags
        elif mtype == "LOCAL_POSITION_NED":
            local_ned = (gmsg.x, gmsg.y, gmsg.z)
        elif mtype == "STATUSTEXT":
            text = gmsg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode(errors="replace")
            if text not in statustexts:
                statustexts.append(text)
                print(f"[statustext] {text}")

    print(f"[nav] sent {sent} ODOMETRY messages")
    if ekf_flags is not None:
        print(f"[gcs] EKF flags = 0x{ekf_flags:04x}")
    if local_ned is not None:
        print(f"[gcs] LOCAL_POSITION_NED = {local_ned}")
    for text in statustexts:
        print(f"[statustext] {text}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
