#!/usr/bin/env python3
"""Check that the EKF follows the position sent over the navigation link.

The vehicle is not simulated (there is no Gazebo), so the EKF has no other
position source.  Sending a sequence of known positions over SERIAL1 and
reading back LOCAL_POSITION_NED shows whether the fusion actually tracks the
external navigation data.
"""

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

from uwb_pozyx_bridge.mavlink_odometry import build_odometry_message

GCS_PORT = 5760
NAV_PORT = 5761


def main() -> int:
    gcs = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{GCS_PORT}", robust_parsing=True
    )
    gcs.wait_heartbeat(timeout=20)
    print(f"[gcs] heartbeat from system {gcs.target_system}")

    gcs.mav.request_data_stream_send(
        gcs.target_system,
        gcs.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )
    time.sleep(0.5)

    nav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{NAV_PORT}", robust_parsing=True)
    print(f"[nav] connected to {NAV_PORT}")

    # A short ramp: north from 0 to 10 m, east fixed at 0.
    steps = 40
    sent = 0
    ekf_flags = 0
    last_ned = None
    t0 = time.time()

    while time.time() - t0 < 15:
        fraction = sent / steps
        north = 10.0 * fraction

        msg = build_odometry_message(
            north,
            0.0,
            0.0,
            int(time.time() * 1e6),
            position_std_m=0.05,
            quality=50,
        )
        nav.mav.send(msg)
        sent += 1

        gmsg = gcs.recv_match(
            type=["EKF_STATUS_REPORT", "LOCAL_POSITION_NED"],
            blocking=True,
            timeout=0.2,
        )
        if gmsg is None:
            continue
        if gmsg.get_type() == "EKF_STATUS_REPORT":
            ekf_flags = gmsg.flags
        else:
            last_ned = (gmsg.x, gmsg.y, gmsg.z)

        if sent % 10 == 0:
            print(
                f"  sent north={north:5.2f} m"
                f"   EKF flags=0x{ekf_flags:04x}"
                f"   local NED={last_ned}"
            )
        if sent > steps:
            break

    print(f"[nav] sent {sent} ODOMETRY messages")
    print(f"[gcs] final EKF flags = 0x{ekf_flags:04x}")
    print(f"[gcs] final local NED = {last_ned}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
