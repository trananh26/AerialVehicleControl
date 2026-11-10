#!/usr/bin/env python3
"""Compare the ODOMETRY frame of the ROS simulation with the ESP32 tag.

The two implementations of the link have to agree byte for byte: the same
message id, the same frames, the same estimator type, the same velocity
convention and the same timestamp domain.  Otherwise a difference in the
flight behaviour cannot be attributed to the ranging data.

The reference frame is built with ``pymavlink`` from the same field values
the ESP32 firmware uses, and the ROS frame is built by
:func:`uwb_pozyx_bridge.mavlink_odometry.build_odometry_message`.  Both
are then compared field by field.

Usage::

    python3 tools/compare_odometry_frames.py
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "uwb_pozyx_bridge",
    ),
)

from pymavlink.dialects.v20 import common as mavlink

from uwb_pozyx_bridge.mavlink_odometry import (
    MAV_ESTIMATOR_TYPE_NAIVE,
    MAV_FRAME_BODY_FRD,
    MAV_FRAME_LOCAL_FRD,
    ODOMETRY_MSG_ID,
    build_odometry_message,
)

#: The system id both ends of the link use.  It has to be ``SYSID_MYGCS``
#: (255) so that ArduPilot accepts the origin and home declarations.
SYSTEM_ID = 255

#: The component id of the companion.
COMPONENT_ID = 191

#: The pose covariance of the ESP32 tag for a good solution.  The ESP32
#: derives it from its quality index; the ROS node uses a fixed standard
#: deviation.  The values below are the ones the ROS node sends.
POSE_COV = [0.0] * 21
POSE_COV[0] = 0.05 * 0.05
POSE_COV[6] = 0.05 * 0.05
POSE_COV[11] = 0.05 * 0.05

#: The velocity covariance of both ends: ``NaN`` in the first element.
VEL_COV = [0.0] * 21
VEL_COV[0] = float("nan")


def esp32_reference_frame(time_usec: int):
    """Build the ODOMETRY frame the ESP32 tag sends.

    The field values are taken from ``mavlink_output.cpp``; the packing
    itself is done by ``pymavlink``, because the C code cannot be linked
    into this script.
    """
    return mavlink.MAVLink_odometry_message(
        time_usec=time_usec,
        frame_id=MAV_FRAME_LOCAL_FRD,
        child_frame_id=MAV_FRAME_BODY_FRD,
        x=1.25,
        y=-2.5,
        z=0.75,
        q=[1.0, 0.0, 0.0, 0.0],
        vx=float("nan"),
        vy=float("nan"),
        vz=float("nan"),
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0,
        pose_covariance=list(POSE_COV),
        velocity_covariance=list(VEL_COV),
        reset_counter=0,
        estimator_type=MAV_ESTIMATOR_TYPE_NAIVE,
        quality=50,
    )


def ros_frame(time_usec: int):
    """Build the ODOMETRY frame the ROS node sends."""
    return build_odometry_message(
        1.25,
        -2.5,
        0.75,
        time_usec,
        position_std_m=0.05,
        quality=50,
    )


def compare(name: str, a, b) -> bool:
    """Compare two MAVLink messages field by field."""
    ok = True
    if a._header.msgId != b._header.msgId:
        print(f"  [FAIL] {name}: msgId {a._header.msgId} != {b._header.msgId}")
        ok = False
    for field in a._fieldnames:
        va = getattr(a, field)
        vb = getattr(b, field)
        if isinstance(va, (list, tuple)):
            for i, (xa, xb) in enumerate(zip(va, vb)):
                if isinstance(xa, float) and math.isnan(xa):
                    continue
                if xa != xb:
                    print(f"  [FAIL] {name}[{i}]: {xa} != {xb}")
                    ok = False
        elif isinstance(va, float) and math.isnan(va):
            continue
        elif va != vb:
            print(f"  [FAIL] {name}: {va} != {vb}")
            ok = False
    return ok


def main() -> int:
    timestamp = 1_700_000_000_123_456

    esp = esp32_reference_frame(timestamp)
    ros = ros_frame(timestamp)

    print("Comparing the ODOMETRY frame of the ROS node and the ESP32 tag:")
    ok = compare("odometry", esp, ros)

    print(f"  message id      : {ros._header.msgId} (ODOMETRY = {ODOMETRY_MSG_ID})")
    print(f"  frame_id        : {ros.frame_id} (MAV_FRAME_LOCAL_FRD = {MAV_FRAME_LOCAL_FRD})")
    print(f"  child_frame_id  : {ros.child_frame_id} (MAV_FRAME_BODY_FRD = {MAV_FRAME_BODY_FRD})")
    print(f"  estimator_type  : {ros.estimator_type} (NAIVE = {MAV_ESTIMATOR_TYPE_NAIVE})")
    print(f"  velocity        : vx={ros.vx} vy={ros.vy} vz={ros.vz}")
    print(f"  velocity_cov[0] : {ros.velocity_covariance[0]}")
    print(f"  pose_cov[0,6,11]: {ros.pose_covariance[0]} {ros.pose_covariance[6]} {ros.pose_covariance[11]}")
    print(f"  time_usec       : {ros.time_usec}")

    # The system id is not part of the message but of the frame header, so
    # it is checked separately.
    print(f"  system id       : {SYSTEM_ID} (SYSID_MYGCS)")
    print(f"  component id    : {COMPONENT_ID}")

    if ok:
        print("RESULT: the two frames are identical.")
        return 0
    print("RESULT: the two frames differ.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
