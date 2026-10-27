"""Pozyx wire protocol encoder.

Implements the binary framing used by ArduPilot's ``AP_Beacon_Pozyx`` backend,
which is the protocol spoken by the ``Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino``
Arduino sketch.

Reference
---------
https://github.com/ArduPilot/ardupilot/blob/master/Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino

Frame layout (all multi-byte fields little-endian)::

    [0x01][msg_id][len][payload ...][checksum]

``len`` is ``payload_len + 1`` because it also covers the trailing checksum
byte.  The checksum is the XOR of ``msg_id``, ``len`` and every payload byte,
so that the XOR over the whole frame (excluding the header) is zero.

Coordinate convention
---------------------
ArduPilot's parser converts the incoming values from ENU to NED itself::

    Vector3f beacon_pos(beacon_x * 0.001f, beacon_y * 0.001f, -beacon_z * 0.001f);

so this module encodes **ENU millimetres** exactly as the Arduino sketch does
and performs no frame conversion.
"""

from __future__ import annotations

import struct
from typing import Iterable, Sequence

__all__ = [
    "MSG_HEADER",
    "MSGID_BEACON_CONFIG",
    "MSGID_BEACON_DIST",
    "MSGID_POSITION",
    "encode_beacon_config",
    "encode_beacon_distance",
    "encode_vehicle_position",
    "encode_message",
]

# --- constants mirrored from IndoorLoiter.ino ---------------------------

MSG_HEADER = 0x01
MSGID_BEACON_CONFIG = 0x02
MSGID_BEACON_DIST = 0x03
MSGID_POSITION = 0x04

# Maximum payload the ArduPilot parser will accept.  Mirrors
# AP_BEACON_POZYX_MSG_LEN_MAX in AP_Beacon_Pozyx.h.
_MAX_PAYLOAD_LEN = 20

#: Beacon distances beyond this are rejected by ArduPilot
#: (``AP_BEACON_DISTANCE_MAX`` in AP_Beacon_Pozyx.h).
BEACON_DISTANCE_MAX_M = 200.0


def _checksum(msg_id: int, msg_len: int, payload: bytes) -> int:
    """XOR checksum over the message id, length and payload."""
    checksum = msg_id ^ msg_len
    for byte in payload:
        checksum ^= byte
    return checksum & 0xFF


def encode_message(msg_id: int, payload: bytes) -> bytes:
    """Wrap ``payload`` in a Pozyx frame.

    Args:
        msg_id: One of the ``MSGID_*`` constants.
        payload: Message body without the trailing checksum.

    Returns:
        The complete frame, ready to be written to the serial port.

    Raises:
        ValueError: If the payload is empty or too long for the ArduPilot
            parser to accept.
    """
    if len(payload) == 0:
        raise ValueError("payload must not be empty")
    if len(payload) > _MAX_PAYLOAD_LEN:
        raise ValueError(
            f"payload of {len(payload)} bytes exceeds the ArduPilot limit of "
            f"{_MAX_PAYLOAD_LEN}"
        )

    # The length field covers the payload plus the checksum byte.
    msg_len = len(payload) + 1
    checksum = _checksum(msg_id, msg_len, payload)
    return bytes((MSG_HEADER, msg_id, msg_len)) + payload + bytes((checksum,))


def encode_beacon_config(
    beacon_id: int, beacon_count: int, x_mm: int, y_mm: int, z_mm: int
) -> bytes:
    """Encode a ``MSGID_BEACON_CONFIG`` message.

    Tells ArduPilot where a single anchor is located, in ENU millimetres
    relative to the beacon system's origin.

    Args:
        beacon_id: Index of the anchor, 0-based.
        beacon_count: Total number of anchors in the system.
        x_mm: Anchor east offset in millimetres.
        y_mm: Anchor north offset in millimetres.
        z_mm: Anchor up offset in millimetres.
    """
    payload = struct.pack("<BBiii", beacon_id, beacon_count, x_mm, y_mm, z_mm)
    return encode_message(MSGID_BEACON_CONFIG, payload)


def encode_beacon_distance(beacon_id: int, distance_mm: int) -> bytes:
    """Encode a ``MSGID_BEACON_DIST`` message.

    Reports the measured range between the vehicle tag and one anchor.

    Args:
        beacon_id: Index of the anchor, 0-based.
        distance_mm: Measured range in millimetres.
    """
    payload = struct.pack("<BI", beacon_id, distance_mm)
    return encode_message(MSGID_BEACON_DIST, payload)


def encode_vehicle_position(
    x_mm: int, y_mm: int, z_mm: int, position_error_mm: int
) -> bytes:
    """Encode a ``MSGID_POSITION`` message.

    Reports the vehicle tag's position in ENU millimetres, relative to the
    beacon system's origin, together with the positioning error estimate.

    .. warning::
       **Unit mismatch in ArduPilot (upstream bug).**

       ``IndoorLoiter.ino`` fills ``position_error`` with ``pos_error.xy``,
       which the Pozyx API reports in **millimetres**.  ArduPilot's
       ``AP_Beacon_Pozyx::parse_buffer()`` forwards that value unscaled to
       ``set_vehicle_position(veh_pos, position_error)``, whose
       ``accuracy_estimate`` argument is documented as being **in meters**
       (see ``AP_Beacon_Backend.cpp``).

       The EKF then compares it against a 1 metre threshold::

           // AP_NavEKF3_Measurements.cpp
           if ((imuSampleTime_ms - rngBcn.last3DmeasTime_ms < 250)
               && (rngBcn.vehiclePosErr < 1.0f)
               && rngBcn.alignmentCompleted) {

       So a realistic error of 50 mm is interpreted as 50 m, the beacon 3D fix
       is rejected, ``goodToAlign`` never becomes true and the EKF never
       sets its origin from the beacon system.

       To work around this without patching ArduPilot, pass the accuracy in
       **metres** here (e.g. ``0.05`` for 50 mm).  The value is transmitted in
       the same ``int16`` field; only its interpretation differs.

    Args:
        x_mm: Vehicle east offset in millimetres.
        y_mm: Vehicle north offset in millimetres.
        z_mm: Vehicle up offset in millimetres.
        position_error_mm: Positioning error estimate.  See the warning above:
            to be interpreted correctly by the EKF this must be in metres,
            not millimetres.
    """
    payload = struct.pack("<iii h", x_mm, y_mm, z_mm, position_error_mm)
    return encode_message(MSGID_POSITION, payload)


def encode_beacon_configs(
    anchors: Sequence[tuple[int, float, float, float]],
    beacon_count: int | None = None,
) -> bytes:
    """Encode every anchor configuration back to back.

    Args:
        anchors: Iterable of ``(beacon_id, x_mm, y_mm, z_mm)`` tuples.
        beacon_count: Total number of anchors.  Defaults to the number of
            anchors supplied.

    Returns:
        All configuration frames concatenated.
    """
    anchors = list(anchors)
    count = len(anchors) if beacon_count is None else beacon_count
    return b"".join(
        encode_beacon_config(beacon_id, count, x, y, z)
        for beacon_id, x, y, z in anchors
    )


def encode_beacon_distances(
    distances: Iterable[tuple[int, float]],
) -> bytes:
    """Encode every beacon distance back to back.

    Args:
        distances: Iterable of ``(beacon_id, distance_mm)`` tuples.

    Returns:
        All distance frames concatenated.
    """
    return b"".join(
        encode_beacon_distance(beacon_id, int(round(distance_mm)))
        for beacon_id, distance_mm in distances
    )
