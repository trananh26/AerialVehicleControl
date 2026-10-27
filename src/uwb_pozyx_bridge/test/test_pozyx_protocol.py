"""Unit tests for the Pozyx wire protocol encoder.

The expected byte sequences are derived by hand from
``Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino`` and cross-checked against the
parser in ``AP_Beacon_Pozyx.cpp``.
"""

import struct

import pytest

from uwb_pozyx_bridge.pozyx_protocol import (
    MSG_HEADER,
    MSGID_BEACON_CONFIG,
    MSGID_BEACON_DIST,
    MSGID_POSITION,
    encode_beacon_config,
    encode_beacon_distance,
    encode_message,
    encode_vehicle_position,
)


def test_header_is_0x01():
    frame = encode_beacon_distance(0, 1000)
    assert frame[0] == MSG_HEADER == 0x01


def test_beacon_config_layout():
    """beacon_id, beacon_count, x, y, z as little-endian int32."""
    frame = encode_beacon_config(0, 4, 0, 0, -1200)

    assert frame[0] == MSG_HEADER
    assert frame[1] == MSGID_BEACON_CONFIG
    assert frame[2] == 15  # 14 payload bytes + checksum

    payload = frame[3:-1]
    assert len(payload) == 14

    beacon_id, beacon_count, x, y, z = struct.unpack("<BBiii", payload)
    assert beacon_id == 0
    assert beacon_count == 4
    assert x == 0
    assert y == 0
    assert z == -1200


def test_beacon_distance_layout():
    frame = encode_beacon_distance(2, 12345)

    assert frame[0] == MSG_HEADER
    assert frame[1] == MSGID_BEACON_DIST
    assert frame[2] == 6  # 5 payload bytes + checksum

    payload = frame[3:-1]
    assert len(payload) == 5

    beacon_id, distance = struct.unpack("<BI", payload)
    assert beacon_id == 2
    assert distance == 12345


def test_vehicle_position_layout():
    frame = encode_vehicle_position(-1000, 2500, 0, 50)

    assert frame[0] == MSG_HEADER
    assert frame[1] == MSGID_POSITION
    assert frame[2] == 15  # 14 payload bytes + checksum

    payload = frame[3:-1]
    assert len(payload) == 14

    x, y, z, error = struct.unpack("<iiih", payload)
    assert x == -1000
    assert y == 2500
    assert z == 0
    assert error == 50


@pytest.mark.parametrize(
    "frame_builder",
    [
        lambda: encode_beacon_config(0, 4, 0, 0, -1200),
        lambda: encode_beacon_distance(0, 1000),
        lambda: encode_vehicle_position(0, 0, 0, 50),
    ],
)
def test_checksum_is_xor_of_everything(frame_builder):
    """The XOR over msg_id, len and payload must equal the checksum byte."""
    frame = frame_builder()

    msg_id = frame[1]
    msg_len = frame[2]
    payload = frame[3:-1]
    checksum = frame[-1]

    expected = msg_id ^ msg_len
    for byte in payload:
        expected ^= byte

    assert checksum == expected & 0xFF


@pytest.mark.parametrize(
    "frame_builder",
    [
        lambda: encode_beacon_config(0, 4, 0, 0, -1200),
        lambda: encode_beacon_distance(0, 1000),
        lambda: encode_vehicle_position(0, 0, 0, 50),
    ],
)
def test_frame_xor_is_zero(frame_builder):
    """XOR over the whole frame (minus header) is zero, as ArduPilot checks."""
    frame = frame_builder()

    result = 0
    for byte in frame[1:]:
        result ^= byte

    assert result == 0


def test_config_matches_induiloiter_ino_example():
    """Anchor 1 at (10000, 0, -1200) mm, exactly as in the Arduino sketch."""
    frame = encode_beacon_config(1, 4, 10000, 0, -1200)

    # Manual byte-by-byte check of the on-the-wire layout.
    assert frame[0] == 0x01
    assert frame[1] == 0x02
    assert frame[2] == 0x0F
    assert frame[3] == 1  # beacon_id
    assert frame[4] == 4  # beacon_count
    assert frame[5:9] == struct.pack("<i", 10000)
    assert frame[9:13] == struct.pack("<i", 0)
    assert frame[13:17] == struct.pack("<i", -1200)


def test_empty_payload_rejected():
    with pytest.raises(ValueError):
        encode_message(MSGID_BEACON_DIST, b"")


def test_oversized_payload_rejected():
    with pytest.raises(ValueError):
        encode_message(MSGID_BEACON_DIST, b"\x00" * 21)
