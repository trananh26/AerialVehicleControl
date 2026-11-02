"""Unit tests for the Pozyx bridge node helpers.

These tests do not import ROS, so they can run without a sourced workspace.
"""

import pytest

from uwb_pozyx_bridge.pozyx_protocol import (
    MSGID_BEACON_CONFIG,
    MSGID_BEACON_DIST,
    MSGID_POSITION,
    encode_beacon_config,
    encode_vehicle_position,
    enu_to_wire_mm,
)
from uwb_pozyx_bridge.uwb_decoding import (
    anchor_offsets_from_world,
    decode_ranging_pose,
)


# ---------------------------------------------------------------- decoding


def test_decode_ranging_pose():
    measurement = decode_ranging_pose(
        (7.25, -55.0, 2.0), 1.0
    )
    assert measurement is not None
    assert measurement.anchor_id == 2
    assert measurement.range_m == pytest.approx(7.25)
    assert measurement.rss_dbm == pytest.approx(-55.0)
    assert measurement.los == 1


def test_decode_ranging_pose_rounds_anchor_id():
    measurement = decode_ranging_pose((1.0, -50.0, 1.9999), 0.0)
    assert measurement is not None
    assert measurement.anchor_id == 2


def test_decode_ranging_pose_rejects_negative_anchor():
    assert decode_ranging_pose((1.0, -50.0, -1.0), 0.0) is None


def test_decode_los_codes():
    for los in (0, 1, 2):
        measurement = decode_ranging_pose((1.0, -50.0, 0.0), float(los))
        assert measurement is not None
        assert measurement.los == los


# ----------------------------------------------------------- anchor offsets

def test_anchor_offsets_from_world_centre_spawn():
    """The drone spawns at the centre of the 10 m x 10 m anchor square."""
    world = [
        (0.0, 0.0, 2.0),
        (10.0, 0.0, 2.0),
        (10.0, 10.0, 2.0),
        (0.0, 10.0, 2.0),
    ]
    offsets = anchor_offsets_from_world(world, (5.0, 5.0, 0.1))

    assert offsets[0] == pytest.approx((-5.0, -5.0, 1.9))
    assert offsets[1] == pytest.approx((5.0, -5.0, 1.9))
    assert offsets[2] == pytest.approx((5.0, 5.0, 1.9))
    assert offsets[3] == pytest.approx((-5.0, 5.0, 1.9))


def test_anchor_offsets_match_default_parameter():
    """The node's default parameter must match the world layout."""
    from uwb_pozyx_bridge.uwb_decoding import DEFAULT_ANCHOR_OFFSETS_MM

    world = [
        (0.0, 0.0, 2.0),
        (10.0, 0.0, 2.0),
        (10.0, 10.0, 2.0),
        (0.0, 10.0, 2.0),
    ]
    offsets = anchor_offsets_from_world(world, (5.0, 5.0, 0.1))
    expected = tuple(tuple(v * 1000.0 for v in o) for o in offsets)
    assert len(expected) == len(DEFAULT_ANCHOR_OFFSETS_MM)
    for got, want in zip(expected, DEFAULT_ANCHOR_OFFSETS_MM):
        assert got == pytest.approx(want)


def test_anchor_offsets_are_symmetric_about_origin():
    """The beacon origin must be the centroid of the anchor square."""
    world = [
        (0.0, 0.0, 2.0),
        (10.0, 0.0, 2.0),
        (10.0, 10.0, 2.0),
        (0.0, 10.0, 2.0),
    ]
    offsets = anchor_offsets_from_world(world, (5.0, 5.0, 0.1))
    cx = sum(o[0] for o in offsets) / len(offsets)
    cy = sum(o[1] for o in offsets) / len(offsets)
    assert cx == pytest.approx(0.0, abs=1e-9)
    assert cy == pytest.approx(0.0, abs=1e-9)


# ------------------------------------------------------------ frame layout


def _split_frames(data: bytes) -> list[tuple[int, bytes]]:
    """Split a byte stream into (msg_id, payload) pairs."""
    frames = []
    i = 0
    while i < len(data):
        assert data[i] == 0x01, "bad header"
        msg_id = data[i + 1]
        length = data[i + 2]
        payload = data[i + 3 : i + 2 + length]
        frames.append((msg_id, payload))
        i += 3 + length
    return frames


def test_frame_stream_contains_all_three_message_types():
    """A full cycle must emit config, distance and position frames."""
    from uwb_pozyx_bridge.pozyx_protocol import (
        encode_beacon_config,
        encode_beacon_distance,
        encode_vehicle_position,
    )

    data = b""
    data += encode_beacon_config(0, 4, -5000, -5000, 1900)
    data += encode_beacon_distance(0, 7000)
    data += encode_beacon_distance(1, 7000)
    data += encode_vehicle_position(0, 0, 0, 0)

    frames = _split_frames(data)
    ids = [msg_id for msg_id, _ in frames]
    assert ids == [
        MSGID_BEACON_CONFIG,
        MSGID_BEACON_DIST,
        MSGID_BEACON_DIST,
        MSGID_POSITION,
    ]


def test_config_frame_carries_millimetre_offsets():
    from uwb_pozyx_bridge.pozyx_protocol import encode_beacon_config

    data = encode_beacon_config(0, 4, -5000, -5000, 1900)
    _, payload = _split_frames(data)[0]
    import struct

    beacon_id, beacon_count, x, y, z = struct.unpack("<BBiii", payload)
    assert beacon_id == 0
    assert beacon_count == 4
    assert x == -5000
    assert y == -5000
    assert z == 1900


def test_position_accuracy_is_sent_in_metres():
    """The EKF compares the accuracy against a 1.0 m threshold."""
    import struct

    from uwb_pozyx_bridge.pozyx_protocol import encode_vehicle_position

    data = encode_vehicle_position(0, 0, 0, 0)
    _, payload = _split_frames(data)[0]
    _, _, _, accuracy = struct.unpack("<iiih", payload)
    # 0.05 m scaled into the int16 millimetre-ish field would be 0; the node
    # sends 0 which is unambiguously "better than 1 m".
    assert accuracy < 1


def test_anchor_offsets_convert_to_metres_for_the_solver():
    """The solver works in metres, the wire protocol in millimetres."""
    from uwb_pozyx_bridge.trilateration import Anchor

    anchor = Anchor(id=0, position=(-5000.0 / 1000.0, -5000.0 / 1000.0, 1900.0 / 1000.0))
    assert anchor.position[0] == pytest.approx(-5.0)
    assert anchor.position[1] == pytest.approx(-5.0)
    assert anchor.position[2] == pytest.approx(1.9)


# --------------------------------------------------------- frame convention


def test_enu_to_wire_swaps_the_horizontal_axes():
    """Gazebo is East-first, the wire is North-first."""
    north_mm, east_mm, up_mm = enu_to_wire_mm(2.0, -1.0, 0.5)

    assert north_mm == -1000
    assert east_mm == 2000
    assert up_mm == 500


def test_enu_to_wire_is_not_the_identity():
    """A point off the diagonal must not survive the conversion unchanged."""
    north_mm, east_mm, _ = enu_to_wire_mm(2.0, -1.0, 0.0)

    assert north_mm != 2000
    assert east_mm != -1000


def test_wire_conversion_is_a_reflection():
    """Swapping two axes has determinant -1; no rotation can undo it.

    This is why a yaw error cannot compensate for the swap: the whole
    beacon system is mirrored across the diagonal.
    """
    # Two points that are not symmetric about the diagonal.
    a = enu_to_wire_mm(2.0, -1.0, 0.0)
    b = enu_to_wire_mm(-0.5, 3.0, 0.0)

    # After the swap the two points are on the other side of the diagonal.
    assert (a[0] > a[1]) != (b[0] > b[1])


def test_anchor_config_frame_uses_north_east_order():
    """The anchor config frame must carry North in the first field."""
    import struct

    # Anchor 1 is at Gazebo (10, 0, 2.0), i.e. 5 m East of the origin.
    east_m, north_m, up_m = 5.0, 0.0, 1.9
    n_mm, e_mm, u_mm = enu_to_wire_mm(east_m, north_m, up_m)

    data = encode_beacon_config(1, 4, n_mm, e_mm, u_mm)
    _, payload = _split_frames(data)[0]
    beacon_id, beacon_count, x, y, z = struct.unpack("<BBiii", payload)

    assert beacon_id == 1
    assert beacon_count == 4
    # North is 0, East is +5000: the first field must be the North one.
    assert x == 0
    assert y == 5000
    assert z == 1900


def test_vehicle_position_frame_uses_north_east_order():
    """The vehicle position frame must carry North in the first field."""
    import struct

    n_mm, e_mm, u_mm = enu_to_wire_mm(-1.0, 2.0, 0.0)
    data = encode_vehicle_position(n_mm, e_mm, u_mm, 0)
    _, payload = _split_frames(data)[0]
    x, y, z, accuracy = struct.unpack("<iiih", payload)

    # Gazebo (East=-1, North=+2) -> wire (North=+2000, East=-1000).
    assert x == 2000
    assert y == -1000
    assert z == 0
    assert accuracy == 0


def test_round_trip_through_the_ardupilot_parser():
    """Simulate what AP_Beacon_Pozyx does with the frames we send.

    ArduPilot builds the NED position as
    ``Vector3f(x * 0.001f, y * 0.001f, -z * 0.001f)``, so the wire's first
    horizontal field becomes NED North.  The result has to match the Gazebo
    ENU position with the axes swapped, because ArduPilot's NED frame for
    this world is Gazebo's North-first.
    """
    import struct

    gazebo_east, gazebo_north, gazebo_up = 1.5, -2.5, 0.75
    n_mm, e_mm, u_mm = enu_to_wire_mm(gazebo_east, gazebo_north, gazebo_up)

    data = encode_vehicle_position(n_mm, e_mm, u_mm, 0)
    _, payload = _split_frames(data)[0]
    x, y, z, _ = struct.unpack("<iiih", payload)

    # This is AP_Beacon_Pozyx::parse_buffer() verbatim.
    ned_north = x * 0.001
    ned_east = y * 0.001
    ned_down = -z * 0.001

    # ArduPilot's NED North is Gazebo's +Y and NED East is Gazebo's +X.
    assert ned_north == pytest.approx(gazebo_north)
    assert ned_east == pytest.approx(gazebo_east)
    assert ned_down == pytest.approx(-gazebo_up)
