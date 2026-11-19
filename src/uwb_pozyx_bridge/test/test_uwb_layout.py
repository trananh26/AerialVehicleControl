"""Unit tests for the UWB anchor layout.

The layout is the single source of truth shared by the ROS simulation and
the ESP32 tag: the ideal rectangle in the anchor frame, the rotation by
``alpha`` relative to NED, and the conversion into the Gazebo world frame.

The anchor ids walk the **N pattern** of the Pozyx reference implementation
of ArduPilot (``IndoorLoiter.ino``): the lowest id is the origin, and the
ids increase as you travel an "N" through the rectangle.
"""

import math

import pytest

from uwb_pozyx_bridge.uwb_layout import (
    ANCHOR_COUNT,
    LAYOUT_SIZE_X_M,
    LAYOUT_SIZE_Y_M,
    N_PATTERN_INDEX_ORDER,
    AnchorLayout,
    default_layout,
    rotate_ned,
)


def test_anchor_frame_is_the_ideal_rectangle_in_the_n_pattern():
    """The anchor frame is a rectangle with a0 at the origin.

    The ids walk the N pattern of the Pozyx reference implementation:
    ``x`` points towards ``a1`` and ``y`` towards ``a2``; ``z`` points
    down, so the anchor plane is ``z = 0``.

    Note that ``a2`` is at ``(0, Y)`` and ``a3`` at ``(X, Y)`` — that is
    the N pattern, not the perimeter order.
    """
    layout = default_layout()
    positions = layout.anchor_frame_positions()

    assert len(positions) == ANCHOR_COUNT
    assert positions[0] == (0.0, 0.0, 0.0)
    assert positions[1] == (LAYOUT_SIZE_X_M, 0.0, 0.0)
    assert positions[2] == (0.0, LAYOUT_SIZE_Y_M, 0.0)
    assert positions[3] == (LAYOUT_SIZE_X_M, LAYOUT_SIZE_Y_M, 0.0)


def test_the_layout_reads_as_a_map_with_north_up():
    """The anchor frame is NED-like: x = North, y = East.

    Drawn as a map with North up and East right, the four anchors are::

        a1 ------- a3
         |         |
         |         |
        a0 ------- a2

    so a1 is at the NORTH end of a0, a2 at its EAST end and a3 is the
    opposite corner.
    """
    layout = AnchorLayout(size_x_m=10.0, size_y_m=6.0)
    positions = layout.anchor_frame_positions()

    a0, a1, a2, a3 = positions

    # a0 is the origin (south-west corner of the map).
    assert a0 == (0.0, 0.0, 0.0)

    # a1 is due North of a0: same East, more North.
    assert a1[0] == 10.0 and a1[1] == 0.0

    # a2 is due East of a0: same North, more East.
    assert a2[0] == 0.0 and a2[1] == 6.0

    # a3 is the opposite corner: North of a2, East of a1.
    assert a3 == (10.0, 6.0, 0.0)


def test_the_anchor_ids_walk_the_n_pattern():
    """Walking the ids in order traces the letter N."""
    layout = AnchorLayout(size_x_m=10.0, size_y_m=6.0)
    positions = layout.anchor_frame_positions()

    # The N is walked in the order of the anchor ids.
    assert N_PATTERN_INDEX_ORDER == (0, 1, 2, 3)

    # Step 1: a0 -> a1 goes up the a0->a1 side (North, the "x" direction).
    assert positions[1][0] > positions[0][0]
    assert positions[1][1] == positions[0][1]

    # Step 2: a1 -> a2 crosses diagonally down-right to the a0->a2 side
    # (East, the "y" direction).
    assert positions[2][1] > positions[1][1]
    assert positions[2][0] == positions[0][0]

    # Step 3: a2 -> a3 goes back up the far side.
    assert positions[3][0] > positions[2][0]
    assert positions[3][1] == positions[2][1]


def test_the_lowest_id_is_the_origin():
    """The lowest anchor id sits at the origin of the anchor frame."""
    layout = AnchorLayout(size_x_m=14.0, size_y_m=7.0, alpha_deg=33.0)
    positions = layout.anchor_frame_positions()

    assert positions[0] == (0.0, 0.0, 0.0)


def test_the_layout_can_be_a_rectangle():
    """The two sides of the layout are independent."""
    layout = AnchorLayout(size_x_m=14.0, size_y_m=7.0)
    positions = layout.anchor_frame_positions()

    assert positions[1] == (14.0, 0.0, 0.0)
    assert positions[2] == (0.0, 7.0, 0.0)
    assert positions[3] == (14.0, 7.0, 0.0)
    assert not layout.is_square


def test_a_square_layout_reports_itself_as_such():
    """The default layout is a square, so the helper agrees."""
    assert default_layout().is_square


def test_alpha_zero_leaves_the_layout_aligned_with_ned():
    """With ``alpha = 0`` the NED positions equal the anchor-frame ones."""
    layout = AnchorLayout(size_x_m=10.0, size_y_m=10.0, alpha_deg=0.0)
    ned = layout.anchor_ned_positions()

    assert ned[0] == (0.0, 0.0, 0.0)
    assert ned[1] == (10.0, 0.0, 0.0)
    assert ned[2] == (0.0, 10.0, 0.0)
    assert ned[3] == (10.0, 10.0, 0.0)


def test_alpha_rotates_the_layout_counter_clockwise_towards_the_east():
    """``alpha`` is the bearing of ``a0 -> a1``, measured from North."""
    layout = AnchorLayout(size_x_m=10.0, size_y_m=10.0, alpha_deg=90.0)
    ned = layout.anchor_ned_positions()

    # At 90 degrees the direction a0 -> a1 points East.
    assert ned[1][0] == pytest.approx(0.0, abs=1e-9)
    assert ned[1][1] == pytest.approx(10.0, abs=1e-9)
    # ...and a0 -> a2 points South.
    assert ned[2][0] == pytest.approx(-10.0, abs=1e-9)
    assert ned[2][1] == pytest.approx(0.0, abs=1e-9)


def test_rotation_preserves_the_layout_shape():
    """A rotation does not change the distances between the anchors."""
    layout = AnchorLayout(size_x_m=14.0, size_y_m=7.0, alpha_deg=37.0)
    ned = layout.anchor_ned_positions()

    for i in range(ANCHOR_COUNT):
        for j in range(i + 1, ANCHOR_COUNT):
            d = math.dist(ned[i][:2], ned[j][:2])
            expected = math.dist(
                layout.anchor_frame_positions()[i][:2],
                layout.anchor_frame_positions()[j][:2],
            )
            assert d == pytest.approx(expected, abs=1e-9)


def test_rotate_ned_is_a_pure_rotation():
    """The rotation has determinant +1, so it does not mirror the frame."""
    north, east = rotate_ned(1.0, 0.0, math.radians(30.0))

    assert north == pytest.approx(math.cos(math.radians(30.0)))
    assert east == pytest.approx(math.sin(math.radians(30.0)))

    # The a0 -> a2 direction stays perpendicular to a0 -> a1.
    north2, east2 = rotate_ned(0.0, 1.0, math.radians(30.0))
    assert north * north2 + east * east2 == pytest.approx(0.0, abs=1e-9)


def test_world_positions_are_the_ned_positions_in_enu_ordering():
    """The Gazebo world is East-first, so the axes are swapped there."""
    layout = AnchorLayout(
        size_x_m=10.0, size_y_m=10.0, alpha_deg=0.0, plane_height_m=2.0
    )
    world = layout.anchor_world_enu()

    assert world[0] == (0.0, 0.0, 2.0)
    assert world[1] == (0.0, 10.0, 2.0)
    assert world[2] == (10.0, 0.0, 2.0)
    assert world[3] == (10.0, 10.0, 2.0)


def test_world_positions_of_a_rectangle():
    """A rectangular layout is mirrored in the Gazebo world frame."""
    layout = AnchorLayout(
        size_x_m=14.0, size_y_m=7.0, alpha_deg=0.0, plane_height_m=2.0
    )
    world = layout.anchor_world_enu()

    assert world[0] == (0.0, 0.0, 2.0)
    assert world[1] == (0.0, 14.0, 2.0)
    assert world[2] == (7.0, 0.0, 2.0)
    assert world[3] == (7.0, 14.0, 2.0)


def test_to_ned_applies_the_rotation_and_the_origin_offset():
    """The MAVLink position is relative to the EKF origin, not to ``a0``."""
    layout = AnchorLayout(
        size_x_m=10.0,
        size_y_m=10.0,
        alpha_deg=0.0,
        origin_north_m=100.0,
        origin_east_m=-50.0,
    )
    north, east = layout.to_ned(2.0, 3.0)

    assert north == pytest.approx(102.0)
    assert east == pytest.approx(-47.0)


def test_to_ned_3d_converts_the_depth_below_the_anchor_plane():
    """The anchor plane sits ``plane_height_m`` above the EKF origin."""
    layout = AnchorLayout(size_x_m=10.0, size_y_m=10.0, plane_height_m=2.0)
    north, east, down = layout.to_ned_3d(5.0, 5.0, 3.0)

    assert north == pytest.approx(5.0)
    assert east == pytest.approx(5.0)
    # 3 m below the anchor plane is 1 m below the ground, so the depth
    # below the EKF origin is +1 m.
    assert down == pytest.approx(1.0)

    # A tag exactly at the anchor plane is at the origin's height.
    _, _, down_at_plane = layout.to_ned_3d(5.0, 5.0, 2.0)
    assert down_at_plane == pytest.approx(0.0)

    # A tag 1 m below the anchor plane is 1 m above the ground.
    _, _, down_below = layout.to_ned_3d(5.0, 5.0, 1.0)
    assert down_below == pytest.approx(-1.0)


def test_random_spawn_stays_inside_the_rectangle():
    """The spawn point is inside the rectangle, whatever the rotation."""
    import random

    layout = AnchorLayout(size_x_m=10.0, size_y_m=10.0, alpha_deg=25.0)
    rng = random.Random(1234)

    for _ in range(100):
        east, north = layout.random_spawn_enu(rng, margin_m=1.0)
        # Convert back into the anchor frame and check the bounds.
        anchor_north, anchor_east = rotate_ned(
            north, east, -layout.alpha_rad
        )
        assert 1.0 <= anchor_north <= 9.0
        assert 1.0 <= anchor_east <= 9.0


def test_random_spawn_respects_the_rectangular_bounds():
    """The spawn bounds follow the two sides of the rectangle."""
    import random

    layout = AnchorLayout(size_x_m=16.0, size_y_m=8.0, alpha_deg=0.0)
    rng = random.Random(7)

    for _ in range(100):
        east, north = layout.random_spawn_enu(rng, margin_m=1.0)
        anchor_north, anchor_east = rotate_ned(
            north, east, -layout.alpha_rad
        )
        assert 1.0 <= anchor_north <= 15.0
        assert 1.0 <= anchor_east <= 7.0


def test_centre_of_a_rectangle():
    """The centre of the rectangle is the middle of the two sides."""
    layout = AnchorLayout(size_x_m=14.0, size_y_m=7.0, alpha_deg=0.0)
    east, north = layout.centre_enu()

    assert east == pytest.approx(3.5)
    assert north == pytest.approx(7.0)
