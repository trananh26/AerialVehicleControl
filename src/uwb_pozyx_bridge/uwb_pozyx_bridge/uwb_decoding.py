"""Decoding of the Gazebo UWB plugin's ranging messages.

The ``uwb_gz_plugin`` publishes one ``gz::msgs::Pose`` per anchor with a
custom encoding (see ``src/UwbSystem.cc`` of that package):

============================  ==========================================
Field                         Meaning
============================  ==========================================
``pose.position.x``           range to the anchor, in metres
``pose.position.y``           RSS, in dBm
``pose.position.z``           anchor id
``pose.orientation.x``        line-of-sight code (0 = LOS, 1 = NLOS soft,
                              2 = NLOS hard)
============================  ==========================================

This module is deliberately free of ROS imports so that the decoding can be
unit tested with plain tuples.

The anchor positions themselves live in :mod:`uwb_pozyx_bridge.uwb_layout`,
which is the single source of truth for the layout.  This module only
provides the decoder and a couple of helpers that convert between the
world frame and the anchor frame.
"""

from __future__ import annotations

import math
from typing import Optional

from .trilateration import RangeMeasurement
from .uwb_layout import AnchorLayout, default_layout

__all__ = [
    "decode_ranging_pose",
    "anchor_offsets_from_world",
    "DEFAULT_ANCHOR_OFFSETS_MM",
]


def _default_offsets_mm() -> tuple[tuple[float, float, float], ...]:
    """The default anchor offsets, in millimetres, in the anchor frame."""
    layout = default_layout()
    return tuple(
        tuple(component * 1000.0 for component in position)
        for position in layout.anchor_frame_positions()
    )


#: The anchor positions the trilateration solver uses, in millimetres,
#: expressed in the **anchor frame** (x towards ``a1``, y towards ``a3``,
#: z down).  The anchor frame is the ideal square; the physical layout is
#: the same square rotated by the layout angle ``alpha``.
#:
#: These are the positions relative to ``a0``, which is the UWB origin.
DEFAULT_ANCHOR_OFFSETS_MM = _default_offsets_mm()


def decode_ranging_pose(
    position: tuple[float, float, float],
    orientation_x: float = 0.0,
) -> Optional[RangeMeasurement]:
    """Decode one ranging message into a :class:`RangeMeasurement`.

    Args:
        position: The ``(x, y, z)`` triple of the message, holding range,
            RSS and anchor id.
        orientation_x: The line-of-sight code.

    Returns:
        The decoded measurement, or ``None`` if the anchor id is negative.
    """
    range_m = float(position[0])
    rss_dbm = float(position[1])
    anchor_id = int(round(float(position[2])))
    los = int(round(float(orientation_x)))

    if anchor_id < 0:
        return None

    return RangeMeasurement(
        anchor_id=anchor_id,
        range_m=range_m,
        rss_dbm=rss_dbm,
        los=los,
    )


def anchor_offsets_from_world(
    world_positions: list[tuple[float, float, float]],
    beacon_origin: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    """Convert anchor positions in the world frame to UWB-relative offsets.

    The offsets are the anchor positions **relative to the UWB system
    origin**, which is the position where the vehicle spawns.

    The offsets are returned in the Gazebo **ENU** frame (x = East,
    y = North); the conversion to the NED ordering that ArduPilot expects is
    done by :func:`uwb_pozyx_bridge.mavlink_odometry.enu_to_ned_m`.

    Args:
        world_positions: Anchor positions in the Gazebo world frame, metres.
        beacon_origin: The beacon system origin in the world frame, metres.

    Returns:
        The offsets in metres, in the same order as ``world_positions``.

    Example:
        >>> anchor_offsets_from_world(
        ...     [(0.0, 0.0, 2.0)], (5.0, 5.0, 0.1)
        ... )
        [(-5.0, -5.0, 1.9)]
    """
    ox, oy, oz = beacon_origin
    return [
        (x - ox, y - oy, z - oz) for x, y, z in world_positions
    ]


def anchor_offsets_from_layout(
    layout: AnchorLayout,
) -> list[tuple[float, float, float]]:
    """The anchor positions of a layout, in metres, in the anchor frame.

    Args:
        layout: The layout to describe.

    Returns:
        Four ``(x_m, y_m, z_m)`` triples.
    """
    return list(layout.anchor_frame_positions())


def layout_alpha_from_offsets(
    offsets: list[tuple[float, float, float]],
) -> float:
    """Recover the layout angle from anchor offsets in the anchor frame.

    The angle is the bearing of the direction ``a0 -> a1``, measured from
    North.  This is the inverse of the rotation the ESP32 tag applies in
    ``convertLocalToNed()``.

    Args:
        offsets: The anchor positions in the anchor frame, in metres.

    Returns:
        The angle in degrees.
    """
    if len(offsets) < 4:
        raise ValueError("four anchor positions are needed")
    # a0 -> a1 is the +x direction of the anchor frame, a0 -> a3 is +y.
    dx = offsets[1][0] - offsets[0][0]
    dy = offsets[1][1] - offsets[0][1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(dy, dx))
