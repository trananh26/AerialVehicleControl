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
"""

from __future__ import annotations

from typing import Optional

from .trilateration import RangeMeasurement

__all__ = [
    "decode_ranging_pose",
    "anchor_offsets_from_world",
    "DEFAULT_ANCHOR_OFFSETS_MM",
]

#: The Gazebo world places the four anchors on the corners of a 10 m x 10 m
#: square at z = 2.0 m, and the vehicle spawns at the centre, (5, 5, 0.1).
#: These are the anchor positions **relative to the beacon origin**, in
#: millimetres, which is what the Pozyx protocol expects.
DEFAULT_ANCHOR_OFFSETS_MM = (
    (-5000.0, -5000.0, 1900.0),
    (5000.0, -5000.0, 1900.0),
    (5000.0, 5000.0, 1900.0),
    (-5000.0, 5000.0, 1900.0),
)


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
    """Convert anchor positions in the world frame to beacon-relative offsets.

    ArduPilot's Pozyx backend expects each anchor's position **relative to the
    beacon system origin**, in ENU millimetres.  The beacon origin is the
    position where the vehicle spawns.

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
