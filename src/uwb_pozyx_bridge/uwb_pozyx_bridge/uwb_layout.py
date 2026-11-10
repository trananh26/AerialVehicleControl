"""The UWB anchor layout — one source of truth for both sides of the link.

The layout is a **rectangle** of four anchors.  In the **anchor frame** (the
frame the ranging solver works in) it is the ideal rectangle::

    a0 = (0, 0)      a1 = (X, 0)
    a3 = (0, Y)      a2 = (X, Y)

with ``x`` pointing towards ``a1`` and ``y`` towards ``a3``, and ``z``
pointing **down** (the anchor plane is the reference, ``z = 0``).

The layout does not have to be a square: in a real hall the anchors are
mounted wherever there is room for them, so the ``a0 -> a1`` side and the
``a0 -> a3`` side generally have different lengths.  ``X`` is the length of
the ``a0 -> a1`` side and ``Y`` the length of the ``a0 -> a3`` side.

The anchor frame is the NED frame rotated by the layout angle ``alpha``:
the physical rectangle is usually not aligned with North/East, because the
anchors are mounted wherever there is room for them.  ``alpha`` is the
bearing of the direction ``a0 -> a1``, measured from North, clockwise seen
from above:

* ``alpha = 0``   -> ``a0 -> a1`` points North, ``a0 -> a3`` points East.
* ``alpha > 0``   -> the whole cluster is rotated towards the East.

Both sides of the MAVLink link use exactly this definition, which is why
the same rectangle is described by

* the ESP32 tag: ``ANCHOR_POS_LOCAL[]`` holds the ideal rectangle and
  ``config_yaw_offset_deg`` holds ``alpha`` (see ``convertLocalToNed()``);
  the defaults come from ``CFG_DEFAULT_LAYOUT_SIZE_X_M`` /
  ``CFG_DEFAULT_LAYOUT_SIZE_Y_M`` in ``uwb_config.h``,
* this module: :meth:`AnchorLayout.rotate_to_ned` does the same rotation on
  the solver result.

The ranging distances are invariant under a rotation about the vertical
axis, so the solver may always use the ideal rectangle; only the *result* has
to be rotated, and only the Gazebo world has to be rendered with the
rotated anchor positions.

Conventions
-----------
* The anchor frame is right-handed with ``x = North-ish``, ``y = East-ish``
  and ``z = Down``.
* All positions are in metres.
* The anchor plane is the ``z = 0`` reference of the anchor frame.  A tag
  below the plane has a **positive** ``z``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

__all__ = [
    "ANCHOR_COUNT",
    "LAYOUT_SIZE_X_M",
    "LAYOUT_SIZE_Y_M",
    "LAYOUT_SIZE_M",
    "ANCHOR_PLANE_HEIGHT_M",
    "AnchorLayout",
    "default_layout",
    "rotate_ned",
]

#: Number of anchors in the layout.
ANCHOR_COUNT = 4

#: Length of the ``a0 -> a1`` side of the layout, in metres.
LAYOUT_SIZE_X_M = 10.0

#: Length of the ``a0 -> a3`` side of the layout, in metres.
LAYOUT_SIZE_Y_M = 10.0

#: Length of a square layout, in metres.  Kept for the layouts that are
#: squares; the default layout happens to be one.
LAYOUT_SIZE_M = LAYOUT_SIZE_X_M

#: Height of the anchor plane above the ground, in metres.  This is a
#: property of the Gazebo world, not of the layout itself, but the ROS
#: simulation needs it to place the anchors and to spawn the vehicle.
ANCHOR_PLANE_HEIGHT_M = 2.0


def rotate_ned(x: float, y: float, alpha_rad: float) -> tuple[float, float]:
    """Rotate a horizontal vector from the anchor frame into NED.

    The rotation is a pure rotation (determinant +1), so it does not mirror
    the frame::

        N = x * cos(alpha) - y * sin(alpha)
        E = x * sin(alpha) + y * cos(alpha)

    With ``alpha = 0`` this is the identity, so an aligned layout needs no
    correction at all.

    Args:
        x: North-ish component in the anchor frame, in metres.
        y: East-ish component in the anchor frame, in metres.
        alpha_rad: Layout angle in radians (bearing of ``a0 -> a1``).

    Returns:
        The ``(north_m, east_m)`` pair in the NED frame.
    """
    cos_a = math.cos(alpha_rad)
    sin_a = math.sin(alpha_rad)
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)


@dataclass(frozen=True)
class AnchorLayout:
    """A rectangular anchor layout with a rotation relative to NED.

    Attributes:
        size_x_m: Length of the ``a0 -> a1`` side, in metres.
        size_y_m: Length of the ``a0 -> a3`` side, in metres.
        alpha_deg: Layout angle in degrees: the bearing of ``a0 -> a1``
            measured from North, clockwise seen from above.
        plane_height_m: Height of the anchor plane above the ground, in
            metres.  Only used by the simulation, to place the anchors and
            to spawn the vehicle.
        origin_north_m: North offset of ``a0`` from the EKF origin, in
            metres.  The EKF origin is the point the MAVLink position is
            relative to, which in the simulation is the Gazebo world
            origin.
        origin_east_m: East offset of ``a0`` from the EKF origin, in
            metres.
    """

    size_x_m: float = LAYOUT_SIZE_X_M
    size_y_m: float = LAYOUT_SIZE_Y_M
    alpha_deg: float = 0.0
    plane_height_m: float = ANCHOR_PLANE_HEIGHT_M
    origin_north_m: float = 0.0
    origin_east_m: float = 0.0

    # -- geometry ---------------------------------------------------------

    @property
    def alpha_rad(self) -> float:
        """The layout angle in radians."""
        return math.radians(self.alpha_deg)

    @property
    def size_m(self) -> float:
        """The length of the ``a0 -> a1`` side.

        Kept for the layouts that are squares, where it is also the length
        of the ``a0 -> a3`` side.
        """
        return self.size_x_m

    @property
    def is_square(self) -> bool:
        """Whether the layout is a square."""
        return abs(self.size_x_m - self.size_y_m) < 1e-9

    def anchor_frame_positions(
        self,
    ) -> tuple[tuple[float, float, float], ...]:
        """The ideal rectangle in the anchor frame.

        The order is the anchor id order used by the Gazebo plugin and by
        the ESP32 tag: ``a0``, ``a1``, ``a2``, ``a3``.

        Returns:
            Four ``(x, y, z)`` triples, in metres.  ``z`` is zero because
            the anchor plane is the reference of the anchor frame.
        """
        sx = self.size_x_m
        sy = self.size_y_m
        return (
            (0.0, 0.0, 0.0),
            (sx, 0.0, 0.0),
            (sx, sy, 0.0),
            (0.0, sy, 0.0),
        )

    def anchor_ned_positions(
        self,
    ) -> tuple[tuple[float, float, float], ...]:
        """The anchors in the NED frame, relative to ``a0``.

        The vertical component is the offset from the anchor plane, so it
        is zero for a coplanar layout.  ``a0`` itself is at the origin of
        this frame, which is what the UWB origin is.

        Returns:
            Four ``(north_m, east_m, down_m)`` triples, in metres.
        """
        out = []
        for x, y, z in self.anchor_frame_positions():
            north, east = rotate_ned(x, y, self.alpha_rad)
            out.append((north, east, z))
        return tuple(out)

    def anchor_world_enu(
        self,
    ) -> tuple[tuple[float, float, float], ...]:
        """The anchors in the Gazebo world frame (ENU).

        The Gazebo world is ``x = East``, ``y = North``, ``z = Up``, and
        the anchor plane sits at ``plane_height_m`` above the ground.

        Returns:
            Four ``(east_m, north_m, up_m)`` triples, in metres.
        """
        out = []
        for north, east, _down in self.anchor_ned_positions():
            out.append((east, north, self.plane_height_m))
        return tuple(out)

    def rotate_to_ned(self, x: float, y: float) -> tuple[float, float]:
        """Rotate a horizontal anchor-frame position into NED.

        The result is relative to ``a0``; use :meth:`to_ned` to get the
        position relative to the EKF origin.

        Args:
            x: North-ish component in the anchor frame, in metres.
            y: East-ish component in the anchor frame, in metres.

        Returns:
            The ``(north_m, east_m)`` pair in the NED frame.
        """
        return rotate_ned(x, y, self.alpha_rad)

    def to_ned(self, x: float, y: float) -> tuple[float, float]:
        """Convert an anchor-frame position into NED, relative to the origin.

        The rotation by ``alpha`` is applied first, and the offset of
        ``a0`` from the EKF origin is added afterwards.

        Args:
            x: North-ish component in the anchor frame, in metres.
            y: East-ish component in the anchor frame, in metres.

        Returns:
            The ``(north_m, east_m)`` pair in the NED frame, relative to
            the EKF origin.
        """
        north, east = self.rotate_to_ned(x, y)
        return (
            north + self.origin_north_m,
            east + self.origin_east_m,
        )

    def to_ned_3d(
        self, x: float, y: float, z: float
    ) -> tuple[float, float, float]:
        """Convert an anchor-frame position into NED, relative to the origin.

        The vertical component of the anchor frame is the depth below the
        anchor plane, so the height above the ground is
        ``plane_height_m - z`` and the depth below the EKF origin (the
        ground) is ``z - plane_height_m``.

        Args:
            x: North-ish component in the anchor frame, in metres.
            y: East-ish component in the anchor frame, in metres.
            z: Depth below the anchor plane, in metres.

        Returns:
            The ``(north_m, east_m, down_m)`` triple in the NED frame.
        """
        north, east = self.to_ned(x, y)
        return (north, east, z - self.plane_height_m)

    # -- sampling ---------------------------------------------------------

    def random_spawn_enu(
        self,
        rng,
        margin_m: float = 1.5,
    ) -> tuple[float, float]:
        """Pick a random ground point inside the rectangle.

        The point is drawn uniformly in the anchor frame, so it is uniform
        in the rectangle no matter how the layout is rotated.

        Args:
            rng: A random number generator with a ``uniform(a, b)`` method,
                e.g. :class:`random.Random`.
            margin_m: Minimum distance to the edges of the rectangle, in
                metres.  Keeps the vehicle away from the anchors, where the
                ranging geometry would be poor.

        Returns:
            The ``(east_m, north_m)`` spawn position in the Gazebo world.
        """
        x = rng.uniform(margin_m, self.size_x_m - margin_m)
        y = rng.uniform(margin_m, self.size_y_m - margin_m)
        north, east = rotate_ned(x, y, self.alpha_rad)
        return (east, north)

    def spawn_positions_enu(
        self,
        rng,
        count: int = 1,
        margin_m: float = 1.5,
    ) -> list[tuple[float, float]]:
        """Pick several random ground points inside the rectangle."""
        return [self.random_spawn_enu(rng, margin_m) for _ in range(count)]

    def centre_enu(self) -> tuple[float, float]:
        """The centre of the rectangle, in the Gazebo world frame.

        Returns:
            The ``(east_m, north_m)`` centre of the rectangle.
        """
        north, east = self.rotate_to_ned(
            self.size_x_m / 2.0, self.size_y_m / 2.0
        )
        return (east, north)


def default_layout() -> AnchorLayout:
    """The layout used by the simulation: a 10 m square, aligned with NED."""
    return AnchorLayout(
        size_x_m=LAYOUT_SIZE_X_M,
        size_y_m=LAYOUT_SIZE_Y_M,
        alpha_deg=0.0,
        plane_height_m=ANCHOR_PLANE_HEIGHT_M,
    )


def ned_to_enu(north_m: float, east_m: float, down_m: float) -> tuple[float, float, float]:
    """Convert an NED position into the Gazebo ENU frame.

    Args:
        north_m: North offset, in metres.
        east_m: East offset, in metres.
        down_m: Down offset, in metres (positive is down).

    Returns:
        The ``(east_m, north_m, up_m)`` triple.
    """
    return (float(east_m), float(north_m), -float(down_m))


def anchor_offsets_mm(
    layout: AnchorLayout,
) -> tuple[tuple[float, float, float], ...]:
    """The anchor positions in the anchor frame, in millimetres.

    The ROS bridge historically took its anchor positions as millimetre
    offsets in the Gazebo ENU frame, relative to the spawn point.  The
    solver now works in the anchor frame instead, so the only thing that
    is left of the old convention is the unit.

    Args:
        layout: The layout to describe.

    Returns:
        Four ``(x_mm, y_mm, z_mm)`` triples.
    """
    return tuple(
        tuple(component * 1000.0 for component in position)
        for position in layout.anchor_frame_positions()
    )


def layout_from_anchor_offsets_mm(
    offsets: Sequence[float],
) -> AnchorLayout:
    """Recover the layout from millimetre anchor offsets.

    This is the inverse of :func:`anchor_offsets_mm` and is used by the
    tools that read the anchor positions from a parameter file.

    Args:
        offsets: A flat sequence of 12 numbers, ``x0, y0, z0, x1, ...``.

    Returns:
        The layout the offsets describe.

    Raises:
        ValueError: If the sequence does not hold 12 numbers.
    """
    values = [float(v) for v in offsets]
    if len(values) < 12:
        raise ValueError("expected 12 anchor offset components, got %d" % len(values))
    positions = [
        (values[3 * i] / 1000.0, values[3 * i + 1] / 1000.0, values[3 * i + 2] / 1000.0)
        for i in range(4)
    ]
    size_x_m = positions[1][0]
    size_y_m = positions[3][1]
    alpha_deg = math.degrees(
        math.atan2(positions[3][1], positions[3][0]) - math.atan2(0.0, 1.0)
    )
    return AnchorLayout(
        size_x_m=size_x_m, size_y_m=size_y_m, alpha_deg=alpha_deg
    )
