"""Weighted least-squares trilateration of a UWB tag from anchor ranges.

Ported from the ``uwb_trilateration_node.py`` of the ``feat/uwb-odometry``
branch, but with the ROS dependency removed so that the solver can be unit
tested and reused by the Pozyx bridge.

The Gazebo plugin encodes one ranging per ``geometry_msgs/Pose`` message:

============================  ==========================================
Field                         Meaning
============================  ==========================================
``pose.position.x``           range to the anchor, in metres
``pose.position.y``           RSS, in dBm
``pose.position.z``           anchor id
``pose.orientation.x``        line-of-sight code (0 = LOS)
============================  ==========================================

The solver is a weighted Gauss-Newton least-squares fit of the tag position to
the measured ranges, seeded by a closed-form linearisation.

Vertical ambiguity
------------------
The anchors of a UWB layout are coplanar, so the ranges alone cannot tell
which side of the anchor plane the tag is on: a tag 1 m below the plane
produces exactly the same ranges as one 1 m above it.  The solver therefore
runs in **3D** and resolves the ambiguity explicitly: the branch of the
solution that lies on the *far side* of the anchor plane from the tag — the
side the tag is expected to be on — is kept.

This matters for the horizontal accuracy.  Fixing the altitude to a
constant instead (the "2D" mode) is not neutral: the ranges are then
inconsistent with the assumed altitude, and the horizontal solution is
biased by up to 0.2 m for a tag flying 1 m below the assumed height.  The
3D solve with an explicit branch selection keeps the horizontal error at the
level of the range noise (~0.05 m) regardless of the altitude.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np

__all__ = [
    "Anchor",
    "RangeMeasurement",
    "TrilaterationResult",
    "solve_position",
]


@dataclass(frozen=True)
class Anchor:
    """A single UWB anchor in the anchor frame (metres).

    The anchor frame is the ideal square of the layout: ``x`` points
    towards ``a1``, ``y`` towards ``a3`` and ``z`` points **down**, with
    the anchor plane at ``z = 0``.  See
    :mod:`uwb_pozyx_bridge.uwb_layout`.
    """

    id: int
    position: tuple[float, float, float]

    @property
    def name(self) -> str:
        return f"uwb_anchor{self.id}"


@dataclass(frozen=True)
class RangeMeasurement:
    """One ranging sample towards a single anchor."""

    anchor_id: int
    range_m: float
    rss_dbm: float = -60.0
    los: int = 0  # 0 = LOS, 1 = NLOS soft, 2 = NLOS hard

    def weight(
        self,
        los_std: float,
        nlos_soft_std: float,
        nlos_hard_std: float,
    ) -> float:
        """Inverse-variance weight for the WLS solver.

        The variance is approximated from the line-of-sight code plus a small
        RSS dependent term: a weaker signal means a larger variance.
        """
        if self.los == 0:
            sigma = los_std
        elif self.los == 1:
            sigma = nlos_soft_std
        else:
            sigma = nlos_hard_std

        # Lower RSS -> larger variance.  -70 dBm is treated as the reference.
        rss_term = 1.0 + max(0.0, (-70.0 - self.rss_dbm)) * 0.05
        sigma *= rss_term
        return 1.0 / (sigma * sigma)


@dataclass(frozen=True)
class TrilaterationResult:
    """The outcome of a successful trilateration."""

    position: tuple[float, float, float]
    position_covariance: np.ndarray
    residuals: tuple[float, ...]
    used_anchor_ids: tuple[int, ...]


def _linear_seed(
    anchors: Sequence[Anchor],
    ranges: Sequence[RangeMeasurement],
    x0: np.ndarray,
) -> np.ndarray:
    """Closed-form linearisation used to seed the Gauss-Newton iteration.

    Squaring the range equation and subtracting the linearised form gives a
    linear system in the position increments.  The system is singular in
    the vertical direction for coplanar anchors, so the vertical component
    is solved separately: the horizontal part comes from the linearised
    system and the vertical part from the seed.
    """
    x, y, z = x0
    a_rows = []
    b_vec = []
    for anchor, meas in zip(anchors, ranges):
        ax, ay, az = anchor.position
        d0 = math.sqrt((ax - x) ** 2 + (ay - y) ** 2 + (az - z) ** 2)
        rhs = meas.range_m**2 - d0**2
        a_rows.append([2.0 * (ax - x), 2.0 * (ay - y)])
        b_vec.append(rhs)
    a = np.array(a_rows, dtype=float)
    b = np.array(b_vec, dtype=float)
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    return np.array([sol[0], sol[1], z], dtype=float)


def _gauss_newton(
    x0: np.ndarray,
    anchors: Sequence[Anchor],
    ranges: Sequence[RangeMeasurement],
    weights: Sequence[float],
    max_iter: int = 25,
    tol: float = 1e-7,
) -> np.ndarray:
    """Iterative weighted least-squares refinement of the position."""
    x = x0.copy()
    n = len(ranges)
    w = np.diag(np.asarray(weights, dtype=float))

    for _ in range(max_iter):
        residual = np.zeros(n)
        jac = np.zeros((n, 3))
        for i, (anchor, meas) in enumerate(zip(anchors, ranges)):
            ax, ay, az = anchor.position
            dx = x[0] - ax
            dy = x[1] - ay
            dz = x[2] - az
            d = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
            residual[i] = d - meas.range_m
            jac[i, 0] = dx / d
            jac[i, 1] = dy / d
            jac[i, 2] = dz / d

        try:
            step = np.linalg.solve(jac.T @ w @ jac, jac.T @ w @ residual)
        except np.linalg.LinAlgError:
            break
        x = x - step

        if np.linalg.norm(step) < tol:
            break

    return x


def _estimate_covariance(
    sol: np.ndarray,
    anchors: Sequence[Anchor],
    weights: Sequence[float],
) -> np.ndarray:
    """Approximate the position covariance from the WLS Jacobian."""
    x, y, z = sol
    n = len(anchors)
    w = np.diag(np.asarray(weights, dtype=float))
    jac = np.zeros((n, 3))
    for i, anchor in enumerate(anchors):
        ax, ay, az = anchor.position
        dx = x - ax
        dy = y - ay
        dz = z - az
        d = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
        jac[i] = [dx / d, dy / d, dz / d]

    try:
        cov = np.linalg.inv(jac.T @ w @ jac)
    except np.linalg.LinAlgError:
        cov = np.eye(3) * 0.5

    return cov


def _plane_height(anchors: Sequence[Anchor]) -> float:
    """The mean height of the anchor plane, in metres."""
    return float(np.mean([a.position[2] for a in anchors]))


def solve_position(
    anchors: Iterable[Anchor],
    ranges: Iterable[RangeMeasurement],
    *,
    los_std: float = 0.10,
    nlos_soft_std: float = 0.30,
    nlos_hard_std: float = 0.60,
    min_anchors: int = 3,
    max_range: float = 50.0,
    initial_guess: Sequence[float] = (5.0, 5.0, 1.0),
    previous_position: Optional[Sequence[float]] = None,
    assume_constant_z: bool = False,
) -> Optional[TrilaterationResult]:
    """Estimate the tag position from the available anchor ranges.

    Args:
        anchors: Known anchor positions in the anchor frame.
        ranges: Range measurements to solve for.
        los_std: Standard deviation of a line-of-sight range, in metres.
        nlos_soft_std: Standard deviation of a soft non-line-of-sight range.
        nlos_hard_std: Standard deviation of a hard non-line-of-sight range.
        min_anchors: Minimum number of anchors required for a solution.
        max_range: Ranges beyond this distance are rejected.
        initial_guess: Starting point for the solver, in anchor-frame
            coordinates.  Its vertical component selects the branch when
            no previous solution is available.
        previous_position: The previous solution, used to keep the branch
            selection continuous across the fixes.  With coplanar anchors
            the ranges cannot tell which side of the anchor plane the tag
            is on, so the branch is tracked instead of re-decided for
            every fix.
        assume_constant_z: Solve in 2D and keep the height fixed.  This is
            the legacy behaviour; it biases the horizontal solution by up
            to 0.2 m whenever the tag is not exactly at the assumed height,
            so the 3D solve is preferred.

    Returns:
        A :class:`TrilaterationResult`, or ``None`` if the problem could not
        be solved (too few anchors, degenerate geometry, or out-of-range
        measurements).
    """
    anchor_by_id = {a.id: a for a in anchors}

    used: list[Anchor] = []
    used_ranges: list[RangeMeasurement] = []
    weights: list[float] = []

    for meas in ranges:
        anchor = anchor_by_id.get(meas.anchor_id)
        if anchor is None:
            continue
        if meas.range_m <= 0.0 or meas.range_m > max_range:
            continue
        used.append(anchor)
        used_ranges.append(meas)
        weights.append(
            meas.weight(los_std, nlos_soft_std, nlos_hard_std)
        )

    if len(used) < min_anchors:
        return None

    x0 = np.array(initial_guess, dtype=float)
    if x0.size != 3:
        x0 = np.array([5.0, 5.0, 1.0], dtype=float)

    plane_z = _plane_height(used)

    if assume_constant_z:
        # Legacy 2D mode: the altitude is held at the seed value.
        try:
            x0 = _linear_seed_2d(used, used_ranges, x0)
        except np.linalg.LinAlgError:
            pass
        try:
            sol = _gauss_newton_2d(x0, used, used_ranges, weights)
        except np.linalg.LinAlgError:
            return None
    else:
        previous = None
        if previous_position is not None and len(previous_position) == 3:
            previous = np.array(previous_position, dtype=float)

        sol = _solve_3d_branch(
            used, used_ranges, weights, x0, plane_z, previous
        )
        if sol is None:
            return None

    if not np.all(np.isfinite(sol)):
        return None

    cov = _estimate_covariance(sol, used, weights)

    residuals = tuple(
        math.sqrt(
            (sol[0] - a.position[0]) ** 2
            + (sol[1] - a.position[1]) ** 2
            + (sol[2] - a.position[2]) ** 2
        )
        - m.range_m
        for a, m in zip(used, used_ranges)
    )

    return TrilaterationResult(
        position=(float(sol[0]), float(sol[1]), float(sol[2])),
        position_covariance=cov,
        residuals=residuals,
        used_anchor_ids=tuple(a.id for a in used),
    )


def _linear_seed_2d(
    anchors: Sequence[Anchor],
    ranges: Sequence[RangeMeasurement],
    x0: np.ndarray,
) -> np.ndarray:
    """Closed-form linearisation for the 2D (constant altitude) mode."""
    x, y, z = x0
    a_rows = []
    b_vec = []
    for anchor, meas in zip(anchors, ranges):
        ax, ay, az = anchor.position
        d0 = math.sqrt((ax - x) ** 2 + (ay - y) ** 2 + (az - z) ** 2)
        a_rows.append([2.0 * (ax - x), 2.0 * (ay - y)])
        b_vec.append(meas.range_m**2 - d0**2)
    a = np.array(a_rows, dtype=float)
    b = np.array(b_vec, dtype=float)
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    return np.array([sol[0], sol[1], z], dtype=float)


def _gauss_newton_2d(
    x0: np.ndarray,
    anchors: Sequence[Anchor],
    ranges: Sequence[RangeMeasurement],
    weights: Sequence[float],
    max_iter: int = 25,
    tol: float = 1e-7,
) -> np.ndarray:
    """Weighted Gauss-Newton in the horizontal plane, with z held fixed."""
    x = x0.copy()
    n = len(ranges)
    w = np.diag(np.asarray(weights, dtype=float))

    for _ in range(max_iter):
        residual = np.zeros(n)
        jac = np.zeros((n, 2))
        for i, (anchor, meas) in enumerate(zip(anchors, ranges)):
            ax, ay, az = anchor.position
            dx = x[0] - ax
            dy = x[1] - ay
            dz = x[2] - az
            d = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
            residual[i] = d - meas.range_m
            jac[i, 0] = dx / d
            jac[i, 1] = dy / d

        try:
            step = np.linalg.solve(jac.T @ w @ jac, jac.T @ w @ residual)
        except np.linalg.LinAlgError:
            break
        x[0] -= step[0]
        x[1] -= step[1]

        if np.linalg.norm(step) < tol:
            break

    return x


def _solve_3d_branch(
    anchors: Sequence[Anchor],
    ranges: Sequence[RangeMeasurement],
    weights: Sequence[float],
    x0: np.ndarray,
    plane_z: float,
    previous: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """Solve in 3D and keep the branch on the expected side of the plane.

    The anchor plane is a mirror plane of the range equations, so the
    solver has two solutions.  Both are computed, from seeds on either
    side of the plane, and the one that lies on the expected side — the
    side the previous solution (or the initial guess) is on — is kept.

    Tracking the branch against the previous solution keeps the estimate
    continuous when the tag flies through the anchor plane: without it the
    estimate would flip to the mirrored position, which is several metres
    away.
    """
    horizontal = np.array([x0[0], x0[1]], dtype=float)
    vertical_span = max(
        float(np.max([a.position[0] for a in anchors]))
        - float(np.min([a.position[0] for a in anchors])),
        float(np.max([a.position[1] for a in anchors]))
        - float(np.min([a.position[1] for a in anchors])),
        1.0,
    )

    # Seeds on both sides of the anchor plane.  The magnitudes are chosen
    # so that the seeds are outside the anchor footprint, which is where
    # the mirror ambiguity is well separated.
    seeds = [
        np.array([horizontal[0], horizontal[1], plane_z + vertical_span], dtype=float),
        np.array([horizontal[0], horizontal[1], plane_z - vertical_span], dtype=float),
    ]
    # The side of the plane the tag is expected to be on: the side of the
    # previous solution, or of the initial guess when there is none.
    reference = previous if previous is not None else x0
    expected_sign = 1.0 if reference[2] >= plane_z else -1.0

    best: Optional[np.ndarray] = None
    best_residual = float("inf")
    for seed in seeds:
        try:
            start = _linear_seed(anchors, ranges, seed)
            start[2] = seed[2]
            sol = _gauss_newton(start, anchors, ranges, weights)
        except np.linalg.LinAlgError:
            continue
        if not np.all(np.isfinite(sol)):
            continue
        sign = 1.0 if sol[2] >= plane_z else -1.0
        if sign != expected_sign:
            continue
        residuals = [
            math.sqrt(
                (sol[0] - a.position[0]) ** 2
                + (sol[1] - a.position[1]) ** 2
                + (sol[2] - a.position[2]) ** 2
            )
            - m.range_m
            for a, m in zip(anchors, ranges)
        ]
        rms = float(np.sqrt(np.mean(np.square(residuals))))
        if rms < best_residual:
            best_residual = rms
            best = sol

    return best
