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
    """A single UWB anchor in the world frame (ENU, metres)."""

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
    assume_z: bool,
) -> np.ndarray:
    """Closed-form linearisation used to seed the Gauss-Newton iteration.

    Squaring the range equation and subtracting the linearised form gives a
    linear system in the position increments.  When ``assume_z`` is set the
    altitude is held at its initial value and only the horizontal components
    are solved for; solving the full 3D system would move the altitude onto
    the mirrored solution of the coplanar-anchor ambiguity.
    """
    x, y, z = x0
    a_rows = []
    b_vec = []
    for anchor, meas in zip(anchors, ranges):
        ax, ay, az = anchor.position
        d0 = math.sqrt((ax - x) ** 2 + (ay - y) ** 2 + (az - z) ** 2)
        rhs = meas.range_m**2 - d0**2
        if assume_z:
            a_rows.append([2.0 * (ax - x), 2.0 * (ay - y)])
            b_vec.append(rhs)
        else:
            a_rows.append([2.0 * (ax - x), 2.0 * (ay - y), 2.0 * (az - z)])
            b_vec.append(
                rhs
                + 2.0 * (ax * x + ay * y + az * z)
                - (ax * ax + ay * ay + az * az)
            )
    a = np.array(a_rows, dtype=float)
    b = np.array(b_vec, dtype=float)
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    if assume_z:
        return np.array([sol[0], sol[1], z], dtype=float)
    return np.array([sol[0], sol[1], sol[2]], dtype=float)


def _gauss_newton(
    x0: np.ndarray,
    anchors: Sequence[Anchor],
    ranges: Sequence[RangeMeasurement],
    weights: Sequence[float],
    assume_z: bool,
    max_iter: int = 10,
    tol: float = 1e-4,
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

        if assume_z:
            # Solve in 2D and keep z fixed.
            j2 = jac[:, :2]
            try:
                step = np.linalg.solve(j2.T @ w @ j2, j2.T @ w @ residual)
            except np.linalg.LinAlgError:
                break
            x_new = x.copy()
            x_new[0] -= step[0]
            x_new[1] -= step[1]
        else:
            try:
                step = np.linalg.solve(jac.T @ w @ jac, jac.T @ w @ residual)
            except np.linalg.LinAlgError:
                break
            x_new = x - step

        if np.linalg.norm(x_new - x) < tol:
            return x_new
        x = x_new

    return x


def _estimate_covariance(
    sol: np.ndarray,
    anchors: Sequence[Anchor],
    weights: Sequence[float],
    assume_z: bool,
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

    if assume_z:
        jac = jac[:, :2]

    try:
        cov = np.linalg.inv(jac.T @ w @ jac)
    except np.linalg.LinAlgError:
        cov = np.eye(jac.shape[1]) * 0.5

    # Expand to a full 3x3 position covariance.
    full = np.zeros((3, 3))
    full[: jac.shape[1], : jac.shape[1]] = cov
    return full


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
    assume_constant_z: bool = True,
) -> Optional[TrilaterationResult]:
    """Estimate the tag position from the available anchor ranges.

    Args:
        anchors: Known anchor positions in the world frame.
        ranges: Range measurements to solve for.
        los_std: Standard deviation of a line-of-sight range, in metres.
        nlos_soft_std: Standard deviation of a soft non-line-of-sight range.
        nlos_hard_std: Standard deviation of a hard non-line-of-sight range.
        min_anchors: Minimum number of anchors required for a solution.
        max_range: Ranges beyond this distance are rejected.
        initial_guess: Starting point for the solver, in world coordinates.
        assume_constant_z: Solve in 2D and keep the height fixed.

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

    try:
        x0 = _linear_seed(used, used_ranges, x0, assume_constant_z)
    except np.linalg.LinAlgError:
        pass  # keep the initial guess

    try:
        sol = _gauss_newton(
            x0, used, used_ranges, weights, assume_z=assume_constant_z
        )
    except np.linalg.LinAlgError:
        return None

    if not np.all(np.isfinite(sol)):
        return None

    cov = _estimate_covariance(sol, used, weights, assume_constant_z)

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
