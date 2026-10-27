"""Unit tests for the UWB trilateration solver."""

import math

import numpy as np
import pytest

from uwb_pozyx_bridge.trilateration import (
    Anchor,
    RangeMeasurement,
    solve_position,
)

# The Gazebo world layout: a 10 m x 10 m square of anchors at z = 2.0 m.
ANCHORS = [
    Anchor(0, (0.0, 0.0, 2.0)),
    Anchor(1, (10.0, 0.0, 2.0)),
    Anchor(2, (10.0, 10.0, 2.0)),
    Anchor(3, (0.0, 10.0, 2.0)),
]


def ranges_to(tag, anchors=ANCHORS, los=0):
    """Build range measurements from a known tag position."""
    out = []
    for a in anchors:
        d = math.dist(tag, a.position)
        out.append(RangeMeasurement(a.id, d, -50.0, los))
    return out


def test_recovers_known_position_2d():
    """With a constant altitude the horizontal position is recovered exactly."""
    for tag in [(5.0, 5.0), (2.0, 3.0), (8.0, 7.0), (0.5, 9.5)]:
        result = solve_position(
            ANCHORS, ranges_to((tag[0], tag[1], 1.0)),
            assume_constant_z=True,
            initial_guess=(5.0, 5.0, 1.0),
        )
        assert result is not None
        assert result.position[0] == pytest.approx(tag[0], abs=1e-3)
        assert result.position[1] == pytest.approx(tag[1], abs=1e-3)


def test_coplanar_anchors_cannot_resolve_altitude():
    """Four coplanar anchors give a mirror ambiguity in the vertical axis.

    A tag at z = 1 m below the anchor plane is indistinguishable from one at
    z = 3 m above it, so the altitude is not observable from the ranges
    alone.  The solver still recovers the correct horizontal position, which
    is what the 2D default mode relies on.
    """
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), assume_constant_z=False,
        initial_guess=(5.0, 5.0, 1.0),
    )
    assert result is not None
    assert result.position[0] == pytest.approx(5.0, abs=1e-3)
    assert result.position[1] == pytest.approx(5.0, abs=1e-3)

    # The same tag one metre higher gives identical ranges, proving the
    # altitude is unobservable with coplanar anchors.
    higher = ranges_to((5.0, 5.0, 3.0))
    for a, b in zip(ranges_to((5.0, 5.0, 1.0)), higher):
        assert a.range_m == pytest.approx(b.range_m, abs=1e-9)


def test_non_coplanar_anchors_resolve_altitude():
    """A non-coplanar anchor layout removes the mirror ambiguity."""
    anchors = [
        Anchor(0, (0.0, 0.0, 2.0)),
        Anchor(1, (10.0, 0.0, 2.0)),
        Anchor(2, (10.0, 10.0, 2.0)),
        Anchor(3, (0.0, 10.0, 4.0)),
    ]
    result = solve_position(
        anchors, ranges_to((5.0, 5.0, 1.0), anchors), assume_constant_z=False,
        initial_guess=(5.0, 5.0, 1.0),
    )
    assert result is not None
    assert result.position[0] == pytest.approx(5.0, abs=1e-3)
    assert result.position[1] == pytest.approx(5.0, abs=1e-3)
    assert result.position[2] == pytest.approx(1.0, abs=1e-3)


def test_center_of_anchor_square():
    """The drone spawns at the centre of the four anchors."""
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), assume_constant_z=False
    )
    assert result is not None
    assert result.position[0] == pytest.approx(5.0, abs=1e-3)
    assert result.position[1] == pytest.approx(5.0, abs=1e-3)


def test_2d_solution_keeps_z_fixed():
    """In 2D mode the altitude stays at the assumed value.

    The ranges must be generated with that same altitude, otherwise they are
    inconsistent with the 2D model and the horizontal solution is biased.
    """
    result = solve_position(
        ANCHORS, ranges_to((3.0, 4.0, 0.7)), assume_constant_z=True,
        initial_guess=(5.0, 5.0, 0.7),
    )
    assert result is not None
    assert result.position[0] == pytest.approx(3.0, abs=1e-3)
    assert result.position[1] == pytest.approx(4.0, abs=1e-3)
    assert result.position[2] == pytest.approx(0.7, abs=1e-9)

def test_too_few_anchors_returns_none():
    result = solve_position(ANCHORS, ranges_to((5.0, 5.0, 1.0))[:2])
    assert result is None


def test_out_of_range_measurement_rejected():
    good = ranges_to((5.0, 5.0, 1.0))
    bad = good[:3] + [RangeMeasurement(3, 999.0, -50.0, 0)]
    result = solve_position(ANCHORS, bad, assume_constant_z=False)
    # The bogus sample is dropped, but the remaining three are still solvable.
    assert result is not None
    assert 3 not in result.used_anchor_ids


def test_unknown_anchor_ignored():
    good = ranges_to((5.0, 5.0, 1.0))
    result = solve_position(
        ANCHORS, good + [RangeMeasurement(99, 5.0, -50.0, 0)],
        assume_constant_z=False,
    )
    assert result is not None
    assert 99 not in result.used_anchor_ids


def test_nlos_anchor_is_down_weighted():
    """The NLOS standard deviation controls how much a bad anchor is trusted.

    A large ``nlos_hard_std`` means the measurement is expected to be very
    inaccurate, so it gets a small weight and barely moves the solution.  A
    small value means the anchor is trusted and drags the solution towards the
    corrupted range.
    """
    good = ranges_to((5.0, 5.0, 1.0))
    # Corrupt one range by 1 m and mark it NLOS-hard.
    corrupted = list(good)
    corrupted[0] = RangeMeasurement(
        0, corrupted[0].range_m + 1.0, -85.0, 2
    )

    # The NLOS anchor is expected to be unreliable -> down-weighted.
    distrustful = solve_position(
        ANCHORS, corrupted, assume_constant_z=False,
        nlos_hard_std=10.0,
    )
    # The NLOS anchor is trusted as much as a good one -> it pulls hard.
    trusting = solve_position(
        ANCHORS, corrupted, assume_constant_z=False,
        nlos_hard_std=0.05,
    )
    assert distrustful is not None
    assert trusting is not None

    err_distrustful = abs(distrustful.position[0] - 5.0)
    err_trusting = abs(trusting.position[0] - 5.0)
    assert err_distrustful < err_trusting


def test_covariance_is_finite_and_positive():
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), assume_constant_z=False
    )
    assert result is not None
    cov = result.position_covariance
    assert cov.shape == (3, 3)
    assert np.all(np.isfinite(cov))
    assert np.all(np.diag(cov) >= 0.0)


def test_residuals_are_small_for_noise_free_data():
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), assume_constant_z=False
    )
    assert result is not None
    for r in result.residuals:
        assert abs(r) < 1e-3
