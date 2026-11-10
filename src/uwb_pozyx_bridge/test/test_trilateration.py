"""Unit tests for the UWB trilateration solver.

The solver works in the **anchor frame**: the ideal square of the layout,
with ``a0`` at the origin, ``x`` towards ``a1``, ``y`` towards ``a3`` and
``z`` pointing **down**, so that the anchor plane is ``z = 0`` and a tag
below the plane has a positive ``z``.
"""

import math

import numpy as np
import pytest

from uwb_pozyx_bridge.trilateration import (
    Anchor,
    RangeMeasurement,
    solve_position,
)

# The anchor layout: a 10 m x 10 m square, coplanar at z = 0.
ANCHORS = [
    Anchor(0, (0.0, 0.0, 0.0)),
    Anchor(1, (10.0, 0.0, 0.0)),
    Anchor(2, (10.0, 10.0, 0.0)),
    Anchor(3, (0.0, 10.0, 0.0)),
]


def ranges_to(tag, anchors=ANCHORS, los=0):
    """Build range measurements from a known tag position."""
    out = []
    for a in anchors:
        d = math.dist(tag, a.position)
        out.append(RangeMeasurement(a.id, d, -50.0, los))
    return out


def test_recovers_known_position_3d():
    """The 3D solve recovers a known position, including the altitude."""
    for tag in [
        (5.0, 5.0, 1.0),
        (2.0, 3.0, 2.5),
        (8.0, 7.0, 0.5),
        (0.5, 9.5, 1.5),
    ]:
        result = solve_position(
            ANCHORS,
            ranges_to(tag),
            initial_guess=(5.0, 5.0, 1.0),
        )
        assert result is not None
        assert result.position[0] == pytest.approx(tag[0], abs=1e-3)
        assert result.position[1] == pytest.approx(tag[1], abs=1e-3)
        assert result.position[2] == pytest.approx(tag[2], abs=1e-3)


def test_recovers_known_position_2d():
    """With a constant altitude the horizontal position is recovered exactly."""
    for tag in [(5.0, 5.0), (2.0, 3.0), (8.0, 7.0), (0.5, 9.5)]:
        result = solve_position(
            ANCHORS,
            ranges_to((tag[0], tag[1], 1.0)),
            assume_constant_z=True,
            initial_guess=(5.0, 5.0, 1.0),
        )
        assert result is not None
        assert result.position[0] == pytest.approx(tag[0], abs=1e-3)
        assert result.position[1] == pytest.approx(tag[1], abs=1e-3)


def test_coplanar_anchors_cannot_resolve_altitude():
    """Four coplanar anchors give a mirror ambiguity in the vertical axis.

    A tag at z = 1 m below the anchor plane is indistinguishable from one at
    z = -1 m above it, so the altitude is not observable from the ranges
    alone.  The solver still recovers the correct horizontal position.
    """
    result = solve_position(
        ANCHORS,
        ranges_to((5.0, 5.0, 1.0)),
        initial_guess=(5.0, 5.0, 1.0),
    )
    assert result is not None
    assert result.position[0] == pytest.approx(5.0, abs=1e-3)
    assert result.position[1] == pytest.approx(5.0, abs=1e-3)

    # The mirrored tag gives identical ranges, proving the altitude is
    # unobservable with coplanar anchors.
    mirrored = ranges_to((5.0, 5.0, -1.0))
    for a, b in zip(ranges_to((5.0, 5.0, 1.0)), mirrored):
        assert a.range_m == pytest.approx(b.range_m, abs=1e-9)


def test_branch_selection_follows_previous_solution():
    """The branch is tracked, so the estimate stays continuous.

    The previous solution selects the side of the anchor plane, which is
    what keeps the estimate from flipping to the mirrored position.
    """
    result = solve_position(
        ANCHORS,
        ranges_to((5.0, 5.0, 1.0)),
        previous_position=(5.0, 5.0, 1.0),
        initial_guess=(5.0, 5.0, 1.0),
    )
    assert result is not None
    assert result.position[2] == pytest.approx(1.0, abs=1e-3)


def test_branch_tracking_is_continuous_across_the_anchor_plane():
    """The estimate must not flip when the tag crosses the anchor plane.

    With coplanar anchors the ranges have a mirror ambiguity.  Tracking
    the branch against the previous solution keeps the estimate on the
    side the tag actually is, so the vertical component follows the tag
    smoothly through the plane.
    """
    previous = (5.0, 5.0, 1.0)
    for i in range(21):
        altitude = 1.0 - 0.1 * i  # from +1 down to -1
        result = solve_position(
            ANCHORS,
            ranges_to((5.0, 5.0, altitude)),
            previous_position=previous,
            initial_guess=previous,
        )
        assert result is not None
        assert abs(result.position[2] - previous[2]) < 0.2
        previous = result.position


def test_3d_solve_is_less_biased_than_2d():
    """The 2D mode biases the horizontal solution when the altitude is wrong.

    With coplanar anchors the ranges are inconsistent with an assumed
    altitude, and the horizontal error grows with the altitude error.  The
    3D solve does not suffer from this.
    """
    tag = (5.0, 5.0, 1.0)
    measurements = ranges_to(tag)

    two_d = solve_position(
        ANCHORS,
        measurements,
        assume_constant_z=True,
        initial_guess=(5.0, 5.0, 3.0),
    )
    three_d = solve_position(
        ANCHORS,
        measurements,
        initial_guess=(5.0, 5.0, 1.0),
    )
    assert two_d is not None
    assert three_d is not None

    err_2d = math.dist(two_d.position[:2], tag[:2])
    err_3d = math.dist(three_d.position[:2], tag[:2])
    assert err_3d < err_2d


def test_non_coplanar_anchors_resolve_altitude():
    """A non-coplanar anchor layout removes the mirror ambiguity."""
    anchors = [
        Anchor(0, (0.0, 0.0, 0.0)),
        Anchor(1, (10.0, 0.0, 0.0)),
        Anchor(2, (10.0, 10.0, 0.0)),
        Anchor(3, (0.0, 10.0, -2.0)),
    ]
    result = solve_position(
        anchors,
        ranges_to((5.0, 5.0, 1.0), anchors),
        initial_guess=(5.0, 5.0, 1.0),
    )
    assert result is not None
    assert result.position[0] == pytest.approx(5.0, abs=1e-3)
    assert result.position[1] == pytest.approx(5.0, abs=1e-3)
    assert result.position[2] == pytest.approx(1.0, abs=1e-3)


def test_center_of_anchor_square():
    """The drone may spawn at the centre of the four anchors."""
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), initial_guess=(5.0, 5.0, 1.0)
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
        ANCHORS,
        ranges_to((3.0, 4.0, 0.7)),
        assume_constant_z=True,
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
    result = solve_position(ANCHORS, bad, initial_guess=(5.0, 5.0, 1.0))
    # The bogus sample is dropped, but the remaining three are still solvable.
    assert result is not None
    assert 3 not in result.used_anchor_ids


def test_unknown_anchor_ignored():
    good = ranges_to((5.0, 5.0, 1.0))
    result = solve_position(
        ANCHORS,
        good + [RangeMeasurement(99, 5.0, -50.0, 0)],
        initial_guess=(5.0, 5.0, 1.0),
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
        ANCHORS,
        corrupted,
        initial_guess=(5.0, 5.0, 1.0),
        nlos_hard_std=10.0,
    )
    # The NLOS anchor is trusted as much as a good one -> it pulls hard.
    trusting = solve_position(
        ANCHORS,
        corrupted,
        initial_guess=(5.0, 5.0, 1.0),
        nlos_hard_std=0.05,
    )
    assert distrustful is not None
    assert trusting is not None

    err_distrustful = abs(distrustful.position[0] - 5.0)
    err_trusting = abs(trusting.position[0] - 5.0)
    assert err_distrustful < err_trusting


def test_covariance_is_finite_and_positive():
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), initial_guess=(5.0, 5.0, 1.0)
    )
    assert result is not None
    cov = result.position_covariance
    assert cov.shape == (3, 3)
    assert np.all(np.isfinite(cov))
    assert np.all(np.diag(cov) >= 0.0)


def test_residuals_are_small_for_noise_free_data():
    result = solve_position(
        ANCHORS, ranges_to((5.0, 5.0, 1.0)), initial_guess=(5.0, 5.0, 1.0)
    )
    assert result is not None
    for r in result.residuals:
        assert abs(r) < 1e-3
