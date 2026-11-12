"""Tests for the M2 finding: the attitude quaternion must not inject a yaw.

The finding is that sending the **identity** quaternion ``(1, 0, 0, 0)`` in
an ``ODOMETRY`` message makes ArduPilot inject ``yaw = 0`` into EKF3, which
drags the heading of the vehicle to North when the external navigation
source is selected for yaw.

These tests model ArduPilot's own processing chain (see
:mod:`uwb_pozyx_bridge.ardupilot_odometry_model`) and assert both halves of
the claim:

* the identity quaternion **does** inject a zero yaw — this is the bug;
* a ``NaN`` quaternion does **not**, while the position is still fused.
"""

import math

import pytest

from uwb_pozyx_bridge.ardupilot_odometry_model import (
    SOURCE_YAW_COMPASS,
    SOURCE_YAW_EXTNAV,
    EKF3State,
    Quaternion,
    handle_odometry,
    select_mag_fusion,
    write_ext_nav_data,
)
from uwb_pozyx_bridge.mavlink_odometry import (
    build_odometry_message,
    unknown_attitude_quaternion,
)

# ------------------------------------------------------------- the model


def test_identity_quaternion_decodes_to_zero_yaw():
    """The identity quaternion is a yaw of zero, i.e. "heading North"."""
    q = Quaternion(1.0, 0.0, 0.0, 0.0)

    _roll, _pitch, yaw = q.to_euler()

    assert yaw == pytest.approx(0.0)


def test_nan_quaternion_is_detected_as_nan():
    """``Quaternion::is_nan()`` is the guard that protects the EKF yaw."""
    assert Quaternion(*unknown_attitude_quaternion()).is_nan() is True
    assert Quaternion(1.0, 0.0, 0.0, 0.0).is_nan() is False


def test_a_partially_nan_quaternion_is_also_rejected():
    """The guard checks every component, not just the first."""
    assert Quaternion(1.0, float("nan"), 0.0, 0.0).is_nan() is True


# ------------------------------------------- the bug: identity => yaw = 0


def test_identity_quaternion_injects_a_zero_yaw_into_the_ekf():
    """Reproduce the bug: the identity quaternion reaches the EKF yaw buffer.

    This is the failure the fix has to prevent.
    """
    q = Quaternion(1.0, 0.0, 0.0, 0.0)

    _position, yaw, _velocity = handle_odometry(
        q, 1.0, 2.0, -0.5, pos_err=0.05, ang_err=0.2
    )

    assert yaw is not None
    assert yaw.yaw_ang == pytest.approx(0.0)


def test_identity_quaternion_aligns_the_ekf_yaw_to_zero():
    """With ``EK3_SRCx_YAW = EXTNAV`` the EKF yaw is reset to the sample."""
    state = EKF3State(yaw_source_last=SOURCE_YAW_EXTNAV)
    q = Quaternion(1.0, 0.0, 0.0, 0.0)

    for _ in range(10):
        position, yaw, _velocity = handle_odometry(
            q, 1.0, 2.0, -0.5, pos_err=0.05, ang_err=0.2
        )
        state.stored_ext_nav.append(position)
        if yaw is not None:
            state.stored_ext_nav_yaw_ang.append(yaw)
        select_mag_fusion(state)

    # The EKF yaw is pinned to zero, no matter what the vehicle does.
    assert state.euler_yaw == pytest.approx(0.0)
    assert len(state.fused_yaw_observations) == 10
    assert all(
        observation == pytest.approx(0.0)
        for observation in state.fused_yaw_observations
    )


def test_a_non_zero_identity_like_yaw_is_also_injected():
    """Any finite quaternion is injected, not just the identity.

    A quaternion that encodes, say, 90° of yaw would drag the vehicle to
    heading East just as effectively.
    """
    # A 90° yaw about the z axis.
    half = math.radians(90.0) / 2.0
    q = Quaternion(math.cos(half), 0.0, 0.0, math.sin(half))

    _position, yaw, _velocity = handle_odometry(
        q, 0.0, 0.0, 0.0, pos_err=0.05, ang_err=0.2
    )

    assert yaw is not None
    assert yaw.yaw_ang == pytest.approx(math.radians(90.0))


# ------------------------------------------------- the fix: NaN => no yaw


def test_nan_quaternion_produces_no_yaw_sample():
    """The ``is_nan()`` guard skips the yaw branch entirely."""
    q = Quaternion(*unknown_attitude_quaternion())

    _position, yaw, _velocity = handle_odometry(
        q, 1.0, 2.0, -0.5, pos_err=0.05, ang_err=0.2
    )

    assert yaw is None


def test_nan_quaternion_still_fuses_the_position():
    """The position branch is independent of the attitude branch."""
    q = Quaternion(*unknown_attitude_quaternion())

    position, yaw, _velocity = handle_odometry(
        q,
        1.0,
        2.0,
        -0.5,
        pos_err=0.05,
        ang_err=0.2,
        pos_noise=0.05,
    )

    assert yaw is None
    assert position.pos == pytest.approx((1.0, 2.0, -0.5))
    assert position.pos_err == pytest.approx(0.05)


def test_nan_quaternion_never_aligns_or_fuses_the_ekf_yaw():
    """The EKF yaw buffer stays empty, so nothing is ever aligned or fused."""
    state = EKF3State(yaw_source_last=SOURCE_YAW_EXTNAV)
    q = Quaternion(*unknown_attitude_quaternion())

    for _ in range(50):
        position, yaw, _velocity = handle_odometry(
            q, 1.0, 2.0, -0.5, pos_err=0.05, ang_err=0.2
        )
        state.stored_ext_nav.append(position)
        if yaw is not None:
            state.stored_ext_nav_yaw_ang.append(yaw)
        select_mag_fusion(state)

    assert state.stored_ext_nav_yaw_ang == []
    assert state.fused_yaw_observations == []
    assert not state.yaw_aligned


def test_nan_quaternion_does_not_disturb_a_compass_yaw():
    """With the compass selected for yaw, the EKF yaw is left alone."""
    state = EKF3State(yaw_source_last=SOURCE_YAW_COMPASS, euler_yaw=1.234)
    q = Quaternion(*unknown_attitude_quaternion())

    position, yaw, _velocity = handle_odometry(
        q, 1.0, 2.0, -0.5, pos_err=0.05, ang_err=0.2
    )
    state.stored_ext_nav.append(position)
    if yaw is not None:
        state.stored_ext_nav_yaw_ang.append(yaw)
    select_mag_fusion(state)

    assert state.euler_yaw == pytest.approx(1.234)
    assert state.fused_yaw_observations == []


# -------------------------------------------------- the message itself


def test_the_bridge_sends_a_nan_quaternion():
    """The message the node actually sends must not carry a finite yaw."""
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)

    assert all(math.isnan(component) for component in msg.q)


def test_the_bridge_message_does_not_inject_a_yaw():
    """End to end: the message the node sends reaches no EKF yaw buffer.

    This is the regression test for the finding itself.
    """
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)
    q = Quaternion(*msg.q)

    position, yaw, _velocity = handle_odometry(
        q,
        msg.x,
        msg.y,
        msg.z,
        pos_err=0.05,
        ang_err=0.2,
    )

    assert yaw is None
    assert position.pos == pytest.approx((msg.x, msg.y, msg.z))


def test_a_nan_quaternion_does_not_poison_the_velocity():
    """A NaN quaternion makes the rotated velocity NaN.

    This is harmless here because the node sends no velocity at all, but it
    is worth pinning down: it means the quaternion must stay NaN only as
    long as the velocity is unknown.
    """
    q = Quaternion(*unknown_attitude_quaternion())

    _position, _yaw, velocity = handle_odometry(
        q, 0.0, 0.0, 0.0, pos_err=0.05, ang_err=0.2, vel=(1.0, 2.0, 3.0)
    )

    assert all(math.isnan(component) for component in velocity)


# ------------------------------------------------------ upstream details


def test_the_yaw_error_is_never_better_than_five_degrees():
    """``MAX(angErr, radians(5.0f))`` — upstream does not trust a good yaw.

    The angle error is first clamped up to ``VISO_YAW_M_NSE`` (0.2 rad by
    default) and only then floored at 5 degrees, so the 5-degree floor is
    the binding one in practice.
    """
    q = Quaternion(1.0, 0.0, 0.0, 0.0)

    _position, yaw, _velocity = handle_odometry(
        q,
        0.0,
        0.0,
        0.0,
        pos_err=0.05,
        ang_err=0.0,
        yaw_noise=0.0,
    )

    assert yaw is not None
    assert yaw.yaw_ang_err == pytest.approx(math.radians(5.0))


def test_write_ext_nav_data_rejects_a_nan_position():
    """The position branch has its own NaN guard."""
    q = Quaternion(1.0, 0.0, 0.0, 0.0)

    with pytest.raises(ValueError):
        write_ext_nav_data(
            q, float("nan"), 0.0, 0.0, pos_err=0.05, ang_err=0.2
        )


def test_the_compass_source_ignores_the_external_nav_yaw():
    """Even a finite quaternion does not disturb a compass-driven yaw."""
    state = EKF3State(yaw_source_last=SOURCE_YAW_COMPASS, euler_yaw=2.0)
    q = Quaternion(1.0, 0.0, 0.0, 0.0)

    position, yaw, _velocity = handle_odometry(
        q, 0.0, 0.0, 0.0, pos_err=0.05, ang_err=0.2
    )
    state.stored_ext_nav.append(position)
    if yaw is not None:
        state.stored_ext_nav_yaw_ang.append(yaw)
    select_mag_fusion(state)

    # The sample is buffered ...
    assert len(state.stored_ext_nav_yaw_ang) == 1
    # ... but never fused.
    assert state.fused_yaw_observations == []
    assert state.euler_yaw == pytest.approx(2.0)
