"""A faithful model of ArduPilot's ODOMETRY → EXTNAV processing chain.

This module reproduces, in Python, the exact code path an ``ODOMETRY``
message takes inside ArduPilot, so that the behaviour of the bridge can be
tested without flying anything.  Every function below carries the file and
line number of the upstream code it models.

The chain is::

    GCS_MAVLINK::handle_odometry()          GCS_Common.cpp
        -> AP_VisualOdom_MAV::handle_pose_estimate()   AP_VisualOdom_MAV.cpp
            -> NavEKF3_core::writeExtNavData()          AP_NavEKF3_Measurements.cpp
                -> NavEKF3_core::SelectMagFusion()      AP_NavEKF3_MagFusion.cpp

The point of the model is the yaw branch of ``writeExtNavData()``: the
attitude quaternion of an ``ODOMETRY`` message is the *only* thing that can
feed the EKF yaw from the external navigation source, and the guard that
decides whether it is fed at all is ``Quaternion::is_nan()``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = [
    "Quaternion",
    "ExtNavSample",
    "EKF3State",
    "handle_odometry",
    "write_ext_nav_data",
    "select_mag_fusion",
]


# --------------------------------------------------------------------------
# Quaternion — mirrors AP_Math/quaternion.h
# --------------------------------------------------------------------------


class Quaternion:
    """A minimal mirror of ``AP_Math::QuaternionT``.

    Only the parts that the ODOMETRY chain uses are modelled: the
    constructor, :meth:`is_nan` and :meth:`to_euler`.

    Args:
        q1: The ``w`` component.
        q2: The ``x`` component.
        q3: The ``y`` component.
        q4: The ``z`` component.
    """

    __slots__ = ("q1", "q2", "q3", "q4")

    def __init__(self, q1: float, q2: float, q3: float, q4: float) -> None:
        """Store the four components of the quaternion."""
        self.q1 = float(q1)
        self.q2 = float(q2)
        self.q3 = float(q3)
        self.q4 = float(q4)

    def is_nan(self) -> bool:
        """Whether any component is NaN.

        Mirrors ``AP_Math/quaternion.h:54-57``::

            bool is_nan(void) const WARN_IF_UNUSED
            {
                return isnan(q1) || isnan(q2) || isnan(q3) || isnan(q4);
            }
        """
        return (
            math.isnan(self.q1)
            or math.isnan(self.q2)
            or math.isnan(self.q3)
            or math.isnan(self.q4)
        )

    def to_euler(self) -> tuple[float, float, float]:
        """Return the roll, pitch and yaw of the quaternion, in radians.

        Mirrors ``AP_Math/quaternion.cpp:608-621`` and the ``get_euler_*``
        helpers it calls.  Only the yaw is of interest for the ODOMETRY
        chain, but the whole triple is returned for completeness.

        Returns:
            The ``(roll, pitch, yaw)`` triple, in radians.
        """
        # get_euler_roll()
        roll = math.atan2(
            2.0 * (self.q1 * self.q2 + self.q3 * self.q4),
            1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3),
        )
        # get_euler_pitch()
        sin_pitch = 2.0 * (self.q1 * self.q3 - self.q4 * self.q2)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)
        # get_euler_yaw()
        yaw = math.atan2(
            2.0 * (self.q1 * self.q4 + self.q2 * self.q3),
            1.0 - 2.0 * (self.q2 * self.q2 + self.q4 * self.q4),
        )
        return (roll, pitch, yaw)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        """Return a readable representation of the quaternion."""
        return (
            f"Quaternion(q1={self.q1!r}, q2={self.q2!r}, "
            f"q3={self.q3!r}, q4={self.q4!r})"
        )


# --------------------------------------------------------------------------
# EKF3 state
# --------------------------------------------------------------------------


@dataclass
class ExtNavSample:
    """One external navigation position sample, as the EKF stores it.

    Mirrors ``ext_nav_elements`` in ``AP_NavEKF3/AP_NavEKF3_core.h``.
    """

    pos: tuple[float, float, float]
    pos_err: float
    time_ms: int = 0
    pos_reset: bool = False


@dataclass
class YawSample:
    """One external navigation yaw sample, as the EKF stores it.

    Mirrors ``yaw_elements`` in ``AP_NavEKF3/AP_NavEKF3_core.h``.
    """

    yaw_ang: float
    yaw_ang_err: float
    time_ms: int = 0


@dataclass
class EKF3State:
    """The parts of the EKF3 state that the ODOMETRY chain touches.

    Attributes:
        stored_ext_nav: The external navigation position buffer.
        stored_ext_nav_yaw_ang: The external navigation yaw buffer.
        yaw_source_last: The current yaw source, as an integer.  ``6`` is
            ``AP_NavEKF_Source::SourceYaw::EXTNAV``.
        yaw_aligned: Whether the yaw alignment has completed.
        euler_yaw: The EKF's current yaw estimate, in radians.
        fused_yaw_observations: The yaw observations that were fused, so
            that a test can tell "fused a yaw of 0" from "never fused a
            yaw".
    """

    stored_ext_nav: list[ExtNavSample] = field(default_factory=list)
    stored_ext_nav_yaw_ang: list[YawSample] = field(default_factory=list)
    yaw_source_last: int = 0
    yaw_aligned: bool = False
    euler_yaw: float = 0.0
    fused_yaw_observations: list[float] = field(default_factory=list)


#: ``AP_NavEKF_Source::SourceYaw::EXTNAV`` (AP_NavEKF_Source.h).
SOURCE_YAW_EXTNAV = 6

#: ``AP_NavEKF_Source::SourceYaw::COMPASS`` (AP_NavEKF_Source.h).
SOURCE_YAW_COMPASS = 1

#: ``VISO_POS_M_NSE`` — the default position noise floor, in metres
#: (``AP_VisualOdom.cpp:99``).
DEFAULT_POS_NOISE_M = 0.2

#: ``VISO_YAW_M_NSE`` — the default yaw noise floor, in radians
#: (``AP_VisualOdom.cpp:107``).
DEFAULT_YAW_NOISE_RAD = 0.2


def _constrain_float(value: float, low: float, high: float) -> float:
    """``AP_Math::constrain_float()``."""
    if value < low:
        return low
    if value > high:
        return high
    return value


# --------------------------------------------------------------------------
# The chain
# --------------------------------------------------------------------------


def handle_odometry(
    q: Quaternion,
    x: float,
    y: float,
    z: float,
    *,
    pos_err: float,
    ang_err: float,
    vel: tuple[float, float, float] = (float("nan"),) * 3,
    pos_noise: float = DEFAULT_POS_NOISE_M,
    yaw_noise: float = DEFAULT_YAW_NOISE_RAD,
) -> tuple[ExtNavSample, YawSample | None, tuple[float, float, float] | None]:
    """Model the handling of an ODOMETRY message by ArduPilot.

    ArduPilot 4.6.3, ``libraries/GCS_MAVLink/GCS_Common.cpp:3876-3908``::

        Quaternion q{m.q[0],m.q[1],m.q[2],m.q[3]};
        ...
        visual_odom->handle_pose_estimate(m.time_usec, timestamp_ms,
                                          m.x, m.y, m.z, q,
                                          posErr, angErr,
                                          m.reset_counter, m.quality);

        // convert velocity vector from FRD to NED frame
        Vector3f vel{m.vx, m.vy, m.vz};
        vel = q * vel;

    The velocity rotation is modelled too, because a ``NaN`` quaternion
    turns a finite velocity into ``NaN`` — which is worth knowing, but
    harmless here because the UWB link sends no velocity at all.

    Args:
        q: The attitude quaternion from the message.
        x: North position, in metres.
        y: East position, in metres.
        z: Down position, in metres.
        pos_err: The derived position error, in metres.
        ang_err: The derived angle error, in radians.
        vel: The body FRD velocity, in m/s.
        pos_noise: ``VISO_POS_M_NSE``, in metres.
        yaw_noise: ``VISO_YAW_M_NSE``, in radians.

    Returns:
        A ``(position_sample, yaw_sample, rotated_velocity)`` triple.  The
        yaw sample is ``None`` when the quaternion was rejected.
    """
    # AP_VisualOdom_MAV::handle_pose_estimate() clamps the errors before
    # handing them on:
    #
    #   posErr = constrain_float(posErr, _frontend.get_pos_noise(), 100.0f);
    #   angErr = constrain_float(angErr, _frontend.get_yaw_noise(), 1.5f);
    pos_err = _constrain_float(pos_err, pos_noise, 100.0)
    ang_err = _constrain_float(ang_err, yaw_noise, 1.5)

    position, yaw = write_ext_nav_data(
        q, x, y, z, pos_err=pos_err, ang_err=ang_err
    )

    # The velocity is rotated by the quaternion, exactly as upstream does.
    # A NaN quaternion therefore makes the velocity NaN as well.
    rotated = tuple(_rotate_by_quaternion(q, vel[0], vel[1], vel[2]))

    return (position, yaw, rotated)


def _rotate_by_quaternion(
    q: Quaternion, vx: float, vy: float, vz: float
) -> tuple[float, float, float]:
    """Rotate a vector by a quaternion.

    Mirrors ``AP_Math/quaternion.cpp:731-752``.  The formula is inlined, as
    it is upstream.
    """
    uv = (
        q.q3 * vz - q.q4 * vy,
        q.q4 * vx - q.q2 * vz,
        q.q2 * vy - q.q3 * vx,
    )
    uv = (2.0 * uv[0], 2.0 * uv[1], 2.0 * uv[2])
    rx = vx + q.q1 * uv[0] + q.q3 * uv[2] - q.q4 * uv[1]
    ry = vy + q.q1 * uv[1] + q.q4 * uv[0] - q.q2 * uv[2]
    rz = vz + q.q1 * uv[2] + q.q2 * uv[1] - q.q3 * uv[0]
    return (rx, ry, rz)


def write_ext_nav_data(
    q: Quaternion,
    x: float,
    y: float,
    z: float,
    *,
    pos_err: float,
    ang_err: float,
) -> tuple[ExtNavSample, YawSample | None]:
    """Model ``NavEKF3_core::writeExtNavData()``.

    ArduPilot 4.6.3, ``libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:1057-1109``::

        // protect against NaN
        if (pos.is_nan() || isnan(posErr)) {
            return;
        }
        ...
        extNavDataNew.pos = pos.toftype();
        extNavDataNew.posErr = posErr;
        ...
        storedExtNav.push(extNavDataNew);

        // protect against attitude or angle being NaN
        if (!quat.is_nan() && !isnan(angErr)) {
            ftype roll_rad, pitch_rad, yaw_rad;
            quat.to_euler(roll_rad, pitch_rad, yaw_rad);
            yaw_elements extNavYawAngDataNew;
            extNavYawAngDataNew.yawAng = yaw_rad;
            extNavYawAngDataNew.yawAngErr = MAX(angErr, radians(5.0f));
            ...
            storedExtNavYawAng.push(extNavYawAngDataNew);
        }

    This is the crux of the whole matter: **the position is pushed
    unconditionally, the yaw only when the quaternion is not NaN.**

    Args:
        q: The attitude quaternion.
        x: North position, in metres.
        y: East position, in metres.
        z: Down position, in metres.
        pos_err: The derived position error, in metres.
        ang_err: The derived angle error, in radians.

    Returns:
        A ``(position_sample, yaw_sample)`` pair.  The yaw sample is
        ``None`` when the quaternion was rejected as NaN.
    """
    if math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(pos_err):
        raise ValueError("writeExtNavData() rejects a NaN position")

    position = ExtNavSample(pos=(x, y, z), pos_err=pos_err)

    if not q.is_nan() and not math.isnan(ang_err):
        _roll, _pitch, yaw = q.to_euler()
        # MAX(angErr, radians(5.0f)) — upstream never trusts a yaw that is
        # better than 5 degrees.
        yaw_sample = YawSample(
            yaw_ang=yaw, yaw_ang_err=max(ang_err, math.radians(5.0))
        )
    else:
        yaw_sample = None

    return (position, yaw_sample)


def select_mag_fusion(
    state: EKF3State,
    *,
    tilt_align_complete: bool = True,
) -> list[float]:
    """Model the external navigation branch of ``SelectMagFusion()``.

    ArduPilot 4.6.3, ``libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp:367-392``::

        const bool extNavYawDataToFuse =
            storedExtNavYawAng.recall(extNavYawAngDataDelayed,
                                      imuDataDelayed.time_ms);
        if (yaw_source_last == AP_NavEKF_Source::SourceYaw::EXTNAV) {
            if (extNavYawDataToFuse) {
                if (tiltAlignComplete &&
                    (!yawAlignComplete || yaw_source_reset)) {
                    alignYawAngle(extNavYawAngDataDelayed);
                    yaw_source_reset = false;
                } else if (tiltAlignComplete && yawAlignComplete) {
                    fuseEulerYaw(yawFusionMethod::EXTNAV);
                }
            }
        }

    The important behaviour is the *first* yaw sample: it goes through
    ``alignYawAngle()``, which **resets** the EKF yaw to the sample value
    (``resetQuatStateYawOnly``).  Every later sample is fused continuously.

    Args:
        state: The EKF state to update.
        tilt_align_complete: Whether the tilt alignment has completed.

    Returns:
        The list of yaw observations that were fused, in radians.
    """
    if state.yaw_source_last != SOURCE_YAW_EXTNAV:
        # The external navigation yaw is ignored entirely when another
        # source is selected — but the samples are still buffered.
        return []

    if not state.stored_ext_nav_yaw_ang:
        # No yaw data at all: upstream falls back to fusing a 'fake' yaw
        # observation to keep the filter stable.  That path does not inject
        # a zero, so it is not modelled here.
        return []

    sample = state.stored_ext_nav_yaw_ang[-1]

    if tilt_align_complete and not state.yaw_aligned:
        # alignYawAngle(): the EKF yaw is RESET to the sample value.
        state.euler_yaw = sample.yaw_ang
        state.yaw_aligned = True
        state.fused_yaw_observations.append(sample.yaw_ang)
    elif tilt_align_complete and state.yaw_aligned:
        # fuseEulerYaw(EXTNAV): the sample is fused into the estimate.
        state.euler_yaw = sample.yaw_ang
        state.fused_yaw_observations.append(sample.yaw_ang)

    return list(state.fused_yaw_observations)
