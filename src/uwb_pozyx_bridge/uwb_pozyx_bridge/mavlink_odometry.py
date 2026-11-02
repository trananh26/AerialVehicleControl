"""MAVLink ``ODOMETRY`` (id 331) encoder for the UWB external navigation link.

The UWB trilateration result is handed to ArduPilot as a MAVLink
``ODOMETRY`` message over a serial port.  ArduPilot's
``GCS_MAVLINK::handle_odometry()`` forwards it to ``AP_VisualOdom``, which
feeds the position into EKF3 as an *external navigation* (EXTNAV)
measurement.

Two upstream requirements shape this module.

Frame ids
---------
ArduPilot drops the message unless both frame ids match::

    if (m.frame_id != MAV_FRAME_LOCAL_FRD ||
        m.child_frame_id != MAV_FRAME_BODY_FRD) {
        // only support local FRD frame data
        return;
    }

so :data:`MAV_FRAME_LOCAL_FRD` and :data:`MAV_FRAME_BODY_FRD` are hard-coded
here and are not configurable.

Position convention
-------------------
The position fields of an ``ODOMETRY`` message are *not* transformed by
ArduPilot; they are fused into the EKF as they arrive, and the EKF works in
NED.  ArduPilot's own ``SIM_Vicon`` backend makes the same choice: it labels
its north/east/down values as ``MAV_FRAME_LOCAL_FRD``.

The Gazebo world is ENU (x east, y north, z up), so the horizontal axes have
to be swapped and the vertical axis flipped.  :func:`enu_to_ned_m` performs
that conversion.

Velocity convention
-------------------
The velocity fields *are* transformed by ArduPilot::

    // convert velocity vector from FRD to NED frame
    Vector3f vel{m.vx, m.vy, m.vz};
    vel = q * vel;

so ``vx``/``vy``/``vz`` are expected in the body FRD frame (x forward,
y right, z down).  This node only publishes positions, so the velocities are
left at zero and the EKF velocity source stays disabled.

Covariance
----------
ArduPilot derives the measurement noise from the covariance matrix rather
than from a dedicated accuracy field::

    posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));

Only the diagonal entries ``[0]``, ``[6]`` and ``[11]`` (the position part of
the 6x6 pose covariance) are used; they hold *variances*, so they are the
square of the standard deviation.
"""

from __future__ import annotations

from typing import Sequence

__all__ = [
    "MAV_FRAME_LOCAL_FRD",
    "MAV_FRAME_BODY_FRD",
    "MAV_ESTIMATOR_TYPE_VIO",
    "ODOMETRY_MSG_ID",
    "enu_to_ned_m",
    "pose_covariance_1d",
    "build_odometry_message",
]

#: ``MAV_FRAME_LOCAL_FRD``.  ArduPilot requires this exact value for the
#: parent frame of an ODOMETRY message.
MAV_FRAME_LOCAL_FRD = 20

#: ``MAV_FRAME_BODY_FRD``.  ArduPilot requires this exact value for the
#: child frame of an ODOMETRY message.
MAV_FRAME_BODY_FRD = 12

#: ``MAV_ESTIMATOR_TYPE_VIO``.  ArduPilot ignores the estimator type, but
#: VIO is the honest description of the source.
MAV_ESTIMATOR_TYPE_VIO = 2

#: The MAVLink message id of ODOMETRY.
ODOMETRY_MSG_ID = 331


def enu_to_ned_m(
    east_m: float, north_m: float, up_m: float
) -> tuple[float, float, float]:
    """Convert a Gazebo ENU position into the NED metres ArduPilot expects.

    Args:
        east_m: East offset in metres.
        north_m: North offset in metres.
        up_m: Up offset in metres.

    Returns:
        The ``(north_m, east_m, down_m)`` triple.

    Example:
        >>> enu_to_ned_m(2.0, -1.0, 0.5)
        (-1.0, 2.0, -0.5)
    """
    return (float(north_m), float(east_m), -float(up_m))


def pose_covariance_1d(
    variance_x: float, variance_y: float, variance_z: float
) -> list[float]:
    """Build the diagonal of a 6x6 pose covariance for the position part.

    The MAVLink field is a 21-element array holding the upper triangle of a
    6x6 row-major matrix.  ArduPilot only reads elements ``[0]``, ``[6]`` and
    ``[11]``, which are the ``(x, x)``, ``(y, y)`` and ``(z, z)`` entries of
    the position block.

    Args:
        variance_x: Position variance along x, in m^2.
        variance_y: Position variance along y, in m^2.
        variance_z: Position variance along z, in m^2.

    Returns:
        A 21-element list with the three variances in place and zeros
        elsewhere.
    """
    cov = [0.0] * 21
    cov[0] = float(variance_x)
    cov[6] = float(variance_y)
    cov[11] = float(variance_z)
    return cov


def build_odometry_message(
    north_m: float,
    east_m: float,
    down_m: float,
    time_usec: int,
    *,
    position_std_m: float = 0.05,
    velocity_ned: Sequence[float] = (0.0, 0.0, 0.0),
    angular_rate: Sequence[float] = (0.0, 0.0, 0.0),
    quaternion: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    quality: int = 50,
    reset_counter: int = 0,
):
    """Build a ``MAVLink_odometry_message`` for the given NED position.

    Args:
        north_m: North position in metres.
        east_m: East position in metres.
        down_m: Down position in metres (positive is down).
        time_usec: Timestamp of the fix, in microseconds since the Unix
            epoch.  ArduPilot's jitter correction maps this onto its own
            clock, so wall-clock time is the right thing to send.
        position_std_m: Standard deviation of the position estimate, in
            metres.  It is squared to obtain the covariance entries.
        velocity_ned: Linear velocity in m/s.  ArduPilot interprets these
            as body FRD and rotates them into NED itself, so leave this at
            zero unless a real body-frame velocity estimate is available.
        angular_rate: Angular rate in rad/s, in the same body FRD frame as
            the velocity.
        quaternion: Body-to-earth attitude quaternion, ``(w, x, y, z)``.
        quality: Quality of the estimate, 0 to 100.
        reset_counter: Incremented when the position jumps discontinuously.

    Returns:
        A ``pymavlink`` ``MAVLink_odometry_message`` instance.
    """
    from pymavlink.dialects.v20 import common as mavlink

    std = float(position_std_m)
    variance = std * std

    return mavlink.MAVLink_odometry_message(
        time_usec=int(time_usec),
        frame_id=MAV_FRAME_LOCAL_FRD,
        child_frame_id=MAV_FRAME_BODY_FRD,
        x=float(north_m),
        y=float(east_m),
        z=float(down_m),
        q=list(quaternion),
        vx=float(velocity_ned[0]),
        vy=float(velocity_ned[1]),
        vz=float(velocity_ned[2]),
        rollspeed=float(angular_rate[0]),
        pitchspeed=float(angular_rate[1]),
        yawspeed=float(angular_rate[2]),
        pose_covariance=pose_covariance_1d(variance, variance, variance),
        velocity_covariance=[0.0] * 21,
        reset_counter=int(reset_counter),
        estimator_type=MAV_ESTIMATOR_TYPE_VIO,
        quality=int(quality),
    )
