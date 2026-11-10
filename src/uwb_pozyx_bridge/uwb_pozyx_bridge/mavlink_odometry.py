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
sent as ``NaN``, which is how the MAVLink specification expresses "no
velocity estimate".  ArduPilot rejects such a sample in
``NavEKF3_core::writeExtNavVelData()``, so the EKF keeps integrating the
IMU instead of fusing a bogus zero velocity.

Covariance
----------
ArduPilot derives the measurement noise from the covariance matrix rather
than from a dedicated accuracy field::

    posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));

Only the diagonal entries ``[0]``, ``[6]`` and ``[11]`` (the position part of
the 6x6 pose covariance) are used; they hold *variances*, so they are the
square of the standard deviation.

The velocity covariance is not read by ArduPilot at all.  It is sent as
``NaN`` in the first element, per the MAVLink specification for an unknown
velocity, which is also what the ESP32 tag does.
"""

from __future__ import annotations

from typing import Sequence

__all__ = [
    "MAV_FRAME_LOCAL_FRD",
    "MAV_FRAME_BODY_FRD",
    "MAV_ESTIMATOR_TYPE_NAIVE",
    "ODOMETRY_MSG_ID",
    "SET_GPS_GLOBAL_ORIGIN_MSG_ID",
    "HOME_POSITION_MSG_ID",
    "MAV_CMD_DO_SET_HOME",
    "MAV_CMD_REQUEST_MESSAGE",
    "enu_to_ned_m",
    "pose_covariance_1d",
    "velocity_covariance_unknown",
    "build_odometry_message",
    "build_set_gps_global_origin_message",
    "build_do_set_home_message",
    "build_request_message_command",
]

#: ``MAV_FRAME_LOCAL_FRD``.  ArduPilot requires this exact value for the
#: parent frame of an ODOMETRY message.
MAV_FRAME_LOCAL_FRD = 20

#: ``MAV_FRAME_BODY_FRD``.  ArduPilot requires this exact value for the
#: child frame of an ODOMETRY message.
MAV_FRAME_BODY_FRD = 12

#: ``MAV_ESTIMATOR_TYPE_NAIVE``.  ArduPilot ignores the estimator type
#: entirely, but both ends of the link have to send the same value so that
#: the two implementations can be compared byte for byte.
#:
#: MAVLink 2.x has no ``MAV_ESTIMATOR_TYPE_UWB``.  The official enum is
#: ``UNKNOWN = 0``, ``NAIVE = 1``, ``VISION = 2``, ``VIO = 3``, ``GPS = 4``,
#: ``GPS_INS = 5``, ``MOCAP = 6``, ``LIDAR = 7``, ``AUTOPILOT = 8``.  A
#: trilateration solver is a *naive* estimator — it produces a position
#: from ranges without any covariance feedback — so ``NAIVE`` is the
#: honest choice.
#:
#: Note that the ESP32 tag used to send ``MAV_ESTIMATOR_TYPE_NAIVE`` as
#: well, so the two sides now agree.
MAV_ESTIMATOR_TYPE_NAIVE = 1

#: The MAVLink message id of ODOMETRY.
ODOMETRY_MSG_ID = 331

#: The MAVLink message id of SET_GPS_GLOBAL_ORIGIN.
SET_GPS_GLOBAL_ORIGIN_MSG_ID = 48

#: The MAVLink message id of HOME_POSITION.  The autopilot only sends it
#: once the home position is known, so receiving it is the confirmation that
#: ``MAV_CMD_DO_SET_HOME`` was accepted.
HOME_POSITION_MSG_ID = 242

#: ``MAV_CMD_DO_SET_HOME``.  This is the only way to establish the home
#: position on a vehicle that has no GPS.
MAV_CMD_DO_SET_HOME = 179

#: ``MAV_CMD_REQUEST_MESSAGE``.  Used to ask the autopilot to re-send
#: ``HOME_POSITION`` so that the node can verify the home position it set.
MAV_CMD_REQUEST_MESSAGE = 512


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


def velocity_covariance_unknown() -> list[float]:
    """Build the velocity covariance for "no velocity estimate".

    The MAVLink specification says:

        *If the velocity is unknown, assign NaN to the first element of the
        covariance matrix.*

    The UWB ranging data carries no velocity estimate at all, so the
    honest thing to send is ``NaN``.  This also matches the ESP32 tag,
    which sends the same thing.

    ArduPilot does not read the velocity covariance at all, so this field
    is purely informational — but it is the difference between "the
    velocities are zero" and "there is no velocity estimate", and the
    latter is what is true.

    Returns:
        A 21-element list whose first entry is ``NaN``.
    """
    cov = [0.0] * 21
    cov[0] = float("nan")
    return cov


def build_odometry_message(
    north_m: float,
    east_m: float,
    down_m: float,
    time_usec: int,
    *,
    position_std_m: float = 0.05,
    velocity_ned: Sequence[float] = (float("nan"),) * 3,
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
        velocity_ned: Linear velocity in m/s, in the body FRD frame.
            ArduPilot rotates these into NED itself.  The default is
            ``NaN``, which is how "no velocity estimate" is expressed:
            ``NavEKF3_core::writeExtNavVelData()`` rejects the whole sample
            when the velocity is ``NaN``, so the EKF keeps integrating the
            IMU instead of fusing a bogus zero velocity.
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
        velocity_covariance=velocity_covariance_unknown(),
        reset_counter=int(reset_counter),
        estimator_type=MAV_ESTIMATOR_TYPE_NAIVE,
        quality=int(quality),
    )


def build_set_gps_global_origin_message(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    target_system: int = 1,
):
    """Build a ``MAVLink_set_gps_global_origin_message``.

    ArduPilot needs a *global* reference for its EKF origin, even when the
    position itself comes from an external navigation system.  Without an
    origin:

    * ``AP_AHRS::get_origin()`` fails, so the EKF cannot convert its height
      between the "above origin" and "above home" frames.
    * ``ModeGuided::do_user_takeoff_start()`` calls
      ``Location::get_alt_cm(ABOVE_ORIGIN)``, which fails, so
      ``MAV_CMD_NAV_TAKEOFF`` is answered with ``MAV_RESULT_FAILED`` and the
      vehicle never leaves the ground.

    The message is handled by ``GCS_MAVLINK::handle_set_gps_global_origin()``,
    which calls ``GCS_MAVLINK::set_ekf_origin()``.  That helper is
    deliberately the only path that sets the origin without a GPS: it is
    documented as "should only be used when there is no GPS to provide an
    absolute position".

    The message has to arrive from the GCS system id (``SYSID_MYGCS``, 255
    by default), otherwise ``GCS_MAVLINK::accept_packet()`` drops it.

    Args:
        latitude_deg: Latitude of the origin, in degrees.
        longitude_deg: Longitude of the origin, in degrees.
        altitude_m: Altitude of the origin above mean sea level, in metres.
            The MAVLink field is in millimetres; ArduPilot divides it by ten
            to obtain centimetres.
        target_system: MAVLink system id of the autopilot.

    Returns:
        A ``pymavlink`` ``MAVLink_set_gps_global_origin_message`` instance.
    """
    from pymavlink.dialects.v20 import common as mavlink

    return mavlink.MAVLink_set_gps_global_origin_message(
        target_system=int(target_system),
        latitude=int(round(float(latitude_deg) * 1e7)),
        longitude=int(round(float(longitude_deg) * 1e7)),
        altitude=int(round(float(altitude_m) * 1000)),
    )


def build_do_set_home_message(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    target_system: int = 1,
    target_component: int = 1,
):
    """Build a ``MAVLink_command_long_message`` that sets the home position.

    The EKF origin is not enough for a guided take-off.  ArduPilot's
    ``ModeGuided::do_user_takeoff_start()`` interprets the requested altitude
    as *above home* and then converts it to *above origin*::

        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            return false;
        }

    ``Location::get_alt_cm()`` needs the home altitude to perform that
    conversion, so it returns false when the home position is unknown and
    the take-off command is answered with ``MAV_RESULT_FAILED``.  This is
    why the vehicle arms but never leaves the ground.

    ``MAV_CMD_DO_SET_HOME`` is the documented way to establish the home
    position on a vehicle without a GPS.  ArduPilot's handler
    (``GCS_MAVLINK::handle_command_do_set_home()``) checks that the EKF
    origin is already set before it accepts the command, so the origin has
    to be declared first.

    The command is conveyed as ``COMMAND_LONG`` here, which means the
    latitude and longitude go into ``param5``/``param6`` as degrees and the
    altitude into ``param7`` as metres.  ArduPilot converts the command to a
    ``COMMAND_INT`` with ``MAV_FRAME_GLOBAL``, which is the frame the
    handler expects.

    Args:
        latitude_deg: Latitude of the home position, in degrees.
        longitude_deg: Longitude of the home position, in degrees.
        altitude_m: Altitude of the home position above mean sea level, in
            metres.
        target_system: MAVLink system id of the autopilot.
        target_component: MAVLink component id of the autopilot.

    Returns:
        A ``pymavlink`` ``MAVLink_command_long_message`` instance.
    """
    from pymavlink.dialects.v20 import common as mavlink

    return mavlink.MAVLink_command_long_message(
        target_system=int(target_system),
        target_component=int(target_component),
        command=MAV_CMD_DO_SET_HOME,
        confirmation=0,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=float(latitude_deg),
        param6=float(longitude_deg),
        param7=float(altitude_m),
    )


def build_request_message_command(
    message_id: int,
    target_system: int = 1,
    target_component: int = 1,
):
    """Build a ``MAVLink_command_long_message`` that requests a message.

    Used to ask the autopilot to re-send ``HOME_POSITION`` so that the node
    can confirm that the home position it set was actually accepted.

    Args:
        message_id: The MAVLink message id to request.
        target_system: MAVLink system id of the autopilot.
        target_component: MAVLink component id of the autopilot.

    Returns:
        A ``pymavlink`` ``MAVLink_command_long_message`` instance.
    """
    from pymavlink.dialects.v20 import common as mavlink

    return mavlink.MAVLink_command_long_message(
        target_system=int(target_system),
        target_component=int(target_component),
        command=MAV_CMD_REQUEST_MESSAGE,
        confirmation=0,
        param1=float(message_id),
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    )
