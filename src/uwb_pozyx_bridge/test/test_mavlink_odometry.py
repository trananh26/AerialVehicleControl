"""Unit tests for the MAVLink ODOMETRY encoder.

These tests do not import ROS, so they can run without a sourced workspace.

The frame conventions asserted here mirror ArduPilot's own source:

* ``GCS_MAVLINK::handle_odometry()`` rejects the message unless
  ``frame_id == MAV_FRAME_LOCAL_FRD`` and
  ``child_frame_id == MAV_FRAME_BODY_FRD``.
* The position is fused into the EKF unchanged, and the EKF works in NED.
* The velocities are interpreted in the body FRD frame.
* The measurement noise is derived from the covariance matrix.
"""

import pytest

from uwb_pozyx_bridge.mavlink_odometry import (
    MAV_CMD_DO_SET_HOME,
    MAV_CMD_REQUEST_MESSAGE,
    MAV_ESTIMATOR_TYPE_VIO,
    MAV_FRAME_BODY_FRD,
    MAV_FRAME_LOCAL_FRD,
    ODOMETRY_MSG_ID,
    SET_GPS_GLOBAL_ORIGIN_MSG_ID,
    build_do_set_home_message,
    build_odometry_message,
    build_request_message_command,
    build_set_gps_global_origin_message,
    enu_to_ned_m,
    pose_covariance_1d,
)

# ------------------------------------------------------------ frame ids


def test_frame_ids_are_the_ones_ardupilot_requires():
    """ArduPilot hard-rejects anything else, so these must not drift."""
    # MAV_FRAME_LOCAL_FRD and MAV_FRAME_BODY_FRD in common.xml.
    assert MAV_FRAME_LOCAL_FRD == 20
    assert MAV_FRAME_BODY_FRD == 12


def test_message_uses_the_required_frames():
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)

    assert msg.frame_id == MAV_FRAME_LOCAL_FRD
    assert msg.child_frame_id == MAV_FRAME_BODY_FRD


def test_message_id_is_odometry():
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)

    assert msg._header.msgId == ODOMETRY_MSG_ID
    assert ODOMETRY_MSG_ID == 331


def test_estimator_type_is_vio():
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)

    assert msg.estimator_type == MAV_ESTIMATOR_TYPE_VIO


# ------------------------------------------------------- frame conversion


def test_enu_to_ned_swaps_the_horizontal_axes():
    """Gazebo is East-first, ArduPilot's EKF is North-first."""
    north_m, east_m, down_m = enu_to_ned_m(2.0, -1.0, 0.5)

    assert north_m == pytest.approx(-1.0)
    assert east_m == pytest.approx(2.0)
    assert down_m == pytest.approx(-0.5)


def test_enu_to_ned_is_not_the_identity():
    """A point off the diagonal must not survive the conversion unchanged."""
    north_m, east_m, _ = enu_to_ned_m(2.0, -1.0, 0.0)

    assert north_m != 2.0
    assert east_m != -1.0


def test_enu_to_ned_conversion_is_a_reflection():
    """Swapping two axes has determinant -1; no rotation can undo it.

    This is why a yaw error cannot compensate for the swap: the whole
    navigation system would be mirrored across the diagonal.
    """
    a = enu_to_ned_m(2.0, -1.0, 0.0)
    b = enu_to_ned_m(-0.5, 3.0, 0.0)

    # After the swap the two points are on the other side of the diagonal.
    assert (a[0] > a[1]) != (b[0] > b[1])


def test_position_reaches_the_ekf_in_ned():
    """The message carries the NED values, not the ENU ones."""
    east_m, north_m, up_m = 1.5, -2.5, 0.75
    north, east, down = enu_to_ned_m(east_m, north_m, up_m)

    msg = build_odometry_message(north, east, down, 1_000_000)

    assert msg.x == pytest.approx(north_m)
    assert msg.y == pytest.approx(east_m)
    assert msg.z == pytest.approx(-up_m)


# ------------------------------------------------------------ covariance


def test_covariance_holds_the_variances():
    """ArduPilot reads the diagonal entries as variances, not deviations."""
    cov = pose_covariance_1d(0.01, 0.04, 0.09)

    assert len(cov) == 21
    assert cov[0] == pytest.approx(0.01)
    assert cov[6] == pytest.approx(0.04)
    assert cov[11] == pytest.approx(0.09)


def test_covariance_is_zero_elsewhere():
    cov = pose_covariance_1d(0.01, 0.01, 0.01)

    for index, value in enumerate(cov):
        if index not in (0, 6, 11):
            assert value == 0.0


def test_standard_deviation_is_squared_into_the_covariance():
    msg = build_odometry_message(
        0.0, 0.0, 0.0, 1_000_000, position_std_m=0.05
    )

    assert msg.pose_covariance[0] == pytest.approx(0.05 * 0.05)
    assert msg.pose_covariance[6] == pytest.approx(0.05 * 0.05)
    assert msg.pose_covariance[11] == pytest.approx(0.05 * 0.05)


def test_position_error_ardupilot_derives_is_not_the_std():
    """Reproduce the error ArduPilot computes from the covariance.

    ArduPilot computes ``posErr = cbrtf(sq(cov[0]) + sq(cov[6]) + sq(cov[11]))``.
    Because the covariance entries are already variances, this squares them a
    second time, so the result is ``sigma ** (4/3) * 3 ** (1/3)`` rather
    than ``sigma``.  For any ``sigma`` below 1 m the derived error is
    *smaller* than the real standard deviation, i.e. ArduPilot would be
    over-confident about the fix.

    This is why ``VISO_POS_M_NSE`` matters: it is a lower bound on
    ``posErr``, so it is what actually determines the EKF measurement
    variance for a well-behaved UWB solution.
    """
    sigma = 0.05
    msg = build_odometry_message(0.0, 0.0, 0.0, 1_000_000, position_std_m=sigma)

    cov = msg.pose_covariance
    pos_err = (cov[0] ** 2 + cov[6] ** 2 + cov[11] ** 2) ** (1 / 3)

    assert pos_err == pytest.approx(sigma ** (4 / 3) * 3 ** (1 / 3))
    assert pos_err < sigma


def test_position_error_is_bounded_by_the_noise_floor():
    """``VISO_POS_M_NSE`` clamps the derived error from below.

    ``AP_VisualOdom_MAV::handle_pose_estimate()`` does
    ``posErr = constrain_float(posErr, get_pos_noise(), 100.0f)``, so the
    measurement variance the EKF uses is at least
    ``VISO_POS_M_NSE ** 2``.
    """
    # Reproduce the upstream clamp.
    floor = 0.05
    sigma = 0.05
    variance = sigma * sigma
    derived = (variance**2 * 3) ** (1 / 3)
    clamped = min(max(derived, floor), 100.0)

    assert derived < floor
    assert clamped == pytest.approx(floor)


# ------------------------------------------------------------- velocities


def test_velocities_default_to_zero():
    """The node sends position only, so no velocity is fused."""
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)

    assert msg.vx == 0.0
    assert msg.vy == 0.0
    assert msg.vz == 0.0
    assert msg.rollspeed == 0.0
    assert msg.pitchspeed == 0.0
    assert msg.yawspeed == 0.0


def test_velocity_covariance_is_zero():
    msg = build_odometry_message(1.0, 2.0, -0.5, 1_000_000)

    assert all(value == 0.0 for value in msg.velocity_covariance)


# ------------------------------------------------------------- timestamps


def test_timestamp_is_passed_through():
    msg = build_odometry_message(0.0, 0.0, 0.0, 1_700_000_000_123_456)

    assert msg.time_usec == 1_700_000_000_123_456


def test_reset_counter_is_carried():
    msg = build_odometry_message(0.0, 0.0, 0.0, 1_000_000, reset_counter=7)

    assert msg.reset_counter == 7


def test_quality_is_carried():
    msg = build_odometry_message(0.0, 0.0, 0.0, 1_000_000, quality=42)

    assert msg.quality == 42


# ------------------------------------------------- EKF origin declaration


def test_origin_message_id_is_set_gps_global_origin():
    msg = build_set_gps_global_origin_message(-35.363262, 149.165237, 584.0)

    assert msg._header.msgId == SET_GPS_GLOBAL_ORIGIN_MSG_ID
    assert SET_GPS_GLOBAL_ORIGIN_MSG_ID == 48


def test_origin_latitude_is_in_degrees_times_1e7():
    """The MAVLink field is an int32 in units of 1e-7 degrees."""
    msg = build_set_gps_global_origin_message(-35.363262, 149.165237, 584.0)

    assert msg.latitude == pytest.approx(-353632620, abs=1)
    assert msg.longitude == pytest.approx(1491652370, abs=1)


def test_origin_altitude_is_in_millimetres():
    """The MAVLink field is in mm; ArduPilot divides it by ten for cm."""
    msg = build_set_gps_global_origin_message(-35.363262, 149.165237, 584.0)

    assert msg.altitude == 584000


def test_origin_targets_the_autopilot():
    msg = build_set_gps_global_origin_message(
        -35.363262, 149.165237, 584.0, target_system=1
    )

    assert msg.target_system == 1


# -------------------------------------------------- home position command


def test_home_command_is_do_set_home():
    msg = build_do_set_home_message(-35.363262, 149.165237, 584.0)

    assert msg.command == MAV_CMD_DO_SET_HOME
    assert MAV_CMD_DO_SET_HOME == 179


def test_home_command_carries_the_position_in_param5_to_7():
    """COMMAND_LONG conveys the location in param5/6/7.

    ArduPilot's ``convert_COMMAND_LONG_to_COMMAND_INT()`` moves param5 and
    param6 into the integer x/y fields (scaled by 1e7) and param7 into z, and
    it uses ``MAV_FRAME_GLOBAL`` for this command.
    """
    msg = build_do_set_home_message(-35.363262, 149.165237, 584.0)

    assert msg.param5 == pytest.approx(-35.363262)
    assert msg.param6 == pytest.approx(149.165237)
    assert msg.param7 == pytest.approx(584.0)


def test_home_command_does_not_use_the_current_location():
    """param1 == 0 means "use the given position", not "use the current one".

    ArduPilot's handler branches on ``is_equal(packet.param1, 1.0f)``, so a
    param1 of 1 would ask the autopilot to use its own position instead.
    """
    msg = build_do_set_home_message(-35.363262, 149.165237, 584.0)

    assert msg.param1 == 0.0


def test_home_command_targets_the_autopilot():
    msg = build_do_set_home_message(
        -35.363262, 149.165237, 584.0, target_system=1, target_component=1
    )

    assert msg.target_system == 1
    assert msg.target_component == 1


# --------------------------------------------------- message request command


def test_request_message_command_is_request_message():
    msg = build_request_message_command(242)

    assert msg.command == MAV_CMD_REQUEST_MESSAGE
    assert MAV_CMD_REQUEST_MESSAGE == 512


def test_request_message_command_carries_the_message_id():
    msg = build_request_message_command(242)

    assert msg.param1 == pytest.approx(242.0)
