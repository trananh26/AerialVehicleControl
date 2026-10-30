#!/usr/bin/env python3

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong


class SmcCircleMission(Node):

    WAIT_CONN     = 0
    SEND_GUIDED   = 1
    WAIT_GUIDED   = 2
    SEND_ARM      = 3
    WAIT_ARM      = 4
    WAIT_STABLE   = 5
    SEND_TAKEOFF  = 6
    WAIT_TAKEOFF  = 7
    CLIMBING      = 8
    FLY_TO_START  = 9
    CIRCLE        = 10
    FLY_TO_CENTER = 11
    SEND_LAND     = 12
    WAIT_LAND     = 13
    WAIT_DISARM   = 14
    DONE          = 15

    def __init__(self):
        super().__init__('smc_circle_mission')

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter('save_log', True)

        self.save_log = (
            self.get_parameter('save_log')
            .get_parameter_value()
            .bool_value
        )

        # ---------------------------------------------------------
        # Circle
        # ---------------------------------------------------------

        self.radius = 3.0
        self.w = 0.3

        # ---------------------------------------------------------
        # Takeoff
        # ---------------------------------------------------------

        self.takeoff_height = 3.0
        self.target_z = 3.0

        # =========================================================
        # PURE VELOCITY-LEVEL SMC
        #
        # Sliding surface:
        #
        #     S = e = xd - x
        #
        # Reaching law:
        #
        #     S_dot = -k1*S - k2*tanh(S/phi)
        #
        # Since:
        #
        #     S_dot = xd_dot - x_dot
        #
        # therefore:
        #
        #     x_dot_cmd =
        #         xd_dot
        #         + k1*S
        #         + k2*tanh(S/phi)
        #
        # IMPORTANT:
        #
        # This controller does NOT use velocity error.
        #
        # There is no:
        #
        #     e_dot = xd_dot - x_dot
        #
        # inside the feedback controller.
        #
        # Therefore this remains a pure SMC
        # at velocity-control level.
        # =========================================================

        # k1: exponential reaching term
        self.k1x = 0.15
        self.k1y = 0.15

        # k2: switching/reaching term
        self.k2x = 0.25
        self.k2y = 0.25

        # Boundary-layer thickness for tanh
        self.phi_x = 0.10
        self.phi_y = 0.10

        # Maximum commanded horizontal velocity
        self.max_velocity = 2.0

        # =========================================================
        # PUBLISHER
        # =========================================================

        self.pub_velocity = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            qos_profile_sensor_data
        )

        # =========================================================
        # SUBSCRIBERS
        # =========================================================

        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pos_callback,
            qos_profile_sensor_data
        )

        self.vel_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.vel_callback,
            qos_profile_sensor_data
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sensor_data
        )

        # =========================================================
        # MAVROS SERVICES
        # =========================================================

        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.command_client = self.create_client(
            CommandLong,
            '/mavros/cmd/command'
        )

        while not self.set_mode_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                'Waiting for /mavros/set_mode...'
            )

        while not self.arming_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                'Waiting for /mavros/cmd/arming...'
            )

        while not self.command_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                'Waiting for /mavros/cmd/command...'
            )

        # =========================================================
        # DRONE STATE
        # =========================================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.current_state = None

        self.have_position = False
        self.have_velocity = False

        # =========================================================
        # CIRCLE CENTER
        # =========================================================

        self.x0 = None
        self.y0 = None

        # =========================================================
        # MISSION TIMING
        # =========================================================

        self.start_time = None
        self.arm_time = None

        self.ARM_STABLE_SEC = 2.0

        # =========================================================
        # STATE MACHINE
        # =========================================================

        self.mission_state = self.WAIT_CONN
        self.pending_future = None

        # =========================================================
        # LOG
        # =========================================================

        self.log_data = []

        # =========================================================
        # TIMER
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        # =========================================================
        # INFO
        # =========================================================

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'SMC Circle Mission started'
        )

        self.get_logger().info(
            'Control: pure velocity-level SMC'
        )

        self.get_logger().info(
            'Sliding surface: S = e = xd - x'
        )

        self.get_logger().info(
            'Reaching law: '
            'S_dot = -k1*S - k2*tanh(S/phi)'
        )

        self.get_logger().info(
            'Control law: '
            'v_cmd = xd_dot + k1*S + k2*tanh(S/phi)'
        )

        self.get_logger().info(
            f'Radius = {self.radius:.2f} m'
        )

        self.get_logger().info(
            f'Angular velocity = {self.w:.2f} rad/s'
        )

        self.get_logger().info(
            f'Tangential velocity = '
            f'{self.radius * self.w:.2f} m/s'
        )

        self.get_logger().info(
            f'X SMC: '
            f'k1={self.k1x:.3f}, '
            f'k2={self.k2x:.3f}, '
            f'phi={self.phi_x:.3f}'
        )

        self.get_logger().info(
            f'Y SMC: '
            f'k1={self.k1y:.3f}, '
            f'k2={self.k2y:.3f}, '
            f'phi={self.phi_y:.3f}'
        )

        self.get_logger().info(
            f'Max velocity = '
            f'{self.max_velocity:.2f} m/s'
        )

        self.get_logger().info(
            f'Takeoff height = '
            f'{self.takeoff_height:.2f} m'
        )

        self.get_logger().info(
            '=========================================='
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def local_pos_callback(self, msg):

        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

        self.have_position = True

    def vel_callback(self, msg):

        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

        self.have_velocity = True

    def state_callback(self, msg):

        self.current_state = msg

    # =========================================================
    # VELOCITY COMMAND
    # =========================================================

    def publish_velocity(self, vx, vy):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub_velocity.publish(msg)

    def stop_velocity(self):

        self.publish_velocity(
            0.0,
            0.0
        )

    # =========================================================
    # SMC CONTROLLER
    #
    # S = e
    #
    # ex = xd - x
    # ey = yd - y
    #
    # Reaching law:
    #
    #     S_dot = -k1*S - k2*tanh(S/phi)
    #
    # For X:
    #
    #     Sx_dot = xd_dot - vx_cmd
    #
    # Therefore:
    #
    #     vx_cmd =
    #         xd_dot
    #         + k1x*Sx
    #         + k2x*tanh(Sx/phi_x)
    #
    # For Y:
    #
    #     Sy_dot = yd_dot - vy_cmd
    #
    #     vy_cmd =
    #         yd_dot
    #         + k1y*Sy
    #         + k2y*tanh(Sy/phi_y)
    #
    # =========================================================

    def smc_controller(
        self,
        xd,
        yd,
        xd_dot,
        yd_dot
    ):

        # ---------------------------------------------------------
        # Position tracking errors
        # ---------------------------------------------------------

        ex = xd - self.current_x
        ey = yd - self.current_y

        # ---------------------------------------------------------
        # Sliding surfaces
        #
        # S = e
        # ---------------------------------------------------------

        sx = ex
        sy = ey

        # ---------------------------------------------------------
        # SMC reaching-law terms
        #
        # delta_v =
        #     k1*S
        #     + k2*tanh(S/phi)
        # ---------------------------------------------------------

        ux = (
            self.k1x * sx
            + self.k2x
            * math.tanh(
                sx / self.phi_x
            )
        )

        uy = (
            self.k1y * sy
            + self.k2y
            * math.tanh(
                sy / self.phi_y
            )
        )

        # ---------------------------------------------------------
        # Velocity control law
        # ---------------------------------------------------------

        vx_cmd = xd_dot + ux
        vy_cmd = yd_dot + uy

        # ---------------------------------------------------------
        # Velocity saturation
        # ---------------------------------------------------------

        vx_cmd = max(
            -self.max_velocity,
            min(
                self.max_velocity,
                vx_cmd
            )
        )

        vy_cmd = max(
            -self.max_velocity,
            min(
                self.max_velocity,
                vy_cmd
            )
        )

        return (
            vx_cmd,
            vy_cmd,
            ex,
            ey,
            sx,
            sy,
            ux,
            uy
        )

    # =========================================================
    # LOGGING
    # =========================================================

    def log_smc_data(
        self,
        xd,
        yd,
        xd_dot,
        yd_dot,
        vx_cmd,
        vy_cmd,
        ex,
        ey,
        sx,
        sy,
        ux,
        uy
    ):

        self.log_data.append([
            time.time(),

            self.current_x,
            self.current_y,
            self.current_z,

            self.vx,
            self.vy,
            self.vz,

            xd,
            yd,

            xd_dot,
            yd_dot,

            ex,
            ey,

            sx,
            sy,

            ux,
            uy,

            vx_cmd,
            vy_cmd
        ])

    def save_log_to_csv(self):

        if (
            not self.save_log
            or not self.log_data
        ):
            return

        filename = os.path.join(
            os.path.expanduser('~'),
            f'smc_circle_'
            f'{time.strftime("%Y%m%d_%H%M%S")}.csv'
        )

        header = [
            'time',

            'x',
            'y',
            'z',

            'vx',
            'vy',
            'vz',

            'xd',
            'yd',

            'xd_dot',
            'yd_dot',

            'ex',
            'ey',

            'sx',
            'sy',

            'ux',
            'uy',

            'vx_cmd',
            'vy_cmd'
        ]

        try:

            with open(
                filename,
                'w',
                newline=''
            ) as f:

                writer = csv.writer(f)

                writer.writerow(header)
                writer.writerows(
                    self.log_data
                )

            self.get_logger().info(
                f'SMC log saved to: {filename}'
            )

            self.get_logger().info(
                f'Samples: '
                f'{len(self.log_data)}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to save log: {e}'
            )

    # =========================================================
    # SET MAVROS MESSAGE STREAM RATE
    # =========================================================

    def set_stream_rate(
        self,
        stream_id,
        rate
    ):

        req = CommandLong.Request()

        req.command = 511

        req.param1 = float(
            stream_id
        )

        req.param2 = float(
            1_000_000
            // max(rate, 1)
        )

        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        self.command_client.call_async(
            req
        )

    # =========================================================
    # STATE MACHINE
    # =========================================================

    def timer_callback(self):

        # =====================================================
        # WAIT CONNECTION
        # =====================================================

        if self.mission_state == self.WAIT_CONN:

            if (
                self.current_state
                and self.current_state.connected
                and self.have_position
                and self.have_velocity
            ):

                self.get_logger().info(
                    'MAVROS connected.'
                )

                self.get_logger().info(
                    'Local position and velocity available.'
                )

                self.set_stream_rate(
                    32,
                    20
                )

                self.mission_state = (
                    self.SEND_GUIDED
                )

            return

        # =====================================================
        # SEND GUIDED
        # =====================================================

        if self.mission_state == self.SEND_GUIDED:

            req = SetMode.Request()

            req.custom_mode = 'GUIDED'

            self.pending_future = (
                self.set_mode_client
                .call_async(req)
            )

            self.get_logger().info(
                'Switching to GUIDED mode...'
            )

            self.mission_state = (
                self.WAIT_GUIDED
            )

            return

        # =====================================================
        # WAIT GUIDED
        # =====================================================

        if self.mission_state == self.WAIT_GUIDED:

            if not self.pending_future.done():
                return

            try:

                resp = (
                    self.pending_future
                    .result()
                )

            except Exception as e:

                self.get_logger().error(
                    f'GUIDED service error: {e}'
                )

                self.mission_state = (
                    self.SEND_GUIDED
                )

                return

            if (
                resp
                and resp.mode_sent
            ):

                self.get_logger().info(
                    'GUIDED mode accepted.'
                )

                self.mission_state = (
                    self.SEND_ARM
                )

            else:

                self.get_logger().warn(
                    'GUIDED rejected. Retrying...'
                )

                self.mission_state = (
                    self.SEND_GUIDED
                )

            return

        # =====================================================
        # SEND ARM
        # =====================================================

        if self.mission_state == self.SEND_ARM:

            req = CommandBool.Request()

            req.value = True

            self.pending_future = (
                self.arming_client
                .call_async(req)
            )

            self.get_logger().info(
                'Arming vehicle...'
            )

            self.mission_state = (
                self.WAIT_ARM
            )

            return

        # =====================================================
        # WAIT ARM
        # =====================================================

        if self.mission_state == self.WAIT_ARM:

            if not self.pending_future.done():
                return

            try:

                resp = (
                    self.pending_future
                    .result()
                )

            except Exception as e:

                self.get_logger().error(
                    f'Arming service error: {e}'
                )

                self.mission_state = (
                    self.SEND_ARM
                )

                return

            if (
                resp
                and resp.success
            ):

                self.get_logger().info(
                    'Arming accepted.'
                )

                self.arm_time = None

                self.mission_state = (
                    self.WAIT_STABLE
                )

            else:

                self.get_logger().warn(
                    'Arming rejected. Retrying...'
                )

                self.mission_state = (
                    self.SEND_ARM
                )

            return

        # =====================================================
        # WAIT STABLE
        # =====================================================

        if self.mission_state == self.WAIT_STABLE:

            if (
                self.current_state
                and self.current_state.armed
            ):

                if self.arm_time is None:

                    self.arm_time = (
                        self.get_clock().now()
                    )

                    self.get_logger().info(
                        f'FCU armed. Waiting '
                        f'{self.ARM_STABLE_SEC:.1f}s...'
                    )

                elapsed = (
                    self.get_clock().now()
                    - self.arm_time
                ).nanoseconds / 1e9

                if (
                    elapsed
                    >= self.ARM_STABLE_SEC
                ):

                    self.get_logger().info(
                        'Stabilisation completed.'
                    )

                    self.mission_state = (
                        self.SEND_TAKEOFF
                    )

            else:

                self.arm_time = None

            return

        # =====================================================
        # SEND TAKEOFF
        # =====================================================

        if self.mission_state == self.SEND_TAKEOFF:

            req = CommandLong.Request()

            req.command = 22

            req.param1 = 0.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = float('nan')
            req.param5 = 0.0
            req.param6 = 0.0

            req.param7 = float(
                self.takeoff_height
            )

            self.pending_future = (
                self.command_client
                .call_async(req)
            )

            self.get_logger().info(
                f'Sending takeoff to '
                f'{self.takeoff_height:.1f} m...'
            )

            self.mission_state = (
                self.WAIT_TAKEOFF
            )

            return

        # =====================================================
        # WAIT TAKEOFF
        # =====================================================

        if self.mission_state == self.WAIT_TAKEOFF:

            if not self.pending_future.done():
                return

            try:

                resp = (
                    self.pending_future
                    .result()
                )

            except Exception as e:

                self.get_logger().error(
                    f'Takeoff service error: {e}'
                )

                self.mission_state = (
                    self.SEND_TAKEOFF
                )

                return

            if (
                resp
                and resp.success
            ):

                self.target_z = (
                    self.takeoff_height
                )

                self.get_logger().info(
                    f'Takeoff accepted. '
                    f'Target Z = '
                    f'{self.target_z:.2f} m'
                )

                self.mission_state = (
                    self.CLIMBING
                )

            else:

                self.get_logger().warn(
                    'Takeoff rejected. Retrying...'
                )

                self.mission_state = (
                    self.SEND_TAKEOFF
                )

            return

        # =====================================================
        # CLIMBING
        # =====================================================

        if self.mission_state == self.CLIMBING:

            if (
                self.current_z
                >= self.target_z - 0.15
            ):

                self.x0 = self.current_x
                self.y0 = self.current_y

                self.get_logger().info(
                    f'Altitude reached: '
                    f'{self.current_z:.2f} m'
                )

                self.get_logger().info(
                    f'Circle center = '
                    f'({self.x0:.2f}, '
                    f'{self.y0:.2f})'
                )

                self.mission_state = (
                    self.FLY_TO_START
                )

            return

        # =====================================================
        # FLY TO CIRCLE START
        # =====================================================

        if self.mission_state == self.FLY_TO_START:

            tx = (
                self.x0
                + self.radius
            )

            ty = self.y0

            dx = (
                tx
                - self.current_x
            )

            dy = (
                ty
                - self.current_y
            )

            distance = math.hypot(
                dx,
                dy
            )

            if distance < 1e-6:

                self.stop_velocity()

                self.start_time = (
                    time.time()
                )

                self.get_logger().info(
                    'Already at circle start.'
                )

                self.mission_state = (
                    self.CIRCLE
                )

                return

            if distance < 0.25:

                self.stop_velocity()

                self.start_time = (
                    time.time()
                )

                self.get_logger().info(
                    'Reached circle start.'
                )

                self.get_logger().info(
                    'Starting pure velocity-level SMC.'
                )

                self.mission_state = (
                    self.CIRCLE
                )

                return

            speed = min(
                1.0,
                distance
            )

            vx_cmd = (
                speed
                * dx
                / distance
            )

            vy_cmd = (
                speed
                * dy
                / distance
            )

            self.publish_velocity(
                vx_cmd,
                vy_cmd
            )

            return

        # =====================================================
        # CIRCLE — PURE VELOCITY-LEVEL SMC
        # =====================================================

        if self.mission_state == self.CIRCLE:

            t = (
                time.time()
                - self.start_time
            )

            angle = (
                self.w * t
            )

            # -------------------------------------------------
            # One complete revolution
            # -------------------------------------------------

            if (
                angle
                >= 2.0 * math.pi
            ):

                self.stop_velocity()

                self.get_logger().info(
                    '=========================================='
                )

                self.get_logger().info(
                    'SMC circle completed.'
                )

                self.get_logger().info(
                    'Flying back to center...'
                )

                self.get_logger().info(
                    '=========================================='
                )

                self.mission_state = (
                    self.FLY_TO_CENTER
                )

                return

            # -------------------------------------------------
            # Desired position
            # -------------------------------------------------

            xd = (
                self.x0
                + self.radius
                * math.cos(angle)
            )

            yd = (
                self.y0
                + self.radius
                * math.sin(angle)
            )

            # -------------------------------------------------
            # Desired velocity
            # -------------------------------------------------

            xd_dot = (
                -self.radius
                * self.w
                * math.sin(angle)
            )

            yd_dot = (
                self.radius
                * self.w
                * math.cos(angle)
            )

            # -------------------------------------------------
            # SMC controller
            # -------------------------------------------------

            (
                vx_cmd,
                vy_cmd,
                ex,
                ey,
                sx,
                sy,
                ux,
                uy
            ) = self.smc_controller(
                xd,
                yd,
                xd_dot,
                yd_dot
            )

            # -------------------------------------------------
            # Send velocity directly to MAVROS
            # -------------------------------------------------

            self.publish_velocity(
                vx_cmd,
                vy_cmd
            )

            # -------------------------------------------------
            # Log
            # -------------------------------------------------

            self.log_smc_data(
                xd,
                yd,
                xd_dot,
                yd_dot,
                vx_cmd,
                vy_cmd,
                ex,
                ey,
                sx,
                sy,
                ux,
                uy
            )

            return

        # =====================================================
        # FLY TO CENTER
        # =====================================================

        if self.mission_state == self.FLY_TO_CENTER:

            dx = (
                self.x0
                - self.current_x
            )

            dy = (
                self.y0
                - self.current_y
            )

            distance = math.hypot(
                dx,
                dy
            )

            if distance < 1e-6:

                self.stop_velocity()

                self.mission_state = (
                    self.SEND_LAND
                )

                return

            if distance < 0.25:

                self.stop_velocity()

                self.get_logger().info(
                    'Reached circle center.'
                )

                self.mission_state = (
                    self.SEND_LAND
                )

                return

            speed = min(
                1.0,
                distance
            )

            vx_cmd = (
                speed
                * dx
                / distance
            )

            vy_cmd = (
                speed
                * dy
                / distance
            )

            self.publish_velocity(
                vx_cmd,
                vy_cmd
            )

            return

        # =====================================================
        # SEND LAND
        # =====================================================

        if self.mission_state == self.SEND_LAND:

            req = SetMode.Request()

            req.custom_mode = 'LAND'

            self.pending_future = (
                self.set_mode_client
                .call_async(req)
            )

            self.get_logger().info(
                'Sending LAND mode...'
            )

            self.mission_state = (
                self.WAIT_LAND
            )

            return

        # =====================================================
        # WAIT LAND
        # =====================================================

        if self.mission_state == self.WAIT_LAND:

            if not self.pending_future.done():
                return

            try:

                resp = (
                    self.pending_future
                    .result()
                )

            except Exception as e:

                self.get_logger().error(
                    f'LAND service error: {e}'
                )

                self.mission_state = (
                    self.SEND_LAND
                )

                return

            if (
                resp
                and resp.mode_sent
            ):

                self.get_logger().info(
                    'LAND mode accepted.'
                )

                self.mission_state = (
                    self.WAIT_DISARM
                )

            else:

                self.get_logger().warn(
                    'LAND rejected. Retrying...'
                )

                self.mission_state = (
                    self.SEND_LAND
                )

            return

        # =====================================================
        # WAIT DISARM
        # =====================================================

        if self.mission_state == self.WAIT_DISARM:

            if (
                self.current_state
                and not self.current_state.armed
            ):

                self.get_logger().info(
                    'Drone disarmed.'
                )

                self.get_logger().info(
                    'Mission completed.'
                )

                self.save_log_to_csv()

                self.mission_state = (
                    self.DONE
                )

            return

        # =====================================================
        # DONE
        # =====================================================

        if self.mission_state == self.DONE:

            return


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = SmcCircleMission()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt.'
        )

    finally:

        if rclpy.ok():

            try:
                node.stop_velocity()
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
