# PDSMC hover mission — bay len, giu o do cao, ha canh sau 30s
#
# Kien truc: PDSMC (port MATLAB goc) cho CONTROL, GUIDED cho takeoff/landing.
#   Cu phap, state machine, Qos, timing giong y
#   takeoff_land_mission.py (da test OK tren mo phong va thuc nghiem).
#
# Tac vu: GUIDED -> ARM -> TAKEOFF -> CLIMBING ->
#         SETTLING (1.5s) -> CONTROL (PDSMC hover) -> LAND

from __future__ import annotations

import csv
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quat_to_euler_rpy(q: Quaternion) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def body_rates_from_euler_rates(
    phi: float, theta: float, phi_d: float, theta_d: float, psi_d: float,
) -> tuple[float, float, float]:
    sp, cp = math.sin(phi), math.cos(phi)
    sq, cq = math.sin(theta), math.cos(theta)
    ct = max(abs(cq), 1e-6)
    p = phi_d - sp * sq / ct * psi_d
    q = cp * theta_d + sp * psi_d
    r = -sp * theta_d + cp * psi_d
    return p, q, r


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class PdsmcMission(Node):

    WAIT_CONN   = 0
    SEND_GUIDED = 1
    WAIT_GUIDED = 2
    SEND_ARM    = 3
    WAIT_ARM    = 4
    WAIT_STABLE = 5   # wait for FCU arm confirmed + stabilisation delay (matches circle_mission.py)
    SEND_TAKEOFF = 6
    WAIT_TAKEOFF = 7
    CLIMBING    = 8
    SETTLING    = 9
    CONTROL     = 10
    SEND_LAND   = 11
    WAIT_LAND   = 12
    WAIT_DISARM = 13
    DONE        = 14

    def __init__(self):

        super().__init__('pdsmc_mission')

        self.declare_parameter('save_log', True)
        self.save_log = self.get_parameter('save_log').get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_attitude = self.create_publisher(
            PoseStamped, '/mavros/setpoint_attitude/attitude', qos,
        )
        self.pub_thrust = self.create_publisher(
            Float64, '/mavros/setpoint_attitude/accel_thrust_throttle', qos,
        )
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.planned_path = Path()
        self.planned_path.header.frame_id = 'map'
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pos_callback,
            qos,
        )
        self.vel_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.vel_callback,
            qos,
        )
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos,
        )
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos,
        )

        self.current_z = 0.0
        self.current_state: State | None = None

        self._px = 0.0
        self._py = 0.0
        self._pz = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._phi = 0.0
        self._theta = 0.0
        self._psi = 0.0
        self._psi_from_pose = False
        self._phid = 0.0
        self._thetd = 0.0
        self._psid = 0.0
        self._imu_valid = False

        self.mission_state = self.WAIT_CONN
        self.pending_future: rclpy.task.Future | None = None
        self.control_t0: float | None = None
        self.arm_time: rclpy.time.Time | None = None
        self.ARM_STABLE_SEC = 2.0
        self.ARM_RETRY_TIMEOUT_SEC = 30.0
        self._arm_retry_start_time: float | None = None
        self._stable_start_time: float | None = None
        self.CLIMB_TIMEOUT_SEC = 30.0
        self.CLIMB_TARGET_Z = 3.0
        self._control_logged = False
        self._z_ref_override: float | None = None   # set when climb times out
        self._land_start_time: float | None = None
        self.LAND_TIMEOUT_SEC = 60.0
        self.DISARM_TIMEOUT_SEC = 120.0
        self._disarm_start_time: float | None = None
        self.climb_start_time: float | None = None
        self._climb_last_log: float | None = None

        # Euler angle log (time, roll, pitch, yaw in rad)
        self.euler_log: list[tuple[float, float, float, float]] = []

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandLong, '/mavros/cmd/command')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('PDSMC hover node ready — 30s hover then land')

    # --- callbacks ---
    def local_pos_callback(self, msg: PoseStamped):
        self._px = msg.pose.position.x
        self._py = msg.pose.position.y
        self._pz = msg.pose.position.z
        self.current_z = msg.pose.position.z
        # Always extract yaw from pose orientation (works even if IMU stream is down)
        phi, theta, psi = quat_to_euler_rpy(msg.pose.orientation)
        self._psi = psi
        self._psi_from_pose = True
        if self.mission_state >= self.CLIMBING and self.mission_state <= self.WAIT_DISARM:
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = 'map'
            pose.pose = msg.pose
            self.actual_path.poses.append(pose)
            self.actual_path.header.stamp = msg.header.stamp
            self.actual_path_pub.publish(self.actual_path)

    def vel_callback(self, msg: TwistStamped):
        self._vx = msg.twist.linear.x
        self._vy = msg.twist.linear.y
        self._vz = msg.twist.linear.z

    def state_callback(self, msg: State):
        self.current_state = msg

    def imu_callback(self, msg: Imu):
        self._imu_valid = True
        self._psi_from_pose = False   # IMU takes precedence
        self._phi, self._theta, self._psi = quat_to_euler_rpy(msg.orientation)
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z
        phi_d, theta_d, psi_d = body_rates_from_euler_rates(
            self._phi, self._theta, p, q, r,
        )
        self._phid = phi_d
        self._thetd = theta_d
        self._psid = psi_d

    def set_stream_rate(self, stream_id: int, rate: int):
        req = CommandLong.Request()
        req.command = 511
        req.param1 = float(stream_id)
        req.param2 = float(1_000_000 // max(rate, 1))
        self.takeoff_client.call_async(req)

    def _publish_attitude(self, roll: float, pitch: float, yaw: float, throttle: float):
        now = self.get_clock().now().to_msg()
        att_msg = PoseStamped()
        att_msg.header.stamp = now
        att_msg.header.frame_id = 'map'
        att_msg.pose.orientation = euler_to_quat(roll, pitch, yaw)
        self.pub_attitude.publish(att_msg)
        thr_msg = Float64()
        thr_msg.data = float(throttle)
        self.pub_thrust.publish(thr_msg)

    # --- state machine ---
    def timer_callback(self):

        # --- WAIT_CONN ---
        if self.mission_state == self.WAIT_CONN:
            if self.current_state and self.current_state.connected:
                self.get_logger().info('MAVROS connected')
                self.set_stream_rate(32, 20)   # LOCAL_POSITION_NED
                self.set_stream_rate(33, 20)   # LOCAL_VELOCITY_NED
                self.set_stream_rate(29, 20)   # RAW_IMU
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_GUIDED ---
        if self.mission_state == self.SEND_GUIDED:
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info('Switching to GUIDED mode...')
            self.mission_state = self.WAIT_GUIDED
            return

        # --- WAIT_GUIDED ---
        if self.mission_state == self.WAIT_GUIDED:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('GUIDED mode accepted')
                self.mission_state = self.SEND_ARM
            else:
                self.get_logger().warn('GUIDED mode rejected, retrying...')
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_ARM ---
        if self.mission_state == self.SEND_ARM:
            req = CommandBool.Request()
            req.value = True
            self.pending_future = self.arming_client.call_async(req)
            self.get_logger().info('Arming vehicle...')
            self._arm_retry_start_time = None
            self._stable_start_time = None
            self.mission_state = self.WAIT_ARM
            return

        # --- WAIT_ARM ---
        if self.mission_state == self.WAIT_ARM:
            if self._arm_retry_start_time is None:
                self._arm_retry_start_time = time.time()
            if (time.time() - self._arm_retry_start_time) >= self.ARM_RETRY_TIMEOUT_SEC:
                self.get_logger().error(f'Arming rejected for {self.ARM_RETRY_TIMEOUT_SEC:.0f}s — giving up')
                self.mission_state = self.DONE
                return
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.get_logger().info('Arming accepted, waiting for FCU arm confirmation + stabilisation...')
                self.arm_time = None
                self._arm_retry_start_time = None
                self.mission_state = self.WAIT_STABLE
            else:
                self.get_logger().warn('Arming rejected, retrying...')
                self.mission_state = self.SEND_ARM
            return

        # --- WAIT_STABLE: wait until FCU confirms armed AND stabilisation delay passes ---
        STABLE_TIMEOUT_SEC = 20.0
        if self.mission_state == self.WAIT_STABLE:
            if self._stable_start_time is None:
                self._stable_start_time = time.time()
            if (time.time() - self._stable_start_time) >= STABLE_TIMEOUT_SEC:
                self.get_logger().warn(f'WAIT_STABLE timeout after {STABLE_TIMEOUT_SEC:.0f}s — forcing takeoff')
                self.control_t0 = time.time()
                self._control_logged = False
                self.mission_state = self.SETTLING
                return
            if self.current_state and self.current_state.armed:
                if self.arm_time is None:
                    self.arm_time = self.get_clock().now()
                    self.get_logger().info(f'FCU armed confirmed, stabilising for {self.ARM_STABLE_SEC:.1f}s...')
                elapsed = (self.get_clock().now() - self.arm_time).nanoseconds / 1e9
                if elapsed >= self.ARM_STABLE_SEC:
                    self.get_logger().info('Stabilisation done, proceeding to takeoff')
                    self.mission_state = self.SEND_TAKEOFF
            else:
                self.arm_time = None
            return

        # --- SEND_TAKEOFF ---
        if self.mission_state == self.SEND_TAKEOFF:
            req = CommandLong.Request()
            req.command = 22
            req.param1 = 0.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = float('nan')
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 3.0
            self.pending_future = self.takeoff_client.call_async(req)
            self.get_logger().info('Sending takeoff command...')
            self.mission_state = self.WAIT_TAKEOFF
            return

        # --- WAIT_TAKEOFF ---
        if self.mission_state == self.WAIT_TAKEOFF:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.get_logger().info('Takeoff accepted, climbing to 3.0m...')
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn('Takeoff rejected, retrying...')
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING ---
        if self.mission_state == self.CLIMBING:
            if self.climb_start_time is None:
                self.climb_start_time = time.time()
                self._climb_last_log = None

            now = time.time()
            elapsed = now - self.climb_start_time

            # Log altitude every 2s so we can see current_z in terminal
            if self._climb_last_log is None or (now - self._climb_last_log) >= 2.0:
                self.get_logger().info(f'Climbing... z={self.current_z:.3f}m (t+{elapsed:.1f}s)')
                self._climb_last_log = now

            # NOTE: Do NOT publish attitude here. GUIDED mode handles takeoff autonomously.
            # Publishing attitude setpoints can interfere with PX4's altitude hold.

            # Use margin to handle EKF drift (target 2.8m so we always pass)
            if self.current_z >= self.CLIMB_TARGET_Z - 0.2:
                self.get_logger().info(
                    f'Altitude reached z={self.current_z:.2f}m — settling 1.5s...',
                )
                self._z_ref_override = self.current_z   # use actual altitude as reference
                self.control_t0 = time.time()
                self.climb_start_time = None
                self._climb_last_log = None
                self.mission_state = self.SETTLING
                return

            # Timeout protection: if still climbing after timeout, proceed anyway
            if elapsed >= self.CLIMB_TIMEOUT_SEC:
                self.get_logger().warn(
                    f'Climb timeout at z={self.current_z:.2f}m after {elapsed:.1f}s — forcing settle',
                )
                self._z_ref_override = self.current_z   # use actual altitude as reference
                self.control_t0 = time.time()
                self.climb_start_time = None
                self._climb_last_log = None
                self.mission_state = self.SETTLING
            return

        # --- SETTLING ---
        if self.mission_state == self.SETTLING:
            t_settle = time.time() - self.control_t0
            # NOTE: Do NOT publish attitude here. Let GUIDED mode hold position/altitude.
            if t_settle >= 1.5:
                self.control_t0 = time.time()
                self._control_logged = False
                self.get_logger().info(
                    f'Settling done — entering PDSMC CONTROL at z={self.current_z:.2f}m',
                )
                self.mission_state = self.CONTROL
            return

        # --- CONTROL: PDSMC hover ---
        if self.mission_state == self.CONTROL:
            #
            # IMPORTANT: In GUIDED mode, ArduPilot runs its own GUIDED position controller
            # which computes attitude setpoints from position error. The setpoint_attitude
            # topic is intended to override this. However, to guarantee no interference
            # from FCU's inner loops, the safest approach is to switch to ACRO mode here
            # (PDSMC takes full authority) and switch back to GUIDED before LAND.
            #
            # If you want to try ACRO mode for CONTROL, uncomment the block below and
            # add an ACRO→GUIDED transition before SEND_LAND. See circle_mission.py for
            # the GUIDED/SEND_LAND pattern which is proven.
            #
            # For now we stay in GUIDED and rely on setpoint_attitude overriding the
            # GUIDED position controller, consistent with the original MATLAB design.
            #
            t_run = time.time() - self.control_t0

            z_ref = self._z_ref_override if self._z_ref_override is not None else 3.0

            if not self._control_logged:
                self.get_logger().info(
                    f'[CONTROL] Started — hovering at z={self._pz:.3f}m (z_ref={z_ref:.3f}m) for {30.0}s then land',
                )
                self._control_logged = True

            # Periodic visibility log every 5s
            if int(t_run) % 5 == 0 and abs(t_run - int(t_run)) < 0.1:
                self.get_logger().info(
                    f'[CONTROL] t={t_run:.1f}s | z={self._pz:.3f}m | yaw={math.degrees(self._psi):.1f}deg',
                )

            m = 0.8
            g = 9.81
            # X500: 4 motors, hover thrust ≈ m*g / (4 motors) = 1.96N per motor
            # throttle = U1 / (max_thrust), where max_thrust = 2.0 * m * g (T/W ratio ~2)
            N_MOTORS = 4
            THRUST_MAX = 2.0 * m * g   # N, max thrust per motor = 1.57 * hover
            ez   = z_ref - self._pz
            ez_d = -self._vz
            kp1, kd1, H1, lam1 = 100.0, 40.0, 160.0, 100.0
            U1_raw = m * g + kp1 * ez + kd1 * ez_d + H1 * math.tanh(ez_d + lam1 * ez)
            U1 = max(0.3 * m * g, U1_raw)
            throttle = float(np.clip(U1 / THRUST_MAX, 0.0, 1.0))

            kpx, kdx, Hx, lamx = 1.5, 1.0, 0.5, 0.5
            kpy, kdy, Hy, lamy = 1.5, 1.0, 0.5, 0.5
            ex   = 0.0 - self._px
            ex_d = 0.0 - self._vx
            ey   = 0.0 - self._py
            ey_d = 0.0 - self._vy
            Ux = kpx * ex + kdx * ex_d + Hx * math.tanh(ex_d + lamx * ex)
            Uy = kpy * ey + kdy * ey_d + Hy * math.tanh(ey_d + lamy * ey)

            T_norm = max(U1 / m, 1e-6)
            sinphi = (Ux * math.sin(self._psi) - Uy * math.cos(self._psi)) / T_norm
            phides = math.asin(float(np.clip(sinphi, -1.0, 1.0)))
            sintheta = (Ux * math.cos(self._psi) + Uy * math.sin(self._psi)) / (
                T_norm * max(math.cos(phides), 1e-6)
            )
            thetades = math.asin(float(np.clip(sintheta, -1.0, 1.0)))

            self._publish_attitude(
                roll=phides, pitch=thetades, yaw=self._psi, throttle=throttle,
            )

            if int(t_run) % 5 == 0 and abs(t_run - int(t_run)) < 0.1:
                self.get_logger().info(
                    f't={t_run:.1f}s | pos=({self._px:.2f},{self._py:.2f},{self._pz:.2f}) '
                    f'| phi={math.degrees(self._phi):.1f}deg the={math.degrees(self._theta):.1f}deg '
                    f'| phides={math.degrees(phides):.2f}deg thdes={math.degrees(thetades):.2f}deg '
                    f'U1={U1:.2f}N thr={throttle:.3f}',
                )

            # Log Euler angles every loop during CONTROL
            self.euler_log.append((time.time(), self._phi, self._theta, self._psi))

            self.add_planned(0.0, 0.0, z_ref)

            dur = 30.0
            if t_run >= dur:
                self.get_logger().info(
                    f'Control duration {dur:.1f}s elapsed — landing',
                )
                self.mission_state = self.SEND_LAND

            return

        # --- SEND_LAND ---
        if self.mission_state == self.SEND_LAND:
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            self.pending_future = self.set_mode_client.call_async(req)
            self._land_start_time = None   # reset timeout tracker
            self.get_logger().info('Sending LAND command...')
            self.mission_state = self.WAIT_LAND
            return

        # --- WAIT_LAND ---
        if self.mission_state == self.WAIT_LAND:
            # Track time for LAND mode timeout
            if self._land_start_time is None:
                self._land_start_time = time.time()

            # NOTE: Do NOT publish attitude here. LAND mode needs full authority from FCU.
            # Publishing attitude setpoints interferes with ArduPilot's LAND controller.

            # Timeout: if LAND mode not accepted after timeout, retry
            land_elapsed = time.time() - self._land_start_time
            if land_elapsed >= self.LAND_TIMEOUT_SEC:
                self.get_logger().warn(f'LAND timeout after {land_elapsed:.1f}s — retrying...')
                self._land_start_time = None
                self.mission_state = self.SEND_LAND
                return

            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('LAND mode accepted, waiting for drone to land...')
                self._land_start_time = None
                self.mission_state = self.WAIT_DISARM
            else:
                self.get_logger().warn('LAND mode rejected, retrying...')
                self._land_start_time = None
                self.mission_state = self.SEND_LAND
            return

        # --- WAIT_DISARM ---
        if self.mission_state == self.WAIT_DISARM:
            # Track time for disarm timeout
            if self._disarm_start_time is None:
                self._disarm_start_time = time.time()
                self.get_logger().info(
                    f'Waiting for disarm (timeout={self.DISARM_TIMEOUT_SEC:.0f}s)...',
                )

            # NOTE: Do NOT publish attitude here. Drone is in LAND mode, FCU handles descent.
            # Publishing attitude setpoints can interfere with the landing controller.

            # Timeout: force finish if disarm takes too long
            disarm_elapsed = time.time() - self._disarm_start_time
            if disarm_elapsed >= self.DISARM_TIMEOUT_SEC:
                self.get_logger().warn(
                    f'Disarm timeout after {disarm_elapsed:.1f}s — finishing mission anyway',
                )
                self._finish_mission()
                return

            if self.current_state and not self.current_state.armed:
                self.get_logger().info('Drone disarmed, mission complete.')
                self._finish_mission()
            return

    def _finish_mission(self):
        if self.save_log:
            self.get_logger().info('Saving paths...')
            self.save_paths_to_csv()
        self.mission_state = self.DONE

    def save_paths_to_csv(self):
        if not self.save_log:
            return
        home = os.path.expanduser('~')
        ts = time.strftime('%Y%m%d_%H%M%S')

        # Planned path CSV — with quaternion (orientation.w=1, others=0 by default)
        planned_file = os.path.join(home, f'pdsmc_planned_{ts}.csv')
        with open(planned_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            for pose in self.planned_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ])
        self.get_logger().info(f'Planned path saved: {planned_file} ({len(self.planned_path.poses)} points)')

        # Actual path CSV — with quaternion from local position
        actual_file = os.path.join(home, f'pdsmc_actual_{ts}.csv')
        with open(actual_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            for pose in self.actual_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ])
        self.get_logger().info(f'Actual path saved: {actual_file} ({len(self.actual_path.poses)} points)')

        # Euler angle log CSV
        if self.euler_log:
            euler_file = os.path.join(home, f'pdsmc_euler_{ts}.csv')
            with open(euler_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_sec', 'roll_rad', 'pitch_rad', 'yaw_rad',
                                'roll_deg', 'pitch_deg', 'yaw_deg'])
                for t, roll, pitch, yaw in self.euler_log:
                    writer.writerow([t,
                                    roll, pitch, yaw,
                                    math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])
            self.get_logger().info(f'Euler log saved: {euler_file} ({len(self.euler_log)} points)')
        else:
            self.get_logger().warn('No Euler data to save')

    def add_planned(self, x: float, y: float, z: float):
        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.planned_path.poses.append(pose)
        self.planned_path.header.stamp = now
        self.planned_path_pub.publish(self.planned_path)


def main():
    rclpy.init()
    node = PdsmcMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
