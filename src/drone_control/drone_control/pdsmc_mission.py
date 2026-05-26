# PDSMC hover mission — port tu run_PDSMC.m, quy dao: x=0,y=0,z=3m,psi=0
#
# Kien truc:
#   PDSMC (port MATLAB goc)  -->  setpoint_attitude  -->  ArduPilot inner loops
#   Thuong ARDUPLANE inner loop xu ly attitude (roll/pitch/yaw) + thrust.
#   Chu khong phai velocity interface nhu AI cu gen (that bai vi gains khong phu hop).
#
#   Ly do chon setpoint_attitude thay vi cmd_vel:
#     - PDSMC tinh phides, thetades, U1 (thrust) — dang attitude/thrust
#     - ArduPilot co san inner PID cho roll/pitch — ta chi can gui goc mong muon
#     - Tranh phai map attitude -> velocity (phep bien doi nhieu thong tin)
#
# Tac vu: GUIDED -> ARM -> TAKEOFF -> PDSMC (hover 3m) -> LAND

from __future__ import annotations

import math
import os
import signal
import time
from contextlib import suppress

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, SetMode, CommandBool
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from drone_control.pdsmc_core import (
    HoverTrajectory,
    PDSMCGains,
    QuadPlantParams,
    pdsmc_step,
)


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
    phi: float, theta: float, phi_d: float, theta_d: float, psi_d: float
) -> tuple[float, float, float]:
    """Doi euler rates -> body angular rates (p,q,r)."""
    sp, cp = math.sin(phi), math.cos(phi)
    sq, cq = math.sin(theta), math.cos(theta)
    ct = max(abs(cq), 1e-6)
    p = phi_d - sp * sq / ct * psi_d
    q = cp * theta_d + sp * psi_d
    r = -sp * theta_d + cp * psi_d
    return p, q, r


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Chuyen Euler (rad) -> Quaternion."""
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


def thrust_from_U1(U1: float, plant: QuadPlantParams) -> float:
    """An toan chuyen U1 (N) -> throttle (0..1) cho ArduPilot.

    U1 la luc hut tong (N). Tai hover: U1 = m*g = 0.8*9.81 = 7.848 N.
    ArduPilot can throttle trong khoang [0,1] trong setpoint_attitude.
    Ta map: throttle = clamp(U1 / (m * g_max_cell), 0, 1).
    Gian doan: g_max_cell = 15 N (uoc tinh cho 4 rotor Iris).
    """
    g_max_cell = 15.0  # N — luc hut toi da 1 rotor
    throttle = float(np.clip(U1 / (plant.m * g_max_cell), 0.0, 1.0))
    return throttle


class PdsmcMission(Node):
    WAIT_CONN  = 0
    SEND_GUIDED = 1
    WAIT_GUIDED = 2
    SEND_ARM    = 3
    WAIT_ARM    = 4
    SEND_TAKEOFF = 5
    WAIT_TAKEOFF = 6
    CLIMBING    = 7
    CONTROL     = 8
    SEND_LAND   = 9
    WAIT_LAND   = 10
    WAIT_DISARM = 11
    DONE        = 12

    def __init__(self):
        super().__init__('pdsmc_mission')

        # --- Launch parameters ---
        self.declare_parameter('takeoff_alt', 3.0)
        self.declare_parameter('hover_alt', 3.0)
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('auto_land', True)
        self.declare_parameter('control_duration_sec', 30.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self._pub_attitude = self.create_publisher(
            PoseStamped, '/mavros/setpoint_attitude/attitude', qos
        )
        self._pub_thrust = self.create_publisher(
            Float64, '/mavros/setpoint_attitude/accel_thrust_throttle', qos
        )
        # Van giữ publisher cmd_vel phòng khi can debug
        self._pub_cmd = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos
        )

        # Subscribers
        self._sub_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._cb_pose, qos
        )
        self._sub_vel = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._cb_vel,
            qos,
        )
        self._sub_state = self.create_subscription(
            State, '/mavros/state', self._cb_state, qos
        )
        self._sub_imu = self.create_subscription(
            Imu, '/mavros/imu/data', self._cb_imu, qos
        )

        # State
        self._px = self._py = self._pz = 0.0
        self._vx = self._vy = self._vz = 0.0
        self._phi = self._theta = self._psi = 0.0
        self._phid = self._thetd = self._psid = 0.0
        self._imu_valid = False
        self._state_msg: State | None = None

        self._mission = self.WAIT_CONN
        self._pending: rclpy.task.Future | None = None
        self._shutdown_requested = False

        # Bắt Ctrl+C (SIGINT), Ctrl+Z (SIGTSTP), và terminal close (SIGTERM)
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTSTP, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)

        # Controller objects
        z_ref = float(self.get_parameter('hover_alt').value)
        self._traj = HoverTrajectory(z_const=z_ref)
        self._gains = PDSMCGains()      # dung gains MATLAB goc
        self._plant = QuadPlantParams()

        hz = float(self.get_parameter('control_rate_hz').value)
        self._Ts = 1.0 / max(hz, 1.0)
        self._control_t0: float | None = None

        # Services
        self._set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self._arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._cmd_long = self.create_client(CommandLong, '/mavros/cmd/command')

        for name, cli in [
            ('set_mode', self._set_mode),
            ('arming', self._arming),
            ('command', self._cmd_long),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

        self._timer = self.create_timer(self._Ts, self._timer_cb)
        self.get_logger().info(
            f'PDSMC hover node ready — rate={hz}Hz, hover_z={z_ref}m, '
            f'duration={float(self.get_parameter("control_duration_sec").value)}s'
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _cb_pose(self, msg: PoseStamped):
        self._px = msg.pose.position.x
        self._py = msg.pose.position.y
        self._pz = msg.pose.position.z

    def _cb_vel(self, msg: TwistStamped):
        self._vx = msg.twist.linear.x
        self._vy = msg.twist.linear.y
        self._vz = msg.twist.linear.z

    def _cb_state(self, msg: State):
        self._state_msg = msg

    def _cb_imu(self, msg: Imu):
        self._imu_valid = True
        self._phi, self._theta, self._psi = quat_to_euler_rpy(msg.orientation)
        # Angular velocity tu IMU la body rates (p,q,r)
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z
        # Chuyen body rates -> euler rates de co phid, thetd, psid cho PDSMC
        phi_d, theta_d, psi_d = body_rates_from_euler_rates(
            self._phi, self._theta, p, q, r
        )
        self._phid = phi_d
        self._thetd = theta_d
        self._psid = psi_d

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _state_vec(self) -> np.ndarray:
        """Xay dung vector 12 trang thai giong MATLAB.
        thu tu: [x, xd, y, yd, z, zd, phi, phid, theta, thetd, psi, psid]
        Velocity lay tu MAVROS /local_position/velocity_local (ENU frame).
        """
        return np.array(
            [
                self._px, self._vx,
                self._py, self._vy,
                self._pz, self._vz,
                self._phi, self._phid,
                self._theta, self._thetd,
                self._psi, self._psid,
            ],
            dtype=float,
        )

    def _set_stream_rate(self, stream_id: int, rate_hz: int):
        req = CommandLong.Request()
        req.command = 511          # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(stream_id)   # LOCAL_POSITION_NED = 32
        req.param2 = float(1_000_000 // max(rate_hz, 1))
        self._cmd_long.call_async(req)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _timer_cb(self):
        if self._mission == self.WAIT_CONN:
            if self._state_msg and self._state_msg.connected:
                self.get_logger().info('MAVROS connected — streaming local position @40Hz')
                self._set_stream_rate(32, 40)
                self._mission = self.SEND_GUIDED
            return

        if self._mission == self.SEND_GUIDED:
            r = SetMode.Request()
            r.custom_mode = 'GUIDED'
            self._pending = self._set_mode.call_async(r)
            self._mission = self.WAIT_GUIDED
            return

        if self._mission == self.WAIT_GUIDED:
            if not self._pending.done():
                return
            resp = self._pending.result()
            ok = resp is not None and resp.mode_sent
            if ok:
                self.get_logger().info('GUIDED mode set — arming...')
                self._mission = self.SEND_ARM
            else:
                self._mission = self.SEND_GUIDED
            return

        if self._mission == self.SEND_ARM:
            r = CommandBool.Request()
            r.value = True
            self._pending = self._arming.call_async(r)
            self._mission = self.WAIT_ARM
            return

        if self._mission == self.WAIT_ARM:
            if not self._pending.done():
                return
            resp = self._pending.result()
            if resp and resp.success:
                self.get_logger().info('Armed — sending takeoff...')
                self._mission = self.SEND_TAKEOFF
            else:
                self._mission = self.SEND_ARM
            return

        if self._mission == self.SEND_TAKEOFF:
            r = CommandLong.Request()
            r.command = 22          # MAV_CMD_NAV_TAKEOFF
            r.param7 = float(self.get_parameter('takeoff_alt').value)
            self._pending = self._cmd_long.call_async(r)
            self._mission = self.WAIT_TAKEOFF
            return

        if self._mission == self.WAIT_TAKEOFF:
            if not self._pending.done():
                return
            resp = self._pending.result()
            if resp and resp.success:
                self.get_logger().info('Takeoff accepted — climbing...')
                self._mission = self.CLIMBING
            else:
                self._mission = self.SEND_TAKEOFF
            return

        if self._mission == self.CLIMBING:
            if self._pz >= float(self.get_parameter('takeoff_alt').value) - 0.05:
                if not self._imu_valid:
                    self.get_logger().warn('IMU not ready — waiting...')
                    return
                self.get_logger().info(
                    f'Altitude reached z={self._pz:.2f}m — '
                    'settling 1.5s before PDSMC'
                )
                self._control_t0 = time.monotonic()
                self._mission = self.CONTROL
            return

        if self._mission == self.CONTROL:
            t_run = time.monotonic() - (self._control_t0 or time.monotonic())

            # Settling: 1.5s dau gui zero de ArduPilot on dinh
            SETTLE_SEC = 1.5
            if t_run < SETTLE_SEC:
                self._publish_attitude(roll=0.0, pitch=0.0, yaw=self._psi, throttle=0.5)
                return

            # PDSMC control step
            x = self._state_vec()
            out = pdsmc_step(x, self._traj, self._gains, self._plant, t_run, psi=self._psi)

            # Lay goc hien tai lam yaw tham chieu (khong doi yaw)
            current_yaw = self._psi

            # Chuyen U1 -> throttle
            throttle = thrust_from_U1(out["U1"], self._plant)

            # Gui setpoint_attitude
            self._publish_attitude(
                roll=out["phides"],
                pitch=out["thetades"],
                yaw=current_yaw,
                throttle=throttle,
            )

            # Log mỗi 5 giây
            if int(t_run) % 5 == 0 and abs(t_run - int(t_run)) < self._Ts:
                self.get_logger().info(
                    f't={t_run:.1f}s | pos=({self._px:.2f},{self._py:.2f},{self._pz:.2f}) '
                    f'| phi={math.degrees(self._phi):.1f}deg the={math.degrees(self._theta):.1f}deg '
                    f'| phides={math.degrees(out["phides"]):.2f}deg '
                    f'thdes={math.degrees(out["thetades"]):.2f}deg '
                    f'U1={out["U1"]:.2f}N thr={throttle:.3f}'
                )

            # Kiem tra het gio dieu khien
            dur = float(self.get_parameter('control_duration_sec').value)
            auto_land = self.get_parameter('auto_land').get_parameter_value().bool_value
            if auto_land and t_run >= dur:
                self.get_logger().info('Control duration elapsed — switching to LAND')
                self._mission = self.SEND_LAND

            # Ctrl+C handler: gửi LAND rồi rclpy.shutdown()
            self._try_shutdown_land()
            return

        if self._mission == self.SEND_LAND:
            r = SetMode.Request()
            r.custom_mode = 'LAND'
            self._pending = self._set_mode.call_async(r)
            self._mission = self.WAIT_LAND
            return

        if self._mission == self.WAIT_LAND:
            if not self._pending.done():
                return
            resp = self._pending.result()
            if resp and resp.mode_sent:
                self._mission = self.WAIT_DISARM
            else:
                self._mission = self.SEND_LAND
            return

        if self._mission == self.WAIT_DISARM:
            if self._shutdown_requested:
                import sys
                self.get_logger().info('Done — shutting down')
                self._mission = self.DONE
                rclpy.shutdown()
                return
            if self._state_msg and not self._state_msg.armed:
                self.get_logger().info('Done — landed and disarmed')
                self._mission = self.DONE
            return

    _shutdown_land_requested: bool = False

    def _on_shutdown(self, signum, frame):  # noqa: ARG001
        """Bắt mọi signal thoát → LAND rồi thoát node."""
        import sys
        signame = signal.Signals(signum).name
        sys.stderr.write(f'Got {signame} — requesting LAND\n')
        sys.stderr.flush()
        self._shutdown_requested = True
        self._shutdown_land_requested = True

    def _try_shutdown_land(self):
        """Được gọi trong timer loop — gửi LAND rồi shutdown."""
        if not self._shutdown_land_requested:
            return
        if self._mission not in (self.WAIT_LAND, self.WAIT_DISARM, self.DONE):
            land_req = SetMode.Request()
            land_req.custom_mode = 'LAND'
            self._pending = self._set_mode.call_async(land_req)
            self._mission = self.WAIT_LAND
            self._shutdown_land_requested = False
            import sys
            sys.stderr.write('LAND request sent\n')
            sys.stderr.flush()
            return
        import sys
        rclpy.shutdown()

    def _publish_attitude(
        self, roll: float, pitch: float, yaw: float, throttle: float
    ):
        now = self.get_clock().now().to_msg()

        # setpoint_attitude/attitude: quaternion + header
        att_msg = PoseStamped()
        att_msg.header.stamp = now
        att_msg.header.frame_id = 'map'
        att_msg.pose.orientation = euler_to_quat(roll, pitch, yaw)
        self._pub_attitude.publish(att_msg)

        # setpoint_attitude/accel_thrust_throttle: throttle (0..1)
        thr_msg = Float64()
        thr_msg.data = float(throttle)
        self._pub_thrust.publish(thr_msg)


def main():
    rclpy.init()
    node = PdsmcMission()
    # Ctrl+C → _on_shutdown gửi LAND → rclpy.shutdown() → spin() thoát
    try:
        rclpy.spin(node)
    except RuntimeError:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
