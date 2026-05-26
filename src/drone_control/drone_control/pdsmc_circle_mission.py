# PDSMC circle mission — bay hinh tron roi quay ve vi tri cu
#
# Kien truc:
#   PDSMC (port MATLAB goc)  -->  setpoint_attitude  -->  ArduPilot inner loops
#   PDSMC tinh phides, thetades, U1 (thrust) — dang attitude/thrust
#
# Tac vu: GUIDED -> ARM -> TAKEOFF -> FLY_TO_START -> CIRCLE (PDSMC) ->
#         FLY_BACK -> LAND
#
# Quy dao: hinh tron tam tai (xc, yc), ban kinh R, bay 1 vong roi quay ve

from __future__ import annotations

import csv
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
    CircleTrajectory,
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


def thrust_from_U1(U1: float, plant: QuadPlantParams) -> float:
    g_max_cell = 15.0
    throttle = float(np.clip(U1 / (plant.m * g_max_cell), 0.0, 1.0))
    return throttle


class PdsmcCircleMission(Node):
    WAIT_CONN     = 0
    SEND_GUIDED   = 1
    WAIT_GUIDED   = 2
    SEND_ARM      = 3
    WAIT_ARM      = 4
    SEND_TAKEOFF  = 5
    WAIT_TAKEOFF  = 6
    CLIMBING      = 7
    FLY_TO_START  = 8
    CIRCLE        = 9
    FLY_BACK      = 10
    SEND_LAND     = 11
    WAIT_LAND     = 12
    WAIT_DISARM   = 13
    DONE          = 14

    def __init__(self):
        super().__init__('pdsmc_circle_mission')

        self.declare_parameter('takeoff_alt', 5.0)
        self.declare_parameter('circle_radius', 5.0)
        self.declare_parameter('circle_w', 0.3)
        self.declare_parameter('control_rate_hz', 40.0)
        self.declare_parameter('auto_land', True)
        self.declare_parameter('circle_laps', 1)
        self.declare_parameter('flyback_speed', 1.5)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_attitude = self.create_publisher(
            PoseStamped, '/mavros/setpoint_attitude/attitude', qos
        )
        self._pub_thrust = self.create_publisher(
            Float64, '/mavros/setpoint_attitude/accel_thrust_throttle', qos
        )
        self._pub_cmd = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos
        )

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

        self._px = self._py = self._pz = 0.0
        self._vx = self._vy = self._vz = 0.0
        self._phi = self._theta = self._psi = 0.0
        self._phid = self._thetd = self._psid = 0.0
        self._imu_valid = False
        self._state_msg: State | None = None

        self._mission = self.WAIT_CONN
        self._pending: rclpy.task.Future | None = None
        self._shutdown_requested = False

        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTSTP, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)

        z_ref = float(self.get_parameter('takeoff_alt').value)
        R = float(self.get_parameter('circle_radius').value)
        w = float(self.get_parameter('circle_w').value)

        self._xc = 0.0
        self._yc = 0.0
        self._R = R
        self._w = w
        self._laps = int(self.get_parameter('circle_laps').value)

        self._traj = CircleTrajectory(
            xc=self._xc, yc=self._yc, R=R, w=w, z_const=z_ref
        )
        self._gains = PDSMCGains()
        self._plant = QuadPlantParams()

        hz = float(self.get_parameter('control_rate_hz').value)
        self._Ts = 1.0 / max(hz, 1.0)
        self._control_t0: float | None = None
        self._circle_t0: float | None = None

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
            f'PDSMC circle node ready — R={R}m, w={w}rad/s, z={z_ref}m, '
            f'laps={self._laps}, rate={hz}Hz'
        )

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
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z
        phi_d, theta_d, psi_d = body_rates_from_euler_rates(
            self._phi, self._theta, p, q, r
        )
        self._phid = phi_d
        self._thetd = theta_d
        self._psid = psi_d

    def _state_vec(self) -> np.ndarray:
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
        req.command = 511
        req.param1 = float(stream_id)
        req.param2 = float(1_000_000 // max(rate_hz, 1))
        self._cmd_long.call_async(req)

    def _publish_attitude(
        self, roll: float, pitch: float, yaw: float, throttle: float
    ):
        now = self.get_clock().now().to_msg()
        att_msg = PoseStamped()
        att_msg.header.stamp = now
        att_msg.header.frame_id = 'map'
        att_msg.pose.orientation = euler_to_quat(roll, pitch, yaw)
        self._pub_attitude.publish(att_msg)

        thr_msg = Float64()
        thr_msg.data = float(throttle)
        self._pub_thrust.publish(thr_msg)

    def _publish_vel(self, vx: float, vy: float, vz: float = 0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self._pub_cmd.publish(msg)

    def _stop_vel(self):
        self._publish_vel(0.0, 0.0, 0.0)

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
            r.command = 22
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
                self._xc = self._px
                self._yc = self._py
                self._traj = CircleTrajectory(
                    xc=self._xc, yc=self._yc, R=self._R, w=self._w,
                    z_const=float(self.get_parameter('takeoff_alt').value)
                )
                self.get_logger().info(
                    f'Altitude reached z={self._pz:.2f}m, center=({self._xc:.2f},{self._yc:.2f}) — '
                    'flying to circle start...'
                )
                self._mission = self.FLY_TO_START
            return

        if self._mission == self.FLY_TO_START:
            tx = self._xc + self._R
            ty = self._yc
            dx = tx - self._px
            dy = ty - self._py
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.3:
                self._control_t0 = time.monotonic()
                self.get_logger().info(
                    f'Reached circle start ({tx:.2f},{ty:.2f}) — beginning PDSMC circle, '
                    f'laps={self._laps}'
                )
                self._mission = self.CIRCLE
                return

            speed = min(float(self.get_parameter('flyback_speed').value), dist)
            self._publish_vel(speed * dx / dist, speed * dy / dist)
            return

        if self._mission == self.CIRCLE:
            t_circle = time.monotonic() - (self._control_t0 or time.monotonic())

            SETTLE_SEC = 1.5
            if t_circle < SETTLE_SEC:
                self._publish_vel(0.0, 0.0)
                return

            t_run = t_circle - SETTLE_SEC
            theta = self._w * t_run

            # Pure feedforward velocity — giong circle_mission
            vx = -self._R * self._w * math.sin(theta)
            vy =  self._R * self._w * math.cos(theta)
            self._publish_vel(vx, vy)

            if int(t_run) % 5 == 0 and abs(t_run - int(t_run)) < 0.1:
                xd = self._xc + self._R * math.cos(theta)
                yd = self._yc + self._R * math.sin(theta)
                self.get_logger().info(
                    f'circle t={t_run:.1f}s | lap={theta/(2*math.pi):.2f}/{self._laps} | '
                    f'pos=({self._px:.2f},{self._py:.2f},{self._pz:.2f}) | '
                    f'ref=({xd:.2f},{yd:.2f}) | err=({xd-self._px:.2f},{yd-self._py:.2f}) | '
                    f'vx={vx:.2f} vy={vy:.2f}'
                )

            if theta / (2.0 * math.pi) >= self._laps:
                self.get_logger().info(
                    f'Circle complete ({self._laps} lap(s)) — flying back to start...'
                )
                self._stop_vel()
                self._mission = self.FLY_BACK
                self._control_t0 = None
                return

            self._try_shutdown_land()
            return

        if self._mission == self.FLY_BACK:
            dx = self._xc - self._px
            dy = self._yc - self._py
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.3:
                self.get_logger().info('Returned to start — landing...')
                self._stop_vel()
                self._mission = self.SEND_LAND
                return

            speed = min(float(self.get_parameter('flyback_speed').value), dist)
            self._publish_vel(speed * dx / dist, speed * dy / dist)
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
        import sys
        signame = signal.Signals(signum).name
        sys.stderr.write(f'Got {signame} — requesting LAND\n')
        sys.stderr.flush()
        self._shutdown_requested = True
        self._shutdown_land_requested = True

    def _try_shutdown_land(self):
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


def main():
    rclpy.init()
    node = PdsmcCircleMission()
    try:
        rclpy.spin(node)
    except RuntimeError:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
