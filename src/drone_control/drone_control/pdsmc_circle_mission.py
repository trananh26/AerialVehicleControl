# PDSMC circle mission — bay hinh tron roi quay ve vi tri cu
#
# Kien truc: PDSMC (port MATLAB goc) cho CIRCLE, velocity control cho cac
#   phan con lai. Cu phap, state machine, Qos, timing giong y
#   circle_mission.py (da test OK tren mo phong va thuc nghiem).
#
# Tac vu: GUIDED -> ARM -> TAKEOFF -> CLIMBING -> FLY_TO_START (vel) ->
#         CIRCLE (PDSMC attitude) -> FLY_TO_CENTER (vel) -> LAND

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


MAV_CMD_NAV_LAND = 21


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
    FLY_TO_CENTER = 10
    SEND_LAND     = 11
    WAIT_LAND     = 12
    WAIT_DISARM   = 13
    DONE          = 14

    def __init__(self):

        super().__init__('pdsmc_circle_mission')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos,
        )
        self.pub_attitude = self.create_publisher(
            PoseStamped, '/mavros/setpoint_attitude/attitude', qos,
        )
        self.pub_thrust = self.create_publisher(
            Float64, '/mavros/setpoint_attitude/accel_thrust_throttle', qos,
        )
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)

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

        self.planned_path = Path()
        self.planned_path.header.frame_id = 'map'
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_state: State | None = None

        self.x0: float | None = None
        self.y0: float | None = None
        self.radius = 3.0
        self.w = 0.3

        self.start_time: float | None = None
        self.imu_valid = False
        self._phi = 0.0
        self._theta = 0.0
        self._psi = 0.0
        self._phid = 0.0
        self._thetd = 0.0
        self._psid = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0

        self.mission_state = self.WAIT_CONN
        self.pending_future: rclpy.task.Future | None = None

        self._stop_publishing = False

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

        self.get_logger().info(
            f'PDSMC circle node ready — R={self.radius}m, w={self.w}rad/s'
        )

    # --- callbacks ---
    def local_pos_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
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
        self.imu_valid = True
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

    def _stop_pub(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        self.pub_cmd.publish(msg)

    def _pub_vel(self, vx: float, vy: float):
        if self._stop_publishing:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        self.pub_cmd.publish(msg)

    def _pub_attitude(self, roll: float, pitch: float, yaw: float, throttle: float):
        if self._stop_publishing:
            return
        now = self.get_clock().now().to_msg()
        att_msg = PoseStamped()
        att_msg.header.stamp = now
        att_msg.header.frame_id = 'map'
        att_msg.pose.orientation = euler_to_quat(roll, pitch, yaw)
        self.pub_attitude.publish(att_msg)
        thr_msg = Float64()
        thr_msg.data = float(throttle)
        self.pub_thrust.publish(thr_msg)

    def _pub_vel_with_throttle(self, vx: float, vy: float, throttle: float):
        """Velocity cmd_vel for horizontal + attitude/throttle for altitude."""
        if self._stop_publishing:
            return
        now = self.get_clock().now().to_msg()
        # Horizontal: velocity command
        vel_msg = TwistStamped()
        vel_msg.header.stamp = now
        vel_msg.header.frame_id = 'map'
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = 0.0
        self.pub_cmd.publish(vel_msg)
        # Altitude: throttle override via attitude topic
        att_msg = PoseStamped()
        att_msg.header.stamp = now
        att_msg.header.frame_id = 'map'
        att_msg.pose.orientation = euler_to_quat(0.0, 0.0, self._psi)
        self.pub_attitude.publish(att_msg)
        thr_msg = Float64()
        thr_msg.data = float(throttle)
        self.pub_thrust.publish(thr_msg)

    def _state_vec(self) -> np.ndarray:
        return np.array(
            [
                self.current_x, self._vx,
                self.current_y, self._vy,
                self.current_z, self._vz,
                self._phi, self._phid,
                self._theta, self._thetd,
                self._psi, self._psid,
            ],
            dtype=float,
        )

    def _pdsmc_circle_step(self, xd: float, yd: float, zd: float,
                            xd_d: float, yd_d: float) -> tuple[float, float, float]:
        """Tinh phides, thetades, throttle cho CIRCLE bang PDSMC."""
        m = 0.8
        g = 9.81
        kpx, kdx, Hx, lamx = 1.5, 1.0, 0.5, 0.5
        kpy, kdy, Hy, lamy = 1.5, 1.0, 0.5, 0.5
        kp1, kd1, H1, lam1 = 100.0, 40.0, 160.0, 100.0

        ez   = zd - self.current_z
        ez_d = -self._vz
        U1_raw = m * g + kp1 * ez + kd1 * ez_d + H1 * math.tanh(ez_d + lam1 * ez)
        U1 = max(0.3 * m * g, U1_raw)
        throttle = float(np.clip(U1 / (m * 15.0), 0.0, 1.0))

        ex   = xd - self.current_x
        ex_d = xd_d - self._vx
        ey   = yd - self.current_y
        ey_d = yd_d - self._vy

        Ux = kpx * ex + kdx * ex_d + Hx * math.tanh(ex_d + lamx * ex)
        Uy = kpy * ey + kdy * ey_d + Hy * math.tanh(ey_d + lamy * ey)

        T_norm = max(U1 / m, 1e-6)
        sinphi = (Ux * math.sin(self._psi) - Uy * math.cos(self._psi)) / T_norm
        phides = math.asin(float(np.clip(sinphi, -1.0, 1.0)))
        sintheta = (Ux * math.cos(self._psi) + Uy * math.sin(self._psi)) / (
            T_norm * max(math.cos(phides), 1e-6)
        )
        thetades = math.asin(float(np.clip(sintheta, -1.0, 1.0)))
        return phides, thetades, throttle

    def add_planned(self, x: float, y: float, z: float):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.planned_path.poses.append(pose)
        self.planned_path.header.stamp = pose.header.stamp
        self.planned_path_pub.publish(self.planned_path)

    def save_paths_to_csv(self):
        home = os.path.expanduser('~')
        ts = time.strftime('%Y%m%d_%H%M%S')

        planned_file = os.path.join(home, f'planned_path_pdsmc_circle_{ts}.csv')
        with open(planned_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z'])
            for pose in self.planned_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ])
        self.get_logger().info(
            f'Planned path saved: {planned_file} ({len(self.planned_path.poses)} pts)',
        )

        actual_file = os.path.join(home, f'actual_path_pdsmc_circle_{ts}.csv')
        with open(actual_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z'])
            for pose in self.actual_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ])
        self.get_logger().info(
            f'Actual path saved: {actual_file} ({len(self.actual_path.poses)} pts)',
        )

    # --- state machine ---
    def timer_callback(self):

        # --- WAIT_CONN ---
        if self.mission_state == self.WAIT_CONN:
            if self.current_state and self.current_state.connected:
                self.get_logger().info('MAVROS connected')
                self.set_stream_rate(32, 20)
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
            self.mission_state = self.WAIT_ARM
            return

        # --- WAIT_ARM ---
        if self.mission_state == self.WAIT_ARM:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.get_logger().info('Arming accepted')
                self.mission_state = self.SEND_TAKEOFF
            else:
                self.get_logger().warn('Arming rejected, retrying...')
                self.mission_state = self.SEND_ARM
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
                self.get_logger().info(
                    f'Takeoff accepted, climbing to 3.0m...',
                )
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn('Takeoff rejected, retrying...')
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING ---
        if self.mission_state == self.CLIMBING:
            if self.current_z >= 3.0:
                if not self.imu_valid:
                    self.get_logger().warn('IMU not ready — waiting...')
                    return
                self.x0 = self.current_x
                self.y0 = self.current_y
                self.get_logger().info(
                    f'Altitude reached, center=({self.x0:.2f}, {self.y0:.2f}), '
                    'flying to circle start...',
                )
                self.mission_state = self.FLY_TO_START
            return

        # --- FLY_TO_START ---
        if self.mission_state == self.FLY_TO_START:
            tx = self.x0 + self.radius
            ty = self.y0
            dx = tx - self.current_x
            dy = ty - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.3:
                self.start_time = time.time()
                self.get_logger().info('Reached circle start, beginning PDSMC circle')
                self.add_planned(tx, ty, 3.0)
                self.mission_state = self.CIRCLE
                return

            speed = min(1.5, dist)
            self._pub_vel(speed * dx / dist, speed * dy / dist)
            return

        # --- CIRCLE: velocity for horizontal + PDSMC altitude throttle override ---
        if self.mission_state == self.CIRCLE:
            t = time.time() - self.start_time
            angle = self.w * t

            xd = self.x0 + self.radius * math.cos(angle)
            yd = self.y0 + self.radius * math.sin(angle)
            xd_d = -self.radius * self.w * math.sin(angle)
            yd_d =  self.radius * self.w * math.cos(angle)

            if angle >= 2.0 * math.pi:
                self._pub_vel(0.0, 0.0)
                self.get_logger().info('Circle completed, flying back to center...')
                self.add_planned(self.x0, self.y0, 3.0)
                self.mission_state = self.FLY_TO_CENTER
                return

            # PDSMC altitude throttle (override z in velocity command)
            if self.imu_valid:
                m = 0.8
                g = 9.81
                kp1, kd1, H1, lam1 = 100.0, 40.0, 160.0, 100.0
                ez   = 3.0 - self.current_z
                ez_d = -self._vz
                U1_raw = m * g + kp1 * ez + kd1 * ez_d + H1 * math.tanh(ez_d + lam1 * ez)
                U1 = max(0.3 * m * g, U1_raw)
                throttle = float(np.clip(U1 / (m * 15.0), 0.0, 1.0))
                self._pub_vel_with_throttle(xd_d, yd_d, throttle)
            else:
                self._pub_vel(xd_d, yd_d)

            self.add_planned(xd, yd, 3.0)
            return

        # --- FLY_TO_CENTER ---
        if self.mission_state == self.FLY_TO_CENTER:
            dx = self.x0 - self.current_x
            dy = self.y0 - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.3:
                self._stop_pub()
                self.get_logger().info('Reached center, landing...')
                self.mission_state = self.SEND_LAND
                return

            speed = min(1.5, dist)
            self._pub_vel(speed * dx / dist, speed * dy / dist)
            return

        # --- SEND_LAND ---
        if self.mission_state == self.SEND_LAND:
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info('Sending LAND command...')
            self.mission_state = self.WAIT_LAND
            return

        # --- WAIT_LAND ---
        if self.mission_state == self.WAIT_LAND:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info('LAND mode accepted, waiting for drone to land...')
                self.mission_state = self.WAIT_DISARM
            else:
                self.get_logger().warn('LAND mode rejected, retrying...')
                self.mission_state = self.SEND_LAND
            return

        # --- WAIT_DISARM ---
        if self.mission_state == self.WAIT_DISARM:
            if self.current_state and not self.current_state.armed:
                self.get_logger().info('Drone disarmed, landing complete. Saving paths...')
                self.save_paths_to_csv()
                self.mission_state = self.DONE
            return


def main():
    rclpy.init()
    node = PdsmcCircleMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
