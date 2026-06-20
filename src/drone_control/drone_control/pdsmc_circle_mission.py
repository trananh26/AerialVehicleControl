# PDSMC circle mission — bay hinh tron roi quay ve vi tri cu
#
# Kien truc: PDSMC (PD+smc) cho CIRCLE, velocity control cho cac
#   phan con lai. Cu phap, state machine, Qos, timing giong y
#   circle_mission.py (da test OK tren mo phong va thuc nghiem).
#
# Tac vu: GUIDED -> ARM -> TAKEOFF -> CLIMBING -> FLY_TO_START (vel) ->
#         CIRCLE (PDSMC vel correction) -> FLY_TO_CENTER (vel) -> LAND

from __future__ import annotations

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


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

        self.planned_path = Path()
        self.planned_path.header.frame_id = 'map'
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'map'

        # drone state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self.current_state: State | None = None

        # circle parameters
        self.x0: float | None = None
        self.y0: float | None = None
        self.radius = 3.0
        self.w = 0.3

        # mission timing
        self.start_time: float | None = None

        # mission state machine
        self.mission_state = self.WAIT_CONN
        self.pending_future: rclpy.task.Future | None = None

        # MAVROS services
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
            f'PDSMC circle node ready — R={self.radius}m, w={self.w}rad/s',
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

    def state_callback(self, msg: State):
        self.current_state = msg

    def vel_callback(self, msg: TwistStamped):
        self._vx = msg.twist.linear.x
        self._vy = msg.twist.linear.y

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
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        self.pub_cmd.publish(msg)

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
            req.command = 22  # MAV_CMD_NAV_TAKEOFF
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
                self.add_planned(self.current_x, self.current_y, 3.0)
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn('Takeoff rejected, retrying...')
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING ---
        if self.mission_state == self.CLIMBING:
            if self.current_z >= 3.0:
                self.x0 = self.current_x
                self.y0 = self.current_y
                self.get_logger().info(
                    f'Altitude reached, center=({self.x0:.2f}, {self.y0:.2f}), '
                    f'flying to circle start...',
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
                self.get_logger().info('Reached circle start, beginning circle')
                self._stop_pub()
                self.add_planned(tx, ty, 3.0)
                self.mission_state = self.CIRCLE
                return

            speed = min(1.5, dist)
            self._pub_vel(speed * dx / dist, speed * dy / dist)
            return

        # --- CIRCLE ---
        if self.mission_state == self.CIRCLE:
            t = time.time() - self.start_time
            angle = self.w * t

            if angle >= 2.0 * math.pi:
                self._stop_pub()

                # planned landing trajectory: descend stepwise to ground
                for z in [3.0 - i * 0.1 for i in range(int(3.0 / 0.1))]:
                    self.add_planned(self.x0, self.y0, z)

                self.get_logger().info('Circle completed, flying back to center...')
                self.add_planned(self.x0, self.y0, 3.0)
                self.mission_state = self.FLY_TO_CENTER
                return

            # Desired position/velocity on circle (feedforward)
            xd = self.x0 + self.radius * math.cos(angle)
            yd = self.y0 + self.radius * math.sin(angle)
            xd_d = -self.radius * self.w * math.sin(angle)
            yd_d =  self.radius * self.w * math.cos(angle)

            # PDSMC-like position/velocity feedback → velocity correction in m/s
            kpx, kdx, Hx, lamx = 0.5, 0.3, 0.1, 0.5
            kpy, kdy, Hy, lamy = 0.5, 0.3, 0.1, 0.5

            ex   = xd - self.current_x
            ey   = yd - self.current_y
            ex_d = xd_d - self._vx
            ey_d = yd_d - self._vy

            dvx = kpx * ex + kdx * ex_d + Hx * math.tanh(lamx * ex + ex_d)
            dvy = kpy * ey + kdy * ey_d + Hy * math.tanh(lamy * ey + ey_d)

            # Feedforward velocity (from circle dynamics) + PDSMC correction
            vx_cmd = xd_d + dvx
            vy_cmd = yd_d + dvy

            self._pub_vel(vx_cmd, vy_cmd)
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
