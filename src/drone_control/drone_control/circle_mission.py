"""Fly an x500 quadrotter along a circle using UWB positioning.

Design notes
------------
The mission is flown entirely in **position control** (SET_POSITION_TARGET_LOCAL_NED
with a position setpoint) instead of velocity control.  This matters for two
reasons in this particular setup:

1.  **The EKF has no velocity observation.**  UWB positioning is fused through
    ``FuseRngBcn()``, which only consumes *ranges*.  There is no velocity
    source configured (``EK3_SRC1_VELXY = 0``), so the EKF's velocity state is
    a pure dead-reckoning estimate from the IMU.  In velocity control
    (``SubMode::VelAccel``) the position controller has no position P-term to
    reject that drift, so any position error accumulates until the vehicle
    tips over.  In position control the P-term actively pulls the vehicle back
    onto the setpoint, which is what makes the flight stable here.

2.  **Position control needs a heading to hold.**  ``setpoint_velocity``
    ignores yaw entirely, so ArduPilot keeps the heading it had at takeoff
    (``AutoYaw::Mode::HOLD``).  Flying a circular path with a fixed heading
    means the vehicle has to bank hard into the turn, which eventually exceeds
    ``ANGLE_MAX`` and flips it.  Here the yaw setpoint is commanded to always
    point at the circle centre, so the required bank angle stays small.

Frame conventions
-----------------
Everything published here is in the **ENU** frame that MAVROS expects:

* position ``(x, y, z)`` = (East, North, Up)
* yaw = heading in radians, counter-clockwise from the East axis

MAVROS converts this to the MAVLink NED frame before it reaches the
autopilot, and the same conversion is applied to the ``/mavros/local_position/pose``
that the mission reads back, so the two are consistent.  The UWB pipeline
uses a *different* convention on the wire (ArduPilot fuses the ODOMETRY
position as-is, in NED), which is handled inside ``uwb_pozyx_bridge``; see
``enu_to_ned_m`` in ``mavlink_odometry.py``.

Setpoint shape
--------------
Every setpoint carries a position, a velocity and a yaw, with the type mask
``0x1C0 | 0x800``.  That mask ignores the acceleration and the yaw rate but
keeps position, velocity and yaw, which puts ArduPilot into the
``SubMode::PosVelAccel`` guided sub-mode: the position P-term stays active
*and* the velocity acts as a feed-forward term.

The feed-forward velocity matters a lot here.  Whenever the position setpoint
jumps (for example straight to the far side of the circle) the position error
suddenly becomes large and the controller has to decelerate hard, which shows
up as a large pitch spike and can tip the vehicle over.  To avoid that, the
setpoint is never allowed to jump: a "carrot" is moved towards the goal at a
limited speed and the *carrot* is what gets published.  Because the carrot's
step shrinks as it approaches the goal, the feed-forward velocity ramps down
smoothly and the position error the autopilot sees stays small.
"""

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class CircleMission(Node):

    # Mission states
    WAIT_CONN = 0
    SEND_GUIDED = 1
    WAIT_GUIDED = 2
    SEND_ARM = 3
    WAIT_ARM = 4
    WAIT_STABLE = 15   # wait for arm confirmed + stabilisation delay
    SEND_TAKEOFF = 5
    WAIT_TAKEOFF = 6
    CLIMBING = 7
    FLY_TO_START = 8
    CIRCLE = 9
    FLY_TO_CENTER = 10
    SEND_LAND = 11
    WAIT_LAND = 12
    WAIT_DISARM = 13
    DONE = 14

    # Type mask: use position, velocity and yaw; ignore acceleration and yaw rate.
    #   bits 6-8   (0x1C0) -> AX/AY/AZ ignored
    #   bit  11    (0x800) -> yaw rate ignored
    #
    # Bits 3-5 (VX/VY/VZ ignore) are deliberately NOT set: the velocity is
    # used as a feed-forward term so the vehicle follows the circle smoothly
    # instead of lagging behind the position setpoint.
    TYPE_MASK_POS_VEL_YAW = 0x1C0 | 0x800

    #: Time constant (s) of the low-pass filter applied to the feed-forward
    #: velocity.  Short enough to track the circle, long enough to smooth
    #: the step when the mission switches between legs.
    FF_TAU = 0.25

    def __init__(self):

        super().__init__('circle_mission')

        self.declare_parameter('save_log', True)
        self.save_log = self.get_parameter('save_log').get_parameter_value().bool_value

        self.declare_parameter('radius', 1.5)
        self.radius = float(self.get_parameter('radius').value)

        self.declare_parameter('omega', 0.3)
        self.w = float(self.get_parameter('omega').value)

        self.declare_parameter('altitude', 1.0)
        self.altitude = float(self.get_parameter('altitude').value)

        # Feed-forward speed limit (m/s) used while flying to the circle
        # start and back to the centre.  Kept modest so the vehicle does not
        # have to bank hard to change direction.
        self.declare_parameter('max_speed', 0.8)
        self.max_speed = float(self.get_parameter('max_speed').value)

        # Proportional gain used to derive the feed-forward speed from the
        # remaining distance, so the vehicle slows down as it arrives.
        self.declare_parameter('speed_gain', 1.0)
        self.speed_gain = float(self.get_parameter('speed_gain').value)

        # QoS profile for MAVROS compatibility.
        # MAVROS subscribes to the setpoint topics with SensorDataQoS
        # (best effort, keep last 5), so a best-effort publisher is
        # compatible and a reliable one would not be delivered reliably.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', qos_profile
        )
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)

        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pos_callback,
            qos_profile
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        self.planned_path = Path()
        self.planned_path.header.frame_id = "map"
        self.actual_path = Path()
        self.actual_path.header.frame_id = "map"

        # drone state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_state = None

        # vertical speed, derived from consecutive position fixes
        self.vz = 0.0
        self._last_z = None
        self._last_time = None

        # circle center
        self.x0 = None
        self.y0 = None

        # takeoff origin
        self.takeoff_z = 0.0
        self.target_z = 0.0

        # the moving setpoint ("carrot") used for the straight-line legs
        self.carrot_x = 0.0
        self.carrot_y = 0.0

        # smoothed feed-forward velocity
        self.ff_vx = 0.0
        self.ff_vy = 0.0

        # mission state machine
        self.mission_state = self.WAIT_CONN
        self.pending_future = None
        self.start_time = None
        self.arm_time = None
        self.ARM_STABLE_SEC = 2.0   # seconds to wait after arm before takeoff

        # MAVROS services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # Wait for services to be available
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')

        self.timer = self.create_timer(0.05, self.timer_callback)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def local_pos_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

        # Vertical speed, from the difference between two consecutive fixes.
        now = self.get_clock().now()
        if self._last_time is not None:
            dt = (now - self._last_time).nanoseconds / 1e9
            if dt > 1e-3:
                self.vz = (self.current_z - self._last_z) / dt
        self._last_time = now
        self._last_z = self.current_z

        if self.mission_state >= self.CLIMBING and self.mission_state <= self.WAIT_DISARM:
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = "map"
            pose.pose = msg.pose
            self.actual_path.poses.append(pose)
            self.actual_path.header.stamp = msg.header.stamp
            self.actual_path_pub.publish(self.actual_path)

    def state_callback(self, msg):
        self.current_state = msg

    def save_paths_to_csv(self):
        home = os.path.expanduser('~')
        ts = time.strftime('%Y%m%d_%H%M%S')

        planned_file = os.path.join(home, f'planned_path_{ts}.csv')
        with open(planned_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z'])
            for pose in self.planned_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ])
        self.get_logger().info(f"Planned path saved: {planned_file} ({len(self.planned_path.poses)} points)")

        actual_file = os.path.join(home, f'actual_path_{ts}.csv')
        with open(actual_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_sec', 'time_nsec', 'x', 'y', 'z'])
            for pose in self.actual_path.poses:
                writer.writerow([
                    pose.header.stamp.sec,
                    pose.header.stamp.nanosec,
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ])
        self.get_logger().info(f"Actual path saved: {actual_file} ({len(self.actual_path.poses)} points)")

    def add_planned(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.planned_path.poses.append(pose)
        self.planned_path.header.stamp = pose.header.stamp
        self.planned_path_pub.publish(self.planned_path)

    def set_stream_rate(self, stream_id, rate):
        req = CommandLong.Request()
        req.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(stream_id)  # message ID
        req.param2 = float(1000000 // rate)  # interval in microseconds
        self.takeoff_client.call_async(req)

    def publish_setpoint(self, x, y, z, yaw, vx=0.0, vy=0.0, vz=0.0):
        """Publish a local position + velocity + yaw setpoint.

        ``/mavros/setpoint_raw/local`` expects the position and velocity in the
        local ENU frame and the yaw as a heading angle (radians,
        counter-clockwise from the local X axis).  MAVROS converts the frame
        to NED and the yaw to the autopilot's convention before sending it.

        The coordinate frame is ``FRAME_LOCAL_NED`` (1): the message content is
        ENU but the frame identifier tells the autopilot the values are
        expressed in a local, north-east-down reference, which is what
        ArduPilot's GUIDED position controller expects.

        Sending both a position and a velocity makes ArduPilot use its
        ``PosVelAccel`` guided sub-mode.  That mode keeps the position
        P-term active (so a position error is actively corrected) while the
        velocity provides the feed-forward that makes the vehicle track the
        circle without lagging behind it.

        The yaw is not ignored (bit 10 is clear), so the yaw field is the
        active heading command.
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = self.TYPE_MASK_POS_VEL_YAW

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        msg.velocity.x = float(vx)
        msg.velocity.y = float(vy)
        msg.velocity.z = float(vz)

        msg.yaw = float(yaw)

        self.pub.publish(msg)

    @staticmethod
    def heading_to(target_x, target_y, from_x, from_y):
        """Heading (rad) from (from_x, from_y) towards (target_x, target_y)."""
        return math.atan2(target_y - from_y, target_x - from_x)

    def advance_carrot(self, goal_x, goal_y, dt):
        """Move the carrot towards the goal and return its velocity.

        The carrot is the position that gets published as the setpoint.  It
        creeps towards the goal at no more than ``max_speed``, and the step
        shrinks as the goal gets closer, so the feed-forward velocity ramps
        down smoothly instead of the setpoint jumping and forcing a hard
        braking manoeuvre.

        Returns ``(vx, vy)`` in m/s.
        """
        dx = goal_x - self.carrot_x
        dy = goal_y - self.carrot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6 or dt <= 0.0:
            return 0.0, 0.0

        step = min(self.max_speed * dt, dist)
        ux = dx / dist
        uy = dy / dist
        self.carrot_x += ux * step
        self.carrot_y += uy * step
        return ux * step / dt, uy * step / dt

    def blend_velocity(self, vx, vy, dt):
        """Low-pass the feed-forward velocity and return the smoothed value.

        The feed-forward velocity changes abruptly when the mission switches
        from one leg to the next: the vehicle flies in towards the circle
        start and then has to follow the circle, and those two velocities
        point in different directions.  Feeding the raw value to the
        autopilot makes it swerve at the transition, which shows up as a
        sharp roll or pitch spike.

        Smoothing the velocity over a short time constant removes that
        step without noticeably lagging the trajectory, because the
        position setpoint itself is still tracked by the P-term.
        """
        if dt <= 0.0:
            return self.ff_vx, self.ff_vy
        alpha = min(1.0, dt / self.FF_TAU)
        self.ff_vx += alpha * (vx - self.ff_vx)
        self.ff_vy += alpha * (vy - self.ff_vy)
        return self.ff_vx, self.ff_vy

    # ------------------------------------------------------------------
    # mission state machine
    # ------------------------------------------------------------------

    def timer_callback(self):

        # --- WAIT_CONN: wait for MAVROS connection ---
        if self.mission_state == self.WAIT_CONN:
            if self.current_state and self.current_state.connected:
                self.get_logger().info("MAVROS connected")
                # LOCAL_POSITION_NED (#32) at 20Hz
                self.set_stream_rate(32, 20)
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_GUIDED ---
        if self.mission_state == self.SEND_GUIDED:
            req = SetMode.Request()
            req.custom_mode = "GUIDED"
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info("Switching to GUIDED mode...")
            self.mission_state = self.WAIT_GUIDED
            return

        # --- WAIT_GUIDED: check response ---
        if self.mission_state == self.WAIT_GUIDED:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info("GUIDED mode accepted")
                self.mission_state = self.SEND_ARM
            else:
                self.get_logger().warn("GUIDED mode rejected, retrying...")
                self.mission_state = self.SEND_GUIDED
            return

        # --- SEND_ARM ---
        if self.mission_state == self.SEND_ARM:
            req = CommandBool.Request()
            req.value = True
            self.pending_future = self.arming_client.call_async(req)
            self.get_logger().info("Arming vehicle...")
            self.mission_state = self.WAIT_ARM
            return

        # --- WAIT_ARM: check response ---
        if self.mission_state == self.WAIT_ARM:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.get_logger().info("Arming accepted, waiting for FCU arm confirmation + stabilisation...")
                self.arm_time = None   # will be set once State.armed is True
                self.mission_state = self.WAIT_STABLE
            else:
                self.get_logger().warn("Arming rejected, retrying...")
                self.mission_state = self.SEND_ARM
            return

        # --- WAIT_STABLE: wait until FCU confirms armed AND stabilisation delay passes ---
        if self.mission_state == self.WAIT_STABLE:
            if self.current_state and self.current_state.armed:
                if self.arm_time is None:
                    self.arm_time = self.get_clock().now()
                    self.get_logger().info(f"FCU armed confirmed, stabilising for {self.ARM_STABLE_SEC:.1f}s...")
                elapsed = (self.get_clock().now() - self.arm_time).nanoseconds / 1e9
                if elapsed >= self.ARM_STABLE_SEC:
                    self.get_logger().info("Stabilisation done, proceeding to takeoff")
                    self.mission_state = self.SEND_TAKEOFF
            else:
                # State not yet armed — reset timer in case of brief blip
                self.arm_time = None
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
            # param7 is the target altitude in metres, relative to home.
            #
            # It has to match the altitude the mission itself commands.
            # ArduPilot's own take-off logic keeps climbing towards this
            # height, and if it is higher than the mission setpoint the two
            # controllers fight each other: the vehicle overshoots to
            # param7, pitches over to stop the climb, and then has to
            # descend again.  That is exactly the behaviour that used to
            # make the drone lurch and tip over right after take-off.
            req.param7 = float(self.altitude)
            self.pending_future = self.takeoff_client.call_async(req)
            self.get_logger().info("Sending takeoff command...")
            self.mission_state = self.WAIT_TAKEOFF
            return

        # --- WAIT_TAKEOFF: check response ---
        if self.mission_state == self.WAIT_TAKEOFF:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.success:
                self.takeoff_z = self.current_z
                self.target_z = self.takeoff_z + self.altitude
                self.carrot_x = self.current_x
                self.carrot_y = self.current_y
                self.get_logger().info(f"Takeoff accepted, climbing to {self.target_z:.1f}m...")
                self.add_planned(self.current_x, self.current_y, self.target_z)
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn("Takeoff rejected, retrying...")
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING: wait until the autopilot has finished its take-off ---
        if self.mission_state == self.CLIMBING:
            # No setpoint is published here on purpose.  ArduPilot's own
            # take-off logic is what actually lifts the vehicle off the
            # ground, and it only runs while the guided sub-mode is
            # ``TakeOff``.  Publishing a position setpoint would switch the
            # sub-mode to ``PosVelAccel``, which bails out while the vehicle
            # is still on the ground and the drone would never take off.
            #
            # The take-off height was given to the autopilot in param7 and
            # matches the mission altitude, so the vehicle settles at the
            # mission altitude without overshooting it.
            if self.current_z >= self.target_z - 0.05 and abs(self.vz) < 0.15:
                self.x0 = self.current_x
                self.y0 = self.current_y
                self.get_logger().info(
                    f"Altitude reached, center=({self.x0:.2f}, {self.y0:.2f}), flying to circle start..."
                )
                self.mission_state = self.FLY_TO_START
            return

        # --- FLY_TO_START: fly to (x0+R, y0) so takeoff point is circle center ---
        if self.mission_state == self.FLY_TO_START:
            tx = self.x0 + self.radius
            ty = self.y0

            dt = 0.05
            vx, vy = self.advance_carrot(tx, ty, dt)
            vx, vy = self.blend_velocity(vx, vy, dt)

            # Face the circle centre while flying in: keep the bank angle small.
            yaw = self.heading_to(self.x0, self.y0, tx, ty)

            self.publish_setpoint(self.carrot_x, self.carrot_y, self.target_z, yaw, vx, vy)

            if math.hypot(self.current_x - tx, self.current_y - ty) < 0.3:
                self.start_time = time.time()
                self.get_logger().info("Reached circle start, beginning circle")
                self.add_planned(tx, ty, self.target_z)
                self.mission_state = self.CIRCLE
            return

        # --- CIRCLE: fly circle ---
        if self.mission_state == self.CIRCLE:
            t = time.time() - self.start_time
            angle = self.w * t

            if angle >= 2 * math.pi:
                # planned landing trajectory
                steps = int(self.target_z / 0.1)
                for z in [self.target_z - i * 0.1 for i in range(steps)]:
                    self.add_planned(self.x0, self.y0, z)

                self.get_logger().info("Circle completed, flying back to center...")
                self.add_planned(self.x0, self.y0, self.target_z)
                # Start the return leg from where the circle ended, so the
                # setpoint does not jump across the circle.
                self.carrot_x = self.current_x
                self.carrot_y = self.current_y
                self.mission_state = self.FLY_TO_CENTER
                return

            # Position setpoint on the circle.
            x = self.x0 + self.radius * math.cos(angle)
            y = self.y0 + self.radius * math.sin(angle)

            # Tangential feed-forward velocity, scaled to the circle speed.
            vx = -self.radius * self.w * math.sin(angle)
            vy = self.radius * self.w * math.cos(angle)
            vx, vy = self.blend_velocity(vx, vy, 0.05)

            # Always look at the circle centre: the tangential component of
            # the required thrust is then zero and the vehicle only has to
            # bank slightly, instead of holding a fixed heading and
            # over-banking through the turn.
            yaw = self.heading_to(self.x0, self.y0, x, y)
            self.publish_setpoint(x, y, self.target_z, yaw, vx, vy)

            self.add_planned(x, y, self.target_z)
            return

        # --- FLY_TO_CENTER: return to circle center before landing ---
        if self.mission_state == self.FLY_TO_CENTER:
            dt = 0.05
            vx, vy = self.advance_carrot(self.x0, self.y0, dt)
            vx, vy = self.blend_velocity(vx, vy, dt)

            # Face the direction of travel.  At the end of the circle the
            # vehicle already faces the centre, so this is continuous with
            # the circle phase and does not cause a yaw step.
            if math.hypot(vx, vy) > 1e-3:
                yaw = math.atan2(vy, vx)
            else:
                yaw = self.heading_to(self.x0, self.y0, self.current_x, self.current_y)

            self.publish_setpoint(self.carrot_x, self.carrot_y, self.target_z, yaw, vx, vy)

            if math.hypot(self.current_x - self.x0, self.current_y - self.y0) < 0.3:
                self.get_logger().info("Reached center, landing...")
                self.mission_state = self.SEND_LAND
            return

        # --- SEND_LAND ---
        if self.mission_state == self.SEND_LAND:
            req = SetMode.Request()
            req.custom_mode = "LAND"
            self.pending_future = self.set_mode_client.call_async(req)
            self.get_logger().info("Sending LAND command...")
            self.mission_state = self.WAIT_LAND
            return

        # --- WAIT_LAND: check response ---
        if self.mission_state == self.WAIT_LAND:
            if not self.pending_future.done():
                return
            resp = self.pending_future.result()
            if resp and resp.mode_sent:
                self.get_logger().info("LAND mode accepted, waiting for drone to land...")
                self.mission_state = self.WAIT_DISARM
            else:
                self.get_logger().warn("LAND mode rejected, retrying...")
                self.mission_state = self.SEND_LAND
            return

        # --- WAIT_DISARM: wait until drone actually lands and disarms ---
        if self.mission_state == self.WAIT_DISARM:
            if self.current_state and not self.current_state.armed:
                self.get_logger().info("Drone disarmed, landing complete.")
                if self.save_log:
                    self.get_logger().info("Saving paths...")
                    self.save_paths_to_csv()
                self.mission_state = self.DONE
            return


def main():

    rclpy.init()

    node = CircleMission()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
