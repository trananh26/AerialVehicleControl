import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong

class TakeoffLandMission(Node):

    # Mission states
    WAIT_CONN = 0
    SEND_GUIDED = 1
    WAIT_GUIDED = 2
    SEND_ARM = 3
    WAIT_ARM = 4
    SEND_TAKEOFF = 5
    WAIT_TAKEOFF = 6
    CLIMBING = 7
    SEND_LAND = 8
    WAIT_LAND = 9
    DONE = 10

    def __init__(self):

        super().__init__('takeoff_land_mission')

        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

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

        # drone state
        self.current_z = 0.0
        self.current_state = None

        # takeoff origin
        self.takeoff_z = 0.0
        self.target_z = 0.0

        # mission state machine
        self.mission_state = self.WAIT_CONN
        self.pending_future = None
        self.start_time = None

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


    def local_pos_callback(self, msg):
        self.current_z = msg.pose.position.z

    def state_callback(self, msg):
        self.current_state = msg


    def set_stream_rate(self, stream_id, rate):
        req = CommandLong.Request()
        req.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(stream_id)  # message ID
        req.param2 = float(1000000 // rate)  # interval in microseconds
        self.takeoff_client.call_async(req)


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
                self.get_logger().info("Arming accepted")
                self.mission_state = self.SEND_TAKEOFF
            else:
                self.get_logger().warn("Arming rejected, retrying...")
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
            req.param7 = 1.5
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
                self.target_z = self.takeoff_z + 1.5
                self.get_logger().info(f"Takeoff accepted, climbing to {self.target_z:.1f}m...")
                self.mission_state = self.CLIMBING
            else:
                self.get_logger().warn("Takeoff rejected, retrying...")
                self.mission_state = self.SEND_TAKEOFF
            return

        # --- CLIMBING: wait until altitude reached ---
        if self.mission_state == self.CLIMBING:
            if self.current_z >= self.target_z:
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
                self.get_logger().info("LAND mode accepted, mission complete")
                self.mission_state = self.DONE
            else:
                self.get_logger().warn("LAND mode rejected, retrying...")
                self.mission_state = self.SEND_LAND
            return

def main():

    rclpy.init()

    node = TakeoffLandMission()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()