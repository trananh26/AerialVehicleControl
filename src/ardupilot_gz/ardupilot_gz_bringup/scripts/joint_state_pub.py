#!/usr/bin/env python3
"""Publish zero joint states for all revolute joints at 10 Hz."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePub(Node):
    def __init__(self):
        super().__init__("joint_state_pub")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.names = [
            "imu_joint",
            "rotor_0_joint",
            "rotor_1_joint",
            "rotor_2_joint",
            "rotor_3_joint",
        ]
        self.create_timer(0.1, self.publish)

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [0.0] * len(self.names)
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(JointStatePub())


if __name__ == "__main__":
    main()
