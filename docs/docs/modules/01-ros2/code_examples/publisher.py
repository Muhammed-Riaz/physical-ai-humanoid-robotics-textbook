#!/usr/bin/env python3

"""
Simple ROS 2 publisher example for the Physical AI textbook
This publisher sends messages to control a humanoid robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanoidCommandPublisher(Node):

    def __init__(self):
        super().__init__('humanoid_command_publisher')
        self.publisher_ = self.create_publisher(String, 'humanoid_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Humanoid! Command number: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    humanoid_command_publisher = HumanoidCommandPublisher()

    rclpy.spin(humanoid_command_publisher)

    # Destroy the node explicitly
    humanoid_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()