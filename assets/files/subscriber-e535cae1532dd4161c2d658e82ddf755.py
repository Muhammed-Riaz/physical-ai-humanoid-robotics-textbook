#!/usr/bin/env python3

"""
Simple ROS 2 subscriber example for the Physical AI textbook
This subscriber receives messages to control a humanoid robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanoidCommandSubscriber(Node):

    def __init__(self):
        super().__init__('humanoid_command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'humanoid_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    humanoid_command_subscriber = HumanoidCommandSubscriber()

    rclpy.spin(humanoid_command_subscriber)

    # Destroy the node explicitly
    humanoid_command_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()