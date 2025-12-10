#!/usr/bin/env python3

"""
Autonomous Simulated Humanoid Control Code for Capstone Project
This node provides high-level control functions for navigation, object interaction, and speaking.
It interacts with Nav2 for navigation, and potentially a separate node for manipulation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
import time

class HumanoidController(Node):

    def __init__(self):
        super().__init__('humanoid_controller')

        # Navigation Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigation action server not available, waiting...')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # For direct movement
        self.speech_publisher = self.create_publisher(String, '/robot_speech', 10) # For robot speech

        self.get_logger().info('Humanoid Controller Node Started')

    def go_to_pose(self, x, y, yaw=0.0):
        """Send a navigation goal to Nav2 to move to a specific (x, y) pose with a yaw orientation."""
        self.get_logger().info(f'Sending navigation goal to: ({x}, {y}, {yaw})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # Assuming 'map' as the global frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = float(yaw) # Simplified yaw for now
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! %s' % result)
        else:
            self.get_logger().info('Goal failed with status: {0}' % status)

    def stop_robot(self):
        """Stop all robot motion immediately."""
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

    def speak(self, text):
        """Make the robot speak a given text message."""
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)
        self.get_logger().info(f'Robot speaking: "{text}"')

    def perform_grasp(self, object_name):
        """Simulate a grasping action for a specified object."""
        self.get_logger().info(f'Simulating grasping of: {object_name}')
        # In a real system, this would trigger a manipulation action server
        time.sleep(2) # Simulate manipulation time
        self.speak(f'I have grasped the {object_name}')

    def navigate_and_grasp(self, x, y, obj_name):
        """Combined navigation and grasping action."""
        self.get_logger().info(f'Navigating to ({x}, {y}) to grasp {obj_name}')
        self.go_to_pose(x, y)
        # Await navigation completion (more robust logic would be needed here)
        time.sleep(5) # Simulate waiting for navigation
        self.perform_grasp(obj_name)


def main(args=None):
    rclpy.init(args=args)

    controller = HumanoidController()

    # Example usage (can be called from VLA pipeline)
    # controller.go_to_pose(1.0, 0.5)
    # controller.speak("Hello, I am a humanoid robot!")
    # controller.navigate_and_grasp(-0.5, 1.0, "red cup")

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()