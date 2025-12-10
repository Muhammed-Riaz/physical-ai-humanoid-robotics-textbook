#!/usr/bin/env python3

"""
Natural Language Interface for Capstone Project
This node acts as the central hub for the VLA pipeline, receiving voice commands (as text), planning actions,
and sending high-level commands to the HumanoidController and ObjectRecognition nodes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
# Assuming HumanoidController and ObjectRecognition nodes are running or integrated
# In a real setup, you might import and call their functions directly or communicate via ROS topics/actions
from capstone.robot_control import HumanoidController
from capstone.object_recognition import ObjectRecognitionNode

class NLPInterfaceNode(Node):

    def __init__(self):
        super().__init__('nlp_interface_node')

        # Subscribers
        self.voice_command_subscriber = self.create_subscription(
            String,
            '/voice_command_text',
            self.voice_command_callback,
            10
        )
        self.object_detection_subscriber = self.create_subscription(
            String, # Assuming a simplified string for object detection info
            '/object_detections_info',
            self.object_detection_callback,
            10
        )

        # Publishers (for internal communication or debugging)
        self.action_plan_publisher = self.create_publisher(String, '/action_plan', 10)

        # Initialize child controllers (conceptual, as they run as separate nodes)
        # In a real integration, these would be ROS clients (e.g., ActionClients for Nav2)
        self.humanoid_controller = HumanoidController()
        # self.object_recognition_node = ObjectRecognitionNode()

        self.get_logger().info('NLP Interface Node Started')

    def voice_command_callback(self, msg):
        """Process incoming voice commands as text."""
        command_text = msg.data
        self.get_logger().info(f'Received voice command: "{command_text}"')
        self.process_natural_language_command(command_text)

    def object_detection_callback(self, msg):
        """Process incoming object detection information."""
        detection_info = msg.data
        self.get_logger().info(f'Received object detection info: {detection_info}')
        # Store or process this information to be used by the cognitive planner
        self.latest_object_detections = json.loads(detection_info) # Example

    def process_natural_language_command(self, command_text):
        """Translate natural language commands into a sequence of robot actions using LLM (conceptual)."""
        self.humanoid_controller.speak(f'Processing your command: {command_text}')
        action_sequence = self.cognitive_planner(command_text)
        self.humanoid_controller.speak('Executing plan.')
        self.execute_action_sequence(action_sequence)

    def cognitive_planner(self, command_text):
        """Simulated LLM-based cognitive planner."""
        command_lower = command_text.lower()
        actions = []

        if "go to kitchen" in command_lower:
            actions.append({"action": "navigate", "target": {"x": 1.0, "y": 2.0}})
            actions.append({"action": "speak", "text": "I have arrived in the kitchen."})
        elif "pick up the red cup" in command_lower:
            actions.append({"action": "speak", "text": "Looking for the red cup."})
            # Simulate object detection being used here
            # If red cup is detected, then grasp
            actions.append({"action": "grasp", "object": "red cup"})
            actions.append({"action": "speak", "text": "I picked up the red cup."})
        elif "bring the red cup to me" in command_lower:
            actions.append({"action": "speak", "text": "Bringing the red cup."})
            actions.append({"action": "navigate", "target": {"x": 0.0, "y": 0.0}})
            actions.append({"action": "speak", "text": "Here is the red cup."})
        elif "clean the room" in command_lower:
            actions.append({"action": "speak", "text": "Starting to clean the room."})
            actions.append({"action": "navigate", "target": {"x": 2.0, "y": 0.0}})
            actions.append({"action": "grasp", "object": "debris"})
            actions.append({"action": "navigate", "target": {"x": 0.0, "y": 0.0}})
            actions.append({"action": "speak", "text": "Room cleaned."})
        else:
            actions.append({"action": "speak", "text": f"I don't understand: {command_text}"})

        self.action_plan_publisher.publish(String(data=json.dumps(actions)))
        return actions

    def execute_action_sequence(self, action_sequence):
        """Execute a sequence of planned actions."""
        for action in action_sequence:
            action_type = action['action']
            params = action.get('target') or action.get('object') or action.get('text')
            self.get_logger().info(f'Executing {action_type} with params {params}')

            if action_type == "navigate":
                self.humanoid_controller.go_to_pose(params['x'], params['y'])
                # In a real system, wait for navigation completion
                time.sleep(5) # Simulate navigation time
            elif action_type == "grasp":
                self.humanoid_controller.perform_grasp(params)
            elif action_type == "speak":
                self.humanoid_controller.speak(params)
            time.sleep(1) # Small delay between actions


def main(args=None):
    rclpy.init(args=args)

    nlp_interface = NLPInterfaceNode()

    # For demonstration, simulate a voice command (in a real system, this comes from speech_recognition)
    # nlp_interface.voice_command_callback(String(data="Go to kitchen"))
    # time.sleep(10) # Give time for navigation
    # nlp_interface.voice_command_callback(String(data="Pick up the red cup"))

    rclpy.spin(nlp_interface)

    nlp_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()