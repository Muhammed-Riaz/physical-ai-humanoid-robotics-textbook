#!/usr/bin/env python3

"""
Object Recognition Component for Capstone Project
This node simulates object detection using camera images and publishes detected object information.
In a real system, this would integrate with a pre-trained deep learning model (e.g., YOLO, EfficientDet).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesis
import random
import time

class ObjectRecognitionNode(Node):

    def __init__(self, objects_to_detect=None):
        super().__init__('object_recognition_node')

        # Default objects to detect
        if objects_to_detect is None:
            self.objects_to_detect = ["red cup", "blue box", "green bottle"]
        else:
            self.objects_to_detect = objects_to_detect

        # Publishers
        self.detection_publisher = self.create_publisher(Detection2DArray, '/object_detections', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(f'Object Recognition Node Started. Looking for: {self.objects_to_detect}')

    def image_callback(self, msg):
        """Simulate object detection from camera image."""
        # In a real system, this would involve image processing and a DL model inference
        # For simulation, we'll randomly detect an object occasionally
        if random.random() < 0.2:  # 20% chance to detect an object
            detected_object_name = random.choice(self.objects_to_detect)
            self.publish_detection(detected_object_name, msg.width, msg.height)

    def publish_detection(self, object_name, img_width, img_height):
        """Publish a simulated object detection."""
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_link' # Assuming camera_link frame

        detection = Detection2D()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = 'camera_link'

        # Object hypothesis
        hypothesis = ObjectHypothesis()
        hypothesis.class_id = object_name
        hypothesis.score = 0.95 # High confidence
        detection.results.append(hypothesis)

        # Bounding box (randomized for simulation)
        bbox = BoundingBox2D()
        bbox.center.x = float(random.randint(img_width // 4, 3 * img_width // 4))
        bbox.center.y = float(random.randint(img_height // 4, 3 * img_height // 4))
        bbox.size_x = float(random.randint(img_width // 8, img_width // 4))
        bbox.size_y = float(random.randint(img_height // 8, img_height // 4))
        detection.bbox = bbox

        detection_array.detections.append(detection)
        self.detection_publisher.publish(detection_array)
        self.get_logger().info(f'Published simulated detection: {object_name} at ({bbox.center.x}, {bbox.center.y})')

def main(args=None):
    rclpy.init(args=args)

    # Example: Detect a red cup, blue box, or green bottle
    object_recognition_node = ObjectRecognitionNode(objects_to_detect=["red cup", "blue box", "green bottle"])

    try:
        rclpy.spin(object_recognition_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_recognition_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()