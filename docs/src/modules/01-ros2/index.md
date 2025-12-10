---
title: Module 1 - The Robotic Nervous System (ROS 2)
sidebar_position: 2
---

# Module 1: The Robotic Nervous System (ROS 2) - Weeks 3-5

## Overview

ROS 2 (Robot Operating System 2) serves as the nervous system for robots, providing a framework for developing robot applications. This module covers the core concepts of ROS 2, including nodes, topics, services, and how to bridge Python agents to ROS controllers.

## Learning Objectives

By the end of this module, students will be able to:
- Understand ROS 2 architecture and core concepts
- Create ROS 2 packages with Python
- Implement nodes, topics, and services
- Use rclpy to bridge Python agents to ROS controllers
- Understand URDF (Unified Robot Description Format) for humanoids

## Key Concepts with Analogies

- **Nodes**: Like organs in the body, each node performs a specific function
- **Topics**: Like the nervous system, topics allow nodes to communicate through messages
- **Services**: Like direct communication between brain and specific organs, services provide request-response communication
- **Parameters**: Like DNA, parameters define the configuration of nodes

## Hands-on Lab: Creating Your First ROS 2 Package

In this lab, students will create a simple ROS 2 package that controls a simulated humanoid robot.

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble or Iron installed

### Steps
1. Create a new ROS 2 workspace
2. Create a package with `ros2 pkg create`
3. Implement a simple publisher and subscriber
4. Test the communication between nodes

### Example Code

Check out the example publisher and subscriber code:

```python
# publisher.py - Simple ROS 2 publisher
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
    humanoid_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

[Code Sandbox Link](https://repl.it/@robotics/ROS2-Publisher-Subscriber) - Try this code in an interactive environment.

For the complete example files, see:
- [publisher.py](./code_examples/publisher.py)
- [subscriber.py](./code_examples/subscriber.py)

## Quiz/Questions

1. What is the difference between a ROS 2 topic and a service?
2. How does rclpy enable Python agents to interact with ROS 2?
3. What is URDF and why is it important for humanoid robots?

### Interactive Quiz

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="question1" label="Question 1">
What is the main difference between ROS 2 topics and services?

A) Topics are synchronous, services are asynchronous
B) Topics are asynchronous, services are synchronous
C) Topics are for sensors, services are for actuators
D) There is no difference

<details>
<summary>Answer</summary>
B) Topics are asynchronous, services are synchronous
</details>
</TabItem>

<TabItem value="question2" label="Question 2">
What does rclpy provide?

A) A C++ interface to ROS 2
B) A Python interface to ROS 2
C) A Java interface to ROS 2
D) A JavaScript interface to ROS 2

<details>
<summary>Answer</summary>
B) A Python interface to ROS 2
</details>
</TabItem>
</Tabs>

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)