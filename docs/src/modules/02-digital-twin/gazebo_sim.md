---
title: Gazebo Simulation Guide
sidebar_position: 9
---

# Gazebo Simulation Guide

## Overview

Gazebo is a 3D simulation environment for robotics that provides realistic physics simulation and sensor feedback. This guide will help you set up and use Gazebo for simulating humanoid robots in the Physical AI curriculum.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble or Iron installed
- NVIDIA GPU with RTX capabilities recommended for visual rendering
- At least 8GB RAM recommended

## Installation

### 1. Install Gazebo Garden

For the most up-to-date version of Gazebo:

```bash
sudo apt install gazebo
```

### 2. Install ROS 2 Gazebo packages

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
# For ROS 2 Iron, use: sudo apt install ros-iron-gazebo-ros-pkgs ros-iron-gazebo-plugins ros-iron-gazebo-dev
```

### 3. Install additional simulation packages

```bash
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## Basic Usage

### 1. Launch Gazebo

```bash
gazebo
```

Or launch with ROS 2 integration:

```bash
# Terminal 1
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so

# Terminal 2 (after setting up ROS 2 environment)
source /opt/ros/humble/setup.bash
```

### 2. Create a simple world

Create a file called `simple_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a model from the online database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a directional light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.6 0.4 -0.8</direction>
    </light>

    <!-- Add a model -->
    <model name="unit_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 1 1</ambient>
            <diffuse>0.3 0.3 1 1</diffuse>
            <specular>0.3 0.3 1 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 3. Launch your custom world

```bash
gazebo simple_world.sdf
```

## Integration with ROS 2

### 1. Spawn a model via ROS 2

```bash
# Make sure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash

# Spawn a model
ros2 run gazebo_ros spawn_entity.py -entity my_model -file /path/to/model.sdf -x 0 -y 0 -z 1
```

### 2. Control a model via ROS 2

Gazebo publishes and subscribes to various ROS 2 topics:

- `/clock` - Simulation time
- `/model_states` - Model poses
- `/tf` and `/tf_static` - Transform information
- `/joint_states` - Joint positions

## Simulating Sensors

### 1. LiDAR Sensor

Add to your robot model SDF:

```xml
<sensor name="lidar" type="ray">
  <pose>0.1 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="scan" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### 2. Camera Sensor

Add to your robot model SDF:

```xml
<sensor name="camera" type="camera">
  <pose>0.1 0 0.1 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>camera</namespace>
    </ros>
  </plugin>
</sensor>
```

## Hands-on Lab: Creating a Humanoid Robot Simulation

In this lab, you will create a simple humanoid robot in Gazebo.

### Steps
1. Create a URDF model of a simple humanoid
2. Launch Gazebo with your robot
3. Control the robot via ROS 2
4. Add sensors to your robot
5. Test sensor data in ROS 2

## Troubleshooting

### Common Issues

1. **Rendering Issues**: If Gazebo doesn't render properly:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gazebo
   ```

2. **Performance Issues**: Reduce rendering quality in Gazebo GUI under View > Rendering

3. **ROS 2 Connection Issues**: Make sure both Gazebo and your ROS 2 terminal have the same environment sourced

## Resources

- [Gazebo Documentation](http://gazebosim.org/)
- [ROS 2 Gazebo Tutorials](https://classic.gazebosim.org/tutorials?tut=ros2_overview)
- [Gazebo Model Database](https://app.gazebosim.org/fuel)