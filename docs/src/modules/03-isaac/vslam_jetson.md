---
title: VSLAM Deployment on Jetson Orin
sidebar_position: 11
---

# VSLAM Deployment on Jetson Orin

## Overview

Visual Simultaneous Localization and Mapping (VSLAM) is crucial for humanoid robots to understand their environment and navigate autonomously. This guide covers deploying VSLAM systems on the NVIDIA Jetson Orin platform, which serves as the "brain" for edge AI in Physical AI applications.

## Prerequisites

- NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
- Ubuntu 22.04 LTS
- Internet connection
- Intel RealSense D435i or D455 camera
- At least 4GB free storage space

## Hardware Setup

### 1. Jetson Orin Setup

1. Flash the Jetson Orin with the latest JetPack SDK
2. Connect the RealSense camera via USB 3.0
3. Ensure proper power supply (official Jetson power adapter)
4. Connect to network (Ethernet recommended for stability)

### 2. Camera Connection

Connect the Intel RealSense D435i/D455 to the Jetson Orin:

- Use a high-quality USB 3.0 cable
- Connect directly to the Jetson (avoid USB hubs)
- Ensure the camera is properly powered

## Software Installation

### 1. Install JetPack

Ensure you have JetPack 5.1 or later installed on your Jetson Orin:

```bash
# Check JetPack version
jetpack_version
# or
cat /etc/nv_tegra_release
```

### 2. Install ROS 2

Install ROS 2 Humble Hawksbill on your Jetson:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

### 3. Install RealSense Drivers

Install the RealSense SDK:

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-dev
```

### 4. Install ROS 2 RealSense Package

```bash
sudo apt install ros-humble-realsense2-camera
```

### 5. Install VSLAM Packages

Install ORB-SLAM2 or other VSLAM packages:

```bash
# Install dependencies
sudo apt install libeigen3-dev libsuitesparse-dev libboost-serialization-dev libyaml-cpp-dev
sudo apt install libopencv-dev

# Clone and build ORB-SLAM2
cd ~
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

## Isaac ROS VSLAM Setup

### 1. Install Isaac ROS VSLAM packages

NVIDIA provides optimized VSLAM packages for Jetson:

```bash
sudo apt install ros-humble-isaac-ros-point-cloud-transport
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-visual-slam
```

### 2. Configure VSLAM pipeline

Create a launch file for the VSLAM pipeline:

```xml
<!-- vslam_pipeline.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Launch argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # Container for all nodes
    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # RealSense camera node
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                parameters=[{
                    'enable_color': True,
                    'enable_depth': True,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'depth_module.profile': '640x480x30',
                    'rgb_camera.profile': '640x480x30',
                    'publish_tf': True,
                    'tf_publish_rate': 0.0
                }]
            ),
            # Isaac ROS Visual SLAM node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='isaac_ros::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'publish_odom_tf': True
                }]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

## Deployment Process

### 1. Build and source workspace

```bash
# Create workspace
mkdir -p ~/vslam_ws/src
cd ~/vslam_ws

# Build packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2. Launch VSLAM pipeline

```bash
# Launch the VSLAM pipeline
ros2 launch vslam_pipeline.launch.py
```

### 3. Monitor performance

Monitor the performance of your VSLAM system:

```bash
# Check CPU usage
htop

# Check GPU usage
sudo tegrastats

# Monitor ROS 2 topics
ros2 topic echo /visual_slam/visual_slam/odometry
```

## Optimization for Jetson Orin

### 1. Power Mode

Set Jetson to maximum performance mode:

```bash
sudo nvpmodel -m 0  # Maximum performance
sudo jetson_clocks  # Lock clocks to maximum
```

### 2. Memory Management

Optimize memory usage for VSLAM:

```bash
# Reduce swappiness
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
```

### 3. RealSense Optimization

Configure RealSense for optimal performance:

```bash
# Lower resolution for better performance
# In the launch file, use: 'depth_module.profile': '640x480x15'
# For less demanding applications, use lower FPS
```

## Testing VSLAM Performance

### 1. Accuracy Testing

Test the accuracy of your VSLAM system:

```bash
# Record a bag file for analysis
ros2 bag record /visual_slam/visual_slam/odometry /camera/color/image_raw

# Play back and analyze
ros2 bag play your_recording.bag
```

### 2. Performance Metrics

Monitor key performance metrics:

- Tracking accuracy (position drift over time)
- Processing latency (time from image capture to pose estimate)
- Frame rate (ensure real-time performance)
- Memory usage (avoid memory leaks)

## Troubleshooting

### Common Issues

1. **Camera Not Detected**: Check USB connection and permissions
   ```bash
   # Check USB devices
   lsusb | grep Intel
   # Add udev rules if needed
   sudo cp ~/.local/lib/python3.8/site-packages/pyrealsense2/udev_rules/setup_udev_rules.sh /tmp/
   sudo /tmp/setup_udev_rules.sh
   ```

2. **Performance Issues**: Reduce image resolution or frame rate
   ```bash
   # In the RealSense parameters, lower the resolution
   'depth_module.profile': '424x240x15'
   ```

3. **Compilation Errors**: Ensure all dependencies are installed
   ```bash
   # Install additional dependencies if needed
   sudo apt install build-essential cmake pkg-config
   ```

## Resources

- [NVIDIA Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/released/)
- [Intel RealSense Documentation](https://www.intelrealsense.com/sdk-2/)
- [Jetson Performance Optimization](https://docs.nvidia.com/jetson/archives/r34.1.1/developer_guide/index.html)