---
title: ROS 2 Installation Guide
sidebar_position: 8
---

# ROS 2 Installation Guide

## Overview

This guide will help you install ROS 2 Humble Hawksbill or Iron Irwini on Ubuntu 22.04. ROS 2 is essential for controlling humanoid robots and developing Physical AI applications.

## Prerequisites

- Ubuntu 22.04 LTS
- Internet connection
- Administrator (sudo) access
- At least 20GB of free disk space recommended

## Installation Steps

### 1. Set up locale

Make sure your locale is set to UTF-8:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Set up sources

Add the ROS 2 apt repository:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2

Update your apt repository and install ROS 2:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

For ROS 2 Iron (if available), use:

```bash
sudo apt install ros-iron-desktop
```

### 4. Environment setup

Add sourcing to your bash environment:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# For ROS 2 Iron, use: echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
```

To source the environment in the current terminal:

```bash
source /opt/ros/humble/setup.bash
# For ROS 2 Iron, use: source /opt/ros/iron/setup.bash
```

### 5. Install colcon

Install colcon for building packages:

```bash
sudo apt install python3-colcon-common-extensions
```

### 6. Install additional dependencies

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-catkin-tools
```

### 7. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

## Verification

Test that ROS 2 is installed correctly:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see no errors, and an empty list of topics.

## Troubleshooting

### Common Issues

1. **GPG Key Error**: If you encounter issues with the GPG key, try importing it manually:
   ```bash
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   ```

2. **Missing Dependencies**: If you get dependency errors, try:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```

3. **Python Package Issues**: Make sure you're using Python 3:
   ```bash
   python3 --version
   ```

## Next Steps

After successful installation, proceed to:
- Creating your first ROS 2 workspace
- Learning about ROS 2 packages and nodes
- Working with the Physical AI textbook examples

## Resources

- [Official ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Ubuntu 22.04 Setup Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)