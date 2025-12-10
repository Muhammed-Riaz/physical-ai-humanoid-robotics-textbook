---
title: Nav2 Deployment on Jetson Orin
sidebar_position: 12
---

# Nav2 Deployment on Jetson Orin

## Overview

Navigation2 (Nav2) provides path planning and navigation capabilities for robots, enabling autonomous movement in known or unknown environments. This guide covers deploying Nav2 on the NVIDIA Jetson Orin platform for humanoid robot navigation.

## Prerequisites

- NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- VSLAM system (from previous module) working
- At least 8GB free storage space
- Network connectivity

## Installation

### 1. Install Nav2 packages

Install the complete Nav2 stack on your Jetson:

```bash
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-dwb-core ros-humble-dwb-plugins
sudo apt install ros-humble-nav2-rviz-plugins
```

### 2. Install Isaac ROS Nav packages (Optional but recommended)

For better performance on Jetson:

```bash
sudo apt install ros-humble-isaac-ros-nav2
```

## Configuration

### 1. Create a navigation configuration

Create a navigation configuration file for your humanoid robot:

```yaml
# config/navigation_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: False

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: False

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_frame: base_link
    odom_frame: odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path to the Behavior Tree XML file
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Recovery
    recovery_enabled: True
    number_of_recovery_attempts: 5

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: False

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      max_linear_vel: 0.5
      min_linear_vel: 0.1
      max_angular_vel: 1.0
      min_angular_vel: 0.4
      carrot_dist: 0.3
      rotate_to_heading_angular_vel: 1.0
      max_heading_rate: 1.0
      speed_regulator_enabled: true
      speed_limit_percentage: 1.0
      use_rotate_to_heading: false
      use_regulated_linear_velocity_scaling: true
      use_regulated_angular_velocity_scaling: true
      regulated_linear_scaling_min_speed: 0.2
      regulated_linear_scaling_limit_margin: 0.1

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: False
  local_costmap_client:
    ros__parameters:
      use_sim_time: False
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: False
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

map_server:
  ros__parameters:
    use_sim_time: False
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: False
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: False
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_smooth: 0.9
      w_data: 0.1

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0

waypoint_follower:
  ros__parameters:
    use_sim_time: False
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_queue_size: 10
```

### 2. Create a launch file

Create a launch file to start the navigation stack:

```python
# launch/navigation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Navigation server
    nav2_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params])

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params])

    # Recoveries server
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params])

    # BT navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['controller_server',
                                   'planner_server',
                                   'recoveries_server',
                                   'bt_navigator']}])

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add nodes
    ld.add_action(nav2_server)
    ld.add_action(planner_server)
    ld.add_action(recoveries_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)

    return ld
```

## Deployment Process

### 1. Set up the workspace

```bash
# Create navigation workspace
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws

# Build packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2. Launch navigation

```bash
# Launch the navigation stack
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

### 3. Launch with RViz for visualization

```bash
# Launch navigation with RViz
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false
```

## Bipedal Navigation Considerations

### 1. Specific challenges for humanoid robots

Humanoid robots present unique navigation challenges:

- **Stability**: Path planning must account for balance constraints
- **Step height**: Obstacle clearance must consider leg reach
- **Foot placement**: Navigation paths must allow for proper foot positioning
- **Dynamic balance**: Motion planning must maintain center of mass

### 2. Configuration adjustments

For humanoid robots, adjust these parameters:

```yaml
# Specialized parameters for humanoid navigation
controller_server:
  ros__parameters:
    # Humanoid-specific constraints
    max_linear_vel: 0.3  # Slower for stability
    min_linear_vel: 0.05
    max_angular_vel: 0.5 # More conservative turning
    # Footstep planning considerations
    carrot_dist: 0.5     # Longer look-ahead for stability
```

## Isaac ROS Nav2 Integration

### 1. Install Isaac ROS Nav2 packages

```bash
sudo apt install ros-humble-isaac-ros-nav2
```

### 2. Use Isaac-optimized navigation

Isaac ROS provides optimized navigation components:

- Isaac ROS Path Planner: Optimized for Jetson hardware
- Isaac ROS Controller: Hardware-accelerated control
- Isaac ROS Safety: Enhanced safety checks for humanoid robots

## Performance Optimization

### 1. Jetson-specific optimizations

```bash
# Set Jetson to maximum performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Optimize for navigation
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
```

### 2. Memory management

Monitor and optimize memory usage:

```bash
# Check memory usage
free -h

# Monitor navigation processes
ps aux | grep nav
```

## Testing Navigation Performance

### 1. Accuracy testing

Test navigation accuracy:

```bash
# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

### 2. Performance metrics

Monitor navigation performance:

- Path planning time
- Execution accuracy
- Obstacle avoidance effectiveness
- Recovery behavior

## Troubleshooting

### Common Issues

1. **Navigation fails to start**: Check parameter file paths and permissions
   ```bash
   # Verify parameter file exists
   ls -la /path/to/your/params.yaml
   ```

2. **Path planning fails**: Ensure proper transforms and sensor data
   ```bash
   # Check transforms
   ros2 run tf2_tools view_frames
   # Check sensor data
   ros2 topic echo /scan
   ```

3. **Performance issues**: Reduce costmap resolution or update frequency
   ```yaml
   # In costmap parameters, increase resolution (e.g., from 0.05 to 0.1)
   resolution: 0.1
   ```

## Resources

- [Navigation2 Documentation](https://navigation.ros.org/)
- [NVIDIA Isaac ROS Navigation](https://nvidia-isaac-ros.github.io/released/packages/isaac_ros_nav2/)
- [ROS 2 Navigation Tutorials](https://docs.ros.org/en/humble/Tutorials/Navigation/Navigation2-Tutorials.html)