---
sidebar_position: 4
---

# Basic Robot Spawning in Gazebo

This tutorial covers how to spawn your first robot model in Gazebo and verify that it works correctly in the simulation environment.

## Prerequisites

Before starting this tutorial, ensure you have:
- Successfully installed Gazebo Garden
- Completed the [Gazebo Environment Setup](./setup-gazebo-environment.md)
- Basic understanding of URDF/SDF formats

## Method 1: Using Gazebo's Built-in Models

### Launching with a Pre-built Model

Gazebo comes with several built-in models that you can use for testing:

```bash
# Launch Gazebo with a simple model
gz sim -r -v 1

# Or launch with a specific world file
gz sim -r empty.sdf
```

### Adding Models from the Database

You can add models from Gazebo's online model database:

```bash
# List available models
gz model --list

# Add a model via command line (this won't work directly, but shows the concept)
# In the Gazebo GUI, you can browse and insert models from the Insert tab
```

## Method 2: Creating and Spawning Your Own Robot

### Step 1: Create a Simple URDF Robot

Create a file named `simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

### Step 2: Convert URDF to SDF

```bash
# Convert the URDF to SDF format for Gazebo
gz sdf -p simple_robot.urdf > simple_robot.sdf
```

### Step 3: Create a World File

Create a world file named `simple_robot_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="simple_robot_world">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your custom robot -->
    <model name="simple_robot">
      <include>
        <uri>model://simple_robot</uri>
        <pose>0 0 0.1 0 0 0</pose>
      </include>
    </model>
  </world>
</sdf>
```

### Step 4: Launch the World

```bash
# Launch your custom world with the robot
gz sim -r simple_robot_world.sdf
```

## Method 3: Programmatic Spawning with ROS 2

### Installing Required Packages

```bash
# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### Using ros_gz_sim for Spawning

Create a launch file `spawn_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package and file paths
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    urdf_file = pkg_share + '/urdf/simple_robot.urdf'

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    # Gazebo launch
    start_gazebo_ros_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        parameters=[{'robot_namespace': '/'}],
        arguments=[
            '-entity', 'simple_robot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        start_gazebo_ros_spawn_node,
    ])
```

## Verification Steps

After spawning your robot, verify it's working correctly:

### 1. Visual Verification
- Check that the robot appears in the Gazebo window
- Verify the robot's shape, size, and color match your URDF/SDF
- Ensure the robot is positioned correctly in the world

### 2. Physics Verification
- Observe if the robot responds to gravity (should rest on the ground)
- Check for any unexpected movements or instabilities
- Verify collision properties are working (robot should not fall through the ground)

### 3. Sensor Verification (if applicable)
If your robot has sensors:

```bash
# List available topics
ros2 topic list | grep -i scan  # For LiDAR
ros2 topic list | grep -i camera  # For cameras
ros2 topic list | grep -i imu  # For IMU

# Check sensor data
ros2 topic echo /your_robot/laser_scan
```

## Troubleshooting Common Issues

### Robot Falls Through the Ground
- Verify inertial properties in your URDF/SDF
- Check that collision geometries are properly defined
- Ensure mass values are positive and reasonable

### Robot Appears with Wrong Dimensions
- Double-check the units in your URDF (meters for Gazebo)
- Verify visual and collision geometries match expectations

### Robot is Unstable or Jittery
- Reduce the `max_step_size` in your world file physics settings
- Ensure proper inertial properties and mass values
- Check that the center of mass is correctly positioned

## Advanced Spawning Techniques

### Spawning Multiple Robots

To spawn multiple robots with different names and positions:

```bash
# Use different namespaces and positions
ros2 run gazebo_ros spawn_entity.py -entity robot1 -file robot.urdf -x 0 -y 0 -z 0.1
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file robot.urdf -x 2 -y 2 -z 0.1
```

### Spawning with Different Configurations

You can use the same URDF with different parameters by using xacro:

```xml
<!-- robot_with_params.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="param_robot">
  <xacro:arg name="robot_name" default="my_robot"/>
  <xacro:arg name="wheel_radius" default="0.1"/>

  <link name="$(arg robot_name)_base_link">
    <!-- Robot definition using parameters -->
  </link>
</robot>
```

## Next Steps

Once you've successfully spawned your first robot:

1. Continue to learn about [URDF and SDF formats](../urdf-sdf-formats/urdf-basics.md) to create more complex robots
2. Learn about [physics simulation](../physics-simulation/gravity-and-collisions.md) to add realistic behavior
3. Explore [sensor simulation](../sensor-simulation/lidar-simulation.md) to add perception capabilities to your robots