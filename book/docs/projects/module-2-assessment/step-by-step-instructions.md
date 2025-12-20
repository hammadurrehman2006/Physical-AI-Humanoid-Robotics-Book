---
sidebar_position: 2
---

# Step-by-Step Instructions for Module 2 Assessment Project

This document provides detailed, step-by-step instructions for completing the Module 2 assessment project. Follow these steps in order to build a complete simulation environment that demonstrates your understanding of Gazebo simulation, robot modeling, physics simulation, sensor integration, and navigation.

## Prerequisites

Before starting the project, ensure you have:

1. **ROS 2 Humble Hawksbill** installed and configured
2. **Gazebo Garden** installed and working
3. **Basic Python and C++ programming knowledge**
4. **Familiarity with Linux command line**
5. **Git version control system**

## Phase 1: Project Setup and Robot Model Creation (Days 1-7)

### Step 1.1: Create Project Structure

1. Open a terminal and navigate to your workspace:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create the project directory structure:
   ```bash
   mkdir -p module_2_assessment/{robot_description/{urdf,launch,config},gazebo_worlds/{worlds,launch},sensors,tests,docs}
   cd module_2_assessment
   ```

3. Create the `package.xml` file:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>module_2_assessment</name>
     <version>1.0.0</version>
     <description>Module 2 Assessment Project</description>
     <maintainer email="student@university.edu">Your Name</maintainer>
     <license>MIT</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>rclpy</depend>
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <depend>sensor_msgs</depend>
     <depend>nav_msgs</depend>
     <depend>tf2_ros</depend>
     <depend>gazebo_ros</depend>
     <depend>gazebo_plugins</depend>
     <depend>robot_state_publisher</depend>
     <depend>joint_state_publisher</depend>
     <depend>xacro</depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

4. Create the `CMakeLists.txt` file:
   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(module_2_assessment)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   find_package(ament_cmake REQUIRED)
   find_package(rclpy REQUIRED)
   find_package(std_msgs REQUIRED)
   find_package(geometry_msgs REQUIRED)
   find_package(sensor_msgs REQUIRED)
   find_package(nav_msgs REQUIRED)
   find_package(tf2_ros REQUIRED)
   find_package(gazebo_ros REQUIRED)
   find_package(gazebo_plugins REQUIRED)
   find_package(robot_state_publisher REQUIRED)
   find_package(joint_state_publisher REQUIRED)
   find_package(xacro REQUIRED)

   install(DIRECTORY
     launch
     DESTINATION share/${PROJECT_NAME}/
   )

   install(DIRECTORY
     config
     DESTINATION share/${PROJECT_NAME}/
   )

   install(DIRECTORY
     urdf
     DESTINATION share/${PROJECT_NAME}/
   )

   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     ament_lint_auto_find_test_dependencies()
   endif()

   ament_package()
   ```

### Step 1.2: Create Robot URDF Model

1. Create the robot URDF file at `robot_description/urdf/robot_model.urdf`:
   ```xml
   <?xml version="1.0"?>
   <robot name="assessment_robot">
     <!-- Materials -->
     <material name="blue">
       <color rgba="0.0 0.0 1.0 1.0"/>
     </material>
     <material name="black">
       <color rgba="0.0 0.0 0.0 1.0"/>
     </material>
     <material name="white">
       <color rgba="1.0 1.0 1.0 1.0"/>
     </material>

     <!-- Base link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.5 0.4 0.2"/>
         </geometry>
         <material name="white"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.5 0.4 0.2"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="10.0"/>
         <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.541" iyz="0.0" izz="0.241"/>
       </inertial>
     </link>

     <!-- Left wheel -->
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <origin rpy="1.5708 0 0"/>
         <material name="black"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <origin rpy="1.5708 0 0"/>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Right wheel -->
     <link name="right_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <origin rpy="1.5708 0 0"/>
         <material name="black"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <origin rpy="1.5708 0 0"/>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Joints -->
     <joint name="left_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="left_wheel"/>
       <origin xyz="0 0.25 -0.1" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <joint name="right_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0 -0.25 -0.1" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <!-- ROS 2 Control interface -->
     <ros2_control name="GazeboSystem" type="system">
       <hardware>
         <plugin>gazebo_ros2_control/GazeboSystem</plugin>
       </hardware>
       <joint name="left_wheel_joint">
         <command_interface name="velocity">
           <param name="min">-10</param>
           <param name="max">10</param>
         </command_interface>
         <state_interface name="position"/>
         <state_interface name="velocity"/>
       </joint>
       <joint name="right_wheel_joint">
         <command_interface name="velocity">
           <param name="min">-10</param>
           <param name="max">10</param>
         </command_interface>
         <state_interface name="position"/>
         <state_interface name="velocity"/>
       </joint>
     </ros2_control>

     <!-- Gazebo plugin -->
     <gazebo>
       <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
         <parameters>$(find module_2_assessment)/config/robot_control.yaml</parameters>
       </plugin>
     </gazebo>
   </robot>
   ```

2. Create the robot control configuration at `robot_description/config/robot_control.yaml`:
   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 100  # Hz

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

       diff_drive_controller:
         type: diff_drive_controller/DiffDriveController

   diff_drive_controller:
     ros__parameters:
       left_wheel_names: ["left_wheel_joint"]
       right_wheel_names: ["right_wheel_joint"]

       wheel_separation: 0.5
       wheel_radius: 0.1

       use_stamped_vel: false

       # Publish rate
       publish_rate: 50.0
       odom_publish_rate: 20.0

       # Topic names
       cmd_vel_topic: "cmd_vel"
       odom_topic: "odom"
       pose_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
       twist_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
   ```

### Step 1.3: Create Robot Launch File

Create `robot_description/launch/spawn_robot.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'robot_model.urdf'])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'urdf.rviz'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node  # Uncomment to visualize in RViz
    ])
```

### Step 1.4: Validate Robot Model

1. Test the URDF model:
   ```bash
   # Check URDF syntax
   check_urdf robot_description/urdf/robot_model.urdf

   # Convert to SDF to check compatibility
   gz sdf -p robot_description/urdf/robot_model.urdf
   ```

2. Visualize the robot in RViz:
   ```bash
   # Build the package
   cd ~/ros2_ws
   colcon build --packages-select module_2_assessment

   # Source the workspace
   source install/setup.bash

   # Launch the robot state publisher
   ros2 launch module_2_assessment spawn_robot.launch.py
   ```

## Phase 2: Gazebo Environment Setup (Days 8-14)

### Step 2.1: Create World File

Create `gazebo_worlds/worlds/navigation_course.world`:
```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="navigation_course">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Navigation course walls -->
    <model name="outer_wall_north">
      <pose>0 3.5 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="outer_wall_south">
      <pose>0 -3.5 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="outer_wall_east">
      <pose>3.5 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="outer_wall_west">
      <pose>-3.5 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>7 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Obstacles in the course -->
    <model name="obstacle_1">
      <pose>1 1 0.2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.01</iyy>
            <iyz>0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.4</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-1 -1 0.2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.01</iyy>
            <iyz>0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Target location -->
    <model name="target">
      <pose>-2 2 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2.2: Create World Launch File

Create `gazebo_worlds/launch/world.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'navigation_course.world'])

    # Launch Gazebo with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
```

### Step 2.3: Test Environment

1. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select module_2_assessment
   source install/setup.bash
   ```

2. Launch the world:
   ```bash
   ros2 launch module_2_assessment world.launch.py
   ```

3. Verify that the environment loads correctly with walls, obstacles, and target.

## Phase 3: Sensor Integration (Days 15-21)

### Step 3.1: Add Sensors to Robot URDF

Update your `robot_description/urdf/robot_model.urdf` to include sensors:

```xml
<!-- Add after the base_link definition -->

<!-- LiDAR sensor -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Camera sensor -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.22 0 0.1" rpy="0 0 0"/>
</joint>

<!-- IMU sensor -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
    <material name="white"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

### Step 3.2: Add Sensor Plugins to URDF

Add these plugins after the existing Gazebo plugin in your URDF:

```xml
<!-- LiDAR sensor plugin -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
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
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>

<!-- Camera sensor plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
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
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>robot</namespace>
        <remapping>~/image_raw:=image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>

<!-- IMU sensor plugin -->
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <update_rate>100</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

### Step 3.3: Create Sensor Test Scripts

Create `sensors/test_sensors.py`:
```python
#!/usr/bin/env python3
"""
Sensor validation script for the assessment project
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Initialize subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/robot/image',
            self.camera_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )

        self.bridge = CvBridge()
        self.lidar_received = False
        self.camera_received = False
        self.imu_received = False

        self.get_logger().info('Sensor validator node started')

    def lidar_callback(self, msg):
        if not self.lidar_received:
            self.get_logger().info(f'LiDAR sensor validated: {len(msg.ranges)} ranges, range {msg.range_min:.2f}-{msg.range_max:.2f}m')
            self.lidar_received = True

    def camera_callback(self, msg):
        if not self.camera_received:
            self.get_logger().info(f'Camera sensor validated: {msg.width}x{msg.height} {msg.encoding}')
            self.camera_received = True

    def imu_callback(self, msg):
        if not self.imu_received:
            self.get_logger().info('IMU sensor validated: orientation and angular velocity available')
            self.imu_received = True

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    # Wait for sensor data
    timer = validator.create_timer(0.1, lambda: None)  # Keep node alive

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Sensor validation completed')

        if validator.lidar_received and validator.camera_received and validator.imu_received:
            validator.get_logger().info('All sensors validated successfully!')
        else:
            validator.get_logger().warn('Some sensors failed validation')

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3.4: Test Sensors

1. Make the script executable:
   ```bash
   chmod +x sensors/test_sensors.py
   ```

2. Build and test:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select module_2_assessment
   source install/setup.bash
   ```

3. Launch Gazebo with your robot and test sensors:
   ```bash
   # Terminal 1: Launch Gazebo world
   ros2 launch module_2_assessment world.launch.py

   # Terminal 2: Spawn your robot (you'll need to create a spawn script)
   # This is just a placeholder - you'll implement this in the next step
   ```

## Phase 4: Navigation Implementation (Days 22-28)

### Step 4.1: Create Navigation Configuration

Create `navigation/config/costmap_common_params.yaml`:
```yaml
map_type: costmap
origin_z: 0.0
z_resolution: 1
z_voxels: 2

obstacle_range: 2.5
raytrace_range: 3.0

publish_voxel_map: false
transform_tolerance: 0.5
meter_scoring: true

# Robot footprint
footprint: [[-0.25, -0.2], [-0.25, 0.2], [0.25, 0.2], [0.25, -0.2]]
footprint_padding: 0.1

# Observation sources
observation_sources: scan
scan: {sensor_frame: lidar_link, data_type: LaserScan, topic: /robot/scan, marking: true, clearing: true, obstacle_range: 2.5, raytrace_range: 3.0}
```

Create `navigation/config/local_costmap_params.yaml`:
```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 5.0
  height: 5.0
  resolution: 0.05
  origin_x: 0.0
  origin_y: 0.0
```

Create `navigation/config/global_costmap_params.yaml`:
```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  static_map: true
  rolling_window: false
  resolution: 0.05
```

### Step 4.2: Create Navigation Launch File

Create `navigation/launch/navigation.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')

    # Navigation nodes
    nav2_bringup_launch_dir = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch'
    ])

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_launch_dir,
            '/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                pkg_share,
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    return LaunchDescription([
        navigation_launch
    ])
```

### Step 4.3: Create Navigation Script

Create `navigation/scripts/simple_navigation.py`:
```python
#!/usr/bin/env python3
"""
Simple navigation script for the assessment project
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF buffer and listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Navigation parameters
        self.target_x = -2.0  # Target position from world file
        self.target_y = 2.0
        self.target_tolerance = 0.3  # 30cm tolerance

        # Timer for navigation loop
        self.timer = self.create_timer(0.1, self.navigate)

        self.get_logger().info(f'Navigator started, targeting ({self.target_x}, {self.target_y})')

    def navigate(self):
        try:
            # Get robot's current pose
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            current_x = t.transform.translation.x
            current_y = t.transform.translation.y

            # Calculate distance to target
            distance = math.sqrt((self.target_x - current_x)**2 + (self.target_y - current_y)**2)

            # Check if we've reached the target
            if distance < self.target_tolerance:
                self.get_logger().info(f'Reached target! Distance: {distance:.2f}m')
                self.stop_robot()
                return

            # Calculate desired angle to target
            desired_angle = math.atan2(self.target_y - current_y, self.target_x - current_x)

            # Get current robot angle from quaternion
            q = t.transform.rotation
            current_angle = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            # Calculate angle difference
            angle_diff = desired_angle - current_angle
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Create twist message
            twist = Twist()

            # Rotate towards target if angle difference is large
            if abs(angle_diff) > 0.2:  # 0.2 radians ~ 11 degrees
                twist.angular.z = max(-0.5, min(0.5, angle_diff * 1.0))
            else:
                # Move forward
                twist.linear.x = min(0.5, distance * 0.5)  # Proportional to distance
                twist.angular.z = angle_diff * 0.5  # Small correction

            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f'Navigating: distance={distance:.2f}m, angle_diff={math.degrees(angle_diff):.1f}°')

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation stopped by user')

    navigator.stop_robot()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4.4: Create Complete Launch File

Create a combined launch file that brings everything together. First, create the complete URDF file:

`robot_description/urdf/complete_robot.urdf`:
```xml
<?xml version="1.0"?>
<robot name="assessment_robot">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.541" iyz="0.0" izz="0.241"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.25 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.25 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.22 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- IMU sensor -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- ROS 2 Control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find module_2_assessment)/config/robot_control.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- LiDAR sensor plugin -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
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
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Camera sensor plugin -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
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
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>robot</namespace>
          <remapping>~/image_raw:=image</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor plugin -->
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>robot</namespace>
          <remapping>~/out:=imu</remapping>
        </ros>
        <update_rate>100</update_rate>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Step 4.5: Create Final Assembly Launch File

Create `launch/assessment_project.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'navigation_course.world'])

    # Launch Gazebo with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('module_2_assessment'),
                'launch',
                'spawn_robot.launch.py'
            ])
        ])
    )

    # Navigation node
    navigation_node = Node(
        package='module_2_assessment',
        executable='simple_navigation.py',
        name='navigator',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        navigation_node
    ])
```

## Phase 5: Testing and Validation (Days 29-35)

### Step 5.1: Create Validation Scripts

Create `tests/validate_project.py`:
```python
#!/usr/bin/env python3
"""
Project validation script for Module 2 assessment
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class ProjectValidator(Node):
    def __init__(self):
        super().__init__('project_validator')

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan, '/robot/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/robot/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publisher for movement commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Validation flags
        self.lidar_valid = False
        self.imu_valid = False
        self.odom_valid = False
        self.movement_tested = False

        # Start validation sequence
        self.timer = self.create_timer(1.0, self.run_validation)
        self.validation_step = 0

        self.get_logger().info('Project validator started')

    def lidar_callback(self, msg):
        if not self.lidar_valid:
            # Basic validation: check for reasonable range values
            valid_ranges = [r for r in msg.ranges if 0.1 <= r <= 10.0]
            if len(valid_ranges) > len(msg.ranges) * 0.5:  # At least 50% valid
                self.lidar_valid = True
                self.get_logger().info('✓ LiDAR sensor validated')

    def imu_callback(self, msg):
        if not self.imu_valid:
            # Basic validation: check for reasonable values
            if abs(msg.linear_acceleration.x) < 20:  # Reasonable acceleration
                self.imu_valid = True
                self.get_logger().info('✓ IMU sensor validated')

    def odom_callback(self, msg):
        if not self.odom_valid:
            # Basic validation: check for reasonable position/velocity
            if abs(msg.pose.pose.position.x) < 100:  # Reasonable position
                self.odom_valid = True
                self.get_logger().info('✓ Odometry validated')

    def run_validation(self):
        if self.validation_step == 0:
            self.get_logger().info('Starting validation sequence...')
            self.validation_step += 1
        elif self.validation_step == 1:
            # Test movement
            if self.odom_valid:  # Wait for odom to be available
                self.test_movement()
                self.validation_step += 1
        elif self.validation_step == 2:
            self.check_complete()
            self.validation_step += 1

    def test_movement(self):
        self.get_logger().info('Testing robot movement...')

        # Send forward command
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_pub.publish(twist)

        # Stop after 2 seconds
        self.create_timer(2.0, self.stop_robot)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.movement_tested = True
        self.get_logger().info('✓ Movement tested')

    def check_complete(self):
        all_valid = all([self.lidar_valid, self.imu_valid, self.odom_valid, self.movement_tested])

        if all_valid:
            self.get_logger().info('🎉 All validation checks passed!')
            self.get_logger().info('Project requirements satisfied:')
            self.get_logger().info('  ✓ Robot model with URDF/SDF')
            self.get_logger().info('  ✓ Gazebo simulation environment')
            self.get_logger().info('  ✓ Multiple sensor integration')
            self.get_logger().info('  ✓ Basic navigation capability')
        else:
            self.get_logger().error('❌ Some validation checks failed:')
            self.get_logger().error(f'  LiDAR: {"✓" if self.lidar_valid else "✗"}')
            self.get_logger().error(f'  IMU: {"✓" if self.imu_valid else "✗"}')
            self.get_logger().error(f'  Odometry: {"✓" if self.odom_valid else "✗"}')
            self.get_logger().error(f'  Movement: {"✓" if self.movement_tested else "✗"}')

def main(args=None):
    rclpy.init(args=args)
    validator = ProjectValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Validation stopped by user')

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5.2: Final Testing

1. Build the complete project:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select module_2_assessment
   source install/setup.bash
   ```

2. Run the complete project:
   ```bash
   # Terminal 1: Launch everything
   ros2 launch module_2_assessment assessment_project.launch.py

   # Terminal 2: Run validation (in another terminal)
   ros2 run module_2_assessment validate_project.py
   ```

## Final Steps and Submission

### Step 6.1: Document Your Work

Create `docs/final_report.md` with:
- Project overview and objectives
- Implementation details for each component
- Challenges encountered and solutions
- Performance metrics and validation results
- Lessons learned and future improvements

### Step 6.2: Clean Up and Prepare Submission

1. Ensure all files are properly organized
2. Test the complete system one final time
3. Document any special instructions for evaluation
4. Create a README with setup and execution instructions

Your project is now complete! The assessment will evaluate:
- Robot model quality and completeness
- Gazebo simulation functionality
- Sensor integration and validation
- Navigation performance
- Code quality and documentation
- Overall system integration

## Next Steps

After completing the step-by-step implementation:

1. Review [Solution Examples](./solution-examples.md) for reference implementations
2. Test your project against [Evaluation Criteria](../../module-2/assessment-project/evaluation-criteria.md)
3. Prepare your final documentation and submission