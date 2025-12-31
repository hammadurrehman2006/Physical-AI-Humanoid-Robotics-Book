---
sidebar_position: 3
---

# Solution Examples for Module 2 Assessment Project

This document provides reference implementations and solution examples for the Module 2 assessment project. These examples demonstrate best practices and complete implementations of each required component.

## Complete Working Robot URDF

Here is a complete, tested URDF file that includes all required components:

### robot_description/urdf/assessment_robot.urdf

```xml
<?xml version="1.0"?>
<robot name="assessment_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
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
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <!-- Robot parameters -->
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.5"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_separation" value="0.4"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.541" iyz="0.0" izz="0.241"/>
    </inertial>
  </link>

  <!-- Left wheel macro -->
  <xacro:macro name="wheel" params="prefix parent x_offset y_offset">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin rpy="1.5708 0 0"/>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin rpy="1.5708 0 0"/>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_offset} ${y_offset} -${wheel_radius}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Create wheels -->
  <xacro:wheel prefix="left" parent="base_link" x_offset="0" y_offset="${wheel_separation/2}"/>
  <xacro:wheel prefix="right" parent="base_link" x_offset="0" y_offset="-${wheel_separation/2}"/>

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
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Sensor mounting points -->
  <link name="sensor_mount">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red"/>
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

  <joint name="sensor_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0003" ixy="0.0" ixz="0.0" iyy="0.0003" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="sensor_mount"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.02" rpy="0 0 0"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="sensor_mount"/>
    <child link="camera_link"/>
    <origin xyz="0.03 0 0.015" rpy="0 0 0"/>
  </joint>

  <!-- IMU sensor -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0000001" ixy="0.0" ixz="0.0" iyy="0.0000001" iyz="0.0" izz="0.0000001"/>
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
      <pose>0 0 0 0 0 0</pose>
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
      <always_on>1</always_on>
      <update_rate>10</update_rate>
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
      <pose>0 0 0 0 0 0</pose>
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
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
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
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>robot</namespace>
          <remapping>~/out:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Complete Gazebo World Example

### gazebo_worlds/worlds/complete_course.world

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="complete_course">
    <!-- Physics configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Lighting and environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Perimeter walls -->
    <model name="perimeter_north">
      <pose>0 4.0 0.5 0 0 0</pose>
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
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="perimeter_south">
      <pose>0 -4.0 0.5 0 0 0</pose>
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
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="perimeter_east">
      <pose>4.0 0 0.5 0 0 1.5708</pose>
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
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="perimeter_west">
      <pose>-4.0 0 0.5 0 0 1.5708</pose>
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
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>8.0 0.1 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Navigation obstacles -->
    <model name="obstacle_1">
      <pose>-1.5 1.0 0.2 0 0 0</pose>
      <static>true</static>
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
            <ambient>0.8 0.4 0.0 1</ambient>
            <diffuse>0.8 0.4 0.0 1</diffuse>
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
      <pose>1.5 -1.0 0.2 0 0 0</pose>
      <static>true</static>
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
              <radius>0.25</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.0 0.6 0.8 1</ambient>
            <diffuse>0.0 0.6 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.25</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="obstacle_3">
      <pose>0 2.5 0.15 0 0 0</pose>
      <static>true</static>
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
          </inertial>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.2 0.3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.0 0.8 1</ambient>
            <diffuse>0.6 0.0 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.2 0.3</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Target location -->
    <model name="navigation_target">
      <pose>-3.0 3.0 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Start location marker -->
    <model name="start_marker">
      <pose>3.0 -3.0 0.05 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.0 1.0 0.0 1</ambient>
            <diffuse>0.0 1.0 0.0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Lighting configuration -->
    <light name="main_light" type="directional">
      <pose>0 0 10 0 -0.5 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>100</range>
      </attenuation>
      <direction>-0.3 -0.3 -1</direction>
    </light>
  </world>
</sdf>
```

## Advanced Navigation Example

### navigation/scripts/advanced_navigation.py

```python
#!/usr/bin/env python3
"""
Advanced navigation script with obstacle avoidance for the assessment project
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from typing import List, Tuple

class AdvancedNavigator(Node):
    def __init__(self):
        super().__init__('advanced_navigator')

        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/robot/scan', self.scan_callback, 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Navigation parameters
        self.target_x = -3.0
        self.target_y = 3.0
        self.target_tolerance = 0.3
        self.obstacle_threshold = 0.8  # meters
        self.safe_distance = 0.6

        # State variables
        self.laser_data = None
        self.current_pose = None
        self.obstacle_detected = False
        self.avoiding = False
        self.avoid_direction = 0  # 1 for left, -1 for right

        # Timer for navigation loop
        self.timer = self.create_timer(0.1, self.navigate)

        self.get_logger().info(f'Advanced navigator started, targeting ({self.target_x}, {self.target_y})')

    def scan_callback(self, msg: LaserScan):
        """Process laser scan data"""
        self.laser_data = msg

        # Check for obstacles in front of robot
        if self.laser_data:
            # Get ranges from front of robot (±30 degrees)
            front_ranges = []
            total_points = len(self.laser_data.ranges)

            # Front angles: roughly -30 to +30 degrees
            start_idx = int(total_points * (2 * math.pi - math.pi/6) / (2 * math.pi))  # -30 degrees
            end_idx = int(total_points * (math.pi/6) / (2 * math.pi))  # +30 degrees

            if start_idx > end_idx:
                front_ranges = self.laser_data.ranges[start_idx:] + self.laser_data.ranges[:end_idx]
            else:
                front_ranges = self.laser_data.ranges[start_idx:end_idx]

            # Filter out invalid ranges
            valid_ranges = [r for r in front_ranges if self.laser_data.range_min < r < self.laser_data.range_max]

            if valid_ranges:
                min_front_range = min(valid_ranges)
                self.obstacle_detected = min_front_range < self.obstacle_threshold
            else:
                self.obstacle_detected = False

    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get robot's current pose from TF"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))

            x = t.transform.translation.x
            y = t.transform.translation.y

            # Convert quaternion to yaw
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            return x, y, yaw
        except TransformException:
            return None, None, None

    def find_gap(self, ranges: List[float], angles: List[float]) -> Tuple[float, float]:
        """Find the largest gap in obstacles"""
        # Convert ranges to points in robot frame
        points = []
        for i, r in enumerate(ranges):
            if self.laser_data.range_min < r < self.laser_data.range_max:
                angle = angles[i]
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

        # Cluster points that are close together
        clusters = []
        for point in points:
            found_cluster = False
            for cluster in clusters:
                # Check if point is close to existing cluster
                for cluster_point in cluster:
                    dist = math.sqrt((point[0] - cluster_point[0])**2 + (point[1] - cluster_point[1])**2)
                    if dist < 0.3:  # 30cm threshold
                        cluster.append(point)
                        found_cluster = True
                        break
                if found_cluster:
                    break
            if not found_cluster:
                clusters.append([point])

        # Find gaps between clusters
        gaps = []
        for i in range(len(clusters) - 1):
            # Calculate gap between clusters
            cluster1 = clusters[i]
            cluster2 = clusters[i + 1]

            # Find closest points between clusters
            min_gap = float('inf')
            for p1 in cluster1:
                for p2 in cluster2:
                    dist = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                    min_gap = min(min_gap, dist)

            gaps.append((min_gap, (cluster1[-1], cluster2[0])))

        if gaps:
            best_gap = max(gaps, key=lambda x: x[0])
            return best_gap[1][0][1], best_gap[1][1][1]  # Return y coordinates of gap

        return -0.5, 0.5  # Default wide gap

    def navigate(self):
        """Main navigation loop"""
        if not self.laser_data:
            self.get_logger().warn('No laser data available')
            return

        x, y, yaw = self.get_robot_pose()
        if x is None:
            self.get_logger().warn('Could not get robot pose')
            return

        # Calculate distance to target
        distance_to_target = math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)

        # Check if target reached
        if distance_to_target < self.target_tolerance:
            self.get_logger().info(f'Reached target! Distance: {distance_to_target:.2f}m')
            self.stop_robot()
            return

        # Calculate desired angle to target
        desired_angle = math.atan2(self.target_y - y, self.target_x - x)
        angle_diff = desired_angle - yaw

        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        twist = Twist()

        if self.obstacle_detected and not self.avoiding:
            # Start obstacle avoidance
            self.get_logger().info('Obstacle detected, starting avoidance')

            # Determine turn direction based on sensor data
            mid_idx = len(self.laser_data.ranges) // 2
            left_ranges = self.laser_data.ranges[mid_idx:mid_idx + len(self.laser_data.ranges)//4]
            right_ranges = self.laser_data.ranges[mid_idx - len(self.laser_data.ranges)//4:mid_idx]

            # Filter valid ranges
            left_valid = [r for r in left_ranges if self.laser_data.range_min < r < self.laser_data.range_max]
            right_valid = [r for r in right_ranges if self.laser_data.range_min < r < self.laser_data.range_max]

            left_avg = sum(left_valid) / len(left_valid) if left_valid else 0
            right_avg = sum(right_valid) / len(right_valid) if right_valid else 0

            if left_avg > right_avg:
                self.avoid_direction = 1  # Turn left
            else:
                self.avoid_direction = -1  # Turn right

            self.avoiding = True
        elif self.avoiding:
            # Continue obstacle avoidance
            if not self.obstacle_detected:
                # Clear path, resume path following
                self.avoiding = False
                self.get_logger().info('Clear path detected, resuming navigation')

            # Turn to avoid obstacle
            twist.angular.z = 0.5 * self.avoid_direction
            twist.linear.x = 0.2  # Slow forward movement
        else:
            # Normal path following
            if abs(angle_diff) > 0.2:  # 0.2 radians ~ 11 degrees
                # Turn toward target
                twist.angular.z = max(-0.5, min(0.5, angle_diff * 1.0))
            else:
                # Move forward
                twist.linear.x = min(0.5, distance_to_target * 0.5)
                twist.angular.z = angle_diff * 0.5  # Small correction

        self.cmd_vel_pub.publish(twist)

        # Log navigation status
        if self.avoiding:
            status = f'Avoiding obstacle, dir={self.avoid_direction}, dist={distance_to_target:.2f}m'
        else:
            status = f'Navigating: dist={distance_to_target:.2f}m, angle_diff={math.degrees(angle_diff):.1f}°'

        self.get_logger().info(status, throttle_duration_sec=1)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = AdvancedNavigator()

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

## Sensor Validation Example

### sensors/validation/laser_scan_validator.py

```python
#!/usr/bin/env python3
"""
Laser scan validation for the assessment project
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy import ndimage

class LaserScanValidator(Node):
    def __init__(self):
        super().__init__('laser_scan_validator')

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/robot/scan', self.scan_callback, 10)

        # Validation parameters
        self.scan_history = []
        self.max_history = 10  # Keep last 10 scans for analysis
        self.range_min = 0.1
        self.range_max = 10.0
        self.expected_objects = [
            {'range': 1.5, 'name': 'obstacle_1'},
            {'range': 2.0, 'name': 'obstacle_2'},
        ]

    def scan_callback(self, msg: LaserScan):
        """Validate laser scan data"""
        # Add to history
        self.scan_history.append(msg)
        if len(self.scan_history) > self.max_history:
            self.scan_history.pop(0)

        # Validate scan properties
        self.validate_scan_properties(msg)

        # Analyze scan data
        self.analyze_scan_data(msg)

    def validate_scan_properties(self, msg: LaserScan):
        """Validate basic scan properties"""
        # Check ranges count
        if len(msg.ranges) < 100:
            self.get_logger().warn(f'Few ranges: {len(msg.ranges)} (expected > 100)')

        # Check angle properties
        angle_diff = (msg.angle_max - msg.angle_min) / msg.angle_increment
        expected_points = int(angle_diff) + 1
        if len(msg.ranges) != expected_points:
            self.get_logger().warn(f'Angle/range mismatch: got {len(msg.ranges)}, expected {expected_points}')

        # Check valid ranges percentage
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        valid_percentage = len(valid_ranges) / len(msg.ranges) * 100

        if valid_percentage < 50:  # Less than 50% valid ranges
            self.get_logger().warn(f'Low valid ranges: {valid_percentage:.1f}%')
        else:
            self.get_logger().info(f'Scan quality: {valid_percentage:.1f}% valid ranges')

    def analyze_scan_data(self, msg: LaserScan):
        """Analyze scan data for meaningful information"""
        # Convert to numpy array for processing
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)  # Clip to valid range

        # Remove infinite values
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)

        # Find potential obstacles (close ranges)
        obstacle_threshold = 1.5  # meters
        obstacle_ranges = ranges[ranges < obstacle_threshold]

        if len(obstacle_ranges) > 0:
            avg_obstacle_dist = np.mean(obstacle_ranges)
            obstacle_count = len(obstacle_ranges)
            self.get_logger().info(f'Obstacles detected: {obstacle_count} at avg {avg_obstacle_dist:.2f}m')

        # Detect continuous surfaces (for walls)
        self.detect_walls(ranges, msg.angle_min, msg.angle_increment)

    def detect_walls(self, ranges, angle_min, angle_increment):
        """Detect walls or continuous surfaces in scan"""
        # Convert to Cartesian coordinates
        angles = np.array([angle_min + i * angle_increment for i in range(len(ranges))])
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        # Filter out distant points (focus on nearby obstacles)
        nearby_mask = ranges < 3.0  # Only consider points within 3m
        x_nearby = x_coords[nearby_mask]
        y_nearby = y_coords[nearby_mask]

        if len(x_nearby) > 10:  # Need enough points to detect walls
            # Try to fit straight lines to nearby points
            wall_segments = self.fit_lines_to_points(x_nearby, y_nearby)

            if len(wall_segments) > 0:
                self.get_logger().info(f'Wall segments detected: {len(wall_segments)}')

    def fit_lines_to_points(self, x, y):
        """Fit straight lines to point cloud data"""
        # Simple approach: group nearby points and fit lines
        segments = []

        # Sort points by angle
        angles = np.arctan2(y, x)
        sorted_indices = np.argsort(angles)

        sorted_x = x[sorted_indices]
        sorted_y = y[sorted_indices]

        # Look for linear segments (simple implementation)
        for i in range(len(sorted_x) - 10):  # At least 10 points per segment
            segment_x = sorted_x[i:i+10]
            segment_y = sorted_y[i:i+10]

            # Fit a line: y = mx + b
            coeffs = np.polyfit(segment_x, segment_y, 1)
            m, b = coeffs

            # Calculate R-squared to measure linearity
            y_pred = m * segment_x + b
            ss_res = np.sum((segment_y - y_pred) ** 2)
            ss_tot = np.sum((segment_y - np.mean(segment_y)) ** 2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0

            # Consider it a wall if R-squared is high
            if r_squared > 0.8:  # High linearity
                segments.append((m, b, r_squared))

        return segments

def main(args=None):
    rclpy.init(args=args)
    validator = LaserScanValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Laser scan validation stopped')

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity Integration Example

### unity_visualization/Unity ROS2 Bridge Configuration

For Unity integration, you would typically set up the ROS2Unity bridge as follows:

```json
// unity_visualization/bridge_config.json
{
  "connection_settings": {
    "host": "127.0.0.1",
    "port": 8888,
    "use_localhost_only": false
  },
  "topics": [
    {
      "name": "/robot/scan",
      "type": "sensor_msgs/LaserScan",
      "qos": {
        "history": "keep_last",
        "depth": 10,
        "reliability": "reliable",
        "durability": "volatile"
      }
    },
    {
      "name": "/robot/imu",
      "type": "sensor_msgs/Imu",
      "qos": {
        "history": "keep_last",
        "depth": 10,
        "reliability": "reliable",
        "durability": "volatile"
      }
    },
    {
      "name": "/robot/image",
      "type": "sensor_msgs/Image",
      "qos": {
        "history": "keep_last",
        "depth": 1,
        "reliability": "best_effort",
        "durability": "volatile"
      }
    },
    {
      "name": "/cmd_vel",
      "type": "geometry_msgs/Twist",
      "qos": {
        "history": "keep_last",
        "depth": 10,
        "reliability": "reliable",
        "durability": "volatile"
      }
    }
  ],
  "transform_settings": {
    "unity_to_ros_coordinate_system": {
      "position": {"x": "y", "y": "z", "z": "x"},
      "rotation": {"x": "y", "y": "z", "z": "x", "w": "w"}
    }
  }
}
```

## Complete Launch System

### launch/complete_assessment.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'complete_course.world'])

    # Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                pkg_share, 'urdf', 'assessment_robot.urdf'
            ])
        }],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Navigation controller
    navigation_node = Node(
        package='module_2_assessment',
        executable='advanced_navigation.py',
        name='navigator',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Sensor validator
    sensor_validator = Node(
        package='module_2_assessment',
        executable='laser_scan_validator.py',
        name='sensor_validator',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        navigation_node,
        sensor_validator
    ])
```

## Testing and Evaluation Scripts

### tests/comprehensive_test.py

```python
#!/usr/bin/env python3
"""
Comprehensive test script for the assessment project
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64
import time
import math

class ComprehensiveTest(Node):
    def __init__(self):
        super().__init__('comprehensive_test')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/robot/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/robot/imu', self.imu_callback, 10)

        # State variables
        self.initial_pose = None
        self.current_pose = None
        self.scan_data = None
        self.imu_data = None
        self.test_phase = 0
        self.test_start_time = None
        self.movement_start_time = None
        self.target_reached = False

        # Test parameters
        self.test_duration = 60.0  # seconds
        self.movement_duration = 10.0  # seconds
        self.target_tolerance = 0.5  # meters

        # Timer for test execution
        self.timer = self.create_timer(0.1, self.run_test)

        self.get_logger().info('Starting comprehensive test...')

    def odom_callback(self, msg):
        """Handle odometry messages"""
        self.current_pose = msg.pose.pose
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        self.scan_data = msg

    def imu_callback(self, msg):
        """Handle IMU messages"""
        self.imu_data = msg

    def run_test(self):
        """Main test execution loop"""
        if self.test_phase == 0:
            # Phase 0: Wait for sensor data to be available
            if self.current_pose and self.scan_data and self.imu_data:
                self.get_logger().info('All sensors are publishing, starting test')
                self.test_start_time = time.time()
                self.test_phase = 1
            else:
                # Log sensor status
                sensor_status = f"Odom: {'✓' if self.current_pose else '✗'}, "
                sensor_status += f"Scan: {'✓' if self.scan_data else '✗'}, "
                sensor_status += f"IMU: {'✓' if self.imu_data else '✗'}"
                self.get_logger().info(sensor_status, throttle_duration_sec=2)

        elif self.test_phase == 1:
            # Phase 1: Move forward for a set duration
            if self.movement_start_time is None:
                self.movement_start_time = time.time()
                self.move_forward(0.3)  # Move forward at 0.3 m/s
                self.get_logger().info('Starting forward movement test')

            elapsed = time.time() - self.movement_start_time
            if elapsed > self.movement_duration:
                self.stop_robot()
                self.get_logger().info(f'Forward movement completed after {elapsed:.1f}s')
                self.test_phase = 2
                self.movement_start_time = None

        elif self.test_phase == 2:
            # Phase 2: Check if we moved as expected
            if self.initial_pose and self.current_pose:
                dx = self.current_pose.position.x - self.initial_pose.position.x
                dy = self.current_pose.position.y - self.initial_pose.position.y
                distance_moved = math.sqrt(dx*dx + dy*dy)

                expected_distance = 0.3 * self.movement_duration  # 0.3 m/s for 10s
                if distance_moved > expected_distance * 0.5:  # At least 50% of expected
                    self.get_logger().info(f'Movement test passed: moved {distance_moved:.2f}m')
                else:
                    self.get_logger().warn(f'Movement test failed: only moved {distance_moved:.2f}m')

            self.test_phase = 3

        elif self.test_phase == 3:
            # Phase 3: Final evaluation
            total_elapsed = time.time() - self.test_start_time if self.test_start_time else 0
            if total_elapsed > self.test_duration:
                self.final_evaluation()
                self.test_phase = 4

    def move_forward(self, speed):
        """Move robot forward at given speed"""
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def final_evaluation(self):
        """Perform final evaluation of the system"""
        self.get_logger().info('=== FINAL EVALUATION ===')

        # Sensor evaluation
        sensors_ok = all([self.scan_data, self.imu_data, self.current_pose])
        if sensors_ok:
            self.get_logger().info('✓ All sensors are functional')
        else:
            self.get_logger().error('✗ Some sensors are not functional')

        # Movement evaluation
        if self.initial_pose and self.current_pose:
            dx = self.current_pose.position.x - self.initial_pose.position.x
            dy = self.current_pose.position.y - self.initial_pose.position.y
            distance_moved = math.sqrt(dx*dx + dy*dy)

            if distance_moved > 0.5:  # At least 50cm moved
                self.get_logger().info(f'✓ Movement functional: {distance_moved:.2f}m')
            else:
                self.get_logger().error(f'✗ Movement not functional: only {distance_moved:.2f}m')

        # Laser scan evaluation
        if self.scan_data:
            valid_ranges = [r for r in self.scan_data.ranges
                          if self.scan_data.range_min < r < self.scan_data.range_max]
            if len(valid_ranges) > len(self.scan_data.ranges) * 0.5:
                self.get_logger().info(f'✓ Laser scan functional: {len(valid_ranges)}/{len(self.scan_data.ranges)} valid ranges')
            else:
                self.get_logger().error('✗ Laser scan not functional')

        self.get_logger().info('=== TEST COMPLETE ===')

def main(args=None):
    rclpy.init(args=args)
    test_node = ComprehensiveTest()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices and Tips

### 1. Performance Optimization
- Use appropriate physics parameters (max_step_size, real_time_factor)
- Optimize collision geometries (use simple shapes when possible)
- Limit sensor update rates to realistic values
- Use fixed joints instead of high-stiffness revolute joints

### 2. Debugging Techniques
- Use Gazebo's visualization tools (contacts, wireframe, transparent models)
- Monitor ROS 2 topics with `ros2 topic echo`
- Use RViz2 for sensor data visualization
- Check simulation real-time factor (`gz stats`)

### 3. Validation Checklist
- [ ] Robot model loads without errors
- [ ] Physics simulation is stable
- [ ] All sensors publish valid data
- [ ] Navigation reaches targets successfully
- [ ] System performs within real-time constraints
- [ ] Error handling is robust

These examples provide complete, working implementations that demonstrate best practices for each component of the Module 2 assessment project. Use these as references when implementing your own solution, but remember to understand and modify them to fit your specific approach and requirements.

## Next Steps

After reviewing these solution examples:

1. Implement your own version based on these patterns
2. Test your implementation against [Evaluation Criteria](../../module-2/assessment-project/evaluation-criteria.md)
3. Prepare your final project submission
4. Review the [Project Overview](./project-scaffolding.md) to ensure all requirements are met