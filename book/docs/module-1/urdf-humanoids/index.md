---
title: URDF for Humanoids
description: Understanding Unified Robot Description Format for humanoid robot modeling
sidebar_position: 8
---

# URDF for Humanoids

## Learning Objectives
- Understand the structure and components of URDF (Unified Robot Description Format)
- Learn to create URDF models for humanoid robots
- Master the use of xacro for complex humanoid descriptions
- Implement kinematic chains and joint constraints
- Validate and visualize humanoid robot models

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links, joints, inertial properties, and visual elements. For humanoid robots, URDF is essential for simulation, visualization, and kinematic analysis.

### URDF Components Overview

A URDF model consists of:

1. **Links**: Rigid bodies that make up the robot structure
2. **Joints**: Connections between links that define motion
3. **Visual**: How the robot appears in simulation/visualization
4. **Collision**: How the robot interacts with the environment
5. **Inertial**: Physical properties for dynamics simulation
6. **Materials**: Visual appearance properties

## Basic URDF Structure

### Simple Link Definition

```xml
<!-- Basic link structure -->
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Joint Definition

```xml
<!-- Joint connecting two links -->
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Humanoid Robot URDF Example

### Complete Humanoid Skeleton URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>

  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>

  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>

  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link - pelvis -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.2"/>
      </geometry>
      <material name="grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="8.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.4"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.15 0.4"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.15 0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.03"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="0.5" effort="50" velocity="2"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <origin xyz="0 0 0.12" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <!-- Right Arm (similar to left, mirrored) -->
  <link name="right_shoulder">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0.15 -0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.03"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="0.5" effort="50" velocity="2"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <origin xyz="0 0 0.12" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_hip">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.03"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="-0.1 0.05 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.08"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_upper_leg"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <origin xyz="0 0 0.18" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.06"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.18" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.36"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.18" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.36"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_joint" type="fixed">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 0.36" rpy="0 0 0"/>
  </joint>

  <!-- Right Leg (similar to left, mirrored) -->
  <link name="right_hip">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.03"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="brown"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="-0.1 -0.05 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.08"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="brown"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_upper_leg"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <origin xyz="0 0 0.18" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.06" ixy="0.0" ixz="0.0" iyy="0.06" iyz="0.0" izz="0.06"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.18" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.36"/>
      </geometry>
      <material name="brown"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.18" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.36"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
  </joint>

  <link name="right_foot">
    <inertial>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="brown"/>
    </visual>

    <collision>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_foot_joint" type="fixed">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 0.36" rpy="0 0 0"/>
  </joint>

</robot>
```

## Using Xacro for Complex Humanoid Models

Xacro (XML Macros) simplifies complex URDF models by allowing macros, constants, and includes.

### Basic Xacro Structure

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_mass" value="50.0" />

  <!-- Define materials -->
  <xacro:macro name="color_material" params="name color_r color_g color_b">
    <material name="${name}">
      <color rgba="${color_r} ${color_g} ${color_b} 1.0"/>
    </material>
  </xacro:macro>

  <!-- Define a generic link macro -->
  <xacro:macro name="generic_link"
               params="name mass xyz ixx iyy izz rpy size_xyz color">
    <link name="${name}">
      <inertial>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <mass value="${mass}"/>
        <inertia ixx="${ixx}" ixy="0.0" ixz="0.0"
                 iyy="${iyy}" iyz="0.0" izz="${izz}"/>
      </inertial>

      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size_xyz}"/>
        </geometry>
        <material name="${color}"/>
      </visual>

      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <box size="${size_xyz}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Define a generic joint macro -->
  <xacro:macro name="generic_joint"
               params="name type parent child xyz rpy axis lower upper effort velocity">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
    </joint>
  </xacro:macro>

  <!-- Use the macros to build the robot -->
  <xacro:color_material name="body_color" color_r="0.8" color_g="0.8" color_b="0.8"/>

  <xacro:generic_link
    name="base_link"
    mass="10.0"
    xyz="0 0 0.1"
    ixx="0.5" iyy="0.5" izz="0.5"
    rpy="0 0 0"
    size_xyz="0.3 0.2 0.2"
    color="body_color"/>

</robot>
```

### Complete Humanoid Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="advanced_humanoid">

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Robot dimensions -->
  <xacro:property name="torso_height" value="0.4" />
  <xacro:property name="torso_width" value="0.25" />
  <xacro:property name="torso_depth" value="0.15" />

  <xacro:property name="head_radius" value="0.1" />
  <xacro:property name="upper_arm_length" value="0.3" />
  <xacro:property name="lower_arm_length" value="0.25" />
  <xacro:property name="upper_leg_length" value="0.4" />
  <xacro:property name="lower_leg_length" value="0.35" />
  <xacro:property name="foot_size" value="0.15 0.08 0.05" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="grey">
    <color rgba="0.6 0.6 0.6 1.0"/>
  </material>

  <!-- Generic link macro -->
  <xacro:macro name="simple_link" params="name mass xyz ixx iyy izz geometry_type *geometry_origin *geometry">
    <link name="${name}">
      <inertial>
        <origin xyz="${xyz}" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia ixx="${ixx}" ixy="0.0" ixz="0.0"
                 iyy="${iyy}" iyz="0.0" izz="${izz}"/>
      </inertial>

      <visual>
        <xacro:insert_block name="geometry_origin"/>
        <xacro:insert_block name="geometry"/>
        <material name="grey"/>
      </visual>

      <collision>
        <xacro:insert_block name="geometry_origin"/>
        <xacro:insert_block name="geometry"/>
      </collision>
    </link>
  </xacro:macro>

  <!-- Generic joint macro -->
  <xacro:macro name="simple_joint"
               params="name type parent child xyz rpy axis lower upper effort velocity">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <xacro:simple_link name="base_link"
                     mass="10.0"
                     xyz="0 0 0.1"
                     ixx="0.5" iyy="0.5" izz="0.5">
    <xacro:origin xyz="0 0 0.1" rpy="0 0 0"/>
    <xacro:box size="0.3 0.2 0.2"/>
  </xacro:simple_link>

  <!-- Torso -->
  <xacro:simple_link name="torso"
                     mass="8.0"
                     xyz="0 0 ${torso_height/2}"
                     ixx="0.4" iyy="0.4" izz="0.4">
    <xacro:origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
    <xacro:box size="${torso_width} ${torso_depth} ${torso_height}"/>
  </xacro:simple_link>

  <xacro:simple_joint name="torso_joint"
                      type="fixed"
                      parent="base_link"
                      child="torso"
                      xyz="0 0 0.2"
                      rpy="0 0 0"
                      axis="1 0 0"
                      lower="0" upper="0"
                      effort="0" velocity="0"/>

  <!-- Head -->
  <xacro:simple_link name="head"
                     mass="2.0"
                     xyz="0 0 0"
                     ixx="0.1" iyy="0.1" izz="0.1">
    <xacro:origin xyz="0 0 0" rpy="0 0 0"/>
    <xacro:sphere radius="${head_radius}"/>
  </xacro:simple_link>

  <xacro:simple_joint name="neck_joint"
                      type="revolute"
                      parent="torso"
                      child="head"
                      xyz="0 0 ${torso_height}"
                      rpy="0 0 0"
                      axis="0 1 0"
                      lower="${-M_PI/3}" upper="${M_PI/3}"
                      effort="10" velocity="1"/>

  <!-- Left Arm -->
  <xacro:macro name="arm_chain" params="side position sign">
    <!-- Shoulder -->
    <xacro:simple_link name="${side}_shoulder"
                       mass="1.5"
                       xyz="0 0 0.1"
                       ixx="0.02" iyy="0.02" izz="0.02">
      <xacro:origin xyz="0 0 0.1" rpy="0 0 0"/>
      <xacro:box size="0.1 0.1 0.2"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_shoulder_joint"
                        type="revolute"
                        parent="torso"
                        child="${side}_shoulder"
                        xyz="${torso_width/2} ${sign * torso_depth/2} ${torso_height/2}"
                        rpy="0 0 0"
                        axis="0 1 0"
                        lower="${-M_PI/2}" upper="${M_PI/2}"
                        effort="50" velocity="2"/>

    <!-- Upper Arm -->
    <xacro:simple_link name="${side}_upper_arm"
                       mass="2.0"
                       xyz="0 0 ${upper_arm_length/2}"
                       ixx="0.03" iyy="0.03" izz="0.03">
      <xacro:origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
      <xacro:cylinder radius="0.05" length="${upper_arm_length}"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_elbow_joint"
                        type="revolute"
                        parent="${side}_shoulder"
                        child="${side}_upper_arm"
                        xyz="0 0 0.2"
                        rpy="0 0 0"
                        axis="0 0 1"
                        lower="${-2*M_PI/3}" upper="${M_PI/6}"
                        effort="50" velocity="2"/>

    <!-- Lower Arm -->
    <xacro:simple_link name="${side}_lower_arm"
                       mass="1.5"
                       xyz="0 0 ${lower_arm_length/2}"
                       ixx="0.02" iyy="0.02" izz="0.02">
      <xacro:origin xyz="0 0 ${lower_arm_length/2}" rpy="0 0 0"/>
      <xacro:cylinder radius="0.04" length="${lower_arm_length}"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_wrist_joint"
                        type="revolute"
                        parent="${side}_upper_arm"
                        child="${side}_lower_arm"
                        xyz="0 0 ${upper_arm_length}"
                        rpy="0 0 0"
                        axis="0 1 0"
                        lower="${-M_PI/2}" upper="${M_PI/2}"
                        effort="20" velocity="3"/>
  </xacro:macro>

  <!-- Create both arms -->
  <xacro:arm_chain side="left" position="left" sign="1"/>
  <xacro:arm_chain side="right" position="right" sign="-1"/>

  <!-- Leg Macro -->
  <xacro:macro name="leg_chain" params="side sign">
    <!-- Hip -->
    <xacro:simple_link name="${side}_hip"
                       mass="2.0"
                       xyz="0 0 0.1"
                       ixx="0.03" iyy="0.03" izz="0.03">
      <xacro:origin xyz="0 0 0.1" rpy="0 0 0"/>
      <xacro:box size="0.1 0.1 0.2"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_hip_joint"
                        type="revolute"
                        parent="base_link"
                        child="${side}_hip"
                        xyz="${-0.1} ${sign * 0.05} 0"
                        rpy="0 0 0"
                        axis="0 0 1"
                        lower="${-M_PI/3}" upper="${M_PI/3}"
                        effort="100" velocity="1"/>

    <!-- Upper Leg -->
    <xacro:simple_link name="${side}_upper_leg"
                       mass="3.0"
                       xyz="0 0 ${upper_leg_length/2}"
                       ixx="0.08" iyy="0.08" izz="0.08">
      <xacro:origin xyz="0 0 ${upper_leg_length/2}" rpy="0 0 0"/>
      <xacro:cylinder radius="0.06" length="${upper_leg_length}"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_knee_joint"
                        type="revolute"
                        parent="${side}_hip"
                        child="${side}_upper_leg"
                        xyz="0 0 0.2"
                        rpy="0 0 0"
                        axis="0 1 0"
                        lower="0" upper="${M_PI/2}"
                        effort="100" velocity="1"/>

    <!-- Lower Leg -->
    <xacro:simple_link name="${side}_lower_leg"
                       mass="2.5"
                       xyz="0 0 ${lower_leg_length/2}"
                       ixx="0.06" iyy="0.06" izz="0.06">
      <xacro:origin xyz="0 0 ${lower_leg_length/2}" rpy="0 0 0"/>
      <xacro:cylinder radius="0.05" length="${lower_leg_length}"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_ankle_joint"
                        type="revolute"
                        parent="${side}_upper_leg"
                        child="${side}_lower_leg"
                        xyz="0 0 ${upper_leg_length}"
                        rpy="0 0 0"
                        axis="0 0 1"
                        lower="${-M_PI/6}" upper="${M_PI/6}"
                        effort="50" velocity="1"/>

    <!-- Foot -->
    <xacro:simple_link name="${side}_foot"
                       mass="1.0"
                       xyz="0.05 0 0"
                       ixx="0.01" iyy="0.01" izz="0.01">
      <xacro:origin xyz="0.05 0 0" rpy="0 0 0"/>
      <xacro:box size="${foot_size}"/>
    </xacro:simple_link>

    <xacro:simple_joint name="${side}_foot_joint"
                        type="fixed"
                        parent="${side}_lower_leg"
                        child="${side}_foot"
                        xyz="0 0 ${lower_leg_length}"
                        rpy="0 0 0"
                        axis="1 0 0"
                        lower="0" upper="0"
                        effort="0" velocity="0"/>
  </xacro:macro>

  <!-- Create both legs -->
  <xacro:leg_chain side="left" sign="1"/>
  <xacro:leg_chain side="right" sign="-1"/>

  <!-- Gazebo plugin for simulation -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

## URDF Validation and Testing

### Python Script for URDF Validation

```python
#!/usr/bin/env python3
"""
URDF Validation Script for Humanoid Robots
"""

import xml.etree.ElementTree as ET
import sys
from collections import defaultdict

def validate_urdf(urdf_file):
    """Validate URDF file structure and content."""
    try:
        # Parse the URDF file
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        if root.tag != 'robot':
            print(f"ERROR: Root element is not 'robot', got '{root.tag}'")
            return False

        robot_name = root.get('name', '')
        if not robot_name:
            print("ERROR: Robot name is missing")
            return False

        print(f"Validating robot: {robot_name}")

        # Get all links and joints
        links = root.findall('link')
        joints = root.findall('joint')

        print(f"Found {len(links)} links and {len(joints)} joints")

        # Check for required elements in each link
        for link in links:
            link_name = link.get('name')
            if not link_name:
                print(f"ERROR: Link without name found")
                return False

            # Check for required sub-elements
            if link.find('inertial') is None:
                print(f"WARNING: Link '{link_name}' missing inertial properties")

            if link.find('visual') is None:
                print(f"WARNING: Link '{link_name}' missing visual properties")

            if link.find('collision') is None:
                print(f"WARNING: Link '{link_name}' missing collision properties")

        # Check joint connections
        link_names = {link.get('name') for link in links}
        connected_links = set()

        for joint in joints:
            joint_name = joint.get('name')
            joint_type = joint.get('type')

            if not joint_name:
                print(f"ERROR: Joint without name found")
                return False

            parent_elem = joint.find('parent')
            child_elem = joint.find('child')

            if parent_elem is None or child_elem is None:
                print(f"ERROR: Joint '{joint_name}' missing parent or child")
                return False

            parent_link = parent_elem.get('link')
            child_link = child_elem.get('link')

            if parent_link not in link_names:
                print(f"ERROR: Joint '{joint_name}' references non-existent parent link '{parent_link}'")
                return False

            if child_link not in link_names:
                print(f"ERROR: Joint '{joint_name}' references non-existent child link '{child_link}'")
                return False

            connected_links.add(parent_link)
            connected_links.add(child_link)

        # Check for disconnected links (except base link)
        disconnected = link_names - connected_links
        if disconnected:
            print(f"WARNING: Disconnected links found: {disconnected}")

        print("URDF validation passed!")
        return True

    except ET.ParseError as e:
        print(f"ERROR: Invalid XML format - {e}")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def check_kinematic_chain(urdf_file):
    """Check if the URDF forms a proper kinematic chain."""
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        joints = root.findall('joint')

        # Build parent-child relationships
        parent_child = {}
        child_parent = {}

        for joint in joints:
            parent_elem = joint.find('parent')
            child_elem = joint.find('child')

            if parent_elem is not None and child_elem is not None:
                parent_link = parent_elem.get('link')
                child_link = child_elem.get('link')

                parent_child.setdefault(parent_link, []).append(child_link)
                child_parent[child_link] = parent_link

        # Find root link (has no parent)
        all_links = {link.get('name') for link in root.findall('link')}
        root_links = all_links - set(child_parent.keys())

        if len(root_links) == 0:
            print("ERROR: No root link found - circular joint structure")
            return False
        elif len(root_links) > 1:
            print(f"WARNING: Multiple root links found: {root_links}")

        print(f"Root link(s): {root_links}")
        return True

    except Exception as e:
        print(f"ERROR in kinematic check: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 urdf_validator.py <urdf_file>")
        sys.exit(1)

    urdf_file = sys.argv[1]

    print(f"Validating URDF file: {urdf_file}")
    print("=" * 50)

    if validate_urdf(urdf_file):
        print("\nChecking kinematic chain...")
        check_kinematic_chain(urdf_file)
    else:
        print("URDF validation failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()
```

## Visualization and Debugging

### Launch File for URDF Visualization

```python
# launch/urdf_display.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='simple_humanoid.urdf',
        description='URDF file name'
    )

    # Get URDF path
    urdf_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        LaunchConfiguration('urdf_file')
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'publish_frequency': 50.0
        }]
    )

    # Joint state publisher (GUI)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('humanoid_description'),
            'rviz',
            'urdf_viewer.rviz'
        )]
    )

    return LaunchDescription([
        urdf_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
```

### RViz Configuration for URDF

```yaml
# rviz/urdf_viewer.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
      Splitter Ratio: 0.5
    Tree Height: 617
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa000025a9000000a3fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000016100000002000000000000000000000000000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1200
  X: 72
  Y: 60
```

## Hands-on Exercise: Humanoid URDF Creation

Create a complete humanoid robot model with proper kinematic structure.

### Exercise Requirements
1. Create a humanoid URDF with all major body parts
2. Implement proper kinematic chains for arms and legs
3. Add visual and collision properties
4. Validate the URDF structure

### Complete Exercise Implementation

```xml
<?xml version="1.0"?>
<robot name="exercise_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include common definitions -->
  <xacro:include filename="$(find humanoid_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/transmission_macros.xacro"/>

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="M_PI_2" value="1.5707963267948966"/>

  <!-- Humanoid dimensions -->
  <xacro:property name="torso_height" value="0.5"/>
  <xacro:property name="torso_width" value="0.3"/>
  <xacro:property name="torso_depth" value="0.2"/>

  <xacro:property name="head_radius" value="0.12"/>
  <xacro:property name="upper_arm_length" value="0.35"/>
  <xacro:property name="lower_arm_length" value="0.3"/>
  <xacro:property name="arm_radius" value="0.06"/>

  <xacro:property name="upper_leg_length" value="0.45"/>
  <xacro:property name="lower_leg_length" value="0.4"/>
  <xacro:property name="leg_radius" value="0.07"/>

  <xacro:property name="foot_size" value="0.18 0.09 0.06"/>

  <!-- Base link (pelvis) -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="12.0"/>
      <inertia ixx="0.8" ixy="0.0" ixz="0.0" iyy="0.8" iyz="0.0" izz="0.8"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.2"/>
      </geometry>
      <material name="light_grey"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>

    <visual>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="tan"/>
    </visual>

    <collision>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${head_radius}"/>
      </geometry>
      <material name="white"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${head_radius}"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI_2/3}" upper="${M_PI_2/3}" effort="10" velocity="1"/>
  </joint>

  <!-- Arms -->
  <xacro:macro name="arm_chain" params="side sign">
    <!-- Shoulder -->
    <link name="${side}_shoulder">
      <inertial>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <mass value="2.0"/>
        <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.03"/>
      </inertial>

      <visual>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.12 0.12 0.1"/>
        </geometry>
        <material name="${side}_arm_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.12 0.12 0.1"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${side}_shoulder"/>
      <origin xyz="${torso_width/2} ${sign * torso_depth/2} ${torso_height*0.7}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI_2}" upper="${M_PI_2}" effort="100" velocity="2"/>
    </joint>

    <!-- Upper Arm -->
    <link name="${side}_upper_arm">
      <inertial>
        <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
        <mass value="2.5"/>
        <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.08"/>
      </inertial>

      <visual>
        <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${arm_radius}" length="${upper_arm_length}"/>
        </geometry>
        <material name="${side}_arm_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${arm_radius}" length="${upper_arm_length}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_shoulder"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-2*M_PI_2/3}" upper="${M_PI_2/6}" effort="100" velocity="2"/>
    </joint>

    <!-- Lower Arm -->
    <link name="${side}_lower_arm">
      <inertial>
        <origin xyz="0 0 ${lower_arm_length/2}" rpy="0 0 0"/>
        <mass value="1.8"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
      </inertial>

      <visual>
        <origin xyz="0 0 ${lower_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${arm_radius*0.8}" length="${lower_arm_length}"/>
        </geometry>
        <material name="${side}_arm_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 ${lower_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${arm_radius*0.8}" length="${lower_arm_length}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_wrist_joint" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_lower_arm"/>
      <origin xyz="0 0 ${upper_arm_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI_2}" upper="${M_PI_2}" effort="50" velocity="3"/>
    </joint>

    <!-- Hand (simplified) -->
    <link name="${side}_hand">
      <inertial>
        <origin xyz="0.05 0 0" rpy="0 0 0"/>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>

      <visual>
        <origin xyz="0.05 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.1 0.06 0.06"/>
        </geometry>
        <material name="${side}_arm_color"/>
      </visual>

      <collision>
        <origin xyz="0.05 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.1 0.06 0.06"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_hand_joint" type="fixed">
      <parent link="${side}_lower_arm"/>
      <child link="${side}_hand"/>
      <origin xyz="0 0 ${lower_arm_length}" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <!-- Define arm colors -->
  <xacro:macro name="arm_color" params="side color">
    <material name="${side}_arm_color">
      <color rgba="${color}"/>
    </material>
  </xacro:macro>

  <xacro:arm_color side="left" color="0.0 0.0 0.8 1.0"/>
  <xacro:arm_color side="right" color="0.0 0.8 0.0 1.0"/>

  <!-- Create arms -->
  <xacro:arm_chain side="left" sign="1"/>
  <xacro:arm_chain side="right" sign="-1"/>

  <!-- Legs -->
  <xacro:macro name="leg_chain" params="side sign">
    <!-- Hip -->
    <link name="${side}_hip">
      <inertial>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <mass value="3.0"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
      </inertial>

      <visual>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.14 0.14 0.1"/>
        </geometry>
        <material name="${side}_leg_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.14 0.14 0.1"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_hip"/>
      <origin xyz="${-0.12} ${sign * 0.06} 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI_2/3}" upper="${M_PI_2/3}" effort="150" velocity="1"/>
    </joint>

    <!-- Upper Leg -->
    <link name="${side}_upper_leg">
      <inertial>
        <origin xyz="0 0 ${upper_leg_length/2}" rpy="0 0 0"/>
        <mass value="4.0"/>
        <inertia ixx="0.15" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.15"/>
      </inertial>

      <visual>
        <origin xyz="0 0 ${upper_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${leg_radius}" length="${upper_leg_length}"/>
        </geometry>
        <material name="${side}_leg_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 ${upper_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${leg_radius}" length="${upper_leg_length}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_knee_joint" type="revolute">
      <parent link="${side}_hip"/>
      <child link="${side}_upper_leg"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="${M_PI_2}" effort="150" velocity="1"/>
    </joint>

    <!-- Lower Leg -->
    <link name="${side}_lower_leg">
      <inertial>
        <origin xyz="0 0 ${lower_leg_length/2}" rpy="0 0 0"/>
        <mass value="3.5"/>
        <inertia ixx="0.12" ixy="0.0" ixz="0.0" iyy="0.12" iyz="0.0" izz="0.12"/>
      </inertial>

      <visual>
        <origin xyz="0 0 ${lower_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${leg_radius*0.9}" length="${lower_leg_length}"/>
        </geometry>
        <material name="${side}_leg_color"/>
      </visual>

      <collision>
        <origin xyz="0 0 ${lower_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${leg_radius*0.9}" length="${lower_leg_length}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_ankle_joint" type="revolute">
      <parent link="${side}_upper_leg"/>
      <child link="${side}_lower_leg"/>
      <origin xyz="0 0 ${upper_leg_length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI_2/6}" upper="${M_PI_2/6}" effort="80" velocity="1"/>
    </joint>

    <!-- Foot -->
    <link name="${side}_foot">
      <inertial>
        <origin xyz="0.07 0 0" rpy="0 0 0"/>
        <mass value="1.2"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>

      <visual>
        <origin xyz="0.07 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${foot_size}"/>
        </geometry>
        <material name="${side}_leg_color"/>
      </visual>

      <collision>
        <origin xyz="0.07 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="${foot_size}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${side}_foot_joint" type="fixed">
      <parent link="${side}_lower_leg"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 ${lower_leg_length}" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <!-- Define leg colors -->
  <xacro:macro name="leg_color" params="side color">
    <material name="${side}_leg_color">
      <color rgba="${color}"/>
    </material>
  </xacro:macro>

  <xacro:leg_color side="left" color="0.8 0.0 0.0 1.0"/>
  <xacro:leg_color side="right" color="0.6 0.3 0.0 1.0"/>

  <!-- Create legs -->
  <xacro:leg_chain side="left" sign="1"/>
  <xacro:leg_chain side="right" sign="-1"/>

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

## Troubleshooting Common URDF Issues

### 1. Joint Connection Issues

```bash
# Problem: Joints not connecting properly
# Solution: Verify parent-child relationships

# Check joint connections
gz topic -e /gazebo/default/world/empty/model/humanoid/joint
# or use ROS2 to check TF tree
ros2 run tf2_tools view_frames
```

### 2. Inertial Property Issues

```xml
<!-- Common mistake: Invalid inertia values -->
<link name="example_link">
  <inertial>
    <!-- WRONG: Negative or zero values can cause simulation errors -->
    <inertia ixx="-1.0" iyy="0.0" izz="0.0"/> <!-- Don't do this -->
  </inertial>
</link>

<!-- CORRECT: Positive definite inertia matrix -->
<link name="example_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0"
             iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

### 3. Xacro Processing Issues

```bash
# Problem: Xacro not processing correctly
# Solution: Check syntax and dependencies

# Process xacro to URDF
ros2 run xacro xacro -o output.urdf input.xacro

# Or with parameters
ros2 run xacro xacro param:=value -o output.urdf input.xacro
```

## Resources for Further Learning

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Documentation](https://wiki.ros.org/xacro)
- [Gazebo URDF Integration](http://gazebosim.org/tutorials?tut=ros2_overview)
- [Robotics Toolbox for Python](https://pypi.org/project/roboticstoolbox-python/)

## Summary

URDF is fundamental for representing humanoid robots in ROS 2. Creating proper URDF models requires understanding of kinematic chains, inertial properties, and visualization elements. Xacro simplifies complex models with macros and parameters. Proper validation and visualization ensure the model works correctly in simulation and real applications.