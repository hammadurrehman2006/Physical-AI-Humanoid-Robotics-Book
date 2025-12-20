---
sidebar_position: 1
---

# URDF Basics: Robot Description Format

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and sensors.

## Understanding URDF Structure

A basic URDF file consists of:
- **Links**: Rigid parts of the robot (e.g., chassis, wheels)
- **Joints**: Connections between links (e.g., hinges, sliders)
- **Visual**: How the robot appears in simulation
- **Collision**: Physical collision properties
- **Inertial**: Mass and inertia properties for physics simulation

## Basic URDF Example

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

## Key Components

### Links
Links represent rigid bodies in the robot. Each link must have:
- A unique name
- Visual properties (shape, color, mesh)
- Collision properties
- Inertial properties

### Joints
Joints connect links and define how they can move relative to each other:
- **Fixed**: No movement (weld)
- **Revolute**: Rotational movement with limits
- **Continuous**: Unlimited rotational movement
- **Prismatic**: Linear sliding movement
- **Floating**: 6DOF movement (no constraints)

## Creating Your First URDF

Let's create a simple robot with a base and a rotating wheel:

```xml
<?xml version="1.0"?>
<robot name="wheel_robot">
  <!-- Base of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.8 0.4 0.2"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.8 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel link -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.57075 0 0"/> <!-- Rotate to align with robot -->
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.57075 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0 -0.1"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Loading URDF in Gazebo

To load your URDF in Gazebo:

1. Save your URDF file with a `.urdf` extension
2. Convert to SDF format for Gazebo:
   ```bash
   gz sdf -p your_robot.urdf
   ```
3. Load in Gazebo:
   ```bash
   gz sim -r your_robot.sdf
   ```

## Best Practices

- Use meaningful names for links and joints
- Always define inertial properties for physics simulation
- Keep visual and collision geometries consistent
- Use separate files for complex robots and include them with `<xacro>` (more advanced)

## Next Steps

Continue to learn about [SDF Advanced Features](./sdf-advanced.md) to understand how Gazebo's native format differs from URDF.