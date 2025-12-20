---
sidebar_position: 4
---

# Conversion Guide: URDF to SDF and Back

Understanding when and how to convert between URDF and SDF formats is crucial for effective robotics simulation. This guide covers the conversion processes and best practices.

## When to Use Each Format

### Use URDF When:
- Working primarily within the ROS ecosystem
- Focusing on robot kinematics and basic descriptions
- Creating models that need ROS 2 integration
- Building reusable robot components
- Leveraging xacro macros for complex models

### Use SDF When:
- Utilizing Gazebo-specific features (sensors, plugins)
- Creating complete simulation worlds
- Requiring advanced physics properties
- Working with non-ROS simulation scenarios
- Needing world descriptions with multiple models

## Converting URDF to SDF

### Command Line Conversion

The simplest way to convert URDF to SDF:

```bash
# Basic conversion
gz sdf -p robot.urdf > robot.sdf

# With xacro preprocessing (if using macros)
xacro robot.urdf.xacro | gz sdf -p /dev/stdin > robot.sdf

# Validate URDF before conversion
check_urdf robot.urdf
```

### Example Conversion Process

**Input URDF (robot.urdf):**
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
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

**Converted SDF:**
```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="simple_robot">
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>1</mass>
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
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
          <specular>0 0 1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

## Adding Gazebo-Specific Elements to URDF

When you need Gazebo features in a URDF, add Gazebo-specific tags:

```xml
<!-- Add to the end of your URDF file -->

<!-- Gazebo plugin for ROS 2 control -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
  </plugin>
</gazebo>

<!-- Gazebo plugin for joint state publisher -->
<gazebo>
  <plugin filename="libgazebo_ros_joint_state_publisher.so" name="joint_state_publisher">
    <ros>
      <namespace>robot</namespace>
    </ros>
    <update_rate>30</update_rate>
    <joint_name>left_wheel_joint</joint_name>
    <joint_name>right_wheel_joint</joint_name>
  </plugin>
</gazebo>

<!-- Sensor in a URDF link -->
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

<!-- Material definitions -->
<gazebo reference="base_link_visual">
  <material>Gazebo/Blue</material>
</gazebo>
```

## Converting SDF to URDF

While SDF to URDF conversion is not directly supported by tools, you can create equivalent URDF from SDF by understanding the mapping:

| SDF Element | URDF Equivalent |
|-------------|-----------------|
| `<model>` | `<robot>` |
| `<link>` | `<link>` |
| `<joint>` | `<joint>` |
| `<visual>` | `<visual>` |
| `<collision>` | `<collision>` |
| `<inertial>` | `<inertial>` |
| `<sensor>` | `<link>` + Gazebo plugin |

## Advanced Conversion Scenarios

### Using xacro for Complex Conversions

For complex robots, use xacro to create parameterized URDFs that can be converted to SDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="param_robot">

  <!-- Define parameters -->
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.5"/>
  <xacro:property name="base_height" value="0.2"/>

  <!-- Include other xacro files -->
  <xacro:include filename="$(find my_robot_description)/urdf/wheel.xacro"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Use macro to create wheels -->
  <xacro:wheel prefix="left" parent="base_link" x="0" y="0.3" z="0"/>
  <xacro:wheel prefix="right" parent="base_link" x="0" y="-0.3" z="0"/>

</robot>
```

### Using URDF with Gazebo Worlds

To use a URDF robot in a complete Gazebo world, you have several options:

1. **Spawn the model programmatically** using ROS services
2. **Convert to SDF and embed in world file**
3. **Use Gazebo's model database** by adding your URDF to the models directory

## Tools and Validation

### URDF Validation
```bash
# Check URDF syntax and structure
check_urdf robot.urdf

# Visualize URDF
urdf_to_graphiz robot.urdf
```

### SDF Validation
```bash
# Validate SDF syntax
gz sdf -k robot.sdf

# Convert and check SDF
gz sdf -p robot.urdf
```

### ROS 2 Integration
```bash
# Test with ros2_control
ros2 launch my_robot_description spawn.launch.py

# Check joint states
ros2 topic echo /joint_states
```

## Common Conversion Issues and Solutions

### Issue 1: Missing Inertial Properties
**Problem**: Robot falls through the ground in simulation
**Solution**: Ensure all links have proper inertial properties

### Issue 2: Sensor Not Working
**Problem**: Gazebo plugins not functioning
**Solution**: Verify plugin names and ROS interfaces are correct

### Issue 3: Joint Limits Not Respected
**Problem**: Joints moving beyond physical limits
**Solution**: Add proper limits in both URDF and SDF formats

### Issue 4: Material Colors Different
**Problem**: Colors not appearing as expected
**Solution**: Use consistent material definitions between formats

## Best Practices

1. **Plan Your Format Strategy**: Decide which format to use based on your primary workflow
2. **Use Tools for Conversion**: Leverage `gz sdf` and `xacro` for complex conversions
3. **Validate After Conversion**: Always test converted models in simulation
4. **Keep Formats in Sync**: Maintain both formats if both are needed
5. **Document Conversion Steps**: Record your conversion process for reproducibility

## Next Steps

Now that you understand URDF and SDF formats, continue to learn about [Physics Simulation](../physics-simulation/gravity-and-collisions.md) to bring your robot models to life with realistic physics.