---
sidebar_position: 3
---

# Creating Robot Models: Practical Examples

This tutorial will guide you through creating practical robot models for simulation, from simple wheeled robots to more complex articulated systems.

## Simple Differential Drive Robot

Let's create a complete differential drive robot model that can be used in both URDF and SDF formats.

### URDF Version (differential_drive_robot.urdf)

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.15"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.15 0 -0.07" rpy="0 0 0"/>
  </joint>
</robot>
```

### Adding ROS 2 Control Integration

To make the robot controllable with ROS 2, we need to add transmission and joint state publisher elements:

```xml
<!-- Add to the URDF after the joints -->
<transmission name="left_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<transmission name="right_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="right_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="right_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!-- Gazebo plugin for ROS 2 control -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
  </plugin>
</gazebo>
```

## Adding Sensors to Your Robot

Let's enhance our robot with sensors:

### URDF with LiDAR sensor

```xml
<!-- Add a mounting link for the LiDAR -->
<link name="lidar_mount">
  <visual>
    <geometry>
      <cylinder radius="0.02" length="0.01"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.02" length="0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<!-- Joint to attach LiDAR mount to base -->
<joint name="lidar_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_mount"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>

<!-- LiDAR sensor link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Joint to attach LiDAR to mount -->
<joint name="lidar_joint" type="fixed">
  <parent link="lidar_mount"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.025" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for LiDAR sensor -->
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
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Creating SDF World with Your Robot

To use your robot in a complete simulation environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="differential_drive_world">
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

    <!-- Include your robot from the URDF file -->
    <include>
      <uri>model://differential_drive_robot</uri>
      <pose>0 0 0.2 0 0 0</pose>
    </include>

    <!-- Add some obstacles -->
    <model name="box_obstacle">
      <pose>2 2 0.2 0 0 0</pose>
      <link name="box_link">
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
        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.4 0.2 1</ambient>
            <diffuse>1.0 0.5 0.25 1</diffuse>
          </material>
        </visual>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.4</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Testing Your Robot Model

1. Save your URDF file as `differential_drive_robot.urdf`
2. Convert to SDF to check for errors:
   ```bash
   gz sdf -p differential_drive_robot.urdf
   ```
3. Launch Gazebo with your robot:
   ```bash
   gz sim -r differential_drive_robot.sdf
   ```

## Best Practices for Robot Modeling

- **Start Simple**: Begin with basic shapes and add complexity gradually
- **Proper Inertial Properties**: Accurate inertial properties prevent simulation instabilities
- **Realistic Dimensions**: Use real-world measurements for accurate simulation
- **Collision vs Visual**: Keep collision geometries simple for performance
- **Organize Files**: Use separate files for complex robots and include them via xacro
- **Validate**: Always test your models in simulation before complex usage

## Next Steps

Continue to learn about [URDF to SDF Conversion](./conversion-guide.md) to understand the best approaches for different use cases.