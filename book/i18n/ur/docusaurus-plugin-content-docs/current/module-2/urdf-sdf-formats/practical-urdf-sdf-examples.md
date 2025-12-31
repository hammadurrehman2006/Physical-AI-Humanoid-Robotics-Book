---
sidebar_position: 5
---

# Practical URDF/SDF Examples and Loading in Gazebo

This tutorial provides practical examples of both URDF and SDF formats and demonstrates how to load them in Gazebo with real-world scenarios.

## Complete Differential Drive Robot Example

Let's create a complete differential drive robot that can be loaded in Gazebo, demonstrating both formats.

### URDF Version (differential_drive_robot.urdf)

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
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
        <box size="0.4 0.3 0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
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
      <parameters>$(find my_robot_description)/config/robot_control.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

### SDF Version (differential_drive_robot.sdf)

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="differential_drive_robot">
    <!-- Base link -->
    <link name="base_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.4 0.3 0.15</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.4 0.3 0.15</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>0 0.15 -0.05 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <pose>0 0 0 1.5708 0 0</pose>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <pose>0 0 0 1.5708 0 0</pose>
      </collision>
    </link>

    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>0 -0.15 -0.05 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <pose>0 0 0 1.5708 0 0</pose>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <pose>0 0 0 1.5708 0 0</pose>
      </collision>
    </link>

    <!-- Caster wheel -->
    <link name="caster_wheel">
      <pose>-0.15 0 -0.07 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="caster_visual">
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="caster_collision">
        <geometry>
          <sphere>
            <radius>0.03</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="caster_joint" type="fixed">
      <parent>base_link</parent>
      <child>caster_wheel</child>
    </joint>

    <!-- Differential drive plugin -->
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>robot</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>false</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </model>
</sdf>
```

## Loading Examples in Gazebo

### Method 1: Direct SDF Loading

```bash
# Load the SDF file directly
gz sim -r differential_drive_robot.sdf

# Or with a world file that includes the robot
gz sim -r robot_world.sdf
```

### Method 2: URDF to SDF Conversion and Loading

```bash
# Convert URDF to SDF
gz sdf -p differential_drive_robot.urdf > robot_as_sdf.sdf

# Load the converted SDF
gz sim -r robot_as_sdf.sdf
```

### Method 3: Using Gazebo Model Database

1. Create a model directory:
```bash
mkdir -p ~/.gazebo/models/differential_drive_robot
```

2. Create the model.config file:
```xml
<?xml version="1.0"?>
<model>
  <name>differential_drive_robot</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A differential drive robot for simulation.</description>
</model>
```

3. Place your SDF file as `model.sdf` in the directory

4. Launch Gazebo and use the Insert tab to add your robot

## Testing Robot Models

### Verification Steps

1. **Load the model**:
   ```bash
   gz sim -r your_robot.sdf
   ```

2. **Check for errors** in the terminal output

3. **Verify physics**:
   - Does the robot rest on the ground properly?
   - Do wheels rotate when commanded?
   - Are collisions detected correctly?

4. **Check ROS 2 integration** (if applicable):
   ```bash
   # List topics
   ros2 topic list | grep robot

   # Check joint states
   ros2 topic echo /joint_states
   ```

### Common Validation Commands

```bash
# Validate URDF
check_urdf your_robot.urdf

# Validate SDF
gz sdf -k your_robot.sdf

# Convert URDF to SDF and check
gz sdf -p your_robot.urdf

# Check for common URDF errors
urdf_to_graphiz your_robot.urdf
```

## Advanced Examples

### Robot with Sensors

Here's an example of a robot with a LiDAR sensor:

#### URDF with LiDAR (robot_with_lidar.urdf)
```xml
<?xml version="1.0"?>
<robot name="robot_with_lidar">
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

  <!-- LiDAR link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
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

  <!-- Joint to attach LiDAR -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugin for LiDAR -->
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
</robot>
```

## Best Practices for Model Creation

### URDF Best Practices
- Use consistent units (meters for length)
- Define proper inertial properties for all links
- Keep collision geometries simple for performance
- Use xacro for parameterized models
- Validate your URDF before simulation

### SDF Best Practices
- Use appropriate physics parameters
- Define proper material properties for visualization
- Organize complex models in separate files
- Use plugins for advanced functionality
- Validate SDF files before use

## Troubleshooting Model Loading

### Common Issues

1. **Model doesn't appear**: Check file paths and SDF syntax
2. **Physics issues**: Verify inertial properties
3. **Plugin errors**: Ensure plugins are installed and configured correctly
4. **Performance problems**: Simplify collision geometries

### Debugging Commands

```bash
# List loaded models
gz model -m

# Get model information
gz model -m your_model_name -i

# Check topics
gz topic -l
```

## Next Steps

Now that you understand how to create and load robot models:

1. Continue to [Physics Simulation](../physics-simulation/gravity-and-collisions.md) to learn about realistic physics
2. Explore [Sensor Simulation](../sensor-simulation/lidar-simulation.md) to add perception capabilities
3. Learn about [Unity Integration](../unity-integration/unity-setup.md) for high-fidelity visualization