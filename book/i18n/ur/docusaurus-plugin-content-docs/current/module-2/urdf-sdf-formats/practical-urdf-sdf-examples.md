---
sidebar_position: 5
---

# عملی URDF/SDF مثالیں اور Gazebo میں لوڈنگ

یہ ٹیوٹوریل URDF اور SDF دونوں فارمیٹس کی عملی مثالیں فراہم کرتا ہے اور یہ ظاہر کرتا ہے کہ انہیں حقیقی دنیا کے منظرناموں کے ساتھ Gazebo میں کیسے لوڈ کیا جائے۔

## مکمل ڈیفرینشل ڈرائیو روبوٹ کی مثال

آئیے ایک مکمل ڈیفرینشل ڈرائیو روبوٹ بنائیں جسے Gazebo میں لوڈ کیا جا سکے، جو دونوں فارمیٹس کا مظاہرہ کرے۔

### URDF ورژن (differential_drive_robot.urdf)

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

### SDF ورژن (differential_drive_robot.sdf)

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

## Gazebo میں مثالیں لوڈ کرنا

### طریقہ 1: براہ راست SDF لوڈنگ

```bash
# Load the SDF file directly
gz sim -r differential_drive_robot.sdf

# Or with a world file that includes the robot
gz sim -r robot_world.sdf
```

### طریقہ 2: URDF سے SDF کی تبدیلی اور لوڈنگ

```bash
# Convert URDF to SDF
gz sdf -p differential_drive_robot.urdf > robot_as_sdf.sdf

# Load the converted SDF
gz sim -r robot_as_sdf.sdf
```

### طریقہ 3: Gazebo ماڈل ڈیٹا بیس کا استعمال

1. ایک ماڈل ڈائرکٹری بنائیں:
```bash
mkdir -p ~/.gazebo/models/differential_drive_robot
```

2. model.config فائل بنائیں:
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

3. اپنی SDF فائل کو ڈائرکٹری میں `model.sdf` کے طور پر رکھیں

4. Gazebo لانچ کریں اور اپنے روبوٹ کو شامل کرنے کے لیے Insert ٹیب کا استعمال کریں

## روبوٹ ماڈلز کی جانچ

### تصدیق کے مراحل

1. **ماڈل لوڈ کریں**:
   ```bash
   gz sim -r your_robot.sdf
   ```

2. **ٹرمینل آؤٹ پٹ میں غلطیوں کی جانچ کریں**

3. **طبیعیات کی تصدیق کریں**:
   - کیا روبوٹ زمین پر صحیح طریقے سے ٹکا ہوا ہے؟
   - کیا پہیے حکم ملنے پر گھومتے ہیں؟
   - کیا ٹکراؤ کا پتہ صحیح طریقے سے لگایا جا رہا ہے؟

4. **ROS 2 انضمام کی جانچ کریں** (اگر لاگو ہو):
   ```bash
   # List topics
   ros2 topic list | grep robot

   # Check joint states
   ros2 topic echo /joint_states
   ```

### عام توثیقی کمانڈز

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

## اعلی درجے کی مثالیں

### سینسرز کے ساتھ روبوٹ

یہاں LiDAR سینسر کے ساتھ روبوٹ کی ایک مثال ہے:

#### LiDAR کے ساتھ URDF (robot_with_lidar.urdf)
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

## ماڈل بنانے کے بہترین طریقے

### URDF بہترین طریقے
- مستقل اکائیاں استعمال کریں (لمبائی کے لیے میٹر)
- تمام لنکس کے لیے مناسب جڑواں خصوصیات (inertial properties) کی وضاحت کریں
- کارکردگی کے لیے ٹکراؤ کی جیومیٹریز (collision geometries) کو آسان رکھیں
- پیرامیٹرائزڈ ماڈلز کے لیے xacro کا استعمال کریں
- سمولیشن سے پہلے اپنے URDF کی توثیق کریں

### SDF بہترین طریقے
- مناسب طبیعیات کے پیرامیٹرز استعمال کریں
- وژولائزیشن کے لیے مناسب میٹریل خصوصیات کی وضاحت کریں
- پیچیدہ ماڈلز کو الگ الگ فائلوں میں منظم کریں
- جدید فعالیت کے لیے پلگ ان استعمال کریں
- استعمال سے پہلے SDF فائلوں کی توثیق کریں

## ماڈل لوڈنگ کے مسائل کا حل (Troubleshooting)

### عام مسائل

1. **ماڈل ظاہر نہیں ہوتا**: فائل پاتھ اور SDF سنٹیکس چیک کریں
2. **طبیعیات کے مسائل**: جڑواں خصوصیات کی تصدیق کریں
3. **پلگ ان کی غلطیاں**: یقینی بنائیں کہ پلگ ان انسٹال ہیں اور صحیح طریقے سے کنفیگر ہیں
4. **کارکردگی کے مسائل**: ٹکراؤ کی جیومیٹریز کو آسان بنائیں

### ڈیبگنگ کمانڈز

```bash
# List loaded models
gz model -m

# Get model information
gz model -m your_model_name -i

# Check topics
gz topic -l
```

## اگلے اقدامات

اب جب کہ آپ سمجھ گئے ہیں کہ روبوٹ ماڈل کیسے بنائیں اور لوڈ کریں:

1. حقیقت پسندانہ طبیعیات کے بارے میں جاننے کے لیے [طبیعیات سمولیشن](../physics-simulation/gravity-and-collisions.md) پر جاری رکھیں
2. ادراک کی صلاحیتیں شامل کرنے کے لیے [سینسر سمولیشن](../sensor-simulation/lidar-simulation.md) دریافت کریں
3. ہائی فیڈیلیٹی وژولائزیشن کے لیے [Unity انضمام](../unity-integration/unity-setup.md) کے بارے میں جانیں
