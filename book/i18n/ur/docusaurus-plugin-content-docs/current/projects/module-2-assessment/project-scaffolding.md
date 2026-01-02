---
sidebar_position: 1
---

# ماڈیول 2 اسیسمنٹ پروجیکٹ سکیفولڈنگ (Scaffolding)

یہ دستاویز ماڈیول 2 اسیسمنٹ پروجیکٹ کے لیے مکمل سکیفولڈنگ اور ڈھانچہ فراہم کرتی ہے۔ اپنے پروجیکٹ کے ماحول کو ترتیب دینے اور متوقع ڈیلیوریبلز کو سمجھنے کے لیے اس گائیڈ پر عمل کریں۔

## پروجیکٹ کا جائزہ

ماڈیول 2 اسیسمنٹ پروجیکٹ کے لیے آپ کو ایک مکمل سمولیشن ماحول بنانے کی ضرورت ہے جو Gazebo سمولیشن، روبوٹ ماڈلنگ، فزکس سمولیشن، سینسر انضمام، اور Unity وژولائزیشن کی آپ کی سمجھ کو ظاہر کرے۔

### پروجیکٹ کے اہداف

1. مناسب URDF/SDF تفصیل کے ساتھ ایک روبوٹ ماڈل بنائیں
2. فزکس کی خصوصیات کے ساتھ Gazebo سمولیشن ماحول نافذ کریں
3. اپنے روبوٹ پر متعدد سینسر کی اقسام ضم کریں
4. نقلی ماحول میں بنیادی نیویگیشن کا مظاہرہ کریں
5. (اختیاری) ہائی فیڈیلیٹی رینڈرنگ کے لیے Unity وژولائزیشن نافذ کریں

### پروجیکٹ ٹائم لائن

- **ہفتہ 1**: روبوٹ ماڈل ڈیزائن اور URDF تخلیق
- **ہفتہ 2**: Gazebo ماحول سیٹ اپ اور فزکس کنفیگریشن
- **ہفتہ 3**: سینسر انضمام اور توثیق
- **ہفتہ 4**: نیویگیشن کا نفاذ اور جانچ
- **ہفتہ 5**: Unity انضمام (اختیاری) اور حتمی جانچ

## پروجیکٹ کا ڈھانچہ

اپنے پروجیکٹ کے لیے درج ذیل ڈائرکٹری ڈھانچہ بنائیں:

```
module-2-assessment/
├── robot_description/
│   ├── urdf/
│   │   ├── robot_model.urdf
│   │   └── robot_model.xacro
│   ├── meshes/
│   │   ├── base_link.dae
│   │   ├── wheel.dae
│   │   └── sensor_mount.dae
│   ├── launch/
│   │   ├── spawn_robot.launch.py
│   │   └── simulation.launch.py
│   └── config/
│       ├── robot_control.yaml
│       └── sensors.yaml
├── gazebo_worlds/
│   ├── worlds/
│   │   ├── simple_room.world
│   │   ├── maze_world.world
│   │   └── navigation_course.world
│   ├── models/
│   │   ├── custom_obstacle/
│   │   │   ├── model.sdf
│   │   │   └── model.config
│   │   └── target_marker/
│   │       ├── model.sdf
│   │       └── model.config
│   └── launch/
│       └── world.launch.py
├── sensors/
│   ├── lidar/
│   │   ├── lidar_config.yaml
│   │   └── lidar_validation.py
│   ├── camera/
│   │   ├── camera_config.yaml
│   │   └── image_processor.py
│   └── imu/
│       ├── imu_config.yaml
│       └── orientation_estimator.py
├── navigation/
│   ├── config/
│   │   ├── costmap_common_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── global_costmap_params.yaml
│   ├── launch/
│   │   └── navigation.launch.py
│   └── scripts/
│       ├── simple_navigation.py
│       └── obstacle_avoidance.py
├── unity_visualization/  # Optional
│   ├── unity_project/
│   │   ├── Assets/
│   │   ├── ProjectSettings/
│   │   └── Packages/
│   └── ros2_unity_bridge/
│       ├── unity_ros2_bridge.unitypackage
│       └── bridge_config.json
├── tests/
│   ├── unit_tests/
│   │   ├── test_robot_model.py
│   │   ├── test_sensors.py
│   │   └── test_navigation.py
│   ├── integration_tests/
│   │   ├── test_simulation.py
│   │   └── test_ros2_integration.py
│   └── validation_scripts/
│       ├── validate_urdf.py
│       ├── validate_sensors.py
│       └── performance_test.py
├── docs/
│   ├── project_plan.md
│   ├── implementation_notes.md
│   └── final_report.md
└── CMakeLists.txt
```

## ابتدائی سیٹ اپ کے مراحل

### 1. پروجیکٹ ڈائرکٹری بنائیں

```bash
mkdir -p module-2-assessment/{robot_description,robot_description/{urdf,meshes,launch,config},gazebo_worlds,gazebo_worlds/{worlds,models,launch},sensors,sensors/{lidar,camera,imu},navigation,navigation/{config,launch,scripts},tests,tests/{unit_tests,integration_tests,validation_scripts},docs}
```

### 2. بنیادی پیکیج فائلیں بنائیں

#### package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>module_2_assessment</name>
  <version>1.0.0</version>
  <description>Module 2 Assessment Project - Complete Simulation Environment</description>
  <maintainer email="student@university.edu">Student Name</maintainer>
  <license>Apache-2.0</license>

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

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(module_2_assessment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/simple_navigation.py
  scripts/obstacle_avoidance.py
  scripts/lidar_validation.py
  scripts/image_processor.py
  scripts/orientation_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 3. بنیادی روبوٹ URDF بنائیں

#### robot_description/urdf/robot_model.urdf
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

### 4. بنیادی دنیا (World) فائل بنائیں

#### gazebo_worlds/worlds/navigation_course.world
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

    <!-- Simple maze structure -->
    <model name="wall_1">
      <pose>0 3 0.5 0 0 0</pose>
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
              <size>6 0.1 1</size>
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
              <size>6 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="wall_2">
      <pose>3 0 0.5 0 0 1.5708</pose>
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
              <size>4 0.1 1</size>
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
              <size>4 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="wall_3">
      <pose>-3 0 0.5 0 0 1.5708</pose>
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
          </inertial>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>4 0.1 1</size>
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
              <size>4 0.1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Target location -->
    <model name="target">
      <pose>0 -2 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
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
              <radius>0.2</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### 5. بنیادی لانچ فائلیں بنائیں

#### robot_description/launch/spawn_robot.launch.py
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')
    default_model_path = pkg_share + '/urdf/robot_model.urdf'

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

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
```

#### gazebo_worlds/launch/world.launch.py
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('module_2_assessment').find('module_2_assessment')
    world_file = pkg_share + '/worlds/navigation_course.world'

    # Launch Gazebo with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
```

## پروجیکٹ کے نفاذ کے مراحل

### مرحلہ 1: روبوٹ ماڈل ڈویلپمنٹ (ہفتہ 1)

1. **اپنا روبوٹ ڈیزائن کریں**:
   - جسمانی طول و عرض اور ماس کی خصوصیات کی وضاحت کریں۔
   - مناسب جڑواں خصوصیات (inertial properties) کے ساتھ URDF بنائیں۔
   - بصری اور ٹکراؤ جیومیٹریز شامل کریں۔

2. **اپنے روبوٹ ماڈل کی توثیق کریں**:
   ```bash
   # Check URDF syntax
   check_urdf robot_description/urdf/robot_model.urdf

   # Convert to SDF and verify
   gz sdf -p robot_description/urdf/robot_model.urdf
   ```

3. **Gazebo میں ٹیسٹ کریں**:
   ```bash
   # Launch Gazebo with your robot
   gz sim -r robot_description/urdf/robot_model.urdf
   ```

### مرحلہ 2: ماحولیاتی سیٹ اپ (ہفتہ 2)

1. **اپنی دنیا (world) بنائیں**:
   - رکاوٹوں کے ساتھ ایک نیویگیشن کورس ڈیزائن کریں۔
   - اہداف اور نشانات شامل کریں۔
   - فزکس کی خصوصیات کو کنفیگر کریں۔

2. **فزکس سمولیشن کی جانچ کریں**:
   - کشش ثقل اور ٹکراؤ کا پتہ لگانے کی تصدیق کریں۔
   - روبوٹ کے استحکام اور حرکت کی جانچ کریں۔
   - رابطہ قوتوں (contact forces) کی توثیق کریں۔

### مرحلہ 3: سینسر انضمام (ہفتہ 3)

1. **سینسرز ضم کریں**:
   - اپنے روبوٹ میں LiDAR، کیمرہ، اور IMU شامل کریں۔
   - سینسر کے پیرامیٹرز کو مناسب طریقے سے کنفیگر کریں۔
   - سینسر ڈیٹا کے معیار کی توثیق کریں۔

2. **سینسر کی فعالیت کی جانچ کریں**:
   ```bash
   # Check if sensor topics are publishing
   ros2 topic list | grep -E "(scan|image|imu)"

   # Monitor sensor data
   ros2 topic echo /robot/scan
   ros2 topic echo /robot/imu
   ```

### مرحلہ 4: نیویگیشن نفاذ (ہفتہ 4)

1. **نیویگیشن اسٹیک نافذ کریں**:
   - کوسٹ میپ کنفیگریشن سیٹ کریں۔
   - پاتھ پلاننگ نافذ کریں۔
   - رکاوٹوں سے بچنا (obstacle avoidance) شامل کریں۔

2. **نیویگیشن کی جانچ کریں**:
   - شروع سے ہدف تک نیویگیٹ کریں۔
   - کامیابی سے رکاوٹوں سے بچیں۔
   - متحرک حالات کو ہینڈل کریں۔

### مرحلہ 5: Unity انضمام (ہفتہ 5، اختیاری)

1. **Unity پروجیکٹ سیٹ اپ کریں**:
   - ROS 2 Unity برج انسٹال کریں۔
   - نیٹ ورک مواصلات کو کنفیگر کریں۔
   - روبوٹ ماڈلز درآمد کریں۔

2. **Unity وژولائزیشن کی جانچ کریں**:
   - ریئل ٹائم ہم وقت سازی (synchronization) کی تصدیق کریں۔
   - وژولائزیشن کے معیار کی جانچ کریں۔
   - کارکردگی کی توثیق کریں۔

## ٹیسٹنگ اور توثیق

### یونٹ ٹیسٹ
- روبوٹ ماڈل کی توثیق
- سینسر ڈیٹا کی توثیق
- نیویگیشن الگورتھم کی جانچ

### انٹیگریشن ٹیسٹ
- مکمل سمولیشن پائپ لائن
- ROS 2 مواصلات
- کراس پلیٹ فارم مطابقت

### کارکردگی کے ٹیسٹ
- سمولیشن ریئل ٹائم فیکٹر
- سینسر اپ ڈیٹ ریٹس
- Unity فریم ریٹس

## دستاویزات کی ضروریات

اپنے پروجیکٹ کو دستاویز کریں:
- ہر جزو کے لیے نفاذ کے نوٹس
- کنفیگریشن پیرامیٹرز اور دلیل (rationale)
- ٹیسٹنگ کے نتائج اور توثیقی ڈیٹا
- کارکردگی میٹرکس اور اصلاحی نوٹس

## اگلے اقدامات

پروجیکٹ سکیفولڈنگ ترتیب دینے کے بعد:

1. تفصیلی نفاذ کی رہنمائی کے لیے [مرحلہ وار ہدایات](./step-by-step-instructions.md) پر جاری رکھیں
2. حوالہ جاتی نفاذ کے لیے [حل کی مثالیں](./solution-examples.md) کا جائزہ لیں
3. اپنے مکمل پروجیکٹ کو [تشخیص کے معیار](../../module-2/assessment-project/evaluation-criteria.md) کے خلاف ٹیسٹ کریں
