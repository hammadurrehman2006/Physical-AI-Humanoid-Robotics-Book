---
sidebar_position: 4
---

# گیزبو میں بنیادی روبوٹ اسپاوننگ

یہ ٹیوٹوریل آپ کو گیزبو میں اپنا پہلا روبوٹ ماڈل اسپاون کرنے اور تصدیق کرنے کے طریقے سے آگاہ کرے گا کہ یہ سیمولیشن ماحول میں درست طریقے سے کام کر رہا ہے۔

## شرائط لازمہ

اس ٹیوٹوریل کا آغاز کرنے سے پہلے، یقینی بنائیں کہ آپ کے پاس ہے:
- کامیابی کے ساتھ گیزبو گارڈن انسٹال ہے
- [گیزبو ماحول سیٹ اپ](./setup-gazebo-environment.md) مکمل ہو چکا ہے
- یوآرڈی ایف/ایس ڈی ایف فارمیٹس کی بنیادی سمجھ

## طریقہ 1: گیزبو کے بلٹ ان ماڈلز کا استعمال

### پری بِلٹ ماڈل کے ساتھ لانچ کرنا

گیزبو کے ساتھ کئی بلٹ ان ماڈلز ہیں جن کا آپ ٹیسٹنگ کے لیے استعمال کر سکتے ہیں:

```bash
# ایک سادہ ماڈل کے ساتھ گیزبو لانچ کریں
gz sim -r -v 1

# یا ایک مخصوص ورلڈ فائل کے ساتھ لانچ کریں
gz sim -r empty.sdf
```

### ڈیٹا بیس سے ماڈلز شامل کرنا

آپ گیزبو کے آن لائن ماڈل ڈیٹا بیس سے ماڈلز شامل کر سکتے ہیں:

```bash
# دستیاب ماڈلز کی فہرست
gz model --list

# کمانڈ لائن کے ذریعے ایک ماڈل شامل کریں (یہ براہ راست کام نہیں کرے گا، لیکن تصور دکھاتا ہے)
# گیزبو جی یو آئی میں، آپ انسرٹ ٹیب سے ماڈلز براؤز اور داخل کر سکتے ہیں
```

## طریقہ 2: اپنا روبوٹ تخلیق کرنا اور اسپاون کرنا

### اقدام 1: ایک سادہ یوآرڈی ایف روبوٹ بنائیں

ایک فائل بنائیں جس کا نام ہے `simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- بیس لنک -->
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

### اقدام 2: یوآرڈی ایف کو ایس ڈی ایف میں تبدیل کریں

```bash
# گیزبو کے لیے یوآرڈی ایف کو ایس ڈی ایف فارمیٹ میں تبدیل کریں
gz sdf -p simple_robot.urdf > simple_robot.sdf
```

### اقدام 3: ایک ورلڈ فائل بنائیں

ایک ورلڈ فائل بنائیں جس کا نام ہے `simple_robot_world.sdf`:

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

    <!-- آپ کا کسٹم روبوٹ -->
    <model name="simple_robot">
      <include>
        <uri>model://simple_robot</uri>
        <pose>0 0 0.1 0 0 0</pose>
      </include>
    </model>
  </world>
</sdf>
```

### اقدام 4: ورلڈ لانچ کریں

```bash
# روبوٹ کے ساتھ اپنا کسٹم ورلڈ لانچ کریں
gz sim -r simple_robot_world.sdf
```

## طریقہ 3: پروگرامیٹک اسپاوننگ ROS 2 کے ساتھ

### ضروری پیکیجز انسٹال کرنا

```bash
# ROS 2 گیزبو پیکیجز انسٹال کریں
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```

### اسپاوننگ کے لیے ros_gz_sim کا استعمال

ایک لانچ فائل بنائیں `spawn_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # پیکیج اور فائل کے راستے
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    urdf_file = pkg_share + '/urdf/simple_robot.urdf'

    # روبوٹ اسٹیٹ پبلشر نوڈ
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    # گیزبو لانچ
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

## تصدیق کے اقدامات

اپنا روبوٹ اسپاون کرنے کے بعد، یقین کریں کہ یہ درست طریقے سے کام کر رہا ہے:

### 1. وژول تصدیق
- چیک کریں کہ روبوٹ گیزبو ونڈو میں ظاہر ہوتا ہے
- یقین کریں کہ روبوٹ کی شکل، سائز، اور رنگ آپ کے یوآرڈی ایف/ایس ڈی ایف سے مماثل ہے
- یقینی بنائیں کہ روبوٹ دنیا میں صحیح طریقے سے پوزیشن کیا گیا ہے

### 2. فزکس تصدیق
- مشاہدہ کریں کہ آیا روبوٹ گریویٹی کا جواب دیتا ہے (زمین پر آرام کرنا چاہیے)
- کوئی غیر متوقع حرکات یا عدم استحکام کے لیے چیک کریں
- یقین کریں کہ کولیژن خصوصیات کام کر رہی ہیں (روبوٹ زمین کے ذریعے نہیں گرنا چاہیے)

### 3. سینسر تصدیق (اگر قابل اطلاق ہو)
اگر آپ کے روبوٹ میں سینسرز ہیں:

```bash
# دستیاب ٹاپکس کی فہرست
ros2 topic list | grep -i scan  # لیڈار کے لیے
ros2 topic list | grep -i camera  # کیمرے کے لیے
ros2 topic list | grep -i imu  # آئی ایم یو کے لیے

# سینسر ڈیٹا چیک کریں
ros2 topic echo /your_robot/laser_scan
```

## عام مسائل کا حل

### روبوٹ زمین کے ذریعے گر جاتا ہے
- اپنے یوآرڈی ایف/ایس ڈی ایف میں انرشل خصوصیات کی تصدیق کریں
- چیک کریں کہ کولیژن جیومیٹریز مناسب طریقے سے وضاحت کی گئی ہیں
- یقینی بنائیں کہ ماس ویلیوز مثبت اور مناسب ہیں

### روبوٹ غلط ابعاد کے ساتھ ظاہر ہوتا ہے
- اپنے یوآرڈی ایف میں یونٹس کی دوبارہ چیک کریں (گیزبو کے لیے میٹر)
- یقین کریں کہ وژول اور کولیژن جیومیٹریز امکانات سے مماثل ہیں

### روبوٹ غیر مستحکم یا جِٹری ہے
- اپنی ورلڈ فائل فزکس سیٹنگز میں `max_step_size` کم کریں
- مناسب انرشل خصوصیات اور ماس ویلیوز کا یقین کریں
- چیک کریں کہ ماس کا مرکز صحیح طریقے سے پوزیشن کیا گیا ہے

## ایڈوانسڈ اسپاوننگ ٹیکنیکس

### متعدد روبوٹس اسپاون کرنا

مختلف ناموں اور پوزیشنز کے ساتھ متعدد روبوٹس اسپاون کرنے کے لیے:

```bash
# مختلف نیمسپیسز اور پوزیشنز کا استعمال کریں
ros2 run gazebo_ros spawn_entity.py -entity robot1 -file robot.urdf -x 0 -y 0 -z 0.1
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file robot.urdf -x 2 -y 2 -z 0.1
```

### مختلف کنفیگریشنز کے ساتھ اسپاون کرنا

آپ ایک ہی یوآرڈی ایف کو مختلف پیرامیٹرز کے ساتھ استعمال کر سکتے ہیں xacro کا استعمال کر کے:

```xml
<!-- robot_with_params.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="param_robot">
  <xacro:arg name="robot_name" default="my_robot"/>
  <xacro:arg name="wheel_radius" default="0.1"/>

  <link name="$(arg robot_name)_base_link">
    <!-- پیرامیٹرز کا استعمال کرتے ہوئے روبوٹ کی تعریف -->
  </link>
</robot>
```

## اگلے اقدامات

جب آپ کامیابی کے ساتھ اپنا پہلا روبوٹ اسپاون کر لیں:

1. مزید پیچیدہ روبوٹس تخلیق کرنے کے لیے [یوآرڈی ایف اور ایس ڈی ایف فارمیٹس](../urdf-sdf-formats/urdf-basics.md) کے بارے میں سیکھنا جاری رکھیں
2. حقیقی رویے کو شامل کرنے کے لیے [فزکس سیمولیشن](../physics-simulation/gravity-and-collisions.md) کے بارے میں سیکھیں
3. اپنے روبوٹس میں ادراک کی صلاحیتوں کو شامل کرنے کے لیے [سینسر سیمولیشن](../sensor-simulation/lidar-simulation.md) کا استعما ل کریں