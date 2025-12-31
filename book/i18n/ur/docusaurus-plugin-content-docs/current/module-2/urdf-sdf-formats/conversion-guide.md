---
sidebar_position: 4
---

# تبدیلی گائیڈ: URDF سے SDF اور واپس

URDF اور SDF فارمیٹس کے درمیان تبدیل کرنا کب اور کیسے کرنا ہے، یہ سمجھنا روبوٹکس سیمولیشن کے لیے انتہائی اہم ہے۔ یہ گائیڈ تبدیلی کے عمل اور بہترین طریقے کو دیکھتا ہے۔

## ہر فارمیٹ کو کب استعمال کرنا ہے

### URDF کا استعمال کریں جب:
- ROS ایکو سسٹم کے اندر کام کر رہے ہوں
- روبوٹ کنیمیٹکس اور بنیادی تفصیلات پر توجہ مرکوز کر رہے ہوں
- ماڈلز بناتے ہوں جن کو ROS 2 انٹیگریشن کی ضرورت ہو
- دوبارہ استعمال کے قابل روبوٹ اجزاء بناتے ہوں
- پیچیدہ ماڈلز کے لیے xacro میکروز کا فائدہ اٹھاتے ہوں

### SDF کا استعمال کریں جب:
- گیزبو کی خصوصی خصوصیات استعمال کر رہے ہوں (سینسرز، پلگ انز)
- مکمل سیمولیشن دنیا تخلیق کر رہے ہوں
- ایڈوانس فزکس خصوصیات کی ضرورت ہو
- غیر ROS سیمولیشن منظرناموں پر کام کر رہے ہوں
- متعدد ماڈلز کے ساتھ دنیا کی تفصیل کی ضرورت ہو

## URDF سے SDF کی تبدیلی

### کمانڈ لائن تبدیلی

URDF سے SDF میں تبدیل کرنے کا سب سے آسان طریقہ:

```bash
# بنیادی تبدیلی
gz sdf -p robot.urdf > robot.sdf

# xacro پری پروسیسنگ کے ساتھ (اگر میکروز استعمال کر رہے ہوں)
xacro robot.urdf.xacro | gz sdf -p /dev/stdin > robot.sdf

# تبدیلی سے پہلے URDF کی تصدیق کریں
check_urdf robot.urdf
```

### مثال تبدیلی کا عمل

**ان پٹ URDF (robot.urdf):**
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

**تبدیل شدہ SDF:**
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

## URDF میں گیزبو کی خصوصی عناصر شامل کرنا

جب آپ کو URDF میں گیزبو کی خصوصیات کی ضرورت ہو، گیزبو کی خصوصی ٹیگز شامل کریں:

```xml
<!-- اپنی URDF فائل کے آخر میں شامل کریں -->

<!-- ROS 2 کنٹرول کے لیے گیزبو پلگ ان -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
  </plugin>
</gazebo>

<!-- جوائنٹ اسٹیٹ پبلشر کے لیے گیزبو پلگ ان -->
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

<!-- URDF لنک میں سینسر -->
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

<!-- میٹریل کی تعریفات -->
<gazebo reference="base_link_visual">
  <material>Gazebo/Blue</material>
</gazebo>
```

## SDF سے URDF کی تبدیلی

جبکہ SDF سے URDF تبدیلی کو ٹولز کے ذریعے براہ راست سپورٹ نہیں دی جاتی، آپ میپنگ کو سمجھ کر SDF سے مساوی URDF تخلیق کر سکتے ہیں:

| SDF عنصر | URDF مساوی |
|-------------|-----------------|
| `<model>` | `<robot>` |
| `<link>` | `<link>` |
| `<joint>` | `<joint>` |
| `<visual>` | `<visual>` |
| `<collision>` | `<collision>` |
| `<inertial>` | `<inertial>` |
| `<sensor>` | `<link>` + گیزبو پلگ ان |

## ایڈوانسڈ تبدیلی کے منظرنامے

### پیچیدہ تبدیلیوں کے لیے xacro کا استعمال

پیچیدہ روبوٹس کے لیے، پیرامیٹرائز URDFs تخلیق کرنے کے لیے xacro استعمال کریں جنہیں SDF میں تبدیل کیا جا سکتا ہے:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="param_robot">

  <!-- پیرامیٹر تعریف کریں -->
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.5"/>
  <xacro:property name="base_height" value="0.2"/>

  <!-- دیگر xacro فائلز شامل کریں -->
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

  <!-- پہیے تخلیق کرنے کے لیے میکرو استعمال کریں -->
  <xacro:wheel prefix="left" parent="base_link" x="0" y="0.3" z="0"/>
  <xacro:wheel prefix="right" parent="base_link" x="0" y="-0.3" z="0"/>

</robot>
```

### گیزبو دنیا کے ساتھ URDF استعمال کرنا

مکمل گیزبو دنیا میں URDF روبوٹ کو استعمال کرنے کے لیے، آپ کے کچھ اختیارات ہیں:

1. **ROS سروسز کا استعمال کر کے ماڈل کو پروگرامی طور پر اسپاون کریں**
2. **SDF میں تبدیل کریں اور دنیا کی فائل میں ایمبیڈ کریں**
3. **اپنی URDF کو ماڈلز ڈائریکٹری میں شامل کر کے گیزبو کے ماڈل ڈیٹا بیس کا استعما ل کریں**

## ٹولز اور توثیق

### URDF کی توثیق
```bash
# URDF سینٹیکس اور سٹرکچر چیک کریں
check_urdf robot.urdf

# URDF کو وژولائز کریں
urdf_to_graphiz robot.urdf
```

### SDF کی توثیق
```bash
# SDF سینٹیکس کی تصدیق کریں
gz sdf -k robot.sdf

# تبدیل کریں اور SDF چیک کریں
gz sdf -p robot.urdf
```

### ROS 2 انٹیگریشن
```bash
# ros2_control کے ساتھ ٹیسٹ کریں
ros2 launch my_robot_description spawn.launch.py

# جوائنٹ اسٹیٹس چیک کریں
ros2 topic echo /joint_states
```

## عام تبدیلی کے مسائل اور حل

### مسئلہ 1: انرشل خصوصیات غائب ہیں
**مسئلہ**: سیمولیشن میں روبوٹ زمین کے ذریعے گر جاتا ہے
**حل**: یقینی بنائیں کہ تمام لنکس کے پاس مناسب انرشل خصوصیات ہیں

### مسئلہ 2: سینسر کام نہیں کر رہا
**مسئلہ**: گیزبو پلگ انز کام نہیں کر رہے
**حل**: یقینی بنائیں کہ پلگ ان نام اور ROS انٹرفیسز درست ہیں

### مسئلہ 3: جوائنٹ کی حدود کا احترام نہیں کیا جا رہا
**مسئلہ**: جوائنٹس جسمانی حدود سے باہر حرکت کر رہے ہیں
**حل**: URDF اور SDF دونوں فارمیٹس میں مناسب حدود شامل کریں

### مسئلہ 4: میٹریل کے رنگ مختلف ہیں
**مسئلہ**: رنگ امید کے مطابق نہیں دکھائی دے رہے
**حل**: دونوں فارمیٹس کے درمیان مسلسل میٹریل کی تعریفات استعمال کریں

## بہترین طریقے

1. **اپنی فارمیٹ حکمت عملی منصوبہ بندی کریں**: اپنے بنیادی ورک فلو کی بنیاد پر فیصلہ کریں کہ کون سا فارمیٹ استعمال کرنا ہے
2. **تبدیلی کے لیے ٹولز کا استعما ل کریں**: پیچیدہ تبدیلیوں کے لیے `gz sdf` اور `xacro` کا فائدہ اٹھائیں
3. **تبدیلی کے بعد توثیق کریں**: ہمیشہ سیمولیشن میں تبدیل شدہ ماڈلز کو ٹیسٹ کریں
4. **فارمیٹس کو مطابقت رکھیں**: اگر دونوں کی ضرورت ہو تو دونوں فارمیٹس کو مطابق رکھیں
5. **تبدیلی کے اقدامات کو دستاویز کریں**: دوبارہ استعمال کے قابل بنانے کے لیے اپنے تبدیلی کے عمل کو ریکارڈ کریں

## اگلے اقدامات

اب جب آپ URDF اور SDF فارمیٹس کو سمجھ چکے ہیں، [فزکس سیمولیشن](../physics-simulation/gravity-and-collisions.md) کے بارے میں سیکھنے کے لیے جاری رکھیں تاکہ اپنے روبوٹ ماڈلز کو حقیقی فزکس کے ساتھ زندگی بخشا جا سکے۔