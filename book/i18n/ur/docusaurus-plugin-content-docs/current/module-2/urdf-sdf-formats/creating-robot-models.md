---
sidebar_position: 3
---

# روبوٹ ماڈلز بنانا: عملی مثالیں

یہ ٹیوٹوریل آپ کو سیمولیشن کے لیے عملی روبوٹ ماڈلز بنانے میں رہنمائی فراہم کرے گا، سادہ پہیہ والے روبوٹس سے لے کر زیادہ پیچیدہ مربوط سسٹمز تک۔

## سادہ ڈفرینشل ڈرائیو روبوٹ

چلو ایک مکمل ڈفرینشل ڈرائیو روبوٹ ماڈل بنائیں جسے URDF اور SDF دونوں فارمیٹس میں استعمال کیا جا سکتا ہے۔

### URDF ورژن (differential_drive_robot.urdf)

```xml
<?xml version="1.0"?>
<robot name="differential_drive_robot">
  <!-- بیس لنک -->
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

  <!-- بائیں پہیہ -->
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

  <!-- دائیں پہیہ -->
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

  <!-- کاسٹر پہیہ -->
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

  <!-- جوائنٹس -->
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

### ROS 2 کنٹرول انٹیگریشن شامل کرنا

روبوٹ کو ROS 2 کے ساتھ کنٹرول کرنا ممکن بنانے کے لیے، ہمیں ٹرانسمیشن اور جوائنٹ اسٹیٹ پبلشر عناصر شامل کرنے کی ضرورت ہے:

```xml
<!-- جوائنٹس کے بعد URDF میں شامل کریں -->
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

<!-- ROS 2 کنٹرول کے لیے گیزبو پلگ ان -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot_description)/config/my_robot_control.yaml</parameters>
  </plugin>
</gazebo>
```

## آپ کے روبوٹ میں سینسرز شامل کرنا

چلو اپنے روبوٹ کو سینسرز کے ساتھ بہتر بنائیں:

### لیڈار سینسر کے ساتھ URDF

```xml
<!-- لیڈار کے لیے ماؤنٹنگ لنک شامل کریں -->
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

<!-- لیڈار ماؤنٹ کو بیس سے جوڑنے کا جوائنٹ -->
<joint name="lidar_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_mount"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>

<!-- لیڈار سینسر لنک -->
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

<!-- لیڈار کو ماؤنٹ سے جوڑنے کا جوائنٹ -->
<joint name="lidar_joint" type="fixed">
  <parent link="lidar_mount"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.025" rpy="0 0 0"/>
</joint>

<!-- لیڈار سینسر کے لیے گیزبو پلگ ان -->
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

## آپ کے روبوٹ کے ساتھ SDF ورلڈ بنانا

مکمل سیمولیشن ماحول میں اپنے روبوٹ کو استعمال کرنے کے لیے:

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

    <!-- URDF فائل سے اپنا روبوٹ شامل کریں -->
    <include>
      <uri>model://differential_drive_robot</uri>
      <pose>0 0 0.2 0 0 0</pose>
    </include>

    <!-- کچھ رکاوٹیں شامل کریں -->
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

## آپ کے روبوٹ ماڈل کی جانچ

1. اپنی URDF فائل کو `differential_drive_robot.urdf` کے نام سے محفوظ کریں
2. خامیوں کے لیے SDF میں تبدیل کریں:
   ```bash
   gz sdf -p differential_drive_robot.urdf
   ```
3. اپنے روبوٹ کے ساتھ گیزبو لانچ کریں:
   ```bash
   gz sim -r differential_drive_robot.sdf
   ```

## روبوٹ ماڈلنگ کے لیے بہترین طریقے

- **سادگی سے شروع کریں**: بنیادی شکلیں سے شروع کریں اور تدریج سے پیچیدگی شامل کریں
- **مناسب انرشل خصوصیات**: درست انرشل خصوصیات سیمولیشن کی عدم استحکامیوں کو روکتی ہیں
- **حقیقی ابعاد**: درست سیمولیشن کے لیے حقیقی دنیا کے پیمائش استعمال کریں
- **کولیژن بمقابلہ وژول**: کارکردگی کے لیے کولیژن جیومیٹریز کو سادہ رکھیں
- **فائلز کو منظم کریں**: پیچیدہ روبوٹس کے لیے الگ فائلز استعمال کریں اور انہیں xacro کے ذریعے شامل کریں
- **توثیق کریں**: پیچیدہ استعمال سے پہلے ہمیشہ اپنے ماڈلز کو سیمولیشن میں ٹیسٹ کریں

## اگلے اقدامات

مختلف استعمال کے معاملات کے لیے بہترین طریقوں کو سمجھنے کے لیے [URDF سے SDF تبدیلی](./conversion-guide.md) کے بارے میں سیکھنے کے لیے جاری رکھیں۔