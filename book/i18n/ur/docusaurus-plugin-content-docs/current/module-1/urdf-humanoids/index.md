---
title: ہیومنوائڈز کے لیے URDF
description: ہیومنوائڈ روبوٹ ماڈلنگ کے لیے یونیفائیڈ روبوٹ ڈیسکرپشن فارمیٹ کو سمجھنا
sidebar_position: 8
---

# ہیومنوائڈز کے لیے URDF

## سیکھنے کے مقاصد
- URDF (یونیفائیڈ روبوٹ ڈیسکرپشن فارمیٹ) کی ساخت اور اجزاء کو سمجھیں
- ہیومنوائڈ روبوٹس کے لیے URDF ماڈل بنانا سیکھیں
- پیچیدہ ہیومنوائڈ وضاحتوں کے لیے xacro کے استعمال میں مہارت حاصل کریں
- کائینیٹک چینز اور جوائنٹ کی پابندیوں کو نافذ کریں
- ہیومنوائڈ روبوٹ ماڈلز کی توثیق اور وژولائز کریں

## URDF کا تعارف

URDF (Unified Robot Description Format) ایک XML پر مبنی فارمیٹ ہے جو ROS میں روبوٹ ماڈلز کو بیان کرنے کے لیے استعمال ہوتا ہے۔ یہ روبوٹ کی جسمانی اور بصری خصوصیات کی وضاحت کرتا ہے، بشمول لنکس، جوائنٹس، جڑواں خصوصیات (inertial properties)، اور بصری عناصر۔ ہیومنوائڈ روبوٹس کے لیے، URDF تخروپن (simulation)، وژولائزیشن، اور کائینیٹک تجزیہ کے لیے ضروری ہے۔

### URDF اجزاء کا جائزہ

ایک URDF ماڈل مندرجہ ذیل پر مشتمل ہوتا ہے:

1. **Links**: سخت اجسام جو روبوٹ کی ساخت بناتے ہیں
2. **Joints**: لنکس کے درمیان رابطے جو حرکت کی وضاحت کرتے ہیں
3. **Visual**: روبوٹ تخروپن/وژولائزیشن میں کیسا نظر آتا ہے
4. **Collision**: روبوٹ ماحول کے ساتھ کیسے تعامل کرتا ہے
5. **Inertial**: حرکیاتی تخروپن کے لیے طبعی خصوصیات
6. **Materials**: بصری ظاہری خصوصیات

## بنیادی URDF ڈھانچہ

### سادہ لنک کی تعریف

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

### جوائنٹ کی تعریف

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

## ہیومنوائڈ روبوٹ URDF مثال

### مکمل ہیومنوائڈ سکیلیٹن URDF

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