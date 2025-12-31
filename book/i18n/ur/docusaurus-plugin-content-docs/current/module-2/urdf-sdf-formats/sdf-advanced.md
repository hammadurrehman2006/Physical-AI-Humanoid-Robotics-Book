---
sidebar_position: 2
---

# ایس ایس ڈی ایف ایڈوانس: سیمولیشن ڈسکرپشن فارمیٹ

ایس ایس ڈی ایف (سیمولیشن ڈسکرپشن فارمیٹ) گیزبو کا مقامی ایکس ایم ایل فارمیٹ ہے جو سیمولیشن ماحول کی تفصیل کے لیے استعمال ہوتا ہے، بشمول روبوٹس، اشیاء، اور دنیا کی خصوصیات۔ جبکہ یوآرڈی ایف صرف ROS کے لیے ہے، ایس ایس ڈی ایف کو فزکس سیمولیشن کے لیے خاص طور پر تیار کیا گیا ہے۔

## ایس ایس ڈی ایف بمقابلہ یوآرڈی ایف

| خصوصیت | یوآرڈی ایف | ایس ایس ڈی ایف |
|---------|------|-----|
| بنیادی استعمال | ROS روبوٹ کی تفصیل | گیزبو سیمولیشن |
| فزکس | محدود | جامع |
| دنیا کی تفصیل | نہیں | ہاں |
| پلگ انز | محدود | وسیع |
| سینسرز | بنیادی | ایڈوانسڈ |

## ایس ایس ڈی ایف فائل سٹرکچر

ایک بنیادی ایس ایس ڈی ایف فائل:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="default">
    <!-- دنیا کی خصوصیات -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- دنیا میں ماڈلز -->
    <model name="my_robot">
      <!-- ماڈل کی تعریف -->
      <link name="chassis">
        <pose>0 0 0.1 0 0 0</pose>
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

        <visual name="chassis_visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.4 0.4 1.0 1</diffuse>
          </material>
        </visual>

        <collision name="chassis_collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## ایڈوانس فزکس خصوصیات

ایس ایس ڈی ایف تفصیلی فزکس کنفیگریشن کی اجازت دیتا ہے:

```xml
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## ایس ایس ڈی ایف میں سینسرز

ایس ایس ڈی ایف مختلف سینسرز کے لیے مقامی سپورٹ فراہم کرتا ہے:

```xml
<link name="sensor_link">
  <!-- لیڈار سینسر -->
  <sensor name="lidar" type="ray">
    <pose>0.1 0 0.1 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>640</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <always_on>1</always_on>
    <update_rate>10</update_rate>
  </sensor>

  <!-- کیمرہ سینسر -->
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
  </sensor>
</link>
```

## دنیا کی خصوصیات

ایس ایس ڈی ایف مکمل سیمولیشن دنیا کی وضاحت کر سکتا ہے:

```xml
<world name="my_world">
  <!-- گیزبو کے ماڈل ڈیٹا بیس سے ماڈلز شامل کریں -->
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <include>
    <uri>model://sun</uri>
  </include>

  <!-- کسٹم لائٹنگ -->
  <light name="my_light" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.3 0.3 -1</direction>
  </light>

  <!-- زمین -->
  <heightmap name="my_terrain">
    <uri>file://path/to/heightmap.png</uri>
    <size>100 100 20</size>
    <pos>0 0 0</pos>
  </heightmap>
</world>
```

## یوآرڈی ایف سے ایس ایس ڈی ایف میں تبدیلی

یوآرڈی ایف کو ایس ایس ڈی ایف میں تبدیل کرنے کے لیے:

```bash
# ایس ایس ڈی ایف کو یوآرڈی ایف سے جنریٹ کریں
gz sdf -p robot.urdf > robot.sdf

# یا میکروز استعمال کرتے ہوئے پری پروسیس کے لیے xacro استعمال کریں
xacro robot.urdf.xacro | gz sdf -p /dev/stdin > robot.sdf
```

## بہترین طریقے

- گیزبو کی خصوصیات کے لیے ایس ایس ڈی ایف استعمال کریں (سینسرز، فزکس، دنیا)
- ROS انٹیگریشن اور بنیادی روبوٹ کی تفصیل کے لیے یوآرڈی ایف استعمال کریں
- پیچیدہ ایس ایس ڈی ایف فائلز کو آسان بنانے کے لیے Xacro میکروز کا استعما ل کریں
- اپنی ایس ایس ڈی ایف فائلز کی توثیق کریں: `gz sdf -k your_file.sdf`
- پیچیدگی کو کم کرنے کے لیے جب بھی ممکن ہو گیزبو کے بلٹ ان ماڈلز استعمال کریں

## اگلے اقدامات

عملی مثالوں کے ساتھ [روبوٹ ماڈلز بنانا](./creating-robot-models.md) کے بارے میں سیکھنے کے لیے جاری رکھیں۔