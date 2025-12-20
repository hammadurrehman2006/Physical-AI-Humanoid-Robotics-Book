---
sidebar_position: 2
---

# SDF Advanced: Simulation Description Format

SDF (Simulation Description Format) is Gazebo's native XML format for describing simulation environments, including robots, objects, and world properties. While URDF is ROS-specific, SDF is designed specifically for physics simulation.

## SDF vs URDF

| Feature | URDF | SDF |
|---------|------|-----|
| Primary Use | ROS robot description | Gazebo simulation |
| Physics | Limited | Comprehensive |
| World Description | No | Yes |
| Plugins | Limited | Extensive |
| Sensors | Basic | Advanced |

## SDF File Structure

A basic SDF file:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="default">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Models in the world -->
    <model name="my_robot">
      <!-- Model definition -->
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

## Advanced Physics Properties

SDF allows detailed physics configuration:

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

## Sensors in SDF

SDF provides native support for various sensors:

```xml
<link name="sensor_link">
  <!-- LiDAR sensor -->
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

  <!-- Camera sensor -->
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

## World Features

SDF can define complete simulation worlds:

```xml
<world name="my_world">
  <!-- Include models from Gazebo's model database -->
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <include>
    <uri>model://sun</uri>
  </include>

  <!-- Custom lighting -->
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

  <!-- Terrain -->
  <heightmap name="my_terrain">
    <uri>file://path/to/heightmap.png</uri>
    <size>100 100 20</size>
    <pos>0 0 0</pos>
  </heightmap>
</world>
```

## Converting URDF to SDF

To convert URDF to SDF:

```bash
# Generate SDF from URDF
gz sdf -p robot.urdf > robot.sdf

# Or use xacro to preprocess if using macros
xacro robot.urdf.xacro | gz sdf -p /dev/stdin > robot.sdf
```

## Best Practices

- Use SDF for Gazebo-specific features (sensors, physics, world)
- Use URDF for ROS integration and basic robot description
- Consider using Xacro macros to simplify complex SDF files
- Validate your SDF files: `gz sdf -k your_file.sdf`
- Use Gazebo's built-in models when possible to reduce complexity

## Next Steps

Continue to learn about [Creating Robot Models](./creating-robot-models.md) with practical examples.