---
sidebar_position: 5
---

# Troubleshooting Common Gazebo Issues

This guide provides solutions to common issues you may encounter when setting up and running Gazebo simulations.

## Common Installation Issues

### Missing Dependencies

If you encounter errors during Gazebo installation:

```bash
# Update package lists
sudo apt update

# Install essential dependencies
sudo apt install libglu1-mesa freeglut3-dev mesa-common-dev

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-*
```

### Graphics Driver Issues

If Gazebo fails to launch with graphics errors:

1. **Check your graphics drivers**:
   ```bash
   # Check current graphics driver
   sudo lshw -c display

   # Install recommended drivers
   sudo ubuntu-drivers autoinstall
   ```

2. **Try software rendering**:
   ```bash
   # For Intel graphics or when using virtual machines
   export MESA_GL_VERSION_OVERRIDE=3.3
   gz sim
   ```

## Common Runtime Issues

### Simulation Instability

If your simulation is unstable or objects are behaving unexpectedly:

1. **Adjust physics parameters**:
   ```bash
   # In your world file, reduce max_step_size
   <max_step_size>0.001</max_step_size>
   ```

2. **Verify inertial properties** in your URDF/SDF files:
   - Ensure all links have proper mass values
   - Check that inertia values are positive and reasonable

### Performance Issues

If simulation is running slowly:

1. **Reduce update rate**:
   ```xml
   <update_rate>100</update_rate>  <!-- Instead of 1000 -->
   ```

2. **Simplify collision geometries**:
   - Use simple shapes (boxes, spheres, cylinders) instead of meshes
   - Reduce the number of triangles in visual meshes

### Sensor Issues

If sensors are not publishing data:

1. **Check sensor configuration**:
   - Verify sensor name and type
   - Ensure `always_on` is set to `1`
   - Check update rate is reasonable

2. **Validate ROS 2 interfaces**:
   ```bash
   # List available topics
   ros2 topic list

   # Check if sensor topic exists
   ros2 topic echo /your_sensor_topic
   ```

## ROS 2 Integration Issues

### Plugin Loading Errors

If Gazebo plugins fail to load:

1. **Verify plugin library paths**:
   ```bash
   # Check if plugin exists
   find /usr/lib -name "*gazebo_ros*"
   ```

2. **Check ROS 2 environment**:
   ```bash
   # Source ROS 2 setup
   source /opt/ros/humble/setup.bash
   ```

### Control Interface Problems

If robot control isn't working:

1. **Verify transmission configuration** in URDF:
   ```xml
   <transmission name="wheel_trans">
     <type>transmission_interface/SimpleTransmission</type>
     <joint name="wheel_joint">
       <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
     </joint>
   </transmission>
   ```

2. **Check control configuration file**:
   - Ensure YAML control config matches URDF
   - Verify joint names match exactly

## Debugging Tips

### Enable Verbose Logging

```bash
# Launch with verbose output
gz sim -v 4 your_world.sdf
```

### Check Model Spawn

```bash
# List spawned models
gz model -m

# Check model pose
gz model -m your_model_name -i
```

### Validate SDF Files

```bash
# Check SDF syntax
gz sdf -k your_file.sdf

# Convert and check URDF
gz sdf -p your_robot.urdf
```

## Testing Your Setup

Create a simple test to verify your Gazebo installation:

1. **Create a minimal world file** (`test_world.sdf`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.10">
     <world name="test_world">
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>
       <model name="box">
         <pose>0 0 0.5 0 0 0</pose>
         <link name="box_link">
           <visual name="visual">
             <geometry>
               <box><size>1 1 1</size></box>
             </geometry>
           </visual>
           <collision name="collision">
             <geometry>
               <box><size>1 1 1</size></box>
             </geometry>
           </collision>
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
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Launch the test world**:
   ```bash
   gz sim -r test_world.sdf
   ```

3. **Verify the box appears and responds to physics**.

## Getting Help

If you encounter issues not covered here:

1. **Check the Gazebo documentation**: [gazebosim.org](https://gazebosim.org/)
2. **ROS Answers**: [answers.ros.org](https://answers.ros.org)
3. **Gazebo community forum**: [community.gazebosim.org](https://community.gazebosim.org)

## Next Steps

Once you've resolved any setup issues, continue to learn about [URDF and SDF robot description formats](../urdf-sdf-formats/urdf-basics.md).