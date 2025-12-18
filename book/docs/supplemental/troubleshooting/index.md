---
title: Troubleshooting Guide
description: Comprehensive troubleshooting guide for Physical AI and humanoid robotics development
sidebar_position: 2
---

# Troubleshooting Guide

## Overview

This troubleshooting guide provides solutions for common issues encountered during Physical AI and humanoid robotics development. Use this guide when you encounter problems with your ROS 2 setup, robot simulation, or development environment.

## Common ROS 2 Issues

### 1. ROS 2 Environment Not Sourced

**Problem**: `ros2` command not found or ROS 2 packages not accessible.

**Symptoms**:
- `command 'ros2' not found`
- ImportError when importing rclpy
- Nodes cannot find each other on the network

**Solutions**:
1. **Immediate fix**: Source the ROS 2 environment
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Permanent fix**: Add to your `~/.bashrc`
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Check ROS_DISTRO**: Verify the environment variable
   ```bash
   echo $ROS_DISTRO
   # Should output: humble
   ```

### 2. Package Not Found Issues

**Problem**: ROS 2 cannot find installed packages.

**Symptoms**:
- `Package 'package_name' not found`
- `ament_cmake' is not a valid build type`

**Solutions**:
1. **Source your workspace**: If working with custom packages
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ```

2. **Check package installation**:
   ```bash
   ros2 pkg list | grep package_name
   ```

3. **Rebuild workspace**:
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build --symlink-install
   source install/setup.bash
   ```

### 3. Node Communication Issues

**Problem**: Nodes cannot communicate with each other.

**Symptoms**:
- Publishers and subscribers not connecting
- Services not responding
- Action clients not receiving feedback

**Solutions**:
1. **Check network configuration**:
   ```bash
   # Check if nodes can see each other
   ros2 node list

   # Check topic connections
   ros2 topic list
   ros2 topic info /topic_name
   ```

2. **Verify ROS_DOMAIN_ID**:
   ```bash
   echo $ROS_DOMAIN_ID
   # If set to different values, nodes won't see each other
   # Set to same value for all terminals: export ROS_DOMAIN_ID=42
   ```

3. **Check QoS compatibility**:
   ```bash
   # Ensure publishers and subscribers have compatible QoS settings
   # Reliable publishers need reliable subscribers, etc.
   ```

## Python Development Issues

### 1. Python Package Import Errors

**Problem**: ImportError when importing Python packages.

**Symptoms**:
- `ModuleNotFoundError: No module named 'rclpy'`
- `ImportError: No module named 'cv2'`

**Solutions**:
1. **Check Python environment**:
   ```bash
   # Verify you're using the correct Python
   which python3
   python3 --version

   # Check installed packages
   pip3 list | grep package_name
   ```

2. **Install missing packages**:
   ```bash
   # For ROS 2 packages
   source /opt/ros/humble/setup.bash
   pip3 install rclpy

   # For other packages
   pip3 install package_name
   ```

3. **Virtual environment issues**:
   ```bash
   # Activate virtual environment
   source ~/physical_ai_env/bin/activate

   # Install packages in virtual environment
   pip install package_name
   ```

### 2. Permission Issues with Python

**Problem**: Permission denied when installing packages.

**Symptoms**:
- `Permission denied: '/usr/local/lib/python3.x/site-packages'`
- `Operation not permitted`

**Solutions**:
1. **Use user flag**:
   ```bash
   pip3 install --user package_name
   ```

2. **Use virtual environment** (recommended):
   ```bash
   source ~/physical_ai_env/bin/activate
   pip install package_name
   ```

3. **Fix pip permissions** (if needed):
   ```bash
   # This is generally not recommended but sometimes necessary
   sudo chown -R $USER:$USER ~/.local/lib/python3.x/site-packages
   ```

## Build System Issues

### 1. Colcon Build Failures

**Problem**: `colcon build` fails with compilation errors.

**Symptoms**:
- Build errors during compilation
- Missing dependencies
- CMake configuration errors

**Solutions**:
1. **Clean build artifacts**:
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   ```

2. **Install missing dependencies**:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build with verbose output**:
   ```bash
   colcon build --packages-select package_name --event-handlers console_direct+
   ```

4. **Build with more detailed logging**:
   ```bash
   colcon build --packages-select package_name --cmake-args -DCMAKE_BUILD_TYPE=Debug
   ```

### 2. CMake Configuration Issues

**Problem**: CMake fails to configure packages.

**Symptoms**:
- `CMake Error at CMakeLists.txt`
- `Could not find a package configuration file`

**Solutions**:
1. **Check for missing dependencies**:
   ```bash
   apt list --installed | grep ros-humble
   sudo apt update && sudo apt upgrade
   ```

2. **Verify CMakeLists.txt**:
   - Ensure `find_package()` calls are correct
   - Check package dependencies in `package.xml`

3. **Clean and rebuild**:
   ```bash
   cd ~/ros2_ws/build/package_name
   rm -rf *
   cd ~/ros2_ws
   colcon build --packages-select package_name
   ```

## Simulation Issues

### 1. Gazebo Not Starting

**Problem**: Gazebo simulation fails to start.

**Symptoms**:
- Gazebo window doesn't appear
- Graphics errors in console
- Segmentation faults

**Solutions**:
1. **Check graphics drivers**:
   ```bash
   # Check OpenGL support
   glxinfo | grep "OpenGL version"

   # If using virtual machine, ensure 3D acceleration is enabled
   ```

2. **Software rendering as fallback**:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```

3. **Reset Gazebo configuration**:
   ```bash
   rm -rf ~/.gazebo
   # Restart Gazebo
   ```

### 2. Robot Model Not Loading in Gazebo

**Problem**: URDF robot model doesn't appear in simulation.

**Symptoms**:
- Robot not visible in Gazebo
- Model spawns with errors
- Joint states not publishing

**Solutions**:
1. **Validate URDF**:
   ```bash
   # Check URDF syntax
   check_urdf /path/to/robot.urdf

   # Use xacro if applicable
   ros2 run xacro xacro -o output.urdf input.xacro
   ```

2. **Check robot state publisher**:
   ```bash
   # Ensure robot_state_publisher is running
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'
   ```

3. **Verify joint state publisher**:
   ```bash
   # For fixed joints, joint_state_publisher might be needed
   ros2 run joint_state_publisher joint_state_publisher
   ```

## Hardware Interface Issues

### 1. USB Device Not Recognized

**Problem**: Robot hardware not detected via USB.

**Symptoms**:
- `Permission denied` when accessing serial port
- Device not listed in `/dev/tty*`
- Connection timeouts

**Solutions**:
1. **Check device detection**:
   ```bash
   # Check available serial ports
   ls /dev/tty*

   # Check with device connected/disconnected
   dmesg | tail -20
   ```

2. **Add user to dialout group**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in for changes to take effect
   ```

3. **Set up udev rules** (if needed):
   ```bash
   # Create udev rule file
   sudo nano /etc/udev/rules.d/99-robot-device.rules
   # Add rule: SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", MODE="0666", GROUP="dialout"
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

### 2. Serial Communication Errors

**Problem**: Errors when communicating with hardware via serial.

**Symptoms**:
- `SerialException` errors
- Connection timeouts
- Data corruption

**Solutions**:
1. **Check connection parameters**:
   ```python
   # Verify baud rate, timeout, and other settings match hardware
   import serial
   ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1.0)
   ```

2. **Test serial connection**:
   ```bash
   # Use screen to test connection
   screen /dev/ttyUSB0 115200

   # Or use minicom
   sudo apt install minicom
   minicom -D /dev/ttyUSB0 -b 115200
   ```

3. **Check permissions**:
   ```bash
   # Verify permissions on serial device
   ls -la /dev/ttyUSB0

   # Change ownership if needed
   sudo chown $USER:$USER /dev/ttyUSB0
   ```

## Performance and Resource Issues

### 1. High CPU Usage

**Problem**: ROS 2 nodes consuming excessive CPU resources.

**Symptoms**:
- High CPU usage shown in system monitor
- Slow response times
- Overheating

**Solutions**:
1. **Optimize node frequency**:
   ```python
   # Reduce timer frequency in nodes
   self.timer = self.create_timer(0.1, callback)  # 10 Hz instead of higher frequencies
   ```

2. **Check for busy loops**:
   ```python
   # Avoid busy waiting
   time.sleep(0.01)  # Instead of while loop without sleep
   ```

3. **Monitor resource usage**:
   ```bash
   # Use htop to monitor processes
   htop

   # Monitor ROS 2 nodes specifically
   ros2 run topicos topico
   ```

### 2. Memory Leaks

**Problem**: Memory usage continuously increasing.

**Symptoms**:
- Memory usage grows over time
- System becomes sluggish
- Out of memory errors

**Solutions**:
1. **Check for circular references**:
   ```python
   # Ensure proper cleanup in destroy_node()
   def destroy_node(self):
       # Clean up timers, publishers, subscribers
       super().destroy_node()
   ```

2. **Monitor memory usage**:
   ```bash
   # Use ROS 2 tools to monitor memory
   ros2 doctor

   # Use system tools
   watch -n 1 'ps aux | grep ros2'
   ```

## Network and Communication Issues

### 1. Multi-Robot Communication Problems

**Problem**: Multiple robots cannot communicate properly.

**Symptoms**:
- Nodes from different robots cannot see each other
- Topic conflicts
- Parameter confusion

**Solutions**:
1. **Use namespaces**:
   ```python
   # Launch nodes with namespaces
   ros2 run package node --ros-args --remap __ns:=/robot1
   ```

2. **Set different domain IDs**:
   ```bash
   export ROS_DOMAIN_ID=1  # For robot 1
   export ROS_DOMAIN_ID=2  # For robot 2
   ```

3. **Use proper remapping**:
   ```bash
   ros2 run package node --ros-args --remap /topic:=/robot1/topic
   ```

### 2. Network Discovery Issues

**Problem**: ROS 2 nodes cannot discover each other across network.

**Symptoms**:
- Nodes on different machines cannot communicate
- Topics not visible across network
- Service calls fail across machines

**Solutions**:
1. **Check firewall settings**:
   ```bash
   # Open necessary ports for DDS communication
   sudo ufw allow 11811:11911/udp
   sudo ufw allow 11811:11911/tcp
   ```

2. **Verify network configuration**:
   ```bash
   # Check if machines can ping each other
   ping other_machine_ip

   # Set ROS_LOCALHOST_ONLY if testing locally
   export ROS_LOCALHOST_ONLY=0
   ```

3. **Configure DDS settings**:
   ```bash
   # Set to use network interfaces properly
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

## Debugging Techniques

### 1. ROS 2 Debugging Tools

Use these tools to debug your ROS 2 applications:

```bash
# View all nodes
ros2 node list

# View topics and their types
ros2 topic list -t

# Echo a topic to see messages
ros2 topic echo /topic_name std_msgs/msg/String

# Check topic information
ros2 topic info /topic_name

# Service call debugging
ros2 service list
ros2 service call /service_name service_type "{request_field: value}"

# Action debugging
ros2 action list
ros2 action send_goal /action_name action_type "{goal_field: value}"
```

### 2. Logging and Monitoring

```bash
# Set log level
ros2 run package node --ros-args --log-level debug

# Monitor system
ros2 run topicos topico

# Check system health
ros2 doctor
```

### 3. Code Debugging

For Python nodes, use debugging techniques:

```python
import rclpy
from rclpy.logging import LoggingSeverity

# Add detailed logging
def some_function(self):
    self.get_logger().debug('Debug message')
    self.get_logger().info('Info message')
    self.get_logger().warn('Warning message')
    self.get_logger().error('Error message')
    self.get_logger().fatal('Fatal message')
```

## Recovery Procedures

### 1. Complete Environment Reset

If you encounter persistent issues, you can reset your environment:

```bash
# Remove workspace
rm -rf ~/ros2_ws

# Remove virtual environment
rm -rf ~/physical_ai_env

# Clean ROS 2 cache
rm -rf ~/.ros

# Reinstall ROS 2 (if necessary)
sudo apt remove ros-humble-*
sudo apt autoremove
# Then reinstall following setup guide
```

### 2. Package Reinstallation

For specific package issues:

```bash
# Remove and reinstall specific package
sudo apt remove ros-humble-package-name
sudo apt autoremove
sudo apt install ros-humble-package-name

# For Python packages
pip3 uninstall package_name
pip3 install package_name
```

## Getting Additional Help

### 1. Community Resources

- **ROS Answers**: [answers.ros.org](https://answers.ros.org)
- **ROS Discourse**: [discourse.ros.org](https://discourse.ros.org)
- **GitHub Issues**: Check the specific package repositories
- **Stack Overflow**: Use tags `ros2`, `robotics`, `python`

### 2. Diagnostic Commands

Run these commands to gather information for support:

```bash
# System information
uname -a
lsb_release -a

# ROS 2 information
ros2 --version
printenv | grep ROS

# Package information
dpkg -l | grep ros-humble

# Python environment
python3 --version
pip3 list | grep ros
```

### 3. Creating Minimal Reproducible Examples

When seeking help, create minimal examples:

1. **Isolate the problem**: Create the smallest code that reproduces the issue
2. **Include environment details**: ROS version, Ubuntu version, etc.
3. **Provide exact error messages**: Copy-paste the full error
4. **Include your steps**: What you did before the error occurred

This troubleshooting guide covers the most common issues in Physical AI and humanoid robotics development. If you encounter a problem not covered here, refer to the official ROS 2 documentation or seek help from the community.