---
sidebar_position: 2
---

# Setting up Gazebo Environment

This tutorial will guide you through setting up your Gazebo simulation environment for robotics development.

## Prerequisites

- ROS 2 Humble Hawksbill
- Compatible Ubuntu version (22.04 LTS recommended)
- At least 8GB RAM

## Installation Steps

### 1. Install Gazebo Garden

For Ubuntu 22.04:

```bash
# Update package lists
sudo apt update

# Install Gazebo Garden
sudo apt install gazebo

# Alternative: Install specific Garden version
sudo apt install gazebo-garden
```

### 2. Verify Installation

```bash
# Check Gazebo version
gz --version

# Test basic Gazebo launch
gz sim
```

### 3. Test Basic Simulation

```bash
# Launch Gazebo with empty world
gz sim -r empty.sdf

# In another terminal, verify you can connect to the simulation
gz topic -l
```

## Troubleshooting

### Common Issues

1. **Gazebo fails to launch with graphics errors**:
   - Ensure you have proper graphics drivers installed
   - Try running with software rendering: `export MESA_GL_VERSION_OVERRIDE=3.3; gz sim`

2. **Missing dependencies**:
   - Run `sudo apt update && sudo apt upgrade` to ensure all packages are current
   - Install missing ROS 2 dependencies: `sudo apt install ros-humble-gazebo-*`

3. **Permission issues**:
   - Ensure your user is in the dialout group: `sudo usermod -a -G dialout $USER`
   - Log out and back in for changes to take effect

## Next Steps

After successfully setting up Gazebo, continue to learn about [URDF and SDF robot description formats](../urdf-sdf-formats/urdf-basics.md).