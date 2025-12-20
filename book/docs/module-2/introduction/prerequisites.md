---
sidebar_position: 1
---

# Prerequisites for Gazebo & Unity Simulation

Before starting Module 2, ensure you have completed the following prerequisites:

## Required Knowledge

- [Module 1: ROS 2 Fundamentals](/docs/module-1) - Complete understanding of ROS 2 concepts
- Basic Python programming knowledge (Python 3.10+)
- Familiarity with command line tools
- Understanding of basic robotics concepts

## Software Requirements

### ROS 2 Installation
- ROS 2 Humble Hawksbill installed and functional
- Verify installation: `ros2 --version`

### System Requirements
- Ubuntu 22.04 LTS (primary target) or compatible system
- Minimum 8GB RAM (16GB recommended for Unity)
- Modern GPU recommended for Unity visualization
- At least 10GB free disk space for simulation environments

## Installation Verification

Before proceeding, verify your environment:

```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Check for Gazebo (installation will be covered in next section)
gz --version
```

## Next Steps

Once you've confirmed all prerequisites are met, continue to [Setting up Gazebo Environment](./setup-gazebo-environment.md) to begin your simulation journey.