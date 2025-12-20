# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

## Prerequisites

Before starting Module 2, ensure you have:
- Completed Module 1 (ROS 2 Fundamentals)
- ROS 2 Humble Hawksbill installed
- Basic Python programming knowledge
- Familiarity with command line tools

## Environment Setup

### 1. Install Gazebo Garden
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install gazebo
# Or install specific Garden version
sudo apt install gazebo-garden
```

### 2. Install Unity Hub and Unity LTS
1. Download Unity Hub from the Unity website
2. Install Unity Hub and use it to install Unity 2022.3.x LTS
3. Create a Unity account if needed

### 3. Verify ROS 2 Installation
```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version
```

## First Simulation

### 1. Create a Basic Robot Model
Create a simple URDF file (`basic_robot.urdf`):

```xml
<?xml version="1.0"?>
<robot name="basic_robot">
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

### 2. Launch Gazebo with Your Robot
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo
gz sim -r empty.sdf

# In another terminal, spawn your robot
ros2 run ros_gz_bridge parameter_bridge /model/basic_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

### 3. Verify Simulation
- Check that your robot appears in Gazebo
- Verify physics properties (gravity, collisions)
- Test basic movement if joints are defined

## Docusaurus Integration

### 1. Update Sidebar Navigation
Add Module 2 to `book/sidebars.js`:

```javascript
module.exports = {
  docs: {
    "Introduction": [...],
    "Module 1: ROS 2 Fundamentals": [...],
    "Module 2: The Digital Twin (Gazebo & Unity)": [
      "module-2/introduction/prerequisites",
      "module-2/introduction/setup-gazebo-environment",
      {
        type: "category",
        label: "URDF/SDF Formats",
        items: [
          "module-2/urdf-sdf-formats/urdf-basics",
          "module-2/urdf-sdf-formats/sdf-advanced",
          "module-2/urdf-sdf-formats/creating-robot-models",
          "module-2/urdf-sdf-formats/conversion-guide"
        ]
      },
      // ... continue with other sections
    ],
    "Projects": [...]
  }
};
```

### 2. Create Your First Documentation Page
Create `book/docs/module-2/introduction/setup-gazebo-environment.md`:

```markdown
---
sidebar_position: 2
---

# Setting up Gazebo Environment

This tutorial will guide you through setting up your Gazebo simulation environment...

## Prerequisites

- ROS 2 Humble Hawksbill
- Compatible Ubuntu version (22.04 LTS recommended)

## Installation Steps

1. Install Gazebo Garden
2. Verify installation
3. Test basic simulation

## Troubleshooting

Common issues and solutions...
```

## Testing Your Setup

Run the automated tests to verify your simulation environment:

```bash
# Navigate to the tests directory
cd tests/simulation

# Run Gazebo environment tests
npx playwright test gazebo-tests/environment-setup.test.js

# Run documentation validation
npm run test:docs
```

## Next Steps

1. Complete the Gazebo environment setup tutorial
2. Learn about URDF and SDF robot description formats
3. Implement physics simulation with gravity and collisions
4. Add sensor simulation to your robots
5. Explore Unity integration for high-fidelity visualization