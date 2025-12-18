---
title: Comprehensive Setup Guide
description: Complete setup guide for Physical AI and humanoid robotics development environment
sidebar_position: 1
---

# Comprehensive Setup Guide

## Overview

This guide provides detailed instructions for setting up your complete development environment for Physical AI and humanoid robotics. Follow these steps in order to ensure all components work together properly.

## System Requirements

### Hardware Requirements
- **Processor**: Multi-core processor (Intel i5 or equivalent recommended)
- **RAM**: Minimum 8GB (16GB recommended)
- **Storage**: Minimum 50GB free space
- **Graphics**: GPU with OpenGL 3.2+ support (for visualization)
- **Network**: Internet connection for package installation

### Operating System
- **Recommended**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Architecture**: 64-bit (amd64)

## Step 1: Operating System Installation

### Installing Ubuntu 22.04 LTS

1. Download Ubuntu 22.04 LTS from [ubuntu.com](https://ubuntu.com/download/desktop)
2. Create a bootable USB drive using:
   - **Windows**: Rufus or Etcher
   - **Linux**: `dd` command or Startup Disk Creator
   - **macOS**: Etcher or Terminal command
3. Boot from USB and follow installation wizard
4. Choose "Install Ubuntu" and follow prompts
5. Set up user account and password

### Post-Installation Setup

Update system packages:
```bash
sudo apt update && sudo apt upgrade -y
```

Install basic development tools:
```bash
sudo apt install build-essential cmake git curl wget vim htop python3-pip python3-dev
```

## Step 2: ROS 2 Humble Hawksbill Installation

ROS 2 Humble Hawksbill is the LTS version required for this curriculum.

### Setting up Locale
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Adding ROS 2 GPG Key and Repository
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Installing ROS 2 Packages
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge ros-humble-tf2-py ros-humble-tf2-ros ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager
```

### Installing Additional Python Packages
```bash
pip3 install -U rosdep rosinstall_generator vcstool setuptools
sudo rosdep init
rosdep update
```

### Setting up ROS 2 Environment
Add to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 3: Development Environment Setup

### Creating ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build --symlink-install

# Add workspace setup to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Python Virtual Environment Setup
```bash
# Create virtual environment for Python development
cd ~
python3 -m venv physical_ai_env
source physical_ai_env/bin/activate

# Install required Python packages
pip install numpy scipy matplotlib opencv-python transforms3d pygame

# Install ROS 2 Python tools
pip install ros2cli

# Create requirements file
pip freeze > ~/physical_ai_requirements.txt
```

### Visual Studio Code Setup
```bash
# Download and install VS Code
wget -qO - https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code
```

After installation, install these VS Code extensions:
- ROS (by Microsoft)
- Python (by Microsoft)
- C/C++ (by Microsoft)
- Docker (by Microsoft)
- GitLens (by GitKraken)

## Step 4: Simulation Environment Setup

### Installing Gazebo Garden
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-ign-*
```

### Installing Additional Simulation Tools
```bash
# Install MoveIt! for motion planning
sudo apt install ros-humble-moveit

# Install Navigation2 stack
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install perception packages
sudo apt install ros-humble-perception ros-humble-vision-opencv ros-humble-depthimage-to-laserscan
```

## Step 5: Version Control Setup

### Git Configuration
```bash
# Configure Git for development
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
git config --global core.editor "vim"
git config --global color.ui auto

# Set up Git credentials helper
git config --global credential.helper store
```

### SSH Key Setup (for GitHub)
```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "your.email@example.com"

# Start SSH agent
eval "$(ssh-agent -s)"

# Add SSH key to agent
ssh-add ~/.ssh/id_ed25519

# Copy public key to clipboard
cat ~/.ssh/id_ed25519.pub
```

## Step 6: Python Development Setup

### Installing Additional Python Packages
```bash
# Activate virtual environment
source ~/physical_ai_env/bin/activate

# Install robotics-specific packages
pip install rclpy transforms3d pygame pybullet

# Install scientific computing packages
pip install numpy scipy matplotlib pandas jupyter

# Install computer vision packages
pip install opencv-python open3d

# Install machine learning packages
pip install torch torchvision tensorflow scikit-learn

# Install visualization packages
pip install plotly seaborn
```

### Jupyter Notebook Setup
```bash
# Install Jupyter
pip install jupyter jupyterlab

# Create Jupyter configuration
jupyter notebook --generate-config

# Set up Jupyter for ROS 2 development
pip install ipywidgets
jupyter nbextension enable --py widgetsnbextension
```

## Step 7: Hardware Interface Setup (Optional)

### Installing USB Serial Drivers
```bash
sudo apt install python3-serial
pip install pyserial
```

### Setting up Udev Rules for USB Devices
Create `/etc/udev/rules.d/99-robot-serial.rules`:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a7b", ATTRS{idProduct}=="0001", MODE="0666", GROUP="dialout"
```

Then reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Step 8: Verification and Testing

### Testing ROS 2 Installation
```bash
# Source the environment
source /opt/ros/humble/setup.bash

# Test basic ROS 2 commands
ros2 --version
ros2 pkg list | head -10

# Test Python client
python3 -c "import rclpy; print('rclpy imported successfully')"
```

### Testing Python Environment
```bash
# Activate virtual environment
source ~/physical_ai_env/bin/activate

# Test key packages
python3 -c "import numpy, scipy, matplotlib, cv2, transforms3d; print('Core packages imported successfully')"
```

### Testing Basic ROS 2 Functionality
```bash
# In one terminal, run a talker
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal, run a listener
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

## Step 9: IDE Configuration

### VS Code Settings for ROS 2 Development

Create `.vscode/settings.json` in your workspace:
```json
{
    "python.defaultInterpreterPath": "~/physical_ai_env/bin/python",
    "python.terminal.activateEnvironment": true,
    "ros.distro": "humble",
    "files.associations": {
        "*.msg": "yaml",
        "*.srv": "yaml",
        "*.action": "yaml"
    },
    "cmake.configureOnOpen": true,
    "C_Cpp.default.compilerPath": "/usr/bin/gcc",
    "C_Cpp.default.cStandard": "c17",
    "C_Cpp.default.cppStandard": "c++17"
}
```

### Creating VS Code Workspace

Create `physical_ai.code-workspace`:
```json
{
    "folders": [
        {
            "name": "ROS2 Workspace",
            "path": "~/ros2_ws/src"
        },
        {
            "name": "Python Projects",
            "path": "~/physical_ai_projects"
        }
    ],
    "settings": {
        "terminal.integrated.cwd": "~/ros2_ws",
        "python.defaultInterpreterPath": "~/physical_ai_env/bin/python",
        "ros.distro": "humble"
    }
}
```

## Step 10: Additional Tools and Utilities

### Installing Development Utilities
```bash
# System monitoring tools
sudo apt install htop iotop iftop

# Network tools
sudo apt install net-tools nmap

# File compression tools
sudo apt install unrar p7zip-full

# Development libraries
sudo apt install libeigen3-dev libboost-all-dev libyaml-cpp-dev
```

### Installing Docker (Optional)
```bash
# Remove old versions
sudo apt remove docker docker-engine docker.io containerd runc

# Install Docker
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
```

## Troubleshooting Common Setup Issues

### 1. ROS 2 Environment Not Sourcing
**Problem**: `ros2` command not found
**Solution**:
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Python Package Import Errors
**Problem**: ImportError for rclpy or other ROS packages
**Solution**:
```bash
source /opt/ros/humble/setup.bash
pip3 install rclpy
```

### 3. Permission Issues with Docker
**Problem**: Permission denied when running Docker commands
**Solution**: Add user to docker group and log out/in
```bash
sudo usermod -aG docker $USER
```

### 4. Gazebo Not Starting
**Problem**: Gazebo fails to start with graphics errors
**Solution**: Check graphics drivers and run:
```bash
export LIBGL_ALWAYS_SOFTWARE=1  # For software rendering
```

### 5. Workspace Build Failures
**Problem**: `colcon build` fails with missing dependencies
**Solution**:
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Final Verification

Run this script to verify your complete setup:

```bash
#!/bin/bash
echo "=== Physical AI Development Environment Verification ==="

echo "1. Checking ROS 2 Installation..."
source /opt/ros/humble/setup.bash
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 installed: $(ros2 --version)"
else
    echo "✗ ROS 2 not found"
fi

echo "2. Checking Python Environment..."
source ~/physical_ai_env/bin/activate
if python3 -c "import rclpy" &> /dev/null; then
    echo "✓ rclpy available in virtual environment"
else
    echo "✗ rclpy not available in virtual environment"
fi

echo "3. Checking Core Python Packages..."
if python3 -c "import numpy, scipy, matplotlib, cv2" &> /dev/null; then
    echo "✓ Core Python packages available"
else
    echo "✗ Some core Python packages missing"
fi

echo "4. Checking Development Tools..."
if command -v git &> /dev/null && command -v cmake &> /dev/null; then
    echo "✓ Git and CMake available"
else
    echo "✗ Git or CMake not available"
fi

echo "5. Testing Basic ROS 2 Functionality..."
timeout 5 ros2 run demo_nodes_py talker --ros-args --log-level error > /dev/null 2>&1 &
TALKER_PID=$!
sleep 1
kill $TALKER_PID 2>/dev/null

if [ $? -ne 137 ]; then  # 137 means it was killed by SIGTERM
    echo "✗ ROS 2 basic functionality test failed"
else
    echo "✓ ROS 2 basic functionality test passed"
fi

echo "=== Setup Verification Complete ==="
```

Save this as `verify_setup.sh`, make it executable, and run it:
```bash
chmod +x verify_setup.sh
./verify_setup.sh
```

## Next Steps

After completing this setup:

1. **Explore the Introduction section** of this book to understand Physical AI fundamentals
2. **Complete the prerequisites module** to ensure your environment is properly configured
3. **Start with Module 1** to learn ROS 2 architecture and core concepts
4. **Practice with the assessment projects** to validate your understanding

Your development environment is now ready for Physical AI and humanoid robotics development!