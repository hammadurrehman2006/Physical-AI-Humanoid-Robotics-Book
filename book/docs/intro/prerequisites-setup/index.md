---
title: Prerequisites and Setup
description: Essential requirements and setup procedures for Physical AI and humanoid robotics development
sidebar_position: 7
---

# Prerequisites and Setup

## Learning Objectives
- Understand the software and hardware prerequisites for Physical AI development
- Learn how to set up a development environment for ROS 2 and humanoid robotics
- Configure necessary tools and dependencies
- Validate the setup with basic tests

## Software Prerequisites

### Operating System Requirements

Physical AI development with ROS 2 Humble Hawksbill requires a Linux-based operating system. Ubuntu 22.04 LTS is the recommended distribution.

#### System Requirements:
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) or later
- **Architecture**: 64-bit (amd64)
- **RAM**: Minimum 8GB (16GB recommended)
- **Storage**: Minimum 50GB free space
- **Processor**: Multi-core processor (Intel i5 or equivalent recommended)

### Python Requirements

Python 3.10+ is required for ROS 2 Humble Hawksbill compatibility:

```bash
# Check Python version
python3 --version

# If Python 3.10+ is not installed:
sudo apt update
sudo apt install python3.10 python3.10-dev python3.10-venv python3-pip
```

### ROS 2 Humble Hawksbill Installation

ROS 2 Humble Hawksbill is the LTS version required for this curriculum:

```bash
# Add ROS 2 GPG key
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge ros-humble-tf2-py ros-humble-tf2-ros ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager
```

### Additional Dependencies

```bash
# Install additional Python packages
pip3 install rclpy transforms3d numpy matplotlib opencv-python

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Install simulation tools
sudo apt install gazebo libgazebo-dev
```

## Development Environment Setup

### Workspace Creation

Create a ROS 2 workspace for Physical AI development:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build --symlink-install

# Add sourcing to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Python Virtual Environment Setup

For Python-only development and experimentation:

```bash
# Create virtual environment
cd ~/physical_ai_projects
python3 -m venv physical_ai_env
source physical_ai_env/bin/activate

# Install required packages
pip install numpy scipy matplotlib opencv-python transforms3d pygame

# Install ROS 2 Python client library
pip install ros2cli

# Create requirements file
pip freeze > requirements.txt
```

## IDE and Development Tools

### Visual Studio Code Setup

VS Code is recommended for ROS 2 development:

```bash
# Install VS Code
wget -qO - https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# Install ROS 2 extensions
code --install-extension ms-iot.vscode-ros
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
```

### Essential VS Code Extensions

- **ROS**: Provides ROS 2 language support and tools
- **Python**: Official Python extension with IntelliSense
- **C/C++**: For C++ ROS 2 nodes
- **Docker**: For containerized development
- **GitLens**: Enhanced Git capabilities

## Simulation Environment Setup

### Gazebo Installation and Configuration

Gazebo is the standard simulation environment for ROS 2:

```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt install ros-humble-gazebo-* ros-humble-ign-*

# Test Gazebo installation
gz sim --version
```

### URDF and Xacro Setup

For humanoid robot modeling:

```bash
# Install URDF tools
sudo apt install ros-humble-urdf ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# Test URDF tools
ros2 pkg create --build-type ament_python test_urdf --dependencies urdf xacro
```

## Theoretical Foundation: Environment Setup Concepts

Understanding the components and requirements for a Physical AI development environment is crucial for successful project implementation. This theoretical foundation covers the essential elements needed to create a robust development setup.

### Core System Requirements

A Physical AI development environment requires several foundational components:

- **Operating System**: Linux-based systems (Ubuntu 22.04 LTS recommended) provide the necessary compatibility with ROS 2 and simulation tools
- **Python Environment**: Python 3.10+ ensures compatibility with ROS 2 Humble Hawksbill and associated libraries
- **ROS 2 Framework**: Robot Operating System 2 provides the middleware and tools for robotics development
- **Simulation Environment**: Gazebo and related tools for testing and validation without physical hardware

### Development Toolchain Components

The complete development toolchain consists of several interconnected elements:

- **Build System**: Colcon for building ROS 2 packages and workspaces
- **Version Control**: Git with proper configuration for collaborative development
- **IDE Integration**: Code editors with ROS 2 extensions for efficient development
- **Package Management**: Both system-level (apt) and Python-level (pip) package managers

### Validation and Testing Framework

Proper environment validation ensures all components work together:

- **Dependency Verification**: Checking that all required packages and libraries are installed
- **Runtime Environment**: Confirming that ROS 2 environment variables are properly set
- **Basic Functionality Tests**: Simple publisher/subscriber tests to validate communication
- **Simulation Integration**: Ensuring simulation tools can be properly launched and controlled

## Development Workflow Setup

### Git Configuration for ROS Projects

```bash
# Configure Git for ROS development
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
git config --global core.precomposeunicode true

# Set up Git hooks for ROS projects (optional)
# Create a template directory for hooks
mkdir -p ~/.git_template/hooks

# Example pre-commit hook to check ROS package format
cat > ~/.git_template/hooks/pre-commit << 'EOF'
#!/bin/bash
# Pre-commit hook for ROS packages

# Check if any package.xml files have been modified
if git diff --cached --name-only | grep -q "package.xml"; then
    echo "Checking package.xml format..."
    # Add ROS package validation here if needed
fi

exit 0
EOF

chmod +x ~/.git_template/hooks/pre-commit

# Apply template to existing repos
git config --global init.templatedir '~/.git_template'
```

### VS Code Workspace Configuration

Create a `.vscode/settings.json` file for ROS 2 development:

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

## Troubleshooting Common Setup Issues

### 1. ROS 2 Environment Not Sourced

```bash
# Problem: Command 'ros2' not found
# Solution: Source ROS 2 environment
source /opt/ros/humble/setup.bash

# To make permanent, add to ~/.bashrc:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Permission Issues with Gazebo

```bash
# Problem: Gazebo fails to start due to permission issues
# Solution: Check user permissions and reset Gazebo configuration
rm -rf ~/.gazebo
# Restart Gazebo
```

### 3. Python Package Import Errors

```bash
# Problem: ImportError for rclpy or other ROS packages
# Solution: Ensure Python packages are installed in the correct environment
pip3 install -U rclpy

# Or source ROS environment before installing
source /opt/ros/humble/setup.bash
pip3 install rclpy
```

### 4. Colcon Build Failures

```bash
# Problem: colcon build fails with missing dependencies
# Solution: Install missing dependencies
sudo apt update
sudo apt upgrade
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Containerized Development Setup (Optional)

For consistent development environments, consider using Docker:

```dockerfile
# Dockerfile for Physical AI development
FROM osrf/ros:humble-desktop

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    vim \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip
RUN pip3 install numpy scipy matplotlib opencv-python transforms3d

# Set up workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Source ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
```

Build and run the container:

```bash
# Build the container
docker build -t physical-ai-dev .

# Run with access to host X11 for GUI applications
xhost +local:docker
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    --name physical-ai-container \
    physical-ai-dev
```

## Resources for Further Learning

- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [Ubuntu Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop)
- [Python Virtual Environments Guide](https://docs.python.org/3/tutorial/venv.html)
- [VS Code ROS Extension Documentation](https://github.com/ms-iot/vscode-ros)

## Summary

Setting up a proper development environment is crucial for Physical AI development. This involves installing ROS 2 Humble Hawksbill, configuring Python 3.10+, setting up simulation tools like Gazebo, and creating a proper workspace. The validation tests ensure that all components work together correctly before starting development. A well-configured environment will save significant time and avoid common issues during development.