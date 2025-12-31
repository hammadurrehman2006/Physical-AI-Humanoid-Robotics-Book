#!/bin/bash

# Isaac Robot Brain Development Environment Setup Script
# This script sets up the development environment for the Isaac AI Robot Brain project

set -e  # Exit on any error

echo "Setting up Isaac Robot Brain development environment..."

# Check if running on the correct OS
if [[ ! -f /etc/os-release ]]; then
    echo "Error: /etc/os-release not found. This script only works on Linux."
    exit 1
fi

. /etc/os-release
if [[ "$ID" != "ubuntu" ]] || [[ "$VERSION_ID" != "22.04" ]]; then
    echo "Warning: This script is designed for Ubuntu 22.04. You are running $NAME $VERSION_ID"
fi

# Check for NVIDIA GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo "Warning: nvidia-smi not found. Make sure you have an NVIDIA GPU and drivers installed."
else
    GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader,nounits)
    echo "Found GPU: $GPU_INFO"
fi

# Install system dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    curl \
    wget \
    vim \
    tmux \
    htop \
    cmake \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-dev-tools

# Install ROS 2 Humble if not already installed
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "Installing ROS 2 Humble..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop
    sudo apt install -y ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-tf2-eigen
else
    echo "ROS 2 Humble already installed."
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --user -U \
    opencv-python \
    opencv-contrib-python \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
    numpy scipy matplotlib \
    pyyaml \
    requests \
    flask \
    fastapi uvicorn \
    pytest \
    black \
    flake8

# Source ROS environment
source /opt/ros/humble/setup.bash

# Create workspace if it doesn't exist
if [ ! -d "ws" ]; then
    mkdir -p ws/src
    echo "Created workspace at $(pwd)/ws"
fi

# Copy the isaac_robot_brain package to workspace if not already there
if [ ! -d "ws/src/isaac_robot_brain" ]; then
    cp -r src/isaac_robot_brain ws/src/isaac_robot_brain
    echo "Copied isaac_robot_brain package to workspace"
fi

# Install package dependencies
cd ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace
colcon build --packages-select isaac_robot_brain

# Return to main directory
cd ..

echo "Development environment setup complete!"
echo ""
echo "To start working:"
echo "1. Source the ROS environment: source /opt/ros/humble/setup.bash"
echo "2. Source the workspace: source ws/install/setup.bash"
echo "3. Run: ros2 run isaac_robot_brain isaac_robot_brain_node (when available)"
echo ""
echo "For Isaac Sim integration, make sure Isaac Sim is installed and running."
echo "For hardware acceleration, ensure CUDA and appropriate drivers are installed."