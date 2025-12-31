#!/bin/bash

# Isaac Robot Brain Setup Script
# This script sets up the development environment for the Isaac Robot Brain project

set -e  # Exit on any error

echo "Isaac Robot Brain Setup Script"
echo "==============================="

# Check if running in a ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not detected!"
    echo "Please source your ROS 2 installation first."
    echo "For example: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS 2 distribution detected: $ROS_DISTRO"

# Create the ROS 2 package if it doesn't exist
if [ ! -d "isaac_robot_brain" ]; then
    echo "Creating ROS 2 package: isaac_robot_brain"
    ros2 pkg create --build-type ament_python isaac_robot_brain --dependencies rclpy std_msgs sensor_msgs geometry_msgs
else
    echo "ROS 2 package already exists"
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Create necessary directories if they don't exist
mkdir -p src/voice_processing src/language_understanding src/action_execution src/multi_modal_fusion
mkdir -p tests

# Create __init__.py files if they don't exist
touch src/__init__.py
touch src/voice_processing/__init__.py
touch src/language_understanding/__init__.py
touch src/action_execution/__init__.py
touch src/multi_modal_fusion/__init__.py

echo "Setup completed successfully!"
echo ""
echo "To run tests: pytest tests/"
echo "To run the main node: ros2 run isaac_robot_brain main_node"