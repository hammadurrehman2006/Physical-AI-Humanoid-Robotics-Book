#!/bin/bash

# Isaac Robot Brain Run Script
# This script runs the Isaac Robot Brain system

set -e  # Exit on any error

echo "Starting Isaac Robot Brain system..."

# Check if workspace is built
if [ ! -d "ws/install" ]; then
    echo "Error: Workspace not built. Please build the workspace first."
    echo "Run: cd ws && colcon build --packages-select isaac_robot_brain"
    exit 1
fi

# Source ROS environment
source /opt/ros/humble/setup.bash
source ws/install/setup.bash

# Start the Isaac Robot Brain API server
echo "Starting Isaac Robot Brain API server..."
cd ws
ros2 launch isaac_robot_brain isaac_robot_brain.launch.py &
API_PID=$!

# Wait a moment for the server to start
sleep 3

# Check if the server started successfully
if kill -0 $API_PID 2>/dev/null; then
    echo "Isaac Robot Brain system started successfully!"
    echo "API server running with PID: $API_PID"
    echo "Press Ctrl+C to stop the system"

    # Wait for the process to finish
    wait $API_PID
else
    echo "Failed to start Isaac Robot Brain system"
    exit 1
fi