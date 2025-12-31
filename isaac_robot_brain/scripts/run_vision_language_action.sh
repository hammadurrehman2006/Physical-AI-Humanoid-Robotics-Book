#!/bin/bash
# run_vision_language_action.sh
# Main script to run the Vision-Language-Action module

set -e  # Exit on any error

echo "Starting Vision-Language-Action module..."

# Check if required environment variables are set
if [ -z "$OPENAI_API_KEY" ]; then
    echo "Error: OPENAI_API_KEY environment variable not set."
    echo "Please set it by running: export OPENAI_API_KEY='your-api-key-here'"
    exit 1
fi

# Activate ROS 2 environment if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Humble environment activated."
fi

# Activate Python virtual environment if available
if [ -d "venv" ]; then
    source venv/bin/activate
    echo "Python virtual environment activated."
fi

# Run the main Vision-Language-Action ROS 2 node
echo "Launching Vision-Language-Action system..."
python3 -m isaac_robot_brain.src.main

echo "Vision-Language-Action module started successfully!"
echo "Listening for voice commands..."