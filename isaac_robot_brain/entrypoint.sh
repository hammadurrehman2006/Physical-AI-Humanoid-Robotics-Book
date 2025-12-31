#!/bin/bash
set -e

# Source ROS environment
source "/opt/ros/humble/setup.bash"
source "/ws/install/setup.bash"

# Execute the command passed as arguments
exec "$@"