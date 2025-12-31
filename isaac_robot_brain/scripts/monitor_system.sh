#!/bin/bash

# Isaac Robot Brain System Monitoring Script
# Monitors the system resources and performance

set -e

# Create log directory if it doesn't exist
mkdir -p /tmp/isaac_robot_brain_monitoring

# Monitoring parameters
INTERVAL=${1:-5}  # Default to 5 seconds
LOG_FILE="/tmp/isaac_robot_brain_monitoring/system_monitor.log"

echo "Starting Isaac Robot Brain system monitoring..."
echo "Monitoring interval: ${INTERVAL}s"
echo "Log file: ${LOG_FILE}"

# Function to log system metrics
log_metrics() {
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

    # CPU usage
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')

    # Memory usage
    MEM_INFO=$(free -m | awk 'NR==2{printf "%.2f%%", $3*100/$2 }')

    # GPU usage (if available)
    if command -v nvidia-smi &> /dev/null; then
        GPU_USAGE=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits)
        GPU_MEM=$(nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits)
    else
        GPU_USAGE="N/A"
        GPU_MEM="N/A"
    fi

    # Disk usage
    DISK_USAGE=$(df -h / | awk 'NR==2{print $5}')

    # ROS nodes status (if ROS is running)
    if source /opt/ros/humble/setup.bash 2>/dev/null; then
        NODES=$(ros2 node list 2>/dev/null | wc -l)
        TOPICS=$(ros2 topic list 2>/dev/null | wc -l)
    else
        NODES="N/A"
        TOPICS="N/A"
    fi

    # Log the metrics
    echo "${TIMESTAMP} | CPU: ${CPU_USAGE}% | MEM: ${MEM_INFO} | GPU: ${GPU_USAGE}% | DISK: ${DISK_USAGE} | Nodes: ${NODES} | Topics: ${TOPICS}" | tee -a "${LOG_FILE}"
}

# Main monitoring loop
while true; do
    log_metrics
    sleep "${INTERVAL}"
done