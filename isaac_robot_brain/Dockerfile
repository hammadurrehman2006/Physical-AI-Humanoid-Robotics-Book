# Use NVIDIA Isaac ROS as base image
FROM nvcr.io/nvidia/isaac-ros:galactic-ros-base-humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install opencv-python opencv-contrib-python && \
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 && \
    pip3 install numpy scipy matplotlib

# Set up ROS workspace
WORKDIR /ws
COPY isaac_robot_brain/src/isaac_robot_brain /ws/src/isaac_robot_brain

# Install package dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select isaac_robot_brain

# Source ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]