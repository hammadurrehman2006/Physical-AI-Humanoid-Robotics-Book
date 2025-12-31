# Quickstart Guide: Vision-Language-Action Module

## Overview
This guide provides a step-by-step introduction to setting up and running the Vision-Language-Action module for humanoid robotics. This module integrates voice processing, language understanding, computer vision, and action execution to enable natural human-robot interaction.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- Python 3.10+ installed
- ROS 2 Humble Hawksbill installed
- NVIDIA GPU with CUDA 11.8+ (for optimal performance)
- At least 16GB RAM (32GB recommended)
- At least 50GB free disk space

### Software Dependencies
```bash
# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install Python dependencies
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install openai-whisper
pip install openai
pip install opencv-python
pip install numpy scipy
pip install rclpy
```

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book
git checkout 001-vision-language-action
```

### 2. Setup ROS 2 Workspace
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p isaac_robot_brain/src
cd isaac_robot_brain

# Build the workspace
colcon build
source install/setup.bash
```

### 3. Install Module Dependencies
```bash
cd isaac_robot_brain
pip install -r requirements.txt
```

### 4. Configure API Keys
Create a `.env` file in the `isaac_robot_brain` directory:
```env
OPENAI_API_KEY=your_openai_api_key_here
WHISPER_MODEL_SIZE=medium  # Options: tiny, base, small, medium, large
CUDA_DEVICE=0  # GPU device ID (0 for first GPU)
```

## Running the Module

### 1. Start ROS 2 Nodes
```bash
# Terminal 1: Start the main system
cd isaac_robot_brain
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision_language_action system.launch.py
```

### 2. Run Voice Processing Component
```bash
# Terminal 2: Start voice processing
cd isaac_robot_brain
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vision_language_action voice_processor_node
```

### 3. Run Vision Processing Component
```bash
# Terminal 3: Start vision processing
cd isaac_robot_brain
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vision_language_action vision_processor_node
```

### 4. Run Action Execution Component
```bash
# Terminal 4: Start action execution
cd isaac_robot_brain
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vision_language_action action_executor_node
```

## Basic Usage Example

### Simple Voice Command
1. Ensure all components are running
2. Speak a command like "Pick up the red ball"
3. Observe the system:
   - Voice processing: Audio captured and transcribed
   - Language understanding: Command interpreted and parsed
   - Vision processing: Red ball identified in environment
   - Action execution: Robot moves to pick up the ball

### Multi-Step Command
Try a more complex command like "Go to the kitchen, find the blue cup, and bring it to me"

## Testing the System

### Unit Tests
```bash
cd isaac_robot_brain
python -m pytest tests/voice_tests.py
python -m pytest tests/language_tests.py
python -m pytest tests/action_tests.py
```

### Integration Test
```bash
cd isaac_robot_brain
python -m pytest tests/integration_tests.py
```

## Docusaurus Documentation

### Start Documentation Server
```bash
cd book
npm install
npm run start
```

### Module Documentation Structure
The Vision-Language-Action module documentation is organized in phases:

1. **Phase 1**: Voice Processing Setup
2. **Phase 2**: Language Understanding Integration
3. **Phase 3**: Action Execution Development
4. **Phase 4**: Multi-Modal Fusion Implementation
5. **Phase 5**: Autonomous Task Execution

## Troubleshooting

### Common Issues

1. **Audio Input Not Working**
   - Check microphone permissions
   - Verify audio device configuration with `arecord -l`
   - Test with `arecord -D hw:0,0 -f cd test.wav`

2. **GPU Memory Issues**
   - Reduce model size in configuration
   - Close other GPU-intensive applications
   - Check available memory with `nvidia-smi`

3. **ROS 2 Communication Issues**
   - Verify all nodes are on the same ROS domain
   - Check network configuration if running distributed
   - Use `ros2 node list` to verify node discovery

### Performance Tuning

1. **Voice Processing**
   - Use smaller Whisper models for faster processing
   - Adjust confidence thresholds in configuration
   - Optimize audio preprocessing pipeline

2. **Vision Processing**
   - Use TensorRT optimization for NVIDIA GPUs
   - Adjust detection confidence thresholds
   - Optimize image resolution for your use case

3. **Action Execution**
   - Tune action server timeouts
   - Optimize path planning parameters
   - Adjust motion control gains

## Next Steps

1. Complete the detailed tutorials in the Docusaurus documentation
2. Experiment with custom voice commands
3. Fine-tune the system for your specific robot platform
4. Extend the module with additional capabilities