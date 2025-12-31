# Isaac AI Robot Brain Documentation

## Overview

The Isaac AI Robot Brain is a complete AI system for humanoid robots using NVIDIA Isaac technology stack. It integrates Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, Nav2 for bipedal navigation, and deep learning models for intelligent behavior.

## Architecture

The system follows a modular architecture with distinct components:

- **Sim Bridge**: Handles communication between Isaac Sim and ROS 2
- **VSLAM**: Visual Simultaneous Localization and Mapping with hardware acceleration
- **Navigation**: Nav2-based path planning for bipedal locomotion
- **Perception**: Multi-sensor fusion and object detection
- **Safety**: Fail-safe mechanisms and constraint management
- **API**: REST API for external system control

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS
- NVIDIA GPU with CUDA support (RTX 4090 or equivalent recommended)
- ROS 2 Humble Hawksbill
- Isaac Sim 2023.1+
- Isaac ROS packages

### Installation

1. Clone the repository
2. Run the setup script:
   ```bash
   ./scripts/setup_dev_environment.sh
   ```

### Running the System

1. Source the ROS environment:
   ```bash
   source /opt/ros/humble/setup.bash
   source ws/install/setup.bash
   ```

2. Start the system:
   ```bash
   ./scripts/run_isaac_robot_brain.sh
   ```

## API Reference

The system exposes a REST API at `http://localhost:8080` with endpoints for:
- Isaac Sim environment management
- VSLAM configuration and control
- Navigation goals and status
- Perception pipeline control
- Safety system management
- System state and health

## Configuration

Configuration files are located in the `config/` directory:
- `sim_bridge_config.yaml` - Isaac Sim bridge settings
- `vslam_config.yaml` - VSLAM algorithm parameters
- `navigation_config.yaml` - Navigation system settings
- `perception_config.yaml` - Perception pipeline parameters
- `safety_config.yaml` - Safety system parameters
- `api_config.yaml` - API server settings

## Contributing

Please read the [Contributing Guide](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.