# Isaac Sim Environment Setup

This section covers the setup and configuration of the NVIDIA Isaac Sim environment for humanoid robotics development. We'll walk through the installation process, system configuration, and initial setup steps.

## System Preparation

Before installing Isaac Sim, ensure your system is properly prepared:

### Update System Packages
```bash
sudo apt update && sudo apt upgrade -y
```

### Install Essential Dependencies
```bash
sudo apt install build-essential cmake git python3-dev python3-pip python3-venv
```

### Verify GPU and Driver Installation
```bash
nvidia-smi
```

You should see your NVIDIA GPU listed with the driver version. Isaac Sim requires a modern NVIDIA GPU with the latest drivers installed.

## Isaac Sim Installation Options

Isaac Sim can be installed in several ways depending on your needs:

### Option 1: Isaac Sim Omniverse App (Recommended for beginners)

1. **Download Isaac Sim**:
   - Visit the NVIDIA Developer website
   - Download the Isaac Sim Omniverse app
   - This provides a complete, pre-configured environment

2. **Install via Omniverse Launcher**:
   - Install the Omniverse Launcher
   - Use the launcher to install Isaac Sim
   - The launcher handles dependencies and updates

### Option 2: Isaac Sim Docker (Recommended for development)

This option provides the most control and is ideal for development work:

1. **Install Docker** (if not already installed):
```bash
sudo apt install docker.io
sudo usermod -aG docker $USER
```
   - Log out and log back in for group changes to take effect

2. **Install NVIDIA Container Toolkit**:
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker
```

3. **Pull Isaac Sim Docker Image**:
```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Option 3: Isaac Sim via Isaac ROS Development Environment

For tight integration with ROS 2:

1. **Install ROS 2 Humble Hawksbill** (if not already installed):
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

2. **Install Isaac ROS packages**:
```bash
sudo apt install ros-humble-isaac-ros-* ros-humble-nitros-*
```

## Docker-Based Installation (Recommended)

For this course, we recommend the Docker-based installation as it provides:

- Isolated environment
- Consistent development experience
- Easy version management
- Clean uninstallation

### Setting up Isaac Sim with Docker

1. **Create a project directory**:
```bash
mkdir ~/isaac_sim_project
cd ~/isaac_sim_project
```

2. **Create a docker-compose.yml file**:
```yaml
version: '3.8'
services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:2023.1.1
    environment:
      - "NVIDIA_VISIBLE_DEVICES=all"
      - "NVIDIA_DRIVER_CAPABILITIES=all"
      - "QT_X11_NO_MITSHM=1"
      - "DISPLAY=${DISPLAY}"
      - "XAUTHORITY=/tmp/.docker.xauth"
    volumes:
      - /tmp/.X11-unix:/tmp/.x11-unix:rw
      - /tmp/.docker.xauth:/tmp/.docker.xauth:rw
      - ./workspace:/workspace:rw
      - ./isaac-sim-cache:/isaac-sim/cache:rw
    network_mode: host
    stdin_open: true
    tty: true
    runtime: nvidia
    privileged: true
```

3. **Start Isaac Sim**:
```bash
xhost +local:docker
docker-compose up -d
```

## Verification and Initial Configuration

### Testing the Installation

1. **Launch Isaac Sim** (if using Docker):
```bash
docker exec -it isaac_sim_project_isaac-sim_1 bash
```

2. **Run a basic test**:
```bash
cd /isaac-sim
python3 -c "import omni; print('Isaac Sim Python API is working!')"
```

### Initial Configuration

1. **Set up your workspace**:
```bash
mkdir -p ~/isaac_sim_workspace/{scenes,robots,scripts,configs}
```

2. **Configure environment variables** (add to ~/.bashrc):
```bash
export ISAAC_SIM_PATH=/isaac-sim  # or path to your Isaac Sim installation
export NVIDIA_OMNIVERSE_LOGGING_LEVEL=0
export ISAACSIM_PYTHON_EXE=/isaac-sim/python.sh
```

3. **Create a basic test script** to verify everything works:
```python
#!/usr/bin/env python3
"""
Basic Isaac Sim test script
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Initialize the world
my_world = World(stage_units_in_meters=1.0)

# Add a simple robot (Franka Panda in this example)
assets_root_path = get_assets_root_path()
if assets_root_path is not None:
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/Franka"
    )

# Reset and step the world
my_world.reset()
for i in range(100):
    my_world.step(render=True)

print("Isaac Sim test completed successfully!")
```

Save this as `test_isaac_sim.py` in your workspace.

## Troubleshooting Common Issues

### GPU Access Issues
- Ensure your user is in the docker group
- Verify nvidia-container-toolkit is installed and configured
- Check that the NVIDIA driver is properly installed

### Display Issues
- Make sure X11 forwarding is properly configured
- For headless systems, consider using VNC or similar solutions
- Verify DISPLAY environment variable is set correctly

### Memory Issues
- Isaac Sim is memory-intensive; ensure you have sufficient RAM
- Monitor GPU memory usage with `nvidia-smi`
- Consider reducing simulation complexity if running low on memory

## Next Steps

Once your Isaac Sim environment is properly set up, you can:

1. Explore the built-in examples and tutorials
2. Create your first simulation scene
3. Configure robot models for humanoid robotics
4. Set up sensor configurations for perception tasks

The next section will cover creating your first photorealistic simulation environment with humanoid robots.