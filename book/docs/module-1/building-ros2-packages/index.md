---
title: Building ROS 2 Packages
description: Understanding how to create and structure ROS 2 packages for robotic applications
sidebar_position: 6
---

# Building ROS 2 Packages

## Learning Objectives
- Understand the structure and components of ROS 2 packages
- Learn how to create packages using ros2 pkg create
- Master the package.xml manifest file configuration
- Organize code following ROS 2 best practices
- Build and install packages using colcon
- Create reusable and maintainable packages

## Introduction to ROS 2 Packages

A ROS 2 package is the fundamental unit of organization in ROS 2. It contains the source code, dependencies, configuration files, and metadata needed to build and run a specific functionality. Packages provide modularity, reusability, and maintainability for robotic applications.

### Package Structure Overview

A typical ROS 2 package follows this structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── src/                    # Source code files
│   ├── main.cpp           # C++ source files
│   └── nodes/             # Node implementations
├── include/                # C++ header files
├── scripts/                # Executable scripts
├── launch/                 # Launch files
├── config/                 # Configuration files
├── test/                   # Test files
├── msg/                    # Custom message definitions
├── srv/                    # Custom service definitions
├── action/                 # Custom action definitions
├── setup.py                # Python package configuration
├── setup.cfg               # Python installation configuration
└── ros2 pkg create         # Package creation command
```

## Creating a New Package

### Using ros2 pkg create Command

The `ros2 pkg create` command creates a new package with the basic structure:

```bash
# Create a Python package
ros2 pkg create --build-type ament_python my_robot_controller

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_robot_driver

# Create a package with dependencies
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs my_robot_controller
```

### Package Creation Example

```bash
# Navigate to your workspace source directory
cd ~/ros2_ws/src

# Create a Python-based robot controller package
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs my_robot_controller

# Create a C++-based robot driver package
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs sensor_msgs my_robot_driver
```

## Package.xml Manifest File

The `package.xml` file contains metadata about the package including name, version, description, maintainers, license, and dependencies.

### Basic package.xml Structure

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>0.0.0</version>
  <description>Robot controller package for my_robot</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### package.xml with Dependencies

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>1.0.0</version>
  <description>Advanced robot controller with navigation capabilities</description>
  <maintainer email="developer@company.com">Robotics Team</maintainer>
  <license>MIT</license>

  <!-- Dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>message_runtime</depend>

  <!-- Build dependencies -->
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Execution dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Python Package Structure

For Python-based packages, the structure differs slightly to follow Python packaging conventions.

### Python Package Example

```bash
# After creating the package, you'll see this structure:
my_robot_controller/
├── package.xml
├── setup.py
├── setup.cfg
├── my_robot_controller/
│   ├── __init__.py
│   ├── robot_controller.py
│   └── utils/
│       ├── __init__.py
│       └── helper_functions.py
└── test/
    └── test_copyright.py
```

### setup.py Configuration

```python
from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot controller package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_controller.robot_controller:main',
            'robot_monitor = my_robot_controller.robot_monitor:main',
        ],
    },
)
```

### Python Node Implementation

```python
# my_robot_controller/robot_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        """Handle laser scan data."""
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} readings')

    def control_loop(self):
        """Main control loop."""
        # Implement robot control logic here
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd_vel.angular.z = 0.0  # No rotation

        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## C++ Package Structure

For C++ packages, you'll have CMake-based build configuration.

### C++ package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_driver</name>
  <version>1.0.0</version>
  <description>C++ robot driver package</description>
  <maintainer email="developer@company.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt Configuration

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(robot_driver src/robot_driver.cpp)

# Link libraries
ament_target_dependencies(robot_driver
  rclcpp
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  robot_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Package Organization Best Practices

### 1. Logical Structure

Organize your package with a clear logical structure:

```bash
# Good organization example
my_robot_navigation/
├── package.xml
├── CMakeLists.txt (or setup.py for Python)
├── src/
│   ├── controllers/
│   │   ├── base_controller.cpp
│   │   └── path_planner.cpp
│   ├── sensors/
│   │   ├── lidar_processor.cpp
│   │   └── camera_processor.cpp
│   └── nodes/
│       ├── navigation_node.cpp
│       └── sensor_fusion_node.cpp
├── include/my_robot_navigation/
│   ├── controllers/
│   │   ├── base_controller.hpp
│   │   └── path_planner.hpp
│   └── sensors/
│       ├── lidar_processor.hpp
│       └── camera_processor.hpp
├── launch/
│   ├── navigation.launch.py
│   └── sensor.launch.py
├── config/
│   ├── robot_params.yaml
│   └── navigation_config.yaml
├── test/
│   ├── test_navigation.cpp
│   └── test_sensor_fusion.cpp
├── msg/
│   └── CustomMessage.msg
└── srv/
    └── CustomService.srv
```

### 2. Launch Files

Create launch files to easily start multiple nodes:

```python
# launch/navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_navigation'),
        'config',
        'navigation_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_navigation',
            executable='path_planner',
            name='path_planner',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='my_robot_navigation',
            executable='localization_node',
            name='localization_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='my_robot_navigation',
            executable='move_base',
            name='move_base',
            parameters=[config],
            output='screen'
        )
    ])
```

### 3. Configuration Files

Use YAML files for configuration:

```yaml
# config/robot_params.yaml
my_robot_controller:
  ros__parameters:
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    safety_distance: 0.5
    control_frequency: 10.0
    enable_obstacle_avoidance: true
    robot_radius: 0.3
```

## Building Packages with Colcon

### Basic Colcon Commands

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build specific package
colcon build --packages-select my_robot_controller

# Build with symlinks (faster rebuilds)
colcon build --packages-select my_robot_controller --symlink-install

# Build all packages
colcon build

# Build with specific options
colcon build --packages-select my_robot_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Advanced Colcon Usage

```bash
# Build with parallel jobs
colcon build --parallel-workers 4

# Build and run tests
colcon build --packages-select my_robot_controller
colcon test --packages-select my_robot_controller
colcon test-result --all

# Clean build artifacts
rm -rf build/ install/ log/

# Build with specific build type
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build only Python packages
colcon build --packages-select my_robot_controller --event-handlers console_cohesion+

# Install to custom location
colcon build --install-base /opt/my_robot
```

## Creating Custom Message Types

### Defining Custom Messages

Create custom message types in the `msg/` directory:

```# msg/RobotState.msg
# Custom message for robot state information
float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
bool is_moving
string status
```

```# srv/MoveRobot.srv
# Custom service for robot movement
float64 target_x
float64 target_y
float64 target_theta
---
bool success
string message
```

```# action/NavigateToPose.action
# Custom action for navigation
geometry_msgs/PoseStamped target_pose
---
geometry_msgs/PoseStamped final_pose
string message
---
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
int32 waypoints_completed
```

### Using Custom Messages in Code

```python
# In your Python node
from my_robot_msgs.msg import RobotState
from my_robot_msgs.srv import MoveRobot
from my_robot_msgs.action import NavigateToPose

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher with custom message
        self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)

        # Service server with custom service
        self.move_srv = self.create_service(
            MoveRobot, 'move_robot', self.move_robot_callback)

    def publish_robot_state(self, x, y, theta):
        """Publish robot state using custom message."""
        state_msg = RobotState()
        state_msg.x = x
        state_msg.y = y
        state_msg.theta = theta
        state_msg.is_moving = True
        state_msg.status = 'moving'

        self.state_pub.publish(state_msg)

    def move_robot_callback(self, request, response):
        """Handle move robot service request."""
        # Implement movement logic
        response.success = True
        response.message = f'Moving to ({request.target_x}, {request.target_y})'
        return response
```

## Hands-on Exercise: Complete Package Development

Create a complete ROS 2 package from scratch that demonstrates all concepts.

### Exercise Requirements
1. Create a new package with proper structure
2. Implement a node with publishers, subscribers, and services
3. Define custom message types
4. Create launch files and configuration
5. Build and test the package

### Example ROS2 Packages

In the Physical AI & Humanoid Robotics Book project, we provide several example ROS2 packages that demonstrate key concepts. These packages are located in the `book/src/ros2-packages/` directory:

#### Publisher-Subscriber Package (py_pubsub)

This package demonstrates the basic publisher-subscriber pattern:

```bash
# Package location
book/src/ros2-packages/py_pubsub/

# Files included:
# - package.xml: Package metadata and dependencies
# - setup.py: Python package configuration
# - py_pubsub/publisher_member_function.py: Publisher node implementation
# - py_pubsub/subscriber_member_function.py: Subscriber node implementation
```

To run the publisher and subscriber nodes:

```bash
# Terminal 1 - Start the publisher
cd book/src/ros2-packages/py_pubsub
python3 -m py_pubsub.publisher_member_function

# Terminal 2 - Start the subscriber
cd book/src/ros2-packages/py_pubsub
python3 -m py_pubsub.subscriber_member_function
```

#### Service-Client Package (py_srv_client)

This package demonstrates the service-client pattern:

```bash
# Package location
book/src/ros2-packages/py_srv_client/

# Files included:
# - package.xml: Package metadata and dependencies
# - setup.py: Python package configuration
# - py_srv_client/service_member_function.py: Service server implementation
# - py_srv_client/client_member_function.py: Service client implementation
```

To run the service server and client:

```bash
# Terminal 1 - Start the service server
cd book/src/ros2-packages/py_srv_client
python3 -m py_srv_client.service_member_function

# Terminal 2 - Run the client
cd book/src/ros2-packages/py_srv_client
python3 -m py_srv_client.client_member_function
```

#### Action Server-Client Package (py_action_server)

This package demonstrates the action server-client pattern:

```bash
# Package location
book/src/ros2-packages/py_action_server/

# Files included:
# - package.xml: Package metadata and dependencies
# - setup.py: Python package configuration
# - py_action_server/fibonacci_action_server.py: Action server implementation
# - py_action_server/fibonacci_action_client.py: Action client implementation
```

To run the action server and client:

```bash
# Terminal 1 - Start the action server
cd book/src/ros2-packages/py_action_server
python3 -m py_action_server.fibonacci_action_server

# Terminal 2 - Run the action client
cd book/src/ros2-packages/py_action_server
python3 -m py_action_server.fibonacci_action_client
```

### Creating Your Own Package

To create your own ROS2 package, you can use the `ros2 pkg create` command:

```bash
# Create a Python-based package
ros2 pkg create --build-type ament_python my_robot_package

# Create a C++-based package
ros2 pkg create --build-type ament_cmake my_robot_driver

# Create a package with dependencies
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs my_robot_controller
```

### Package Structure Template

After creating a package, you'll have this basic structure:

```bash
my_robot_package/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package configuration (for Python packages)
├── setup.cfg            # Installation configuration
├── my_robot_package/    # Python module directory
│   ├── __init__.py
│   └── robot_node.py    # Your node implementation
└── test/                # Test files
    └── test_copyright.py
```

### Complete Package Creation Example

Here's a complete example of creating a physical AI demo package:

```bash
#!/bin/bash
# create_physical_ai_package.sh

echo "Creating Physical AI Package..."

# Create the package
ros2 pkg create --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs \
  physical_ai_demo

cd physical_ai_demo

# Create directory structure
mkdir -p launch config msg srv test

# Create custom message
cat > msg/PhysicalState.msg << 'EOF'
# Physical state of the robot
float64 position_x
float64 position_y
float64 position_z
float64 orientation_x
float64 orientation_y
float64 orientation_z
float64 orientation_w
float64 linear_velocity
float64 angular_velocity
bool is_stable
string environment_type
EOF

# Create custom service
cat > srv/PhysicalInteraction.srv << 'EOF'
# Request for physical interaction
string interaction_type
float64 force_magnitude
float64[] target_position
---
bool success
string message
float64 actual_force_applied
EOF

# Update package.xml with custom message dependencies
sed -i '/<build_type>ament_python<\/build_type>/i \
  <depend>message_runtime</depend>' package.xml

# Create main Python module
mkdir -p physical_ai_demo
touch physical_ai_demo/__init__.py

# Create the main node implementation
cat > physical_ai_demo/physical_controller.py << 'PYTHON_EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from .msg import PhysicalState
from .srv import PhysicalInteraction

class PhysicalAIController(Node):
    def __init__(self):
        super().__init__('physical_ai_controller')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(PhysicalState, 'physical_state', 10)
        self.status_pub = self.create_publisher(String, 'controller_status', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Services
        self.interaction_srv = self.create_service(
            PhysicalInteraction, 'physical_interaction',
            self.interaction_callback)

        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.state_timer = self.create_timer(0.5, self.publish_state)

        # Internal state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.is_stable = True

        self.get_logger().info('Physical AI Controller initialized')

    def scan_callback(self, msg):
        """Handle laser scan data."""
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r > 0], default=float('inf'))
            if min_range < self.get_parameter('safety_distance').value:
                self.is_stable = False
                self.get_logger().warn(f'Obstacle detected at {min_range:.2f}m')
            else:
                self.is_stable = True

    def control_loop(self):
        """Main control loop."""
        cmd = Twist()

        # Simple control logic
        if self.is_stable:
            cmd.linear.x = self.get_parameter('max_linear_speed').value
            cmd.angular.z = 0.0
            status = "Moving safely"
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            status = "Stopped - obstacle detected"

        # Publish command
        self.cmd_pub.publish(cmd)

        # Publish status
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def publish_state(self):
        """Publish physical state."""
        state_msg = PhysicalState()
        state_msg.position_x = self.current_position[0]
        state_msg.position_y = self.current_position[1]
        state_msg.position_z = self.current_position[2]
        state_msg.orientation_x = self.current_orientation[0]
        state_msg.orientation_y = self.current_orientation[1]
        state_msg.orientation_z = self.current_orientation[2]
        state_msg.orientation_w = self.current_orientation[3]
        state_msg.is_stable = self.is_stable
        state_msg.environment_type = "indoor"

        self.state_pub.publish(state_msg)

    def interaction_callback(self, request, response):
        """Handle physical interaction requests."""
        self.get_logger().info(f'Received interaction: {request.interaction_type}')

        # Simulate interaction
        response.success = True
        response.message = f'Completed {request.interaction_type} interaction'
        response.actual_force_applied = request.force_magnitude * 0.9  # Simulate efficiency

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAIController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

# Create launch file
cat > launch/physical_ai_demo.launch.py << 'LAUNCH_EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get config file path
    config = os.path.join(
        get_package_share_directory('physical_ai_demo'),
        'config',
        'controller_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='physical_ai_demo',
            executable='physical_controller',
            name='physical_ai_controller',
            parameters=[config] if os.path.exists(config) else [],
            output='screen'
        )
    ])
LAUNCH_EOF

# Create configuration file
cat > config/controller_config.yaml << 'CONFIG_EOF'
physical_ai_controller:
  ros__parameters:
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    safety_distance: 0.8
    control_frequency: 10.0
CONFIG_EOF

# Update setup.py
cat > setup.py << 'SETUP_EOF'
from setuptools import find_packages, setup

package_name = 'physical_ai_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/physical_ai_demo.launch.py']),
        ('share/' + package_name + '/config', ['config/controller_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Team',
    maintainer_email='physical-ai@example.com',
    description='Demo package for Physical AI concepts',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'physical_controller = physical_ai_demo.physical_controller:main',
        ],
    },
)
SETUP_EOF

# Create setup.cfg
cat > setup.cfg << 'SETUP_CFG_EOF'
[develop]
script-dir=$base/lib/physical_ai_demo
[install]
install-scripts=$base/lib/physical_ai_demo
SETUP_CFG_EOF

echo "Package structure created successfully!"
echo "Directory structure:"
tree .
echo ""
echo "To build this package, run:"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select physical_ai_demo"
echo "  source install/setup.bash"
echo "  ros2 run physical_ai_demo physical_controller"
echo ""
echo "To launch with launch file:"
echo "  ros2 launch physical_ai_demo physical_ai_demo.launch.py"
echo ""
echo "To test the service:"
echo "  ros2 service call /physical_interaction physical_ai_demo/srv/PhysicalInteraction \"{interaction_type: 'push', force_magnitude: 10.0, target_position: [1.0, 2.0, 0.0]}\""
```

## Testing and Quality Assurance

### Unit Testing

Create comprehensive tests for your packages:

```python
# test/test_physical_controller.py
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from physical_ai_demo.physical_controller import PhysicalAIController

class TestPhysicalAIController(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = PhysicalAIController()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_initialization(self):
        """Test that the node initializes correctly."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'physical_ai_controller')

    def test_publishers_created(self):
        """Test that all required publishers are created."""
        # Check if publishers exist (they're created as attributes)
        self.assertTrue(hasattr(self.node, 'cmd_pub'))
        self.assertTrue(hasattr(self.node, 'state_pub'))
        self.assertTrue(hasattr(self.node, 'status_pub'))

    def test_parameters_declared(self):
        """Test that required parameters are declared."""
        params = self.node.get_parameters([
            'max_linear_speed',
            'max_angular_speed',
            'safety_distance'
        ])

        for param_name, param_value in params.items():
            self.assertIsNotNone(param_value.value)

if __name__ == '__main__':
    unittest.main()
```

### Linting and Code Quality

ROS 2 provides tools for code quality checking:

```bash
# Run code quality checks
ament_copyright --verbose src/my_robot_package
ament_flake8 src/my_robot_package
ament_pep257 src/my_robot_package

# Or use ament_lint_auto for all checks
ament_lint_auto src/my_robot_package
```

## Troubleshooting Common Package Issues

### 1. Build Errors

```bash
# Problem: Package doesn't build
# Solution: Check dependencies and build configuration

# Check for missing dependencies
rosdep check --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select my_package --event-handlers console_direct+

# Check build logs
find build/my_package -name "*.log" -exec echo "=== {} ===" \; -exec cat {} \;
```

### 2. Import Errors

```bash
# Problem: Python modules not found
# Solution: Check setup.py and PYTHONPATH

# Ensure package is installed
cd ~/ros2_ws
source install/setup.bash
python3 -c "import my_robot_package"  # Should not raise ImportError
```

### 3. Package Not Found

```bash
# Problem: ros2 run or ros2 launch can't find package
# Solution: Check installation and environment

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Check if package is found
ros2 pkg list | grep my_package

# Check package path
ros2 pkg prefix my_package
```

## Resources for Further Learning

- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Package.xml Format](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [Colcon Build Tool](https://colcon.readthedocs.io/en/released/)
- [ROS 2 Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## Summary

Building ROS 2 packages requires understanding of package structure, dependency management, build systems (CMake for C++, setuptools for Python), and proper organization. A well-structured package includes proper metadata in package.xml, organized source code, configuration files, launch files, and comprehensive testing. Following ROS 2 best practices for package development ensures maintainable, reusable, and robust robotic applications.