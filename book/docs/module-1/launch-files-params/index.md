---
title: Launch Files and Parameters
description: Understanding ROS 2 launch files and parameter management for complex robotic systems
sidebar_position: 7
---

# Launch Files and Parameters

## Learning Objectives
- Understand the purpose and structure of ROS 2 launch files
- Create launch files for single and multiple node configurations
- Master parameter management and configuration
- Use launch arguments and conditional execution
- Organize complex robotic systems with launch composition

## Introduction to Launch Files

Launch files in ROS 2 provide a way to start multiple nodes with specific configurations simultaneously. They replace the older roslaunch system from ROS 1 and offer more flexibility and Python-based scripting capabilities.

### Why Use Launch Files?

Launch files are essential for:
- Starting multiple nodes with a single command
- Managing complex system configurations
- Setting parameters for multiple nodes at once
- Handling node dependencies and startup order
- Enabling different configurations for various environments

## Launch File Structure

### Basic Launch File

```python
# launch/basic_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node'
        )
    ])
```

### Launch File with Parameters

```python
# launch/parameterized_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to configuration file
    config = os.path.join(
        get_package_share_directory('my_robot_package'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=[config],
            remappings=[
                ('/original_topic', '/remapped_topic')
            ]
        )
    ])
```

## Launch Arguments

Launch arguments allow you to pass parameters to launch files at runtime:

```python
# launch/argument_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,

        Node(
            package='my_robot_package',
            executable='robot_controller',
            name=['robot_controller_', robot_name],
            parameters=[{'use_sim_time': use_sim_time}],
            namespace=robot_name
        )
    ])
```

## Advanced Launch File Features

### Conditional Launch

```python
# launch/conditional_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch camera node'
    )

    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Launch LIDAR node'
    )

    # Get configurations
    launch_camera = LaunchConfiguration('launch_camera')
    launch_lidar = LaunchConfiguration('launch_lidar')

    # Define nodes
    camera_node = Node(
        condition=IfCondition(launch_camera),
        package='image_proc',
        executable='image_proc',
        name='camera_node'
    )

    lidar_node = Node(
        condition=IfCondition(launch_lidar),
        package='velodyne_driver',
        executable='velodyne_node',
        name='lidar_node'
    )

    return LaunchDescription([
        launch_camera_arg,
        launch_lidar_arg,
        camera_node,
        lidar_node
    ])
```

### Including Other Launch Files

```python
# launch/combined_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include other launch files
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/localization_launch.py'
        ])
    )

    return LaunchDescription([
        navigation_launch,
        localization_launch
    ])
```

## Parameter Management

### YAML Parameter Files

YAML files provide a structured way to manage parameters:

```yaml
# config/robot_controller.yaml
robot_controller:
  ros__parameters:
    # Navigation parameters
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    min_distance_to_obstacle: 0.5
    safety_margin: 0.2

    # Control parameters
    control_frequency: 50.0
    enable_pid_control: true
    pid_gains:
      linear:
        kp: 1.0
        ki: 0.1
        kd: 0.05
      angular:
        kp: 2.0
        ki: 0.2
        kd: 0.1

    # Sensor parameters
    laser_scan_topic: "/scan"
    camera_topic: "/camera/color/image_raw"
    imu_topic: "/imu/data"

    # Behavior parameters
    enable_obstacle_avoidance: true
    enable_path_planning: true
    enable_localization: true
```

### Loading Parameters in Nodes

```python
# my_robot_package/robot_controller.py
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters with default values
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_distance_to_obstacle', 0.5)
        self.declare_parameter('enable_obstacle_avoidance', True)

        # Get parameter values
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_distance = self.get_parameter('min_distance_to_obstacle').value
        self.enable_avoidance = self.get_parameter('enable_obstacle_avoidance').value

        self.get_logger().info(f'Controller initialized with speed: {self.max_linear_speed}')

    def update_parameters_callback(self, parameter_list):
        """Callback for parameter updates."""
        for param in parameter_list:
            if param.name == 'max_linear_speed':
                self.max_linear_speed = param.value
                self.get_logger().info(f'Updated max_linear_speed to {self.max_linear_speed}')
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = param.value
                self.get_logger().info(f'Updated max_angular_speed to {self.max_angular_speed}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    # Add parameter callback
    node.add_on_set_parameters_callback(node.update_parameters_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Import needed for parameter callback
from rclpy.parameter_service import SetParametersResult
```

### Parameter Validation

```python
# my_robot_package/validated_controller.py
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterException

class ValidatedController(Node):
    def __init__(self):
        super().__init__('validated_controller')

        # Declare parameters with validation
        self.declare_parameter('max_linear_speed', 0.5,
                              rclpy.ParameterDescriptor(
                                  description='Maximum linear speed (m/s)',
                                  floating_point_range=[rclpy.ParameterDescriptor().floating_point_range[0].from_value(0.0),
                                                      rclpy.ParameterDescriptor().floating_point_range[0].to_value(5.0)]))

        self.declare_parameter('robot_name', 'default_robot',
                              rclpy.ParameterDescriptor(
                                  description='Name of the robot',
                                  type=rclpy.ParameterType.PARAMETER_STRING))

        # Set parameter callback for validation
        self.add_on_set_parameters_callback(self.validate_parameters)

    def validate_parameters(self, params):
        """Validate parameter changes."""
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == 'max_linear_speed':
                if not (0.0 <= param.value <= 5.0):
                    result.successful = False
                    result.reason = f'max_linear_speed must be between 0.0 and 5.0, got {param.value}'
                    return result
            elif param.name == 'robot_name':
                if not isinstance(param.value, str) or len(param.value.strip()) == 0:
                    result.successful = False
                    result.reason = f'robot_name must be a non-empty string, got {param.value}'
                    return result

        return result
```

## Complex Launch File Examples

### Multi-Robot Launch File

```python
# launch/multi_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to launch'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='small_room',
        description='Name of the Gazebo world to load'
    )

    num_robots = LaunchConfiguration('num_robots')
    world_name = LaunchConfiguration('world_name')

    # Create nodes for each robot
    nodes = []

    # Gazebo server and client
    nodes.append(
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=[f'/usr/share/gazebo-11/worlds/{world_name}.world'],
            output='screen'
        )
    )

    nodes.append(
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        )
    )

    # Robot-specific nodes
    for i in range(int(num_robots.perform(None))):
        robot_name = f'robot{i}'

        # Robot state publisher
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'robot_state_publisher_{robot_name}',
                namespace=robot_name,
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': f'$(find my_robot_description)/urdf/{robot_name}.urdf'
                }]
            )
        )

        # Robot controller
        nodes.append(
            Node(
                package='my_robot_controller',
                executable='robot_controller',
                name=f'controller_{robot_name}',
                namespace=robot_name,
                parameters=[{
                    'use_sim_time': True,
                    'robot_name': robot_name
                }]
            )
        )

        # Navigation stack
        nodes.append(
            Node(
                package='nav2_bringup',
                executable='nav2_bringup',
                name=f'navigation_{robot_name}',
                namespace=robot_name,
                parameters=[{
                    'use_sim_time': True,
                    'bt_xml_filename': 'navigate_w_replanning_and_recovery.xml'
                }]
            )
        )

    return LaunchDescription([
        num_robots_arg,
        world_name_arg,
    ] + nodes)
```

### Parameter Composition Launch

```python
# launch/parameter_composition_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Robot namespace'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    config_file = LaunchConfiguration('config_file')

    # Create a group with namespace
    namespaced_nodes = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),

            Node(
                package='my_robot_driver',
                executable='motor_controller',
                name='motor_controller',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'motor_pwm_frequency': 20000},
                    {'max_rpm': 3000}
                ]
            ),

            Node(
                package='my_robot_sensors',
                executable='imu_processor',
                name='imu_processor',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'imu_rate': 100},
                    {'calibration_offset': [0.0, 0.0, 0.0]}
                ]
            ),

            Node(
                package='my_robot_localization',
                executable='ekf_localization',
                name='ekf_localization',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'frequency': 50.0},
                    {'sensor_timeout': 1.0}
                ]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_namespace_arg,
        config_file_arg,
        namespaced_nodes
    ])
```

## Launch File Best Practices

### 1. Organized Structure

```python
# launch/well_structured_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def get_config_file(package_name, config_file):
    """Helper function to get config file path."""
    return os.path.join(
        get_package_share_directory(package_name),
        'config',
        config_file
    )

def declare_launch_arguments():
    """Declare all launch arguments."""
    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='my_robot',
            description='Name of the robot'
        )
    ]

def create_nodes(use_sim_time_config, robot_name_config):
    """Create all nodes for the launch."""
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time_config},
                {'robot_description': get_config_file('my_robot_description', 'robot.urdf')}
            ]
        ),
        Node(
            package='my_robot_controller',
            executable='controller',
            name='controller',
            namespace=robot_name_config,
            parameters=[
                {'use_sim_time': use_sim_time_config},
                get_config_file('my_robot_controller', 'controller.yaml')
            ]
        )
    ]

def generate_launch_description():
    """Main launch description function."""
    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        *declare_launch_arguments(),
        *create_nodes(use_sim_time, robot_name)
    ])
```

### 2. Error Handling and Validation

```python
# launch/validated_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

def validate_launch_configurations(context: LaunchContext):
    """Validate launch configurations."""
    # Get values and validate them
    try:
        robot_name = perform_substitutions(context, [LaunchConfiguration('robot_name')])
        if not robot_name or robot_name.strip() == '':
            raise ValueError("robot_name cannot be empty")

        context.launch_configurations['validated_robot_name'] = robot_name
        return [LogInfo(msg=f'Validated robot name: {robot_name}')]
    except Exception as e:
        return [LogInfo(msg=f'Validation error: {str(e)}')]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='default_robot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Enable detailed logging'
        ),

        # Conditional logging based on parameter
        LogInfo(
            condition=IfCondition(LaunchConfiguration('enable_logging')),
            msg='Launch configuration validation enabled'
        ),

        Node(
            package='my_robot_package',
            executable='robot_node',
            name=LaunchConfiguration('robot_name'),
            parameters=[
                {'robot_name': LaunchConfiguration('robot_name')}
            ]
        )
    ])
```

## Hands-on Exercise: Launch System Implementation

Create a comprehensive launch system for a robot with multiple configurations.

### Exercise Requirements
1. Create launch files for different robot configurations
2. Implement parameter management with validation
3. Create launch arguments for flexibility
4. Demonstrate conditional node launching

### Complete Implementation

```python
#!/usr/bin/env python3
"""
Comprehensive Launch System for Physical AI Robot
Demonstrates launch files and parameter management
"""

# launch/physical_ai_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='physical_ai_robot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable camera node'
        ),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Enable LIDAR node'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='robot_config.yaml',
            description='Configuration file name'
        ),
        DeclareLaunchArgument(
            'startup_delay',
            default_value='0.0',
            description='Delay before starting nodes (seconds)'
        )
    ]

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_lidar = LaunchConfiguration('enable_lidar')
    config_file = LaunchConfiguration('config_file')
    startup_delay = LaunchConfiguration('startup_delay')

    # Get configuration file path
    config_path = os.path.join(
        get_package_share_directory('physical_ai_robot'),
        'config',
        config_file
    )

    # Define nodes
    nodes = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': os.path.join(
                    get_package_share_directory('physical_ai_description'),
                    'urdf',
                    'physical_ai_robot.urdf'
                )}
            ],
            condition=IfCondition(
                PythonExpression(["'", config_file, "' != 'minimal_config.yaml'"])
            )
        ),

        # Main controller
        Node(
            package='physical_ai_robot',
            executable='robot_controller',
            name='robot_controller',
            namespace=robot_name,
            parameters=[
                {'use_sim_time': use_sim_time},
                config_path
            ]
        ),

        # Sensor processor
        Node(
            package='physical_ai_robot',
            executable='sensor_processor',
            name='sensor_processor',
            namespace=robot_name,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'sensor_processing_rate': 30.0}
            ]
        ),

        # Camera node (conditional)
        Node(
            condition=IfCondition(enable_camera),
            package='image_proc',
            executable='image_proc',
            name='camera_node',
            namespace=robot_name,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'camera_rate': 15.0}
            ]
        ),

        # LIDAR node (conditional)
        Node(
            condition=IfCondition(enable_lidar),
            package='velodyne_driver',
            executable='velodyne_node',
            name='lidar_node',
            namespace=robot_name,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'scan_rate': 10.0}
            ]
        ),

        # Diagnostics node (only in debug mode)
        Node(
            condition=IfCondition(
                PythonExpression(["'", config_file, "'.endswith('debug.yaml')"])
            ),
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            namespace=robot_name
        )
    ]

    # Add startup delay if specified
    if startup_delay != '0.0':
        # Wrap nodes in TimerAction for delay
        delayed_nodes = [
            TimerAction(
                period=startup_delay,
                actions=nodes
            )
        ]
        nodes = delayed_nodes

    # Add informational log
    info_log = LogInfo(
        msg=["Starting Physical AI Robot with parameters: ",
             "robot_name: ", robot_name, ", ",
             "use_sim_time: ", use_sim_time, ", ",
             "enable_camera: ", enable_camera, ", ",
             "enable_lidar: ", enable_lidar]
    )

    return LaunchDescription([
        *launch_args,
        info_log,
        *nodes
    ])
```

### Configuration Files

```yaml
# config/robot_config.yaml
physical_ai_robot:
  ros__parameters:
    # Robot specifications
    robot_radius: 0.3
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    acceleration_limit: 2.0

    # Sensor parameters
    laser_scan_topic: "/scan"
    camera_topic: "/camera/color/image_raw"
    imu_topic: "/imu/data_raw"
    odometry_topic: "/odom"

    # Control parameters
    control_frequency: 50.0
    enable_pid_control: true
    pid_gains:
      linear:
        kp: 1.0
        ki: 0.1
        kd: 0.05
      angular:
        kp: 2.0
        ki: 0.2
        kd: 0.1

    # Navigation parameters
    min_distance_to_obstacle: 0.5
    safety_margin: 0.2
    enable_obstacle_avoidance: true
    enable_path_planning: true

    # Behavior parameters
    enable_autonomous_mode: true
    enable_manual_control: true
    emergency_stop_distance: 0.3

    # Debug parameters
    enable_diagnostics: true
    log_level: "info"
```

```yaml
# config/minimal_config.yaml
physical_ai_robot:
  ros__parameters:
    # Minimal configuration for testing
    robot_radius: 0.3
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    control_frequency: 10.0

    # Minimal sensor setup
    laser_scan_topic: "/scan"
    enable_obstacle_avoidance: false
    enable_path_planning: false

    # Debug parameters
    enable_diagnostics: false
    log_level: "warn"
```

### Parameter Validation Node

```python
# physical_ai_robot/parameter_validator.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import InvalidParameterValueException

class ParameterValidator(Node):
    def __init__(self):
        super().__init__('parameter_validator')

        # Declare parameters with validation
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('min_distance_to_obstacle', 0.5)

        # Set parameter callback
        self.add_on_set_parameters_callback(self.validate_parameters)

        self.get_logger().info('Parameter validator initialized')

    def validate_parameters(self, parameters):
        """Validate parameter changes."""
        from rcl_interfaces.msg import SetParametersResult

        result = SetParametersResult()
        result.successful = True

        for param in parameters:
            if param.name == 'robot_radius':
                if not (0.1 <= param.value <= 2.0):
                    result.successful = False
                    result.reason = f'robot_radius must be between 0.1 and 2.0, got {param.value}'
                    return result
            elif param.name == 'max_linear_speed':
                if not (0.0 <= param.value <= 5.0):
                    result.successful = False
                    result.reason = f'max_linear_speed must be between 0.0 and 5.0, got {param.value}'
                    return result
            elif param.name == 'max_angular_speed':
                if not (0.0 <= param.value <= 5.0):
                    result.successful = False
                    result.reason = f'max_angular_speed must be between 0.0 and 5.0, got {param.value}'
                    return result
            elif param.name == 'control_frequency':
                if not (1.0 <= param.value <= 200.0):
                    result.successful = False
                    result.reason = f'control_frequency must be between 1.0 and 200.0, got {param.value}'
                    return result
            elif param.name == 'min_distance_to_obstacle':
                if not (0.1 <= param.value <= 5.0):
                    result.successful = False
                    result.reason = f'min_distance_to_obstacle must be between 0.1 and 5.0, got {param.value}'
                    return result

        return result

def main(args=None):
    rclpy.init(args=args)
    node = ParameterValidator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Troubleshooting Common Launch Issues

### 1. Launch File Not Found

```bash
# Problem: Launch file not found when running ros2 launch
# Solution: Check file location and permissions

# Verify the launch file exists
find ~/ros2_ws/install -name "*launch.py" | grep your_package

# Check if the package is properly installed
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix your_package

# Run with full path if needed
python3 /path/to/your/launch/file.py
```

### 2. Parameter Loading Issues

```python
# Problem: Parameters not loading from YAML file
# Solution: Verify file path and structure

import os
from ament_index_python.packages import get_package_share_directory

def check_config_file(package_name, config_file):
    """Check if config file exists and is properly formatted."""
    try:
        config_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            config_file
        )

        if not os.path.exists(config_path):
            print(f"Config file does not exist: {config_path}")
            return False

        # Try to read the file
        with open(config_path, 'r') as f:
            content = f.read()
            print(f"Config file exists and is readable: {config_path}")
            return True

    except Exception as e:
        print(f"Error accessing config file: {e}")
        return False
```

### 3. Namespace Issues

```python
# Problem: Nodes not communicating due to namespace issues
# Solution: Use proper namespace handling

from launch.actions import PushRosNamespace
from launch_ros.actions import Node

def create_namespaced_nodes(namespace):
    """Create nodes within a specific namespace."""
    return [
        PushRosNamespace(namespace),
        Node(
            package='my_package',
            executable='node1',
            name='node1',
            # No need to namespace individual nodes when using PushRosNamespace
        ),
        Node(
            package='my_package',
            executable='node2',
            name='node2',
        )
    ]
```

## Performance Optimization Tips

### 1. Efficient Parameter Loading

```python
# Optimize parameter loading for multiple nodes
def create_nodes_with_shared_params(param_file_path):
    """Create multiple nodes sharing the same parameter file."""
    base_params = [
        {'use_sim_time': True},
        param_file_path  # Load from file
    ]

    return [
        Node(
            package='pkg1',
            executable='node1',
            parameters=base_params + [{'specific_param': 'value1'}]
        ),
        Node(
            package='pkg2',
            executable='node2',
            parameters=base_params + [{'specific_param': 'value2'}]
        )
    ]
```

### 2. Conditional Node Launching

```python
# Launch nodes conditionally to save resources
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def create_conditional_nodes(enable_feature):
    """Create nodes only when feature is enabled."""
    return [
        Node(
            condition=IfCondition(enable_feature),
            package='heavy_pkg',
            executable='heavy_node',
            name='conditional_node'
        )
    ]
```

## Resources for Further Learning

- [ROS 2 Launch System Documentation](https://docs.ros.org/en/humble/p/launch/)
- [ROS 2 Launch Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-Python.html)
- [Launch File Best Practices](https://index.ros.org/doc/ros2/How-To-Guides/Launch-file-different-formats/)

## Summary

Launch files and parameter management are crucial for organizing and configuring complex ROS 2 systems. Launch files allow you to start multiple nodes with specific configurations simultaneously, while parameter management provides flexible configuration options. Understanding how to create well-structured launch files with proper parameter validation, conditional execution, and error handling is essential for building maintainable and scalable robotic applications.