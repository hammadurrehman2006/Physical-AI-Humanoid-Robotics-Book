---
title: لانچ فائلز اور پیرامیٹرز
description: پیچیدہ روبوٹک سسٹم کے لیے ROS 2 لانچ فائلز اور پیرامیٹر مینجمنٹ کو سمجھنا
sidebar_position: 7
---

# لانچ فائلز اور پیرامیٹرز

## سیکھنے کے اہداف
- ROS 2 لانچ فائلز کے مقصد اور ساخت کو سمجھیں
- واحد اور متعدد نوڈ کنفیگریشنز کے لیے لانچ فائلز بنائیں
- پیرامیٹر مینجمنٹ اور کنفیگریشن میں مہارت حاصل کریں
- لانچ آرگومنٹس اور مشروط ایگزیکوشن استعمال کریں
- لانچ کمپوزیشن کے ساتھ پیچیدہ روبوٹک سسٹم کو منظم کریں

## لانچ فائلز کا تعارف

ROS 2 میں لانچ فائلز ایک وقت میں متعدد نوڈس کو مخصوص کنفیگریشنز کے ساتھ شروع کرنے کا طریقہ فراہم کرتی ہیں۔ وہ ROS 1 کے پرانے roslaunch سسٹم کی جگہ لیتی ہیں اور زیادہ لچک اور Python-مبنی اسکرپٹنگ کی صلاحیتیں فراہم کرتی ہیں۔

### لانچ فائلز کیوں استعمال کریں؟

لانچ فائلز ضروری ہیں:
- ایک کمانڈ کے ساتھ متعدد نوڈس شروع کرنے کے لیے
- پیچیدہ سسٹم کنفیگریشنز کا انتظام کرنے کے لیے
- ایک وقت میں متعدد نوڈس کے لیے پیرامیٹرز سیٹ کرنے کے لیے
- نوڈ انحصاریات اور شروع ہونے کے آرڈر کو ہینڈل کرنے کے لیے
- مختلف ماحول کے لیے مختلف کنفیگریشنز کو فعال کرنے کے لیے

## لانچ فائل ساخت

### بنیادی لانچ فائل

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

### پیرامیٹر والی لانچ فائل

```python
# launch/parameterized_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # کنفیگریشن فائل کا پاتھ حاصل کریں
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

## لانچ آرگومنٹس

لانچ آرگومنٹس آپ کو لانچ فائلز میں رن ٹائم پر پیرامیٹرز پاس کرنے کی اجازت دیتے ہیں:

```python
# launch/argument_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # لانچ آرگومنٹس کا اعلان کریں
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

    # لانچ کنفیگریشنز حاصل کریں
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

## اعلی درجے کی لانچ فائل خصوصیات

### مشروط لانچ

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
    # آرگومنٹس کا اعلان کریں
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

    # کنفیگریشنز حاصل کریں
    launch_camera = LaunchConfiguration('launch_camera')
    launch_lidar = LaunchConfiguration('launch_lidar')

    # نوڈس کی تعریف کریں
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

### دیگر لانچ فائلز کو شامل کرنا

```python
# launch/combined_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # دیگر لانچ فائلز شامل کریں
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

## پیرامیٹر مینجمنٹ

### YAML پیرامیٹر فائلز

YAML فائلز پیرامیٹرز کا انتظام کرنے کے لیے ایک منظم طریقہ فراہم کرتی ہیں:

```yaml
# config/robot_controller.yaml
robot_controller:
  ros__parameters:
    # نیویگیشن پیرامیٹرز
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    min_distance_to_obstacle: 0.5
    safety_margin: 0.2

    # کنٹرول پیرامیٹرز
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

    # سینسر پیرامیٹرز
    laser_scan_topic: "/scan"
    camera_topic: "/camera/color/image_raw"
    imu_topic: "/imu/data"

    # برتاؤ پیرامیٹرز
    enable_obstacle_avoidance: true
    enable_path_planning: true
    enable_localization: true
```

### نوڈس میں پیرامیٹرز لوڈ کرنا

```python
# my_robot_package/robot_controller.py
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # ڈیفالٹ ویلیوز کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_distance_to_obstacle', 0.5)
        self.declare_parameter('enable_obstacle_avoidance', True)

        # پیرامیٹر ویلیوز حاصل کریں
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_distance = self.get_parameter('min_distance_to_obstacle').value
        self.enable_avoidance = self.get_parameter('enable_obstacle_avoidance').value

        self.get_logger().info(f'Controller initialized with speed: {self.max_linear_speed}')

    def update_parameters_callback(self, parameter_list):
        """پیرامیٹر اپ ڈیٹس کے لیے کال بیک۔"
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

    # پیرامیٹر کال بیک شامل کریں
    node.add_on_set_parameters_callback(node.update_parameters_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# پیرامیٹر کال بیک کے لیے ضروری درآمد
from rclpy.parameter_service import SetParametersResult
```

### پیرامیٹر کی توثیق

```python
# my_robot_package/validated_controller.py
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterException

class ValidatedController(Node):
    def __init__(self):
        super().__init__('validated_controller')

        # توثیق کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('max_linear_speed', 0.5,
                              rclpy.ParameterDescriptor(
                                  description='Maximum linear speed (m/s)',
                                  floating_point_range=[rclpy.ParameterDescriptor().floating_point_range[0].from_value(0.0),
                                                      rclpy.ParameterDescriptor().floating_point_range[0].to_value(5.0)]))

        self.declare_parameter('robot_name', 'default_robot',
                              rclpy.ParameterDescriptor(
                                  description='Name of the robot',
                                  type=rclpy.ParameterType.PARAMETER_STRING))

        # توثیق کے لیے پیرامیٹر کال بیک سیٹ کریں
        self.add_on_set_parameters_callback(self.validate_parameters)

    def validate_parameters(self, params):
        """پیرامیٹر تبدیلیوں کی توثیق کریں۔"
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

## پیچیدہ لانچ فائل ایکسز

### متعدد روبوٹ لانچ فائل

```python
# launch/multi_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # آرگومنٹس کا اعلان کریں
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

    # ہر روبوٹ کے لیے نوڈس بنائیں
    nodes = []

    # Gazebo سرور اور کلائنٹ
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

    # روبوٹ-مخصوص نوڈس
    for i in range(int(num_robots.perform(None))):
        robot_name = f'robot{i}'

        # روبوٹ اسٹیٹ پبلشر
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

        # روبوٹ کنٹرولر
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

        # نیویگیشن اسٹیک
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

### پیرامیٹر کمپوزیشن لانچ

```python
# launch/parameter_composition_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # لانچ آرگومنٹس کا اعلان کریں
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

    # لانچ کنفیگریشنز حاصل کریں
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    config_file = LaunchConfiguration('config_file')

    # نیمسپیس کے ساتھ ایک گروپ بنائیں
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

## لانچ فائل بہترین مشقیں

### 1. منظم ساخت

```python
# launch/well_structured_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def get_config_file(package_name, config_file):
    """کنفیگ فائل پاتھ حاصل کرنے کے لیے مددگار فنکشن۔"
    return os.path.join(
        get_package_share_directory(package_name),
        'config',
        config_file
    )

def declare_launch_arguments():
    """تمام لانچ آرگومنٹس کا اعلان کریں۔"
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
    """لانچ کے لیے تمام نوڈس بنائیں۔"
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
    """مرکزی لانچ کی وضاحت فنکشن۔"
    # کنفیگریشنز حاصل کریں
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        *declare_launch_arguments(),
        *create_nodes(use_sim_time, robot_name)
    ])
```

### 2. خامات کا انتظام اور توثیق

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
    """لانچ کنفیگریشنز کی توثیق کریں۔"
    # ویلیوز حاصل کریں اور ان کی توثیق کریں
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

        # پیرامیٹر کے مطابق مشروط لاگنگ
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

## ہاتھوں سے مشق: لانچ سسٹم نافذ کاری

متعدد کنفیگریشنز والے روبوٹ کے لیے ایک جامع لانچ سسٹم بنائیں۔

### مشق کی ضروریات
1. مختلف روبوٹ کنفیگریشنز کے لیے لانچ فائلز بنائیں
2. توثیق کے ساتھ پیرامیٹر مینجمنٹ نافذ کریں
3. لچک کے لیے لانچ آرگومنٹس بنائیں
4. مشروط نوڈ لانچ کا مظاہرہ کریں

### مکمل نافذ کاری

```python
#!/usr/bin/env python3
"""
فزیکل AI روبوٹ کے لیے جامع لانچ سسٹم
لانچ فائلز اور پیرامیٹر مینجمنٹ کا مظاہرہ کرتا ہے
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
    # لانچ آرگومنٹس کا اعلان کریں
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

    # لانچ کنفیگریشنز حاصل کریں
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_lidar = LaunchConfiguration('enable_lidar')
    config_file = LaunchConfiguration('config_file')
    startup_delay = LaunchConfiguration('startup_delay')

    # کنفیگریشن فائل پاتھ حاصل کریں
    config_path = os.path.join(
        get_package_share_directory('physical_ai_robot'),
        'config',
        config_file
    )

    # نوڈس کی وضاحت کریں
    nodes = [
        # روبوٹ اسٹیٹ پبلشر
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

        # مرکزی کنٹرولر
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

        # سینسر پروسیسر
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

        # کیمرہ نوڈ (مشروط)
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

        # LIDAR نوڈ (مشروط)
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

        # تشخیص نوڈ (صرف ڈیبگ موڈ میں)
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

    # اگر مخصوص کیا گیا تو اسٹارٹ اپ تاخیر شامل کریں
    if startup_delay != '0.0':
        # تاخیر کے لیے نوڈس کو ٹائمر ایکشن میں لپیٹیں
        delayed_nodes = [
            TimerAction(
                period=startup_delay,
                actions=nodes
            )
        ]
        nodes = delayed_nodes

    # معلوماتی لاگ شامل کریں
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

### کنفیگریشن فائلز

```yaml
# config/robot_config.yaml
physical_ai_robot:
  ros__parameters:
    # روبوٹ کی خصوصیات
    robot_radius: 0.3
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    acceleration_limit: 2.0

    # سینسر پیرامیٹرز
    laser_scan_topic: "/scan"
    camera_topic: "/camera/color/image_raw"
    imu_topic: "/imu/data_raw"
    odometry_topic: "/odom"

    # کنٹرول پیرامیٹرز
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

    # نیویگیشن پیرامیٹرز
    min_distance_to_obstacle: 0.5
    safety_margin: 0.2
    enable_obstacle_avoidance: true
    enable_path_planning: true

    # برتاؤ پیرامیٹرز
    enable_autonomous_mode: true
    enable_manual_control: true
    emergency_stop_distance: 0.3

    # ڈیبگ پیرامیٹرز
    enable_diagnostics: true
    log_level: "info"
```

```yaml
# config/minimal_config.yaml
physical_ai_robot:
  ros__parameters:
    # ٹیسٹنگ کے لیے منیمل کنفیگریشن
    robot_radius: 0.3
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    control_frequency: 10.0

    # منیمل سینسر سیٹ اپ
    laser_scan_topic: "/scan"
    enable_obstacle_avoidance: false
    enable_path_planning: false

    # ڈیبگ پیرامیٹرز
    enable_diagnostics: false
    log_level: "warn"
```

### پیرامیٹر توثیق نوڈ

```python
# physical_ai_robot/parameter_validator.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import InvalidParameterValueException

class ParameterValidator(Node):
    def __init__(self):
        super().__init__('parameter_validator')

        # توثیق کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('min_distance_to_obstacle', 0.5)

        # پیرامیٹر کال بیک سیٹ کریں
        self.add_on_set_parameters_callback(self.validate_parameters)

        self.get_logger().info('Parameter validator initialized')

    def validate_parameters(self, parameters):
        """پیرامیٹر تبدیلیوں کی توثیق کریں۔"
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

## عام لانچ مسائل کا حل

### 1. لانچ فائل نہیں ملی

```bash
# مسئلہ: ros2 launch چلانے پر لانچ فائل نہیں ملی
# حل: فائل لوکیشن اور اجازتیں چیک کریں

# یقینی بنائیں کہ لانچ فائل موجود ہے
find ~/ros2_ws/install -name "*launch.py" | grep your_package

# چیک کریں کہ آیا پیکج مناسب طریقے سے انسٹال ہے
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix your_package

# ضرورت پڑنے پر مکمل پاتھ کے ساتھ چلائیں
python3 /path/to/your/launch/file.py
```

### 2. پیرامیٹر لوڈنگ کے مسائل

```python
# مسئلہ: YAML فائل سے پیرامیٹرز لوڈ نہیں ہو رہے
# حل: فائل پاتھ اور ساخت کی تصدیق کریں

import os
from ament_index_python.packages import get_package_share_directory

def check_config_file(package_name, config_file):
    """چیک کریں کہ آیا کنفیگ فائل موجود ہے اور مناسب طریقے سے ساخت یافتہ ہے۔"
    try:
        config_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            config_file
        )

        if not os.path.exists(config_path):
            print(f"Config file does not exist: {config_path}")
            return False

        # فائل کو پڑھنے کی کوشش کریں
        with open(config_path, 'r') as f:
            content = f.read()
            print(f"Config file exists and is readable: {config_path}")
            return True

    except Exception as e:
        print(f"Error accessing config file: {e}")
        return False
```

### 3. نیمسپیس کے مسائل

```python
# مسئلہ: نیمسپیس کے مسائل کی وجہ سے نوڈس مواصلات نہیں کر رہے
# حل: مناسب نیمسپیس ہینڈلنگ استعمال کریں

from launch.actions import PushRosNamespace
from launch_ros.actions import Node

def create_namespaced_nodes(namespace):
    """مخصوص نیمسپیس میں نوڈس بنائیں۔"
    return [
        PushRosNamespace(namespace),
        Node(
            package='my_package',
            executable='node1',
            name='node1',
            # PushRosNamespace استعمال کرتے ہوئے انفرادی نوڈس کا نیمسپیس کی ضرورت نہیں
        ),
        Node(
            package='my_package',
            executable='node2',
            name='node2',
        )
    ]
```

## کارکردگی کی بہتری کے نکات

### 1. کارکردہ پیرامیٹر لوڈنگ

```python
# متعدد نوڈس کے لیے پیرامیٹر لوڈنگ کو بہتر بنائیں
def create_nodes_with_shared_params(param_file_path):
    """ایک ہی پیرامیٹر فائل شیئر کرنے والے متعدد نوڈس بنائیں۔"
    base_params = [
        {'use_sim_time': True},
        param_file_path  # فائل سے لوڈ کریں
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

### 2. مشروط نوڈ لانچنگ

```python
# وسائل بچانے کے لیے نوڈس کو مشروط طور پر لانچ کریں
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def create_conditional_nodes(enable_feature):
    """نوڈس کو صرف جب فیچر فعال ہو تو بنائیں۔"
    return [
        Node(
            condition=IfCondition(enable_feature),
            package='heavy_pkg',
            executable='heavy_node',
            name='conditional_node'
        )
    ]
```

## مزید سیکھنے کے لیے وسائل

- [ROS 2 لانچ سسٹم دستاویزات](https://docs.ros.org/en/humble/p/launch/)
- [ROS 2 لانچ ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 پیرامیٹرز گائیڈ](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-Python.html)
- [لانچ فائل بہترین مشقیں](https://index.ros.org/doc/ros2/How-To-Guides/Launch-file-different-formats/)

## خلاصہ

لانچ فائلز اور پیرامیٹر مینجمنٹ پیچیدہ ROS 2 سسٹم کو منظم کرنے اور کنفیگر کرنے کے لیے اہم ہیں۔ لانچ فائلز آپ کو ایک وقت میں متعدد نوڈس کو مخصوص کنفیگریشنز کے ساتھ شروع کرنے کی اجازت دیتی ہیں، جبکہ پیرامیٹر مینجمنٹ لچکدار کنفیگریشن کے اختیارات فراہم کرتا ہے۔ مناسب پیرامیٹر توثیق، مشروط ایگزیکوشن، اور خامات کا انتظام کے ساتھ اچھی طرح سے ساخت والی لانچ فائلز بنانا سیکھنا ضروری ہے تاکہ قابل برقرار رکھنے اور قابل توسیع روبوٹک ایپلی کیشنز بنائے جا سکیں۔