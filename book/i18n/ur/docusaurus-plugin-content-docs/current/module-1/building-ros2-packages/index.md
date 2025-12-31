---
title: ROS 2 پیکجز بنانا
description: روبوٹک ایپلی کیشنز کے لیے ROS 2 پیکجز کو تخلیق اور ساخت دینا سمجھنا
sidebar_position: 6
---

# ROS 2 پیکجز بنانا

## سیکھنے کے اہداف
- ROS 2 پیکجز کی ساخت اور اجزاء کو سمجھیں
- ros2 pkg create کا استعمال کرتے ہوئے پیکجز بنانا سیکھیں
- package.xml مینیفیسٹ فائل کنفیگریشن میں مہارت حاصل کریں
- ROS 2 کی بہترین مشقیں کے مطابق کوڈ کو منظم کریں
- colcon کا استعمال کرتے ہوئے پیکجز کو بنائیں اور انسٹال کریں
- دوبارہ استعمال کے قابل اور قابل برقرار رکھنے والے پیکجز تیار کریں

## ROS 2 پیکجز کا تعارف

ایک ROS 2 پیکج ROS 2 میں تنظیم کی بنیادی اکائی ہے۔ یہ سورس کوڈ، انحصاریات، کنفیگریشن فائلز، اور میٹا ڈیٹا کو مشتمل ہوتا ہے جس کی ضرورت ایک مخصوص فعالیت کو بنانے اور چلانے کے لیے ہوتی ہے۔ پیکجز روبوٹک ایپلی کیشنز کے لیے ماڈولرٹی، دوبارہ استعمال کے قابلیت، اور قابل برقرار رکھنے کی خصوصیت فراہم کرتے ہیں۔

### پیکج ساخت کا جائزہ

ایک عام ROS 2 پیکج اس ساخت کو فالو کرتا ہے:

```
my_robot_package/
├── CMakeLists.txt          # C++ کے لیے بلڈ کنفیگریشن
├── package.xml             # پیکج میٹا ڈیٹا اور انحصاریات
├── src/                    # سورس کوڈ فائلز
│   ├── main.cpp           # C++ سورس فائلز
│   └── nodes/             # نوڈ نافذ کاریاں
├── include/                # C++ ہیڈر فائلز
├── scripts/                # ایگزیکوٹیبل اسکرپٹس
├── launch/                 # لانچ فائلز
├── config/                 # کنفیگریشن فائلز
├── test/                   # ٹیسٹ فائلز
├── msg/                    # کسٹم میسج کی تعریفیں
├── srv/                    # کسٹم سروس کی تعریفیں
├── action/                 # کسٹم ایکشن کی تعریفیں
├── setup.py                # Python پیکج کنفیگریشن
├── setup.cfg               # Python انسٹالیشن کنفیگریشن
└── ros2 pkg create         # پیکج تخلیق کمانڈ
```

## ایک نیا پیکج بنانا

### ros2 pkg create کمانڈ کا استعمال

`ros2 pkg create` کمانڈ بنیادی ساخت کے ساتھ ایک نیا پیکج بناتی ہے:

```bash
# ایک Python پیکج بنائیں
ros2 pkg create --build-type ament_python my_robot_controller

# ایک C++ پیکج بنائیں
ros2 pkg create --build-type ament_cmake my_robot_driver

# انحصاریات کے ساتھ ایک پیکج بنائیں
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs my_robot_controller
```

### پیکج تخلیق کی مثال

```bash
# اپنے ورک سپیس سورس ڈائریکٹری میں جائیں
cd ~/ros2_ws/src

# Python-مبنی روبوٹ کنٹرولر پیکج بنائیں
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs my_robot_controller

# C++-مبنی روبوٹ ڈرائیور پیکج بنائیں
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs sensor_msgs my_robot_driver
```

## package.xml مینیفیسٹ فائل

`package.xml` فائل پیکج کے بارے میں میٹا ڈیٹا رکھتی ہے بشمول نام، ورژن، تفصیل، مینٹینرز، لائسنس، اور انحصاریات۔

### بنیادی package.xml ساخت

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

### انحصاریات کے ساتھ package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>1.0.0</version>
  <description>Advanced robot controller with navigation capabilities</description>
  <maintainer email="developer@company.com">Robotics Team</maintainer>
  <license>MIT</license>

  <!-- انحصاریات -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>message_runtime</depend>

  <!-- بلڈ انحصاریات -->
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- ایگزیکوشن انحصاریات -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <!-- ٹیسٹ انحصاریات -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Python پیکج ساخت

Python-مبنی پیکجز کے لیے، ساخت ہلکے سے مختلف ہوتی ہے تاکہ Python پیکجنگ کے رواج کو فالو کیا جا سکے۔

### Python پیکج کی مثال

```bash
# پیکج بنانے کے بعد، آپ کو یہ ساخت نظر آئے گی:
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

### setup.py کنفیگریشن

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

### Python نوڈ نافذ کاری

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

        # پبلشرز
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # سبسکرائبزرز
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # پیرامیٹرز
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)

        # ٹائمرز
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        """لیزر اسکین ڈیٹا کو ہینڈل کریں۔"
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} readings')

    def control_loop(self):
        """مرکزی کنٹرول لوپ۔"
        # یہاں روبوٹ کنٹرول لاگک نافذ کریں
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5  # 0.5 میٹر/سیکنڈ پر آگے بڑھیں
        cmd_vel.angular.z = 0.0  # کوئی گردش نہیں

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

## C++ پیکج ساخت

C++ پیکجز کے لیے، آپ کو CMake-مبنی بلڈ کنفیگریشن ہوگی۔

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

### CMakeLists.txt کنفیگریشن

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# انحصاریات تلاش کریں
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# انکلود ڈائریکٹریز
include_directories(include)

# ایگزیکوٹیبل شامل کریں
add_executable(robot_driver src/robot_driver.cpp)

# لائبریریز لنک کریں
ament_target_dependencies(robot_driver
  rclcpp
  std_msgs
  sensor_msgs
)

# ٹارگٹس انسٹال کریں
install(TARGETS
  robot_driver
  DESTINATION lib/${PROJECT_NAME}
)

# دیگر فائلز انسٹال کریں
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

## پیکج تنظیم کی بہترین مشقیں

### 1. منطقی ساخت

اپنے پیکج کو ایک صاف منطقی ساخت کے ساتھ منظم کریں:

```bash
# اچھی تنظیم کی مثال
my_robot_navigation/
├── package.xml
├── CMakeLists.txt (یا Python کے لیے setup.py)
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

### 2. لانچ فائلز

متعدد نوڈس کو آسانی سے شروع کرنے کے لیے لانچ فائلز بنائیں:

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

### 3. کنفیگریشن فائلز

کنفیگریشن کے لیے YAML فائلز استعمال کریں:

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

## Colcon کے ساتھ پیکجز بنانا

### بنیادی Colcon کمانڈز

```bash
# ورک سپیس کے روٹ میں جائیں
cd ~/ros2_ws

# ROS 2 ماحول سورس کریں
source /opt/ros/humble/setup.bash

# مخصوص پیکج بنائیں
colcon build --packages-select my_robot_controller

# سیم لنکس کے ساتھ بنائیں (تیز ری بیلڈز)
colcon build --packages-select my_robot_controller --symlink-install

# تمام پیکجز بنائیں
colcon build

# مخصوص اختیارات کے ساتھ بنائیں
colcon build --packages-select my_robot_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### اعلی درجے کا Colcon استعمال

```bash
# متوازی جابز کے ساتھ بنائیں
colcon build --parallel-workers 4

# بنائیں اور ٹیسٹ چلائیں
colcon build --packages-select my_robot_controller
colcon test --packages-select my_robot_controller
colcon test-result --all

# بلڈ آرٹیفیکٹس صاف کریں
rm -rf build/ install/ log/

# مخصوص بلڈ ٹائپ کے ساتھ بنائیں
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# صرف Python پیکجز بنائیں
colcon build --packages-select my_robot_controller --event-handlers console_cohesion+

# حسب ضرورت لوکیشن پر انسٹال کریں
colcon build --install-base /opt/my_robot
```

## کسٹم میسج ٹائپس بنانا

### کسٹم میسجز کی تعریف

`msg/` ڈائریکٹری میں کسٹم میسج ٹائپس بنائیں:

```# msg/RobotState.msg
# روبوٹ اسٹیٹ کی معلومات کے لیے کسٹم میسج
float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
bool is_moving
string status
```

```# srv/MoveRobot.srv
# روبوٹ حرکت کے لیے کسٹم سروس
float64 target_x
float64 target_y
float64 target_theta
---
bool success
string message
```

```# action/NavigateToPose.action
# نیویگیشن کے لیے کسٹم ایکشن
geometry_msgs/PoseStamped target_pose
---
geometry_msgs/PoseStamped final_pose
string message
---
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
int32 waypoints_completed
```

### کوڈ میں کسٹم میسجز استعمال کرنا

```python
# آپ کے Python نوڈ میں
from my_robot_msgs.msg import RobotState
from my_robot_msgs.srv import MoveRobot
from my_robot_msgs.action import NavigateToPose

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # کسٹم میسج کے ساتھ پبلشر
        self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)

        # کسٹم سروس کے ساتھ سروس سرور
        self.move_srv = self.create_service(
            MoveRobot, 'move_robot', self.move_robot_callback)

    def publish_robot_state(self, x, y, theta):
        """کسٹم میسج کا استعمال کرتے ہوئے روبوٹ اسٹیٹ شائع کریں۔"
        state_msg = RobotState()
        state_msg.x = x
        state_msg.y = y
        state_msg.theta = theta
        state_msg.is_moving = True
        state_msg.status = 'moving'

        self.state_pub.publish(state_msg)

    def move_robot_callback(self, request, response):
        """روبوٹ چلنے کی سروس کی درخواست کو ہینڈل کریں۔"
        # حرکت کا لاگک نافذ کریں
        response.success = True
        response.message = f'Moving to ({request.target_x}, {request.target_y})'
        return response
```

## ہاتھوں سے مشق: مکمل پیکج ڈیولپمنٹ

تمام تصورات کو ظاہر کرنے والا ایک مکمل ROS 2 پیکج صفر سے بنائیں۔

### مشق کی ضروریات
1. مناسب ساخت کے ساتھ ایک نیا پیکج بنائیں
2. پبلشرز، سبسکرائبزرز، اور سروسز کے ساتھ ایک نوڈ نافذ کریں
3. کسٹم میسج ٹائپس کی تعریف کریں
4. لانچ فائلز اور کنفیگریشن بنائیں
5. پیکج کو بنائیں اور ٹیسٹ کریں

### ROS2 پیکجز کی مثالیں

فزیکل AI اور ہیومنوائڈ روبوٹکس کتاب منصوبے میں، ہم کچھ مثال ROS2 پیکجز فراہم کرتے ہیں جو کلیدی تصورات کو ظاہر کرتے ہیں۔ یہ پیکجز `book/src/ros2-packages/` ڈائریکٹری میں واقع ہیں:

#### پبلشر-سبسکرائب پیکج (py_pubsub)

یہ پیکج بنیادی پبلشر-سبسکرائب نمونہ کو ظاہر کرتا ہے:

```bash
# پیکج لوکیشن
book/src/ros2-packages/py_pubsub/

# شامل فائلز:
# - package.xml: پیکج میٹا ڈیٹا اور انحصاریات
# - setup.py: Python پیکج کنفیگریشن
# - py_pubsub/publisher_member_function.py: پبلشر نوڈ نافذ کاری
# - py_pubsub/subscriber_member_function.py: سبسکرائبر نوڈ نافذ کاری
```

پبلشر اور سبسکرائبر نوڈس چلانے کے لیے:

```bash
# ٹرمنل 1 - پبلشر شروع کریں
cd book/src/ros2-packages/py_pubsub
python3 -m py_pubsub.publisher_member_function

# ٹرمنل 2 - سبسکرائبر شروع کریں
cd book/src/ros2-packages/py_pubsub
python3 -m py_pubsub.subscriber_member_function
```

#### سروس-کلائنٹ پیکج (py_srv_client)

یہ پیکج سروس-کلائنٹ نمونہ کو ظاہر کرتا ہے:

```bash
# پیکج لوکیشن
book/src/ros2-packages/py_srv_client/

# شامل فائلز:
# - package.xml: پیکج میٹا ڈیٹا اور انحصاریات
# - setup.py: Python پیکج کنفیگریشن
# - py_srv_client/service_member_function.py: سروس سرور نافذ کاری
# - py_srv_client/client_member_function.py: سروس کلائنٹ نافذ کاری
```

سروس سرور اور کلائنٹ چلانے کے لیے:

```bash
# ٹرمنل 1 - سروس سرور شروع کریں
cd book/src/ros2-packages/py_srv_client
python3 -m py_srv_client.service_member_function

# ٹرمنل 2 - کلائنٹ چلائیں
cd book/src/ros2-packages/py_srv_client
python3 -m py_srv_client.client_member_function
```

#### ایکشن سرور-کلائنٹ پیکج (py_action_server)

یہ پیکج ایکشن سرور-کلائنٹ نمونہ کو ظاہر کرتا ہے:

```bash
# پیکج لوکیشن
book/src/ros2-packages/py_action_server/

# شامل فائلز:
# - package.xml: پیکج میٹا ڈیٹا اور انحصاریات
# - setup.py: Python پیکج کنفیگریشن
# - py_action_server/fibonacci_action_server.py: ایکشن سرور نافذ کاری
# - py_action_server/fibonacci_action_client.py: ایکشن کلائنٹ نافذ کاری
```

ایکشن سرور اور کلائنٹ چلانے کے لیے:

```bash
# ٹرمنل 1 - ایکشن سرور شروع کریں
cd book/src/ros2-packages/py_action_server
python3 -m py_action_server.fibonacci_action_server

# ٹرمنل 2 - ایکشن کلائنٹ چلائیں
cd book/src/ros2-packages/py_action_server
python3 -m py_action_server.fibonacci_action_client
```

### اپنا پیکج بنانا

اپنا ROS2 پیکج بنانے کے لیے، آپ `ros2 pkg create` کمانڈ استعمال کر سکتے ہیں:

```bash
# ایک Python-مبنی پیکج بنائیں
ros2 pkg create --build-type ament_python my_robot_package

# ایک C++-مبنی پیکج بنائیں
ros2 pkg create --build-type ament_cmake my_robot_driver

# انحصاریات کے ساتھ ایک پیکج بنائیں
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs my_robot_controller
```

### پیکج ساخت ٹیمپلیٹ

ایک پیکج بنانے کے بعد، آپ کو یہ بنیادی ساخت ملے گی:

```bash
my_robot_package/
├── package.xml          # پیکج میٹا ڈیٹا اور انحصاریات
├── setup.py             # Python پیکج کنفیگریشن (Python پیکجز کے لیے)
├── setup.cfg            # انسٹالیشن کنفیگریشن
├── my_robot_package/    # Python ماڈیول ڈائریکٹری
│   ├── __init__.py
│   └── robot_node.py    # آپ کی نوڈ نافذ کاری
└── test/                # ٹیسٹ فائلز
    └── test_copyright.py
```

### مکمل پیکج تخلیق کی مثال

یہاں ایک فزیکل AI ڈیمو پیکج بنانے کی ایک مکمل مثال ہے:

```bash
#!/bin/bash
# create_physical_ai_package.sh

echo "Creating Physical AI Package..."

# پیکج بنائیں
ros2 pkg create --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs \
  physical_ai_demo

cd physical_ai_demo

# ڈائریکٹری ساخت بنائیں
mkdir -p launch config msg srv test

# کسٹم میسج بنائیں
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

# کسٹم سروس بنائیں
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

# package.xml کو کسٹم میسج انحصاریات کے ساتھ اپ ڈیٹ کریں
sed -i '/<build_type>ament_python<\/build_type>/i \
  <depend>message_runtime</depend>' package.xml

# مرکزی Python ماڈیول بنائیں
mkdir -p physical_ai_demo
touch physical_ai_demo/__init__.py

# مرکزی نوڈ نافذ کاری بنائیں
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

        # پبلشرز
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(PhysicalState, 'physical_state', 10)
        self.status_pub = self.create_publisher(String, 'controller_status', 10)

        # سبسکرائبزرز
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # سروسز
        self.interaction_srv = self.create_service(
            PhysicalInteraction, 'physical_interaction',
            self.interaction_callback)

        # پیرامیٹرز
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # ٹائمرز
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.state_timer = self.create_timer(0.5, self.publish_state)

        # داخلی اسٹیٹ
        self.current_position = [0.0, 0.0, 0.0]
        self.current_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
        self.is_stable = True

        self.get_logger().info('Physical AI Controller initialized')

    def scan_callback(self, msg):
        """لیزر اسکین ڈیٹا کو ہینڈل کریں۔"
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r > 0], default=float('inf'))
            if min_range < self.get_parameter('safety_distance').value:
                self.is_stable = False
                self.get_logger().warn(f'Obstacle detected at {min_range:.2f}m')
            else:
                self.is_stable = True

    def control_loop(self):
        """مرکزی کنٹرول لوپ۔"
        cmd = Twist()

        # سادہ کنٹرول لاگک
        if self.is_stable:
            cmd.linear.x = self.get_parameter('max_linear_speed').value
            cmd.angular.z = 0.0
            status = "Moving safely"
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            status = "Stopped - obstacle detected"

        # کمانڈ شائع کریں
        self.cmd_pub.publish(cmd)

        # حیثیت شائع کریں
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def publish_state(self):
        """فزیکل اسٹیٹ شائع کریں۔"
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
        """فزیکل انٹرایکشن کی درخواستوں کو ہینڈل کریں۔"
        self.get_logger().info(f'Received interaction: {request.interaction_type}')

        # انٹرایکشن کی شبیہہ بنائیں
        response.success = True
        response.message = f'Completed {request.interaction_type} interaction'
        response.actual_force_applied = request.force_magnitude * 0.9  # کارکردگی کی شبیہہ بنائیں

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

# لانچ فائل بنائیں
cat > launch/physical_ai_demo.launch.py << 'LAUNCH_EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # کنفیگ فائل پاتھ حاصل کریں
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

# کنفیگریشن فائل بنائیں
cat > config/controller_config.yaml << 'CONFIG_EOF'
physical_ai_controller:
  ros__parameters:
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    safety_distance: 0.8
    control_frequency: 10.0
CONFIG_EOF

# setup.py اپ ڈیٹ کریں
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

# setup.cfg بنائیں
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

## ٹیسٹنگ اور معیار کی ضمانت

### یونٹ ٹیسٹنگ

اپنے پیکجز کے لیے جامع ٹیسٹس بنائیں:

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
        """ٹیسٹ کریں کہ نوڈ صحیح طریقے سے شروع ہوتا ہے۔"
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'physical_ai_controller')

    def test_publishers_created(self):
        """ٹیسٹ کریں کہ تمام ضروری پبلشرز بنائے گئے ہیں۔"
        # چیک کریں کہ پبلشرز موجود ہیں (وہ ایٹری بیوٹس کے طور پر بنائے گئے ہیں)
        self.assertTrue(hasattr(self.node, 'cmd_pub'))
        self.assertTrue(hasattr(self.node, 'state_pub'))
        self.assertTrue(hasattr(self.node, 'status_pub'))

    def test_parameters_declared(self):
        """ٹیسٹ کریں کہ ضروری پیرامیٹرز کا اعلان کیا گیا ہے۔"
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

### لینٹنگ اور کوڈ کا معیار

ROS 2 کوڈ کے معیار کی جانچ کے لیے ٹولز فراہم کرتا ہے:

```bash
# کوڈ کے معیار کی چیکس چلائیں
ament_copyright --verbose src/my_robot_package
ament_flake8 src/my_robot_package
ament_pep257 src/my_robot_package

# یا تمام چیکس کے لیے ament_lint_auto استعمال کریں
ament_lint_auto src/my_robot_package
```

## عام پیکج مسائل کا حل

### 1. بلڈ ایررز

```bash
# مسئلہ: پیکج نہیں بن رہا
# حل: انحصاریات اور بلڈ کنفیگریشن چیک کریں

# غائب انحصاریات کے لیے چیک کریں
rosdep check --from-paths src --ignore-src -r -y

# صاف اور دوبارہ بنائیں
rm -rf build/ install/ log/
colcon build --packages-select my_package --event-handlers console_direct+

# بلڈ لاگس چیک کریں
find build/my_package -name "*.log" -exec echo "=== {} ===" \; -exec cat {} \;
```

### 2. درآمد کے ایررز

```bash
# مسئلہ: Python ماڈیولز نہیں مل رہے
# حل: setup.py اور PYTHONPATH چیک کریں

# یقینی بنائیں کہ پیکج انسٹال ہے
cd ~/ros2_ws
source install/setup.bash
python3 -c "import my_robot_package"  # ImportError نہیں اٹھانا چاہیے
```

### 3. پیکج نہیں ملا

```bash
# مسئلہ: ros2 run یا ros2 launch پیکج نہیں ڈھونڈ سکتا
# حل: انسٹالیشن اور ماحول چیک کریں

# ورک سپیس سورس کریں
source ~/ros2_ws/install/setup.bash

# چیک کریں کہ آیا پیکج ملا
ros2 pkg list | grep my_package

# پیکج پاتھ چیک کریں
ros2 pkg prefix my_package
```

## مزید سیکھنے کے لیے وسائل

- [ROS 2 پیکج تخلیق ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 package.xml فارمیٹ](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [Colcon بلڈ ٹول](https://colcon.readthedocs.io/en/released/)
- [ROS 2 کوالٹی آف سروس](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## خلاصہ

ROS 2 پیکجز بنانا پیکج ساخت، انحصاریات کا انتظام، بلڈ سسٹم (C++ کے لیے CMake، Python کے لیے setuptools)، اور مناسب تنظیم کی سمجھ کی ضرورت رکھتا ہے۔ ایک اچھی طرح سے ساخت والے پیکج میں package.xml میں مناسب میٹا ڈیٹا، منظم سورس کوڈ، کنفیگریشن فائلز، لانچ فائلز، اور جامع ٹیسٹنگ شامل ہے۔ پیکج ڈیولپمنٹ کے لیے ROS 2 کی بہترین مشقیں کو فالو کرنا قابل برقرار رکھنے، دوبارہ استعمال کے قابل، اور مضبوط روبوٹک ایپلی کیشنز کو یقینی بناتا ہے۔