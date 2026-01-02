---
sidebar_position: 5
---

# سینسر ڈیٹا کی توثیق اور حقیقی دنیا کی توقعات کے خلاف ٹیسٹنگ

یہ سبق Gazebo سمولیشنز میں سینسر ڈیٹا کی توثیق کرنے کا احاطہ کرتا ہے تاکہ یہ یقینی بنایا جا سکے کہ یہ حقیقی دنیا کی توقعات سے مطابقت رکھتا ہے اور روبوٹکس ایپلی کیشنز میں قابل اعتماد طریقے سے استعمال کیا جا سکتا ہے۔

## سینسر کی توثیق کو سمجھنا

### سینسر ڈیٹا کی توثیق کیوں کریں؟

سینسر ڈیٹا کی توثیق بہت اہم ہے کیونکہ:
- سمولیشن سینسرز کو حقیقی سینسرز کی طرح برتاؤ کرنا چاہیے
- ادراک (perception) کے الگورتھم حقیقت پسندانہ سینسر خصوصیات پر انحصار کرتے ہیں
- کنٹرول سسٹمز درست سینسر ریڈنگز پر انحصار کرتے ہیں
- سمولیشن سے تربیتی ڈیٹا حقیقی روبوٹ کی تعیناتی کے لیے استعمال کیا جا سکتا ہے

### توثیق کے اہم اہداف

1. **رینج کی توثیق**: سینسر ریڈنگز متوقع حدود میں ہونی چاہئیں
2. **ریزولیوشن کی توثیق**: سینسر ڈیٹا میں مناسب ریزولیوشن ہونی چاہیے
3. **شور کی خصوصیات**: نقلی شور کو حقیقی سینسر کی خصوصیات سے مطابقت رکھنی چاہیے
4. **ٹائمنگ کی توثیق**: سینسر اپ ڈیٹ ریٹس حقیقت پسندانہ ہونے چاہئیں
5. **جیومیٹرک توثیق**: سینسر کے پوز اور ماؤنٹنگ درست ہونی چاہیے

## LiDAR سینسر کی توثیق

### متوقع LiDAR رویہ

ایک اچھی طرح سے کنفیگرڈ LiDAR کو:
- اپنی رینج کی حدود میں اشیاء کا پتہ لگانا چاہیے
- کونیی (angular) ریزولیوشن کی بنیاد پر حقیقت پسندانہ ریزولیوشن ہونی چاہیے
- متوقع شور کی خصوصیات دکھانی چاہئیں
- مختلف مواد اور سطح کی خصوصیات کو صحیح طریقے سے ہینڈل کرنا چاہیے

### LiDAR توثیق کی مثال

```xml
<!-- LiDAR validation test.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="lidar_validation">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- LiDAR platform -->
    <model name="lidar_platform">
      <pose>0 0 1 0 0 0</pose>
      <link name="base">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual>
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
        </collision>
      </link>

      <!-- LiDAR sensor -->
      <link name="lidar_link">
        <visual>
          <geometry>
            <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <cylinder><radius>0.05</radius><length>0.05</length></collision>
        </collision>
        <sensor name="lidar" type="ray">
          <pose>0.1 0 0 0 0 0</pose>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.1</min>
              <max>10.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
        </sensor>
      </link>

      <joint name="lidar_joint" type="fixed">
        <parent>base</parent>
        <child>lidar_link</child>
        <pose>0 0 0.1 0 0 0</pose>
      </joint>
    </model>

    <!-- Test objects at known positions -->
    <model name="test_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <visual>
          <geometry>
            <box><size>0.5 0.5 1</size></box>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box><size>0.5 0.5 1</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### LiDAR توثیق کا کوڈ

LiDAR ڈیٹا چیک کرنے کے لیے توثیقی اسکرپٹ بنائیں:

```python
#!/usr/bin/env python3
"""
LiDAR validation script for Gazebo simulations
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarValidator(Node):
    def __init__(self):
        super().__init__('lidar_validator')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',  # Adjust topic name as needed
            self.lidar_callback,
            10)

        self.validation_results = []
        self.test_distances = [2.0]  # Known distance to test object

    def lidar_callback(self, msg):
        # Get ranges within a small angle around expected direction
        # For a test object at 2m in front (angle ~0)
        angle_increment = msg.angle_increment
        center_idx = int(len(msg.ranges) / 2)  # Approximately front-facing

        # Check a small range of angles around the front
        check_range = 5  # Check 5 indices around center
        front_readings = msg.ranges[center_idx-check_range:center_idx+check_range]

        # Filter out invalid readings (inf, nan)
        valid_readings = [r for r in front_readings if np.isfinite(r)]

        if valid_readings:
            avg_front_distance = np.mean(valid_readings)
            expected_distance = 2.0  # Known distance to test object

            # Check if reading is within tolerance (e.g., 10cm)
            tolerance = 0.1
            is_valid = abs(avg_front_distance - expected_distance) <= tolerance

            if is_valid:
                self.get_logger().info(f'LiDAR validation PASSED: {avg_front_distance:.2f}m vs {expected_distance}m')
            else:
                self.get_logger().warn(f'LiDAR validation FAILED: {avg_front_distance:.2f}m vs {expected_distance}m')

            # Store result for summary
            self.validation_results.append({
                'expected': expected_distance,
                'measured': avg_front_distance,
                'valid': is_valid
            })

            # Print validation statistics periodically
            if len(self.validation_results) % 50 == 0:
                self.print_validation_summary()

    def print_validation_summary(self):
        if not self.validation_results:
            return

        total = len(self.validation_results)
        passed = sum(1 for r in self.validation_results if r['valid'])
        accuracy = passed / total

        self.get_logger().info(f'LiDAR validation summary: {passed}/{total} ({accuracy*100:.1f}% accuracy)')

def main(args=None):
    rclpy.init(args=args)
    validator = LidarValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.print_validation_summary()

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## کیمرہ سینسر کی توثیق

### متوقع کیمرہ رویہ

ایک اچھی طرح سے کنفیگرڈ کیمرہ کو:
- مناسب ریزولیوشن اور فیلڈ آف ویو کے ساتھ تصاویر بنانا چاہیے
- حقیقت پسندانہ تناظر کا بگاڑ (perspective distortion) ہونا چاہیے
- روشنی کے حالات کو صحیح طریقے سے ہینڈل کرنا چاہیے
- سٹیریو/ڈیپتھ کیمروں میں گہرائی کی درست نمائندگی کرنی چاہیے

### کیمرہ توثیق کی مثال

```xml
<!-- Camera validation test.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="camera_validation">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Camera platform -->
    <model name="camera_platform">
      <pose>0 0 1 0 0 0</pose>
      <link name="base">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual>
          <geometry>
            <box><size>0.1 0.1 0.1</size></box>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box><size>0.1 0.1 0.1</size></box>
          </geometry>
        </collision>
      </link>

      <!-- RGB camera -->
      <link name="camera_link">
        <sensor name="camera" type="camera">
          <pose>0.05 0 0 0 0 0</pose>
          <camera>
            <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
            <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.007</stddev>
            </noise>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </sensor>
      </link>

      <joint name="camera_joint" type="fixed">
        <parent>base</parent>
        <child>camera_link</child>
        <pose>0 0 0.05 0 0 0</pose>
      </joint>
    </model>

    <!-- Test object with known dimensions -->
    <model name="checkerboard">
      <pose>2 0 1 0 0 0</pose>
      <link name="link">
        <visual>
          <geometry>
            <box><size>1 1 0.01</size></box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/CheckerBlue</name>
            </script>
          </material>
        </visual>
        <collision>
          <geometry>
            <box><size>1 1 0.01</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### کیمرہ توثیق کا کوڈ

```python
#!/usr/bin/env python3
"""
Camera validation script for Gazebo simulations
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraValidator(Node):
    def __init__(self):
        super().__init__('camera_validator')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust topic name as needed
            self.image_callback,
            10)

        self.bridge = CvBridge()
        self.validation_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform basic validation
            height, width, channels = cv_image.shape

            # Check image dimensions
            expected_width = 640
            expected_height = 480
            expected_channels = 3

            dims_ok = (width == expected_width and
                      height == expected_height and
                      channels == expected_channels)

            # Check if image is not all black or all white (basic sanity check)
            mean_intensity = np.mean(cv_image)
            valid_intensity = 10 < mean_intensity < 245  # Reasonable range

            if dims_ok and valid_intensity:
                self.get_logger().info(f'Camera validation PASSED: {width}x{height}x{channels}')
            else:
                self.get_logger().warn(f'Camera validation ISSUES: dims_ok={dims_ok}, valid_intensity={valid_intensity}')

            self.validation_count += 1
            if self.validation_count % 30 == 0:  # Every 30 frames
                self.get_logger().info(f'Camera validation running, processed {self.validation_count} frames')

        except Exception as e:
            self.get_logger().error(f'Error in camera validation: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    validator = CameraValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info(f'Camera validation stopped after {validator.validation_count} frames')

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU سینسر کی توثیق

### متوقع IMU رویہ

ایک اچھی طرح سے کنفیگرڈ IMU کو:
- کشش ثقل کی وجہ سے حقیقت پسندانہ ایکسلریشن کی اطلاع دینی چاہیے
- روبوٹ کے حرکت کرنے پر واقفیت میں مناسب تبدیلیاں دکھانی چاہئیں
- مناسب شور کی خصوصیات ہونی چاہئیں
- گھومتے وقت کونیی (angular) رفتار کی درست نمائندگی کرنی چاہیے

### IMU توثیق کا کوڈ

```python
#!/usr/bin/env python3
"""
IMU validation script for Gazebo simulations
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuValidator(Node):
    def __init__(self):
        super().__init__('imu_validator')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Adjust topic name as needed
            self.imu_callback,
            10)

        self.gravity_magnitude = 9.81
        self.validation_results = []

    def imu_callback(self, msg):
        # Extract linear acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Calculate magnitude of acceleration
        accel_magnitude = np.sqrt(ax**2 + ay**2 + az**2)

        # For a static IMU, the magnitude should be close to gravity (9.81 m/s²)
        # with some tolerance for noise and simulation accuracy
        expected_gravity = self.gravity_magnitude
        tolerance = 0.5  # 0.5 m/s² tolerance

        # Check if magnitude is reasonable
        is_reasonable = abs(accel_magnitude - expected_gravity) <= tolerance

        # Check if at least one axis shows gravity (for different orientations)
        # In static case, one axis should be near ±9.81
        gravity_axis_found = (abs(ax) > 8.0 or abs(ay) > 8.0 or abs(az) > 8.0)

        if is_reasonable and gravity_axis_found:
            self.get_logger().info(f'IMU validation PASSED: |a|={accel_magnitude:.2f}')
        else:
            self.get_logger().warn(f'IMU validation WARNING: |a|={accel_magnitude:.2f}, gravity_found={gravity_axis_found}')

        self.validation_results.append({
            'magnitude': accel_magnitude,
            'reasonable': is_reasonable,
            'gravity_axis': gravity_axis_found
        })

def main(args=None):
    rclpy.init(args=args)
    validator = ImuValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        if validator.validation_results:
            avg_magnitude = np.mean([r['magnitude'] for r in validator.validation_results])
            self.get_logger().info(f'IMU validation summary: avg magnitude = {avg_magnitude:.2f}')

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ملٹی سینسر کی توثیق

### سینسر فیوژن کی توثیق

توثیق کریں کہ متعدد سینسرز ایک ساتھ صحیح طریقے سے کام کرتے ہیں:

```python
#!/usr/bin/env python3
"""
Multi-sensor validation script
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import threading
import time

class MultiSensorValidator(Node):
    def __init__(self):
        super().__init__('multi_sensor_validator')

        # Initialize sensors
        self.lidar_data = None
        self.imu_data = None
        self.image_data = None

        self.lock = threading.Lock()

        # Subscribe to all sensors
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Timer for periodic validation
        self.timer = self.create_timer(1.0, self.validate_synchronization)

    def lidar_callback(self, msg):
        with self.lock:
            self.lidar_data = msg

    def imu_callback(self, msg):
        with self.lock:
            self.imu_data = msg

    def image_callback(self, msg):
        with self.lock:
            self.image_data = msg

    def validate_synchronization(self):
        with self.lock:
            sensors_active = {
                'lidar': self.lidar_data is not None,
                'imu': self.imu_data is not None,
                'camera': self.image_data is not None
            }

            all_active = all(sensors_active.values())

            if all_active:
                self.get_logger().info('Multi-sensor validation PASSED: All sensors active')

                # Additional checks could go here
                # For example: verify timestamp synchronization
                lidar_time = self.lidar_data.header.stamp.sec + self.lidar_data.header.stamp.nanosec * 1e-9
                imu_time = self.imu_data.header.stamp.sec + self.imu_data.header.stamp.nanosec * 1e-9

                time_diff = abs(lidar_time - imu_time)
                if time_diff > 0.1:  # More than 100ms apart
                    self.get_logger().warn(f'Sensors not synchronized: {time_diff:.3f}s difference')
            else:
                missing = [name for name, active in sensors_active.items() if not active]
                self.get_logger().warn(f'Multi-sensor validation: Missing sensors: {missing}')

def main(args=None):
    rclpy.init(args=args)
    validator = MultiSensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Multi-sensor validation stopped')

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## توثیق کے ٹولز اور تکنیکیں

### وژولائزیشن کے لیے rviz2 کا استعمال

```bash
# Launch rviz2 to visualize sensor data
ros2 run rviz2 rviz2

# Add displays for:
# - LaserScan for LiDAR data
# - Image for camera feeds
# - Imu for IMU orientation
# - PointCloud2 for 3D point clouds
```

### کمانڈ لائن توثیق

```bash
# Check if sensor topics are publishing
ros2 topic echo /lidar/scan --field ranges | head -n 10

# Monitor sensor frequency
ros2 topic hz /camera/image_raw

# List available sensor topics
ros2 topic list | grep -E "(scan|camera|imu)"
```

### لانچ فائلوں کے ساتھ خودکار ٹیسٹنگ

سینسر کی توثیق کو خودکار کرنے کے لیے ایک لانچ فائل بنائیں:

```python
# sensor_validation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with sensor world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'sensor_validation_world.sdf'],
            output='screen'
        ),

        # Launch sensor validation nodes
        Node(
            package='your_package',
            executable='lidar_validator',
            name='lidar_validator',
            output='screen'
        ),

        Node(
            package='your_package',
            executable='camera_validator',
            name='camera_validator',
            output='screen'
        ),

        Node(
            package='your_package',
            executable='imu_validator',
            name='imu_validator',
            output='screen'
        ),

        Node(
            package='your_package',
            executable='multi_sensor_validator',
            name='multi_sensor_validator',
            output='screen'
        )
    ])
```

## کارکردگی کی توثیق

### سینسر اپ ڈیٹ ریٹ کی ٹیسٹنگ

توثیق کریں کہ سینسرز متوقع شرحوں پر پبلش ہو رہے ہیں:

```python
# Rate validation script
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class RateValidator(Node):
    def __init__(self):
        super().__init__('rate_validator')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10)

        self.received_times = []
        self.target_rate = 10.0  # 10 Hz for example

    def scan_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.received_times.append(current_time)

        # Calculate actual rate from last 10 messages
        if len(self.received_times) > 10:
            self.received_times = self.received_times[-10:]  # Keep last 10

            time_diff = self.received_times[-1] - self.received_times[0]
            actual_rate = len(self.received_times) / time_diff if time_diff > 0 else 0

            rate_error = abs(actual_rate - self.target_rate)
            tolerance = self.target_rate * 0.1  # 10% tolerance

            if rate_error <= tolerance:
                self.get_logger().info(f'Rate validation PASSED: {actual_rate:.2f} Hz (target: {self.target_rate})')
            else:
                self.get_logger().warn(f'Rate validation FAILED: {actual_rate:.2f} Hz (target: {self.target_rate})')
```

## عام توثیقی مسائل اور حل

### مسئلہ 1: کوئی سینسر ڈیٹا نہیں
**علامات**: سینسر ٹاپکس پر کوئی پیغامات نہیں
**حل**:
- چیک کریں کہ سینسر لنک کے ساتھ صحیح طریقے سے منسلک ہے
- تصدیق کریں کہ `always_on` `1` پر سیٹ ہے
- یقینی بنائیں کہ سینسر پلگ ان صحیح طریقے سے لوڈ ہوا ہے
- چیک کریں کہ Gazebo صحیح دنیا (world) کے ساتھ چل رہا ہے

### مسئلہ 2: غلط رینج ریڈنگز
**علامات**: LiDAR تمام صفر، inf، یا NaN واپس کرتا ہے
**حل**:
- SDF میں سینسر کی رینج کے پیرامیٹرز چیک کریں
- تصدیق کریں کہ سینسر کے فیلڈ آف ویو میں اشیاء موجود ہیں
- چیک کریں کہ اشیاء کے لیے ٹکراؤ کی جیومیٹریز (collision geometries) کی وضاحت کی گئی ہے
- توثیق کریں کہ سینسر لنک صحیح پوزیشن پر ہے

### مسئلہ 3: غیر حقیقت پسندانہ شور
**علامات**: سینسر ڈیٹا بہت صاف یا بہت زیادہ شور والا ہے
**حل**:
- SDF کنفیگریشن میں شور کے پیرامیٹرز کو ایڈجسٹ کریں
- حقیقی سینسر کی تفصیلات کی بنیاد پر حقیقت پسندانہ شور کے ماڈلز شامل کریں
- ماحولیاتی عوامل پر غور کریں (مثال کے طور پر، کیمروں کے لیے روشنی)

## توثیقی چیک لسٹ

سینسر ڈیٹا کو درست ماننے سے پہلے:

- [ ] LiDAR متوقع حدود میں اشیاء کا پتہ لگاتا ہے
- [ ] کیمرہ درست ریزولیوشن اور فارمیٹ کے ساتھ تصاویر تیار کرتا ہے
- [ ] IMU حقیقت پسندانہ ایکسلریشن اور واقفیت کی رپورٹ دیتا ہے
- [ ] سینسر ڈیٹا کی شرح کنفیگریشن سے مطابقت رکھتی ہے
- [ ] شور کی خصوصیات حقیقت پسندانہ ہیں
- [ ] متعدد سینسرز ہم وقت ساز (synchronized) ہیں (اگر ضرورت ہو)
- [ ] ڈیٹا کو ادراک (perception) کے الگورتھم کے ذریعے پروسیس کیا جا سکتا ہے
- [ ] ٹائم اسٹیمپس معقول اور مستقل ہیں

## اگلے اقدامات

اپنی سینسر سمولیشن کی توثیق کرنے کے بعد:

1. [سینسر فیوژن](./sensor-fusion.md) پر جاری رکھیں تاکہ سیکھیں کہ متعدد سینسرز کو کیسے اکٹھا کیا جائے
2. ہائی فیڈیلیٹی وژولائزیشن کے لیے [Unity انضمام](../unity-integration/unity-setup.md) دریافت کریں
3. اپنے علم کا اطلاق کرنے کے لیے [اسیسمنٹ پروجیکٹ](../assessment-project/project-overview.md) پر کام کریں
