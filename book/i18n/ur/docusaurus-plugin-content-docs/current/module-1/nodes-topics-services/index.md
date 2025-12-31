---
title: ROS 2 نوڈس، ٹاپکس، اور سروسز
description: ROS 2 میں بنیادی مواصلاتی نمونوں کو سمجھنا
sidebar_position: 3
---

# ROS 2 نوڈس، ٹاپکس، اور سروسز

## سیکھنے کے اہداف
- ROS 2 نوڈس کی تخلیق اور انتظام میں مہارت حاصل کریں
- پبلشر-سبسکرائب مواصلاتی نمونہ کو سمجھیں
- درخواست-جواب مواصلات کے لیے سروسز کو نافذ کریں
- میسج کی اقسام اور کسٹم میسج کی تعریفیں سیکھیں
- نوڈس کے درمیان مضبوط مواصلاتی سسٹم تیار کریں

## ROS 2 مواصلات کا تعارف

ROS 2 تین بنیادی مواصلاتی نمونے فراہم کرتا ہے جو روبوٹک ایپلی کیشنز کی پشت پر چلتے ہیں:

1. **نوڈس**: انجام دہندہ یونٹس جو حسابات انجام دیتے ہیں
2. **ٹاپکس**: غیر ہم وقت، کثیر سے کثیر مواصلات پبلشر-سبسکرائب نمونے کے ذریعے
3. **سروسز**: ہم وقت درخواست-جواب مواصلات

یہ نمونے ایک ساتھ کام کر کے تقسیم شدہ روبوٹک سسٹم تیار کرتے ہیں جہاں مختلف اجزاء موثر طریقے سے مواصلات کر سکتے ہیں۔

## ROS 2 نوڈس کو سمجھنا

نوڈس ROS 2 ایپلی کیشنز کے بنیادی اجزاء ہیں۔ ہر نوڈ ایک الگ عمل ہے جس میں پبلشرز، سبسکرائبزرز، سروسز، اور دیگر ROS ادارتیں ہو سکتی ہیں۔

### بنیادی نوڈ ساخت

```python
import rclpy
from rclpy.node import Node

class BasicNodeExample(Node):
    def __init__(self):
        # ایک نام کے ساتھ نوڈ کو شروع کریں
        super().__init__('basic_node_example')

        # ایک پیغام لاگ کریں
        self.get_logger().info('Basic node has been initialized')

def main(args=None):
    # ROS 2 کو شروع کریں
    rclpy.init(args=args)

    # نوڈ بنائیں
    node = BasicNodeExample()

    try:
        # نوڈ کو چلتا رکھیں
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        # صاف کریں
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### نوڈ پیرامیٹرز

نوڈس کنفیگریشن کے لیے پیرامیٹرز قبول کر سکتے ہیں:

```python
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # ڈیفالٹ ویلیوز کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('update_rate', 10)
        self.declare_parameter('safety_enabled', True)

        # پیرامیٹر ویلیوز حاصل کریں
        self.robot_name = self.get_parameter('robot_name').value
        self.update_rate = self.get_parameter('update_rate').value
        self.safety_enabled = self.get_parameter('safety_enabled').value

        self.get_logger().info(
            f'Initialized with: name={self.robot_name}, '
            f'rate={self.update_rate}Hz, safety={self.safety_enabled}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## پبلشر-سبسکرائب نمونہ (ٹاپکس)

پبلشر-سبسکرائب نمونہ نوڈس کے درمیان غیر ہم وقت مواصلات کو فعال کرتا ہے۔ پبلشرز ٹاپکس پر میسجز بھیجتے ہیں، اور سبسکرائبزرز ٹاپکس سے میسجز وصول کرتے ہیں۔

### ایک پبلشر بنانا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')

        # ایک پبلشر بنائیں
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # میسجز کو م_PERIODICALLY_ شائع کرنے کے لیے ایک ٹائمر بنائیں
        self.timer = self.create_timer(0.5, self.timer_callback)  # ہر 0.5 سیکنڈ پر شائع کریں
        self.i = 0

        self.get_logger().info('Talker node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i} at {time.time()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()
```

### ایک سبسکرائبر بنانا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')

        # ایک سبسکرپشن بنائیں
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10  # قیو سائز
        )
        # غیر استعمال شدہ متغیر کی وارننگ کو روکیں
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()
```

### پیچیدہ میسج کی اقسام

ROS 2 مختلف ڈیٹا کے لیے مختلف میسج کی اقسام فراہم کرتا ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Header
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan, Image
import numpy as np

class ComplexMessageNode(Node):
    def __init__(self):
        super().__init__('complex_message_node')

        # رفتار کمانڈز کے لیے پبلشر
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # سینسر ڈیٹا کے لیے پبلشر
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # متعدد ابعاد ڈیٹا کے لیے پبلشر
        self.array_pub = self.create_publisher(Float64MultiArray, 'joint_positions', 10)

        # م_PERIODICALLY_ شائع کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.1, self.publish_data)

        self.get_logger().info('Complex message node initialized')

    def publish_data(self):
        # رفتار کمانڈ شائع کریں
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # 0.5 میٹر/سیکنڈ پر آگے بڑھیں
        twist_msg.angular.z = 0.2  # 0.2 ریڈین/سیکنڈ پر بائیں طرف مڑیں
        self.cmd_vel_pub.publish(twist_msg)

        # لیزر اسکین ڈیٹا شائع کریں
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = -np.pi / 2  # -90 ڈگری
        scan_msg.angle_max = np.pi / 2   # 90 ڈگری
        scan_msg.angle_increment = np.pi / 180  # 1 ڈگری
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # کچھ رینج ڈیٹا کی شبیہہ بنائیں
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [2.0 + 0.5 * np.sin(i * 0.1) for i in range(num_readings)]

        self.scan_pub.publish(scan_msg)

        # جوائنٹ پوزیشنز شائع کریں
        array_msg = Float64MultiArray()
        array_msg.data = [0.1, 0.2, 0.3, 0.4, 0.5]  # مثال جوائنٹ اینگلز
        self.array_pub.publish(array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ComplexMessageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## سروسز - درخواست-جواب مواصلات

سروسز ہم وقت مواصلات فراہم کرتی ہیں جہاں ایک کلائنٹ ایک درخواست بھیجتا ہے اور جواب کا انتظار کرتا ہے۔

### ایک سروس سرور بنانا

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # ایک سروس بنائیں
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server started')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service = MinimalService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()
```

### ایک سروس کلائنٹ بنانا

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # ایک کلائنٹ بنائیں
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # سروس کی دستیابی کا انتظار کریں
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # سروس کو غیر ہم وقت طور پر کال کریں
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    client = MinimalClient()

    # درخواست بھیجیں
    future = client.send_request(1, 2)

    try:
        # جواب کا انتظار کریں
        rclpy.spin_until_future_complete(client, future)
        response = future.result()
        client.get_logger().info(f'Result: {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()
```

## کسٹم میسج کی تعریفیں

آپ اپنی مخصوص ایپلی کیشن کی ضروریات کے لیے کسٹم میسج کی اقسام کی تعریف کر سکتے ہیں:

### کسٹم میسجز بنانا

سب سے پہلے، اپنے پیکج میں ایک `msg` ڈائریکٹری بنائیں اور ایک کسٹم میسج کی تعریف کریں:

```# JointState.msg
# جوائنٹ اسٹیٹ معلومات کے لیے کسٹم میسج
string name
float64 position
float64 velocity
float64 effort
time timestamp
```

پھر اسے اپنے نوڈس میں استعمال کریں:

```python
import rclpy
from rclpy.node import Node
# فرض کریں کہ آپ کے پاس ایک کسٹم میسج پیکج ہے
# from your_robot_msgs.msg import JointState

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')

        # کسٹم میسج کے لیے پبلشر بنائیں
        # self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # اس مثال کے لیے، ہم ایک معیاری میسج استعمال کریں گے
        self.data_pub = self.create_publisher(String, 'custom_data', 10)

        self.timer = self.create_timer(1.0, self.publish_custom_data)

    def publish_custom_data(self):
        # کسٹم میسج ڈیٹا بنانا کی شبیہہ بنائیں
        msg = String()
        msg.data = f'Custom data at {self.get_clock().now().to_msg()}'
        self.data_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomMessageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## اعلی درجے کے مواصلاتی نمونے

### متعدد پبلشرز اور سبسکرائبزرز

```python
class MultiCommunicationNode(Node):
    def __init__(self):
        super().__init__('multi_comm_node')

        # متعدد پبلشرز
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_odom = self.create_publisher(String, 'odometry', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)

        # متعدد سبسکرائبزرز
        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel_input', self.cmd_callback, 10)
        self.sub_sensor = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)
        self.sub_control = self.create_subscription(
            String, 'control_commands', self.control_callback, 10)

        self.get_logger().info('Multi-communication node initialized')

    def cmd_callback(self, msg):
        self.get_logger().info(f'Received command: linear={msg.linear.x}, angular={msg.angular.z}')
        # کمانڈ کو پروسیس کریں اور ممکنہ طور پر دیگر ٹاپکس پر شائع کریں

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def control_callback(self, msg):
        self.get_logger().info(f'Received control command: {msg.data}')
```

### شرطی منطق کے ساتھ پبلشر

```python
class ConditionalPublisherNode(Node):
    def __init__(self):
        super().__init__('conditional_publisher')

        self.pub = self.create_publisher(String, 'conditional_topic', 10)
        self.sub = self.create_subscription(String, 'trigger_topic', self.trigger_callback, 10)

        self.publishing_enabled = False
        self.publish_timer = self.create_timer(1.0, self.publish_if_enabled)

    def trigger_callback(self, msg):
        if msg.data == 'enable':
            self.publishing_enabled = True
            self.get_logger().info('Publishing enabled')
        elif msg.data == 'disable':
            self.publishing_enabled = False
            self.get_logger().info('Publishing disabled')

    def publish_if_enabled(self):
        if self.publishing_enabled:
            msg = String()
            msg.data = f'Conditional message at {self.get_clock().now().to_msg()}'
            self.pub.publish(msg)
```

## ہاتھوں سے مشق: مواصلاتی سسٹم

پبلشر-سبسکرائب اور سروس نمونوں کو ظاہر کرنے والے متعدد نوڈس کے ساتھ ایک مکمل مواصلاتی سسٹم بنائیں۔

### مشق کی ضروریات
1. ایک سینسر نوڈ بنائیں جو سینسر ڈیٹا شائع کرتا ہے
2. ایک پروسیسنگ نوڈ بنائیں جو سینسر ڈیٹا کو سبسکرائب کرتا ہے اور اسے پروسیس کرتا ہے
3. ایک سروس نوڈ بنائیں جو ڈیٹا تجزیہ فراہم کرتا ہے
4. ایک کلائنٹ نوڈ بنائیں جو تجزیہ سروس استعمال کرتا ہے

### Python نافذ کاری

```python
#!/usr/bin/env python3
"""
ROS 2 مواصلاتی سسٹم مشق
پبلشر-سبسکرائب اور سروس نمونوں کا مظاہرہ کرتا ہے
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from example_interfaces.srv import Trigger
import random
import time
from collections import deque

class SensorNode(Node):
    """نود جو سینسر ڈیٹا شائع کرنے کی شبیہہ بناتا ہے"""
    def __init__(self):
        super().__init__('sensor_node')

        # سینسر ڈیٹا کے لیے پبلشر
        self.sensor_pub = self.create_publisher(Float32, 'sensor_data', 10)

        # سینسر ڈیٹا کو م_PERIODICALLY_ شائع کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.2, self.publish_sensor_data)

        self.get_logger().info('Sensor node started')

    def publish_sensor_data(self):
        # کچھ نوائز کے ساتھ سینسر ریڈنگ کی شبیہہ بنائیں
        msg = Float32()
        msg.data = random.uniform(0.0, 100.0)  # سینسر ویلیو کی شبیہہ بنائیں
        self.sensor_pub.publish(msg)
        self.get_logger().info(f'Published sensor data: {msg.data:.2f}')

class ProcessingNode(Node):
    """نود جو سینسر ڈیٹا کو پروسیس کرتا ہے اور نتائج شائع کرتا ہے"""
    def __init__(self):
        super().__init__('processing_node')

        # سینسر ڈیٹا کے لیے سبسکرپشن
        self.sensor_sub = self.create_subscription(
            Float32, 'sensor_data', self.sensor_callback, 10)

        # پروسیسڈ ڈیٹا کے لیے پبلشر
        self.processed_pub = self.create_publisher(String, 'processed_data', 10)

        # اوسط کے لیے حالیہ ویلیوز اسٹور کریں
        self.recent_values = deque(maxlen=5)

        self.get_logger().info('Processing node started')

    def sensor_callback(self, msg):
        # اوسط کے لیے حالیہ ویلیوز میں شامل کریں
        self.recent_values.append(msg.data)

        # حالیہ ویلیوز کا اوسط حساب لگائیں
        avg_value = sum(self.recent_values) / len(self.recent_values)

        # پروسیسڈ ڈیٹا میسج بنائیں
        processed_msg = String()
        processed_msg.data = f'Raw: {msg.data:.2f}, Avg: {avg_value:.2f}, Count: {len(self.recent_values)}'

        self.processed_pub.publish(processed_msg)
        self.get_logger().info(f'Processed: {processed_msg.data}')

class AnalysisServiceNode(Node):
    """نود جو ڈیٹا تجزیہ سروس فراہم کرتا ہے"""
    def __init__(self):
        super().__init__('analysis_service')

        # سروس بنائیں
        self.srv = self.create_service(
            Trigger, 'analyze_data', self.analyze_callback)

        # تاریخی ڈیٹا اسٹور کریں
        self.historical_data = deque(maxlen=100)

        # پروسیسڈ ڈیٹا کے لیے سبسکرپشن
        self.data_sub = self.create_subscription(
            String, 'processed_data', self.data_callback, 10)

        self.get_logger().info('Analysis service started')

    def data_callback(self, msg):
        """تجزیہ کے لیے پروسیسڈ ڈیٹا اسٹور کریں"""
        try:
            # پروسیسڈ میسج سے عددی ویلیوز نکالیں
            parts = msg.data.split(', ')
            raw_val = float(parts[0].split(': ')[1])
            self.historical_data.append(raw_val)
        except (ValueError, IndexError):
            self.get_logger().warn(f'Could not parse data: {msg.data}')

    def analyze_callback(self, request, response):
        """اسٹورڈ ڈیٹا کا تجزیہ فراہم کریں"""
        if not self.historical_data:
            response.success = False
            response.message = 'No data available for analysis'
            return response

        # تجزیہ کریں
        avg = sum(self.historical_data) / len(self.historical_data)
        min_val = min(self.historical_data)
        max_val = max(self.historical_data)
        count = len(self.historical_data)

        response.success = True
        response.message = f'Analysis - Avg: {avg:.2f}, Min: {min_val:.2f}, Max: {max_val:.2f}, Count: {count}'

        self.get_logger().info(f'Analysis provided: {response.message}')
        return response

class ClientNode(Node):
    """نود جو تجزیہ سروس استعمال کرتا ہے"""
    def __init__(self):
        super().__init__('client_node')

        # نگرانی کے لیے پروسیسڈ ڈیٹا کے لیے سبسکرپشن بنائیں
        self.monitor_sub = self.create_subscription(
            String, 'processed_data', self.monitor_callback, 10)

        # سروس کلائنٹ بنائیں
        self.cli = self.create_client(Trigger, 'analyze_data')

        # تجزیہ کی درخواست کے لیے ٹائمر
        self.timer = self.create_timer(5.0, self.request_analysis)

        self.get_logger().info('Client node started')

    def monitor_callback(self, msg):
        """پروسیسڈ ڈیٹا کی نگرانی کریں"""
        self.get_logger().info(f'Monitoring: {msg.data}')

    def request_analysis(self):
        """سروس سے ڈیٹا تجزیہ کی درخواست کریں"""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Analysis service not available, waiting...')

        future = self.cli.call_async(Trigger.Request())
        future.add_done_callback(self.analysis_response_callback)

    def analysis_response_callback(self, future):
        """تجزیہ جواب کو ہینڈل کریں"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Analysis result: {response.message}')
            else:
                self.get_logger().warn(f'Analysis failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def run_communication_demo():
    """مکمل مواصلاتی سسٹم ڈیمو چلائیں"""
    rclpy.init()

    print("Starting ROS 2 Communication System Demo")
    print("=" * 45)

    # تمام نوڈس بنائیں
    sensor_node = SensorNode()
    processing_node = ProcessingNode()
    analysis_node = AnalysisServiceNode()
    client_node = ClientNode()

    try:
        print("Running communication system... Press Ctrl+C to stop")
        # تمام نوڈس چلانے کے لیے متعدد تھرڈڈ ایگزیکیوٹر استعمال کریں
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(sensor_node)
        executor.add_node(processing_node)
        executor.add_node(analysis_node)
        executor.add_node(client_node)

        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down communication system...")
    finally:
        sensor_node.destroy_node()
        processing_node.destroy_node()
        analysis_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_communication_demo()
```

## عام مواصلاتی مسائل کا حل

### 1. ٹاپک کنکشن مسائل

```python
# مسئلہ: نوڈس ٹاپکس سے جڑ نہیں سکتے
# حل: ٹاپک نامز اور QoS ترتیبات کی تصدیق کریں

def verify_topic_connection(node, topic_name, expected_type):
    """تصدیق کریں کہ ایک ٹاپک دستیاب اور قابل رسائی ہے"""
    # ٹاپک کے لیے پبلشرز کی فہرست حاصل کریں
    publishers = node.get_publishers_info_by_topic(topic_name)
    subscribers = node.get_subscriptions_info_by_topic(topic_name)

    print(f"Topic '{topic_name}' info:")
    print(f"  Publishers: {len(publishers)}")
    print(f"  Subscribers: {len(subscribers)}")

    for pub in publishers:
        print(f"    Publisher: {pub.node_name}, QoS: {pub.qos_profile}")

    return len(publishers) > 0 or len(subscribers) > 0
```

### 2. سروس کنکشن مسائل

```python
# مسئلہ: سروس کلائنٹ سروس سے جڑ نہیں سکتا
# حل: سروس دستیابی اور دوبارہ کوشش کا منطق چیک کریں

import time
from rclpy.task import Future

def call_service_with_retry(client, request, max_retries=5, delay=1.0):
    """دوبارہ کوشش کے منطق کے ساتھ سروس کال کریں"""
    for attempt in range(max_retries):
        try:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(request)

                # جواب کے لیے ٹائم آؤٹ کے ساتھ انتظار کریں
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 5.0:  # 5 سیکنڈ ٹائم آؤٹ
                        raise TimeoutError("Service call timed out")
                    time.sleep(0.1)

                return future.result()
            else:
                print(f"Service not available, attempt {attempt + 1}/{max_retries}")
                time.sleep(delay)
        except Exception as e:
            print(f"Service call failed (attempt {attempt + 1}): {e}")
            if attempt == max_retries - 1:
                raise
            time.sleep(delay)

    raise RuntimeError(f"Failed to call service after {max_retries} attempts")
```

### 3. میسج سیریلائزیشن مسائل

```python
# مسئلہ: میسج سیریلائزیشن/ڈی سیریلائزیشن کے ساتھ مسائل
# حل: شائع کرنے سے پہلے میسج مواد کی تصدیق کریں

def validate_message_content(msg):
    """شائع کرنے سے پہلے میسج مواد کی تصدیق کریں"""
    if hasattr(msg, 'data'):
        if isinstance(msg.data, (int, float)) and (msg.data != msg.data):  # NaN چیک کریں
            print("Warning: Message contains NaN value")
            return False
        if isinstance(msg.data, str) and len(msg.data) > 10000:  # ارbitrary بڑے سٹرنگ چیک
            print("Warning: Message string is very large")
            return False
    return True

def safe_publish(publisher, msg):
    """تصدیق کے ساتھ محفوظ طریقے سے ایک میسج شائع کریں"""
    if validate_message_content(msg):
        publisher.publish(msg)
    else:
        print("Message validation failed, not publishing")
```

## مزید سیکھنے کے لیے وسائل

- [ROS 2 ٹاپکس اور سروسز ٹیوٹوریل](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 میسج ٹائپس](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [ROS 2 کوالٹی آف سروس ترتیبات](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS 2 نوڈ ترقی گائیڈ](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Node.html)

## تعلیم دہندگان کے لیے

### تدریسی اہداف برائے اساتذہ
- طلباء کو سیکھائیں کہ مضبوط ROS 2 نوڈس کیسے بنائیں جن میں مناسب خامات کا انتظام اور صاف کرنا شامل ہو
- طلباء کو پبلشر-سبسکرائب اور سروس مواصلاتی نمونوں کے درمیان فرق سمجھنے میں مدد کریں
- حقیقی دنیا کی مثالوں کا استعمال کرتے ہوئے طلباء کو روبوٹک مواصلاتی سسٹم نافذ کرنا سکھائیں

### مواصلاتی نمونوں کے لیے جائزہ روبک
طلباء کو یہ کرنا چاہیے:
- مناسب شروع اور صاف کاری کے ساتھ کام کرنے والے ROS 2 نوڈس بنائیں (25%)
- مناسب میسج ٹائپس کے ساتھ پبلشر-سبسکرائب مواصلات نافذ کریں (35%)
- درخواست-جواب مواصلات کے لیے سروسز بنائیں اور استعمال کریں (25%)
- عام مواصلاتی مسائل کا حل تلاش کریں (15%)

### تدریسی تجاویز
- سروسز متعارف کرانے سے پہلے سادہ پبلشر-سبسکرائب مثالوں سے شروع کریں
- مختلف نمونوں کو ایک ساتھ کیسے کام کرتے ہیں یہ دکھانے کے لیے مواصلاتی سسٹم مشق استعمال کریں
- مناسب نوڈ شروع اور تباہی کی اہمیت پر زور دیں
- یہ دکھائیں کہ ہر مواصلاتی نمونہ کب سب سے مناسب ہے

### عام طالب علم چیلنجز
- پبلشر-سبسکرائب مواصلات کی غیر ہم وقت نوعیت کو سمجھنا
- نوڈ لائف سائیکل کا انتظام اور مناسب وسائل کی صاف کاری
- مختلف استعمال کے مواقع کے لیے ٹاپکس بمقابلہ سروسز کب استعمال کرنا ہے اس کے درمیان فرق کرنا
- نوڈس کے درمیان مواصلاتی مسائل کو ڈیبگ کرنا

### سہولت کے اختیارات
- بنیادی نوڈ ساختوں کے لیے اسٹارٹر کوڈ ٹیمپلیٹس فراہم کریں
- مواصلاتی تصورات کے ساتھ مشکل میں مبتلا طلباء کے لیے اضافی ڈیبگنگ مشقیں پیش کریں
- نوڈس کے درمیان میسج کے بہاؤ کو دیکھنے میں مدد کے لیے وژول ٹولز یا سیمولیٹر شامل کریں

## خلاصہ

پبلشر-سبسکرائب اور سروس نمونے ROS 2 میں مرکزی مواصلاتی میکنزم بناتے ہیں۔ نوڈس، ٹاپکس، اور سروسز کو مناسب طریقے سے نافذ کرنا سیکھنا تقسیم شدہ روبوٹک سسٹم بنانے کے لیے ضروری ہے۔ ان نمونوں کی لچک اجازت دیتی ہے کہ پیچیدہ، مربوط روبوٹک ایپلی کیشنز بنائیں جبکہ اجزاء کے درمیان کم جوڑاؤ برقرار رکھا جائے۔