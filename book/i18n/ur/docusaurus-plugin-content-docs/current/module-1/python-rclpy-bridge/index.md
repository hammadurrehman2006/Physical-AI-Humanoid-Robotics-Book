---
title: "Python-rclpy برج: Python کو ROS 2 سے جوڑنا"
description: rclpy کو ROS 2 سے Python ایپلی کیشنز کو جوڑنے کے لیے استعمال کرنا سمجھنا
sidebar_position: 5
---

# Python-rclpy برج: Python کو ROS 2 سے جوڑنا

## سیکھنے کے اہداف
- rclpy کلائنٹ لائبریری تعمیر سمجھیں
- Python میں ROS 2 نوڈس کی تخلیق میں مہارت حاصل کریں
- rclpy کا استعمال کرتے ہوئے پبلشرز، سبسکرائبزرز، سروسز، اور ایکشنز نافذ کریں
- پیرامیٹر مینجمنٹ اور نوڈ لائف سائیکل کے بارے میں سیکھیں
- کارکردہ Python-مبنی ROS 2 ایپلی کیشنز تیار کریں

## rclpy کا تعارف

rclpy ROS 2 کے لیے Python کلائنٹ لائبریری ہے جو Python ایپلی کیشنز اور ROS 2 مڈل ویئر کے درمیان انٹرفیس فراہم کرتا ہے۔ یہ Python ڈیولپرز کو ROS 2 نوڈس، پبلشرز، سبسکرائبزرز، سروسز، اور ایکشنز کو ایک Pythonic API کے ساتھ بنانے کی اجازت دیتا ہے۔

### rclpy تعمیر

rclpy لائبریری آپ کی Python ایپلی کیشن اور بنیادی ROS 2 کلائنٹ لائبریری (rcl) کے درمیان بیٹھتا ہے، جو DDS مڈل ویئر کے ساتھ انٹرفیس کرتا ہے:

```
+------------------+    +----------+    +----------+    +------------+
| Python ایپلی کیشن| -> | rclpy    | -> | rcl      | -> | DDS اہتمام |
| (صارف کوڈ)      |    | (Python) |    | (C)      |    | (FastDDS,  |
|                  |    |          |    |          |    | CycloneDDS)|
+------------------+    +----------+    +----------+    +------------+
```

### بنیادی rclpy تصورات

```python
import rclpy
from rclpy.node import Node

class BasicRclpyNode(Node):
    def __init__(self):
        # ایک نام کے ساتھ نوڈ کو شروع کریں
        super().__init__('basic_rclpy_node')

        # نوڈ کے لاگر کا استعمال کرتے ہوئے پیغامات لاگ کریں
        self.get_logger().info('Basic rclpy node initialized')

        # نوڈ کی گھڑی حاصل کریں
        current_time = self.get_clock().now()
        self.get_logger().info(f'Current time: {current_time}')

def main(args=None):
    # rclpy کو شروع کریں
    rclpy.init(args=args)

    # نوڈ بنائیں
    node = BasicRclpyNode()

    try:
        # کال بیکس کو پروسیس کرنے کے لیے نوڈ کو سپن کریں
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

## نوڈ تخلیق اور انتظام

### بنیادی نوڈ ساخت

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # نوڈ نام کے ساتھ والد کنسٹرکٹر کال کریں
        super().__init__('my_node')

        # اختیاری: نیمسپیس مخصوص کریں
        # super().__init__('my_node', namespace='robot1')

        # نوڈ شروع کاری کوڈ یہاں
        self.get_logger().info('MyNode initialized')

    def __del__(self):
        # صاف کاری کوڈ (اگر چہ destroy_node() کو صراحت کے ساتھ کال کیا جانا چاہیے)
        self.get_logger().info('MyNode destroyed')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # صراحت کے ساتھ صاف کاری
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### نوڈ پیرامیٹرز

پیرامیٹرز نوڈس کو رن ٹائم پر کنفیگر کرنے کی اجازت دیتے ہیں:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # ڈیفالٹ ویلیوز اور اقسام کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('use_camera', True)
        self.declare_parameter('sensor_topics', ['camera/image_raw', 'depth/image_raw'])

        # پیرامیٹر ویلیوز حاصل کریں
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.use_camera = self.get_parameter('use_camera').value
        self.sensor_topics = self.get_parameter('sensor_topics').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Use camera: {self.use_camera}')
        self.get_logger().info(f'Sensor topics: {self.sensor_topics}')

        # پیرامیٹر تبدیلیوں کے لیے کال بیک مقرر کریں
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """پیرامیٹر تبدیلیوں کو ہینڈل کریں۔"
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

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

# پیرامیٹر کال بیک کے لیے ضروری درآمد
from rclpy.parameter_service import SetParametersResult
```

## پبلشرز اور سبسکرائبزرز

### پبلشرز بنانا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # مختلف QoS ترتیبات کے ساتھ پبلشرز بنائیں
        self.string_pub = self.create_publisher(
            String,
            'string_topic',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.int_pub = self.create_publisher(
            Int32,
            'int_topic',
            10  # سادہ QoS کے لیے عدد استعمال کریں (depth=10)
        )

        self.float_pub = self.create_publisher(
            Float64,
            'sensor_data',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # میسجز کو م_PERIODICALLY_ شائع کرنے کے لیے ایک ٹائمر بنائیں
        self.counter = 0
        self.timer = self.create_timer(0.5, self.publish_messages)

        self.get_logger().info('Publisher node initialized')

    def publish_messages(self):
        """مختلف ٹاپکس پر میسجز شائع کریں۔"
        # سٹرنگ میسج شائع کریں
        string_msg = String()
        string_msg.data = f'Hello from Python: {self.counter}'
        self.string_pub.publish(string_msg)

        # عددی میسج شائع کریں
        int_msg = Int32()
        int_msg.data = self.counter
        self.int_pub.publish(int_msg)

        # فلوٹ میسج شائع کریں
        float_msg = Float64()
        float_msg.data = 3.14 * self.counter
        self.float_pub.publish(float_msg)

        self.counter += 1
        self.get_logger().info(f'Published messages with counter: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### سبسکرائبزرز بنانا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # مختلف میسج اقسام کے لیے سبسکرائبزرز بنائیں
        self.string_sub = self.create_subscription(
            String,
            'string_topic',
            self.string_callback,
            10
        )

        self.int_sub = self.create_subscription(
            Int32,
            'int_topic',
            self.int_callback,
            10
        )

        self.float_sub = self.create_subscription(
            Float64,
            'sensor_data',
            self.float_callback,
            10
        )

        # غیر استعمال شدہ متغیر وارننگس روکیں
        self.string_sub  # یہ سبسکرپشن کو زندہ رکھتا ہے
        self.int_sub
        self.float_sub

        self.get_logger().info('Subscriber node initialized')

    def string_callback(self, msg):
        """سٹرنگ میسجز کو ہینڈل کریں۔"
        self.get_logger().info(f'String message received: {msg.data}')

    def int_callback(self, msg):
        """عددی میسجز کو ہینڈل کریں۔"
        self.get_logger().info(f'Integer message received: {msg.data}')

    def float_callback(self, msg):
        """فلوٹ میسجز کو ہینڈل کریں۔"
        self.get_logger().info(f'Float message received: {msg.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## rclpy کے ساتھ سروسز اور ایکشنز

### سروسز بنانا

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, Trigger
from std_msgs.msg import String

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # ایک سادہ سروس بنائیں
        self.add_srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        # ایک ٹرگر سروس بنائیں
        self.trigger_srv = self.create_service(
            Trigger,
            'robot_trigger',
            self.trigger_callback
        )

        # ٹوپکس کا استعمال کرتے ہوئے کسٹم سروس-جیسی صلاحیت بنائیں
        self.command_sub = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        self.get_logger().info('Service node initialized')

    def add_callback(self, request, response):
        """دو اعداد کا مجموعہ سروس کی درخواست کو ہینڈل کریں۔"
        response.sum = request.a + request.b
        self.get_logger().info(f'Calculated {request.a} + {request.b} = {response.sum}')
        return response

    def trigger_callback(self, request, response):
        """ٹرگر سروس کی درخواست کو ہینڈل کریں۔"
        # کچھ کام کی شبیہہ بنائیں
        import time
        time.sleep(0.1)

        response.success = True
        response.message = f'Trigger activated at {self.get_clock().now().to_msg()}'
        self.get_logger().info(f'Trigger service called: {response.message}')
        return response

    def command_callback(self, msg):
        """کمانڈ ٹوپک کو ہینڈل کریں (سروسز کے متبادل کے طور پر)."
        self.get_logger().info(f'Command received: {msg.data}')
        # کمانڈ کو یہاں پروسیس کریں

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### rclpy کے ساتھ ایکشن سرورز بنانا

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server_node')

        # دوبارہ داخل ہونے والے کال بیک گروپ کے ساتھ ایکشن سرور بنائیں
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_action',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Action server node initialized')

    def goal_callback(self, goal_request):
        """گول کی درخواستوں کو قبول یا مسترد کریں۔"
        self.get_logger().info(f'Received goal: {goal_request.order}')

        # مناسب آرڈر ویلیوز والے گولز قبول کریں
        if 1 <= goal_request.order <= 20:
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f'Rejected goal with order: {goal_request.order}')
            return rclpy.action.server.GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """گول منسوخی کی درخواستوں کو ہینڈل کریں۔"
        self.get_logger().info('Goal cancellation requested')
        return rclpy.action.server.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """ایکشن گول کو انجام دیں۔"
        self.get_logger().info('Executing goal...')

        # فیڈ بیک اور نتیجہ شروع کریں
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # فیبونیچی ترتیب کا حساب لگائیں
        order = goal_handle.request.order
        sequence = [0, 1]

        # ابتدائی فیڈ بیک اگر آرڈر > 2 ہے تو بھیجیں
        if order > 2:
            for i in range(2, order):
                # منسوخی کے لیے چیک کریں
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled during execution')
                    result_msg.sequence = sequence
                    goal_handle.canceled()
                    return result_msg

                # اگلے فیبونیچی نمبر کا حساب لگائیں
                next_num = sequence[i-1] + sequence[i-2]
                sequence.append(next_num)

                # فیڈ بیک بھیجیں
                feedback_msg.sequence = sequence.copy()
                goal_handle.publish_feedback(feedback_msg)

                # کام کی شبیہہ بنائیں
                import time
                time.sleep(0.2)

        # مکمل ہونے سے پہلے منسوخی کے لیے چیک کریں
        if goal_handle.is_cancel_requested:
            result_msg.sequence = sequence
            goal_handle.canceled()
            return result_msg

        # کامیابی کے ساتھ مکمل کریں
        result_msg.sequence = sequence
        goal_handle.succeed()
        self.get_logger().info(f'Goal completed successfully: {result_msg.sequence}')

        return result_msg

def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## اعلی درجے کی rclpy خصوصیات

### ٹائمرز اور کال بیکس

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # پبلشر بنائیں
        self.publisher = self.create_publisher(String, 'timer_messages', 10)

        # مختلف اقسام کے ٹائمرز بنائیں
        self.rate_timer = self.create_timer(1.0, self.rate_callback)  # 1 Hz
        self.fast_timer = self.create_timer(0.1, self.fast_callback)  # 10 Hz
        self.slow_timer = self.create_timer(5.0, self.slow_callback)  # 0.2 Hz

        # میسجز کے لیے کاؤنٹر
        self.counters = {'rate': 0, 'fast': 0, 'slow': 0}

        self.get_logger().info('Timer node initialized')

    def rate_callback(self):
        """1 Hz پر کال کیا جاتا ہے۔"
        self.counters['rate'] += 1
        msg = String()
        msg.data = f'1Hz message #{self.counters["rate"]}'
        self.publisher.publish(msg)
        self.get_logger().info(f'1Hz: {msg.data}')

    def fast_callback(self):
        """10 Hz پر کال کیا جاتا ہے۔"
        self.counters['fast'] += 1
        self.get_logger().info(f'10Hz tick #{self.counters["fast"]}')

    def slow_callback(self):
        """0.2 Hz پر کال کیا جاتا ہے۔"
        self.counters['slow'] += 1
        self.get_logger().info(f'0.2Hz tick #{self.counters["slow"]}')

    def destroy_node(self):
        """ٹائمرز صاف کریں۔"
        # ٹائمرز خود بخود تباہ ہو جاتے ہیں جب نوڈ تباہ ہو جاتا ہے
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### کلائنٹ لائبریریز اور اعلی درجے کے نمونے

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from collections import deque

class AdvancedRobotNode(Node):
    def __init__(self):
        super().__init__('advanced_robot_node')

        # مختلف روبوٹ سسٹم کے لیے پبلشرز
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # سینسر ڈیٹا کے لیے سبسکرائبزرز
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # سینسر ڈیٹا اسٹور کریں
        self.scan_data = None
        self.recent_commands = deque(maxlen=10)

        # روبوٹ کنٹرول کے لیے ایک ٹائمر بنائیں
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Advanced robot node initialized')

    def scan_callback(self, msg):
        """لیزر اسکین ڈیٹا پروسیس کریں۔"
        self.scan_data = msg
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} ranges')

    def control_loop(self):
        """مرکزی روبوٹ کنٹرول لوپ۔"
        if self.scan_data is None:
            return

        # سادہ رکاوٹ سے بچاؤ
        safe_distance = 1.0
        min_range = min([r for r in self.scan_data.ranges if not math.isinf(r) and not math.isnan(r)], default=float('inf'))

        cmd_vel = Twist()

        if min_range < safe_distance:
            # رکاوٹ کا پتہ چلا، بائیں طرف مڑیں
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # دائیں طرف مڑیں
            status = "Obstacle detected, turning"
        else:
            # صاف راستہ، آگے بڑھیں
            cmd_vel.linear.x = 0.5  # آگے بڑھیں
            cmd_vel.angular.z = 0.0
            status = "Moving forward"

        # کمانڈ شائع کریں
        self.cmd_vel_pub.publish(cmd_vel)
        self.recent_commands.append(cmd_vel)

        # حیثیت شائع کریں
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Control: {status}, Lin: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## ہاتھوں سے مشق: Python-rclpy انضمام

مختلف rclpy خصوصیات کو ایک ساتھ کام کرتا ہوا دکھانے والی ایک جامع مثال بنائیں۔

### مشق کی ضروریات
1. ایک نوڈ بنائیں جس میں پیرامیٹرز، پبلشرز، اور سبسکرائبزرز ہوں
2. سروس اور ایکشن صلاحیت نافذ کریں
3. مناسب وسائل کا انتظام دکھائیں
4. خامات کا انتظام اور لاگنگ دکھائیں

### Python نافذ کاری

```python
#!/usr/bin/env python3
"""
جامع rclpy انضمام مثال
ایک ہی ایپلی کیشن میں مختلف rclpy خصوصیات کا مظاہرہ کرتا ہے
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci

import time
import random
from enum import Enum
from collections import deque

class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    PROCESSING = 2
    ERROR = 3

class ComprehensiveRclpyNode(Node):
    def __init__(self):
        super().__init__('comprehensive_rclpy_node')

        # ڈیفالٹ ویلیوز کے ساتھ پیرامیٹرز کا اعلان کریں
        self.declare_parameter('robot_name', 'rclpy_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_frequency', 10)
        self.declare_parameter('log_level', 'info')

        # پیرامیٹر ویلیوز حاصل کریں
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.sensor_frequency = self.get_parameter('sensor_frequency').value

        self.get_logger().info(f'Initialized {self.robot_name} with max speed {self.max_speed}')

        # پبلشرز
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.sensor_pub = self.create_publisher(Float64, 'sensor_data', 10)
        self.log_pub = self.create_publisher(String, 'system_log', 10)

        # سبسکرائبزرز
        self.command_sub = self.create_subscription(
            String, 'robot_commands', self.command_callback, 10)

        # سروسز
        self.calc_srv = self.create_service(
            AddTwoInts, 'calculate_sum', self.calculate_callback)

        # ایکشن سرور
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'compute_fibonacci',
            execute_callback=self.fibonacci_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.fibonacci_goal_callback
        )

        # داخلی حیثیت
        self.current_state = RobotState.IDLE
        self.sensor_timer = self.create_timer(
            1.0 / self.sensor_frequency, self.sensor_update)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # ڈیٹا اسٹوریج
        self.sensor_history = deque(maxlen=100)
        self.command_history = deque(maxlen=50)

        self.get_logger().info('Comprehensive rclpy node initialized successfully')

    def command_callback(self, msg):
        """روبوٹ کمانڈز کو ہینڈل کریں۔"
        command = msg.data.lower()
        self.command_history.append(command)

        self.get_logger().info(f'Received command: {command}')

        if command == 'start':
            self.current_state = RobotState.MOVING
            self.log_event(f'Start command received, state: {self.current_state.name}')
        elif command == 'stop':
            self.current_state = RobotState.IDLE
            self.log_event(f'Stop command received, state: {self.current_state.name}')
        elif command.startswith('speed'):
            try:
                parts = command.split()
                if len(parts) >= 2:
                    new_speed = float(parts[1])
                    if 0 <= new_speed <= self.max_speed * 2:  # کچھ اوور اسپیڈ کی اجازت دیں
                        self.max_speed = new_speed
                        self.log_event(f'Speed changed to {new_speed}')
                    else:
                        self.get_logger().warn(f'Invalid speed: {new_speed}')
            except ValueError:
                self.get_logger().warn(f'Invalid speed command: {command}')

    def calculate_callback(self, request, response):
        """حساب کی سروس کی درخواست کو ہینڈل کریں۔"
        try:
            result = request.a + request.b
            response.sum = result
            self.log_event(f'Calculated {request.a} + {request.b} = {result}')
            self.get_logger().info(f'Service calculation: {request.a} + {request.b} = {result}')
        except Exception as e:
            self.get_logger().error(f'Calculation error: {e}')
            response.sum = 0  # خامات پر ڈیفالٹ ویلیو

        return response

    def fibonacci_goal_callback(self, goal_request):
        """فیبونیچی گول کی درخواست کو ہینڈل کریں۔"
        self.get_logger().info(f'Fibonacci goal requested: order={goal_request.order}')

        # گول کی تصدیق کریں
        if 1 <= goal_request.order <= 30:  # مناسب حد
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f'Invalid Fibonacci order: {goal_request.order}')
            return rclpy.action.server.GoalResponse.REJECT

    async def fibonacci_callback(self, goal_handle):
        """فیبونیچی ایکشن انجام دیں۔"
        self.get_logger().info('Executing Fibonacci action...')

        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        order = goal_handle.request.order
        sequence = []

        # فیبونیچی ترتیب کا حساب لگائیں
        for i in range(order):
            try:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Fibonacci action cancelled')
                    result_msg.sequence = sequence
                    goal_handle.canceled()
                    return result_msg

                if i == 0:
                    sequence.append(0)
                elif i == 1:
                    sequence.append(1)
                else:
                    sequence.append(sequence[i-1] + sequence[i-2])

                # ہر چند اسٹیپس پر فیڈ بیک شائع کریں
                if i % 2 == 0 or i == order - 1:
                    feedback_msg.sequence = sequence.copy()
                    goal_handle.publish_feedback(feedback_msg)

                # حساب کا وقت کی شبیہہ بنائیں
                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f'Fibonacci calculation error at step {i}: {e}')
                result_msg.sequence = sequence
                goal_handle.abort()
                return result_msg

        # کامیابی کے ساتھ مکمل کریں
        result_msg.sequence = sequence
        goal_handle.succeed()
        self.get_logger().info(f'Fibonacci completed: {sequence}')

        return result_msg

    def sensor_update(self):
        """سینسر ڈیٹا اپ ڈیٹس کی شبیہہ بنائیں۔"
        try:
            # شبیہ سینسر ڈیٹا جنریٹ کریں
            sensor_value = random.uniform(0.0, 10.0)

            # سینسر ڈیٹا شائع کریں
            sensor_msg = Float64()
            sensor_msg.data = sensor_value
            self.sensor_pub.publish(sensor_msg)

            # تاریخ میں اسٹور کریں
            self.sensor_history.append(sensor_value)

            # اگر ویلیو عجیب ہے تو لاگ کریں
            if sensor_value > 8.0:
                self.log_event(f'High sensor reading: {sensor_value:.2f}')

        except Exception as e:
            self.get_logger().error(f'Sensor update error: {e}')
            self.current_state = RobotState.ERROR

    def publish_status(self):
        """روبوٹ کی حیثیت شائع کریں۔"
        try:
            status_msg = String()
            status_msg.data = f'{self.robot_name}: state={self.current_state.name}, speed={self.max_speed:.2f}, sensors={len(self.sensor_history)}'
            self.status_pub.publish(status_msg)

            # حالات کے مطابق حیثیت کو اپ ڈیٹ کریں
            if self.current_state == RobotState.MOVING:
                # حرکت کے مکمل ہونے کی شبیہہ بنائیں
                if random.random() < 0.1:  # 10% امکان آرام پر واپس جانے کا
                    self.current_state = RobotState.IDLE
                    self.log_event('Movement completed')

        except Exception as e:
            self.get_logger().error(f'Status update error: {e}')

    def log_event(self, message):
        """سسٹم لاگ ٹوپک پر ایک واقعہ لاگ کریں۔"
        try:
            log_msg = String()
            log_msg.data = f'[{self.get_clock().now().seconds_nanoseconds()}] {message}'
            self.log_pub.publish(log_msg)
        except Exception as e:
            self.get_logger().error(f'Log publication error: {e}')

def run_comprehensive_demo():
    """جامع rclpy ڈیمو چلائیں۔"
    rclpy.init()

    print("Starting Comprehensive rclpy Demo")
    print("=" * 40)

    # نوڈ بنائیں
    node = ComprehensiveRclpyNode()

    # سروسز کو ٹیسٹ کرنے کے لیے ایک کلائنٹ بنائیں (ایک الگ تھریڈ میں)
    import threading

    def test_services():
        """ایک الگ تھریڈ میں سروسز کو ٹیسٹ کریں۔"
        time.sleep(2)  # نوڈ کو شروع ہونے کا انتظار کریں

        # کلائنٹ آپریشنز کے لیے ایک عارضی نوڈ بنائیں
        rclpy_client = rclpy.create_node('test_client')

        # ٹیسٹ سروس
        cli = rclpy_client.create_client(AddTwoInts, 'calculate_sum')
        while not cli.wait_for_service(timeout_sec=1.0):
            rclpy_client.get_logger().info('Service not available, waiting...')

        # درخواست بھیجیں
        req = AddTwoInts.Request()
        req.a = 5
        req.b = 7
        future = cli.call_async(req)

        try:
            rclpy.spin_until_future_complete(rclpy_client, future, timeout_sec=5.0)
            response = future.result()
            if response:
                print(f"Service test result: 5 + 7 = {response.sum}")
        except Exception as e:
            print(f"Service call failed: {e}")

        # ٹوپک کے ذریعے کچھ کمانڈز بھیجیں
        cmd_pub = rclpy_client.create_publisher(String, 'robot_commands', 10)
        time.sleep(1)

        commands = ['start', 'speed 2.5', 'stop']
        for cmd in commands:
            msg = String()
            msg.data = cmd
            cmd_pub.publish(msg)
            print(f"Sent command: {cmd}")
            time.sleep(1)

        rclpy_client.destroy_node()

    # پس منظر میں سروس ٹیسٹنگ شروع کریں
    test_thread = threading.Thread(target=test_services, daemon=True)
    test_thread.start()

    try:
        # تمام کال بیکس کو ہینڈل کرنے کے لیے متعدد تھریڈڈ ایگزیکیوٹر استعمال کریں
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        print("Running comprehensive demo... Press Ctrl+C to stop")
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutting down comprehensive demo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_comprehensive_demo()
```

## عام rclpy مسائل کا حل

### 1. درآمد اور انسٹالیشن کے مسائل

```python
# مسئلہ: rclpy نہیں ملا یا درآمد کے خامات
# حل: انسٹالیشن اور Python ماحول کی تصدیق کریں

def check_rclpy_installation():
    """چیک کریں کہ آیا rclpy مناسب طریقے سے انسٹال ہے۔"
    try:
        import rclpy
        print(f"✓ rclpy version: {rclpy.__version__}")
        print(f"✓ rclpy location: {rclpy.__file__}")
        return True
    except ImportError as e:
        print(f"✗ rclpy import failed: {e}")
        print("Make sure ROS 2 is sourced and rclpy is installed")
        print("Source ROS 2: source /opt/ros/humble/setup.bash")
        return False

def check_ros_environment():
    """ROS 2 ماحولیاتی متغیرات چیک کریں۔"
    import os

    ros_distro = os.environ.get('ROS_DISTRO')
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', 'Not set')

    print(f"ROS_DISTRO: {ros_distro}")
    print(f"ROS_DOMAIN_ID: {ros_domain_id}")

    if not ros_distro or ros_distro != 'humble':
        print("Warning: ROS_DISTRO is not set to 'humble'")

    return bool(ros_distro)
```

### 2. نوڈ لائف سائیکل اور وسائل کا انتظام

```python
# مسئلہ: میموری لیکس یا وسائل صاف نہیں ہوتے
# حل: مناسب نوڈ تباہی

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')

        # تمام بنائے گئے آبجیکٹس کا حوالہ رکھیں
        self.publishers = []
        self.subscribers = []
        self.services = []
        self.action_servers = []
        self.timers = []

        # وسائل بنائیں
        self._create_resources()

    def _create_resources(self):
        """تمام ضروری وسائل بنائیں۔"
        # پبلشرز
        pub = self.create_publisher(String, 'test_topic', 10)
        self.publishers.append(pub)

        # سبسکرائبزرز
        sub = self.create_subscription(String, 'test_topic', lambda msg: None, 10)
        self.subscribers.append(sub)

        # ٹائمرز
        timer = self.create_timer(1.0, lambda: None)
        self.timers.append(timer)

    def destroy_node(self):
        """تمام وسائل کو مناسب طریقے سے صاف کریں۔"
        self.get_logger().info('Cleaning up resources...')

        # پہلے ٹائمرز تباہ کریں
        for timer in self.timers:
            timer.destroy()

        # سبسکرائبزرز تباہ کریں
        for sub in self.subscribers:
            sub.destroy()

        # پبلشرز تباہ کریں
        for pub in self.publishers:
            pub.destroy()

        # حوالہ جات صاف کریں
        self.timers.clear()
        self.subscribers.clear()
        self.publishers.clear()

        # والد تباہ کاری کال کریں
        super().destroy_node()
        self.get_logger().info('Node destroyed successfully')
```

### 3. تھریڈنگ اور متزامن مسائل

```python
# مسئلہ: rclpy کے ساتھ تھریڈنگ کے مسائل
# حل: مناسب ایگزیکیوٹرز اور کال بیک گروپس استعمال کریں

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ThreadSafeNode(Node):
    def __init__(self):
        super().__init__('thread_safe_node')

        # ان کال بیکس کے لیے دوبارہ داخل ہونے والے کال بیک گروپ استعمال کریں جو دیگر کال بیکس کو کال کر سکتے ہیں
        reentrant_group = ReentrantCallbackGroup()

        # ان کال بیکس کے لیے میوچلی ایکسکلوزو گروپ استعمال کریں جو ہم وقت نہ چلنا چاہیے
        exclusive_group = MutuallyExclusiveCallbackGroup()

        # دوبارہ داخل ہونے والے گروپ کے ساتھ ایکشن سرور بنائیں
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'safe_fibonacci',
            execute_callback=self.safe_execute_callback,
            callback_group=reentrant_group
        )

        # انحصاری گروپ کے ساتھ سروس بنائیں
        self._service = self.create_service(
            AddTwoInts,
            'safe_service',
            self.safe_service_callback,
            callback_group=exclusive_group
        )

    def safe_execute_callback(self, goal_handle):
        """تھریڈ-محفوظ ایکشن انجام دہی۔"
        # یہاں نافذ کاری
        result_msg = Fibonacci.Result()
        result_msg.sequence = [1, 1, 2, 3, 5]  # مثال
        goal_handle.succeed()
        return result_msg

    def safe_service_callback(self, request, response):
        """تھریڈ-محفوظ سروس کال بیک۔"
        response.sum = request.a + request.b
        return response
```

## کارکردگی کی بہتری کے نکات

### کارکردہ میسج ہینڈلنگ

```python
# زیادہ فریکوینسی ٹوپکس کے لیے میسج ہینڈلنگ کو بہتر بنائیں
class OptimizedNode(Node):
    def __init__(self):
        super().__init__('optimized_node')

        # زیادہ فریکوینسی ٹوپکس کے لیے مناسب QoS استعمال کریں
        high_freq_qos = QoSProfile(
            depth=1,  # صرف تازہ ترین میسج رکھیں
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.high_freq_sub = self.create_subscription(
            String, 'high_freq_topic', self.high_freq_callback, high_freq_qos)

    def high_freq_callback(self, msg):
        """زیادہ فریکوینسی میسجز کے لیے بہتر کال بیک۔"
        # میسج کو کارکردہ طریقے سے پروسیس کریں
        # زیادہ فریکوینسی کال بیکس میں بھاری کمپیوٹیشن سے گریز کریں
        pass
```

## مزید سیکھنے کے لیے وسائل

- [rclpy دستاویزات](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Python ٹیوٹوریلز](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 پیرامیٹر گائیڈ](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-Python.html)
- [ROS 2 کوالٹی آف سروس گائیڈ](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## خلاصہ

rclpy کلائنٹ لائبریری ROS 2 کے لیے ایک جامع Python انٹرفیس فراہم کرتا ہے، جو Python ڈیولپرز کو ترقی یافتہ روبوٹک ایپلی کیشنز بنانے کی اجازت دیتا ہے۔ نوڈ تخلیق، پیرامیٹر مینجمنٹ، پبلشر/سبسکرائب نمونے، اور سروس/ایکشن نافذ کاری کو سمجھنا ROS 2 سسٹم بنانے کے لیے اہم ہے۔ مناسب وسائل کا انتظام، خامات کا انتظام، اور کارکردگی کی بہتری ROS 2 مبنی Python ایپلی کیشنز کو مضبوط اور کارکردہ بناتا ہے۔