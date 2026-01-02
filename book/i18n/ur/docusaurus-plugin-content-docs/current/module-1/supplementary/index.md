---
title: ماڈیول 1 کا اضافی مواد
description: ماڈیول 1 کے لیے اضافی وسائل، حوالہ جات، اور جدید موضوعات
sidebar_position: 10
---

# ماڈیول 1 کا اضافی مواد

اس حصے میں اضافی وسائل، جدید موضوعات اور حوالہ جاتی مواد شامل ہے جو ROS 2 روبوٹک اعصابی نظام پر ماڈیول 1 کے بنیادی مواد کی تکمیل کرتا ہے۔ ان مواد کا استعمال ROS 2 کے تصورات کو گہرائی سے سمجھنے اور عام مسائل کو حل کرنے کے لیے کریں جن کا آپ سامنا کر سکتے ہیں۔

## اضافی وسائل

### سرکاری ROS 2 دستاویزات
- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### تجویز کردہ مطالعہ
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- "Effective Robotics Programming with ROS" by Anil Mahtani, Luis Sánchez Crespo, and Enrique Fernández Perdomo
- "ROS Robotics Projects" by Ramon Sanchez

### آن لائن وسائل
- [ROS Discourse Forum](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/questions/)
- [ROS Wiki](http://wiki.ros.org/)

## جدید موضوعات

### کوالٹی آف سروس (QoS) کی تفصیل

کوالٹی آف سروس کی سیٹنگز آپ کو یہ ترتیب دینے کی اجازت دیتی ہیں کہ نوڈز کے درمیان پیغامات کیسے پہنچائے جاتے ہیں۔ یہاں اہم QoS پالیسیاں ہیں:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example QoS configuration for sensor data
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Example QoS configuration for periodic status updates
status_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

### ایڈوانسڈ پیرامیٹر ہینڈلنگ

پیرامیٹرز کو گروپ کیا جا سکتا ہے اور پیرامیٹر ڈسکرپٹرز کا استعمال کرتے ہوئے توثیق کی جا سکتی ہے:

```python
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

# Declare a parameter with descriptor
descriptor = ParameterDescriptor(
    description='Maximum linear velocity for the robot',
    type=ParameterType.PARAMETER_DOUBLE,
    additional_constraints='Must be a positive value between 0.1 and 2.0',
    read_only=False
)

self.declare_parameter('max_linear_velocity', 0.5, descriptor)
```

### حسب ضرورت میسج کی اقسام (Custom Message Types)

خصوصی ڈیٹا کے لیے اپنی مرضی کے مطابق پیغام کی اقسام بنانا:

```python
# In your package's msg directory, create RobotStatus.msg:
# string robot_name
# float32 battery_level
# bool is_charging
# int32 error_code
# time last_update
```

## عام مسائل کا حل (Troubleshooting)

### نوڈ کمیونیکیشن کے مسائل

**مسئلہ**: نوڈز ایک دوسرے سے بات چیت نہیں کر سکتے۔
**حل**:
1. چیک کریں کہ نوڈز ایک ہی ROS ڈومین آئی ڈی پر ہیں۔
2. تصدیق کریں کہ ٹاپک/سروس کے نام بالکل ایک جیسے ہیں۔
3. یقینی بنائیں کہ پبلشر اور سبسکرائبر کے درمیان QoS سیٹنگز مطابقت رکھتی ہیں۔
4. تصدیق کریں کہ دونوں نوڈز چل رہے ہیں۔

### پیرامیٹر کنفیگریشن کے مسائل

**مسئلہ**: پیرامیٹرز صحیح طریقے سے لوڈ نہیں ہو رہے ہیں۔
**حل**:
1. تصدیق کریں کہ لانچ فائلوں میں پیرامیٹر کے نام نوڈ میں موجود ناموں سے مماثل ہیں۔
2. چیک کریں کہ پیرامیٹر فائلیں صحیح جگہ پر ہیں۔
3. یقینی بنائیں کہ پیرامیٹر فائل کا سنٹیکس درست ہے (YAML فارمیٹ)۔
4. تصدیق کریں کہ پیرامیٹرز لوڈ ہونے کے بعد نوڈز شروع ہوتے ہیں۔

### کارکردگی کے مسائل

**مسئلہ**: روبوٹ سسٹم آہستہ چل رہا ہے یا تاخیر کا سامنا کر رہا ہے۔
**حل**:
1. ہائی بینڈوڈتھ والے ٹاپکس کے لیے پیغام کی فریکوئنسی کم کریں۔
2. QoS سیٹنگز کو بہتر بنائیں (غیر اہم ڈیٹا کے لیے BEST_EFFORT استعمال کریں)۔
3. بیک وقت چلنے والے کال بیکس کی تعداد کو محدود کریں۔
4. رکاوٹوں کی نشاندہی کرنے کے لیے اپنے کوڈ کی پروفائلنگ کریں۔

## بہترین طریقے (Best Practices)

### کوڈ کی تنظیم
- ٹاپکس، سروسز اور پیرامیٹرز کے لیے نام رکھنے کے مستقل اصول استعمال کریں۔
- متعلقہ فعالیت کو منطقی نوڈز میں گروپ کریں۔
- کاروباری منطق (business logic) کو ROS مخصوص کوڈ سے الگ کریں۔
- نوڈز بناتے وقت وراثت (inheritance) پر ترکیب (composition) کو ترجیح دیں۔

### ایرر ہینڈلنگ
- کال بیکس میں ہمیشہ مناسب ایرر ہینڈلنگ نافذ کریں۔
- ان آپریشنز کے لیے try-catch بلاکس استعمال کریں جو ناکام ہو سکتے ہیں۔
- جب اجزاء ناکام ہو جائیں تو گریسی فل ڈیگریڈیشن (graceful degradation) نافذ کریں۔
- مناسب شدت کی سطح کے ساتھ غلطیوں کو لاگ کریں۔

### ٹیسٹنگ
- انفرادی فنکشنز کے لیے یونٹ ٹیسٹ لکھیں۔
- نوڈ مواصلات کے لیے انٹیگریشن ٹیسٹ بنائیں۔
- ٹیسٹنگ کے لیے نقلی ماحول (simulation environments) کا استعمال کریں۔
- خودکار ٹیسٹنگ کے لیے مسلسل انضمام (CI) نافذ کریں۔

## کوڈ ٹیمپلیٹس

### بنیادی نوڈ ٹیمپلیٹ

```python
#!/usr/bin/env python3
"""
Template for creating a basic ROS 2 node
"""

import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node_name')

        # Initialize node components here
        self.get_logger().info('Basic node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### پبلشر-سبسکرائبر ٹیمپلیٹ

```python
#!/usr/bin/env python3
"""
Template for a node with publisher and subscriber
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create subscriber
        self.subscriber = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10
        )

        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)

    def callback(self, msg):
        """Handle incoming messages"""
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        """Publish messages periodically"""
        msg = String()
        msg.data = 'Hello from communication node'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ڈویلپمنٹ ٹولز

### ضروری ROS 2 کمانڈز

```bash
# List all available topics
ros2 topic list

# Echo messages from a specific topic
ros2 topic echo /topic_name std_msgs/msg/String

# List all available services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty

# List all running nodes
ros2 node list

# View the graph of nodes and topics
rqt_graph

# Check parameter values
ros2 param list
ros2 param get /node_name parameter_name
```

### rqt کے ساتھ ڈیبگنگ

rqt سویٹ مختلف ڈیبگنگ ٹولز فراہم کرتا ہے:

- `rqt_graph`: نوڈ گراف کو وژولائز کریں
- `rqt_plot`: وقت کے ساتھ عددی اقدار (numeric values) کو پلاٹ کریں
- `rqt_console`: لاگ پیغامات کی نگرانی کریں
- `rqt_bag`: ڈیٹا کو ریکارڈ اور ری پلے کریں
- `rqt_reconfigure`: متحرک طور پر پیرامیٹرز تبدیل کریں

## فرہنگ (Glossary)

- **Node**: ایک عمل جو ROS میں کمپیوٹیشن کرتا ہے
- **Topic**: ایک نامزد بس جس پر نوڈز پیغامات کا تبادلہ کرتے ہیں
- **Message**: ایک ڈیٹا پیکٹ جو ایک ٹاپک پر نوڈز کے درمیان بھیجا جاتا ہے
- **Publisher**: ایک نوڈ جو کسی ٹاپک پر پیغامات بھیجتا ہے
- **Subscriber**: ایک نوڈ جو کسی ٹاپک سے پیغامات وصول کرتا ہے
- **Service**: ایک ہم وقت (synchronous) درخواست/جواب مواصلاتی پیٹرن
- **Action**: ایک غیر ہم وقت (asynchronous) مقصد پر مبنی مواصلاتی پیٹرن
- **Parameter**: کنفیگریشن ویلیو جو کسی نوڈ کے لیے قابل رسائی ہو
- **Launch File**: کنفیگریشن فائل جو ایک ساتھ متعدد نوڈز شروع کرتی ہے
- **Package**: ROS فعالیت کے لیے ایک کنٹینر
- **QoS**: کوالٹی آف سروس کی پالیسیاں جو پیغام کی ترسیل کی ضمانتوں کی وضاحت کرتی ہیں

## اکثر پوچھے گئے سوالات (FAQs)

**س: میں ایک ہی ROS نیٹ ورک میں متعدد روبوٹس کو کیسے ہینڈل کروں؟**
ج: ہر روبوٹ کے لیے مختلف ROS_DOMAIN_ID اقدار استعمال کریں، یا ٹاپکس اور سروسز کو الگ کرنے کے لیے namespace استعمال کریں۔

**س: سروسز اور ایکشنز میں کیا فرق ہے؟**
ج: سروسز synchronous ہوتی ہیں اور انہیں جلد مکمل ہونا چاہیے، جبکہ ایکشنز asynchronous ہوتے ہیں اور مکمل ہونے میں زیادہ وقت لے سکتے ہیں، اور عملدرآمد کے دوران فیڈ بیک فراہم کرتے ہیں۔

**س: میں اپنے نوڈز کو زیادہ موثر کیسے بنا سکتا ہوں؟**
ج: مناسب QoS سیٹنگز استعمال کریں، ہائی بینڈوڈتھ ڈیٹا کے لیے پیغام کی فریکوئنسی کو محدود کریں، اور غیر ضروری دوبارہ کوششوں سے بچنے کے لیے مناسب ایرر ہینڈلنگ نافذ کریں۔
