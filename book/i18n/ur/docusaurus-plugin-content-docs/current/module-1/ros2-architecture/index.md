---
title: ROS 2 آرکیٹیکچر اور بنیادی تصورات
description: ROS 2 کی بنیادی تعمیر اور اس کے بنیادی تصورات کو سمجھنا
sidebar_position: 2
---

# ROS 2 آرکیٹیکچر اور بنیادی تصورات

## سیکھنے کے اہداف
- ROS 2 کی بنیادی تعمیر کو سمجھیں
- کلیدی اجزاء اور ان کے کرداروں کی شناخت کریں
- ROS 1 اور ROS 2 کے درمیان فرق سیکھیں
- DDS-مبنی مواصلاتی لیئر کو پہچانیں
- سیکورٹی اور ریل ٹائم صلاحیتوں کی قدر کریں

## ROS 2 کا تعارف

ROS 2 (روبوٹ آپریٹنگ سسٹم 2) جدید روبوٹکس ایپلی کیشنز کے لیے تیار کردہ اگلی نسل کا روبوٹکس فریم ورک ہے جو ROS 1 کی حدود کو دور کرتا ہے جبکہ جدید روبوٹکس ایپلی کیشنز کے لیے بہتر صلاحیتیں فراہم کرتا ہے۔ ROS 1 کے برعکس، جس نے مرکزی ماسٹر نوڈ پر انحصار کیا، ROS 2 DDS (ڈیٹا ڈسٹری بیوشن سروس) معیار پر مبنی غیر مرکزی تعمیر کا استعمال کرتا ہے۔

### ROS 1 کے مقابلے کلیدی بہتریاں

1. **غیر مرکزی تعمیر**: ناکامی کا واحد نقطہ نہیں
2. **ریل ٹائم سپورٹ**: وقت کے اہم ایپلی کیشنز کے لیے تعینات رویہ
3. **سیکورٹی**: بلٹ ان توثیق اور انکرپشن
4. **متعدد روبوٹ سسٹم**: متعدد روبوٹ کوآرڈی نیشن کے لیے قدرتی سپورٹ
5. **پیشہ ورانہ انتظام**: پیداوار کے لیے تیار خصوصیات

## بنیادی تعمیر کے اجزاء

### 1. نوڈس

نوڈس ROS 2 میں بنیادی انجام دہندہ یونٹ ہیں۔ ہر نوڈ ایک عمل ہے جو مخصوص حسابات انجام دیتا ہے اور دیگر نوڈس کے ساتھ مواصلات کرتا ہے:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

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

### 2. DDS (ڈیٹا ڈسٹری بیوشن سروس)

DDS وہ مڈل ویئر ہے جو ROS 2 میں مواصلاتی لیئر فراہم کرتا ہے:

```python
# QoS (کوالٹی آف سروس) کی ترتیبات کو سمجھنا
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# سینسر ڈیٹا کے لیے (ریل ٹائم، کچھ پیغامات ضائع ہو سکتے ہیں)
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# اہم کمانڈز کے لیے (ضرور ترسیل، تاریخ محفوظ)
command_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

### 3. مواصلاتی نمونے

ROS 2 متعدد مواصلاتی نمونوں کی حمایت کرتا ہے:

#### پبلشر-سبسکرائب (ٹاپکس)
- غیر ہم وقت، کثیر سے کثیر مواصلات
- سینسر کے پڑھنے جیسے ڈیٹا اسٹریم کے لیے استعمال ہوتا ہے

#### سروس-کلائنٹ
- ہم وقت درخواست-جواب مواصلات
- اس اعمال کے لیے استعمال ہوتا ہے جن کی تصدیق کی ضرورت ہو

#### ایکشن سرور-کلائنٹ
- فیڈ بیک اور گول کے انتظام کے ساتھ غیر ہم وقت
- طویل مدتی کاموں کے لیے استعمال ہوتا ہے

## کلائنٹ لائبریری تعمیر کو سمجھنا

ROS 2 کلائنٹ لائبریریز (Python کے لیے rclpy، C++ کے لیے rclcpp) استعمال کرتا ہے جو بنیادی DDS اہتمام کے ساتھ مطابقت رکھتا ہے:

```python
# تعمیر کی وضاحت
"""
+-------------------+    +------------------+    +------------------+
|   ایپلی کیشن     |    |   کلائنٹ لائبریری |    |    DDS اہتمام      |
|   (صارف کوڈ)     | -> |   (rclpy)        | -> |   (FastDDS، وغیرہ)|
+-------------------+    +------------------+    +------------------+
| نوڈ کی تعریف     |    | ROS 2 کو ہینڈل کرنا|    | DDS ادارت       |
| پبلشرز          |    | امتصاص           |    | ادارت کا نظم کرنا |
| سبسکرائبزرز     |    | لائف سائیکل       |    | مواصلات         |
| سروسز            |    | انتظام           |    | پروٹوکولز        |
+-------------------+    +------------------+    +------------------+
"""
```

### لائف سائیکل نوڈس

ROS 2 بہتر وسائل کے انتظام کے لیے لائف سائیکل نوڈس متعارف کرتا ہے:

```python
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleMinimalNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_minimal_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up lifecycle node')
        return TransitionCallbackReturn.SUCCESS
```

## کوالٹی آف سروس (QoS) ترتیبات

QoS ترتیبات مواصلاتی رویے کو بہتر بنانے کی اجازت دیتی ہیں:

```python
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class QoSDemoNode(Node):
    def __init__(self):
        super().__init__('qos_demo_node')

        # مختلف استعمال کے مواقع کے لیے مختلف QoS پروفائلز

        # سینسر ڈیٹا: بہترین کوشش، متغیر
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # کمانڈ ڈیٹا: قابل اعتماد، دائمی
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # اسٹیٹس ڈیٹا: آخری رکھیں، قابل اعتماد
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # مختلف QoS کے ساتھ پبلشرز بنائیں
        self.sensor_pub = self.create_publisher(String, 'sensor_data', sensor_qos)
        self.command_pub = self.create_publisher(String, 'commands', command_qos)
        self.status_pub = self.create_publisher(String, 'status', status_qos)
```

## نیمسپیسز اور کمپوزیشنز

ROS 2 نیمسپیسز کے ذریعے سلسلہ وار تنظیم کی حمایت کرتا ہے:

```python
# نیمسپیس استعمال کی مثال
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self):
        # نیمسپیس کے ساتھ شروع کریں
        super().__init__('robot_controller', namespace='robot1')

        # ٹاپک نام ہوں گے: /robot1/joint_commands
        self.joint_pub = self.create_publisher(String, 'joint_commands', 10)

        # سروس نام ہوں گے: /robot1/get_robot_state
        self.srv = self.create_service(GetState, 'get_robot_state', self.state_callback)

def main(args=None):
    rclpy.init(args=args)

    # مختلف نیمسپیسز کے ساتھ متعدد روبوٹس
    robot1 = RobotNode()
    robot2 = RobotNode()  # الگ نیمسپیس میں ہو گا اگر مخصوص کیا گیا

    try:
        rclpy.spin(robot1)
    except KeyboardInterrupt:
        pass
    finally:
        robot1.destroy_node()
        rclpy.shutdown()
```

## سیکورٹی تعمیر

ROS 2 میں بلٹ ان سیکورٹی خصوصیات شامل ہیں:

```python
# سیکورٹی کنفیگریشن کی مثال (تصوراتی)
"""
سیکورٹی فائلز کی ساخت:
robot_security/
├── identities/
│   ├── robot1.cert.pem      # روبوٹ 1 سرٹیفکیٹ
│   ├── robot1.key.pem       # روبوٹ 1 نجی کلید
│   └── ca.cert.pem          # سرٹیفکیٹ اتھارٹی
├── permissions/
│   ├── robot1_permissions.xml  # روبوٹ 1 اجازتیں
│   └── robot2_permissions.xml  # روبوٹ 2 اجازتیں
└── governance.xml              # عالمی سیکورٹی پالیسی
"""
```

## ہاتھوں سے مشق: تعمیر کی تحقیق

بنیادی تعمیری تصورات کو ظاہر کرنے والا ایک سادہ ROS 2 سسٹم بنائیں۔

### مشق کی ضروریات
1. مختلف QoS ترتیبات کے ساتھ بات چیت کرنے والے متعدد نوڈس بنائیں
2. ایک سادہ پبلشر-سبسکرائب نمونہ نافذ کریں
3. نیمسپیس استعمال کا مظاہرہ کریں
4. نوڈ لائف سائیکل دکھائیں

### Python نافذ کاری

```python
#!/usr/bin/env python3
"""
ROS 2 تعمیر کی تحقیق
بنیادی تعمیری تصورات کا مظاہرہ کرتا ہے
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class ArchitectureDemoNode(Node):
    def __init__(self, node_name, namespace=None):
        super().__init__(node_name, namespace=namespace)

        # مختلف QoS ترتیبات کے ساتھ پبلشرز بنائیں
        self.best_effort_pub = self.create_publisher(
            String,
            'sensor_data',
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST
            )
        )

        self.reliable_pub = self.create_publisher(
            String,
            'command_data',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
        )

        # سبسکرائبزرز بنائیں
        self.sensor_sub = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.command_sub = self.create_subscription(
            String,
            'command_data',
            self.command_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )
        )

        # شائع کرنے کے لیے ٹائمر
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info(f'Architecture Demo Node initialized in namespace: {namespace or "default"}')

    def sensor_callback(self, msg):
        self.get_logger().info(f'Sensor received: {msg.data}')

    def command_callback(self, msg):
        self.get_logger().info(f'Command received: {msg.data}')

    def timer_callback(self):
        # سینسر ڈیٹا شائع کریں (بہترین کوشش)
        sensor_msg = String()
        sensor_msg.data = f'Sensor reading {self.counter} at {time.time()}'
        self.best_effort_pub.publish(sensor_msg)

        # کمانڈ ڈیٹا شائع کریں (قابل اعتماد)
        command_msg = String()
        command_msg.data = f'Command {self.counter}'
        self.reliable_pub.publish(command_msg)

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    print("Starting ROS 2 Architecture Demo")
    print("=" * 40)

    # مختلف نیمسپیسز میں نوڈس بنائیں
    node1 = ArchitectureDemoNode('demo_node_1', namespace='robot1')
    node2 = ArchitectureDemoNode('demo_node_2', namespace='robot2')

    try:
        print("Running nodes... Press Ctrl+C to stop")
        rclpy.spin_multi_threaded([node1, node2])
    except KeyboardInterrupt:
        print("\nShutting down nodes...")
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## عام تعمیر کے مسائل کا حل

### 1. نوڈ ڈسکوری مسائل

```python
# مسئلہ: نوڈس ایک دوسرے کو نہیں ڈھونڈ سکتے
# حل: نیٹ ورک کنفیگریشن اور ڈومین IDs چیک کریں

import os

# ROS ڈومین ID سیٹ کریں تاکہ مداخلت نہ ہو
os.environ['ROS_DOMAIN_ID'] = '42'  # مخصوص ڈومین ID استعمال کریں

# ڈومین ID کی تصدیق کریں
import rclpy
print(f"ROS Domain ID: {os.environ.get('ROS_DOMAIN_ID', 'Not set')}")
```

### 2. QoS مطابقت کے مسائل

```python
# مسئلہ: مطابقت نہ رکھنے والے QoS کے ساتھ پبلشرز اور سبسکرائبزرز
# حل: QoS مطابقت یقینی بنائیں

def check_qos_compatibility(publisher_qos, subscriber_qos):
    """
    چیک کریں کہ آیا پبلشر اور سبسکرائبر QoS ترتیبات مطابقت رکھتی ہیں
    """
    # سادہ مطابقت چیک
    if (publisher_qos.reliability == ReliabilityPolicy.RELIABLE and
        subscriber_qos.reliability == ReliabilityPolicy.BEST_EFFORT):
        print("Warning: Reliable publisher with best-effort subscriber - may lose messages")

    return True
```

### 3. وسائل کا نظم

```python
# مسئلہ: غیر منظم وسائل سے میموری لیکس
# حل: مناسب طریقے سے نوڈس کو تباہ کریں اور وسائل صاف کریں

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')
        self.timers = []
        self.subscribers = []
        self.publishers = []

    def destroy_node(self):
        # تمام وسائل صاف کریں
        for timer in self.timers:
            timer.destroy()
        for sub in self.subscribers:
            sub.destroy()
        for pub in self.publishers:
            pub.destroy()

        super().destroy_node()
```

## مزید سیکھنے کے لیے وسائل

- [ROS 2 ڈیزائن دستاویزات](https://design.ros2.org/)
- [DDS اسپیسیفکیشن](https://www.omg.org/spec/DDS/About-DDS/)
- [ROS 2 QoS ٹیوٹوریلز](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- "Programming Robots with ROS" by Morgan Quigley وغیرہ

## تعلیم دہندگان کے لیے

### تدریسی اہداف برائے اساتذہ
- ROS 2 تعمیری تصورات کو مختلف تکنیکی پس منظر کے طلباء کو سکھانے کا طریقہ سمجھیں
- ROS 1 اور ROS 2 کے درمیان وہ کلیدی تعمیری فروقات کی شناخت کریں جنہیں طلباء کو سمجھنا ضروری ہے
- تعمیری تصورات کو ظاہر کرنے والی مؤثر ہاتھوں سے مشقوں کی ترقی کریں جو مبتدئین کو تنگ نہ کریں

### تعمیر کی سمجھ کے لیے جائزہ روبک
طلباء کو یہ کرنا چاہیے:
- ROS 2 کی غیر مرکزی نوعیت vs. ROS 1 کی مرکزی تعمیر کی وضاحت کر سکیں (40%)
- بنیادی نوڈس، ٹاپکس، اور سروسز کو نافذ کر سکیں جو مواصلاتی نمونے کو ظاہر کریں (30%)
- مختلف استعمال کے مواقع کے لیے مناسب QoS ترتیبات لاگو کر سکیں (30%)

### تدریسی تجاویز
- طلباء کو تعمیری ڈائریم وژولائزیشن کا استعمال کر کے کلائنٹ لائبریری لیئر کو سمجھنے میں مدد کریں
- حقیقی دنیا کے اطلاقیوں میں مثالوں کے ساتھ QoS ترتیبات کی اہمیت کو زور دیں
- متعدد روبوٹ کے مناظر ناموں کے ساتھ نیمسپیس استعمال کا مظاہرہ کر کے عملی اطلاقیات دکھائیں

### عام طالب علم چیلنجز
- DDS کے طور پر مڈل ویئر کے تصور اور اس کے ذریعے مواصلات کو فعال کرنا سمجھنا
- مختلف QoS ترتیبات کی اہمیت کو سمجھنا اور جب ان کا استعمال کرنا ہے
- پبلشر-سبسکرائب، سروس-کلائنٹ، اور ایکشن-کلائنٹ نمونوں کے درمیان فرق کرنا

### سہولت کے اختیارات
- محدود لینکس تجربے والے طلباء کے لیے پیش ساز ROS 2 پیکجز فراہم کریں
- تقسیم شدہ سسٹم کے تصورات سے ناواقف طلباء کے لیے اضافی وسائل پیش کریں
- مختلف سیکھنے کے انداز کے لیے وژول ایڈز اور ڈائریمز شامل کریں

## خلاصہ

ROS 2 کی تعمیر ROS 1 سے ایک نمایاں ترقی کی نمائندگی کرتی ہے، DDS پر مبنی غیر مرکزی ڈیزائن، بہتر سیکورٹی، ریل ٹائم صلاحیتیں، اور بہتر قابلیت کے ساتھ۔ ان تعمیری تصورات کو سمجھنا مضبوط روبوٹک سسٹم بنانے کے لیے بنیادی ہے۔ QoS ترتیبات، نیمسپیسز، اور لائف سائیکل انتظام کی خصوصیات جٹل، حقیقی دنیا کے اطلاقیوں کے لیے ضروری لچک فراہم کرتی ہیں۔