---
sidebar_position: 10
title: "انٹیگریشن"
---

# انٹیگریشن

VLA (وژن لینگویج ایکشن) سسٹم کے تمام اجزا کو مربوط کرنا ایک اہم مرحلہ ہے۔ یہ سیکشن مختلف ماڈلز، ROS 2 نوڈس، اور ہارڈویئر کمپونینٹس کو ایک جامع سسٹم میں مربوط کرنے کے طریقے کی وضاحت کرتا ہے۔

## انٹیگریشن کے اہداف

- وژن، لینگویج، اور ایکشن سسٹم کو مربوط کرنا
- ریئل ٹائم ڈیٹا فلو کو یقینی بنانا
- سیفٹی اور سٹیبلٹی کو برقرار رکھنا
- کارکردگی کے معیار کو پورا کرنا

## کمپونینٹس کا انٹیگریشن

### 1. وژن کمپونینٹ

#### کام:
- کیمرہ فیڈ کو پروسیس کرنا
- آبجیکٹ ڈیٹیکشن اور کلاسیفکیشن
- 3D پوزیشننگ

#### ROS 2 انٹیگریشن:
- `vision_node` کو اسٹارٹ کرنا
- کیمرہ ٹاپکس کو سبسکرائب کرنا
- ڈیٹیکشن ٹاپکس کو پبلش کرنا

#### کوڈ مثال:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json

class VisionIntegrationNode(Node):
    def __init__(self):
        super().__init__('vision_integration_node')

        # سبسکرائبرز
        self.camera_subscriber = self.create_subscription(
            Image, 'camera/color/image_raw', self.camera_callback, 10
        )

        # پبلیشرز
        self.detection_publisher = self.create_publisher(
            String, 'vision_detections', 10
        )

        self.get_logger().info('Vision Integration Node Started')

    def camera_callback(self, msg):
        """کیمرہ فیڈ کو پروسیس کریں اور ڈیٹیکشنز کو پبلش کریں"""
        # یہاں وژن پروسیسنگ کوڈ ہوگا
        # ڈیٹیکشنز کو JSON فارمیٹ میں پبلش کریں
        pass
```

### 2. لینگویج کمپونینٹ

#### کام:
- اسپیچ ریکوگنیشن
- کمانڈ انٹرپری ٹیشن
- کنٹیکسٹ مینجمنٹ

#### ROS 2 انٹیگریشن:
- `language_node` کو اسٹارٹ کرنا
- آڈیو ٹاپکس کو سبسکرائب کرنا
- پارسڈ کمانڈز کو پبلش کرنا

#### کوڈ مثال:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class LanguageIntegrationNode(Node):
    def __init__(self):
        super().__init__('language_integration_node')

        # سبسکرائبرز
        self.text_command_subscriber = self.create_subscription(
            String, 'text_commands', self.text_command_callback, 10
        )

        # پبلیشرز
        self.parsed_command_publisher = self.create_publisher(
            String, 'parsed_commands', 10
        )

        self.get_logger().info('Language Integration Node Started')

    def text_command_callback(self, msg):
        """ٹیکسٹ کمانڈ کو پروسیس کریں اور پارس کریں"""
        # یہاں لینگویج پروسیسنگ کوڈ ہوگا
        # پارسڈ کمانڈز کو JSON فارمیٹ میں پبلش کریں
        pass
```

### 3. ایکشن کمپونینٹ

#### کام:
- ایکشن پلاننگ
- ٹریجکٹری جنریشن
- روبوٹ کنٹرول

#### ROS 2 انٹیگریشن:
- `action_node` کو اسٹارٹ کرنا
- ایکشن سیکوئنس کو سبسکرائب کرنا
- روبوٹ کنٹرول کمانڈز کو پبلش کرنا

#### کوڈ مثال:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ActionIntegrationNode(Node):
    def __init__(self):
        super().__init__('action_integration_node')

        # سبسکرائبرز
        self.action_sequence_subscriber = self.create_subscription(
            String, 'action_sequences', self.action_sequence_callback, 10
        )

        # پبلیشرز
        self.robot_command_publisher = self.create_publisher(
            String, 'robot_commands', 10
        )

        self.get_logger().info('Action Integration Node Started')

    def action_sequence_callback(self, msg):
        """ایکشن سیکوئنس کو پروسیس کریں اور روبوٹ کمانڈز جاری کریں"""
        # یہاں ایکشن پلاننگ کوڈ ہوگا
        # روبوٹ کمانڈز کو JSON فارمیٹ میں پبلش کریں
        pass
```

## فیوژن انجن

### کام:
- تمام کمپونینٹس کو مربوط کرنا
- کراس ماڈل کمیونیکیشن
- کنٹیکسٹ کے مطابق فیصلے

### ROS 2 انٹیگریشن:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class FusionIntegrationNode(Node):
    def __init__(self):
        super().__init__('fusion_integration_node')

        # سبسکرائبرز
        self.vision_subscriber = self.create_subscription(
            String, 'vision_detections', self.vision_callback, 10
        )
        self.language_subscriber = self.create_subscription(
            String, 'parsed_commands', self.language_callback, 10
        )

        # پبلیشرز
        self.action_sequence_publisher = self.create_publisher(
            String, 'action_sequences', 10
        )

        # سسٹم کی حالت کو ٹریک کریں
        self.vision_data = {}
        self.language_data = {}

        self.get_logger().info('Fusion Integration Node Started')

    def vision_callback(self, msg):
        """وژن ڈیٹا کو اپ ڈیٹ کریں"""
        try:
            self.vision_data = json.loads(msg.data)
            self.fuse_data()
        except Exception as e:
            self.get_logger().error(f'Error processing vision data: {e}')

    def language_callback(self, msg):
        """لینگویج ڈیٹا کو اپ ڈیٹ کریں"""
        try:
            self.language_data = json.loads(msg.data)
            self.fuse_data()
        except Exception as e:
            self.get_logger().error(f'Error processing language data: {e}')

    def fuse_data(self):
        """وژن اور لینگویج ڈیٹا کو مربوط کریں"""
        if self.vision_data and self.language_data:
            # ڈیٹا کو فیوژن کریں اور ایکشن سیکوئنس تیار کریں
            fused_data = self.create_action_sequence(
                self.vision_data, self.language_data
            )

            # ایکشن سیکوئنس کو پبلش کریں
            action_msg = String()
            action_msg.data = json.dumps(fused_data)
            self.action_sequence_publisher.publish(action_msg)

    def create_action_sequence(self, vision_data, language_data):
        """وژن اور لینگویج ڈیٹا سے ایکشن سیکوئنس تیار کریں"""
        # یہاں فیوژن الگورتھم ہوگا
        # کنٹیکسٹ کے مطابق ایکشن سیکوئنس تیار کریں
        return {
            'sequence_id': 'fused_sequence',
            'actions': [],
            'context': {
                'vision': vision_data,
                'language': language_data
            }
        }
```

## لانچ فائلیں

### vla_system.launch.py:
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # وژن نوڈ
        Node(
            package='vla_system',
            executable='vision_node',
            name='vision_node',
            parameters=[],
            output='screen'
        ),

        # لینگویج نوڈ
        Node(
            package='vla_system',
            executable='language_node',
            name='language_node',
            parameters=[],
            output='screen'
        ),

        # ایکشن نوڈ
        Node(
            package='vla_system',
            executable='action_node',
            name='action_node',
            parameters=[],
            output='screen'
        ),

        # فیوژن نوڈ
        Node(
            package='vla_system',
            executable='fusion_node',
            name='fusion_node',
            parameters=[],
            output='screen'
        )
    ])
```

## کنفیگریشن فائلیں

### params.yaml:
```yaml
vision_node:
  ros__parameters:
    confidence_threshold: 0.5
    nms_threshold: 0.4
    target_fps: 20

language_node:
  ros__parameters:
    confidence_threshold: 0.7
    max_command_length: 100
    context_window_size: 5

action_node:
  ros__parameters:
    max_planning_time: 5.0
    safety_margin: 0.1
    execution_timeout: 30.0

fusion_node:
  ros__parameters:
    fusion_frequency: 10.0
    context_timeout: 5.0
    confidence_threshold: 0.6
```

## ٹیسٹنگ اور والیڈیشن

### انٹیگریشن ٹیسٹس:
- کمپونینٹس کے مابین میسج فلو
- ڈیٹا سینکرونائزیشن
- کارکردگی کے معیار
- سیفٹی چیکس

### والیڈیشن میٹرکس:
- سسٹم ریسپانس ٹائم
- ڈیٹا ایکویسی
- کامیابی کی شرح
- سسٹم اسٹیبلٹی

## ڈیبگنگ اور مانیٹرنگ

### ٹوپک مانیٹرنگ:
```bash
# تمام ٹوپکس کو لسٹ کریں
ros2 topic list

# مخصوص ٹوپک کو مانیٹر کریں
ros2 topic echo /vision_detections

# ٹوپک کی معلومات حاصل کریں
ros2 topic info /parsed_commands
```

### نوڈ مانیٹرنگ:
```bash
# تمام نوڈس کو لسٹ کریں
ros2 node list

# مخصوص نوڈ کی معلومات حاصل کریں
ros2 node info /fusion_node
```

## کارکردگی کی بہتری

### لیٹنسی کم کرنا:
- میسج کیو کے سائز کو ایڈجسٹ کرنا
- تھریڈنگ کو بہتر بنانا
- ہارڈویئر ایکسلریشن

### ریسورس مینجمنٹ:
- میموری استعمال کو مانیٹر کرنا
- CPU استعمال کو اپٹیمائز کرنا
- GPU ریسورسز کا مناسب استعمال

یہ انٹیگریشن گائیڈ VLA سسٹم کے تمام اجزا کو کامیابی کے ساتھ مربوط کرنے کے لیے ایک کمپری ہینسیو اپروچ فراہم کرتا ہے۔