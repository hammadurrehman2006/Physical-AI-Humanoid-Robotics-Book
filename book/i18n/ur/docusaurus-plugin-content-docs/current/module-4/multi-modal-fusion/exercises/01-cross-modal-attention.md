---
sidebar_position: 1
title: "ورکشاپ 4.1: کراس ماڈل اتنشن"
---

# ورکشاپ 4.1: کراس ماڈل اتنشن

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے کراس ماڈل اتنشن مکینزم کو نافذ کریں، جس سے مختلف ماڈلز کے درمیان موثر انٹرایکشن ممکن ہو۔

## ضروریات
- Python 3.10+
- PyTorch 1.12+
- ROS 2 ہمبل ہاکسبل
- NumPy 1.21+
- CUDA 11.8+ (اگر GPU استعمال کر رہے ہیں)

## ورکشاپ کے اقدامات

### اقدام 1: کراس ماڈل اتنشن ماڈل کو نافذ کریں
ایک نیا فائل `cross_modal_attention.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Dict, List, Tuple, Optional
import math

class CrossModalAttention(nn.Module):
    """وژن اور لینگویج ماڈلز کے درمیان کراس ماڈل اتنشن"""

    def __init__(self,
                 vision_dim: int = 512,
                 language_dim: int = 512,
                 fusion_dim: int = 512,
                 num_heads: int = 8):
        super(CrossModalAttention, self).__init__()

        self.vision_dim = vision_dim
        self.language_dim = language_dim
        self.fusion_dim = fusion_dim
        self.num_heads = num_heads
        self.head_dim = fusion_dim // num_heads

        assert self.head_dim * num_heads == fusion_dim, "Fusion dim must be divisible by num_heads"

        # لینئر لیئرز برائے Q, K, V
        self.vision_to_qkv = nn.Linear(vision_dim, fusion_dim * 3)
        self.language_to_qkv = nn.Linear(language_dim, fusion_dim * 3)

        # آؤٹ پٹ لیئر
        self.output_projection = nn.Linear(fusion_dim, fusion_dim)

        # ڈروپ آؤٹ
        self.dropout = nn.Dropout(0.1)

        # لےئر نارم
        self.norm_vision = nn.LayerNorm(fusion_dim)
        self.norm_language = nn.LayerNorm(fusion_dim)
        self.norm_fused = nn.LayerNorm(fusion_dim)

    def forward(self,
                vision_features: torch.Tensor,
                language_features: torch.Tensor) -> Dict[str, torch.Tensor]:
        """
        کراس ماڈل اتنشن کا فوروارڈ پاس

        Args:
            vision_features: [batch_size, num_vision_tokens, vision_dim]
            language_features: [batch_size, num_language_tokens, language_dim]

        Returns:
            Dictionary of fused features and attention weights
        """
        batch_size, num_vision_tokens, _ = vision_features.shape
        _, num_language_tokens, _ = language_features.shape

        # وژن فیچرز کو Q, K, V میں تبدیل کریں
        vision_qkv = self.vision_to_qkv(vision_features)
        vision_q, vision_k, vision_v = vision_qkv.chunk(3, dim=-1)

        # لینگویج فیچرز کو Q, K, V میں تبدیل کریں
        language_qkv = self.language_to_qkv(language_features)
        language_q, language_k, language_v = language_qkv.chunk(3, dim=-1)

        # Q, K, V کو ملٹی ہیڈ کے لیے ریشیپ کریں
        vision_q = vision_q.view(batch_size, num_vision_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        vision_k = vision_k.view(batch_size, num_vision_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        vision_v = vision_v.view(batch_size, num_vision_tokens, self.num_heads, self.head_dim).transpose(1, 2)

        language_q = language_q.view(batch_size, num_language_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        language_k = language_k.view(batch_size, num_language_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        language_v = language_v.view(batch_size, num_language_tokens, self.num_heads, self.head_dim).transpose(1, 2)

        # وژن سے لینگویج کی طرف اتنشن
        vision_to_language_attn = torch.matmul(vision_q, language_k.transpose(-2, -1))
        vision_to_language_attn = vision_to_language_attn / math.sqrt(self.head_dim)
        vision_to_language_attn = F.softmax(vision_to_language_attn, dim=-1)
        vision_to_language_attn = self.dropout(vision_to_language_attn)

        vision_to_language_output = torch.matmul(vision_to_language_attn, language_v)
        vision_to_language_output = vision_to_language_output.transpose(1, 2).contiguous().view(
            batch_size, num_vision_tokens, self.fusion_dim
        )

        # لینگویج سے وژن کی طرف اتنشن
        language_to_vision_attn = torch.matmul(language_q, vision_k.transpose(-2, -1))
        language_to_vision_attn = language_to_vision_attn / math.sqrt(self.head_dim)
        language_to_vision_attn = F.softmax(language_to_vision_attn, dim=-1)
        language_to_vision_attn = self.dropout(language_to_vision_attn)

        language_to_vision_output = torch.matmul(language_to_vision_attn, vision_v)
        language_to_vision_output = language_to_vision_output.transpose(1, 2).contiguous().view(
            batch_size, num_language_tokens, self.fusion_dim
        )

        # فیوژن: اصل فیچرز اور اٹینڈیڈ فیچرز کو جوڑیں
        fused_vision = self.norm_fused(
            vision_features + self.output_projection(vision_to_language_output)
        )
        fused_language = self.norm_fused(
            language_features + self.output_projection(language_to_vision_output)
        )

        return {
            'fused_vision': fused_vision,
            'fused_language': fused_language,
            'vision_to_language_attention': vision_to_language_attn,
            'language_to_vision_attention': language_to_vision_attn
        }

class MultiModalFusion(nn.Module):
    """ملٹی ماڈل فیوژن کے لیے کمپلیٹ ماڈل"""

    def __init__(self,
                 vision_dim: int = 512,
                 language_dim: int = 512,
                 action_dim: int = 256,
                 fusion_dim: int = 512,
                 num_heads: int = 8,
                 num_layers: int = 2):
        super(MultiModalFusion, self).__init__()

        self.vision_dim = vision_dim
        self.language_dim = language_dim
        self.action_dim = action_dim
        self.fusion_dim = fusion_dim
        self.num_layers = num_layers

        # انپٹ ایڈجسٹمنٹ لیئرز
        self.vision_projection = nn.Linear(vision_dim, fusion_dim)
        self.language_projection = nn.Linear(language_dim, fusion_dim)
        self.action_projection = nn.Linear(action_dim, fusion_dim)

        # کراس ماڈل اتنشن لیئرز
        self.cross_attention_layers = nn.ModuleList([
            CrossModalAttention(fusion_dim, fusion_dim, fusion_dim, num_heads)
            for _ in range(num_layers)
        ])

        # ایکشن جنریشن لیئر
        self.action_generator = nn.Sequential(
            nn.Linear(fusion_dim * 2, fusion_dim),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(fusion_dim, action_dim)
        )

        # کنفیڈنس اسکور لیئر
        self.confidence_predictor = nn.Linear(fusion_dim, 1)

    def forward(self,
                vision_features: torch.Tensor,
                language_features: torch.Tensor,
                previous_action: Optional[torch.Tensor] = None) -> Dict[str, torch.Tensor]:
        """
        ملٹی ماڈل فیوژن کا فوروارڈ پاس

        Args:
            vision_features: [batch_size, num_vision_tokens, vision_dim]
            language_features: [batch_size, num_language_tokens, language_dim]
            previous_action: [batch_size, action_dim] optional

        Returns:
            Dictionary of fused features and action predictions
        """
        batch_size = vision_features.size(0)

        # فیچرز کو فیوژن ڈائیمینشن میں پروجیکٹ کریں
        vision_proj = self.vision_projection(vision_features)
        language_proj = self.language_projection(language_features)

        # کراس اتنشن لیئرز کے ذریعے پاس کریں
        current_vision = vision_proj
        current_language = language_proj

        attention_weights = []

        for layer in self.cross_attention_layers:
            result = layer(current_vision, current_language)
            current_vision = result['fused_vision']
            current_language = result['fused_language']
            attention_weights.append({
                'vision_to_language': result['vision_to_language_attention'],
                'language_to_vision': result['language_to_vision_attention']
            })

        # وژن اور لینگویج فیچرز کو جوڑیں (پول کریں)
        vision_pooled = current_vision.mean(dim=1)  # [batch_size, fusion_dim]
        language_pooled = current_language.mean(dim=1)  # [batch_size, fusion_dim]

        # فیوژن فیچرز
        combined_features = torch.cat([vision_pooled, language_pooled], dim=-1)

        # ایکشن جنریٹ کریں
        predicted_action = self.action_generator(combined_features)

        # کنفیڈنس اسکور
        confidence = torch.sigmoid(self.confidence_predictor(vision_pooled))

        return {
            'predicted_action': predicted_action,
            'confidence': confidence,
            'fused_vision': current_vision,
            'fused_language': current_language,
            'attention_weights': attention_weights
        }

def test_cross_modal_attention():
    """کراس ماڈل اتنشن کو ٹیسٹ کریں"""
    # ماڈل تیار کریں
    model = MultiModalFusion(
        vision_dim=512,
        language_dim=512,
        action_dim=256,
        fusion_dim=512,
        num_heads=8,
        num_layers=2
    )

    # ٹیسٹ ڈیٹا تیار کریں
    batch_size = 4
    num_vision_tokens = 10
    num_language_tokens = 20

    vision_features = torch.randn(batch_size, num_vision_tokens, 512)
    language_features = torch.randn(batch_size, num_language_tokens, 512)

    # فارورڈ پاس
    result = model(vision_features, language_features)

    # چیک کریں کہ آؤٹ پٹس صحیح ہیں
    assert result['predicted_action'].shape == (batch_size, 256), "Action shape mismatch"
    assert result['confidence'].shape == (batch_size, 1), "Confidence shape mismatch"
    assert result['fused_vision'].shape == (batch_size, num_vision_tokens, 512), "Fused vision shape mismatch"
    assert result['fused_language'].shape == (batch_size, num_language_tokens, 512), "Fused language shape mismatch"

    print("Cross-modal attention test passed!")
    print(f"Predicted action shape: {result['predicted_action'].shape}")
    print(f"Confidence range: [{result['confidence'].min():.3f}, {result['confidence'].max():.3f}]")

if __name__ == "__main__":
    test_cross_modal_attention()
```

### اقدام 2: ROS 2 انٹیگریشن کو نافذ کریں
ایک فائل `multi_modal_fusion_node.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import torch
import numpy as np
import json
from cv_bridge import CvBridge
from typing import Dict, Any
import base64

class MultiModalFusionNode(Node):
    def __init__(self):
        super().__init__('multi_modal_fusion_node')

        # CV Bridge
        self.bridge = CvBridge()

        # سبسکرائبرز
        self.vision_subscriber = self.create_subscription(
            String, 'vision_detections', self.vision_callback, 10
        )
        self.language_subscriber = self.create_subscription(
            String, 'parsed_commands', self.language_callback, 10
        )
        self.action_subscriber = self.create_subscription(
            String, 'robot_state', self.robot_state_callback, 10
        )

        # پبلیشرز
        self.action_publisher = self.create_publisher(
            String, 'fused_actions', 10
        )
        self.fusion_status_publisher = self.create_publisher(
            String, 'fusion_status', 10
        )

        # فیوژن ماڈل
        self.fusion_model = self.load_fusion_model()

        # ڈیٹا کیش
        self.current_vision_data = None
        self.current_language_data = None
        self.current_robot_state = None

        # سینکرونائزیشن کا وقت
        self.sync_timeout = 2.0  # 2 سیکنڈ
        self.last_vision_time = 0.0
        self.last_language_time = 0.0

        self.get_logger().info('Multi-Modal Fusion Node initialized')

    def load_fusion_model(self):
        """فیوژن ماڈل لوڈ کریں"""
        try:
            # ہمارا کراس ماڈل اتنشن ماڈل لوڈ کریں
            model = MultiModalFusion(
                vision_dim=512,
                language_dim=512,
                action_dim=256,
                fusion_dim=512,
                num_heads=8,
                num_layers=2
            )

            # اگر ٹرینڈ ماڈل فائل دستیاب ہو تو لوڈ کریں
            # model.load_state_dict(torch.load('fusion_model.pth'))

            return model
        except Exception as e:
            self.get_logger().warn(f'Could not load fusion model: {e}')
            return None

    def vision_callback(self, msg: String):
        """وژن ڈیٹا کو وصول کریں"""
        try:
            vision_data = json.loads(msg.data)
            self.current_vision_data = vision_data
            self.last_vision_time = self.get_clock().now().nanoseconds / 1e9

            # اگر لینگویج ڈیٹا بھی دستیاب ہو تو فیوژن کریں
            if self.current_language_data:
                self.perform_fusion()

        except Exception as e:
            self.get_logger().error(f'Error processing vision data: {e}')

    def language_callback(self, msg: String):
        """لینگویج ڈیٹا کو وصول کریں"""
        try:
            language_data = json.loads(msg.data)
            self.current_language_data = language_data
            self.last_language_time = self.get_clock().now().nanoseconds / 1e9

            # اگر وژن ڈیٹا بھی دستیاب ہو تو فیوژن کریں
            if self.current_vision_data:
                self.perform_fusion()

        except Exception as e:
            self.get_logger().error(f'Error processing language data: {e}')

    def robot_state_callback(self, msg: String):
        """روبوٹ کی حالت کو وصول کریں"""
        try:
            robot_state = json.loads(msg.data)
            self.current_robot_state = robot_state
        except Exception as e:
            self.get_logger().error(f'Error processing robot state: {e}')

    def perform_fusion(self):
        """وژن اور لینگویج ڈیٹا کو مربوط کریں"""
        if not self.fusion_model or not self.current_vision_data or not self.current_language_data:
            return

        try:
            # وژن ڈیٹا کو پروسیس کریں
            vision_features = self.process_vision_data(self.current_vision_data)

            # لینگویج ڈیٹا کو پروسیس کریں
            language_features = self.process_language_data(self.current_language_data)

            # فیوژن ماڈل کے ذریعے پاس کریں
            with torch.no_grad():
                fusion_result = self.fusion_model(
                    vision_features.unsqueeze(0),  # بیچ ڈائیمینشن شامل کریں
                    language_features.unsqueeze(0)
                )

            # ایکشن کو پبلش کریں
            self.publish_fused_action(fusion_result)

            # فیوژن اسٹیٹس کو پبلش کریں
            self.publish_fusion_status(fusion_result)

        except Exception as e:
            self.get_logger().error(f'Error in fusion process: {e}')

    def process_vision_data(self, vision_data: Dict[str, Any]) -> torch.Tensor:
        """وژن ڈیٹا کو فیچر ٹینسر میں تبدیل کریں"""
        detections = vision_data.get('detections', [])

        if not detections:
            # خالی فیچر ٹینسر لوٹائیں
            return torch.zeros(10, 512)  # 10 ڈیٹیکشنز، 512 dim

        features = []
        for detection in detections[:10]:  # زیادہ سے زیادہ 10 ڈیٹیکشنز
            # ہر ڈیٹیکشن کے لیے فیچر تیار کریں
            feature = self.extract_detection_features(detection)
            features.append(feature)

        # کم ڈیٹیکشنز کے لیے پیڈ کریں
        while len(features) < 10:
            features.append(torch.zeros(512))

        return torch.stack(features)

    def extract_detection_features(self, detection: Dict[str, Any]) -> torch.Tensor:
        """ڈیٹیکشن سے فیچر نکالیں"""
        # یہاں آپ اصل فیچر ایکسٹریکشن لاگو کریں گے
        # مثال کے طور پر جنرک فیچر تیار کر رہے ہیں

        bbox = detection.get('bbox', {})
        class_name = detection.get('class_name', 'unknown')
        confidence = detection.get('confidence', 0.0)

        # باؤنڈنگ باکس کے فیچرز
        x_center = (bbox.get('x1', 0) + bbox.get('x2', 0)) / 2
        y_center = (bbox.get('y1', 0) + bbox.get('y2', 0)) / 2
        width = bbox.get('x2', 0) - bbox.get('x1', 0)
        height = bbox.get('y2', 0) - bbox.get('y1', 0)

        # کلاس نام کے لیے ایمبیڈنگ (سادہ مثال)
        class_embedding = hash(class_name) % 100  # جنرک ایمبیڈنگ

        # فیچر ویکٹر تیار کریں
        feature_vector = torch.zeros(512)
        feature_vector[0] = x_center / 640.0  # نارملائز کریں
        feature_vector[1] = y_center / 480.0  # نارملائز کریں
        feature_vector[2] = width / 640.0
        feature_vector[3] = height / 480.0
        feature_vector[4] = confidence
        feature_vector[5] = class_embedding / 100.0

        return feature_vector

    def process_language_data(self, language_data: Dict[str, Any]) -> torch.Tensor:
        """لینگویج ڈیٹا کو فیچر ٹینسر میں تبدیل کریں"""
        # یہاں آپ اصل ٹیکسٹ ایمبیڈنگ ماڈل استعمال کریں گے
        # مثال کے طور پر جنرک فیچر تیار کر رہے ہیں

        command_text = language_data.get('command', '')
        intent = language_data.get('intent', 'unknown')

        # جنرک فیچر ویکٹر
        features = torch.zeros(20, 512)  # 20 الفاظ، 512 dim

        # کمانڈ کے حروف کے اساس پر فیچر تیار کریں
        for i, char in enumerate(command_text[:20]):
            if i < 20:
                features[i, 0] = ord(char) / 255.0  # ASCII ویلیو
                features[i, 1] = i / 20.0  # پوزیشن

        return features

    def publish_fused_action(self, fusion_result: Dict[str, torch.Tensor]):
        """فیوژن کا نتیجہ پبلش کریں"""
        action_msg = String()

        action_data = {
            'action_vector': fusion_result['predicted_action'].squeeze(0).tolist(),
            'confidence': fusion_result['confidence'].squeeze(0).item(),
            'timestamp': self.get_clock().now().nanoseconds
        }

        action_msg.data = json.dumps(action_data)
        self.action_publisher.publish(action_msg)

    def publish_fusion_status(self, fusion_result: Dict[str, torch.Tensor]):
        """فیوژن کا اسٹیٹس پبلش کریں"""
        status_msg = String()

        status_data = {
            'status': 'fused',
            'confidence': fusion_result['confidence'].mean().item(),
            'timestamp': self.get_clock().now().nanoseconds
        }

        status_msg.data = json.dumps(status_data)
        self.fusion_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = MultiModalFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

# ہمیں CrossModalAttention اور MultiModalFusion کلاسز کو اوپر نافذ کرنا ہوگا
# یہاں ہم ان کو دوبارہ وضاحت کر رہے ہیں تاکہ فائل مکمل ہو
class CrossModalAttention(nn.Module):
    """وژن اور لینگویج ماڈلز کے درمیان کراس ماڈل اتنشن"""

    def __init__(self,
                 vision_dim: int = 512,
                 language_dim: int = 512,
                 fusion_dim: int = 512,
                 num_heads: int = 8):
        super(CrossModalAttention, self).__init__()

        self.vision_dim = vision_dim
        self.language_dim = language_dim
        self.fusion_dim = fusion_dim
        self.num_heads = num_heads
        self.head_dim = fusion_dim // num_heads

        assert self.head_dim * num_heads == fusion_dim, "Fusion dim must be divisible by num_heads"

        # لینئر لیئرز برائے Q, K, V
        self.vision_to_qkv = nn.Linear(vision_dim, fusion_dim * 3)
        self.language_to_qkv = nn.Linear(language_dim, fusion_dim * 3)

        # آؤٹ پٹ لیئر
        self.output_projection = nn.Linear(fusion_dim, fusion_dim)

        # ڈروپ آؤٹ
        self.dropout = nn.Dropout(0.1)

        # لےئر نارم
        self.norm_vision = nn.LayerNorm(fusion_dim)
        self.norm_language = nn.LayerNorm(fusion_dim)
        self.norm_fused = nn.LayerNorm(fusion_dim)

    def forward(self,
                vision_features: torch.Tensor,
                language_features: torch.Tensor) -> Dict[str, torch.Tensor]:
        """
        کراس ماڈل اتنشن کا فوروارڈ پاس
        """
        batch_size, num_vision_tokens, _ = vision_features.shape
        _, num_language_tokens, _ = language_features.shape

        # وژن فیچرز کو Q, K, V میں تبدیل کریں
        vision_qkv = self.vision_to_qkv(vision_features)
        vision_q, vision_k, vision_v = vision_qkv.chunk(3, dim=-1)

        # لینگویج فیچرز کو Q, K, V میں تبدیل کریں
        language_qkv = self.language_to_qkv(language_features)
        language_q, language_k, language_v = language_qkv.chunk(3, dim=-1)

        # Q, K, V کو ملٹی ہیڈ کے لیے ریشیپ کریں
        vision_q = vision_q.view(batch_size, num_vision_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        vision_k = vision_k.view(batch_size, num_vision_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        vision_v = vision_v.view(batch_size, num_vision_tokens, self.num_heads, self.head_dim).transpose(1, 2)

        language_q = language_q.view(batch_size, num_language_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        language_k = language_k.view(batch_size, num_language_tokens, self.num_heads, self.head_dim).transpose(1, 2)
        language_v = language_v.view(batch_size, num_language_tokens, self.num_heads, self.head_dim).transpose(1, 2)

        # وژن سے لینگویج کی طرف اتنشن
        vision_to_language_attn = torch.matmul(vision_q, language_k.transpose(-2, -1))
        vision_to_language_attn = vision_to_language_attn / math.sqrt(self.head_dim)
        vision_to_language_attn = F.softmax(vision_to_language_attn, dim=-1)
        vision_to_language_attn = self.dropout(vision_to_language_attn)

        vision_to_language_output = torch.matmul(vision_to_language_attn, language_v)
        vision_to_language_output = vision_to_language_output.transpose(1, 2).contiguous().view(
            batch_size, num_vision_tokens, self.fusion_dim
        )

        # لینگویج سے وژن کی طرف اتنشن
        language_to_vision_attn = torch.matmul(language_q, vision_k.transpose(-2, -1))
        language_to_vision_attn = language_to_vision_attn / math.sqrt(self.head_dim)
        language_to_vision_attn = F.softmax(language_to_vision_attn, dim=-1)
        language_to_vision_attn = self.dropout(language_to_vision_attn)

        language_to_vision_output = torch.matmul(language_to_vision_attn, vision_v)
        language_to_vision_output = language_to_vision_output.transpose(1, 2).contiguous().view(
            batch_size, num_language_tokens, self.fusion_dim
        )

        # فیوژن: اصل فیچرز اور اٹینڈیڈ فیچرز کو جوڑیں
        fused_vision = self.norm_fused(
            vision_features + self.output_projection(vision_to_language_output)
        )
        fused_language = self.norm_fused(
            language_features + self.output_projection(language_to_vision_output)
        )

        return {
            'fused_vision': fused_vision,
            'fused_language': fused_language,
            'vision_to_language_attention': vision_to_language_attn,
            'language_to_vision_attention': language_to_vision_attn
        }

class MultiModalFusion(nn.Module):
    """ملٹی ماڈل فیوژن کے لیے کمپلیٹ ماڈل"""

    def __init__(self,
                 vision_dim: int = 512,
                 language_dim: int = 512,
                 action_dim: int = 256,
                 fusion_dim: int = 512,
                 num_heads: int = 8,
                 num_layers: int = 2):
        super(MultiModalFusion, self).__init__()

        self.vision_dim = vision_dim
        self.language_dim = language_dim
        self.action_dim = action_dim
        self.fusion_dim = fusion_dim
        self.num_layers = num_layers

        # انپٹ ایڈجسٹمنٹ لیئرز
        self.vision_projection = nn.Linear(vision_dim, fusion_dim)
        self.language_projection = nn.Linear(language_dim, fusion_dim)
        self.action_projection = nn.Linear(action_dim, fusion_dim)

        # کراس ماڈل اتنشن لیئرز
        self.cross_attention_layers = nn.ModuleList([
            CrossModalAttention(fusion_dim, fusion_dim, fusion_dim, num_heads)
            for _ in range(num_layers)
        ])

        # ایکشن جنریشن لیئر
        self.action_generator = nn.Sequential(
            nn.Linear(fusion_dim * 2, fusion_dim),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(fusion_dim, action_dim)
        )

        # کنفیڈنس اسکور لیئر
        self.confidence_predictor = nn.Linear(fusion_dim, 1)

    def forward(self,
                vision_features: torch.Tensor,
                language_features: torch.Tensor,
                previous_action: Optional[torch.Tensor] = None) -> Dict[str, torch.Tensor]:
        """
        ملٹی ماڈل فیوژن کا فوروارڈ پاس
        """
        batch_size = vision_features.size(0)

        # فیچرز کو فیوژن ڈائیمینشن میں پروجیکٹ کریں
        vision_proj = self.vision_projection(vision_features)
        language_proj = self.language_projection(language_features)

        # کراس اتنشن لیئرز کے ذریعے پاس کریں
        current_vision = vision_proj
        current_language = language_proj

        attention_weights = []

        for layer in self.cross_attention_layers:
            result = layer(current_vision, current_language)
            current_vision = result['fused_vision']
            current_language = result['fused_language']
            attention_weights.append({
                'vision_to_language': result['vision_to_language_attention'],
                'language_to_vision': result['language_to_vision_attention']
            })

        # وژن اور لینگویج فیچرز کو جوڑیں (پول کریں)
        vision_pooled = current_vision.mean(dim=1)  # [batch_size, fusion_dim]
        language_pooled = current_language.mean(dim=1)  # [batch_size, fusion_dim]

        # فیوژن فیچرز
        combined_features = torch.cat([vision_pooled, language_pooled], dim=-1)

        # ایکشن جنریٹ کریں
        predicted_action = self.action_generator(combined_features)

        # کنفیڈنس اسکور
        confidence = torch.sigmoid(self.confidence_predictor(vision_pooled))

        return {
            'predicted_action': predicted_action,
            'confidence': confidence,
            'fused_vision': current_vision,
            'fused_language': current_language,
            'attention_weights': attention_weights
        }

if __name__ == '__main__':
    main()
```

### اقدام 3: فیوژن ٹیسٹنگ اور والیڈیشن کو نافذ کریں
ایک فائل `fusion_tester.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import numpy as np

class FusionTesterNode(Node):
    def __init__(self):
        super().__init__('fusion_tester_node')

        # سبسکرائبرز
        self.fusion_result_subscriber = self.create_subscription(
            String, 'fused_actions', self.fusion_result_callback, 10
        )
        self.fusion_status_subscriber = self.create_subscription(
            String, 'fusion_status', self.fusion_status_callback, 10
        )

        # ٹیسٹ متغیرات
        self.test_results = []
        self.test_start_time = None
        self.test_duration = 30.0  # 30 سیکنڈ کے لیے ٹیسٹ
        self.fusion_count = 0
        self.total_confidence = 0.0

        self.get_logger().info('Fusion Tester Node initialized')

    def fusion_result_callback(self, msg: String):
        """فیوژن کے نتائج کو وصول کریں اور ریکارڈ کریں"""
        try:
            result_data = json.loads(msg.data)

            self.fusion_count += 1
            confidence = result_data.get('confidence', 0.0)
            self.total_confidence += confidence

            # نتائج کو ریکارڈ کریں
            self.test_results.append({
                'timestamp': result_data.get('timestamp'),
                'confidence': confidence,
                'action_vector_norm': np.linalg.norm(result_data.get('action_vector', []))
            })

            self.get_logger().info(f'Fusion result received. Confidence: {confidence:.3f}')

        except Exception as e:
            self.get_logger().error(f'Error processing fusion result: {e}')

    def fusion_status_callback(self, msg: String):
        """فیوژن اسٹیٹس کو وصول کریں"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status', 'unknown')

            if status == 'fused':
                self.get_logger().info('Fusion successful')
            else:
                self.get_logger().warn(f'Fusion status: {status}')

        except Exception as e:
            self.get_logger().error(f'Error processing fusion status: {e}')

    def run_comprehensive_test(self):
        """کمپری ہینسیو ٹیسٹ چلائیں"""
        self.get_logger().info('Starting comprehensive fusion test...')
        self.test_start_time = time.time()

        # 30 سیکنڈ تک ٹیسٹ چلائیں
        while time.time() - self.test_start_time < self.test_duration:
            time.sleep(0.1)  # 100ms کے لیے انتظار کریں

        self.print_test_results()

    def print_test_results(self):
        """ٹیسٹ کے نتائج کو پرنٹ کریں"""
        total_time = time.time() - self.test_start_time
        avg_fusions_per_second = self.fusion_count / total_time if total_time > 0 else 0
        avg_confidence = self.total_confidence / self.fusion_count if self.fusion_count > 0 else 0

        self.get_logger().info('=== Multi-Modal Fusion Test Results ===')
        self.get_logger().info(f'Total test time: {total_time:.2f}s')
        self.get_logger().info(f'Total fusions: {self.fusion_count}')
        self.get_logger().info(f'Average fusions per second: {avg_fusions_per_second:.2f}')
        self.get_logger().info(f'Average confidence: {avg_confidence:.3f}')
        self.get_logger().info(f'Fusion success rate: {avg_confidence * 100:.1f}%')

        # کارکردگی کے اشاریے
        if avg_fusions_per_second >= 10:
            self.get_logger().info('✅ Performance: GOOD (>10 fusions/sec)')
        else:
            self.get_logger().info('⚠️ Performance: NEEDS IMPROVEMENT (<10 fusions/sec)')

        if avg_confidence >= 0.7:
            self.get_logger().info('✅ Confidence: GOOD (>0.7)')
        else:
            self.get_logger().info('⚠️ Confidence: NEEDS IMPROVEMENT (<0.7)')

def main(args=None):
    rclpy.init(args=args)
    tester = FusionTesterNode()

    # ٹیسٹ شروع کریں
    tester.run_comprehensive_test()

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- کراس ماڈل اتنشن کے لیے ماڈلز تیار کیے
- وژن اور لینگویج ڈیٹا کو مربوط کرنے کا طریقہ سیکھا
- ROS 2 میں فیوژن نوڈ نافذ کیا
- ٹیسٹنگ اور والیڈیشن کے لیے ٹیسٹر نوڈ تیار کیا

یہ کراس ماڈل اتنشن VLA سسٹم کے لیے اہم ہے جہاں مختلف ماڈلز کے درمیان مؤثر مواصلت کی ضرورت ہوتی ہے۔