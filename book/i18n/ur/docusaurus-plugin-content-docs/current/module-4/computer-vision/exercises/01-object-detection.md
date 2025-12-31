---
sidebar_position: 1
title: "ورکشاپ 4.1: آبجیکٹ ڈیٹیکشن اور کلاسیفکیشن"
---

# ورکشاپ 4.1: آبجیکٹ ڈیٹیکشن اور کلاسیفکیشن

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے کمپیوٹر وژن کوڈ کو نافذ کریں، جس میں آبجیکٹ ڈیٹیکشن، کلاسیفکیشن، اور سپیشل اندراج شامل ہے۔

## ضروریات
- Python 3.10+
- ROS 2 ہمبل ہاکسبل
- OpenCV 4.5+
- PyTorch 1.12+
- CUDA 11.8+ (اگر GPU استعمال کر رہے ہیں)
- RGB-D کیمرہ

## ورکشاپ کے اقدامات

### اقدام 1: ROS 2 وژن نوڈ تیار کریں
ایک نیا فائل `vision_node.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import json
from typing import List, Tuple, Dict, Any

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image, 'camera/color/image_raw', self.image_callback, 10
        )
        self.depth_subscriber = self.create_subscription(
            Image, 'camera/depth/image_raw', self.depth_callback, 10
        )
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, 'camera/color/camera_info', self.camera_info_callback, 10
        )

        # Publishers
        self.detection_publisher = self.create_publisher(
            String, 'object_detections', 10
        )
        self.visualization_publisher = self.create_publisher(
            Image, 'vision_visualization', 10
        )

        # Vision model
        self.model = self.load_detection_model()

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Detection parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4

        self.get_logger().info('Vision Node initialized')

    def load_detection_model(self):
        """YoloV5 یا YoloV8 ماڈل لوڈ کریں"""
        try:
            # YoloV8 ماڈل کو لوڈ کریں
            from ultralytics import YOLO
            model = YOLO('yolov8n.pt')  # یا yolov5n.pt
            return model
        except ImportError:
            self.get_logger().warn('YOLO models not available, using placeholder')
            return None

    def camera_info_callback(self, msg: CameraInfo):
        """کیمرہ کی معلومات کو اپ ڈیٹ کریں"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        """تصویری فریم کو پروسیس کریں"""
        try:
            # CV2 تصویر میں تبدیل کریں
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # آبجیکٹ ڈیٹیکشن کریں
            detections = self.detect_objects(cv_image)

            # ڈیٹیکشنز کو پبلش کریں
            self.publish_detections(detections)

            # وژولائزیشن تیار کریں
            vis_image = self.draw_detections(cv_image, detections)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            self.visualization_publisher.publish(vis_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def depth_callback(self, msg: Image):
        """ڈیپتھ فریم کو پروسیس کریں"""
        try:
            # ڈیپتھ تصویر کو CV2 فارمیٹ میں تبدیل کریں
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # ڈیپتھ انفارمیشن کو اسٹور کریں یا استعمال کریں
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def detect_objects(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """تصویر میں آبجیکٹس کو ڈیٹیکٹ کریں"""
        if self.model is None:
            return []

        try:
            # YOLO ماڈل کے ذریعے ڈیٹیکشن کریں
            results = self.model(image, conf=self.confidence_threshold)

            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # ڈیٹا حاصل کریں
                        xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])

                        # کلاس نام حاصل کریں
                        class_name = self.model.names[cls] if hasattr(self.model, 'names') else f'class_{cls}'

                        detection = {
                            'class_name': class_name,
                            'confidence': conf,
                            'bbox': {
                                'x1': float(xyxy[0]),
                                'y1': float(xyxy[1]),
                                'x2': float(xyxy[2]),
                                'y2': float(xyxy[3])
                            },
                            'center': {
                                'x': float((xyxy[0] + xyxy[2]) / 2),
                                'y': float((xyxy[1] + xyxy[3]) / 2)
                            }
                        }

                        # اگر کیمرہ میٹرکس دستیاب ہے تو 3D پوزیشن حاصل کریں
                        if self.camera_matrix is not None:
                            detection['position_3d'] = self.calculate_3d_position(
                                detection['center'], self.get_depth_at_point(detection['center'])
                            )

                        detections.append(detection)

            return detections

        except Exception as e:
            self.get_logger().error(f'Error in object detection: {e}')
            return []

    def get_depth_at_point(self, center: Dict[str, float]) -> float:
        """مخصوص پوائنٹ پر ڈیپتھ حاصل کریں (سادہ مثال)"""
        # یہ ایک جنرل فنکشن ہے، حقیقی ایپلیکیشن میں آپ ڈیپتھ فریم سے ویلیو حاصل کریں گے
        return 1.0  # جنرک ڈیپتھ

    def calculate_3d_position(self, center: Dict[str, float], depth: float) -> Dict[str, float]:
        """2D سے 3D پوزیشن حساب کریں"""
        if self.camera_matrix is None:
            return {'x': 0.0, 'y': 0.0, 'z': depth}

        # 2D پوائنٹس کو نارملائز کریں
        x2d = center['x']
        y2d = center['y']

        # کیمرہ میٹرکس کے پیرامیٹرز
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # 3D پوزیشن حساب کریں
        x3d = (x2d - cx) * depth / fx
        y3d = (y2d - cy) * depth / fy
        z3d = depth

        return {'x': x3d, 'y': y3d, 'z': z3d}

    def draw_detections(self, image: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """تصویر پر ڈیٹیکشنز کو ڈرائی کریں"""
        vis_image = image.copy()

        for detection in detections:
            bbox = detection['bbox']
            class_name = detection['class_name']
            conf = detection['confidence']

            # باؤنڈنگ باکس ڈرائی کریں
            cv2.rectangle(vis_image,
                         (int(bbox['x1']), int(bbox['y1'])),
                         (int(bbox['x2']), int(bbox['y2'])),
                         (0, 255, 0), 2)

            # لیبل ڈرائی کریں
            label = f"{class_name}: {conf:.2f}"
            cv2.putText(vis_image, label,
                       (int(bbox['x1']), int(bbox['y1']) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # سینٹر پوائنٹ ڈرائی کریں
            center = detection['center']
            cv2.circle(vis_image, (int(center['x']), int(center['y'])), 5, (0, 0, 255), -1)

        return vis_image

    def publish_detections(self, detections: List[Dict[str, Any]]):
        """ڈیٹیکشنز کو ROS 2 میسج کے طور پر پبلش کریں"""
        detection_msg = String()
        detection_msg.data = json.dumps({
            'detections': detections,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.detection_publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 2: کسٹم ڈیٹیکشن ماڈل کو نافذ کریں
ایک فائل `custom_detector.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import List, Dict, Any, Tuple
import cv2

class CustomObjectDetector(nn.Module):
    def __init__(self, num_classes: int = 80):
        super(CustomObjectDetector, self).__init__()

        # بیک بون نیٹ ورک (سادہ CNN)
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )

        # ڈیٹیکشن ہیڈ
        self.detection_head = nn.Sequential(
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(128, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes + 5),  # classes + bbox (4) + confidence (1)
        )

        self.num_classes = num_classes

    def forward(self, x):
        """فروارڈ پاس"""
        features = self.backbone(x)
        detections = self.detection_head(features)

        # ڈیٹیکشنز کو YOLO فارمیٹ میں تبدیل کریں
        batch_size = detections.size(0)

        # باؤنڈنگ باکس کوڈس (x, y, w, h)
        bbox_coords = detections[:, :4]
        # کنفیڈنس اسکور
        confidence = torch.sigmoid(detections[:, 4:5])
        # کلاس سکورز
        class_scores = torch.softmax(detections[:, 5:], dim=1)

        return {
            'bbox': bbox_coords,
            'confidence': confidence,
            'class_scores': class_scores
        }

class VisionPreprocessor:
    def __init__(self, target_size: Tuple[int, int] = (416, 416)):
        self.target_size = target_size

    def preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        """تصویر کو ماڈل کے لیے پری پروسیس کریں"""
        # سائز تبدیل کریں
        resized = cv2.resize(image, self.target_size)

        # BGR سے RGB میں تبدیل کریں
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        # نارملائز کریں
        normalized = rgb.astype(np.float32) / 255.0

        # CHW فارمیٹ میں تبدیل کریں
        transposed = np.transpose(normalized, (2, 0, 1))

        # بیچ ڈائیمینشن شامل کریں
        batched = np.expand_dims(transposed, axis=0)

        # PyTorch ٹینسر میں تبدیل کریں
        tensor = torch.from_numpy(batched)

        return tensor

class VisionPostprocessor:
    def __init__(self, confidence_threshold: float = 0.5, nms_threshold: float = 0.4):
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold

    def postprocess_detections(self, raw_detections: Dict[str, torch.Tensor],
                              original_image_size: Tuple[int, int]) -> List[Dict[str, Any]]:
        """ڈیٹیکشنز کو پوسٹ پروسیس کریں"""
        bbox_coords = raw_detections['bbox'].squeeze(0).cpu().numpy()
        confidence = raw_detections['confidence'].squeeze(0).cpu().numpy()
        class_scores = raw_detections['class_scores'].squeeze(0).cpu().numpy()

        # فلٹر ہائی کنفیڈنس ڈیٹیکشنز
        high_conf_indices = confidence > self.confidence_threshold

        detections = []
        for i in range(len(high_conf_indices)):
            if high_conf_indices[i]:
                class_idx = np.argmax(class_scores[i])
                class_score = class_scores[i][class_idx]

                if class_score > self.confidence_threshold:
                    detection = {
                        'class_id': int(class_idx),
                        'confidence': float(class_score),
                        'bbox': self.convert_bbox_format(bbox_coords[i], original_image_size)
                    }
                    detections.append(detection)

        # NMS (Non-Maximum Suppression) لاگو کریں
        detections = self.apply_nms(detections)

        return detections

    def convert_bbox_format(self, bbox_coords: np.ndarray,
                           original_size: Tuple[int, int]) -> Dict[str, float]:
        """باؤنڈنگ باکس کو فارمیٹ میں تبدیل کریں"""
        h, w = original_size

        # YOLO فارمیٹ سے XYXY فارمیٹ میں تبدیل کریں
        x_center, y_center, width, height = bbox_coords

        x1 = (x_center - width/2) * w
        y1 = (y_center - height/2) * h
        x2 = (x_center + width/2) * w
        y2 = (y_center + height/2) * h

        return {
            'x1': float(x1),
            'y1': float(y1),
            'x2': float(x2),
            'y2': float(y2),
            'center_x': float((x1 + x2) / 2),
            'center_y': float((y1 + y2) / 2)
        }

    def apply_nms(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Non-Maximum Suppression لاگو کریں"""
        if not detections:
            return []

        # IOU کے اساس ڈوپلیکیٹس کو ہٹائیں
        # یہ ایک سادہ NMS ایمپلیمنٹیشن ہے
        filtered_detections = []

        # کنفیڈنس کے اساس ترتیب دیں
        sorted_detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)

        for detection in sorted_detections:
            is_duplicate = False
            for kept_detection in filtered_detections:
                iou = self.calculate_iou(detection['bbox'], kept_detection['bbox'])
                if iou > self.nms_threshold:
                    is_duplicate = True
                    break

            if not is_duplicate:
                filtered_detections.append(detection)

        return filtered_detections

    def calculate_iou(self, bbox1: Dict[str, float], bbox2: Dict[str, float]) -> float:
        """IOU (Intersection over Union) حساب کریں"""
        x1_1, y1_1, x2_1, y2_1 = bbox1['x1'], bbox1['y1'], bbox1['x2'], bbox1['y2']
        x1_2, y1_2, x2_2, y2_2 = bbox2['x1'], bbox2['y1'], bbox2['x2'], bbox2['y2']

        # انٹرسیکشن ایریا
        x1_int = max(x1_1, x1_2)
        y1_int = max(y1_1, y1_2)
        x2_int = min(x2_1, x2_2)
        y2_int = min(y2_1, y2_2)

        if x2_int <= x1_int or y2_int <= y1_int:
            return 0.0

        intersection_area = (x2_int - x1_int) * (y2_int - y1_int)

        # یونین ایریا
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = area1 + area2 - intersection_area

        return intersection_area / union_area if union_area > 0 else 0.0
```

### اقدام 3: وژن ٹیسٹنگ اور والیڈیشن کو نافذ کریں
ایک فائل `vision_tester.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time

class VisionTesterNode(Node):
    def __init__(self):
        super().__init__('vision_tester_node')

        self.bridge = CvBridge()

        # سبسکرائبرز
        self.detection_subscriber = self.create_subscription(
            String, 'object_detections', self.detection_callback, 10
        )

        # ٹیسٹ متغیرات
        self.test_results = []
        self.test_start_time = None
        self.test_duration = 10.0  # 10 سیکنڈ کے لیے ٹیسٹ
        self.detection_count = 0

        self.get_logger().info('Vision Tester Node initialized')

    def detection_callback(self, msg: String):
        """ڈیٹیکشنز کو وصول کریں اور ٹیسٹ کریں"""
        try:
            detection_data = json.loads(msg.data)
            detections = detection_data.get('detections', [])

            self.detection_count += len(detections)

            # ہر ڈیٹیکشن کو لاگ کریں
            for detection in detections:
                class_name = detection.get('class_name', 'unknown')
                confidence = detection.get('confidence', 0.0)

                self.get_logger().info(f'Detected: {class_name} with confidence {confidence:.2f}')

            # ٹیسٹ کے نتائج کو اسٹور کریں
            self.test_results.append({
                'timestamp': detection_data.get('timestamp'),
                'detection_count': len(detections),
                'detections': detections
            })

        except Exception as e:
            self.get_logger().error(f'Error processing detection: {e}')

    def run_performance_test(self):
        """کارکردگی ٹیسٹ چلائیں"""
        self.get_logger().info('Starting performance test...')
        self.test_start_time = time.time()

        # 10 سیکنڈ تک ٹیسٹ چلائیں
        while time.time() - self.test_start_time < self.test_duration:
            time.sleep(0.1)  # 100ms کے لیے انتظار کریں

        self.print_test_results()

    def print_test_results(self):
        """ٹیسٹ کے نتائج کو پرنٹ کریں"""
        total_time = time.time() - self.test_start_time
        avg_detections_per_second = self.detection_count / total_time if total_time > 0 else 0

        self.get_logger().info('=== Vision Performance Test Results ===')
        self.get_logger().info(f'Total time: {total_time:.2f}s')
        self.get_logger().info(f'Total detections: {self.detection_count}')
        self.get_logger().info(f'Average detections per second: {avg_detections_per_second:.2f}')
        self.get_logger().info(f'Average detections per frame: {len(self.test_results) / len([r for r in self.test_results if r["detection_count"] > 0]) if len([r for r in self.test_results if r["detection_count"] > 0]) > 0 else 0:.2f}')

        # کلاسز کے ہسٹوگرام
        class_counts = {}
        for result in self.test_results:
            for detection in result['detections']:
                class_name = detection.get('class_name', 'unknown')
                class_counts[class_name] = class_counts.get(class_name, 0) + 1

        self.get_logger().info('Class distribution:')
        for class_name, count in sorted(class_counts.items(), key=lambda x: x[1], reverse=True):
            self.get_logger().info(f'  {class_name}: {count}')

def main(args=None):
    rclpy.init(args=args)
    tester = VisionTesterNode()

    # ٹیسٹ شروع کریں
    tester.run_performance_test()

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- ROS 2 میں کمپیوٹر وژن نوڈ تخلیق کیا
- YOLO ماڈل کے ساتھ آبجیکٹ ڈیٹیکشن نافذ کیا
- 3D پوزیشن حساب کا انطباق کیا
- ڈیٹیکشن والیڈیشن اور ٹیسٹنگ کو نافذ کیا

یہ سسٹم VLA فریم ورک کے لیے وژن کا بنیادی ڈھانچہ فراہم کرتا ہے جہاں روبوٹ اپنے ماحول کو دیکھ سکتا ہے اور اشیاء کو سمجھ سکتا ہے۔