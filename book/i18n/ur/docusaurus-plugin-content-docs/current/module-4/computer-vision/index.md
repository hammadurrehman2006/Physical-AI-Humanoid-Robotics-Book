---
sidebar_position: 3
title: "کمپیوٹر وژن انٹیگریشن"
---

# کمپیوٹر وژن انٹیگریشن

اس سیکشن میں، ہم اپنے وژن لینگویج ایکشن سسٹم کے کمپیوٹر وژن جزو کو امپلیمنٹ کریں گے۔ یہ ماڈیول روبوٹ کو وژوئل سینسرز کے ذریعے اس کے ماحول کو سمجھنے اور سمجھ بوجھ کے قابل بناتا ہے، زمینی زبان کی سمجھ بوجھ اور ایکشن پلاننگ کے لیے اہم معلومات فراہم کرتا ہے۔

## جائزہ

کمپیوٹر وژن روبوٹ کی آنکھیں ہیں، جو اسے اس کے ماحول میں اشیاء کو ڈیٹیکٹ، ریکوگنائز، اور سمجھنے کے قابل بناتی ہیں۔ یہ سیکشن یہ احاطہ کرتا ہے::
- اشیاء کا ڈیٹیکشن اور ریکوگنیشن سسٹم
- منظر کی سمجھ بوجھ اور سپیشل ریلیشن شپس
- وژوئل فیچر ایکسٹریکشن اور ریپریزنٹیشن
- ملٹی ماڈل ریزننگ کے لیے زبان کی سمجھ بوجھ کے ساتھ انٹیگریشن
- ریل ٹائم پروسیسنگ اور کارکردگی کی اصلاح

## سیکھنے کے اہداف

اس سیکشن کے اختتام تک، آپ کے اہل ہوگا::
- روبوٹک ایپلیکیشنز کے لیے اشیاء کا ڈیٹیکشن اور ریکوگنیشن سسٹم امپلیمنٹ کریں
- ملٹی ماڈل سمجھ بوجھ کے لیے معنی خیز وژوئل فیچر ایکسٹریکٹ کریں
- ایسے سپیشل سمجھ بوجھ کے سسٹم تخلیق کریں جو وژوئل معلومات کو روبوٹ کوآرڈینیٹس میں میپ کریں
- زمینی تشریح کے لیے زبان کی سمجھ بوجھ کے ساتھ کمپیوٹر وژن کو انٹیگریٹ کریں
- ریل ٹائم روبوٹک ایپلیکیشنز کے لیے کمپیوٹر وژن پائپ لائنز کو اصلاح کریں

## اشیاء کا ڈیٹیکشن اور ریکوگنیشن

### YOLO بیسڈ اشیاء کا ڈیٹیکشن

ہم ریل ٹائم اشیاء کے ڈیٹیکشن کے لیے YOLO (You Only Look Once) استعمال کریں گے، جو روبوٹک ایپلیکیشنز کے لیے اچھی طرح سے مناسب ہے::

```python
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
import json

class YOLOObjectDetector:
    def __init__(self, model_path='yolov8n.pt', confidence_threshold=0.5):
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.class_names = self.model.names  # COCO ڈیٹا سیٹ کلاس نیمز

    def detect_objects(self, image):
        """YOLO کا استعمال کرتے ہوئے ایک تصویر میں اشیاء کا پتہ لگائیں"""
        results = self.model(image, conf=self.confidence_threshold)

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())

                    detection = {
                        'bbox': [int(x1), int(y1), int(x2-x1), int(y2-y1)],  # x, y, width, height
                        'confidence': float(conf),
                        'class_id': cls,
                        'class_name': self.class_names[cls]
                    }
                    detections.append(detection)

        return detections

    def visualize_detections(self, image, detections):
        """تصویر پر باؤنڈنگ باکسز ڈرا کریں"""
        img_copy = image.copy()
        for detection in detections:
            x, y, w, h = detection['bbox']
            cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)
            label = f"{detection['class_name']}: {detection['confidence']:.2f}"
            cv2.putText(img_copy, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return img_copy
```

### کسٹم اشیاء کا ریکوگنیشن

مخصوص روبوٹک اشیاء کے لیے، ہم فیچر میچنگ کا استعمال کرتے ہوئے کسٹم ریکوگنیشن امپلیمنٹ کر سکتے ہیں::

```python
import cv2
import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass

@dataclass
class ObjectMatch:
    object_name: str
    confidence: float
    location: Tuple[int, int]
    bbox: Tuple[int, int, int, int]  # x, y, width, height

class CustomObjectRecognizer:
    def __init__(self):
        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()
        self.object_templates = {}  # object_name -> keypoints and descriptors

    def add_template(self, object_name: str, template_image):
        """اشیاء کی ریکوگنیشن کے لیے ٹیمپلیٹ شامل کریں"""
        gray_template = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.sift.detectAndCompute(gray_template, None)
        self.object_templates[object_name] = {'keypoints': kp, 'descriptors': desc}

    def recognize_object(self, image) -> List[ObjectMatch]:
        """ٹیمپلیٹ میچنگ کا استعمال کرتے ہوئے تصویر میں اشیاء کو پہچانیں"""
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.sift.detectAndCompute(gray_image, None)

        matches = []

        for obj_name, template_data in self.object_templates.items():
            if desc is None or template_data['descriptors'] is None:
                continue

            matches_found = self.bf.knnMatch(template_data['descriptors'], desc, k=2)

            # لاؤ کا ریشیو ٹیسٹ لاگو کریں
            good_matches = []
            for match_pair in matches_found:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            # اچھے میچز کی تعداد کی بنیاد پر یقین کا حساب لگائیں
            if len(good_matches) > 10:  # درست میچ کے لیے حد
                confidence = min(1.0, len(good_matches) / 50.0)  # نارملائز کریں

                # میچڈ کی پوائنٹس کی بنیاد پر باؤنڈنگ باکس کا تخمینہ لگائیں
                src_pts = np.float32([template_data['keypoints'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                if len(src_pts) >= 4:
                    homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    if homography is not None:
                        h, w = template_data['keypoints'][0].size, template_data['keypoints'][0].size
                        pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                        dst = cv2.perspectiveTransform(pts, homography)

                        # باؤنڈنگ باکس کا حساب لگائیں
                        x_min = int(np.min(dst[:, 0, 0]))
                        y_min = int(np.min(dst[:, 0, 1]))
                        x_max = int(np.max(dst[:, 0, 0]))
                        y_max = int(np.max(dst[:, 0, 1]))

                        bbox = (x_min, y_min, x_max - x_min, y_max - y_min)
                        center = ((x_min + x_max) // 2, (y_min + y_max) // 2)

                        matches.append(ObjectMatch(
                            object_name=obj_name,
                            confidence=confidence,
                            location=center,
                            bbox=bbox
                        ))

        return matches
```

## 3D اشیاء کی پوز ایسٹیمیشن

### RGB-D ڈیٹا کے ساتھ پوز ایسٹیمیشن

روبوٹک مینیپولیشن کے لیے، ہمیں 3D پوز کی معلومات کی ضرورت ہوتی ہے::

```python
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple

class PoseEstimator:
    def __init__(self):
        self.camera_matrix = None  # کیمرہ انفارمیشن سے سیٹ کیا جائے گا
        self.dist_coeffs = None    # کیمرہ انفارمیشن سے سیٹ کیا جائے گا

    def set_camera_parameters(self, camera_matrix, dist_coeffs):
        """کیمرہ انٹرنسک پیرامیٹرز سیٹ کریں"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def estimate_pose_3d(self, image, depth_image, object_2d_bbox, object_model):
        """2D-3D کاریسپانڈنس کا استعمال کرتے ہوئے اشیاء کی 3D پوز کا تخمینہ لگائیں"""
        x, y, w, h = object_2d_bbox

        # ریجن آف انٹریسٹ نکالیں
        roi_color = image[y:y+h, x:x+w]
        roi_depth = depth_image[y:y+h, x:x+w]

        # 3D پوائنٹس حاصل کریں
        height, width = roi_depth.shape
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
        u_coords = u_coords.flatten()
        v_coords = v_coords.flatten()
        depths = roi_depth.flatten()

        # 3D پوائنٹس میں تبدیل کریں
        valid_depths = depths > 0
        u_valid = u_coords[valid_depths] + x
        v_valid = v_coords[valid_depths] + y
        z_valid = depths[valid_depths]

        # 3D کوآرڈینیٹس میں تبدیل کریں
        x_3d = (u_valid - self.camera_matrix[0, 2]) * z_valid / self.camera_matrix[0, 0]
        y_3d = (v_valid - self.camera_matrix[1, 2]) * z_valid / self.camera_matrix[1, 1]

        points_3d = np.vstack([x_3d, y_3d, z_valid]).T

        if len(points_3d) < 10:  # پوز ایسٹیمیشن کے لیے کافی پوائنٹس نہیں
            return None

        # پوز کا تخمینہ لگانے کے لیے PnP استعمال کریں
        # اس مثال کے لیے، ہم ایک سادہ نقطہ نظر استعمال کریں گے
        # عمل میں، آپ کے پاس اشیاء کا 3D ماڈل ہوگا
        object_center_3d = np.mean(points_3d, axis=0)

        # اورینٹیشن کا تخمینہ لگائیں (سادہ)
        # عمل میں، آپ ICP جیسے زیادہ ترقی یافتہ طریقے استعمال کریں گے
        rotation_matrix = np.eye(3)  # شناخت کے طور پر ڈیفالٹ

        pose = {
            'position': object_center_3d.tolist(),
            'orientation': rotation_matrix.tolist(),
            'confidence': min(1.0, len(points_3d) / 100.0)  # یقین کو اسکیل کریں
        }

        return pose

    def get_object_world_pose(self, object_pose_3d, camera_pose_world):
        """کیمرہ فریم سے ورلڈ فریم میں اشیاء کی پوز ٹرانسفر کریں"""
        # کیمرہ پوز کو ٹرانسفارمیشن میٹرکس میں تبدیل کریں
        cam_rot = R.from_euler('xyz', camera_pose_world['orientation']).as_matrix()
        cam_pos = np.array(camera_pose_world['position'])

        cam_transform = np.eye(4)
        cam_transform[:3, :3] = cam_rot
        cam_transform[:3, 3] = cam_pos

        # اشیاء کی پوز کو ٹرانسفارمیشن میٹرکس میں تبدیل کریں
        obj_rot = np.array(object_pose_3d['orientation'])
        obj_pos = np.array(object_pose_3d['position'])

        obj_transform = np.eye(4)
        obj_transform[:3, :3] = obj_rot
        obj_transform[:3, 3] = obj_pos

        # کیمرہ فریم سے ورلڈ فریم میں ٹرانسفر کریں
        world_transform = cam_transform @ obj_transform

        world_pose = {
            'position': world_transform[:3, 3].tolist(),
            'orientation': R.from_matrix(world_transform[:3, :3]).as_euler('xyz').tolist(),
            'confidence': object_pose_3d['confidence']
        }

        return world_pose
```

## منظر کی سمجھ بوجھ اور سپیشل ریزننگ

### سپیشل ریلیشن شپ ڈیٹیکشن

زمینی زبان کی سمجھ بوجھ کے لیے سپیشل ریلیشن شپس کو سمجھنا انتہائی ضروری ہے::

```python
from typing import Dict, List
import numpy as np

class SpatialRelationshipDetector:
    def __init__(self):
        self.spatial_predicates = {
            'left_of': self._is_left_of,
            'right_of': self._is_right_of,
            'above': self._is_above,
            'below': self._is_below,
            'near': self._is_near,
            'far_from': self._is_far_from,
            'on': self._is_on,
            'in': self._is_in
        }

    def detect_relationships(self, objects: List[Dict]) -> List[Dict]:
        """اشیاء کے درمیان سپیشل ریلیشن شپس کا پتہ لگائیں"""
        relationships = []

        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    obj1_pos = np.array(obj1.get('position', [0, 0, 0]))
                    obj2_pos = np.array(obj2.get('position', [0, 0, 0]))

                    for predicate_name, predicate_func in self.spatial_predicates.items():
                        if predicate_func(obj1_pos, obj2_pos):
                            relationship = {
                                'subject': obj1['name'],
                                'predicate': predicate_name,
                                'object': obj2['name'],
                                'confidence': 0.8  # ڈیفالٹ یقین
                            }
                            relationships.append(relationship)

        return relationships

    def _is_left_of(self, pos1, pos2, threshold=0.1):
        """چیک کریں کہ کیا pos1 pos2 کے بائیں ہے"""
        return pos1[0] < pos2[0] - threshold

    def _is_right_of(self, pos1, pos2, threshold=0.1):
        """چیک کریں کہ کیا pos1 pos2 کے دائیں ہے"""
        return pos1[0] > pos2[0] + threshold

    def _is_above(self, pos1, pos2, threshold=0.1):
        """چیک کریں کہ کیا pos1 pos2 کے اوپر ہے"""
        return pos1[2] > pos2[2] + threshold

    def _is_below(self, pos1, pos2, threshold=0.1):
        """چیک کریں کہ کیا pos1 pos2 کے نیچے ہے"""
        return pos1[2] < pos2[2] - threshold

    def _is_near(self, pos1, pos2, threshold=1.0):
        """چیک کریں کہ کیا pos1 pos2 کے قریب ہے"""
        distance = np.linalg.norm(pos1 - pos2)
        return distance < threshold

    def _is_far_from(self, pos1, pos2, threshold=2.0):
        """چیک کریں کہ کیا pos1 pos2 سے دور ہے"""
        distance = np.linalg.norm(pos1 - pos2)
        return distance > threshold

    def _is_on(self, pos1, pos2, vertical_threshold=0.1, horizontal_threshold=0.5):
        """چیک کریں کہ کیا pos1 pos2 کے اوپر ہے (جیسے، ٹیبل پر چیز)"""
        vertical_alignment = abs(pos1[2] - pos2[2]) < vertical_threshold
        horizontal_proximity = np.linalg.norm(pos1[:2] - pos2[:2]) < horizontal_threshold
        return vertical_alignment and horizontal_proximity and pos1[2] > pos2[2]

    def _is_in(self, pos1, pos2, container_size, threshold=0.1):
        """چیک کریں کہ کیا pos1 pos2 میں ہے"""
        # اس کے لیے کنٹینر کے ابعاد کی ضرورت ہوگی
        # سادہ ورژن: باؤنڈز کے اندر ہونے کی چیک کریں
        return False  # جگہ دار - کنٹینر کی معلومات کی ضرورت ہوگی
```

## وژوئل فیچر ایکسٹریکشن

### ڈیپ فیچر ایکسٹریکشن

زیادہ ترقی یافتہ وژوئل سمجھ بوجھ کے لیے، ہم ڈیپ لرننگ فیچر استعمال کر سکتے ہیں::

```python
import torch
import torchvision.transforms as transforms
from torchvision.models import resnet50
import torch.nn as nn
import numpy as np

class VisualFeatureExtractor:
    def __init__(self, model_name='resnet50', use_gpu=True):
        self.device = torch.device('cuda' if torch.cuda.is_available() and use_gpu else 'cpu')

        # پری ٹرینڈ ResNet ماڈل لوڈ کریں
        self.model = resnet50(pretrained=True)
        # فیچر حاصل کرنے کے لیے آخری کلاسیفکیشن لیئر کو ہٹا دیں
        self.model = nn.Sequential(*list(self.model.children())[:-1])
        self.model = self.model.to(self.device)
        self.model.eval()

        # امیج پری پروسیسنگ
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

    def extract_features(self, image):
        """تصویر سے ڈیپ وژوئل فیچر ایکسٹریکٹ کریں"""
        # ضرورت کے مطابق BGR سے RGB میں تبدیل کریں
        if len(image.shape) == 3 and image.shape[2] == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = image

        # ٹرانسفارم لاگو کریں
        image_tensor = self.transform(image_rgb).unsqueeze(0).to(self.device)

        # فیچر ایکسٹریکٹ کریں
        with torch.no_grad():
            features = self.model(image_tensor)
            features = features.view(features.size(0), -1)  # فلیٹن
            features = features.cpu().numpy()

        return features.flatten()

    def compute_similarity(self, features1, features2):
        """دو فیچر ویکٹرز کے درمیان کوسائن سیمیلرٹی کا حساب لگائیں"""
        dot_product = np.dot(features1, features2)
        norm1 = np.linalg.norm(features1)
        norm2 = np.linalg.norm(features2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        return dot_product / (norm1 * norm2)

class ObjectSimilarityMatcher:
    def __init__(self):
        self.feature_extractor = VisualFeatureExtractor()
        self.object_features = {}  # object_name -> feature_vector

    def register_object(self, object_name: str, image):
        """وژوئل فیچر کے ساتھ ایک شے کو رجسٹر کریں"""
        features = self.feature_extractor.extract_features(image)
        self.object_features[object_name] = features

    def find_similar_objects(self, query_image, threshold=0.7):
        """کویری تصویر کے مشابہ اشیاء تلاش کریں"""
        query_features = self.feature_extractor.extract_features(query_image)

        similarities = {}
        for obj_name, obj_features in self.object_features.items():
            similarity = self.feature_extractor.compute_similarity(
                query_features, obj_features
            )
            if similarity > threshold:
                similarities[obj_name] = similarity

        # سیمیلرٹی کے مطابق ترتیب دیں
        sorted_similarities = sorted(
            similarities.items(),
            key=lambda x: x[1],
            reverse=True
        )

        return sorted_similarities
```

## کمپیوٹر وژن ROS نوڈ

### مکمل ROS 2 امپلیمنٹیشن

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class ComputerVisionNode(Node):
    def __init__(self):
        super().__init__('computer_vision_node')

        # پبلشرز اور سبسکرائبرز
        self.object_pub = self.create_publisher(String, 'detected_objects', 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10
        )

        # جزوات کو شروع کریں
        self.bridge = CvBridge()
        self.object_detector = YOLOObjectDetector()
        self.pose_estimator = PoseEstimator()
        self.spatial_detector = SpatialRelationshipDetector()
        self.feature_extractor = VisualFeatureExtractor()

        # کیمرہ پیرامیٹرز
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # اشیاء کا ٹریکنگ
        self.tracked_objects = {}

        self.get_logger().info("کمپیوٹر وژن نوڈ شروع ہو گیا")

    def camera_info_callback(self, msg):
        """کیمرہ کیلیبریشن پیرامیٹرز وصول کریں"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.pose_estimator.set_camera_parameters(self.camera_matrix, self.dist_coeffs)
        self.camera_info_received = True

    def image_callback(self, msg):
        """آنے والے کیمرہ امیجز کو پروسیس کریں"""
        try:
            # ROS امیج کو OpenCV فارمیٹ میں تبدیل کریں
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # اشیاء کا پتہ لگائیں
            detections = self.object_detector.detect_objects(cv_image)

            # شائع کرنے کے لیے اشیاء کا ڈیٹا تیار کریں
            objects_data = []

            for detection in detections:
                obj_data = {
                    'name': detection['class_name'],
                    'confidence': detection['confidence'],
                    'bbox': detection['bbox'],
                    'position': None,  # ڈیپتھ معلومات دستیاب ہونے پر بھر دیا جائے گا
                    'features': None   # ضرورت پڑنے پر حساب لگایا جائے گا
                }

                # وژوئل فیچر ایکسٹریکٹ کریں
                x, y, w, h = detection['bbox']
                roi = cv_image[y:y+h, x:x+w]
                features = self.feature_extractor.extract_features(roi)
                obj_data['features'] = features.tolist()

                objects_data.append(obj_data)

            # اشیاء کا میسج بنائیں اور شائع کریں
            objects_msg = String()
            objects_msg.data = json.dumps({
                'timestamp': self.get_clock().now().to_msg(),
                'objects': objects_data,
                'image_width': msg.width,
                'image_height': msg.height
            })

            self.object_pub.publish(objects_msg)

            self.get_logger().info(f"{len(detections)} اشیاء کا پتہ چلا")

        except Exception as e:
            self.get_logger().error(f"امیج پروسیس کرنے میں خرابی: {e}")

    def process_with_depth(self, image_msg, depth_msg):
        """3D پوز ایسٹیمیشن کے لیے ڈیپتھ معلومات کے ساتھ امیج پروسیس کریں"""
        try:
            # امیجز تبدیل کریں
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            # اشیاء کا پتہ لگائیں
            detections = self.object_detector.detect_objects(cv_image)

            objects_with_pose = []

            for detection in detections:
                x, y, w, h = detection['bbox']

                # 3D پوز کا تخمینہ لگائیں
                pose_3d = self.pose_estimator.estimate_pose_3d(
                    cv_image, cv_depth, (x, y, w, h), None  # object_model اشیاء کے مطابق ہوگا
                )

                obj_data = {
                    'name': detection['class_name'],
                    'confidence': detection['confidence'],
                    'bbox': detection['bbox'],
                    'pose_3d': pose_3d,
                    'features': None
                }

                # فیچر ایکسٹریکٹ کریں
                roi = cv_image[y:y+h, x:x+w]
                features = self.feature_extractor.extract_features(roi)
                obj_data['features'] = features.tolist()

                objects_with_pose.append(obj_data)

            # 3D معلومات کے ساتھ شائع کریں
            objects_msg = String()
            objects_msg.data = json.dumps({
                'timestamp': self.get_clock().now().to_msg(),
                'objects': objects_with_pose,
                'has_3d_info': True
            })

            self.object_pub.publish(objects_msg)

        except Exception as e:
            self.get_logger().error(f"ڈیپتھ امیج پروسیس کرنے میں خرابی: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = ComputerVisionNode()

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

## زبان کی سمجھ بوجھ کے ساتھ انٹیگریشن

### ملٹی ماڈل سمجھ بوجھ

```python
class MultimodalUnderstanding:
    def __init__(self):
        self.object_detector = YOLOObjectDetector()
        self.language_parser = SemanticParser()  # پچھلے مرحلے سے
        self.spatial_detector = SpatialRelationshipDetector()

    def grounded_understanding(self, command_text: str, image):
        """زبان کی سمجھ بوجھ کو وژوئل معلومات کے ساتھ جوڑیں"""
        # کمانڈ کو پارس کریں
        parsed_command = self.language_parser.parse_command(command_text)

        # امیج میں اشیاء کا پتہ لگائیں
        detections = self.object_detector.detect_objects(image)

        # حوالہ حل کرنے کے لیے اشیاء کا میپ بنائیں
        object_map = {}
        for detection in detections:
            obj_name = detection['class_name']
            if obj_name not in object_map:
                object_map[obj_name] = []
            object_map[obj_name].append(detection)

        # وژوئل سیاق کا استعمال کرتے ہوئے ابہام کے حوالہ جات کو حل کریں
        resolved_entities = self._resolve_visual_references(
            parsed_command.entities, object_map, command_text
        )

        # حل شدہ ادارت کے ساتھ کمانڈ کو اپ ڈیٹ کریں
        parsed_command.entities.update(resolved_entities)

        # سیاق کے لیے سپیشل ریلیشن شپس کا پتہ لگائیں
        spatial_context = self._extract_spatial_context(detections)

        return {
            'command': parsed_command,
            'detected_objects': detections,
            'resolved_entities': resolved_entities,
            'spatial_context': spatial_context
        }

    def _resolve_visual_references(self, entities, object_map, command_text):
        """وژوئل سیاق کا استعمال کرتے ہوئے ابہام کے حوالہ جات کو حل کریں"""
        resolved_entities = {}

        # رنگ بیسڈ حوالہ جات کو حل کریں
        if 'color' in entities:
            target_color = entities['color'][0]
            for obj_name, obj_list in object_map.items():
                for obj in obj_list:
                    # حقیقی سسٹم میں، آپ کے پاس رنگ کی معلومات ہوگی
                    # فی الحال، ہم اشیاء کے ناموں کو پراکسی کے طور پر استعمال کریں گے
                    if target_color in obj_name.lower():
                        resolved_entities['target_object'] = obj['class_name']
                        resolved_entities['target_bbox'] = obj['bbox']
                        break

        # سپیشل حوالہ جات کو حل کریں
        if 'location' in entities:
            target_location = entities['location'][0]
            # یہ سپیشل ریزننگ کا استعمال کرے گا تاکہ مقام میں اشیاء تلاش کیے جا سکیں
            # فی الحال، ہم ایک سادہ نقطہ نظر استعمال کریں گے
            if target_location in object_map:
                obj = object_map[target_location][0]
                resolved_entities['target_object'] = obj['class_name']
                resolved_entities['target_bbox'] = obj['bbox']

        return resolved_entities

    def _extract_spatial_context(self, detections):
        """اشیاء کے ڈیٹیکشن سے سپیشل سیاق نکالیں"""
        objects_with_pos = []

        for detection in detections:
            # 2D باؤنڈنگ باکس کو تقریبی 3D پوزیشن میں تبدیل کریں
            x, y, w, h = detection['bbox']
            # یہ ایک سادہ تقریب ہے - حقیقت میں، آپ ڈیپتھ کا استعمال کریں گے
            position = [x + w/2, y + h/2, 0]  # x, y, z (z ڈیپتھ کے بغیر معلوم نہیں)

            obj_with_pos = {
                'name': detection['class_name'],
                'position': position,
                'bbox': detection['bbox'],
                'confidence': detection['confidence']
            }
            objects_with_pos.append(obj_with_pos)

        # سپیشل ریلیشن شپس کا پتہ لگائیں
        relationships = self.spatial_detector.detect_relationships(objects_with_pos)

        return {
            'objects': objects_with_pos,
            'relationships': relationships
        }
```

## کارکردگی کی اصلاح

### کارآمد پروسیسنگ پائپ لائن

```python
import threading
import queue
from collections import deque
import time

class OptimizedVisionPipeline:
    def __init__(self, max_queue_size=5):
        self.object_detector = YOLOObjectDetector()
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        self.result_queue = queue.Queue(maxsize=max_queue_size)

        # کارکردگی کے لیے فریم سکپنگ
        self.frame_counter = 0
        self.frame_skip = 2  # ہر تیسرے فریم کو پروسیس کریں

        # پروسیسنگ تھریڈ
        self.processing_thread = threading.Thread(target=self._process_frames, daemon=True)
        self.processing_thread.start()

        # ٹیمپورل کنسسٹنسی کے لیے فریم ہسٹری
        self.frame_history = deque(maxlen=10)

    def process_frame(self, image):
        """اصلاحی تکنیکس کے ساتھ ایک فریم کو پروسیس کریں"""
        self.frame_counter += 1

        # کارکردگی کے لیے فریم سکپ کریں
        if self.frame_counter % (self.frame_skip + 1) != 0:
            return None

        try:
            # پروسیسنگ قطار میں شامل کریں
            self.frame_queue.put_nowait({
                'image': image,
                'timestamp': time.time()
            })
        except queue.Full:
            # قطار بھر گئی ہے، اس فریم کو چھوڑ دیں
            return None

        # نتیجہ حاصل کرنے کی کوشش کریں
        try:
            result = self.result_queue.get_nowait()
            self.frame_history.append(result)
            return result
        except queue.Empty:
            # کوئی نتیجہ تیار نہیں، دستیاب ہونے پر ہسٹری سے لوٹائیں
            if self.frame_history:
                return self.frame_history[-1]
            return None

    def _process_frames(self):
        """فریم پروسیسنگ کے لیے بیک گراؤنڈ تھریڈ"""
        while True:
            try:
                frame_data = self.frame_queue.get(timeout=1.0)
                image = frame_data['image']

                # اشیاء کا پتہ لگائیں
                detections = self.object_detector.detect_objects(image)

                # نتیجہ تیار کریں
                result = {
                    'detections': detections,
                    'timestamp': frame_data['timestamp'],
                    'image_shape': image.shape
                }

                # نتیجہ قطار میں شامل کریں
                try:
                    self.result_queue.put_nowait(result)
                except queue.Full:
                    pass  # قطار بھر جانے پر پرانا نتیجہ چھوڑ دیں

            except queue.Empty:
                continue

    def get_latest_detections(self):
        """سب سے حالیہ ڈیٹیکشن حاصل کریں"""
        if self.result_queue.qsize() > 0:
            # پرانے نتائج صاف کریں، صرف حالیہ رکھیں
            latest = None
            while not self.result_queue.empty():
                try:
                    latest = self.result_queue.get_nowait()
                except queue.Empty:
                    break
            return latest
        elif self.frame_history:
            return self.frame_history[-1]
        return None
```

## ٹیسٹنگ اور تصدیق

### یونٹ ٹیسٹس

```python
import unittest
import numpy as np

class TestComputerVision(unittest.TestCase):
    def setUp(self):
        self.detector = YOLOObjectDetector()
        self.spatial_detector = SpatialRelationshipDetector()
        self.feature_extractor = VisualFeatureExtractor()

    def test_object_detection(self):
        """اشیاء کے ڈیٹیکشن کی فعالیت کا ٹیسٹ کریں"""
        # ایک جانا ہوا شے کے ساتھ ایک سادہ ٹیسٹ امیج بنائیں
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # ایک سادہ شکل ڈرا کریں جسے YOLO پہچان سکے
        cv2.rectangle(test_image, (100, 100), (200, 200), (255, 255, 255), -1)

        detections = self.detector.detect_objects(test_image)

        # ہم کم از کم ایک ڈیٹیکشن کی توقع کرتے ہیں ('person', 'bicycle' وغیرہ)
        self.assertIsInstance(detections, list)
        # نوٹ: یہ ٹیسٹ ماڈل کے مطابق اشیاء نہیں تلاش کر سکتا

    def test_spatial_relationships(self):
        """سپیشل ریلیشن شپ ڈیٹیکشن کا ٹیسٹ کریں"""
        objects = [
            {'name': 'object1', 'position': [0, 0, 0]},
            {'name': 'object2', 'position': [1, 0, 0]},  # object1 کے دائیں
            {'name': 'object3', 'position': [0, 1, 0]}   # object1 کے اوپر
        ]

        relationships = self.spatial_detector.detect_relationships(objects)

        # چیک کریں کہ کچھ ریلیشن شپس کا پتہ چلا ہے
        self.assertIsInstance(relationships, list)

        # مخصوص ریلیشن شپس کی تصدیق کریں
        right_relationships = [
            r for r in relationships
            if r['subject'] == 'object1' and r['predicate'] == 'left_of' and r['object'] == 'object2'
        ]
        self.assertTrue(len(right_relationships) > 0)

    def test_feature_extraction(self):
        """وژوئل فیچر ایکسٹریکشن کا ٹیسٹ کریں"""
        # ایک سادہ ٹیسٹ امیج بنائیں
        test_image = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)

        features = self.feature_extractor.extract_features(test_image)

        # فیچر ایک 1D ارے ہونا چاہیے
        self.assertIsInstance(features, np.ndarray)
        self.assertEqual(len(features.shape), 1)
        self.assertGreater(features.shape[0], 0)

    def test_feature_similarity(self):
        """فیچر سیمیلرٹی کمپیوٹیشن کا ٹیسٹ کریں"""
        # دو مشابہ امیجز بنائیں
        img1 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        img2 = img1.copy()  # شناختی امیج

        feat1 = self.feature_extractor.extract_features(img1)
        feat2 = self.feature_extractor.extract_features(img2)

        similarity = self.feature_extractor.compute_similarity(feat1, feat2)

        # شناختی امیجز کو زیادہ سیمیلرٹی ہونی چاہیے
        self.assertGreaterEqual(similarity, 0.9)

if __name__ == '__main__':
    unittest.main()
```

## کنفیگریشن اور سیٹ اپ

### کنفیگریشن فائل

```yaml
# config/computer_vision.yaml
computer_vision:
  object_detection:
    model_path: "yolov8n.pt"
    confidence_threshold: 0.5
    max_objects: 20
    enable_custom_recognition: true

  pose_estimation:
    enable_3d_pose: true
    min_points_for_pose: 10
    confidence_threshold: 0.7

  feature_extraction:
    model_name: "resnet50"
    enable_gpu: true
    feature_dimension: 2048

  performance:
    frame_skip: 2
    max_queue_size: 5
    enable_multithreading: true

  spatial_reasoning:
    enable_relationship_detection: true
    distance_thresholds:
      near: 1.0
      far: 3.0
      vertical: 0.1
      horizontal: 0.5
```

### لانچ فائل

```xml
<!-- launch/computer_vision.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="computer_vision_node" name="computer_vision">
    <param name="model_path" value="$(var model_path)"/>
    <param name="confidence_threshold" value="0.5"/>
  </node>
</launch>
```

## ٹربل شوٹنگ

### عام مسائل

1. **سست پروسیسنگ کارکردگی**
   - حل: فریم سکپنگ استعمال کریں، ماڈل سائز کو اصلاح کریں، یا GPU ایکسیلریشن استعمال کریں
   - چیک کریں: یقینی بنائیں کہ CUDA استعمال کرتے وقت GPU مناسب طریقے سے کنفیگر ہے

2. **غریب ڈیٹیکشن کی درستگی**
   - حل: روبوٹ مخصوص اشیاء پر ماڈل کو فائن ٹیون کریں یا یقین کی حدیں ایڈجسٹ کریں
   - چیک کریں: لائٹنگ کنڈیشنز اور کیمرہ کوالٹی کی تصدیق کریں

3. **میموری کے مسائل**
   - حل: بیچ سائز کم کریں، اصلاح شدہ ماڈل استعمال کریں، یا فریم ڈراپنگ لاگو کریں
   - چیک کریں: آپریشن کے دوران میموری استعمال کو مانیٹر کریں

4. **انٹیگریشن کے مسائل**
   - حل: نوڈس کے درمیان میسج فارمیٹس اور ٹائمنگ کی تصدیق کریں
   - چیک کریں: یقینی بنائیں کہ کیمرہ کیلیبریشن پیرامیٹرز درست طریقے سے کنفیگر ہیں

## اگلے اقدامات

اگلے سیکشن میں، ہم ایکشن ایکزیکیوشن سسٹم کو امپلیمنٹ کریں گے جو وژوئل اور لینگویسٹک معلومات کا استعمال کرتے ہوئے جسمانی ایکشنز انجام دے گا۔ ہم نے جو کمپیوٹر وژن سسٹم تعمیر کیا ہے وہ انٹیلیجنٹ روبوٹ برتاؤ کے لیے ضروری پریسیپشن کی صلاحیتیں فراہم کرتا ہے۔

جاری رکھیں [ایکشن ایکزیکیوشن](../action-execution/index.md).