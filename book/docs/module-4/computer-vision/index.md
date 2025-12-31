---
sidebar_position: 3
title: "Computer Vision Integration"
---

# Computer Vision Integration

In this section, we'll implement the computer vision component of our Vision-Language-Action system. This module enables the robot to perceive and understand its environment through visual sensors, providing crucial information for grounded language understanding and action planning.

## Overview

Computer vision is the eyes of the robot, enabling it to detect, recognize, and understand objects in its environment. This section covers:
- Object detection and recognition systems
- Scene understanding and spatial relationships
- Visual feature extraction and representation
- Integration with language understanding for multimodal reasoning
- Real-time processing and performance optimization

## Learning Objectives

By the end of this section, you will be able to:
- Implement object detection and recognition systems for robotic applications
- Extract meaningful visual features for multimodal understanding
- Create spatial understanding systems that map visual information to robot coordinates
- Integrate computer vision with language understanding for grounded interpretation
- Optimize computer vision pipelines for real-time robotic applications

## Object Detection and Recognition

### YOLO-based Object Detection

We'll use YOLO (You Only Look Once) for real-time object detection, which is well-suited for robotic applications:

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
        self.class_names = self.model.names  # COCO dataset class names

    def detect_objects(self, image):
        """Detect objects in an image using YOLO"""
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
        """Draw bounding boxes on image"""
        img_copy = image.copy()
        for detection in detections:
            x, y, w, h = detection['bbox']
            cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)
            label = f"{detection['class_name']}: {detection['confidence']:.2f}"
            cv2.putText(img_copy, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return img_copy
```

### Custom Object Recognition

For specific robotic objects, we can implement custom recognition using feature matching:

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
        """Add a template for object recognition"""
        gray_template = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.sift.detectAndCompute(gray_template, None)
        self.object_templates[object_name] = {'keypoints': kp, 'descriptors': desc}

    def recognize_object(self, image) -> List[ObjectMatch]:
        """Recognize objects in image using template matching"""
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.sift.detectAndCompute(gray_image, None)

        matches = []

        for obj_name, template_data in self.object_templates.items():
            if desc is None or template_data['descriptors'] is None:
                continue

            matches_found = self.bf.knnMatch(template_data['descriptors'], desc, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches_found:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            # Calculate confidence based on number of good matches
            if len(good_matches) > 10:  # Threshold for valid match
                confidence = min(1.0, len(good_matches) / 50.0)  # Normalize

                # Estimate bounding box based on matched keypoints
                src_pts = np.float32([template_data['keypoints'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                if len(src_pts) >= 4:
                    homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    if homography is not None:
                        h, w = template_data['keypoints'][0].size, template_data['keypoints'][0].size
                        pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                        dst = cv2.perspectiveTransform(pts, homography)

                        # Calculate bounding box
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

## 3D Object Pose Estimation

### Pose Estimation with RGB-D Data

For robotic manipulation, we need 3D pose information:

```python
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple

class PoseEstimator:
    def __init__(self):
        self.camera_matrix = None  # Will be set from camera info
        self.dist_coeffs = None    # Will be set from camera info

    def set_camera_parameters(self, camera_matrix, dist_coeffs):
        """Set camera intrinsic parameters"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def estimate_pose_3d(self, image, depth_image, object_2d_bbox, object_model):
        """Estimate 3D pose of object using 2D-3D correspondence"""
        x, y, w, h = object_2d_bbox

        # Extract region of interest
        roi_color = image[y:y+h, x:x+w]
        roi_depth = depth_image[y:y+h, x:x+w]

        # Get 3D points from depth
        height, width = roi_depth.shape
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
        u_coords = u_coords.flatten()
        v_coords = v_coords.flatten()
        depths = roi_depth.flatten()

        # Convert to 3D points
        valid_depths = depths > 0
        u_valid = u_coords[valid_depths] + x
        v_valid = v_coords[valid_depths] + y
        z_valid = depths[valid_depths]

        # Convert to 3D coordinates
        x_3d = (u_valid - self.camera_matrix[0, 2]) * z_valid / self.camera_matrix[0, 0]
        y_3d = (v_valid - self.camera_matrix[1, 2]) * z_valid / self.camera_matrix[1, 1]

        points_3d = np.vstack([x_3d, y_3d, z_valid]).T

        if len(points_3d) < 10:  # Not enough points for pose estimation
            return None

        # Use PnP to estimate pose
        # For this example, we'll use a simplified approach
        # In practice, you'd have a 3D model of the object
        object_center_3d = np.mean(points_3d, axis=0)

        # Estimate orientation (simplified)
        # In practice, you'd use more sophisticated methods like ICP
        rotation_matrix = np.eye(3)  # Identity as default

        pose = {
            'position': object_center_3d.tolist(),
            'orientation': rotation_matrix.tolist(),
            'confidence': min(1.0, len(points_3d) / 100.0)  # Scale confidence
        }

        return pose

    def get_object_world_pose(self, object_pose_3d, camera_pose_world):
        """Transform object pose from camera frame to world frame"""
        # Convert camera pose to transformation matrix
        cam_rot = R.from_euler('xyz', camera_pose_world['orientation']).as_matrix()
        cam_pos = np.array(camera_pose_world['position'])

        cam_transform = np.eye(4)
        cam_transform[:3, :3] = cam_rot
        cam_transform[:3, 3] = cam_pos

        # Convert object pose to transformation matrix
        obj_rot = np.array(object_pose_3d['orientation'])
        obj_pos = np.array(object_pose_3d['position'])

        obj_transform = np.eye(4)
        obj_transform[:3, :3] = obj_rot
        obj_transform[:3, 3] = obj_pos

        # Transform from camera frame to world frame
        world_transform = cam_transform @ obj_transform

        world_pose = {
            'position': world_transform[:3, 3].tolist(),
            'orientation': R.from_matrix(world_transform[:3, :3]).as_euler('xyz').tolist(),
            'confidence': object_pose_3d['confidence']
        }

        return world_pose
```

## Scene Understanding and Spatial Reasoning

### Spatial Relationship Detection

Understanding spatial relationships is crucial for grounded language understanding:

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
        """Detect spatial relationships between objects"""
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
                                'confidence': 0.8  # Default confidence
                            }
                            relationships.append(relationship)

        return relationships

    def _is_left_of(self, pos1, pos2, threshold=0.1):
        """Check if pos1 is to the left of pos2"""
        return pos1[0] < pos2[0] - threshold

    def _is_right_of(self, pos1, pos2, threshold=0.1):
        """Check if pos1 is to the right of pos2"""
        return pos1[0] > pos2[0] + threshold

    def _is_above(self, pos1, pos2, threshold=0.1):
        """Check if pos1 is above pos2"""
        return pos1[2] > pos2[2] + threshold

    def _is_below(self, pos1, pos2, threshold=0.1):
        """Check if pos1 is below pos2"""
        return pos1[2] < pos2[2] - threshold

    def _is_near(self, pos1, pos2, threshold=1.0):
        """Check if pos1 is near pos2"""
        distance = np.linalg.norm(pos1 - pos2)
        return distance < threshold

    def _is_far_from(self, pos1, pos2, threshold=2.0):
        """Check if pos1 is far from pos2"""
        distance = np.linalg.norm(pos1 - pos2)
        return distance > threshold

    def _is_on(self, pos1, pos2, vertical_threshold=0.1, horizontal_threshold=0.5):
        """Check if pos1 is on top of pos2 (e.g., object on table)"""
        vertical_alignment = abs(pos1[2] - pos2[2]) < vertical_threshold
        horizontal_proximity = np.linalg.norm(pos1[:2] - pos2[:2]) < horizontal_threshold
        return vertical_alignment and horizontal_proximity and pos1[2] > pos2[2]

    def _is_in(self, pos1, pos2, container_size, threshold=0.1):
        """Check if pos1 is inside container at pos2"""
        # This would require container dimensions
        # Simplified version: check if within bounds
        return False  # Placeholder - would need container info
```

## Visual Feature Extraction

### Deep Feature Extraction

For more sophisticated visual understanding, we can use deep learning features:

```python
import torch
import torchvision.transforms as transforms
from torchvision.models import resnet50
import torch.nn as nn
import numpy as np

class VisualFeatureExtractor:
    def __init__(self, model_name='resnet50', use_gpu=True):
        self.device = torch.device('cuda' if torch.cuda.is_available() and use_gpu else 'cpu')

        # Load pre-trained ResNet model
        self.model = resnet50(pretrained=True)
        # Remove the final classification layer to get features
        self.model = nn.Sequential(*list(self.model.children())[:-1])
        self.model = self.model.to(self.device)
        self.model.eval()

        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

    def extract_features(self, image):
        """Extract deep visual features from image"""
        # Convert BGR to RGB if needed
        if len(image.shape) == 3 and image.shape[2] == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = image

        # Apply transforms
        image_tensor = self.transform(image_rgb).unsqueeze(0).to(self.device)

        # Extract features
        with torch.no_grad():
            features = self.model(image_tensor)
            features = features.view(features.size(0), -1)  # Flatten
            features = features.cpu().numpy()

        return features.flatten()

    def compute_similarity(self, features1, features2):
        """Compute cosine similarity between two feature vectors"""
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
        """Register an object with its visual features"""
        features = self.feature_extractor.extract_features(image)
        self.object_features[object_name] = features

    def find_similar_objects(self, query_image, threshold=0.7):
        """Find objects similar to the query image"""
        query_features = self.feature_extractor.extract_features(query_image)

        similarities = {}
        for obj_name, obj_features in self.object_features.items():
            similarity = self.feature_extractor.compute_similarity(
                query_features, obj_features
            )
            if similarity > threshold:
                similarities[obj_name] = similarity

        # Sort by similarity
        sorted_similarities = sorted(
            similarities.items(),
            key=lambda x: x[1],
            reverse=True
        )

        return sorted_similarities
```

## Computer Vision ROS Node

### Complete ROS 2 Implementation

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

        # Publishers and subscribers
        self.object_pub = self.create_publisher(String, 'detected_objects', 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10
        )

        # Initialize components
        self.bridge = CvBridge()
        self.object_detector = YOLOObjectDetector()
        self.pose_estimator = PoseEstimator()
        self.spatial_detector = SpatialRelationshipDetector()
        self.feature_extractor = VisualFeatureExtractor()

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # Object tracking
        self.tracked_objects = {}

        self.get_logger().info("Computer Vision Node initialized")

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.pose_estimator.set_camera_parameters(self.camera_matrix, self.dist_coeffs)
        self.camera_info_received = True

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect objects
            detections = self.object_detector.detect_objects(cv_image)

            # Prepare object data for publishing
            objects_data = []

            for detection in detections:
                obj_data = {
                    'name': detection['class_name'],
                    'confidence': detection['confidence'],
                    'bbox': detection['bbox'],
                    'position': None,  # Will be filled if depth info available
                    'features': None   # Will be computed if needed
                }

                # Extract visual features
                x, y, w, h = detection['bbox']
                roi = cv_image[y:y+h, x:x+w]
                features = self.feature_extractor.extract_features(roi)
                obj_data['features'] = features.tolist()

                objects_data.append(obj_data)

            # Create and publish object message
            objects_msg = String()
            objects_msg.data = json.dumps({
                'timestamp': self.get_clock().now().to_msg(),
                'objects': objects_data,
                'image_width': msg.width,
                'image_height': msg.height
            })

            self.object_pub.publish(objects_msg)

            self.get_logger().info(f"Detected {len(detections)} objects")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_with_depth(self, image_msg, depth_msg):
        """Process image with depth information for 3D pose estimation"""
        try:
            # Convert images
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            # Detect objects
            detections = self.object_detector.detect_objects(cv_image)

            objects_with_pose = []

            for detection in detections:
                x, y, w, h = detection['bbox']

                # Estimate 3D pose
                pose_3d = self.pose_estimator.estimate_pose_3d(
                    cv_image, cv_depth, (x, y, w, h), None  # object_model would be specific to object
                )

                obj_data = {
                    'name': detection['class_name'],
                    'confidence': detection['confidence'],
                    'bbox': detection['bbox'],
                    'pose_3d': pose_3d,
                    'features': None
                }

                # Extract features
                roi = cv_image[y:y+h, x:x+w]
                features = self.feature_extractor.extract_features(roi)
                obj_data['features'] = features.tolist()

                objects_with_pose.append(obj_data)

            # Publish with 3D information
            objects_msg = String()
            objects_msg.data = json.dumps({
                'timestamp': self.get_clock().now().to_msg(),
                'objects': objects_with_pose,
                'has_3d_info': True
            })

            self.object_pub.publish(objects_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

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

## Integration with Language Understanding

### Multimodal Understanding

```python
class MultimodalUnderstanding:
    def __init__(self):
        self.object_detector = YOLOObjectDetector()
        self.language_parser = SemanticParser()  # From previous phase
        self.spatial_detector = SpatialRelationshipDetector()

    def grounded_understanding(self, command_text: str, image):
        """Combine language understanding with visual information"""
        # Parse the command
        parsed_command = self.language_parser.parse_command(command_text)

        # Detect objects in the image
        detections = self.object_detector.detect_objects(image)

        # Create object map for reference resolution
        object_map = {}
        for detection in detections:
            obj_name = detection['class_name']
            if obj_name not in object_map:
                object_map[obj_name] = []
            object_map[obj_name].append(detection)

        # Resolve ambiguous references using visual context
        resolved_entities = self._resolve_visual_references(
            parsed_command.entities, object_map, command_text
        )

        # Update command with resolved entities
        parsed_command.entities.update(resolved_entities)

        # Detect spatial relationships for context
        spatial_context = self._extract_spatial_context(detections)

        return {
            'command': parsed_command,
            'detected_objects': detections,
            'resolved_entities': resolved_entities,
            'spatial_context': spatial_context
        }

    def _resolve_visual_references(self, entities, object_map, command_text):
        """Resolve ambiguous references using visual context"""
        resolved_entities = {}

        # Resolve color-based references
        if 'color' in entities:
            target_color = entities['color'][0]
            for obj_name, obj_list in object_map.items():
                for obj in obj_list:
                    # In a real system, you'd have color information
                    # For now, we'll use object names as proxy
                    if target_color in obj_name.lower():
                        resolved_entities['target_object'] = obj['class_name']
                        resolved_entities['target_bbox'] = obj['bbox']
                        break

        # Resolve spatial references
        if 'location' in entities:
            target_location = entities['location'][0]
            # This would use spatial reasoning to find objects in the location
            # For now, we'll use a simple approach
            if target_location in object_map:
                obj = object_map[target_location][0]
                resolved_entities['target_object'] = obj['class_name']
                resolved_entities['target_bbox'] = obj['bbox']

        return resolved_entities

    def _extract_spatial_context(self, detections):
        """Extract spatial context from object detections"""
        objects_with_pos = []

        for detection in detections:
            # Convert 2D bbox to approximate 3D position
            x, y, w, h = detection['bbox']
            # This is a simplified approximation - in reality, you'd use depth
            position = [x + w/2, y + h/2, 0]  # x, y, z (z is unknown without depth)

            obj_with_pos = {
                'name': detection['class_name'],
                'position': position,
                'bbox': detection['bbox'],
                'confidence': detection['confidence']
            }
            objects_with_pos.append(obj_with_pos)

        # Detect spatial relationships
        relationships = self.spatial_detector.detect_relationships(objects_with_pos)

        return {
            'objects': objects_with_pos,
            'relationships': relationships
        }
```

## Performance Optimization

### Efficient Processing Pipeline

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

        # Frame skipping for performance
        self.frame_counter = 0
        self.frame_skip = 2  # Process every 3rd frame

        # Processing thread
        self.processing_thread = threading.Thread(target=self._process_frames, daemon=True)
        self.processing_thread.start()

        # Frame history for temporal consistency
        self.frame_history = deque(maxlen=10)

    def process_frame(self, image):
        """Process a frame with optimization techniques"""
        self.frame_counter += 1

        # Skip frames for performance
        if self.frame_counter % (self.frame_skip + 1) != 0:
            return None

        try:
            # Add to processing queue
            self.frame_queue.put_nowait({
                'image': image,
                'timestamp': time.time()
            })
        except queue.Full:
            # Queue is full, skip this frame
            return None

        # Try to get result
        try:
            result = self.result_queue.get_nowait()
            self.frame_history.append(result)
            return result
        except queue.Empty:
            # No result ready, return from history if available
            if self.frame_history:
                return self.frame_history[-1]
            return None

    def _process_frames(self):
        """Background thread for frame processing"""
        while True:
            try:
                frame_data = self.frame_queue.get(timeout=1.0)
                image = frame_data['image']

                # Detect objects
                detections = self.object_detector.detect_objects(image)

                # Prepare result
                result = {
                    'detections': detections,
                    'timestamp': frame_data['timestamp'],
                    'image_shape': image.shape
                }

                # Add to result queue
                try:
                    self.result_queue.put_nowait(result)
                except queue.Full:
                    pass  # Drop old result if queue is full

            except queue.Empty:
                continue

    def get_latest_detections(self):
        """Get the most recent detections"""
        if self.result_queue.qsize() > 0:
            # Clear old results, keep only the latest
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

## Testing and Validation

### Unit Tests

```python
import unittest
import numpy as np

class TestComputerVision(unittest.TestCase):
    def setUp(self):
        self.detector = YOLOObjectDetector()
        self.spatial_detector = SpatialRelationshipDetector()
        self.feature_extractor = VisualFeatureExtractor()

    def test_object_detection(self):
        """Test object detection functionality"""
        # Create a simple test image with a known object
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw a simple shape that YOLO might recognize
        cv2.rectangle(test_image, (100, 100), (200, 200), (255, 255, 255), -1)

        detections = self.detector.detect_objects(test_image)

        # We expect at least one detection (could be 'person', 'bicycle', etc.)
        self.assertIsInstance(detections, list)
        # Note: This test might not find objects depending on the model

    def test_spatial_relationships(self):
        """Test spatial relationship detection"""
        objects = [
            {'name': 'object1', 'position': [0, 0, 0]},
            {'name': 'object2', 'position': [1, 0, 0]},  # to the right of object1
            {'name': 'object3', 'position': [0, 1, 0]}   # above object1
        ]

        relationships = self.spatial_detector.detect_relationships(objects)

        # Check that some relationships are detected
        self.assertIsInstance(relationships, list)

        # Verify specific relationships exist
        right_relationships = [
            r for r in relationships
            if r['subject'] == 'object1' and r['predicate'] == 'left_of' and r['object'] == 'object2'
        ]
        self.assertTrue(len(right_relationships) > 0)

    def test_feature_extraction(self):
        """Test visual feature extraction"""
        # Create a simple test image
        test_image = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)

        features = self.feature_extractor.extract_features(test_image)

        # Features should be a 1D array
        self.assertIsInstance(features, np.ndarray)
        self.assertEqual(len(features.shape), 1)
        self.assertGreater(features.shape[0], 0)

    def test_feature_similarity(self):
        """Test feature similarity computation"""
        # Create two similar images
        img1 = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
        img2 = img1.copy()  # Identical image

        feat1 = self.feature_extractor.extract_features(img1)
        feat2 = self.feature_extractor.extract_features(img2)

        similarity = self.feature_extractor.compute_similarity(feat1, feat2)

        # Identical images should have high similarity
        self.assertGreaterEqual(similarity, 0.9)

if __name__ == '__main__':
    unittest.main()
```

## Configuration and Setup

### Configuration File

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

### Launch File

```xml
<!-- launch/computer_vision.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="computer_vision_node" name="computer_vision">
    <param name="model_path" value="$(var model_path)"/>
    <param name="confidence_threshold" value="0.5"/>
  </node>
</launch>
```

## Troubleshooting

### Common Issues

1. **Slow Processing Performance**
   - Solution: Use frame skipping, optimize model size, or use GPU acceleration
   - Check: Ensure GPU is properly configured if using CUDA

2. **Poor Detection Accuracy**
   - Solution: Fine-tune model on robot-specific objects or adjust confidence thresholds
   - Check: Verify lighting conditions and camera quality

3. **Memory Issues**
   - Solution: Reduce batch sizes, use optimized models, or implement frame dropping
   - Check: Monitor memory usage during operation

4. **Integration Problems**
   - Solution: Verify message formats and timing between nodes
   - Check: Ensure camera calibration parameters are correctly configured

## Next Steps

In the next section, we'll implement the action execution system that will use the visual and linguistic information to perform physical actions. The computer vision system we've built provides the essential perception capabilities needed for intelligent robot behavior.

Continue to [Action Execution](../action-execution/index.md).