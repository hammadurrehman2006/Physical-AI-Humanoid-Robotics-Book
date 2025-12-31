---
sidebar_position: 4
title: "Vision Processing Examples"
---

# Vision Processing Examples

## Overview

This document provides practical examples and use cases for vision processing in robotics applications. These examples demonstrate how to implement and use computer vision capabilities in real-world scenarios, particularly in the context of our Vision-Language-Action system.

## Basic Vision Processing Examples

### Object Detection and Recognition

```python
#!/usr/bin/env python3
"""
Object Detection and Recognition Example
Demonstrates basic object detection and recognition using OpenCV and YOLO
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(String, '/object_detections', 10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Load YOLO model (example configuration)
        # Note: In practice, you'd load actual model files
        self.net = self.load_yolo_model()

        # Class names for COCO dataset
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        self.get_logger().info("Object Detection Node initialized")

    def load_yolo_model(self):
        """Load YOLO model - simplified for example"""
        # In practice, load actual model files
        # net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')
        # return net
        return None

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Publish detections
            self.publish_detections(detections, msg.header.stamp)

            # Optionally visualize detections
            if self.get_parameter_or('visualize', True):
                self.visualize_detections(cv_image, detections)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def detect_objects(self, image):
        """Detect objects in the image"""
        height, width = image.shape[:2]

        # In a real implementation, you would:
        # 1. Preprocess the image for YOLO
        # blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        # self.net.setInput(blob)
        # outputs = self.net.forward(self.get_output_layers())

        # For this example, we'll simulate detection results
        # In practice, you'd parse YOLO outputs
        detections = [
            {
                'class': 'person',
                'confidence': 0.89,
                'bbox': [100, 50, 200, 300],  # [x, y, width, height]
                'center': [200, 200]  # Center coordinates
            },
            {
                'class': 'cup',
                'confidence': 0.75,
                'bbox': [300, 200, 100, 100],
                'center': [350, 250]
            }
        ]

        # Filter detections by confidence threshold
        confidence_threshold = 0.5
        filtered_detections = [det for det in detections if det['confidence'] > confidence_threshold]

        return filtered_detections

    def get_output_layers(self):
        """Get output layer names for YOLO"""
        # In a real implementation:
        # layer_names = self.net.getLayerNames()
        # output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        # return output_layers
        return []

    def publish_detections(self, detections, timestamp):
        """Publish object detection results"""
        detection_msg = String()
        detection_data = {
            'detections': detections,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9,
            'count': len(detections)
        }
        detection_msg.data = json.dumps(detection_data)
        self.detection_pub.publish(detection_msg)

    def visualize_detections(self, image, detections):
        """Draw bounding boxes on detected objects"""
        output_image = image.copy()

        for detection in detections:
            bbox = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']

            # Draw bounding box
            cv2.rectangle(output_image, (bbox[0], bbox[1]),
                         (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 2)

            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(output_image, label, (bbox[0], bbox[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image (in a real system, you might publish it to a visualization topic)
        cv2.imshow('Object Detection', output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Color-Based Object Detection

```python
#!/usr/bin/env python3
"""
Color-Based Object Detection Example
Demonstrates detection of objects based on color properties
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.color_pub = self.create_publisher(String, '/color_detections', 10)

        self.bridge = CvBridge()

        # Define color ranges in HSV
        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255), (170, 50, 50), (180, 255, 255)],  # Two ranges for red
            'green': [(40, 50, 50), (80, 255, 255)],
            'blue': [(100, 50, 50), (130, 255, 255)],
            'yellow': [(20, 50, 50), (30, 255, 255)],
            'purple': [(130, 50, 50), (160, 255, 255)],
            'orange': [(10, 50, 50), (20, 255, 255)]
        }

        self.get_logger().info("Color Detection Node initialized")

    def image_callback(self, msg):
        """Process incoming camera images for color detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Detect colors
            color_detections = self.detect_colors(hsv_image)

            # Publish results
            self.publish_color_detections(color_detections, msg.header.stamp)

            # Visualize results
            if self.get_parameter_or('visualize', True):
                self.visualize_color_detections(cv_image, color_detections)

        except Exception as e:
            self.get_logger().error(f"Error processing color detection: {e}")

    def detect_colors(self, hsv_image):
        """Detect objects based on color ranges"""
        color_detections = {}

        for color_name, ranges in self.color_ranges.items():
            if isinstance(ranges[0], tuple) and len(ranges) == 4:  # Two ranges (like red)
                mask1 = cv2.inRange(hsv_image, ranges[0], ranges[1])
                mask2 = cv2.inRange(hsv_image, ranges[2], ranges[3])
                mask = cv2.bitwise_or(mask1, mask2)
            else:  # Single range
                mask = cv2.inRange(hsv_image, ranges[0], ranges[1])

            # Apply morphological operations to clean up the mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter contours by area (remove small noise)
            min_area = 500
            filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

            # Calculate bounding boxes and centers
            detections = []
            for contour in filtered_contours:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                detections.append({
                    'bbox': [x, y, w, h],
                    'center': [center_x, center_y],
                    'area': cv2.contourArea(contour)
                })

            if detections:
                color_detections[color_name] = detections

        return color_detections

    def publish_color_detections(self, detections, timestamp):
        """Publish color detection results"""
        detection_msg = String()
        detection_data = {
            'detections': detections,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9,
            'color_count': len(detections)
        }
        detection_msg.data = json.dumps(detection_data)
        self.color_pub.publish(detection_msg)

    def visualize_color_detections(self, image, detections):
        """Visualize color detections on the image"""
        output_image = image.copy()

        # Color map for visualization
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'purple': (255, 0, 255),
            'orange': (0, 165, 255)
        }

        for color_name, color_detections in detections.items():
            if color_name in color_map:
                color_bgr = color_map[color_name]

                for detection in color_detections:
                    bbox = detection['bbox']
                    center = detection['center']

                    # Draw bounding box
                    cv2.rectangle(output_image, (bbox[0], bbox[1]),
                                 (bbox[0] + bbox[2], bbox[1] + bbox[3]), color_bgr, 2)

                    # Draw center point
                    cv2.circle(output_image, (center[0], center[1]), 5, color_bgr, -1)

                    # Draw label
                    cv2.putText(output_image, color_name, (bbox[0], bbox[1] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        cv2.imshow('Color Detection', output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = ColorDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Feature Matching and Object Recognition

```python
#!/usr/bin/env python3
"""
Feature Matching and Object Recognition Example
Demonstrates object recognition using feature matching
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class FeatureMatchingNode(Node):
    def __init__(self):
        super().__init__('feature_matching_node')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.feature_pub = self.create_publisher(String, '/feature_matches', 10)

        self.bridge = CvBridge()

        # Initialize SIFT detector
        self.sift = cv2.SIFT_create()

        # Initialize FLANN matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Load reference objects (in practice, these would be loaded from files)
        self.reference_objects = self.load_reference_objects()

        self.get_logger().info("Feature Matching Node initialized")

    def load_reference_objects(self):
        """Load reference objects for matching"""
        # In practice, you would load actual reference images
        # For this example, we'll create some placeholder data
        return {
            'cup': {
                'name': 'cup',
                'keypoints': [],
                'descriptors': np.array([]),
                'template_image': None
            },
            'book': {
                'name': 'book',
                'keypoints': [],
                'descriptors': np.array([]),
                'template_image': None
            }
        }

    def image_callback(self, msg):
        """Process incoming camera images for feature matching"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect and match features
            matches = self.match_features(cv_image)

            # Publish results
            self.publish_feature_matches(matches, msg.header.stamp)

            # Visualize results
            if self.get_parameter_or('visualize', True):
                self.visualize_feature_matches(cv_image, matches)

        except Exception as e:
            self.get_logger().error(f"Error processing feature matching: {e}")

    def match_features(self, image):
        """Match features between input image and reference objects"""
        # Detect keypoints and descriptors in input image
        kp2, des2 = self.sift.detectAndCompute(image, None)

        if des2 is None:
            return []

        matches_result = []

        # Match against each reference object
        for obj_name, obj_data in self.reference_objects.items():
            if obj_data['descriptors'] is not None and len(obj_data['descriptors']) > 0:
                # Match descriptors
                matches = self.flann.knnMatch(obj_data['descriptors'], des2, k=2)

                # Apply Lowe's ratio test
                good_matches = []
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.7 * n.distance:
                            good_matches.append(m)

                # Consider object detected if enough good matches
                if len(good_matches) >= 10:  # Threshold for detection
                    matches_result.append({
                        'object': obj_name,
                        'matches_count': len(good_matches),
                        'confidence': len(good_matches) / 50.0  # Normalize confidence
                    })

        return matches_result

    def publish_feature_matches(self, matches, timestamp):
        """Publish feature matching results"""
        matches_msg = String()
        matches_data = {
            'matches': matches,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9,
            'match_count': len(matches)
        }
        matches_msg.data = json.dumps(matches_data)
        self.feature_pub.publish(matches_msg)

    def visualize_feature_matches(self, image, matches):
        """Visualize feature matching results"""
        output_image = image.copy()

        # Draw match information on image
        y_offset = 30
        for i, match in enumerate(matches):
            text = f"Detected: {match['object']} (confidence: {match['confidence']:.2f})"
            cv2.putText(output_image, text, (10, y_offset + i*30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Feature Matching', output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = FeatureMatchingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Vision Processing Examples

### 3D Object Pose Estimation

```python
#!/usr/bin/env python3
"""
3D Object Pose Estimation Example
Demonstrates estimation of 3D object poses from 2D images
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import json

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/object_poses', 10)
        self.detection_pub = self.create_publisher(String, '/pose_detections', 10)

        self.bridge = CvBridge()

        # Camera intrinsic parameters (will be updated from camera info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # Reference object models (3D points of known objects)
        self.object_models = self.load_object_models()

        self.get_logger().info("Pose Estimation Node initialized")

    def camera_info_callback(self, msg):
        """Update camera intrinsic parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_info_received = True

    def load_object_models(self):
        """Load 3D models of reference objects"""
        # Define simple 3D models for common objects
        # These are simplified examples - in practice, you'd load actual 3D models

        # Example: A simple rectangular object (like a book or box)
        book_model = np.array([
            [-0.1, -0.15, 0],    # Bottom-left corner
            [0.1, -0.15, 0],     # Bottom-right corner
            [0.1, 0.15, 0],      # Top-right corner
            [-0.1, 0.15, 0],     # Top-left corner
            [-0.1, -0.15, 0.01], # Same corner at different depth
            [0.1, -0.15, 0.01],  # ...
            [0.1, 0.15, 0.01],
            [-0.1, 0.15, 0.01]
        ], dtype=np.float32)

        cup_model = np.array([
            [0, 0, 0],           # Bottom center
            [0.04, 0, 0],        # Bottom edge
            [0, 0.04, 0],        # Bottom edge
            [-0.04, 0, 0],       # Bottom edge
            [0, -0.04, 0],       # Bottom edge
            [0, 0, 0.08],        # Top center
            [0.04, 0, 0.08],     # Top edge
            [0, 0.04, 0.08],     # Top edge
            [-0.04, 0, 0.08],    # Top edge
            [0, -0.04, 0.08]     # Top edge
        ], dtype=np.float32)

        return {
            'book': book_model,
            'cup': cup_model
        }

    def image_callback(self, msg):
        """Process incoming camera images for pose estimation"""
        if not self.camera_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect objects first (simplified - in practice use object detection)
            # For this example, we'll simulate detection of a book
            object_detections = self.simulate_object_detection(cv_image)

            # Estimate poses
            pose_results = []
            for detection in object_detections:
                pose = self.estimate_pose(detection, cv_image)
                if pose:
                    pose_results.append(pose)
                    self.publish_pose(pose, msg.header)

            # Publish detection results
            self.publish_pose_detections(pose_results, msg.header.stamp)

            # Visualize results
            if self.get_parameter_or('visualize', True):
                self.visualize_poses(cv_image, pose_results)

        except Exception as e:
            self.get_logger().error(f"Error processing pose estimation: {e}")

    def simulate_object_detection(self, image):
        """Simulate object detection (in practice, use actual object detection)"""
        # This is a placeholder - in reality, you'd use object detection
        # to find objects and their 2D bounding boxes
        height, width = image.shape[:2]

        # Simulate detection of a book in the center of the image
        detections = [
            {
                'class': 'book',
                'bbox': [width//2 - 50, height//2 - 75, 100, 150],  # [x, y, w, h]
                'keypoints': [  # Simulated 2D keypoints
                    (width//2 - 50, height//2 - 75),  # Top-left
                    (width//2 + 50, height//2 - 75),  # Top-right
                    (width//2 + 50, height//2 + 75),  # Bottom-right
                    (width//2 - 50, height//2 + 75)   # Bottom-left
                ]
            }
        ]

        return detections

    def estimate_pose(self, detection, image):
        """Estimate 3D pose of detected object"""
        obj_class = detection['class']

        if obj_class not in self.object_models:
            return None

        # Get 2D keypoints from detection
        img_points = np.array(detection['keypoints'], dtype=np.float32)

        # Get corresponding 3D model points
        obj_points = self.object_models[obj_class]

        # Solve PnP to find rotation and translation
        try:
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # Create pose dictionary
                pose = {
                    'object_class': obj_class,
                    'translation': tvec.flatten().tolist(),
                    'rotation_matrix': rotation_matrix.flatten().tolist(),
                    'rotation_vector': rvec.flatten().tolist(),
                    'confidence': 0.9  # Placeholder confidence
                }

                return pose
        except Exception as e:
            self.get_logger().error(f"Error in pose estimation: {e}")

        return None

    def publish_pose(self, pose, header):
        """Publish object pose"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "camera_link"  # Assuming camera frame

        # Set position
        pose_msg.pose.position.x = pose['translation'][0]
        pose_msg.pose.position.y = pose['translation'][1]
        pose_msg.pose.position.z = pose['translation'][2]

        # Convert rotation matrix to quaternion
        rotation_matrix = np.array(pose['rotation_matrix']).reshape(3, 3)
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion"""
        # Method from: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        trace = np.trace(rotation_matrix)

        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # S=4*qw
            qw = 0.25 * s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                qx = 0.25 * s
                qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                qy = 0.25 * s
                qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
                qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                qz = 0.25 * s

        return [qx, qy, qz, qw]

    def publish_pose_detections(self, poses, timestamp):
        """Publish pose detection results"""
        detection_msg = String()
        detection_data = {
            'poses': poses,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9,
            'count': len(poses)
        }
        detection_msg.data = json.dumps(detection_data)
        self.detection_pub.publish(detection_msg)

    def visualize_poses(self, image, poses):
        """Visualize pose estimation results"""
        output_image = image.copy()

        for i, pose in enumerate(poses):
            # Draw pose information
            text = f"Object: {pose['object_class']}"
            cv2.putText(output_image, text, (10, 30 + i*60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Draw position
            pos_text = f"Pos: ({pose['translation'][0]:.2f}, {pose['translation'][1]:.2f}, {pose['translation'][2]:.2f})"
            cv2.putText(output_image, pos_text, (10, 60 + i*60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow('Pose Estimation', output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = PoseEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Multi-Object Tracking

```python
#!/usr/bin/env python3
"""
Multi-Object Tracking Example
Demonstrates tracking of multiple objects across video frames
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
from collections import defaultdict

class MultiObjectTracker(Node):
    def __init__(self):
        super().__init__('multi_object_tracker')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.tracking_pub = self.create_publisher(String, '/object_tracks', 10)

        self.bridge = CvBridge()

        # Initialize CSRT trackers for different objects
        self.trackers = cv2.legacy.MultiTracker_create()

        # Object detection for initial detection
        self.detector = self.initialize_detector()

        # Track history
        self.track_history = defaultdict(list)
        self.object_ids = {}
        self.next_id = 0

        # Frame counter
        self.frame_count = 0

        self.get_logger().info("Multi-Object Tracker initialized")

    def initialize_detector(self):
        """Initialize object detector (using a simple HOG descriptor as example)"""
        # In practice, you might use YOLO, SSD, or other detectors
        # For this example, we'll use HOG descriptor for people detection
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        return hog

    def image_callback(self, msg):
        """Process incoming camera images for multi-object tracking"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to RGB for HOG detector
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Update existing tracks
            success, boxes = self.trackers.update(cv_image)

            # Update track history
            for i, box in enumerate(boxes):
                if success[i]:
                    x, y, w, h = [int(v) for v in box]
                    obj_id = list(self.object_ids.keys())[i]
                    self.track_history[obj_id].append((x + w//2, y + h//2))  # Store center

                    # Limit history length
                    if len(self.track_history[obj_id]) > 100:
                        self.track_history[obj_id] = self.track_history[obj_id][-100:]

            # Perform new detections periodically to handle new objects
            if self.frame_count % 30 == 0:  # Detect every 30 frames
                self.detect_and_add_objects(cv_image)

            # Publish tracking results
            self.publish_tracking_results(boxes, success, msg.header.stamp)

            # Visualize results
            if self.get_parameter_or('visualize', True):
                self.visualize_tracking(cv_image, boxes, success)

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error in multi-object tracking: {e}")

    def detect_and_add_objects(self, image):
        """Detect new objects and add them to trackers"""
        # Perform object detection
        # For this example, using HOG for people detection
        boxes, weights = self.detector.detectMultiScale(image)

        # Convert to format expected by tracker
        new_boxes = []
        for (x, y, w, h) in boxes:
            # Filter by size to avoid very small detections
            if w > 30 and h > 60:  # Reasonable size for people
                new_boxes.append((x, y, w, h))

        # Add new objects to tracker if they're not already tracked
        for box in new_boxes:
            x, y, w, h = box
            center_x, center_y = x + w//2, y + h//2

            # Check if this detection is already tracked
            already_tracked = False
            for i, tracked_box in enumerate(self.trackers.getObjects()):
                tx, ty, tw, th = [int(v) for v in tracked_box]
                t_center_x, t_center_y = tx + tw//2, ty + th//2

                # Calculate distance between centers
                dist = np.sqrt((center_x - t_center_x)**2 + (center_y - t_center_y)**2)

                # If close enough, consider it the same object
                if dist < max(w, h) * 0.5:
                    already_tracked = True
                    break

            if not already_tracked:
                # Create new tracker for this object
                tracker = cv2.legacy.TrackerCSRT_create()
                self.trackers.add(tracker, image, (x, y, w, h))

                # Assign ID to this object
                obj_id = self.next_id
                self.object_ids[len(self.object_ids)] = obj_id
                self.next_id += 1

    def publish_tracking_results(self, boxes, success, timestamp):
        """Publish tracking results"""
        tracking_msg = String()
        tracking_data = {
            'boxes': [[int(x) for x in box] for box in boxes],
            'success': success,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9,
            'object_count': len([s for s in success if s])
        }
        tracking_msg.data = json.dumps(tracking_data)
        self.tracking_pub.publish(tracking_msg)

    def visualize_tracking(self, image, boxes, success):
        """Visualize tracking results"""
        output_image = image.copy()

        for i, (box, ok) in enumerate(zip(boxes, success)):
            if ok:
                # Tracking success
                p1 = (int(box[0]), int(box[1]))
                p2 = (int(box[0] + box[2]), int(box[1] + box[3])
                cv2.rectangle(output_image, p1, p2, (0, 255, 0), 2, 1)

                # Draw ID
                cv2.putText(output_image, f'ID: {list(self.object_ids.values())[i]}',
                           (p1[0], p1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Draw trajectory
                obj_id = list(self.object_ids.values())[i]
                if obj_id in self.track_history:
                    points = self.track_history[obj_id]
                    for j in range(1, len(points)):
                        cv2.line(output_image, points[j-1], points[j], (0, 255, 0), 2)
            else:
                # Tracking failure
                cv2.putText(output_image, 'Tracking failure', (100, 80),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        cv2.imshow('Multi-Object Tracking', output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = MultiObjectTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Vision-Language Integration Examples

### Grounded Language Understanding

```python
#!/usr/bin/env python3
"""
Grounded Language Understanding Example
Demonstrates integration of vision and language processing
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import json
import re
from typing import Dict, List, Optional

class GroundedLanguageNode(Node):
    def __init__(self):
        super().__init__('grounded_language_node')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.voice_cmd_sub = self.create_subscription(String, '/voice_commands', self.voice_command_callback, 10)
        self.action_pub = self.create_publisher(String, '/grounded_actions', 10)

        self.bridge = CvBridge()

        # Store current scene information
        self.current_objects = []
        self.scene_description = ""

        # Command pattern matching
        self.command_patterns = {
            'find': r'find\s+(?P<target>[\w\s]+)',
            'grasp': r'(grasp|pick up|take)\s+(?P<target>[\w\s]+)',
            'point_to': r'point to\s+(?P<target>[\w\s]+)',
            'describe': r'describe|what do you see|tell me about',
            'count': r'how many\s+(?P<target>[\w\s]+)',
            'color': r'what color is the\s+(?P<target>[\w\s]+)'
        }

        self.get_logger().info("Grounded Language Node initialized")

    def image_callback(self, msg):
        """Process camera images to understand current scene"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection to update scene understanding
            self.current_objects = self.detect_scene_objects(cv_image)
            self.scene_description = self.describe_scene(self.current_objects)

            self.get_logger().debug(f"Updated scene: {self.scene_description}")

        except Exception as e:
            self.get_logger().error(f"Error processing scene: {e}")

    def voice_command_callback(self, msg):
        """Process voice commands with visual context"""
        try:
            command_text = msg.data.lower()

            # Parse command
            action = self.parse_command_with_context(command_text)

            if action:
                # Publish grounded action
                action_msg = String()
                action_msg.data = json.dumps(action)
                self.action_pub.publish(action_msg)

                self.get_logger().info(f"Grounded action: {action}")
            else:
                self.get_logger().warn(f"Could not parse command: {command_text}")

        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {e}")

    def detect_scene_objects(self, image):
        """Detect objects in the current scene"""
        # In practice, use a real object detection model
        # For this example, we'll simulate detection

        # Convert image to get some basic visual features
        height, width = image.shape[:2]

        # Simulate object detection results
        objects = [
            {
                'class': 'person',
                'confidence': 0.92,
                'bbox': [width//2 - 50, height//2 - 100, 100, 200],
                'center': [width//2, height//2],
                'color': 'blue shirt'
            },
            {
                'class': 'cup',
                'confidence': 0.85,
                'bbox': [width//3, height//2, 50, 80],
                'center': [width//3 + 25, height//2 + 40],
                'color': 'white'
            },
            {
                'class': 'book',
                'confidence': 0.78,
                'bbox': [2*width//3, height//3, 80, 100],
                'center': [2*width//3 + 40, height//3 + 50],
                'color': 'red cover'
            }
        ]

        return objects

    def describe_scene(self, objects):
        """Create a textual description of the scene"""
        if not objects:
            return "I don't see any objects."

        descriptions = []
        for obj in objects:
            desc = f"a {obj['color']} {obj['class']} at position {obj['center']}"
            descriptions.append(desc)

        return f"I see: {', '.join(descriptions)}."

    def parse_command_with_context(self, command_text):
        """Parse command using both linguistic and visual context"""
        # First, try to match command patterns
        for action_type, pattern in self.command_patterns.items():
            match = re.search(pattern, command_text, re.IGNORECASE)
            if match:
                target = match.group('target').strip() if 'target' in match.groupdict() else None

                if action_type == 'find':
                    return self.handle_find_command(target)
                elif action_type == 'grasp':
                    return self.handle_grasp_command(target)
                elif action_type == 'point_to':
                    return self.handle_point_command(target)
                elif action_type == 'describe':
                    return self.handle_describe_command()
                elif action_type == 'count':
                    return self.handle_count_command(target)
                elif action_type == 'color':
                    return self.handle_color_command(target)

        # If no pattern matches, try general understanding
        return self.handle_general_command(command_text)

    def handle_find_command(self, target):
        """Handle find commands with visual context"""
        if not target:
            return None

        # Look for target object in current scene
        for obj in self.current_objects:
            if target.lower() in obj['class'].lower() or target.lower() in obj['color'].lower():
                return {
                    'action': 'find_object',
                    'object_class': obj['class'],
                    'position': obj['center'],
                    'bbox': obj['bbox'],
                    'confidence': obj['confidence'],
                    'target': target
                }

        # Object not found in scene
        return {
            'action': 'object_not_found',
            'target': target,
            'scene_description': self.scene_description
        }

    def handle_grasp_command(self, target):
        """Handle grasp commands with visual context"""
        if not target:
            return None

        # Look for graspable object in scene
        for obj in self.current_objects:
            if (target.lower() in obj['class'].lower() or
                target.lower() in obj['color'].lower()) and \
               obj['class'] in ['cup', 'book', 'bottle', 'box']:  # Graspable objects

                return {
                    'action': 'grasp_object',
                    'object_class': obj['class'],
                    'position': obj['center'],
                    'bbox': obj['bbox'],
                    'confidence': obj['confidence'],
                    'target': target
                }

        # Object not found or not graspable
        return {
            'action': 'cannot_grasp',
            'target': target,
            'reason': 'object not found or not graspable',
            'scene_description': self.scene_description
        }

    def handle_point_command(self, target):
        """Handle point-to commands with visual context"""
        if not target:
            return {'action': 'point_to', 'position': [320, 240]}  # Center of image

        # Look for target in scene
        for obj in self.current_objects:
            if target.lower() in obj['class'].lower() or target.lower() in obj['color'].lower():
                return {
                    'action': 'point_to',
                    'position': obj['center'],
                    'object_class': obj['class'],
                    'target': target
                }

        return {
            'action': 'object_not_found',
            'target': target,
            'scene_description': self.scene_description
        }

    def handle_describe_command(self):
        """Handle describe commands"""
        return {
            'action': 'describe_scene',
            'description': self.scene_description,
            'object_count': len(self.current_objects)
        }

    def handle_count_command(self, target):
        """Handle count commands"""
        if not target:
            return {'action': 'count_objects', 'count': len(self.current_objects), 'type': 'total_objects'}

        count = 0
        for obj in self.current_objects:
            if target.lower() in obj['class'].lower() or target.lower() in obj['color'].lower():
                count += 1

        return {
            'action': 'count_objects',
            'count': count,
            'type': target,
            'scene_description': self.scene_description
        }

    def handle_color_command(self, target):
        """Handle color query commands"""
        if not target:
            return None

        for obj in self.current_objects:
            if target.lower() in obj['class'].lower():
                return {
                    'action': 'report_color',
                    'object': obj['class'],
                    'color': obj['color'],
                    'position': obj['center']
                }

        return {
            'action': 'color_not_found',
            'target': target,
            'scene_description': self.scene_description
        }

    def handle_general_command(self, command_text):
        """Handle commands that don't match specific patterns"""
        # For complex commands, you might use an LLM to interpret
        # For this example, return a simple response
        return {
            'action': 'unknown_command',
            'command': command_text,
            'scene_context': self.scene_description
        }

def main(args=None):
    rclpy.init(args=args)

    node = GroundedLanguageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization Examples

### Real-time Vision Processing Pipeline

```python
#!/usr/bin/env python3
"""
Real-time Vision Processing Pipeline
Demonstrates optimized real-time computer vision processing
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import queue
import time
from typing import Optional

class RealTimeVisionPipeline(Node):
    def __init__(self):
        super().__init__('realtime_vision_pipeline')

        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)

        # Publishers for different processing results
        self.detection_pub = self.create_publisher(String, '/realtime_detections', 10)
        self.tracking_pub = self.create_publisher(String, '/realtime_tracking', 10)
        self.performance_pub = self.create_publisher(String, '/vision_performance', 10)

        self.bridge = CvBridge()

        # Processing queues
        self.input_queue = queue.Queue(maxsize=2)  # Only keep most recent 2 frames
        self.output_queue = queue.Queue(maxsize=10)

        # Processing thread
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

        # Performance tracking
        self.frame_times = []
        self.last_process_time = time.time()

        # Processing flags
        self.enable_detection = True
        self.enable_tracking = True

        self.get_logger().info("Real-time Vision Pipeline initialized")

    def image_callback(self, msg):
        """Receive and queue incoming images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Put image in processing queue (non-blocking)
            try:
                self.input_queue.put_nowait({
                    'image': cv_image,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    'frame_id': msg.header.frame_id
                })
            except queue.Full:
                # Drop old frame if queue is full (keep most recent)
                pass

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def processing_loop(self):
        """Main processing loop running in separate thread"""
        while rclpy.ok():
            try:
                # Get image from queue
                try:
                    frame_data = self.input_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                # Process the image
                start_time = time.time()

                # Perform detection
                if self.enable_detection:
                    detections = self.process_detections(frame_data['image'])

                # Perform tracking
                if self.enable_tracking:
                    tracking_results = self.process_tracking(frame_data['image'])

                # Calculate processing time
                process_time = time.time() - start_time
                self.frame_times.append(process_time)

                # Maintain performance history (last 100 frames)
                if len(self.frame_times) > 100:
                    self.frame_times = self.frame_times[-100:]

                # Publish results
                self.publish_results(detections, tracking_results, frame_data, process_time)

            except Exception as e:
                self.get_logger().error(f"Error in processing loop: {e}")

    def process_detections(self, image):
        """Process object detection on image"""
        # In practice, use a real detection model
        # For this example, simulate detection
        height, width = image.shape[:2]

        # Simulate detection results
        detections = [
            {
                'class': 'person',
                'confidence': 0.89,
                'bbox': [width//2 - 50, height//2 - 100, 100, 200],
                'center': [width//2, height//2]
            }
        ]

        return detections

    def process_tracking(self, image):
        """Process object tracking on image"""
        # In practice, use a real tracking algorithm
        # For this example, simulate tracking

        # Simulate tracking results
        tracking_results = [
            {
                'object_id': 1,
                'bbox': [100, 100, 50, 100],
                'center': [125, 150],
                'velocity': [2, 1]  # pixels per frame
            }
        ]

        return tracking_results

    def publish_results(self, detections, tracking_results, frame_data, process_time):
        """Publish processing results"""
        # Publish detections
        detection_msg = String()
        detection_data = {
            'detections': detections,
            'timestamp': frame_data['timestamp'],
            'frame_time': process_time,
            'frame_id': frame_data['frame_id']
        }
        detection_msg.data = json.dumps(detection_data)
        self.detection_pub.publish(detection_msg)

        # Publish tracking results
        tracking_msg = String()
        tracking_data = {
            'tracking_results': tracking_results,
            'timestamp': frame_data['timestamp'],
            'frame_time': process_time
        }
        tracking_msg.data = json.dumps(tracking_data)
        self.tracking_pub.publish(tracking_msg)

        # Publish performance metrics periodically
        if len(self.frame_times) % 10 == 0:  # Every 10 frames
            avg_time = np.mean(self.frame_times) if self.frame_times else 0
            fps = 1.0 / avg_time if avg_time > 0 else 0

            performance_msg = String()
            performance_data = {
                'avg_processing_time': float(avg_time),
                'current_fps': float(fps),
                'frame_count': len(self.frame_times),
                'timestamp': time.time()
            }
            performance_msg.data = json.dumps(performance_data)
            self.performance_pub.publish(performance_msg)

def main(args=None):
    rclpy.init(args=args)

    node = RealTimeVisionPipeline()

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

## Testing and Validation Examples

### Vision Processing Unit Tests

```python
#!/usr/bin/env python3
"""
Vision Processing Unit Tests
Demonstrates testing of vision processing components
"""

import unittest
import numpy as np
import cv2
from unittest.mock import Mock, patch

class TestVisionProcessing(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        # Create a test image
        self.test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Create a simple test pattern
        cv2.rectangle(self.test_image, (100, 100), (200, 200), (255, 0, 0), -1)  # Blue square
        cv2.circle(self.test_image, (300, 300), 50, (0, 255, 0), -1)  # Green circle

    def test_color_detection(self):
        """Test color-based object detection"""
        # Convert to HSV
        hsv = cv2.cvtColor(self.test_image, cv2.COLOR_BGR2HSV)

        # Define color ranges
        blue_range = ([100, 50, 50], [130, 255, 255])
        green_range = ([40, 50, 50], [80, 255, 255])

        # Create masks
        blue_mask = cv2.inRange(hsv, np.array(blue_range[0]), np.array(blue_range[1]))
        green_mask = cv2.inRange(hsv, np.array(green_range[0]), np.array(green_range[1]))

        # Check that we detected the blue square and green circle
        blue_pixels = np.sum(blue_mask > 0)
        green_pixels = np.sum(green_mask > 0)

        self.assertGreater(blue_pixels, 0, "Should detect blue pixels")
        self.assertGreater(green_pixels, 0, "Should detect green pixels")

    def test_object_detection_simulation(self):
        """Test simulated object detection"""
        # Simulate detection results similar to our example code
        height, width = self.test_image.shape[:2]

        simulated_detections = [
            {
                'class': 'square',
                'confidence': 0.9,
                'bbox': [100, 100, 100, 100],  # [x, y, width, height]
                'center': [150, 150]
            },
            {
                'class': 'circle',
                'confidence': 0.85,
                'bbox': [250, 250, 100, 100],
                'center': [300, 300]
            }
        ]

        # Verify detection format
        self.assertEqual(len(simulated_detections), 2)
        self.assertIn('class', simulated_detections[0])
        self.assertIn('confidence', simulated_detections[0])
        self.assertIn('bbox', simulated_detections[0])
        self.assertIn('center', simulated_detections[0])

        # Verify confidence ranges
        for detection in simulated_detections:
            self.assertGreaterEqual(detection['confidence'], 0.0)
            self.assertLessEqual(detection['confidence'], 1.0)

    def test_pose_estimation_components(self):
        """Test pose estimation components"""
        # Test rotation matrix to quaternion conversion
        from scipy.spatial.transform import Rotation as R

        # Create a test rotation matrix
        r = R.from_euler('xyz', [0.1, 0.2, 0.3])
        rotation_matrix = r.as_matrix()

        # Convert to quaternion using our method
        # (Implementation would be in the actual node)
        self.assertEqual(rotation_matrix.shape, (3, 3))

        # Test that it's a valid rotation matrix
        det = np.linalg.det(rotation_matrix)
        self.assertAlmostEqual(det, 1.0, places=5,
                              msg="Determinant should be 1 for rotation matrix")

        # Test that R^T * R = I
        identity_check = np.dot(rotation_matrix.T, rotation_matrix)
        expected_identity = np.eye(3)
        np.testing.assert_array_almost_equal(identity_check, expected_identity,
                                           decimal=5)

    def test_image_preprocessing(self):
        """Test image preprocessing functions"""
        # Test image normalization
        normalized = self.test_image.astype(np.float32) / 255.0
        self.assertTrue(normalized.max() <= 1.0)
        self.assertTrue(normalized.min() >= 0.0)

        # Test image resizing
        resized = cv2.resize(self.test_image, (320, 240))
        self.assertEqual(resized.shape, (240, 320, 3))

    def test_feature_matching_components(self):
        """Test feature matching components"""
        # Test that SIFT can be initialized
        try:
            sift = cv2.SIFT_create()
            self.assertIsNotNone(sift, "SIFT detector should be created successfully")
        except Exception as e:
            # SIFT might not be available in all OpenCV builds
            print(f"SIFT not available: {e}")
            # Skip this test if SIFT is not available
            self.skipTest("SIFT not available in this OpenCV build")

if __name__ == '__main__':
    # Run the tests
    unittest.main(verbosity=2)
```

## Integration with ROS 2 Examples

### Vision Processing Launch Configuration

```xml
<!-- launch/vision_processing.launch.xml -->
<launch>
  <!-- Load parameters -->
  <arg name="camera_topic" default="/camera/image_raw"/>
  <arg name="enable_visualization" default="true"/>

  <!-- Object Detection Node -->
  <node pkg="your_robot_vision" exec="object_detection_node" name="object_detector">
    <param name="camera_topic" value="$(var camera_topic)"/>
    <param name="visualize" value="$(var enable_visualization)"/>
    <param name="confidence_threshold" value="0.5"/>
  </node>

  <!-- Color Detection Node -->
  <node pkg="your_robot_vision" exec="color_detection_node" name="color_detector">
    <param name="camera_topic" value="$(var camera_topic)"/>
    <param name="visualize" value="$(var enable_visualization)"/>
  </node>

  <!-- Pose Estimation Node -->
  <node pkg="your_robot_vision" exec="pose_estimation_node" name="pose_estimator">
    <param name="camera_topic" value="$(var camera_topic)"/>
    <param name="visualize" value="$(var enable_visualization)"/>
  </node>

  <!-- Grounded Language Node -->
  <node pkg="your_robot_vision" exec="grounded_language_node" name="grounded_language">
    <param name="camera_topic" value="$(var camera_topic)"/>
  </node>

  <!-- Real-time Vision Pipeline -->
  <node pkg="your_robot_vision" exec="realtime_vision_pipeline" name="realtime_vision">
    <param name="camera_topic" value="$(var camera_topic)"/>
  </node>
</launch>
```

### Configuration File

```yaml
# config/vision_processing.yaml
vision_processing:
  object_detection:
    model_type: "yolo"
    model_path: "/path/to/yolo/model"
    confidence_threshold: 0.5
    nms_threshold: 0.4
    input_size: [416, 416]

  color_detection:
    color_ranges:
      red: [[0, 50, 50], [10, 255, 255], [170, 50, 50], [180, 255, 255]]
      green: [[40, 50, 50], [80, 255, 255]]
      blue: [[100, 50, 50], [130, 255, 255]]
    min_area: 500

  pose_estimation:
    solver_method: "pnp"
    reprojection_error_threshold: 2.0
    minimum_inliers: 4

  tracking:
    algorithm: "csrt"
    max_objects: 10
    history_length: 100

  performance:
    max_processing_time: 0.1  # seconds
    target_fps: 30
    queue_size: 2

  visualization:
    enable: true
    window_name: "Vision Processing"
    scale_factor: 1.0
```

## Best Practices and Troubleshooting

### Vision Processing Best Practices

1. **Always validate image data** before processing
2. **Handle different image encodings** properly
3. **Implement proper error handling** for vision algorithms
4. **Use appropriate data types** to avoid precision loss
5. **Optimize for real-time performance** when needed
6. **Consider lighting conditions** in algorithm design
7. **Validate detection confidence** before using results
8. **Implement fallback strategies** for failed detections

### Common Issues and Solutions

1. **Memory Issues**: Use queues with size limits to prevent memory buildup
2. **Timing Issues**: Process most recent frames and drop old ones when needed
3. **Detection Accuracy**: Fine-tune thresholds and preprocessing based on environment
4. **Performance**: Use appropriate algorithms for your hardware constraints

These examples demonstrate practical implementations of computer vision for robotics applications, covering various scenarios from basic object detection to complex multi-modal processing. The examples include real-time optimization techniques and integration with ROS 2 for practical deployment.