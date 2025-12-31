# Exercise 3.1: Object Detection with YOLO

## Objective
Implement real-time object detection using YOLO (You Only Look Once) for the Vision-Language-Action system, enabling the robot to perceive and identify objects in its environment.

## Prerequisites
- Python 3.10+
- ROS 2 Humble Hawksbill installed
- OpenCV (cv2)
- PyTorch and Ultralytics YOLO
- Camera hardware or simulated camera feed
- Basic understanding of computer vision concepts

## Exercise Steps

### Step 1: Install Computer Vision Dependencies
First, install the required computer vision libraries:

```bash
# Activate your virtual environment
source voice_processing_env/bin/activate  # or create a new one

# Install computer vision libraries
pip install opencv-python ultralytics torch torchvision numpy

# Install ROS 2 computer vision packages
pip install cv-bridge sensor-msgs vision-msgs

# For 3D processing (optional)
pip install open3d scipy
```

### Step 2: Create YOLO Object Detection Class
Create a new file `yolo_detector.py`:

```python
#!/usr/bin/env python3
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from typing import List, Dict, Tuple, Optional
import json

class YOLOObjectDetector:
    """YOLO-based object detection for robotic applications"""

    def __init__(self, model_path: str = 'yolov8n.pt', confidence_threshold: float = 0.5):
        """
        Initialize YOLO object detector

        Args:
            model_path: Path to YOLO model file
            confidence_threshold: Minimum confidence for detections
        """
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.class_names = self.model.names  # COCO dataset class names
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")

        # Move model to device if using GPU
        if self.device.type == 'cuda':
            self.model.to(self.device)

    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """
        Detect objects in an image using YOLO

        Args:
            image: Input image as numpy array (BGR format)

        Returns:
            List of detection dictionaries
        """
        # Run inference
        results = self.model(image, conf=self.confidence_threshold)

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Extract bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())

                    detection = {
                        'bbox': [int(x1), int(y1), int(x2-x1), int(y2-y1)],  # x, y, width, height
                        'confidence': float(conf),
                        'class_id': cls,
                        'class_name': self.class_names[cls],
                        'center': [int((x1 + x2) / 2), int((y1 + y2) / 2)]  # Center coordinates
                    }
                    detections.append(detection)

        return detections

    def visualize_detections(self, image: np.ndarray, detections: List[Dict],
                           show_confidence: bool = True) -> np.ndarray:
        """
        Draw bounding boxes on image with detection information

        Args:
            image: Input image
            detections: List of detection dictionaries
            show_confidence: Whether to show confidence scores

        Returns:
            Image with bounding boxes drawn
        """
        img_copy = image.copy()

        for detection in detections:
            x, y, w, h = detection['bbox']
            center_x, center_y = detection['center']

            # Draw bounding box
            cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(img_copy, (center_x, center_y), 5, (0, 0, 255), -1)

            # Prepare label
            label = detection['class_name']
            if show_confidence:
                label += f": {detection['confidence']:.2f}"

            # Draw label background
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(img_copy, (x, y - label_size[1] - 10),
                         (x + label_size[0], y), (0, 255, 0), -1)

            # Draw label text
            cv2.putText(img_copy, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (0, 0, 0), 2)

        return img_copy

    def filter_detections_by_class(self, detections: List[Dict],
                                 target_classes: List[str]) -> List[Dict]:
        """
        Filter detections to only include specific classes

        Args:
            detections: List of detection dictionaries
            target_classes: List of class names to keep

        Returns:
            Filtered list of detections
        """
        target_class_ids = []
        for class_name in target_classes:
            for class_id, name in self.class_names.items():
                if name.lower() == class_name.lower():
                    target_class_ids.append(class_id)
                    break

        filtered_detections = [
            det for det in detections
            if det['class_id'] in target_class_ids
        ]

        return filtered_detections

    def get_largest_detection(self, detections: List[Dict]) -> Optional[Dict]:
        """
        Get the detection with the largest bounding box area

        Args:
            detections: List of detection dictionaries

        Returns:
            Detection with largest area, or None if no detections
        """
        if not detections:
            return None

        # Calculate areas and find the largest
        largest = max(detections, key=lambda x: x['bbox'][2] * x['bbox'][3])  # width * height
        return largest

    def get_objects_in_region(self, detections: List[Dict],
                            region: Tuple[int, int, int, int]) -> List[Dict]:
        """
        Get detections that are within a specified region

        Args:
            detections: List of detection dictionaries
            region: (x, y, width, height) of the region

        Returns:
            List of detections within the region
        """
        x, y, w, h = region
        region_x2, region_y2 = x + w, y + h

        objects_in_region = []
        for detection in detections:
            det_x, det_y, det_w, det_h = detection['bbox']
            det_x2, det_y2 = det_x + det_w, det_y + det_h

            # Check if detection center is within region
            center_x, center_y = detection['center']
            if (x <= center_x <= region_x2 and y <= center_y <= region_y2):
                objects_in_region.append(detection)

        return objects_in_region

class ObjectTracker:
    """Simple object tracker to maintain object identities across frames"""

    def __init__(self, max_displacement: int = 50):
        """
        Initialize object tracker

        Args:
            max_displacement: Maximum pixel displacement to consider same object
        """
        self.max_displacement = max_displacement
        self.tracked_objects = {}  # object_id -> detection
        self.next_id = 0

    def update(self, detections: List[Dict]) -> List[Dict]:
        """
        Update tracked objects with new detections

        Args:
            detections: New detections from current frame

        Returns:
            Detections with assigned tracking IDs
        """
        if not self.tracked_objects:
            # First frame - assign IDs to all detections
            for detection in detections:
                detection['track_id'] = self.next_id
                self.tracked_objects[self.next_id] = detection.copy()
                self.next_id += 1
            return detections

        # For subsequent frames, match detections to existing tracks
        updated_detections = []
        used_tracks = set()

        for detection in detections:
            center_x, center_y = detection['center']

            # Find closest existing track
            best_match = None
            min_distance = float('inf')

            for track_id, track_obj in self.tracked_objects.items():
                if track_id in used_tracks:
                    continue

                track_center_x, track_center_y = track_obj['center']
                distance = np.sqrt((center_x - track_center_x)**2 + (center_y - track_center_y)**2)

                if distance < min_distance and distance < self.max_displacement:
                    min_distance = distance
                    best_match = track_id

            if best_match is not None:
                # Update existing track
                detection['track_id'] = best_match
                self.tracked_objects[best_match] = detection.copy()
                used_tracks.add(best_match)
            else:
                # Create new track
                detection['track_id'] = self.next_id
                self.tracked_objects[self.next_id] = detection.copy()
                self.next_id += 1

            updated_detections.append(detection)

        return updated_detections

def main():
    """Test the YOLO detector with webcam feed"""
    print("Initializing YOLO Object Detector...")

    # Initialize detector
    detector = YOLOObjectDetector(confidence_threshold=0.5)

    # Initialize tracker
    tracker = ObjectTracker()

    # Open webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    print("Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect objects
        detections = detector.detect_objects(frame)

        # Update tracking
        tracked_detections = tracker.update(detections)

        # Visualize results
        result_frame = detector.visualize_detections(frame, tracked_detections)

        # Display stats
        cv2.putText(result_frame, f'Objects detected: {len(detections)}',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Show the frame
        cv2.imshow('YOLO Object Detection', result_frame)

        # Break on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

### Step 3: Create Custom Object Recognition
Create a file for custom object recognition `custom_recognizer.py`:

```python
#!/usr/bin/env python3
import cv2
import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass
import os

@dataclass
class ObjectMatch:
    """Data structure for object recognition results"""
    object_name: str
    confidence: float
    location: Tuple[int, int]
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    keypoints: List[cv2.KeyPoint]

class CustomObjectRecognizer:
    """Custom object recognizer using feature matching"""

    def __init__(self, detector_type: str = 'SIFT'):
        """
        Initialize custom object recognizer

        Args:
            detector_type: Feature detector type ('SIFT', 'ORB', 'AKAZE')
        """
        self.detector_type = detector_type

        if detector_type == 'SIFT':
            self.detector = cv2.SIFT_create()
        elif detector_type == 'ORB':
            self.detector = cv2.ORB_create()
        elif detector_type == 'AKAZE':
            self.detector = cv2.AKAZE_create()
        else:
            raise ValueError(f"Unsupported detector type: {detector_type}")

        self.bf = cv2.BFMatcher()
        self.object_templates = {}  # object_name -> {'keypoints': ..., 'descriptors': ...}
        self.min_matches = 10  # Minimum matches for valid recognition

    def add_template(self, object_name: str, template_image: np.ndarray):
        """
        Add a template for object recognition

        Args:
            object_name: Name to identify this object
            template_image: Template image for recognition
        """
        gray_template = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.detector.detectAndCompute(gray_template, None)

        if kp is not None and desc is not None:
            self.object_templates[object_name] = {
                'keypoints': kp,
                'descriptors': desc,
                'image': template_image
            }
            print(f"Added template for '{object_name}' with {len(kp)} keypoints")
        else:
            print(f"Warning: Could not extract features from template for '{object_name}'")

    def recognize_object(self, image: np.ndarray) -> List[ObjectMatch]:
        """
        Recognize objects in image using template matching

        Args:
            image: Input image to search for objects

        Returns:
            List of object matches
        """
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        kp, desc = self.detector.detectAndCompute(gray_image, None)

        if desc is None:
            return []

        matches = []

        for obj_name, template_data in self.object_templates.items():
            template_desc = template_data['descriptors']

            if template_desc is None:
                continue

            try:
                # Find matches between template and current image
                matches_found = self.bf.knnMatch(template_desc, desc, k=2)

                # Apply Lowe's ratio test to filter good matches
                good_matches = []
                for match_pair in matches_found:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.75 * n.distance:
                            good_matches.append(m)

                # Check if we have enough good matches
                if len(good_matches) >= self.min_matches:
                    # Calculate confidence based on number of good matches
                    confidence = min(1.0, len(good_matches) / 50.0)  # Normalize

                    # Estimate bounding box using homography
                    if len(good_matches) >= 4:
                        # Get corresponding points
                        src_pts = np.float32([template_data['keypoints'][m.queryIdx].pt
                                            for m in good_matches]).reshape(-1, 1, 2)
                        dst_pts = np.float32([kp[m.trainIdx].pt
                                            for m in good_matches]).reshape(-1, 1, 2)

                        # Find homography
                        homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                        if homography is not None:
                            # Get template dimensions
                            h, w = template_data['image'].shape[:2]
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
                                bbox=bbox,
                                keypoints=[kp[m.trainIdx] for m in good_matches]
                            ))
            except cv2.error:
                # Handle OpenCV errors gracefully
                continue

        return matches

    def load_templates_from_directory(self, directory_path: str):
        """
        Load object templates from a directory

        Args:
            directory_path: Path to directory containing template images
        """
        for filename in os.listdir(directory_path):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                image_path = os.path.join(directory_path, filename)
                template_image = cv2.imread(image_path)

                if template_image is not None:
                    # Use filename (without extension) as object name
                    object_name = os.path.splitext(filename)[0]
                    self.add_template(object_name, template_image)

    def visualize_matches(self, image: np.ndarray, matches: List[ObjectMatch]) -> np.ndarray:
        """
        Visualize object matches on image

        Args:
            image: Input image
            matches: List of object matches

        Returns:
            Image with matches visualized
        """
        img_copy = image.copy()

        for match in matches:
            x, y, w, h = match.bbox
            center_x, center_y = match.location

            # Draw bounding box
            cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(img_copy, (center_x, center_y), 5, (0, 0, 255), -1)

            # Draw label
            label = f"{match.object_name}: {match.confidence:.2f}"
            cv2.putText(img_copy, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (0, 255, 0), 2)

        return img_copy

def main():
    """Test custom object recognition"""
    print("Testing Custom Object Recognition...")

    # Initialize recognizer
    recognizer = CustomObjectRecognizer(detector_type='SIFT')

    # Create some simple templates (in a real scenario, you'd load actual images)
    # For this example, we'll create simple shapes as templates
    template1 = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(template1, (20, 20), (80, 80), (255, 255, 255), -1)  # White square

    template2 = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.circle(template2, (50, 50), 40, (255, 255, 255), -1)  # White circle

    recognizer.add_template("square", template1)
    recognizer.add_template("circle", template2)

    # Test with a frame from webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    print("Showing custom object recognition. Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Recognize objects
        matches = recognizer.recognize_object(frame)

        # Visualize matches
        result_frame = recognizer.visualize_matches(frame, matches)

        # Show stats
        cv2.putText(result_frame, f'Objects recognized: {len(matches)}',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Custom Object Recognition', result_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

### Step 4: Create Installation Script
Create an installation script `install_cv_requirements.sh`:

```bash
#!/bin/bash

echo "Installing computer vision requirements..."

# Create virtual environment if it doesn't exist
if [ ! -d "cv_env" ]; then
    python3 -m venv cv_env
fi

# Activate virtual environment
source cv_env/bin/activate

# Install core computer vision libraries
pip install opencv-python ultralytics torch torchvision numpy

# Install ROS 2 related packages
pip install cv-bridge sensor-msgs vision-msgs

# Install 3D processing libraries (optional)
pip install open3d scipy

# Install visualization libraries
pip install matplotlib

echo "Installation complete!"
echo "To activate the environment: source cv_env/bin/activate"
```

## Expected Outcomes
- Successfully implement YOLO-based object detection
- Create a custom object recognizer using feature matching
- Test both recognition systems with live camera feed
- Understand the trade-offs between general and custom recognition

## Verification Steps
1. Verify YOLO model downloads and loads correctly
2. Confirm object detection works with webcam feed
3. Test custom recognition with simple templates
4. Validate that both systems can identify objects in real-time

## Troubleshooting
- If YOLO model fails to load, ensure internet connectivity and try different model sizes
- If detection is slow, consider using smaller models or enabling GPU acceleration
- For custom recognition failures, verify template images have sufficient features

## Next Exercise
Continue to Exercise 3.2: 3D Pose Estimation and Spatial Reasoning