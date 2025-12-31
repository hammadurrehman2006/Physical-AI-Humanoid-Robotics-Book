#!/bin/bash
# setup_vision_systems.sh
# Script to set up vision processing components for Vision-Language-Action module

set -e  # Exit on any error

echo "Setting up vision processing components..."

# Install computer vision dependencies
echo "Installing computer vision dependencies..."
pip3 install opencv-contrib-python

# Download YOLOv8 model if not already present
echo "Downloading YOLOv8 model..."
python3 -c "
from ultralytics import YOLO
print('Loading YOLOv8 model...')
model = YOLO('yolov8n.pt')  # Load a small, fast model
print('YOLOv8 model loaded successfully.')
"

# Test camera access
echo "Testing camera access..."
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print('Camera access successful!')
    else:
        print('Warning: Could not read from camera.')
    cap.release()
else:
    print('Warning: Could not access camera.')
"

echo "Vision processing setup completed successfully!"