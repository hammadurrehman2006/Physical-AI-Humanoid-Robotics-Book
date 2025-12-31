#!/usr/bin/env python3
"""
Test script for Visual Perception Data Model
This script tests that the VisualPerception model meets all requirements from the data-model.md specification.
"""

import sys
import os

# Add the src directory to the path so we can import the models
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'isaac_robot_brain', 'src'))

from models.visual_perception import VisualPerception, DetectedObject, BoundingBox, Position3D, ObjectSize
from datetime import datetime


def test_visual_perception_model():
    """Test the VisualPerception model with all required fields and validation rules"""

    print("Testing VisualPerception model...")

    # Test 1: Create a basic VisualPerception instance
    try:
        perception = VisualPerception(
            camera_source="front_camera"
        )
        print("✓ Basic VisualPerception instance created successfully")
    except Exception as e:
        print(f"✗ Failed to create basic instance: {e}")
        return False

    # Test 2: Verify all required fields are present
    required_fields = [
        'id', 'timestamp', 'objects', 'environment_map',
        'camera_source', 'image_path', 'confidence_threshold', 'detection_accuracy'
    ]

    for field in required_fields:
        if not hasattr(perception, field):
            print(f"✗ Missing required field: {field}")
            return False
    print("✓ All required fields are present")

    # Test 3: Test confidence_threshold validation (must be between 0.0 and 1.0)
    try:
        invalid_perception = VisualPerception(
            camera_source="test_camera",
            confidence_threshold=1.5  # Invalid value
        )
        print("✗ Failed to validate confidence_threshold properly")
        return False
    except ValueError:
        print("✓ confidence_threshold validation works correctly")

    # Test 4: Create a valid DetectedObject with all required properties
    try:
        bounding_box = BoundingBox(x=10.0, y=20.0, width=100.0, height=80.0)
        position_3d = Position3D(x=1.0, y=2.0, z=0.5)
        size = ObjectSize(width=0.1, height=0.1, depth=0.1)

        detected_obj = DetectedObject(
            object_id="obj_001",
            class_name="ball",  # Using class_name to match implementation
            bounding_box=bounding_box,
            confidence=0.85,
            position_3d=position_3d,
            size=size,
            color="red"
        )
        print("✓ DetectedObject created successfully with all properties")
    except Exception as e:
        print(f"✗ Failed to create DetectedObject: {e}")
        return False

    # Test 5: Test object properties validation
    try:
        # Test confidence validation for object
        invalid_obj = DetectedObject(
            object_id="obj_002",
            class_name="cube",
            bounding_box=BoundingBox(x=5.0, y=5.0, width=30.0, height=30.0),
            confidence=1.5,  # Invalid confidence
            position_3d=Position3D(x=1.0, y=1.0, z=0.5),
            size=ObjectSize(width=0.1, height=0.1, depth=0.1),
            color="blue"
        )
        print("✗ Failed to validate object confidence properly")
        return False
    except ValueError:
        print("✓ Object confidence validation works correctly")

    # Test 6: Test 3D position validation (must be within robot's operational space)
    try:
        invalid_pos_obj = DetectedObject(
            object_id="obj_003",
            class_name="table",
            bounding_box=BoundingBox(x=0.0, y=0.0, width=200.0, height=100.0),
            confidence=0.7,
            position_3d=Position3D(x=10.0, y=10.0, z=5.0),  # Outside operational space
            size=ObjectSize(width=1.0, height=0.8, depth=0.8),
            color="brown"
        )
        print("✗ Failed to validate 3D position properly")
        return False
    except ValueError:
        print("✓ 3D position validation works correctly")

    # Test 7: Add object to perception and verify
    try:
        perception.add_object(detected_obj)
        if len(perception.objects) != 1:
            print("✗ Object was not added to perception")
            return False
        print("✓ Object added to perception successfully")
    except Exception as e:
        print(f"✗ Failed to add object to perception: {e}")
        return False

    # Test 8: Test object filtering by class
    try:
        ball_objects = perception.get_objects_by_class("ball")
        if len(ball_objects) != 1:
            print("✗ Failed to filter objects by class")
            return False
        print("✓ Object filtering by class works correctly")
    except Exception as e:
        print(f"✗ Failed object filtering test: {e}")
        return False

    # Test 9: Test serialization (to ensure 'class' field is used in JSON)
    try:
        json_data = perception.dict()
        if 'camera_source' not in json_data:
            print("✗ camera_source field missing from serialized data")
            return False
        if 'objects' in json_data and len(json_data['objects']) > 0:
            obj_data = json_data['objects'][0]
            if 'class' not in obj_data:  # Check that 'class' field is used in serialization
                print("✗ 'class' field not found in serialized object (should be 'class', not 'class_name')")
                return False
        print("✓ Serialization works correctly with 'class' field")
    except Exception as e:
        print(f"✗ Serialization failed: {e}")
        return False

    # Test 10: Test objects list validation
    try:
        # Create perception with invalid objects list
        invalid_perception = VisualPerception(
            camera_source="test_camera",
            objects=["invalid_object"]  # Should be DetectedObject instances
        )
        print("✗ Failed to validate objects list properly")
        return False
    except ValueError:
        print("✓ Objects list validation works correctly")

    print("\n✓ All tests passed! The VisualPerception model meets all requirements from the specification.")
    return True


def demonstrate_usage():
    """Demonstrate the usage of the VisualPerception model"""
    print("\n" + "="*60)
    print("VISUAL PERCEPTION MODEL USAGE DEMONSTRATION")
    print("="*60)

    # Create a visual perception instance
    perception = VisualPerception(
        camera_source="front_camera",
        confidence_threshold=0.7,
        image_path="/path/to/image.jpg"
    )

    # Add some detected objects
    objects_data = [
        {
            "object_id": "ball_001",
            "class_name": "ball",
            "bounding_box": BoundingBox(x=50.0, y=60.0, width=80.0, height=80.0),
            "confidence": 0.92,
            "position_3d": Position3D(x=1.2, y=0.5, z=0.0),
            "size": ObjectSize(width=0.15, height=0.15, depth=0.15),
            "color": "red"
        },
        {
            "object_id": "cup_001",
            "class_name": "cup",
            "bounding_box": BoundingBox(x=200.0, y=150.0, width=60.0, height=80.0),
            "confidence": 0.87,
            "position_3d": Position3D(x=0.8, y=-0.3, z=0.0),
            "size": ObjectSize(width=0.08, height=0.1, depth=0.08),
            "color": "white"
        },
        {
            "object_id": "table_001",
            "class_name": "table",
            "bounding_box": BoundingBox(x=0.0, y=200.0, width=300.0, height=150.0),
            "confidence": 0.95,
            "position_3d": Position3D(x=1.0, y=0.0, z=0.0),
            "size": ObjectSize(width=1.2, height=0.8, depth=0.8),
            "color": "brown"
        }
    ]

    for obj_data in objects_data:
        obj = DetectedObject(**obj_data)
        perception.add_object(obj)

    print(f"Created perception snapshot with ID: {perception.id}")
    print(f"Captured at: {perception.timestamp}")
    print(f"From camera: {perception.camera_source}")
    print(f"Detected {len(perception.objects)} objects")
    print(f"Average confidence: {perception.calculate_average_confidence():.2f}")

    print("\nDetected objects:")
    for obj in perception.objects:
        print(f"  - ID: {obj.object_id}, Class: {obj.class_name}, Confidence: {obj.confidence:.2f}")

    # Filter objects by confidence
    high_confidence_objects = perception.filter_objects_by_confidence(0.9)
    print(f"\nObjects with confidence >= 0.9: {len(high_confidence_objects)}")

    # Get objects by class
    balls = perception.get_objects_by_class("ball")
    print(f"Number of balls detected: {len(balls)}")

    # Serialize to dict (this will use 'class' field in output)
    serialized = perception.dict()
    print(f"\nSerialized object example:")
    if serialized['objects']:
        first_obj = serialized['objects'][0]
        print(f"  First object serialized keys: {list(first_obj.keys())}")


if __name__ == "__main__":
    print("Testing VisualPerception Data Model Implementation")
    print("="*60)

    success = test_visual_perception_model()

    if success:
        demonstrate_usage()
    else:
        print("\n✗ Tests failed - model does not meet specification requirements")
        sys.exit(1)