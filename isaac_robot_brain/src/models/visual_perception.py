"""
Visual Perception Data Model for Isaac Robot Brain

This module defines the data model for visual perception snapshots,
including detected objects and environmental information.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid
from pydantic import BaseModel, Field, validator
from typing_extensions import Literal


@dataclass
class BoundingBox:
    """Represents a 2D bounding box for object detection"""
    x: float
    y: float
    width: float
    height: float

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Bounding box width and height must be positive")
        if self.x < 0 or self.y < 0:
            raise ValueError("Bounding box coordinates must be non-negative")


@dataclass
class Position3D:
    """Represents a 3D position in robot's coordinate system"""
    x: float
    y: float
    z: float

    def __post_init__(self):
        # Define robot's operational space limits (in meters)
        # These can be adjusted based on the specific robot's capabilities
        operational_limits = {
            'x_min': -5.0, 'x_max': 5.0,
            'y_min': -5.0, 'y_max': 5.0,
            'z_min': 0.0, 'z_max': 3.0
        }

        if (self.x < operational_limits['x_min'] or self.x > operational_limits['x_max'] or
            self.y < operational_limits['y_min'] or self.y > operational_limits['y_max'] or
            self.z < operational_limits['z_min'] or self.z > operational_limits['z_max']):
            raise ValueError(
                f"Position {self} is outside robot's operational space: "
                f"X: [{operational_limits['x_min']}, {operational_limits['x_max']}], "
                f"Y: [{operational_limits['y_min']}, {operational_limits['y_max']}], "
                f"Z: [{operational_limits['z_min'], operational_limits['z_max']}]"
            )


@dataclass
class ObjectSize:
    """Represents the estimated size of an object"""
    width: float
    height: float
    depth: float

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or self.depth <= 0:
            raise ValueError("Object size dimensions must be positive")


@dataclass
class DetectedObject:
    """Represents a detected object in the visual perception"""
    object_id: str
    class_name: str  # Using class_name instead of class to avoid Python keyword conflict
    bounding_box: BoundingBox
    confidence: float
    position_3d: Position3D
    size: ObjectSize
    color: str

    def __post_init__(self):
        if not self.object_id:
            raise ValueError("Object ID cannot be empty")
        if not self.class_name:
            raise ValueError("Object class name cannot be empty")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Confidence score must be between 0.0 and 1.0")
        if not self.color:
            raise ValueError("Object color cannot be empty")

    @property
    def class_(self):
        """Property to access class name, matching the specification's 'class' field"""
        return self.class_name

    @class_.setter
    def class_(self, value):
        """Setter for class property"""
        self.class_name = value


class VisualPerception(BaseModel):
    """
    Visual Perception Data Model

    Represents a snapshot of visual perception data captured by the robot.
    """

    # Unique identifier for the perception snapshot
    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the perception snapshot")

    # Time when perception was captured
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Time when perception was captured")

    # List of detected objects with their properties
    objects: List[DetectedObject] = Field(default_factory=list, description="List of detected objects with their properties")

    # Spatial representation of the environment
    environment_map: Optional[Dict[str, Any]] = Field(None, description="Spatial representation of the environment")

    # Identifier of the camera that captured the data
    camera_source: str = Field(..., description="Identifier of the camera that captured the data")

    # Path to the source image (if applicable)
    image_path: Optional[str] = Field(None, description="Path to the source image (if applicable)")

    # Minimum confidence for object detection
    confidence_threshold: float = Field(0.5, ge=0.0, le=1.0, description="Minimum confidence for object detection")

    # Overall accuracy score for the detection
    detection_accuracy: Optional[float] = Field(None, ge=0.0, le=1.0, description="Overall accuracy score for the detection")

    class Config:
        # Allow arbitrary types for dataclass objects
        arbitrary_types_allowed = True
        # Serialize datetime objects to ISO format
        json_encoders = {
            datetime: lambda v: v.isoformat(),
            BoundingBox: lambda v: {"x": v.x, "y": v.y, "width": v.width, "height": v.height},
            Position3D: lambda v: {"x": v.x, "y": v.y, "z": v.z},
            ObjectSize: lambda v: {"width": v.width, "height": v.height, "depth": v.depth},
            DetectedObject: lambda v: {
                "object_id": v.object_id,
                "class": v.class_name,  # Using 'class' to match specification
                "bounding_box": v.bounding_box,
                "confidence": v.confidence,
                "position_3d": v.position_3d,
                "size": v.size,
                "color": v.color
            }
        }

    @validator('confidence_threshold')
    def validate_confidence_threshold(cls, value):
        """Validate that confidence threshold is between 0.0 and 1.0"""
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"Confidence threshold must be between 0.0 and 1.0, got {value}")
        return value

    @validator('detection_accuracy')
    def validate_detection_accuracy(cls, value):
        """Validate that detection accuracy is between 0.0 and 1.0 if provided"""
        if value is not None and not 0.0 <= value <= 1.0:
            raise ValueError(f"Detection accuracy must be between 0.0 and 1.0, got {value}")
        return value

    @validator('objects', each_item=True)
    def validate_detected_object(cls, obj):
        """Validate each detected object in the list"""
        if not isinstance(obj, DetectedObject):
            raise ValueError("All objects must be of type DetectedObject")
        return obj

    @validator('objects')
    def validate_objects_list(cls, objects):
        """Validate that objects list contains valid object structures as per specification"""
        if not isinstance(objects, list):
            raise ValueError("Objects must be a list")

        for obj in objects:
            if not isinstance(obj, DetectedObject):
                raise ValueError(f"All objects must be of type DetectedObject, got {type(obj)}")

            # Additional validation for each object's structure
            if not obj.object_id:
                raise ValueError("Each object must have a valid object_id")
            if not obj.class_name:
                raise ValueError("Each object must have a valid class name")
            if not 0.0 <= obj.confidence <= 1.0:
                raise ValueError(f"Each object's confidence must be between 0.0 and 1.0, got {obj.confidence}")

        return objects

    def __init__(self, **data):
        super().__init__(**data)

    def add_object(self, detected_object: DetectedObject):
        """Add a detected object to the perception snapshot"""
        self.objects.append(detected_object)

    def get_objects_by_class(self, class_name: str) -> List[DetectedObject]:
        """Get all objects of a specific class"""
        return [obj for obj in self.objects if obj.class_name == class_name]

    def get_objects_by_class_spec(self, class_name: str) -> List[DetectedObject]:
        """Get all objects of a specific class using the specification field name"""
        return [obj for obj in self.objects if obj.class_name == class_name]

    def get_object_by_id(self, object_id: str) -> Optional[DetectedObject]:
        """Get an object by its ID"""
        for obj in self.objects:
            if obj.object_id == object_id:
                return obj
        return None

    def filter_objects_by_confidence(self, min_confidence: float) -> List[DetectedObject]:
        """Filter objects by minimum confidence threshold"""
        return [obj for obj in self.objects if obj.confidence >= min_confidence]

    def calculate_average_confidence(self) -> float:
        """Calculate the average confidence of all detected objects"""
        if not self.objects:
            return 0.0
        return sum(obj.confidence for obj in self.objects) / len(self.objects)


