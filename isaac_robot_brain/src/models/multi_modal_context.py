"""
Multi-Modal Context Data Model

This module defines the MultiModalContext data model representing the combined
understanding of visual, linguistic, and situational information used for decision making.
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Any, Optional
import uuid
from .voice_command import VoiceCommand
from .visual_perception import VisualPerception
from .action_plan import ActionPlan


class FusionStrategy:
    """Enumeration of fusion strategies for combining modalities"""
    EARLY_FUSION = "early_fusion"  # Combine at feature level
    LATE_FUSION = "late_fusion"    # Combine at decision level
    HYBRID_FUSION = "hybrid_fusion"  # Combination of both
    CONFIDENCE_WEIGHTED = "confidence_weighted"  # Weight by confidence scores


@dataclass
class VoiceContext:
    """
    Represents voice-related context information.

    Attributes:
        command: The voice command that initiated this context
        confidence: Confidence in the voice processing
        language: Detected language
        intent: Extracted intent
        parameters: Extracted parameters from the voice command
    """

    command: Optional[VoiceCommand] = None
    confidence: float = 0.0
    language: str = "en"
    intent: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert voice context to dictionary"""
        return {
            "command": self.command.to_dict() if self.command else None,
            "confidence": self.confidence,
            "language": self.language,
            "intent": self.intent,
            "parameters": self.parameters
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VoiceContext':
        """Create voice context from dictionary"""
        command = None
        if data.get("command"):
            command = VoiceCommand.from_dict(data["command"])

        return cls(
            command=command,
            confidence=data.get("confidence", 0.0),
            language=data.get("language", "en"),
            intent=data.get("intent"),
            parameters=data.get("parameters", {})
        )


@dataclass
class VisualContext:
    """
    Represents visual context information.

    Attributes:
        perception: The visual perception data
        confidence: Confidence in the visual processing
        objects: Detected objects with their properties
        environment_map: Spatial representation of the environment
    """

    perception: Optional[VisualPerception] = None
    confidence: float = 0.0
    objects: Dict[str, Any] = field(default_factory=dict)
    environment_map: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert visual context to dictionary"""
        return {
            "perception": self.perception.to_dict() if self.perception else None,
            "confidence": self.confidence,
            "objects": self.objects,
            "environment_map": self.environment_map
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VisualContext':
        """Create visual context from dictionary"""
        perception = None
        if data.get("perception"):
            perception = VisualPerception.from_dict(data["perception"])

        return cls(
            perception=perception,
            confidence=data.get("confidence", 0.0),
            objects=data.get("objects", {}),
            environment_map=data.get("environment_map")
        )


@dataclass
class ActionContext:
    """
    Represents action context information.

    Attributes:
        plan: The action plan being executed
        status: Current status of action execution
        resources: Resources currently allocated
        progress: Progress information for ongoing actions
    """

    plan: Optional[ActionPlan] = None
    status: str = "pending"
    resources: Dict[str, Any] = field(default_factory=dict)
    progress: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert action context to dictionary"""
        return {
            "plan": self.plan.to_dict() if self.plan else None,
            "status": self.status,
            "resources": self.resources,
            "progress": self.progress
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ActionContext':
        """Create action context from dictionary"""
        plan = None
        if data.get("plan"):
            plan = ActionPlan.from_dict(data["plan"])

        return cls(
            plan=plan,
            status=data.get("status", "pending"),
            resources=data.get("resources", {}),
            progress=data.get("progress", {})
        )


@dataclass
class EnvironmentContext:
    """
    Represents environment context information.

    Attributes:
        location: Current location of the robot
        lighting: Lighting conditions
        noise_level: Ambient noise level
        obstacles: Detected obstacles in the environment
        map: Environmental map data
    """

    location: str = ""
    lighting: str = "normal"  # "bright", "dim", "dark"
    noise_level: float = 0.0  # 0.0-1.0 scale
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    map: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert environment context to dictionary"""
        return {
            "location": self.location,
            "lighting": self.lighting,
            "noise_level": self.noise_level,
            "obstacles": self.obstacles,
            "map": self.map
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'EnvironmentContext':
        """Create environment context from dictionary"""
        return cls(
            location=data.get("location", ""),
            lighting=data.get("lighting", "normal"),
            noise_level=data.get("noise_level", 0.0),
            obstacles=data.get("obstacles", []),
            map=data.get("map")
        )


@dataclass
class UserContext:
    """
    Represents user context information.

    Attributes:
        id: User identifier
        preferences: User preferences and settings
        history: Interaction history with the user
        profile: User profile information
    """

    id: str = ""
    preferences: Dict[str, Any] = field(default_factory=dict)
    history: List[Dict[str, Any]] = field(default_factory=list)
    profile: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert user context to dictionary"""
        return {
            "id": self.id,
            "preferences": self.preferences,
            "history": self.history,
            "profile": self.profile
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'UserContext':
        """Create user context from dictionary"""
        return cls(
            id=data.get("id", ""),
            preferences=data.get("preferences", {}),
            history=data.get("history", []),
            profile=data.get("profile", {})
        )


@dataclass
class TemporalContext:
    """
    Represents temporal context information.

    Attributes:
        timestamp: When the context was created
        duration: Duration of the current interaction
        sequence: Sequence number in the interaction
        temporal_relationships: Relationships to previous interactions
    """

    timestamp: datetime = field(default_factory=datetime.now)
    duration: float = 0.0  # seconds
    sequence: int = 0
    temporal_relationships: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert temporal context to dictionary"""
        return {
            "timestamp": self.timestamp.isoformat(),
            "duration": self.duration,
            "sequence": self.sequence,
            "temporal_relationships": self.temporal_relationships
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TemporalContext':
        """Create temporal context from dictionary"""
        timestamp = datetime.fromisoformat(data.get("timestamp", datetime.now().isoformat()))

        return cls(
            timestamp=timestamp,
            duration=data.get("duration", 0.0),
            sequence=data.get("sequence", 0),
            temporal_relationships=data.get("temporal_relationships", {})
        )


@dataclass
class MultiModalContext:
    """
    Represents the combined understanding of visual, linguistic, and situational
    information used for decision making.

    Attributes:
        id: Unique identifier for the context
        timestamp: Time when context was created
        voice_context: Information from voice command processing
        visual_context: Information from visual perception
        action_context: Information from action planning
        environment_context: Information about the environment
        user_context: Information about the user (if available)
        temporal_context: Information about time and sequence
        confidence_score: Overall confidence in the context (0.0-1.0)
        fusion_strategy: Strategy used to combine modalities
    """

    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = field(default_factory=datetime.now)
    voice_context: Optional[VoiceContext] = None
    visual_context: Optional[VisualContext] = None
    action_context: Optional[ActionContext] = None
    environment_context: Optional[EnvironmentContext] = None
    user_context: Optional[UserContext] = None
    temporal_context: Optional[TemporalContext] = None
    confidence_score: float = 0.0
    fusion_strategy: str = FusionStrategy.HYBRID_FUSION

    def __post_init__(self):
        """Validate the multi-modal context after initialization"""
        self.validate()

    def validate(self) -> bool:
        """
        Validate the multi-modal context according to the specified validation rules.

        Returns:
            bool: True if validation passes, False otherwise

        Raises:
            ValueError: If validation fails
        """
        if not 0.0 <= self.confidence_score <= 1.0:
            raise ValueError(f"Confidence score must be between 0.0 and 1.0, got {self.confidence_score}")

        if self.fusion_strategy not in [
            FusionStrategy.EARLY_FUSION,
            FusionStrategy.LATE_FUSION,
            FusionStrategy.HYBRID_FUSION,
            FusionStrategy.CONFIDENCE_WEIGHTED
        ]:
            raise ValueError(f"Invalid fusion strategy: {self.fusion_strategy}")

        return True

    def update_confidence(self, new_confidence: float):
        """Update the overall confidence score"""
        if not 0.0 <= new_confidence <= 1.0:
            raise ValueError(f"Confidence must be between 0.0 and 1.0, got {new_confidence}")
        self.confidence_score = new_confidence
        return self

    def merge_context(self, other_context: 'MultiModalContext'):
        """Merge another context with this one"""
        # This is a simplified merge - in practice, you'd want more sophisticated merging logic
        if other_context.confidence_score > self.confidence_score:
            self.confidence_score = other_context.confidence_score

        # Merge voice context if available
        if other_context.voice_context:
            self.voice_context = other_context.voice_context

        # Merge visual context if available
        if other_context.visual_context:
            self.visual_context = other_context.visual_context

        # Merge action context if available
        if other_context.action_context:
            self.action_context = other_context.action_context

        # Merge environment context if available
        if other_context.environment_context:
            self.environment_context = other_context.environment_context

        # Merge user context if available
        if other_context.user_context:
            self.user_context = other_context.user_context

        # Update timestamp to the latest
        if other_context.timestamp > self.timestamp:
            self.timestamp = other_context.timestamp

        return self

    def to_dict(self) -> Dict[str, Any]:
        """Convert the multi-modal context to a dictionary representation"""
        return {
            "id": self.id,
            "timestamp": self.timestamp.isoformat(),
            "voice_context": self.voice_context.to_dict() if self.voice_context else None,
            "visual_context": self.visual_context.to_dict() if self.visual_context else None,
            "action_context": self.action_context.to_dict() if self.action_context else None,
            "environment_context": self.environment_context.to_dict() if self.environment_context else None,
            "user_context": self.user_context.to_dict() if self.user_context else None,
            "temporal_context": self.temporal_context.to_dict() if self.temporal_context else None,
            "confidence_score": self.confidence_score,
            "fusion_strategy": self.fusion_strategy
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MultiModalContext':
        """Create a MultiModalContext instance from a dictionary"""
        timestamp = datetime.fromisoformat(data.get("timestamp", datetime.now().isoformat()))

        voice_context = None
        if data.get("voice_context"):
            voice_context = VoiceContext.from_dict(data["voice_context"])

        visual_context = None
        if data.get("visual_context"):
            visual_context = VisualContext.from_dict(data["visual_context"])

        action_context = None
        if data.get("action_context"):
            action_context = ActionContext.from_dict(data["action_context"])

        environment_context = None
        if data.get("environment_context"):
            environment_context = EnvironmentContext.from_dict(data["environment_context"])

        user_context = None
        if data.get("user_context"):
            user_context = UserContext.from_dict(data["user_context"])

        temporal_context = None
        if data.get("temporal_context"):
            temporal_context = TemporalContext.from_dict(data["temporal_context"])

        return cls(
            id=data.get("id", str(uuid.uuid4())),
            timestamp=timestamp,
            voice_context=voice_context,
            visual_context=visual_context,
            action_context=action_context,
            environment_context=environment_context,
            user_context=user_context,
            temporal_context=temporal_context,
            confidence_score=data.get("confidence_score", 0.0),
            fusion_strategy=data.get("fusion_strategy", FusionStrategy.HYBRID_FUSION)
        )