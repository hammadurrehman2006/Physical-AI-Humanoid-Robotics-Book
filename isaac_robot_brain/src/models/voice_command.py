"""
Voice Command Data Model

This module defines the VoiceCommand data model representing a spoken instruction
that undergoes speech processing, language understanding, and intent extraction.
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Any, Optional
from enum import Enum
import uuid


class VoiceCommandStatus(Enum):
    """Enumeration of possible voice command statuses"""
    RECEIVED = "received"
    PROCESSING = "processing"
    EXECUTED = "executed"
    FAILED = "failed"


@dataclass
class VoiceCommand:
    """
    Represents a spoken instruction that undergoes speech processing,
    language understanding, and intent extraction.

    Attributes:
        id: Unique identifier for the command
        audio_data: Raw audio input or path to audio file
        transcript: Text transcription of the spoken command
        confidence: Confidence score for speech recognition (0.0-1.0)
        timestamp: Time when command was received
        language: Detected language of the command
        intent: Parsed intent from natural language processing
        parameters: Extracted parameters from the command (objects, locations, etc.)
        status: Current processing status (received, processing, executed, failed)
    """

    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    audio_data: Optional[str] = None
    transcript: Optional[str] = None
    confidence: float = 0.0
    timestamp: datetime = field(default_factory=datetime.now)
    language: str = "en"
    intent: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    status: VoiceCommandStatus = VoiceCommandStatus.RECEIVED

    def __post_init__(self):
        """Validate the voice command after initialization"""
        self.validate()

    def validate(self) -> bool:
        """
        Validate the voice command according to the specified validation rules.

        Returns:
            bool: True if validation passes, False otherwise

        Raises:
            ValueError: If validation fails
        """
        # Validate confidence is between 0.0 and 1.0
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(f"Confidence must be between 0.0 and 1.0, got {self.confidence}")

        # Validate transcript is not empty
        if self.transcript is None or self.transcript.strip() == "":
            raise ValueError("Transcript must not be empty")

        # Validate timestamp is in ISO 8601 format by attempting to convert to ISO format
        try:
            # This will succeed for datetime objects, but we'll also check if it's a valid format
            self.timestamp.isoformat()
        except Exception:
            raise ValueError(f"Timestamp must be in ISO 8601 format, got {self.timestamp}")

        # Validate status is one of the allowed values
        if not isinstance(self.status, VoiceCommandStatus):
            raise ValueError(f"Invalid status: {self.status}")

        return True

    def update_status(self, new_status: VoiceCommandStatus):
        """Update the command status"""
        self.status = new_status
        return self

    def to_dict(self) -> Dict[str, Any]:
        """Convert the voice command to a dictionary representation"""
        return {
            "id": self.id,
            "audio_data": self.audio_data,
            "transcript": self.transcript,
            "confidence": self.confidence,
            "timestamp": self.timestamp.isoformat(),
            "language": self.language,
            "intent": self.intent,
            "parameters": self.parameters,
            "status": self.status.value
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VoiceCommand':
        """Create a VoiceCommand instance from a dictionary"""
        status = VoiceCommandStatus(data.get("status", "received"))
        timestamp = datetime.fromisoformat(data.get("timestamp", datetime.now().isoformat()))

        return cls(
            id=data.get("id", str(uuid.uuid4())),
            audio_data=data.get("audio_data"),
            transcript=data.get("transcript"),
            confidence=data.get("confidence", 0.0),
            timestamp=timestamp,
            language=data.get("language", "en"),
            intent=data.get("intent"),
            parameters=data.get("parameters", {}),
            status=status
        )