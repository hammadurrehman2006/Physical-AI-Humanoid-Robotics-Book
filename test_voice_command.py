#!/usr/bin/env python3
"""
Test script for VoiceCommand data model
"""

import sys
import os
from datetime import datetime

# Add the src directory to the path so we can import the model
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from isaac_robot_brain.src.models.voice_command import VoiceCommand, VoiceCommandStatus


def test_voice_command_creation():
    """Test basic VoiceCommand creation with validation"""
    print("Testing VoiceCommand creation...")

    # Test 1: Create a valid VoiceCommand
    try:
        vc = VoiceCommand(
            transcript="Move to the kitchen",
            confidence=0.85,
            language="en",
            intent="navigation",
            parameters={"location": "kitchen"},
            status=VoiceCommandStatus.RECEIVED
        )
        print(f"✓ Valid VoiceCommand created: {vc.id}")
        print(f"  - Transcript: {vc.transcript}")
        print(f"  - Confidence: {vc.confidence}")
        print(f"  - Status: {vc.status.value}")
        print(f"  - Timestamp: {vc.timestamp}")
    except Exception as e:
        print(f"✗ Failed to create valid VoiceCommand: {e}")
        return False

    # Test 2: Test validation with invalid confidence
    try:
        vc_invalid = VoiceCommand(
            transcript="Move to the kitchen",
            confidence=1.5,  # Invalid: > 1.0
            language="en",
            intent="navigation",
            parameters={"location": "kitchen"},
            status=VoiceCommandStatus.RECEIVED
        )
        print("✗ Should have failed with invalid confidence")
        return False
    except ValueError as e:
        print(f"✓ Correctly rejected invalid confidence: {e}")
    except Exception as e:
        print(f"✗ Unexpected error with invalid confidence: {e}")
        return False

    # Test 3: Test validation with empty transcript
    try:
        vc_invalid = VoiceCommand(
            transcript="",  # Invalid: empty
            confidence=0.85,
            language="en",
            intent="navigation",
            parameters={"location": "kitchen"},
            status=VoiceCommandStatus.RECEIVED
        )
        print("✗ Should have failed with empty transcript")
        return False
    except ValueError as e:
        print(f"✓ Correctly rejected empty transcript: {e}")
    except Exception as e:
        print(f"✗ Unexpected error with empty transcript: {e}")
        return False

    # Test 4: Test validation with None transcript
    try:
        vc_invalid = VoiceCommand(
            transcript=None,  # Invalid: None
            confidence=0.85,
            language="en",
            intent="navigation",
            parameters={"location": "kitchen"},
            status=VoiceCommandStatus.RECEIVED
        )
        print("✗ Should have failed with None transcript")
        return False
    except ValueError as e:
        print(f"✓ Correctly rejected None transcript: {e}")
    except Exception as e:
        print(f"✗ Unexpected error with None transcript: {e}")
        return False

    # Test 5: Test status update
    try:
        vc.update_status(VoiceCommandStatus.PROCESSING)
        print(f"✓ Status updated successfully: {vc.status.value}")
    except Exception as e:
        print(f"✗ Failed to update status: {e}")
        return False

    # Test 6: Test to_dict method
    try:
        vc_dict = vc.to_dict()
        print(f"✓ Converted to dictionary successfully")
        print(f"  - Dictionary keys: {list(vc_dict.keys())}")
        assert "id" in vc_dict
        assert "transcript" in vc_dict
        assert "confidence" in vc_dict
        assert "timestamp" in vc_dict
        assert "status" in vc_dict
    except Exception as e:
        print(f"✗ Failed to convert to dictionary: {e}")
        return False

    # Test 7: Test from_dict method
    try:
        new_vc = VoiceCommand.from_dict(vc_dict)
        print(f"✓ Created from dictionary successfully")
        print(f"  - New command ID: {new_vc.id}")
        print(f"  - New command transcript: {new_vc.transcript}")
    except Exception as e:
        print(f"✗ Failed to create from dictionary: {e}")
        return False

    return True


if __name__ == "__main__":
    print("Running VoiceCommand model tests...")
    success = test_voice_command_creation()

    if success:
        print("\n✓ All tests passed!")
        sys.exit(0)
    else:
        print("\n✗ Some tests failed!")
        sys.exit(1)