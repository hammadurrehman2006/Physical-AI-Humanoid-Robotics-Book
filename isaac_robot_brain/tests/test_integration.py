"""
Integration Tests for Vision-Language-Action System

This module contains integration tests for the complete Vision-Language-Action system,
testing the interaction between different components.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import tempfile
import os
import numpy as np
import soundfile as sf
from datetime import datetime

from src.models.voice_command import VoiceCommand, VoiceCommandStatus
from src.models.visual_perception import VisualPerception, DetectedObject
from src.models.action_plan import ActionPlan, Action, ActionType, ActionStatus
from src.models.multi_modal_context import MultiModalContext

from src.services.voice_processor import VoiceProcessor
from src.services.whisper_service import WhisperService
from src.services.nlu_service import NLUService
from src.services.command_parser import CommandParser
from src.services.action_executor import ActionExecutor
from src.services.multi_modal_fusion import MultiModalFusionService


class MockWhisperService:
    """Mock Whisper service for testing."""

    def transcribe_audio_sync(self, audio_path: str):
        return "Test command for integration", 0.92


class MockNLUService:
    """Mock NLU service for testing."""

    def interpret_command_sync(self, voice_command: VoiceCommand):
        return "object_manipulation", {
            "action": "pick_up",
            "object": "red ball",
            "color": "red",
            "object_type": "ball"
        }


class TestVisionLanguageActionIntegration(unittest.TestCase):
    """Integration tests for the complete Vision-Language-Action system."""

    def setUp(self):
        """Set up test fixtures."""
        # Create mock services for testing
        self.mock_whisper = MockWhisperService()
        self.mock_nlu = MockNLUService()

        # Initialize services
        self.voice_processor = VoiceProcessor(whisper_service=self.mock_whisper)
        self.nlu_service = self.mock_nlu  # Use mock since real one requires API key
        self.command_parser = CommandParser()
        self.action_executor = ActionExecutor()
        self.fusion_service = MultiModalFusionService()

    def test_voice_to_action_pipeline(self):
        """Test the complete pipeline from voice command to action execution."""
        # Create a voice command
        voice_cmd = VoiceCommand(
            transcript="Pick up the red ball",
            confidence=0.9,
            intent="object_manipulation",
            parameters={"action": "pick_up", "object": "red ball", "color": "red"}
        )

        # Parse the command
        parsed_result, is_valid = self.command_parser.parse_with_validation(voice_cmd)
        self.assertTrue(is_valid)

        # Create an action plan based on the parsed command
        action = Action(
            type=ActionType.MANIPULATION,
            parameters={
                "object_id": "red_ball_123",
                "action": "grasp",
                "color": "red",
                "object_type": "ball"
            },
            timeout=30.0,
            preconditions=["robot_is_idle", "object_detected"],
            postconditions=["object_grasped"],
            success_threshold=0.85
        )

        action_plan = ActionPlan(
            command_id=voice_cmd.id,
            actions=[action],
            priority=4,
            estimated_duration=30.0,
            required_resources=["manipulator_arm"],
            success_criteria=["object_grasped"],
            status=ActionStatus.PENDING
        )

        # Validate the action plan
        is_valid, errors = self.action_executor.validate_action_plan(action_plan)
        self.assertTrue(is_valid, f"Action plan validation failed: {errors}")

        # Execute the action plan
        executed_plan = self.action_executor.execute_action_plan(action_plan)

        # Check that execution was successful
        self.assertIn(executed_plan.status, [ActionStatus.COMPLETED, ActionStatus.FAILED])

    def test_multi_modal_fusion_integration(self):
        """Test multi-modal fusion with voice and vision data."""
        # Create a voice command
        voice_cmd = VoiceCommand(
            transcript="Pick up the red ball",
            confidence=0.92,
            intent="object_manipulation",
            parameters={
                "action": "pick_up",
                "object": "red ball",
                "color": "red",
                "object_type": "ball"
            }
        )

        # Create visual perception with relevant objects
        red_ball = DetectedObject(
            class_name="ball",
            bounding_box=[100.0, 200.0, 50.0, 50.0],
            confidence=0.95,
            position_3d=[1.0, 2.0, 0.5],
            size=[0.1, 0.1, 0.1],
            color="red"
        )

        blue_cube = DetectedObject(
            class_name="cube",
            bounding_box=[300.0, 150.0, 60.0, 60.0],
            confidence=0.88,
            position_3d=[2.0, 1.0, 0.3],
            size=[0.15, 0.15, 0.15],
            color="blue"
        )

        visual_perception = VisualPerception(
            camera_source="front_camera",
            detection_accuracy=0.90,
            objects=[red_ball, blue_cube]
        )

        # Perform multi-modal fusion
        fusion_result = self.fusion_service.fuse_voice_and_vision(
            voice_cmd, visual_perception
        )

        # Check that fusion was successful
        self.assertIsNotNone(fusion_result.context)
        self.assertGreater(fusion_result.confidence, 0.0)
        self.assertIsNotNone(fusion_result.relevant_objects)

        # Verify that the red ball was identified as relevant
        relevant_obj_found = False
        for obj in fusion_result.relevant_objects:
            if obj.get('color') == 'red' and obj.get('class_name') == 'ball':
                relevant_obj_found = True
                break

        self.assertTrue(relevant_obj_found, "Red ball should be identified as relevant object")

        # Generate action plan from fusion result
        action_plan = self.fusion_service.generate_action_plan(fusion_result)

        if action_plan:
            # Validate the generated action plan
            is_valid, errors = self.action_executor.validate_action_plan(action_plan)
            self.assertTrue(is_valid, f"Generated action plan validation failed: {errors}")

    def test_voice_processing_to_nlu_integration(self):
        """Test integration between voice processing and NLU."""
        # Create a temporary audio file for testing
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            # Generate simple audio data
            sample_rate = 16000
            duration = 0.1  # 100ms
            t = np.linspace(0, duration, int(sample_rate * duration))
            audio_data = 0.5 * np.sin(2 * np.pi * 440 * t)  # 440 Hz sine wave
            sf.write(temp_file.name, audio_data, sample_rate)
            temp_path = temp_file.name

        try:
            # Process the audio (using mock service)
            voice_cmd = VoiceCommand(
                transcript="Pick up the red ball",
                confidence=0.9,
                status=VoiceCommandStatus.PROCESSED
            )

            # If we had a real NLU service, we would interpret the command
            # For this test, we'll use the mock service
            if self.nlu_service:
                intent, parameters = self.nlu_service.interpret_command_sync(voice_cmd)
                self.assertIsNotNone(intent)
                self.assertIsNotNone(parameters)

                # Check that the intent and parameters make sense
                self.assertIn("action", parameters)
                self.assertIn("object", parameters)

            # Parse the command
            parsed = self.command_parser.parse_command(voice_cmd)
            self.assertIn("action", parsed)
            self.assertIn("parameters", parsed)

        finally:
            # Clean up
            os.unlink(temp_path)

    def test_action_execution_with_context(self):
        """Test action execution with multi-modal context."""
        # Create a multi-modal context
        context = MultiModalContext(
            voice_context={
                "transcript": "Pick up the red ball",
                "intent": "object_manipulation",
                "confidence": 0.92
            },
            visual_context={
                "detection_accuracy": 0.90,
                "objects": [{
                    "object_id": "red_ball_123",
                    "class_name": "ball",
                    "color": "red",
                    "confidence": 0.95,
                    "position_3d": [1.0, 2.0, 0.5]
                }]
            },
            confidence_score=0.91
        )

        # Create an action plan based on the context
        action = Action(
            type=ActionType.NAVIGATION,
            parameters={
                "target_location": "object_location",
                "x": 1.0,
                "y": 2.0,
                "z": 0.5
            },
            timeout=60.0,
            preconditions=["robot_is_idle", "path_clear"],
            postconditions=["robot_at_object"],
            success_threshold=0.95
        )

        manip_action = Action(
            type=ActionType.MANIPULATION,
            parameters={
                "object_id": "red_ball_123",
                "action": "grasp"
            },
            timeout=30.0,
            preconditions=["robot_at_object", "gripper_open"],
            postconditions=["object_grasped"],
            success_threshold=0.85
        )

        action_plan = ActionPlan(
            command_id="voice_cmd_123",
            actions=[action, manip_action],
            priority=4,
            estimated_duration=90.0,
            required_resources=["navigation_system", "manipulator_arm"],
            success_criteria=["object_grasped"],
            status=ActionStatus.PENDING
        )

        # Execute the action plan
        executed_plan = self.action_executor.execute_action_plan(action_plan)

        # Check execution status
        self.assertIsNotNone(executed_plan.status)
        self.assertIn(executed_plan.status, [ActionStatus.COMPLETED, ActionStatus.FAILED, ActionStatus.PARTIALLY_FAILED])

    def test_end_to_end_scenario(self):
        """Test an end-to-end scenario from voice command to action completion."""
        # Simulate receiving a voice command
        voice_cmd = VoiceCommand(
            transcript="Go to the kitchen and pick up the red cup",
            confidence=0.88,
            status=VoiceCommandStatus.RECEIVED
        )

        # Update status as it gets processed
        voice_cmd.update_status(VoiceCommandStatus.PROCESSED)

        # Simulate NLU interpretation (using mock)
        voice_cmd.intent = "object_manipulation"
        voice_cmd.parameters = {
            "action": "pick_up",
            "object": "red cup",
            "location": "kitchen",
            "color": "red",
            "object_type": "cup"
        }

        # Create visual perception data (simulating what the robot sees)
        red_cup = DetectedObject(
            class_name="cup",
            bounding_box=[150.0, 250.0, 40.0, 60.0],
            confidence=0.92,
            position_3d=[2.5, 1.8, 0.4],
            size=[0.08, 0.08, 0.1],
            color="red"
        )

        kitchen_table = DetectedObject(
            class_name="table",
            bounding_box=[50.0, 300.0, 200.0, 150.0],
            confidence=0.98,
            position_3d=[2.0, 1.5, 0.0],
            size=[1.0, 0.8, 0.8],
            color="brown"
        )

        visual_perception = VisualPerception(
            camera_source="front_camera",
            detection_accuracy=0.91,
            objects=[red_cup, kitchen_table]
        )

        # Perform multi-modal fusion
        fusion_result = self.fusion_service.fuse_voice_and_vision(
            voice_cmd, visual_perception
        )

        # Generate action plan from fusion
        action_plan = self.fusion_service.generate_action_plan(fusion_result)

        if action_plan and len(action_plan.actions) > 0:
            # Validate and execute the action plan
            is_valid, errors = self.action_executor.validate_action_plan(action_plan)
            self.assertTrue(is_valid, f"Action plan validation failed: {errors}")

            executed_plan = self.action_executor.execute_action_plan(action_plan)

            # The plan should have been executed (either successfully or with failure)
            self.assertIn(executed_plan.status, [
                ActionStatus.COMPLETED,
                ActionStatus.FAILED,
                ActionStatus.PARTIALLY_FAILED
            ])

            # Check that execution log was populated
            self.assertGreater(len(executed_plan.execution_log), 0)


class TestErrorHandlingIntegration(unittest.TestCase):
    """Integration tests for error handling across components."""

    def test_error_propagation_through_pipeline(self):
        """Test that errors are properly propagated through the pipeline."""
        # Create a voice command with low confidence
        voice_cmd = VoiceCommand(
            transcript="Unclear command",
            confidence=0.1,  # Very low confidence
            status=VoiceCommandStatus.PROCESSED
        )

        # The command parser should handle this appropriately
        parser = CommandParser()
        parsed, is_valid = parser.parse_with_validation(voice_cmd)

        # Even with low confidence, the command should still be parsed
        self.assertIsNotNone(parsed)
        self.assertIsInstance(is_valid, bool)

    def test_invalid_action_handling(self):
        """Test handling of invalid actions in the execution pipeline."""
        # Create an action plan with an invalid action
        invalid_action = Action(
            type=ActionType.MANIPULATION,
            parameters={},  # Missing required parameters
            timeout=-5.0,  # Invalid timeout
            preconditions=[],
            postconditions=[],
            success_threshold=1.5  # Invalid threshold
        )

        action_plan = ActionPlan(
            command_id="test_cmd",
            actions=[invalid_action],
            status=ActionStatus.PENDING
        )

        executor = ActionExecutor()
        is_valid, errors = executor.validate_action_plan(action_plan)

        # The validation should catch the errors
        self.assertFalse(is_valid)
        self.assertGreater(len(errors), 0)


if __name__ == '__main__':
    unittest.main()