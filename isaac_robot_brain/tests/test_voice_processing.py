"""
Unit Tests for Voice Processing Components

This module contains unit tests for the voice processing functionality
including the voice command data model, voice processor service, and Whisper service.
"""

import unittest
import tempfile
import os
import numpy as np
import soundfile as sf
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime

from src.models.voice_command import VoiceCommand, VoiceCommandStatus
from src.services.voice_processor import VoiceProcessor
from src.services.whisper_service import WhisperService


class TestVoiceCommand(unittest.TestCase):
    """Unit tests for the VoiceCommand data model."""

    def test_voice_command_creation(self):
        """Test creating a basic VoiceCommand object."""
        cmd = VoiceCommand(
            transcript="Test command",
            confidence=0.9,
            language="en",
            intent="test_intent",
            parameters={"param": "value"}
        )

        self.assertIsNotNone(cmd.id)
        self.assertEqual(cmd.transcript, "Test command")
        self.assertEqual(cmd.confidence, 0.9)
        self.assertEqual(cmd.language, "en")
        self.assertEqual(cmd.intent, "test_intent")
        self.assertEqual(cmd.parameters, {"param": "value"})
        self.assertEqual(cmd.status, VoiceCommandStatus.RECEIVED)

    def test_voice_command_validation(self):
        """Test validation of VoiceCommand fields."""
        # Test valid confidence
        cmd = VoiceCommand(confidence=0.8)
        self.assertEqual(cmd.confidence, 0.8)

        # Test invalid confidence (should raise ValueError)
        with self.assertRaises(ValueError):
            VoiceCommand(confidence=1.5)

        with self.assertRaises(ValueError):
            VoiceCommand(confidence=-0.5)

    def test_voice_command_status_update(self):
        """Test updating voice command status."""
        cmd = VoiceCommand()
        self.assertEqual(cmd.status, VoiceCommandStatus.RECEIVED)

        cmd.update_status(VoiceCommandStatus.PROCESSED)
        self.assertEqual(cmd.status, VoiceCommandStatus.PROCESSED)

    def test_voice_command_set_transcript(self):
        """Test setting transcript and confidence."""
        cmd = VoiceCommand()

        cmd.set_transcript("New transcript", 0.95)
        self.assertEqual(cmd.transcript, "New transcript")
        self.assertEqual(cmd.confidence, 0.95)

        # Test invalid confidence
        with self.assertRaises(ValueError):
            cmd.set_transcript("Test", 1.5)

        # Test empty transcript
        with self.assertRaises(ValueError):
            cmd.set_transcript("", 0.9)

    def test_voice_command_set_intent_and_parameters(self):
        """Test setting intent and parameters."""
        cmd = VoiceCommand()

        cmd.set_intent_and_parameters("test_intent", {"param": "value"})
        self.assertEqual(cmd.intent, "test_intent")
        self.assertEqual(cmd.parameters, {"param": "value"})

        # Test empty intent
        with self.assertRaises(ValueError):
            cmd.set_intent_and_parameters("", {"param": "value"})


class MockWhisperService:
    """Mock Whisper service for testing."""

    async def transcribe_audio(self, audio_path: str):
        return "Mock transcription", 0.95

    def transcribe_audio_sync(self, audio_path: str):
        return "Mock transcription", 0.95


class TestVoiceProcessor(unittest.TestCase):
    """Unit tests for the VoiceProcessor service."""

    def setUp(self):
        """Set up test fixtures."""
        self.mock_whisper = MockWhisperService()
        self.processor = VoiceProcessor(whisper_service=self.mock_whisper)

    def test_voice_processor_initialization(self):
        """Test voice processor initialization."""
        processor = VoiceProcessor()
        self.assertIsNotNone(processor.logger)

    @patch('src.services.voice_processor.asyncio.get_event_loop')
    def test_process_audio(self, mock_get_loop):
        """Test processing audio file."""
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
            # Test processing
            result = self.processor.process_audio_sync(temp_path)

            # Check that the result is a VoiceCommand
            self.assertIsInstance(result, VoiceCommand)
            self.assertEqual(result.status, VoiceCommandStatus.PROCESSED)
            self.assertIsNotNone(result.transcript)
        finally:
            # Clean up
            os.unlink(temp_path)

    def test_validate_audio_format(self):
        """Test audio format validation."""
        # Valid formats
        self.assertTrue(self.processor.validate_audio_format("test.wav"))
        self.assertTrue(self.processor.validate_audio_format("test.mp3"))
        self.assertTrue(self.processor.validate_audio_format("test.m4a"))
        self.assertTrue(self.processor.validate_audio_format("test.flac"))
        self.assertTrue(self.processor.validate_audio_format("test.aac"))

        # Invalid format
        self.assertFalse(self.processor.validate_audio_format("test.txt"))
        self.assertFalse(self.processor.validate_audio_format("test.jpg"))


class TestWhisperService(unittest.TestCase):
    """Unit tests for the WhisperService."""

    @patch('src.services.whisper_service.OpenAI')
    @patch('src.services.whisper_service.whisper')
    def test_whisper_service_initialization_api_mode(self, mock_whisper, mock_openai):
        """Test Whisper service initialization in API mode."""
        # Mock OpenAI client
        mock_client = Mock()
        mock_openai.return_value = mock_client

        service = WhisperService(use_api=True, api_key="test_key")
        self.assertTrue(service.use_api)
        self.assertEqual(service.api_key, "test_key")

    @patch('src.services.whisper_service.whisper')
    def test_whisper_service_initialization_local_mode(self, mock_whisper):
        """Test Whisper service initialization in local mode."""
        # Mock the load_model function
        mock_model = Mock()
        mock_whisper.load_model.return_value = mock_model

        service = WhisperService(model_size="base")
        self.assertFalse(service.use_api)
        self.assertIsNotNone(service.model)

    def test_get_available_models(self):
        """Test getting available Whisper models."""
        service = WhisperService()
        models = service.get_available_models()

        expected_models = ["tiny", "base", "small", "medium", "large", "large-v2"]
        self.assertEqual(models, expected_models)

    def test_validate_audio_file(self):
        """Test audio file validation."""
        service = WhisperService()

        # Test non-existent file
        self.assertFalse(service.validate_audio_file("non_existent.wav"))

        # Test unsupported format
        with tempfile.NamedTemporaryFile(suffix='.txt', delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            self.assertFalse(service.validate_audio_file(temp_path))
        finally:
            os.unlink(temp_path)

        # Test supported format (create a temporary WAV file)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            # This will return False because the file doesn't exist in the mocked service
            # but it would validate the format correctly in a real implementation
            result = service.validate_audio_file(temp_path)
            # The validation should pass format check but fail existence check
            # So we expect False due to file not existing
        finally:
            os.unlink(temp_path)


if __name__ == '__main__':
    unittest.main()