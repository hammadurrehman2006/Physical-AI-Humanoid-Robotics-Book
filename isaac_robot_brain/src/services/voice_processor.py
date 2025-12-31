"""
Voice Processor Service

This module implements the voice processing service that handles speech recognition
and voice command processing functionality.
"""

import asyncio
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

from ..models.voice_command import VoiceCommand, VoiceCommandStatus
from .whisper_service import WhisperService


class VoiceProcessor:
    """
    Voice processing service that handles speech recognition and voice command processing.
    """

    def __init__(self, whisper_service: WhisperService = None):
        """
        Initialize the voice processor service.

        Args:
            whisper_service: Optional Whisper service for speech recognition.
                           If not provided, a default one will be created.
        """
        self.whisper_service = whisper_service or WhisperService()
        self.logger = logging.getLogger(__name__)
        self.active_commands = {}

    async def process_audio(self, audio_path: str, language: str = "en") -> VoiceCommand:
        """
        Process audio file and convert to voice command.

        Args:
            audio_path: Path to the audio file to process
            language: Language of the audio (default: "en")

        Returns:
            VoiceCommand with transcript and confidence
        """
        try:
            # Update command status to processing
            command = VoiceCommand(
                audio_data=audio_path,
                language=language,
                status=VoiceCommandStatus.PROCESSING
            )

            self.logger.info(f"Processing audio file: {audio_path}")

            # Perform speech recognition using Whisper
            transcript, confidence = await self.whisper_service.transcribe_audio(audio_path)

            # Set the transcript and confidence
            command.set_transcript(transcript, confidence)

            # Update status to processed
            command.update_status(VoiceCommandStatus.PROCESSED)

            self.logger.info(f"Successfully processed audio: {transcript[:50]}...")

            return command

        except Exception as e:
            self.logger.error(f"Error processing audio {audio_path}: {str(e)}")
            # Create a command with error status
            error_command = VoiceCommand(
                audio_data=audio_path,
                language=language,
                status=VoiceCommandStatus.FAILED_PROCESSING
            )
            return error_command

    def process_audio_sync(self, audio_path: str, language: str = "en") -> VoiceCommand:
        """
        Synchronous version of process_audio.

        Args:
            audio_path: Path to the audio file to process
            language: Language of the audio (default: "en")

        Returns:
            VoiceCommand with transcript and confidence
        """
        # Run the async method in a new event loop
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        return loop.run_until_complete(self.process_audio(audio_path, language))

    async def batch_process_audio(self, audio_paths: list, language: str = "en") -> list:
        """
        Process multiple audio files concurrently.

        Args:
            audio_paths: List of audio file paths to process
            language: Language of the audio (default: "en")

        Returns:
            List of processed VoiceCommand objects
        """
        tasks = [self.process_audio(path, language) for path in audio_paths]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Handle any exceptions that occurred during processing
        processed_commands = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                self.logger.error(f"Error processing audio {audio_paths[i]}: {str(result)}")
                # Create an error command for this audio
                error_command = VoiceCommand(
                    audio_data=audio_paths[i],
                    language=language,
                    status=VoiceCommandStatus.FAILED_PROCESSING
                )
                processed_commands.append(error_command)
            else:
                processed_commands.append(result)

        return processed_commands

    def add_noise_filtering(self, audio_data: bytes) -> bytes:
        """
        Apply noise filtering to audio data.

        Args:
            audio_data: Raw audio data to filter

        Returns:
            Filtered audio data
        """
        # Placeholder for noise filtering implementation
        # In a real implementation, this would use audio processing libraries
        # like librosa or scipy to apply noise reduction filters
        self.logger.info("Applying noise filtering to audio data")
        return audio_data

    def validate_audio_format(self, audio_path: str) -> bool:
        """
        Validate that the audio file format is supported.

        Args:
            audio_path: Path to the audio file

        Returns:
            True if format is supported, False otherwise
        """
        supported_formats = ['.wav', '.mp3', '.m4a', '.flac', '.aac']
        return any(audio_path.lower().endswith(fmt) for fmt in supported_formats)

    def get_command_status(self, command_id: str) -> Optional[VoiceCommandStatus]:
        """
        Get the status of a voice command.

        Args:
            command_id: ID of the command to check

        Returns:
            Status of the command, or None if not found
        """
        command = self.active_commands.get(command_id)
        return command.status if command else None


# Example usage
if __name__ == "__main__":
    import tempfile
    import os

    # Create a mock Whisper service for testing
    class MockWhisperService:
        async def transcribe_audio(self, audio_path: str) -> tuple:
            # Mock transcription
            return "Test command for demonstration", 0.95

    # Test the voice processor
    mock_whisper = MockWhisperService()
    processor = VoiceProcessor(whisper_service=mock_whisper)

    # Create a temporary audio file path for testing
    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
        temp_path = temp_file.name

    try:
        # Process the mock audio file
        result = processor.process_audio_sync(temp_path)
        print(f"Processed command: {result.transcript}")
        print(f"Confidence: {result.confidence}")
        print(f"Status: {result.status.value}")
    finally:
        # Clean up the temporary file
        os.unlink(temp_path)