"""
Whisper Service for Speech Recognition

This module provides integration with OpenAI Whisper for speech-to-text conversion.
"""
import asyncio
import logging
from typing import Optional, Dict, Any, Union
import numpy as np
import torch
from pathlib import Path
import whisper
import tempfile
import wave
import io
import base64
from ..models.voice_command import VoiceCommand, VoiceCommandStatus


class WhisperServiceError(Exception):
    """Custom exception for Whisper service errors"""
    pass


class WhisperService:
    """
    Service class for handling Whisper-based speech recognition.

    This service provides methods to convert audio data to text using OpenAI's Whisper model.
    """

    def __init__(self, model_size: str = "base", device: str = None, compute_type: str = "float32"):
        """
        Initialize the Whisper service.

        Args:
            model_size: Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
            device: Device to run the model on ('cpu', 'cuda', or None for auto-detection)
            compute_type: Type of computation ('float32', 'float16', etc.)
        """
        self.logger = logging.getLogger(__name__)
        self.model_size = model_size
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.compute_type = compute_type

        try:
            self.logger.info(f"Loading Whisper model '{model_size}' on {self.device}...")
            self.model = whisper.load_model(model_size, device=self.device)
            self.logger.info("Whisper model loaded successfully")
        except Exception as e:
            self.logger.error(f"Failed to load Whisper model: {e}")
            raise WhisperServiceError(f"Failed to load Whisper model: {e}")

    def _audio_to_wav_bytes(self, audio_data: Union[str, bytes, np.ndarray], sample_rate: int = 16000) -> bytes:
        """
        Convert various audio formats to WAV bytes suitable for Whisper.

        Args:
            audio_data: Audio data in various formats (file path, bytes, numpy array)
            sample_rate: Sample rate for the audio

        Returns:
            bytes: WAV formatted audio data
        """
        try:
            if isinstance(audio_data, str):
                # If it's a file path
                with open(audio_data, 'rb') as f:
                    return f.read()
            elif isinstance(audio_data, bytes):
                # If it's already bytes, assume it's WAV format
                return audio_data
            elif isinstance(audio_data, np.ndarray):
                # If it's a numpy array, convert to WAV
                with io.BytesIO() as buffer:
                    with wave.open(buffer, 'wb') as wav_file:
                        wav_file.setnchannels(1)  # Mono
                        wav_file.setsampwidth(2)  # 16-bit
                        wav_file.setframerate(sample_rate)
                        # Convert numpy array to bytes
                        audio_bytes = (audio_data * 32767).astype(np.int16).tobytes()
                        wav_file.writeframes(audio_bytes)
                    return buffer.getvalue()
            else:
                raise ValueError(f"Unsupported audio data type: {type(audio_data)}")
        except Exception as e:
            self.logger.error(f"Error converting audio to WAV: {e}")
            raise WhisperServiceError(f"Error converting audio to WAV: {e}")

    def transcribe_audio(self,
                        audio_data: Union[str, bytes, np.ndarray],
                        language: str = None,
                        temperature: float = 0.0) -> Dict[str, Any]:
        """
        Transcribe audio data to text using Whisper.

        Args:
            audio_data: Audio data in various formats
            language: Language code (e.g., 'en', 'es', 'fr') or None for auto-detection
            temperature: Temperature for sampling (0.0 for deterministic output)

        Returns:
            Dict containing the transcription result
        """
        try:
            # Convert audio to WAV bytes if needed
            wav_bytes = self._audio_to_wav_bytes(audio_data)

            # Create a temporary file for Whisper to process
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_file.write(wav_bytes)
                temp_path = temp_file.name

            try:
                # Transcribe the audio
                result = self.model.transcribe(
                    temp_path,
                    language=language,
                    temperature=temperature,
                    verbose=False
                )

                # Clean up temporary file
                import os
                os.unlink(temp_path)

                return {
                    'text': result['text'].strip(),
                    'segments': result['segments'],
                    'language': result['language'],
                    'confidence': self._calculate_confidence(result)
                }
            except Exception as e:
                # Clean up temporary file in case of error
                import os
                if os.path.exists(temp_path):
                    os.unlink(temp_path)
                raise e

        except Exception as e:
            self.logger.error(f"Error during transcription: {e}")
            raise WhisperServiceError(f"Error during transcription: {e}")

    async def transcribe_audio_async(self,
                                   audio_data: Union[str, bytes, np.ndarray],
                                   language: str = None,
                                   temperature: float = 0.0) -> Dict[str, Any]:
        """
        Asynchronously transcribe audio data to text using Whisper.

        Args:
            audio_data: Audio data in various formats
            language: Language code or None for auto-detection
            temperature: Temperature for sampling

        Returns:
            Dict containing the transcription result
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None,
            self.transcribe_audio,
            audio_data,
            language,
            temperature
        )

    def _calculate_confidence(self, result: Dict[str, Any]) -> float:
        """
        Calculate an overall confidence score based on the transcription result.

        Args:
            result: The transcription result from Whisper

        Returns:
            float: Confidence score between 0.0 and 1.0
        """
        try:
            # Calculate confidence based on segment probabilities if available
            segments = result.get('segments', [])
            if segments:
                # Average the probabilities of all segments
                total_prob = 0.0
                count = 0
                for segment in segments:
                    if 'probability' in segment:
                        total_prob += segment['probability']
                        count += 1
                    elif 'avg_logprob' in segment:
                        # Convert log probability to a more intuitive confidence score
                        avg_logprob = segment['avg_logprob']
                        # Normalize log probability to 0-1 range (this is a simplified approach)
                        confidence = max(0.0, min(1.0, (avg_logprob + 10) / 10))
                        total_prob += confidence
                        count += 1

                if count > 0:
                    return total_prob / count

            # If no segment probabilities available, return a default confidence
            return 0.8  # Default confidence for Whisper transcriptions

        except Exception as e:
            self.logger.warning(f"Error calculating confidence: {e}")
            return 0.8  # Default confidence

    def create_voice_command(self,
                           audio_data: Union[str, bytes, np.ndarray],
                           language: str = "en") -> VoiceCommand:
        """
        Create a VoiceCommand object from audio data by transcribing it.

        Args:
            audio_data: Audio data to transcribe
            language: Language code for transcription

        Returns:
            VoiceCommand: The created voice command object
        """
        try:
            # Transcribe the audio
            transcription_result = self.transcribe_audio(audio_data, language=language)

            # Create the VoiceCommand object
            voice_command = VoiceCommand(
                audio_data=None,  # We don't store the raw audio data in the object
                transcript=transcription_result['text'],
                confidence=transcription_result['confidence'],
                language=transcription_result['language'],
                status=VoiceCommandStatus.PROCESSED
            )

            return voice_command

        except Exception as e:
            self.logger.error(f"Error creating voice command: {e}")
            # Create a failed voice command
            voice_command = VoiceCommand(
                status=VoiceCommandStatus.FAILED_PROCESSING,
                confidence=0.0,
                transcript=""
            )
            return voice_command

    def batch_transcribe(self, audio_files: list) -> list:
        """
        Transcribe multiple audio files in batch.

        Args:
            audio_files: List of audio file paths or data

        Returns:
            List of transcription results
        """
        results = []
        for audio_file in audio_files:
            try:
                result = self.transcribe_audio(audio_file)
                results.append(result)
            except Exception as e:
                self.logger.error(f"Error transcribing {audio_file}: {e}")
                results.append({
                    'text': '',
                    'segments': [],
                    'language': 'unknown',
                    'confidence': 0.0,
                    'error': str(e)
                })
        return results

    def get_available_models(self) -> list:
        """
        Get list of available Whisper model sizes.

        Returns:
            List of available model sizes
        """
        return ['tiny', 'base', 'small', 'medium', 'large', 'large-v2', 'large-v3']