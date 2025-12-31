# Step-by-Step Tutorial: Voice Processing System Setup

## Overview

This tutorial will guide you through setting up a complete voice processing system for the Vision-Language-Action module. By the end of this tutorial, you'll have a working system that can capture audio, preprocess it, recognize speech, and parse commands.

## Prerequisites

- Python 3.10+
- ROS 2 Humble Hawksbill installed
- Microphone hardware (real or simulated)
- OpenAI API key (for Whisper integration)
- Basic understanding of audio processing concepts

## Step 1: Environment Setup

### 1.1 Create Virtual Environment

```bash
# Create and activate a virtual environment
python3 -m venv voice_processing_env
source voice_processing_env/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 1.2 Install System Dependencies

```bash
# On Ubuntu/Debian
sudo apt-get update
sudo apt-get install portaudio19-dev python3-pyaudio build-essential

# On macOS (using Homebrew)
brew install portaudio
```

### 1.3 Install Python Dependencies

```bash
# Install core dependencies
pip install pyaudio numpy scipy librosa pydub vosk rclpy

# Install OpenAI for Whisper integration
pip install openai

# Install additional audio processing libraries
pip install soundfile webrtcvad
```

### 1.4 Set Up Environment Variables

```bash
# Set your OpenAI API key
export OPENAI_API_KEY="your_openai_api_key_here"

# Set Vosk model path (optional, for local recognition)
export VOSK_MODEL_PATH="/path/to/vosk/model"
```

## Step 2: Download Speech Recognition Models

### 2.1 Download Vosk English Model

```bash
# Create models directory
mkdir -p models

# Download Vosk English model (about 50MB)
wget -O models/vosk-model-en-us-0.22.zip https://alphacephei.com/vosk/models/vosk-model-en-us-0.22.zip

# Extract the model
cd models && unzip vosk-model-en-us-0.22.zip && cd ..

# Clean up
rm models/vosk-model-en-us-0.22.zip

# Set environment variable
export VOSK_MODEL_PATH=$(pwd)/models/vosk-model-en-us-0.22
```

## Step 3: Create Core Audio Processing Components

### 3.1 Create Audio Input Node

Create a file `audio_input_node.py`:

```python
#!/usr/bin/env python3
"""
Audio Input Node for Voice Processing System
Captures audio from microphone and publishes to ROS 2 topic
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import Header
import pyaudio
import numpy as np
import threading
import time

class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')

        # Audio configuration parameters
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('format', 16)  # 16-bit

        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.audio_format = pyaudio.paInt16

        # Publisher for audio data
        self.audio_publisher = self.create_publisher(AudioData, 'audio_stream', 10)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Audio stream
        self.stream = None
        self.is_recording = False

        self.get_logger().info(f'Audio input node initialized with sample_rate: {self.sample_rate}, channels: {self.channels}')

    def start_audio_capture(self):
        """Start audio capture from microphone"""
        try:
            self.stream = self.audio.open(
                format=self.audio_format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            self.is_recording = True

            # Start recording in a separate thread
            self.recording_thread = threading.Thread(target=self.record_audio)
            self.recording_thread.daemon = True
            self.recording_thread.start()

            self.get_logger().info('Audio capture started successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start audio capture: {e}')
            return False

    def record_audio(self):
        """Record audio in a continuous loop"""
        while self.is_recording:
            try:
                # Read audio data from stream
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Convert to AudioData message
                audio_msg = AudioData()
                audio_msg.data = data

                # Add timestamp
                audio_msg.header = Header()
                audio_msg.header.stamp = self.get_clock().now().to_msg()

                # Publish audio data
                self.audio_publisher.publish(audio_msg)

            except Exception as e:
                self.get_logger().error(f'Error recording audio: {e}')
                break

    def stop_audio_capture(self):
        """Stop audio capture"""
        self.is_recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)

    node = AudioInputNode()

    # Start audio capture
    if node.start_audio_capture():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted, shutting down...')
        finally:
            node.stop_audio_capture()
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.2 Create Audio Preprocessing Module

Create a file `audio_preprocessor.py`:

```python
#!/usr/bin/env python3
"""
Audio Preprocessing Module
Implements noise reduction, voice activity detection, and signal enhancement
"""
import numpy as np
from scipy import signal
import librosa
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class AudioFeatures:
    """Data structure for audio features"""
    energy: float
    zero_crossing_rate: float
    spectral_centroid: float
    mfccs: np.ndarray
    is_speech: bool

class AudioPreprocessor:
    def __init__(self, sample_rate=44100, frame_length=1024, hop_length=512):
        self.sample_rate = sample_rate
        self.frame_length = frame_length
        self.hop_length = hop_length

        # Noise estimation parameters
        self.noise_buffer_size = 100  # frames for noise estimation
        self.noise_buffer = []
        self.noise_profile = None

        # Voice activity detection parameters
        self.energy_threshold = 0.01
        self.zero_crossing_threshold = 50
        self.snr_threshold = 10.0  # dB

    def preprocess_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Apply preprocessing pipeline to audio data"""
        # Convert to float32 if needed
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)

        # Normalize audio
        audio_data = self.normalize_audio(audio_data)

        # Apply noise reduction
        audio_data = self.noise_reduction(audio_data)

        # Apply pre-emphasis filter
        audio_data = self.pre_emphasis_filter(audio_data)

        return audio_data

    def normalize_audio(self, audio_data: np.ndarray) -> np.ndarray:
        """Normalize audio to unit amplitude"""
        max_amplitude = np.max(np.abs(audio_data))
        if max_amplitude > 0:
            return audio_data / max_amplitude
        return audio_data

    def pre_emphasis_filter(self, audio_data: np.ndarray, alpha=0.97) -> np.ndarray:
        """Apply pre-emphasis filter to boost high frequencies"""
        return np.append(audio_data[0], audio_data[1:] - alpha * audio_data[:-1])

    def noise_reduction(self, audio_data: np.ndarray, reduction_factor=0.3) -> np.ndarray:
        """Apply noise reduction using spectral subtraction"""
        # Convert to frequency domain using STFT
        stft = librosa.stft(audio_data, n_fft=self.frame_length, hop_length=self.hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # Estimate noise profile if not available
        if self.noise_profile is None:
            self.estimate_noise_profile(magnitude)

        # Apply spectral subtraction
        enhanced_magnitude = np.maximum(magnitude - self.noise_profile * reduction_factor, 0)

        # Reconstruct audio using inverse STFT
        enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
        enhanced_audio = librosa.istft(enhanced_stft, hop_length=self.hop_length, length=len(audio_data))

        return enhanced_audio.astype(np.float32)

    def estimate_noise_profile(self, magnitude_spectrum: np.ndarray):
        """Estimate noise profile from initial frames"""
        # For this exercise, use the first few frames as noise estimate
        # In practice, this would be updated adaptively
        if magnitude_spectrum.shape[1] >= 10:
            self.noise_profile = np.mean(magnitude_spectrum[:, :10], axis=1, keepdims=True)
        else:
            self.noise_profile = np.mean(magnitude_spectrum, axis=1, keepdims=True)

    def voice_activity_detection(self, audio_data: np.ndarray) -> bool:
        """Detect voice activity in audio segment"""
        # Calculate energy
        energy = np.mean(audio_data ** 2)

        # Calculate zero-crossing rate
        zero_crossings = np.sum(audio_data[:-1] * audio_data[1:] < 0)
        zcr = zero_crossings / len(audio_data)

        # Calculate SNR (simplified approach)
        signal_power = np.mean(audio_data ** 2)
        noise_power = self.estimate_noise_power(audio_data)
        snr = 10 * np.log10(signal_power / (noise_power + 1e-10))

        # Voice activity if all conditions are met
        has_energy = energy > self.energy_threshold
        has_appropriate_zcr = zcr > 0.001 and zcr < 0.05
        has_good_snr = snr > self.snr_threshold

        return has_energy and has_appropriate_zcr and has_good_snr

    def estimate_noise_power(self, audio_data: np.ndarray) -> float:
        """Estimate noise power using minimum statistics"""
        # Simplified noise estimation
        frame_size = 1024
        frames = self._frame_audio(audio_data, frame_size)
        frame_energies = [np.mean(frame ** 2) for frame in frames]

        # Use minimum of frame energies as noise estimate
        if frame_energies:
            return min(frame_energies)
        return 0.0

    def extract_features(self, audio_data: np.ndarray) -> AudioFeatures:
        """Extract audio features for analysis"""
        # Calculate energy
        energy = np.mean(audio_data ** 2)

        # Calculate zero-crossing rate
        zero_crossings = np.sum(audio_data[:-1] * audio_data[1:] < 0)
        zcr = zero_crossings / len(audio_data)

        # Calculate spectral centroid
        spectral_centroid = np.mean(librosa.feature.spectral_centroid(
            y=audio_data, sr=self.sample_rate
        ))

        # Calculate MFCCs (first 13 coefficients)
        mfccs = librosa.feature.mfcc(y=audio_data, sr=self.sample_rate, n_mfcc=13)
        mfccs = np.mean(mfccs, axis=1)  # Average over time

        # Voice activity detection
        is_speech = self.voice_activity_detection(audio_data)

        return AudioFeatures(
            energy=energy,
            zero_crossing_rate=zcr,
            spectral_centroid=spectral_centroid,
            mfccs=mfccs,
            is_speech=is_speech
        )

    def _frame_audio(self, audio_data: np.ndarray, frame_length: int) -> list:
        """Split audio into overlapping frames"""
        frames = []
        for i in range(0, len(audio_data) - frame_length, self.hop_length):
            frames.append(audio_data[i:i + frame_length])
        return frames

class StreamingAudioProcessor:
    """Process audio in real-time with buffering"""
    def __init__(self, preprocessor: AudioPreprocessor):
        self.preprocessor = preprocessor
        self.audio_buffer = np.array([])
        self.min_speech_duration = 0.5 * preprocessor.sample_rate  # 0.5 seconds
        self.max_buffer_duration = 5.0 * preprocessor.sample_rate  # 5 seconds
        self.speech_detected = False
        self.speech_start_time = 0

    def process_chunk(self, audio_chunk: np.ndarray) -> Optional[np.ndarray]:
        """Process an incoming audio chunk and return speech segments"""
        # Add chunk to buffer
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])

        # Check if buffer is too large
        if len(self.audio_buffer) > self.max_buffer_duration:
            self.audio_buffer = self.audio_buffer[-int(self.max_buffer_duration):]

        # Check for voice activity in the current buffer
        is_speech = self.preprocessor.voice_activity_detection(self.audio_buffer)

        if is_speech:
            if not self.speech_detected:
                # Speech just started
                self.speech_detected = True
                self.speech_start_time = len(self.audio_buffer)
            # Continue buffering
        else:
            if self.speech_detected:
                # Speech just ended - return the speech segment
                speech_segment = self.audio_buffer
                self.audio_buffer = np.array([])
                self.speech_detected = False

                # Only return if segment is long enough
                if len(speech_segment) >= self.min_speech_duration:
                    return speech_segment

        return None  # No complete speech segment ready yet
```

### 3.3 Create Speech Recognition Module

Create a file `speech_recognizer.py`:

```python
#!/usr/bin/env python3
"""
Speech Recognition Module
Integrates cloud-based (Whisper) and local (Vosk) recognition
"""
import openai
import base64
import io
import numpy as np
from pydub import AudioSegment
from vosk import Model, KaldiRecognizer
import json
import threading
import queue
from typing import Optional, List, Dict, Any
import tempfile
import os

class WhisperSpeechRecognizer:
    """Cloud-based speech recognition using OpenAI Whisper"""
    def __init__(self, api_key: str, model: str = "whisper-1"):
        self.api_key = api_key
        self.model = model
        openai.api_key = api_key

    def transcribe_audio(self, audio_data: bytes, language: str = "en") -> str:
        """Transcribe audio using OpenAI Whisper API"""
        try:
            # Create a temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                # Convert audio data to AudioSegment
                audio_segment = AudioSegment(
                    data=audio_data,
                    sample_width=2,  # 16-bit
                    frame_rate=44100,
                    channels=1
                )

                # Export to temporary file
                temp_filename = temp_file.name
                audio_segment.export(temp_filename, format="wav")

            # Transcribe using Whisper API
            with open(temp_filename, "rb") as audio_file:
                response = openai.Audio.transcribe(
                    model=self.model,
                    file=audio_file,
                    language=language,
                    response_format="text"
                )

            # Clean up temporary file
            os.unlink(temp_filename)

            return response.strip()
        except Exception as e:
            print(f"Whisper transcription error: {e}")
            return ""

    def transcribe_file(self, file_path: str, language: str = "en") -> str:
        """Transcribe audio file using OpenAI Whisper API"""
        try:
            with open(file_path, "rb") as audio_file:
                response = openai.Audio.transcribe(
                    model=self.model,
                    file=audio_file,
                    language=language,
                    response_format="text"
                )
            return response.strip()
        except Exception as e:
            print(f"Whisper file transcription error: {e}")
            return ""

class LocalSpeechRecognizer:
    """Local speech recognition using Vosk"""
    def __init__(self, model_path: str, sample_rate: int = 16000):
        self.model_path = model_path
        self.sample_rate = sample_rate
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, sample_rate)
        self.partial_result = ""

    def transcribe_audio_chunk(self, audio_chunk: bytes) -> str:
        """Transcribe a single audio chunk"""
        if self.recognizer.AcceptWaveform(audio_chunk):
            result = self.recognizer.Result()
            result_dict = json.loads(result)
            final_text = result_dict.get('text', '')
            self.partial_result = ""
            return final_text
        else:
            # Partial result
            partial_result = self.recognizer.PartialResult()
            partial_dict = json.loads(partial_result)
            self.partial_result = partial_dict.get('partial', '')
            return ""

    def finalize_recognition(self) -> str:
        """Get final result after all chunks are processed"""
        result = self.recognizer.FinalResult()
        result_dict = json.loads(result)
        return result_dict.get('text', '')

    def reset(self):
        """Reset the recognizer for a new audio stream"""
        self.recognizer = KaldiRecognizer(self.model, self.sample_rate)
        self.partial_result = ""

class MultiModalSpeechRecognizer:
    """Combines cloud and local speech recognition with fallback"""
    def __init__(self, whisper_api_key: str, vosk_model_path: str):
        self.whisper_recognizer = WhisperSpeechRecognizer(whisper_api_key)
        self.local_recognizer = LocalSpeechRecognizer(vosk_model_path)
        self.use_cloud_fallback = True
        self.confidence_threshold = 0.7

    def transcribe_audio(self, audio_data: bytes, language: str = "en", use_local: bool = True) -> Dict[str, Any]:
        """Transcribe audio with fallback options"""
        result = {
            'text': '',
            'confidence': 0.0,
            'method': 'none',
            'success': False
        }

        if use_local:
            # Try local recognition first
            try:
                # Process audio through local recognizer
                # Note: Vosk expects 16kHz audio, so we may need to resample
                local_text = self.local_recognizer.transcribe_audio_chunk(audio_data)
                if local_text:
                    result.update({
                        'text': local_text,
                        'confidence': 0.8,  # Local confidence estimation
                        'method': 'local',
                        'success': True
                    })
                    return result
            except Exception as e:
                print(f"Local recognition failed: {e}")

        # Fallback to cloud recognition
        if self.use_cloud_fallback:
            try:
                cloud_text = self.whisper_recognizer.transcribe_audio(audio_data, language)
                if cloud_text:
                    result.update({
                        'text': cloud_text,
                        'confidence': 0.9,  # Whisper typically has high confidence
                        'method': 'cloud',
                        'success': True
                    })
                    return result
            except Exception as e:
                print(f"Cloud recognition failed: {e}")

        return result

    def transcribe_continuous_stream(self, audio_stream, language: str = "en") -> List[Dict[str, Any]]:
        """Transcribe a continuous audio stream"""
        results = []
        buffer = b""

        for chunk in audio_stream:
            buffer += chunk

            # Process when buffer reaches appropriate size
            if len(buffer) >= 16000 * 2:  # 2 seconds of 16kHz audio
                result = self.transcribe_audio(buffer, language)
                if result['success'] and result['text'].strip():
                    results.append(result)
                buffer = b""  # Clear buffer

        # Process remaining buffer
        if buffer:
            result = self.transcribe_audio(buffer, language)
            if result['success'] and result['text'].strip():
                results.append(result)

        return results
```

### 3.4 Create Voice Command Parser

Create a file `command_parser.py`:

```python
#!/usr/bin/env python3
"""
Voice Command Parser
Extracts intents and entities from transcribed speech
"""
import re
import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Any, Union
import numpy as np

@dataclass
class VoiceCommand:
    """Data structure for parsed voice commands"""
    intent: str
    entities: Dict[str, Any]
    confidence: float
    original_text: str
    processed_text: str

class VoiceCommandParser:
    """Parses transcribed voice commands into structured actions"""
    def __init__(self):
        self.command_patterns = {
            'move': [
                r'move\s+(?P<direction>forward|backward|left|right|up|down|north|south|east|west)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>meters|meter|m|cm|centimeters|steps)?',
                r'go\s+(?P<direction>forward|backward|left|right|north|south|east|west)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>meters|meter|m|cm|centimeters|steps)?',
                r'walk\s+(?P<direction>forward|backward|left|right|north|south|east|west)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>meters|meter|m|cm|centimeters|steps)?',
                r'go\s+(?P<distance>\d+\.?\d*)\s*(?P<unit>meters|meter|m|cm|centimeters)\s+(?P<direction>forward|backward|left|right|north|south|east|west)'
            ],
            'grasp': [
                r'grasp\s+(?P<object>\w+(?:\s+\w+)*)',
                r'pick\s+up\s+(?P<object>\w+(?:\s+\w+)*)',
                r'grab\s+(?P<object>\w+(?:\s+\w+)*)',
                r'take\s+(?P<object>\w+(?:\s+\w+)*)',
                r'pick\s+(?P<object>\w+(?:\s+\w+)*)\s+up'
            ],
            'navigate': [
                r'go\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'navigate\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'move\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'go\s+(?P<location>\w+(?:\s+\w+)*)',
                r'head\s+to\s+(?P<location>\w+(?:\s+\w+)*)'
            ],
            'inspect': [
                r'look\s+at\s+(?P<object>\w+(?:\s+\w+)*)',
                r'inspect\s+(?P<object>\w+(?:\s+\w+)*)',
                r'check\s+(?P<object>\w+(?:\s+\w+)*)',
                r'observe\s+(?P<object>\w+(?:\s+\w+)*)',
                r'find\s+(?P<object>\w+(?:\s+\w+)*)'
            ],
            'stop': [
                r'stop',
                r'halt',
                r'pause',
                r'freeze',
                r'wait'
            ],
            'follow': [
                r'follow\s+(?P<target>\w+(?:\s+\w+)*)',
                r'come\s+with\s+(?P<target>\w+(?:\s+\w+)*)',
                r'accompany\s+(?P<target>\w+(?:\s+\w+)*)'
            ],
            'bring': [
                r'bring\s+(?P<object>\w+(?:\s+\w+)*)\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'take\s+(?P<object>\w+(?:\s+\w+)*)\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'carry\s+(?P<object>\w+(?:\s+\w+)*)\s+to\s+(?P<location>\w+(?:\s+\w+)*)'
            ]
        }

        # Entity extraction patterns
        self.entity_patterns = {
            'number': r'\d+\.?\d*',
            'distance': r'(\d+\.?\d*)\s*(meters|meter|m|cm|centimeters|steps)',
            'object': r'(red|blue|green|yellow|small|large|big|medium)\s+(\w+)',
            'location': r'(kitchen|living room|bedroom|office|bathroom|hallway|garage|garden|dining room)'
        }

        # Intent confidence weights
        self.intent_weights = {
            'move': 0.9,
            'grasp': 0.9,
            'navigate': 0.9,
            'inspect': 0.8,
            'stop': 0.95,
            'follow': 0.85,
            'bring': 0.85
        }

    def parse_command(self, text: str) -> Optional[VoiceCommand]:
        """Parse voice command from text"""
        if not text or not text.strip():
            return None

        original_text = text
        text = text.lower().strip()

        # Preprocess text
        processed_text = self._preprocess_text(text)

        # Try to match against each intent pattern
        best_match = None
        best_confidence = 0.0

        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, processed_text)
                if match:
                    entities = match.groupdict()

                    # Post-process entities
                    entities = self._post_process_entities(entities)

                    # Calculate confidence based on match quality
                    confidence = self._calculate_confidence(text, pattern, entities)

                    # Apply intent-specific weight
                    confidence *= self.intent_weights.get(intent, 0.8)

                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_match = VoiceCommand(
                            intent=intent,
                            entities=entities,
                            confidence=confidence,
                            original_text=original_text,
                            processed_text=processed_text
                        )

        # If no high-confidence match, try entity extraction
        if best_confidence < 0.5:
            fallback_result = self._fallback_parsing(processed_text, original_text)
            if fallback_result and fallback_result.confidence > best_confidence:
                best_match = fallback_result

        return best_match

    def _preprocess_text(self, text: str) -> str:
        """Preprocess text for better matching"""
        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text.strip())

        # Normalize common variations
        text = re.sub(r'\bstaight\b', 'forward', text)
        text = re.sub(r'\bgoto\b', 'go to', text)
        text = re.sub(r'\bwalk\s+to\b', 'go to', text)

        return text

    def _post_process_entities(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Post-process extracted entities"""
        processed_entities = {}

        for key, value in entities.items():
            if value is None:
                continue

            # Convert distance to numeric value
            if key == 'distance' and value:
                try:
                    processed_entities[key] = float(value)
                except ValueError:
                    processed_entities[key] = value
            # Process object names
            elif key == 'object' and value:
                processed_entities[key] = value.strip()
            # Process location names
            elif key == 'location' and value:
                processed_entities[key] = value.strip().replace(' ', '_')
            # Process direction names
            elif key == 'direction' and value:
                processed_entities[key] = value.strip()
            # Process target names
            elif key == 'target' and value:
                processed_entities[key] = value.strip()
            else:
                processed_entities[key] = value

        return processed_entities

    def _calculate_confidence(self, text: str, pattern: str, entities: Dict[str, Any]) -> float:
        """Calculate confidence score for pattern match"""
        # Base confidence on pattern match
        base_confidence = 0.7

        # Boost confidence if we have meaningful entities
        if entities:
            entity_count = len([v for v in entities.values() if v])
            base_confidence += entity_count * 0.1

        # Boost confidence for longer, more specific matches
        match_length = len(text)
        base_confidence = min(0.95, base_confidence + (match_length / 200))

        # Penalty for ambiguous matches
        if any(word in text for word in ['maybe', 'perhaps', 'possibly']):
            base_confidence *= 0.8

        return base_confidence

    def _fallback_parsing(self, text: str, original_text: str) -> Optional[VoiceCommand]:
        """Fallback parsing using entity extraction"""
        # Try to extract entities even without perfect pattern match
        entities = {}

        # Extract numbers
        number_matches = re.findall(self.entity_patterns['number'], text)
        if number_matches:
            entities['number'] = [float(n) for n in number_matches]

        # Extract objects with adjectives
        object_matches = re.findall(self.entity_patterns['object'], text)
        if object_matches:
            for adj, obj in object_matches:
                entities['object'] = f"{adj}_{obj}"

        # Extract known locations
        for location in ['kitchen', 'living room', 'bedroom', 'office', 'bathroom', 'hallway']:
            if location in text:
                entities['location'] = location.replace(' ', '_')
                break

        # Determine intent based on keywords
        intent = 'unknown'
        if any(word in text for word in ['move', 'go', 'walk', 'forward', 'backward', 'left', 'right']):
            intent = 'move'
        elif any(word in text for word in ['grasp', 'pick', 'grab', 'take', 'hold']):
            intent = 'grasp'
        elif any(word in text for word in ['go to', 'navigate', 'move to', 'head to']):
            intent = 'navigate'
        elif any(word in text for word in ['stop', 'halt', 'pause']):
            intent = 'stop'
        elif any(word in text for word in ['look', 'inspect', 'check', 'find']):
            intent = 'inspect'

        if intent != 'unknown':
            confidence = 0.4  # Lower confidence for fallback

            # Boost if we found entities
            if entities:
                confidence += len(entities) * 0.1

            return VoiceCommand(
                intent=intent,
                entities=entities,
                confidence=min(0.6, confidence),  # Cap fallback confidence
                original_text=original_text,
                processed_text=text
            )

        return None

    def batch_parse(self, texts: List[str]) -> List[Optional[VoiceCommand]]:
        """Parse multiple commands at once"""
        return [self.parse_command(text) for text in texts]

class AdvancedCommandParser(VoiceCommandParser):
    """Extended command parser with more sophisticated NLU capabilities"""
    def __init__(self):
        super().__init__()

        # Add more complex patterns for compound commands
        self.compound_patterns = [
            r'first\s+(?P<action1>.*?),?\s+then\s+(?P<action2>.*)',
            r'after\s+(?P<action1>.*?),?\s+(?P<action2>.*)',
            r'while\s+(?P<action1>.*?),?\s+(?P<action2>.*)'
        ]

        # Add temporal patterns
        self.temporal_patterns = {
            'repeat': r'(repeat|again|redo|do it again)',
            'delay': r'(wait|pause|delay)\s*(?P<time>\d+\.?\d*)?\s*(seconds|second|secs|sec|minutes|minute|min)?',
            'sequence': r'(first|second|third|next|then|afterwards|finally)'
        }

    def parse_compound_command(self, text: str) -> Optional[VoiceCommand]:
        """Parse compound commands with multiple actions"""
        for pattern in self.compound_patterns:
            match = re.search(pattern, text.lower())
            if match:
                groups = match.groupdict()

                # Parse individual actions
                action1 = self.parse_command(groups.get('action1', ''))
                action2 = self.parse_command(groups.get('action2', ''))

                if action1 and action2:
                    return VoiceCommand(
                        intent='compound',
                        entities={
                            'actions': [action1, action2],
                            'sequence': 'sequential'
                        },
                        confidence=0.85,
                        original_text=text,
                        processed_text=text.lower()
                    )

        return super().parse_command(text)

    def extract_temporal_info(self, text: str) -> Dict[str, Any]:
        """Extract temporal information from command"""
        temporal_info = {}

        for key, pattern in self.temporal_patterns.items():
            match = re.search(pattern, text.lower())
            if match:
                if key == 'delay' and 'time' in match.groupdict():
                    temporal_info[key] = float(match.group('time') or 1.0)
                else:
                    temporal_info[key] = match.group(0)

        return temporal_info
```

## Step 4: Create Integration Node

Create a file `voice_processing_node.py`:

```python
#!/usr/bin/env python3
"""
Main Voice Processing Node
Integrates audio input, preprocessing, recognition, and command parsing
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import threading
import queue
import numpy as np
import os
from audio_preprocessor import AudioPreprocessor, StreamingAudioProcessor
from speech_recognizer import MultiModalSpeechRecognizer
from command_parser import VoiceCommandParser

class VoiceProcessingNode(Node):
    def __init__(self):
        super().__init__('voice_processing_node')

        # Publishers and subscribers
        self.voice_command_pub = self.create_publisher(String, 'voice_commands', 10)
        self.audio_sub = self.create_subscription(
            AudioData, 'audio_stream', self.audio_callback, 10
        )

        # Initialize components
        self.preprocessor = AudioPreprocessor()
        self.streaming_processor = StreamingAudioProcessor(self.preprocessor)
        self.command_parser = VoiceCommandParser()

        # Get API key from parameter or environment
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('vosk_model_path', '')

        whisper_api_key = self.get_parameter('openai_api_key').value
        if not whisper_api_key:
            whisper_api_key = os.getenv('OPENAI_API_KEY', '')

        vosk_model_path = self.get_parameter('vosk_model_path').value
        if not vosk_model_path:
            vosk_model_path = os.getenv('VOSK_MODEL_PATH', '')

        if not whisper_api_key:
            self.get_logger().warn("OpenAI API key not provided. Cloud recognition will be disabled.")

        if not vosk_model_path:
            self.get_logger().warn("Vosk model path not provided. Local recognition will be disabled.")

        # Initialize speech recognizer if API key is available
        if whisper_api_key and vosk_model_path:
            try:
                self.speech_recognizer = MultiModalSpeechRecognizer(whisper_api_key, vosk_model_path)
                self.get_logger().info("Speech recognizer initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize speech recognizer: {e}")
                self.speech_recognizer = None
        else:
            self.speech_recognizer = None

        # Audio processing queue
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # Processing thread
        self.is_processing = True
        self.processing_thread = threading.Thread(target=self.process_audio_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Timer for result processing
        self.timer = self.create_timer(0.1, self.process_results)

        self.get_logger().info("Voice processing node initialized")

    def audio_callback(self, msg: AudioData):
        """Callback for incoming audio data"""
        try:
            # Convert ROS AudioData to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
            self.audio_queue.put(audio_data)
        except Exception as e:
            self.get_logger().error(f"Error in audio callback: {e}")

    def process_audio_queue(self):
        """Process audio data in separate thread"""
        while self.is_processing:
            try:
                audio_chunk = self.audio_queue.get(timeout=1.0)

                # Process chunk through streaming processor
                speech_segment = self.streaming_processor.process_chunk(audio_chunk)

                if speech_segment is not None and self.speech_recognizer:
                    # Convert to appropriate format for speech recognition
                    int16_audio = (speech_segment * 32767).astype(np.int16)

                    # Transcribe speech
                    result = self.speech_recognizer.transcribe_audio(int16_audio.tobytes())

                    if result['success'] and result['text'].strip():
                        # Parse the command
                        command = self.command_parser.parse_command(result['text'])
                        if command and command.confidence > 0.7:
                            self.result_queue.put(command)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in audio processing thread: {e}")

    def process_results(self):
        """Process recognition results"""
        while not self.result_queue.empty():
            try:
                command = self.result_queue.get_nowait()
                if command:
                    self.publish_voice_command(command)
            except queue.Empty:
                break

    def publish_voice_command(self, command):
        """Publish parsed voice command"""
        cmd_msg = String()
        cmd_data = {
            'intent': command.intent,
            'entities': command.entities,
            'confidence': command.confidence,
            'original_text': command.original_text
        }
        cmd_msg.data = str(cmd_data)
        self.voice_command_pub.publish(cmd_msg)
        self.get_logger().info(f"Published voice command: {command.intent} - {command.original_text}")

    def destroy_node(self):
        """Clean up before node destruction"""
        self.is_processing = False
        if hasattr(self, 'processing_thread'):
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = VoiceProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: Create Test and Verification Scripts

Create a file `test_setup.py`:

```python
#!/usr/bin/env python3
"""
Test script to verify voice processing system setup
"""
import pyaudio
import numpy as np
import wave
import time
import sys
import os

def test_audio_devices():
    """Test audio device availability"""
    print("Testing audio devices...")

    try:
        import pyaudio
        audio = pyaudio.PyAudio()

        print("Available audio input devices:")
        for i in range(audio.get_device_count()):
            device_info = audio.get_device_info_by_index(i)
            if device_info['maxInputChannels'] > 0:
                print(f"  Device {i}: {device_info['name']}")
                print(f"    Sample rate: {device_info['defaultSampleRate']}")
                print(f"    Max input channels: {device_info['maxInputChannels']}")

        audio.terminate()
        return True
    except Exception as e:
        print(f"Error testing audio devices: {e}")
        return False

def test_audio_recording():
    """Test audio recording functionality"""
    print("\nTesting audio recording...")

    try:
        chunk = 1024
        format = pyaudio.paInt16
        channels = 1
        sample_rate = 44100
        record_seconds = 3

        audio = pyaudio.PyAudio()

        # Open stream
        stream = audio.open(
            format=format,
            channels=channels,
            rate=sample_rate,
            input=True,
            frames_per_buffer=chunk
        )

        print(f"Recording for {record_seconds} seconds...")
        frames = []

        for i in range(0, int(sample_rate / chunk * record_seconds)):
            data = stream.read(chunk)
            frames.append(data)

        print("Recording finished!")

        # Save to WAV file
        filename = f"test_recording_{int(time.time())}.wav"
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(audio.get_sample_size(format))
            wf.setframerate(sample_rate)
            wf.writeframes(b''.join(frames))

        print(f"Audio saved to {filename}")

        # Verify file was created
        if os.path.exists(filename):
            file_size = os.path.getsize(filename)
            print(f"File created successfully (size: {file_size} bytes)")
            # Clean up
            os.remove(filename)
        else:
            print("Error: File was not created")

        # Close stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        return True
    except Exception as e:
        print(f"Error during audio recording test: {e}")
        return False

def test_speech_recognition():
    """Test speech recognition setup"""
    print("\nTesting speech recognition setup...")

    try:
        # Test if required libraries are available
        import openai
        import vosk

        print("  ✓ OpenAI library available")
        print("  ✓ Vosk library available")

        # Check if API key is set
        api_key = os.getenv('OPENAI_API_KEY')
        if api_key:
            print("  ✓ OpenAI API key is set")
        else:
            print("  ⚠ OpenAI API key not set (cloud recognition will be disabled)")

        # Check if Vosk model path is set
        model_path = os.getenv('VOSK_MODEL_PATH')
        if model_path and os.path.exists(model_path):
            print("  ✓ Vosk model path is set and accessible")
        else:
            print("  ⚠ Vosk model path not set or not accessible (local recognition will be disabled)")

        return True
    except ImportError as e:
        print(f"  ✗ Missing required library: {e}")
        return False
    except Exception as e:
        print(f"  ✗ Error testing speech recognition: {e}")
        return False

def test_audio_preprocessing():
    """Test audio preprocessing functionality"""
    print("\nTesting audio preprocessing...")

    try:
        from audio_preprocessor import AudioPreprocessor, StreamingAudioProcessor

        # Create preprocessor instance
        preprocessor = AudioPreprocessor()
        streaming_processor = StreamingAudioProcessor(preprocessor)

        # Create test audio
        sample_rate = 44100
        duration = 0.5  # seconds
        t = np.linspace(0, duration, int(sample_rate * duration))
        test_audio = 0.1 * np.sin(2 * np.pi * 440 * t)  # 440Hz tone

        # Test preprocessing
        processed_audio = preprocessor.preprocess_audio(test_audio)
        print(f"  ✓ Preprocessing completed (input: {len(test_audio)}, output: {len(processed_audio)})")

        # Test feature extraction
        features = preprocessor.extract_features(test_audio)
        print(f"  ✓ Feature extraction completed (energy: {features.energy:.4f}, is_speech: {features.is_speech})")

        # Test streaming processing
        result = streaming_processor.process_chunk(test_audio)
        print(f"  ✓ Streaming processing completed")

        return True
    except Exception as e:
        print(f"  ✗ Error testing audio preprocessing: {e}")
        return False

def test_command_parsing():
    """Test command parsing functionality"""
    print("\nTesting command parsing...")

    try:
        from command_parser import VoiceCommandParser

        parser = VoiceCommandParser()

        # Test commands
        test_commands = [
            "move forward 2 meters",
            "go to kitchen",
            "pick up the red ball",
            "stop immediately",
            "grasp the object"
        ]

        for cmd in test_commands:
            result = parser.parse_command(cmd)
            if result:
                print(f"  ✓ '{cmd}' -> intent: {result.intent}, confidence: {result.confidence:.2f}")
            else:
                print(f"  ⚠ '{cmd}' -> no match")

        return True
    except Exception as e:
        print(f"  ✗ Error testing command parsing: {e}")
        return False

def main():
    """Run all tests"""
    print("Voice Processing System Setup Test")
    print("=" * 40)

    tests = [
        ("Audio Devices", test_audio_devices),
        ("Audio Recording", test_audio_recording),
        ("Speech Recognition", test_speech_recognition),
        ("Audio Preprocessing", test_audio_preprocessing),
        ("Command Parsing", test_command_parsing)
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 40)
    print("Test Results Summary:")
    print("=" * 40)

    all_passed = True
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")
        if not result:
            all_passed = False

    print("=" * 40)
    if all_passed:
        print("✓ All tests passed! Your voice processing system is ready.")
    else:
        print("⚠ Some tests failed. Please check the error messages above.")

    return all_passed

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
```

## Step 6: Create Launch File

Create a file `voice_processing.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get environment variables
    openai_api_key = os.getenv('OPENAI_API_KEY', '')
    vosk_model_path = os.getenv('VOSK_MODEL_PATH', '')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'openai_api_key',
            default_value=openai_api_key,
            description='OpenAI API key for Whisper'
        ),
        DeclareLaunchArgument(
            'vosk_model_path',
            default_value=vosk_model_path,
            description='Path to Vosk model directory'
        ),
        DeclareLaunchArgument(
            'sample_rate',
            default_value='44100',
            description='Audio sample rate'
        ),
        DeclareLaunchArgument(
            'channels',
            default_value='1',
            description='Number of audio channels'
        ),
        DeclareLaunchArgument(
            'chunk_size',
            default_value='1024',
            description='Audio chunk size'
        ),

        # Audio input node
        Node(
            package='your_robot_package',  # Replace with your package name
            executable='audio_input_node',
            name='audio_input_node',
            parameters=[
                {
                    'sample_rate': LaunchConfiguration('sample_rate'),
                    'channels': LaunchConfiguration('channels'),
                    'chunk_size': LaunchConfiguration('chunk_size')
                }
            ],
            output='screen'
        ),

        # Voice processing node
        Node(
            package='your_robot_package',  # Replace with your package name
            executable='voice_processing_node',
            name='voice_processing_node',
            parameters=[
                {
                    'openai_api_key': LaunchConfiguration('openai_api_key'),
                    'vosk_model_path': LaunchConfiguration('vosk_model_path')
                }
            ],
            output='screen'
        )
    ])
```

## Step 7: Run the System

### 7.1 Verify Setup

First, run the test script to verify your setup:

```bash
# Make sure your virtual environment is activated
source voice_processing_env/bin/activate

# Run the test script
python test_setup.py
```

### 7.2 Set Environment Variables

```bash
# Set your OpenAI API key
export OPENAI_API_KEY="your_openai_api_key_here"

# Set Vosk model path
export VOSK_MODEL_PATH="$(pwd)/models/vosk-model-en-us-0.22"
```

### 7.3 Run the Voice Processing System

```bash
# Run the audio input node
python audio_input_node.py

# In another terminal, run the voice processing node
python voice_processing_node.py
```

### 7.4 Test with ROS 2 Tools

```bash
# Listen to the audio stream
ros2 topic echo /audio_stream

# Listen to the voice commands
ros2 topic echo /voice_commands
```

## Expected Outcomes

After completing this tutorial, you should have:

1. A working audio input system that captures microphone input
2. Audio preprocessing that reduces noise and detects voice activity
3. Speech recognition that converts speech to text using both cloud and local methods
4. Command parsing that extracts intents and entities from transcribed text
5. A complete ROS 2 system that processes voice commands in real-time

## Troubleshooting

### Common Issues:

1. **No audio devices detected**: Check microphone connections and permissions
2. **PyAudio installation fails**: Install system dependencies first
3. **OpenAI API key not working**: Verify the API key format and billing
4. **Vosk model not found**: Ensure the model path is correct and accessible
5. **High CPU usage**: Reduce audio processing frequency or optimize algorithms

### Verification Steps:

1. Check that audio devices are properly detected
2. Verify that the test recording script works
3. Confirm that speech recognition libraries are available
4. Test that command parsing works with sample commands
5. Validate that ROS 2 nodes are communicating properly