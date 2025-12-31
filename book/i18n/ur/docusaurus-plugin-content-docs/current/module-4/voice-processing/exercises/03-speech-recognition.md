# Exercise 1.3: Speech Recognition Integration

## Objective
Integrate speech recognition capabilities using both cloud-based (OpenAI Whisper) and local (Vosk) approaches to convert audio to text for the Vision-Language-Action system.

## Prerequisites
- Completion of Exercises 1.1 and 1.2
- Python 3.10+
- Required libraries: openai, vosk, pydub, librosa
- OpenAI API key (for cloud recognition)
- Vosk model files (for local recognition)

## Exercise Steps

### Step 1: Set Up Speech Recognition Environment
Create a new file `speech_recognizer.py`:

```python
#!/usr/bin/env python3
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

class SpeechRecognitionNode:
    """ROS 2 node for speech recognition"""
    def __init__(self):
        # This would be integrated with ROS 2 in a real implementation
        self.recognizer: Optional[MultiModalSpeechRecognizer] = None
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.is_running = False
        self.processing_thread = None

    def initialize(self, whisper_api_key: str, vosk_model_path: str):
        """Initialize the speech recognizer"""
        self.recognizer = MultiModalSpeechRecognizer(whisper_api_key, vosk_model_path)

    def start_processing(self):
        """Start the speech recognition processing thread"""
        if self.recognizer is None:
            raise ValueError("Recognizer not initialized")

        self.is_running = True
        self.processing_thread = threading.Thread(target=self._process_audio_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def _process_audio_queue(self):
        """Process audio data from queue"""
        while self.is_running:
            try:
                audio_data = self.audio_queue.get(timeout=1.0)
                if self.recognizer:
                    result = self.recognizer.transcribe_audio(audio_data)
                    self.result_queue.put(result)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Processing error: {e}")

    def stop_processing(self):
        """Stop the speech recognition processing"""
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join()

    def add_audio(self, audio_data: bytes):
        """Add audio data to processing queue"""
        self.audio_queue.put(audio_data)

    def get_result(self) -> Optional[Dict[str, Any]]:
        """Get the next recognition result"""
        try:
            return self.result_queue.get_nowait()
        except queue.Empty:
            return None
```

### Step 2: Create Speech Recognition Test Script
Create a test file `test_speech_recognition.py`:

```python
#!/usr/bin/env python3
import numpy as np
import wave
import io
from speech_recognizer import WhisperSpeechRecognizer, LocalSpeechRecognizer, MultiModalSpeechRecognizer

def create_test_audio():
    """Create a simple test audio signal"""
    # Generate a simple sine wave as test audio
    sample_rate = 44100
    duration = 2  # seconds
    frequency = 440  # Hz (A4 note)

    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    audio_data = 0.5 * np.sin(2 * np.pi * frequency * t)

    # Convert to 16-bit integers
    audio_data = (audio_data * 32767).astype(np.int16)

    # Create WAV format bytes
    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, 'wb') as wav_file:
        wav_file.setnchannels(1)  # Mono
        wav_file.setsampwidth(2)  # 16-bit
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_data.tobytes())

    return wav_buffer.getvalue()

def test_whisper_recognizer(api_key: str):
    """Test Whisper speech recognizer with dummy audio"""
    print("Testing Whisper Speech Recognizer...")

    recognizer = WhisperSpeechRecognizer(api_key)

    # Create test audio (this would normally be actual speech)
    test_audio = create_test_audio()

    # This will fail without actual speech, but tests the API connection
    result = recognizer.transcribe_audio(test_audio)
    print(f"Whisper result: '{result}'")

    return result

def test_local_recognizer(model_path: str):
    """Test local speech recognizer"""
    print("Testing Local Speech Recognizer...")

    recognizer = LocalSpeechRecognizer(model_path)

    # Create a short silence audio for testing
    sample_rate = 16000
    silence = np.zeros(int(sample_rate * 0.5), dtype=np.int16)  # 0.5 seconds of silence
    audio_bytes = silence.tobytes()

    result = recognizer.transcribe_audio_chunk(audio_bytes)
    print(f"Local result: '{result}'")

    return result

def test_multi_modal_recognizer(whisper_api_key: str, vosk_model_path: str):
    """Test multi-modal speech recognizer"""
    print("Testing Multi-Modal Speech Recognizer...")

    try:
        recognizer = MultiModalSpeechRecognizer(whisper_api_key, vosk_model_path)

        # Create test audio
        test_audio = create_test_audio()

        result = recognizer.transcribe_audio(test_audio)
        print(f"Multi-modal result: {result}")

        return result
    except Exception as e:
        print(f"Multi-modal recognizer test failed: {e}")
        return None

if __name__ == "__main__":
    import os

    # Get API key from environment variable
    whisper_api_key = os.getenv("OPENAI_API_KEY")
    vosk_model_path = os.getenv("VOSK_MODEL_PATH", "./model")  # Default path

    if not whisper_api_key:
        print("Warning: OPENAI_API_KEY not set. Cloud recognition will fail.")
        print("Set it using: export OPENAI_API_KEY='your-api-key-here'")
    else:
        print("Testing with Whisper API key")

    # Test different recognizers
    if whisper_api_key:
        test_whisper_recognizer(whisper_api_key)

    try:
        test_local_recognizer(vosk_model_path)
    except Exception as e:
        print(f"Local recognizer test failed (model may not be installed): {e}")
        print("To install Vosk model, download from: https://alphacephei.com/vosk/models")

    if whisper_api_key:
        test_multi_modal_recognizer(whisper_api_key, vosk_model_path)
```

### Step 3: Create Installation Script
Create an installation script `install_speech_models.sh`:

```bash
#!/bin/bash

echo "Installing speech recognition models..."

# Create models directory
mkdir -p models

# Download Vosk English model (about 50MB)
echo "Downloading Vosk English model..."
if [ ! -d "models/vosk-model-en-us-0.22" ]; then
    wget -O models/vosk-model-en-us-0.22.zip https://alphacephei.com/vosk/models/vosk-model-en-us-0.22.zip
    cd models && unzip vosk-model-en-us-0.22.zip && cd ..
    rm models/vosk-model-en-us-0.22.zip
    echo "Vosk model downloaded and extracted."
else
    echo "Vosk model already exists."
fi

echo "Installation complete!"
echo "Set environment variables:"
echo "export VOSK_MODEL_PATH=$(pwd)/models/vosk-model-en-us-0.22"
echo "export OPENAI_API_KEY=your_openai_api_key_here"
```

## Expected Outcomes
- Successfully implement both cloud and local speech recognition
- Create a multi-modal recognizer that can fallback between methods
- Test the recognizers with sample audio data
- Understand the trade-offs between cloud and local recognition

## Verification Steps
1. Verify Whisper API key is properly configured
2. Confirm Vosk model is downloaded and accessible
3. Test that both recognition methods work independently
4. Validate the multi-modal approach with fallback behavior

## Troubleshooting
- If Whisper fails, check API key and internet connectivity
- If Vosk fails, verify model path and format compatibility
- For audio format issues, ensure 16kHz sample rate for Vosk

## Next Exercise
Continue to Exercise 1.4: Voice Command Parsing and Natural Language Understanding