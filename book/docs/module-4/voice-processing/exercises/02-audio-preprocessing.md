# Exercise 1.2: Audio Preprocessing and Noise Reduction

## Objective
Implement audio preprocessing techniques including noise reduction, voice activity detection, and signal enhancement for improved speech recognition accuracy.

## Prerequisites
- Completion of Exercise 1.1 (Audio Input Setup)
- Python 3.10+
- Required libraries: numpy, scipy, librosa, pyaudio
- Basic understanding of digital signal processing

## Exercise Steps

### Step 1: Set Up Audio Preprocessing Environment
Create a new file `audio_preprocessor.py`:

```python
#!/usr/bin/env python3
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