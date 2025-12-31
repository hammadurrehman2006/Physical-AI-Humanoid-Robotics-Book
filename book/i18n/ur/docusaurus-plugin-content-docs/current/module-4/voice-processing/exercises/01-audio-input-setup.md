---
sidebar_position: 1
title: "ورکشاپ 4.1: آڈیو ان پٹ سیٹ اپ"
---

# ورکشاپ 4.1: آڈیو ان پٹ سیٹ اپ

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے آڈیو ان پٹ کو سیٹ اپ کرنا اور ترتیب دینا۔

## ضروریات
- Python 3.10+
- PyAudio
- NumPy
- ROS 2 ہمبل ہاکسبل
- مائیکروفون ڈیوائس

## ورکشاپ کے اقدامات

### اقدام 1: آڈیو سیٹ اپ کو سمجھنا
ایک نیا فائل `audio_setup.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import pyaudio
import numpy as np
import wave
import time
from typing import Dict, List, Tuple
import threading
import queue

class AudioSetup:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.is_recording = False
        self.recording_thread = None

        # ڈیفالٹ آڈیو پیرامیٹرز
        self.rate = 16000  # 16kHz sampling rate
        self.channels = 1  # مونو
        self.chunk_size = 1024  # 1024 samples per chunk
        self.format = pyaudio.paInt16  # 16-bit

        print("Audio setup initialized")
        print(f"Default rate: {self.rate}Hz")
        print(f"Default channels: {self.channels}")
        print(f"Default chunk size: {self.chunk_size}")

    def list_audio_devices(self):
        """تمام دستیاب آڈیو ڈیوائسز کو فہرست کریں"""
        print("\nAvailable Audio Devices:")
        print("-" * 60)

        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            print(f"Device {i}: {info['name']}")
            print(f"  - Max input channels: {info['maxInputChannels']}")
            print(f"  - Max output channels: {info['maxOutputChannels']}")
            print(f"  - Default sample rate: {info['defaultSampleRate']}")
            print(f"  - Host API: {info['hostApi']}")

            # اگر یہ ان پٹ ڈیوائس ہے
            if info['maxInputChannels'] > 0:
                print("  - [INPUT DEVICE]")
            if info['maxOutputChannels'] > 0:
                print("  - [OUTPUT DEVICE]")
            print()

    def find_microphone_device(self) -> int:
        """مائیکروفون ڈیوائس تلاش کریں"""
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:  # ان پٹ ڈیوائس ہے
                print(f"Found microphone device {i}: {info['name']}")
                return i

        print("No microphone device found!")
        return -1

    def configure_audio_parameters(self, rate=16000, channels=1, chunk_size=1024):
        """آڈیو پیرامیٹرز کو کنفیگر کریں"""
        self.rate = rate
        self.channels = channels
        self.chunk_size = chunk_size

        print(f"Audio parameters configured:")
        print(f"  Rate: {self.rate}Hz")
        print(f"  Channels: {self.channels}")
        print(f"  Chunk size: {self.chunk_size}")

    def test_microphone(self, duration=3):
        """مائیکروفون کو ٹیسٹ کریں"""
        print(f"Testing microphone for {duration} seconds...")

        device_index = self.find_microphone_device()
        if device_index == -1:
            print("Cannot test microphone: no device found")
            return False

        try:
            # اسٹریم کھولیں
            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size
            )

            frames = []
            print("Recording... Speak now!")

            for _ in range(0, int(self.rate / self.chunk_size * duration)):
                data = stream.read(self.chunk_size)
                frames.append(data)

            print("Recording completed!")

            # اسٹریم بند کریں
            stream.stop_stream()
            stream.close()

            # ایوریج ایمپلی ٹیوڈ چیک کریں (سادہ نوائز ڈیٹیکشن)
            audio_data = b''.join(frames)
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            avg_amplitude = np.mean(np.abs(audio_array))

            print(f"Average amplitude: {avg_amplitude:.2f}")
            if avg_amplitude > 500:  # نوائز کا تخمینہ
                print("✓ Microphone test successful - good signal detected")
                return True
            else:
                print("⚠ Microphone test completed but low signal detected")
                return False

        except Exception as e:
            print(f"Microphone test failed: {e}")
            return False

    def start_continuous_recording(self, callback=None):
        """مسلسل ریکارڈنگ شروع کریں"""
        if self.is_recording:
            print("Recording is already active")
            return

        device_index = self.find_microphone_device()
        if device_index == -1:
            print("Cannot start recording: no microphone device found")
            return

        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size
            )

            self.is_recording = True
            self.recording_thread = threading.Thread(target=self._recording_loop, args=(callback,))
            self.recording_thread.start()

            print("Continuous recording started")

        except Exception as e:
            print(f"Failed to start continuous recording: {e}")

    def _recording_loop(self, callback=None):
        """ریکارڈنگ کا لُوپ"""
        while self.is_recording:
            try:
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                if callback:
                    # کال بیک کو کال کریں
                    callback(data)

            except Exception as e:
                print(f"Recording loop error: {e}")
                break

    def stop_recording(self):
        """ریکارڈنگ بند کریں"""
        if self.is_recording:
            self.is_recording = False

            if self.recording_thread:
                self.recording_thread.join()

            if self.stream:
                self.stream.stop_stream()
                self.stream.close()

            print("Recording stopped")

    def save_audio_file(self, audio_data: bytes, filename: str):
        """آڈیو ڈیٹا کو فائل میں محفوظ کریں"""
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(audio_data)

        print(f"Audio saved to {filename}")

    def cleanup(self):
        """ریسورسز کو صاف کریں"""
        self.stop_recording()
        self.audio.terminate()
        print("Audio resources cleaned up")

def main():
    setup = AudioSetup()

    try:
        # ڈیوائسز کو فہرست کریں
        setup.list_audio_devices()

        # مائیکروفون ٹیسٹ کریں
        success = setup.test_microphone(duration=3)
        if success:
            print("\n✓ Audio setup test successful!")
        else:
            print("\n✗ Audio setup test failed")

        # کنفیگریشن کو ایڈجسٹ کریں
        setup.configure_audio_parameters(rate=16000, channels=1, chunk_size=1024)

    finally:
        setup.cleanup()

if __name__ == "__main__":
    main()
```

### اقدام 2: ROS 2 انٹیگریشن کے لیے آڈیو نوڈ
ایک فائل `audio_input_node.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import pyaudio
import numpy as np
import threading
import time
from typing import Dict, Any

class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')

        # PyAudio instance
        self.audio = pyaudio.PyAudio()

        # ROS 2 publishers
        self.audio_publisher = self.create_publisher(
            AudioData, 'audio_input', 10
        )
        self.status_publisher = self.create_publisher(
            String, 'audio_status', 10
        )

        # آڈیو پیرامیٹرز
        self.rate = 16000
        self.channels = 1
        self.chunk_size = 1024
        self.format = pyaudio.paInt16

        # ریکارڈنگ کے لیے متغیرات
        self.is_recording = False
        self.recording_thread = None

        # ڈیوائس انڈیکس
        self.device_index = self.find_microphone_device()

        # شروع کریں
        self.start_recording()

        self.get_logger().info('Audio Input Node initialized')

    def find_microphone_device(self) -> int:
        """مائیکروفون ڈیوائس تلاش کریں"""
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:  # ان پٹ ڈیوائس ہے
                self.get_logger().info(f'Found microphone device {i}: {info["name"]}')
                return i

        self.get_logger().error('No microphone device found!')
        return -1

    def start_recording(self):
        """ریکارڈنگ شروع کریں"""
        if self.device_index == -1:
            self.get_logger().error('Cannot start recording: no microphone device found')
            return

        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.chunk_size
            )

            self.is_recording = True
            self.recording_thread = threading.Thread(target=self.recording_loop)
            self.recording_thread.start()

            self.get_logger().info('Audio recording started')

        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {e}')

    def recording_loop(self):
        """ریکارڈنگ کا لُوپ"""
        while self.is_recording:
            try:
                # آڈیو ڈیٹا پڑھیں
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # ROS 2 میسج تیار کریں
                audio_msg = AudioData()
                audio_msg.data = data

                # ہیڈر سیٹ کریں
                audio_msg.header.stamp = self.get_clock().now().to_msg()
                audio_msg.header.frame_id = 'microphone'

                # فریکوئنسی اور چینلز کے بارے میں معلومات
                audio_msg.encoding = 'L16'  # 16-bit linear PCM
                audio_msg.sample_rate = self.rate
                audio_msg.channels = self.channels

                # پبلش کریں
                self.audio_publisher.publish(audio_msg)

            except Exception as e:
                self.get_logger().error(f'Recording loop error: {e}')
                break

    def destroy_node(self):
        """نوڈ کو ڈسٹرائے کریں"""
        self.is_recording = False

        if hasattr(self, 'recording_thread') and self.recording_thread:
            self.recording_thread.join()

        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()

        self.audio.terminate()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 3: آڈیو پروسیسنگ اور ٹیسٹنگ
ایک فائل `audio_processing_test.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import pyaudio
import numpy as np
import time
from scipy import signal
import matplotlib.pyplot as plt
from typing import Dict, List

class AudioProcessingTester:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.rate = 16000
        self.channels = 1
        self.chunk_size = 1024
        self.format = pyaudio.paInt16

    def record_audio(self, duration=5) -> bytes:
        """آڈیو ریکارڈ کریں"""
        print(f"Recording audio for {duration} seconds...")

        # مائیکروفون ڈیوائس تلاش کریں
        device_index = self.find_microphone_device()
        if device_index == -1:
            print("No microphone found")
            return b""

        try:
            stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size
            )

            frames = []
            print("Recording... Speak now!")

            for _ in range(0, int(self.rate / self.chunk_size * duration)):
                data = stream.read(self.chunk_size)
                frames.append(data)

            print("Recording completed!")

            stream.stop_stream()
            stream.close()

            return b''.join(frames)

        except Exception as e:
            print(f"Recording failed: {e}")
            return b""

    def find_microphone_device(self) -> int:
        """مائیکروفون ڈیوائس تلاش کریں"""
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"Found microphone device {i}: {info['name']}")
                return i
        return -1

    def analyze_audio(self, audio_data: bytes) -> Dict:
        """آڈیو کا تجزیہ کریں"""
        # NumPy ارے میں تبدیل کریں
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # بنیادی میٹرکس
        analysis = {
            'duration': len(audio_array) / self.rate,
            'rms': np.sqrt(np.mean(audio_array**2)),
            'max_amplitude': np.max(np.abs(audio_array)),
            'zero_crossing_rate': np.sum(np.diff(np.sign(audio_array)) != 0) / len(audio_array),
            'energy': np.sum(audio_array**2),
            'sample_rate': self.rate
        }

        print(f"Audio Analysis:")
        print(f"  Duration: {analysis['duration']:.2f}s")
        print(f"  RMS: {analysis['rms']:.4f}")
        print(f"  Max Amplitude: {analysis['max_amplitude']:.4f}")
        print(f"  Zero Crossing Rate: {analysis['zero_crossing_rate']:.4f}")
        print(f"  Energy: {analysis['energy']:.4f}")

        return analysis

    def detect_speech(self, audio_data: bytes, threshold=0.01) -> List[Dict]:
        """اسپیچ ڈیٹیکٹ کریں"""
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # ہر چنک کے لیے ایمپلی ٹیوڈ کا حساب
        chunk_size = int(0.1 * self.rate)  # 100ms chunks
        chunks = []

        for i in range(0, len(audio_array), chunk_size):
            chunk = audio_array[i:i+chunk_size]
            rms = np.sqrt(np.mean(chunk**2))
            chunks.append({
                'start_time': i / self.rate,
                'rms': rms,
                'is_speech': rms > threshold
            })

        # اسپیچ سیکشنز کو ڈیٹیکٹ کریں
        speech_segments = []
        current_segment = None

        for chunk in chunks:
            if chunk['is_speech']:
                if current_segment is None:
                    # نیا سیگمینٹ شروع کریں
                    current_segment = {
                        'start': chunk['start_time'],
                        'end': chunk['start_time'],
                        'chunks': [chunk]
                    }
                else:
                    # موجودہ سیگمینٹ کو اپ ڈیٹ کریں
                    current_segment['end'] = chunk['start_time']
                    current_segment['chunks'].append(chunk)
            else:
                if current_segment is not None:
                    # سیگمینٹ ختم کریں
                    speech_segments.append(current_segment)
                    current_segment = None

        if current_segment is not None:
            speech_segments.append(current_segment)

        print(f"Detected {len(speech_segments)} speech segments:")
        for i, segment in enumerate(speech_segments):
            print(f"  Segment {i+1}: {segment['start']:.2f}s - {segment['end']:.2f}s "
                  f"({segment['end']-segment['start']:.2f}s)")

        return speech_segments

    def apply_noise_reduction(self, audio_data: bytes) -> bytes:
        """نوائز ریموال لاگو کریں"""
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # سادہ نوائز ریموال (spectral gating)
        # پہلے 0.5s کو نوائز کے طور پر سمجھیں
        noise_sample = audio_array[:int(0.5 * self.rate)]
        noise_mean = np.mean(np.abs(noise_sample))
        noise_std = np.std(np.abs(noise_sample))

        # گیٹنگ تھریشولڈ
        threshold = noise_mean + 2 * noise_std  # 2 سگما

        # ہر سیمپل کو فلٹر کریں
        filtered_audio = np.where(np.abs(audio_array) > threshold, audio_array, 0)

        # NumPy سے bytes میں تبدیل کریں
        filtered_bytes = (filtered_audio * 32767).astype(np.int16).tobytes()

        return filtered_bytes

    def save_wav_file(self, audio_data: bytes, filename: str):
        """WAV فائل میں محفوظ کریں"""
        import wave

        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(audio_data)

        print(f"Audio saved to {filename}")

    def plot_audio_waveform(self, audio_data: bytes, title="Audio Waveform"):
        """آڈیو ویو فارم پلاٹ کریں"""
        try:
            audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            time_axis = np.linspace(0, len(audio_array) / self.rate, len(audio_array))

            plt.figure(figsize=(12, 4))
            plt.plot(time_axis, audio_array)
            plt.title(title)
            plt.xlabel('Time (seconds)')
            plt.ylabel('Amplitude')
            plt.grid(True)
            plt.tight_layout()
            plt.savefig('/tmp/audio_waveform.png')
            print("Waveform plot saved to /tmp/audio_waveform.png")

        except ImportError:
            print("Matplotlib not available, skipping plot generation")

    def run_comprehensive_test(self):
        """کمپری ہینسیو ٹیسٹ چلائیں"""
        print("Running comprehensive audio processing test...")

        # آڈیو ریکارڈ کریں
        audio_data = self.record_audio(duration=5)

        if len(audio_data) == 0:
            print("No audio recorded, test failed")
            return

        # تجزیہ کریں
        analysis = self.analyze_audio(audio_data)

        # اسپیچ ڈیٹیکٹ کریں
        speech_segments = self.detect_speech(audio_data)

        # نوائز ریموال
        clean_audio = self.apply_noise_reduction(audio_data)

        # ویو فارم پلاٹ کریں
        self.plot_audio_waveform(audio_data, "Original Audio")
        self.plot_audio_waveform(clean_audio, "Noise Reduced Audio")

        # صاف آڈیو کو محفوظ کریں
        self.save_wav_file(clean_audio, "/tmp/clean_audio.wav")

        print("\nComprehensive test completed!")
        print(f"Original duration: {analysis['duration']:.2f}s")
        print(f"Speech segments detected: {len(speech_segments)}")
        print("Clean audio saved for further processing")

    def cleanup(self):
        """ریسورسز کو صاف کریں"""
        self.audio.terminate()

def main():
    tester = AudioProcessingTester()

    try:
        tester.run_comprehensive_test()
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- آڈیو ڈیوائسز کو سیٹ اپ کیا
- مائیکروفون ڈیوائسز کو ڈیٹیکٹ کیا
- ROS 2 میں آڈیو ان پٹ نوڈ تیار کیا
- آڈیو پروسیسنگ اور ٹیسٹنگ کی
- نوائز ریموال اور اسپیچ ڈیٹیکشن کیا

یہ سیٹ اپ VLA سسٹم کے لیے وائس ان پٹ کی بنیاد ہے۔