---
sidebar_position: 17
title: "ورکشاپ 4.3: ٹربل شوٹنگ"
---

# ورکشاپ 4.3: ٹربل شوٹنگ

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے وائس پروسیسنگ کے مسائل کو حل کرنا اور ٹربل شوٹ کرنا۔

## ضروریات
- Python 3.10+
- ROS 2 ہمبل ہاکسبل
- PyAudio
- OpenAI Whisper
- NumPy
- مائیکروفون

## عام مسائل اور حل

### 1. وائس ریکوگنیشن کے مسائل

#### مسئلہ: کم ٹرانسکرپشن کی ایکویسی

**عوامل**:
- نوائز والے ماحول
- مائیکروفون کی خراب کوالٹی
- نامناسب آڈیو فارمیٹ
- Whisper ماڈل کا سائز

**حل**:

```python
#!/usr/bin/env python3
import pyaudio
import numpy as np
import whisper
import torch
from scipy import signal

class VoiceTroubleshooter:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.noise_threshold = 0.01
        self.sampling_rate = 16000

    def improve_audio_quality(self, audio_data):
        """آڈیو کوالٹی بہتر کریں"""
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # نوائز ریموال (سادہ ایپروچ)
        # ہائی پاس فلٹر برائے DC آف سیٹ
        audio_array = signal.detrend(audio_array, type='constant')

        # نوائز کی سطح کا تعین
        noise_level = np.std(audio_array[:int(0.1 * self.sampling_rate)])  # پہلے 0.1 سیکنڈ کا نوائز

        # اگر نوائز زیادہ ہو تو وارننگ
        if noise_level > self.noise_threshold:
            print(f"Warning: High noise level detected: {noise_level:.4f}")

        return audio_array

    def adjust_sensitivity(self, audio_data, threshold_multiplier=1.5):
        """سینسیٹیویٹی ایڈجسٹ کریں"""
        audio_array = self.improve_audio_quality(audio_data)

        # اسپیچ اینرژی کا حساب
        energy = np.mean(np.abs(audio_array))

        # اگر اسپیچ کم ہو تو متناسب کریں
        if energy < 0.02:  # اسپیچ کم ہے
            audio_array = audio_array * (0.02 / energy)  # نارملائز کریں
            audio_array = np.clip(audio_array, -1.0, 1.0)  # کلپ کریں

        return audio_array

    def record_with_noise_gate(self, duration=5, silence_duration=1.0):
        """نوائز گیٹ کے ساتھ ریکارڈ کریں"""
        print("Recording with noise gate...")

        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sampling_rate,
            input=True,
            frames_per_buffer=1024
        )

        frames = []
        silent_frames = 0
        max_silent_frames = int(silence_duration * self.sampling_rate / 1024)

        recording_started = False
        total_frames = 0
        max_frames = int(duration * self.sampling_rate / 1024)

        while total_frames < max_frames:
            data = stream.read(1024)
            frames.append(data)
            total_frames += 1

            # اسپیچ ڈیٹیکشن
            audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            energy = np.mean(np.abs(audio_chunk))

            if energy > self.noise_threshold:
                recording_started = True
                silent_frames = 0
            elif recording_started:  # اگر ریکارڈنگ شروع ہو چکی ہو
                silent_frames += 1
                if silent_frames > max_silent_frames:
                    print(f"Silence detected, stopping recording at {total_frames * 1024 / self.sampling_rate:.2f}s")
                    break

        stream.stop_stream()
        stream.close()

        return b''.join(frames)

def troubleshoot_low_accuracy():
    """کم ایکویسی کے مسائل کو حل کریں"""
    troubleshooter = VoiceTroubleshooter()

    # نوائز گیٹ کے ساتھ ریکارڈ کریں
    audio_data = troubleshooter.record_with_noise_gate(duration=10)

    # آڈیو کو بہتر بنائیں
    improved_audio = troubleshooter.adjust_sensitivity(audio_data)

    # Whisper ماڈل لوڈ کریں
    model = whisper.load_model("base")
    if torch.cuda.is_available():
        model = model.cuda()

    # ٹرانسکرائیب کریں
    result = model.transcribe(improved_audio)

    print(f"Transcription result: {result['text']}")
    print("Accuracy should be improved with noise reduction")
```

### 2. مائیکروفون کے مسائل

#### مسئلہ: مائیکروفون کام نہ کرنا

**عوامل**:
- ڈرائیور کی خرابی
- اجازت کی کمی
- آڈیو ڈیوائس کا انتخاب
- کنیکشن کے مسائل

**حل**:

```python
def troubleshoot_microphone():
    """مائیکروفون کے مسائل کو حل کریں"""
    import pyaudio
    import subprocess

    audio = pyaudio.PyAudio()

    # دستیاب ڈیوائسز کو دیکھیں
    print("Available audio devices:")
    for i in range(audio.get_device_count()):
        info = audio.get_device_info_by_index(i)
        print(f"Device {i}: {info['name']}")
        print(f"  - Max input channels: {info['maxInputChannels']}")
        print(f"  - Max output channels: {info['maxOutputChannels']}")
        print(f"  - Default sample rate: {info['defaultSampleRate']}")
        print()

    # مائیکروفون ٹیسٹ
    try:
        # کسی مائیکروفون ڈیوائس کو منتخب کریں
        input_device_index = None
        for i in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                input_device_index = i
                print(f"Using device {i}: {info['name']}")
                break

        if input_device_index is not None:
            # چھوٹا ٹیسٹ ریکارڈ کریں
            stream = audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                input_device_index=input_device_index,
                frames_per_buffer=1024
            )

            print("Recording test audio...")
            frames = []
            for _ in range(10):  # 10 * 1024 / 16000 = ~0.64s
                data = stream.read(1024)
                frames.append(data)

            stream.stop_stream()
            stream.close()
            audio.terminate()

            print("Microphone test successful!")
            return True

    except Exception as e:
        print(f"Microphone test failed: {e}")
        print("Possible solutions:")
        print("1. Check audio device permissions")
        print("2. Verify microphone is not in use by another application")
        print("3. Update audio drivers")
        print("4. Check audio device in system settings")
        return False

def check_system_audio():
    """سسٹم آڈیو کو چیک کریں"""
    import platform
    system = platform.system()

    if system == "Linux":
        # ALSA/PulseAudio کو چیک کریں
        try:
            result = subprocess.run(['pactl', 'list', 'sources'], capture_output=True, text=True)
            print("PulseAudio sources:")
            print(result.stdout)
        except:
            print("PulseAudio not available or pactl command failed")

    elif system == "Windows":
        # Windows کے لیے
        try:
            import winsound
            print("Windows audio system available")
        except:
            print("Windows audio system check failed")

    elif system == "Darwin":  # macOS
        try:
            result = subprocess.run(['system_profiler', 'SPAudioDataType'], capture_output=True, text=True)
            print("Audio hardware on macOS:")
            print(result.stdout[:500])  # پہلے 500 کریکٹرز
        except:
            print("Could not check macOS audio hardware")
```

### 3. ROS 2 انٹیگریشن کے مسائل

#### مسئلہ: وائس نوڈ کام نہ کرنا

**عوامل**:
- ROS 2 نوڈس کا کنیکشن
- میسج فارمیٹس
- QoS پالیسیز
- تھریڈنگ کے مسائل

**حل**:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import pyaudio
import numpy as np
import whisper
import threading
import time

class RobustVoiceNode(Node):
    def __init__(self):
        super().__init__('robust_voice_node')

        # QoS پروفائلز
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # یا RELIABLE
            history=HistoryPolicy.KEEP_LAST
        )

        # سبسکرائبرز
        self.audio_subscriber = self.create_subscription(
            AudioData, 'audio_input', self.safe_audio_callback, qos_profile
        )
        self.command_subscriber = self.create_subscription(
            String, 'voice_commands', self.safe_command_callback, qos_profile
        )

        # پبلیشرز
        self.text_publisher = self.create_publisher(
            String, 'transcribed_text', qos_profile
        )
        self.status_publisher = self.create_publisher(
            String, 'voice_status', qos_profile
        )

        # تھریڈنگ
        self.audio_queue = []
        self.audio_lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_audio_safe, daemon=True)
        self.processing_thread.start()

        # وائس سسٹم
        self.whisper_model = None
        self.load_whisper_model()

        # ٹیمرز
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat_check)
        self.diagnostic_timer = self.create_timer(5.0, self.diagnostic_check)

        self.get_logger().info('Robust Voice Node initialized with error handling')

    def safe_audio_callback(self, msg):
        """محفوظ آڈیو کیل بیک"""
        try:
            with self.audio_lock:
                # آڈیو کو کیو میں ڈالیں
                self.audio_queue.append({
                    'data': msg.data,
                    'timestamp': msg.header.stamp,
                    'seq': msg.header.seq
                })

            self.get_logger().debug(f'Audio queued, queue size: {len(self.audio_queue)}')

        except Exception as e:
            self.get_logger().error(f'Safe audio callback error: {e}')
            self.publish_error_status(f'Audio callback error: {e}')

    def safe_command_callback(self, msg):
        """محفوظ کمانڈ کیل بیک"""
        try:
            # کمانڈ کو سیف طریقے سے ہینڈل کریں
            self.get_logger().info(f'Safe command received: {msg.data}')
            self.process_voice_command(msg.data)

        except Exception as e:
            self.get_logger().error(f'Safe command callback error: {e}')
            self.publish_error_status(f'Command callback error: {e}')

    def process_audio_safe(self):
        """آڈیو کو محفوظ تھریڈ میں پروسیس کریں"""
        while rclpy.ok():
            try:
                with self.audio_lock:
                    if self.audio_queue:
                        audio_item = self.audio_queue.pop(0)  # FIFO
                    else:
                        audio_item = None

                if audio_item:
                    # آڈیو کو پروسیس کریں
                    self.process_audio_item(audio_item)

                time.sleep(0.01)  # 10ms delay

            except Exception as e:
                self.get_logger().error(f'Safe audio processing error: {e}')
                time.sleep(0.1)  # بچاؤ کا وقت

    def process_audio_item(self, audio_item):
        """آڈیو آئٹم کو پروسیس کریں"""
        try:
            # آڈیو کو NumPy میں تبدیل کریں
            audio_array = np.frombuffer(audio_item['data'], dtype=np.int16).astype(np.float32) / 32768.0

            if self.whisper_model:
                # ٹرانسکرائیب کریں
                result = self.whisper_model.transcribe(audio_array)

                # ٹیکسٹ پبلش کریں
                text_msg = String()
                text_msg.data = result['text']
                self.text_publisher.publish(text_msg)

                self.get_logger().info(f'Transcribed: {result["text"]}')

        except Exception as e:
            self.get_logger().error(f'Process audio item error: {e}')

    def load_whisper_model(self):
        """Whisper ماڈل کو محفوظ طریقے سے لوڈ کریں"""
        try:
            self.get_logger().info('Loading Whisper model...')
            self.whisper_model = whisper.load_model("base")

            if torch.cuda.is_available():
                self.whisper_model = self.whisper_model.cuda()
                self.get_logger().info('Whisper model loaded on GPU')
            else:
                self.get_logger().info('Whisper model loaded on CPU')

        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.publish_error_status(f'Model loading error: {e}')

    def heartbeat_check(self):
        """ہارٹ بیٹ چیک"""
        try:
            status_msg = String()
            status_msg.data = f"Voice node heartbeat at {time.time()}"
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Heartbeat error: {e}')

    def diagnostic_check(self):
        """ڈائیگنوسٹک چیک"""
        try:
            # سسٹم ڈائیگنوسٹکس
            import psutil
            cpu_percent = psutil.cpu_percent()
            memory_percent = psutil.virtual_memory().percent

            diagnostic_msg = String()
            diagnostic_msg.data = f"CPU: {cpu_percent}%, Memory: {memory_percent}%"
            self.status_publisher.publish(diagnostic_msg)

            # وارننگز کے لیے چیک کریں
            if cpu_percent > 80:
                self.get_logger().warn(f'High CPU usage: {cpu_percent}%')
            if memory_percent > 80:
                self.get_logger().warn(f'High memory usage: {memory_percent}%')

        except Exception as e:
            self.get_logger().error(f'Diagnostic check error: {e}')

    def publish_error_status(self, error_msg):
        """ایرر اسٹیٹس پبلش کریں"""
        status_msg = String()
        status_msg.data = f"ERROR: {error_msg}"
        self.status_publisher.publish(status_msg)

def run_troubleshooting_checklist():
    """ٹربل شوٹنگ چیک لسٹ چلائیں"""
    print("Voice Processing Troubleshooting Checklist")
    print("="*50)

    # 1. سسٹم آڈیو کو چیک کریں
    print("\n1. Checking system audio...")
    audio_ok = troubleshoot_microphone()
    print(f"   Audio test: {'PASS' if audio_ok else 'FAIL'}")

    # 2. ماڈل لوڈنگ
    print("\n2. Testing model loading...")
    try:
        model = whisper.load_model("tiny")  # چھوٹا ماڈل ٹیسٹ کے لیے
        print("   Model loading: PASS")
    except Exception as e:
        print(f"   Model loading: FAIL - {e}")

    # 3. یاداشت
    print("\n3. Checking memory usage...")
    import psutil
    memory = psutil.virtual_memory()
    print(f"   Memory usage: {memory.percent}%")
    print(f"   Available: {memory.available / 1e9:.2f} GB")

    # 4. GPU دستیابی
    print("\n4. Checking GPU availability...")
    if torch.cuda.is_available():
        print(f"   GPU available: {torch.cuda.get_device_name()}")
        print(f"   GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.2f} GB")
    else:
        print("   GPU not available, using CPU")

    # 5. ROS 2 کنکشن
    print("\n5. Checking ROS 2 connection...")
    try:
        rclpy.init()
        temp_node = rclpy.create_node('troubleshooting_tester')
        print("   ROS 2 connection: PASS")
        temp_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"   ROS 2 connection: FAIL - {e}")

    print("\nTroubleshooting checklist completed!")

def common_solutions():
    """عام حل پیش کریں"""
    solutions = [
        "1. Verify microphone permissions in system settings",
        "2. Update audio drivers to latest version",
        "3. Use smaller Whisper model for better performance",
        "4. Check for conflicting audio applications",
        "5. Verify network connectivity for cloud services",
        "6. Increase system memory or use quantized models",
        "7. Use noise cancellation techniques",
        "8. Optimize ROS 2 QoS settings for audio streaming"
    ]

    print("\nCommon Solutions for Voice Processing Issues:")
    for solution in solutions:
        print(f"   {solution}")

def main():
    print("Voice Processing Troubleshooting Guide")

    # چیک لسٹ چلائیں
    run_troubleshooting_checklist()

    # عام حل
    common_solutions()

if __name__ == "__main__":
    main()
```

### 4. کارکردگی کے مسائل

#### مسئلہ: زیادہ لیٹنسی

**عوامل**:
- GPU میموری کی کمی
- CPU استعمال زیادہ
- بڑا ماڈل استعمال کرنا
- آڈیو پروسیسنگ کے مسائل

**حل**:

```python
def optimize_for_performance():
    """کارکردگی کے لیے آپٹیمائز کریں"""
    import os
    import gc

    # GPU میموری کو صاف کریں
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()

    # CPU کورس استعمال کریں
    num_cores = os.cpu_count()
    print(f"Available CPU cores: {num_cores}")

    # کم بیچ سائز
    batch_size = 1  # وائس پروسیسنگ کے لیے

    # کوئنٹائزیشن
    # یہ پہلے والی مثال میں دکھایا گیا ہے

    print("Performance optimizations applied")

def memory_management_tips():
    """میموری مینجمنٹ کے مشورے"""
    tips = [
        "1. Use smaller model sizes (tiny, base) for real-time processing",
        "2. Clear GPU cache regularly: torch.cuda.empty_cache()",
        "3. Use half precision (float16) when possible",
        "4. Implement sliding window for audio processing",
        "5. Limit the number of concurrent requests",
        "6. Use memory profiling tools to identify bottlenecks",
        "7. Consider using CPU for non-critical processing"
    ]

    print("Memory Management Tips:")
    for tip in tips:
        print(f"   {tip}")
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- وائس پروسیسنگ کے عام مسائل سیکھے
- ٹربل شوٹنگ کے اوزار اور ٹیکنیکس حاصل کیے
- محفوظ کوڈنگ کے طریقے سیکھے
- کارکردگی کے مسائل کو حل کیا

یہ ٹربل شوٹنگ کے طریقے VLA سسٹم کو مستحکم اور قابل اعتماد بنانے کے لیے اہم ہیں۔