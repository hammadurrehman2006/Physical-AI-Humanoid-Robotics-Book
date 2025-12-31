---
sidebar_position: 15
title: "ورکشاپ 4.1: وائس ریکوگنیشن ٹیسٹنگ"
---

# ورکشاپ 4.1: وائس ریکوگنیشن ٹیسٹنگ

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے وائس ریکوگنیشن کو ٹیسٹ کرنا اور اس کی کارکردگی کو جانچنا۔

## ضروریات
- Python 3.10+
- PyAudio
- OpenAI Whisper
- NumPy
- ROS 2 ہمبل ہاکسبل
- مائیکروفون

## ورکشاپ کے اقدامات

### اقدام 1: وائس ریکوگنیشن ٹیسٹر تیار کریں
ایک نیا فائل `voice_recognition_tester.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import pyaudio
import numpy as np
import whisper
import torch
import time
import wave
from typing import Dict, List, Tuple
import json
import os

class VoiceRecognitionTester:
    def __init__(self, model_size="base"):
        self.model_size = model_size
        self.model = None
        self.audio = pyaudio.PyAudio()

        # آڈیو پیرامیٹرز
        self.rate = 16000
        self.channels = 1
        self.chunk_size = 1024

        # ٹیسٹ متغیرات
        self.test_results = []
        self.total_tests = 0
        self.successful_tests = 0

        # ماڈل لوڈ کریں
        self.load_model()

        print("Voice Recognition Tester initialized")

    def load_model(self):
        """Whisper ماڈل لوڈ کریں"""
        try:
            print(f"Loading Whisper model: {self.model_size}")
            self.model = whisper.load_model(self.model_size)

            if torch.cuda.is_available():
                print("Using GPU for inference")
                self.model = self.model.cuda()
            else:
                print("Using CPU for inference")

        except Exception as e:
            print(f"Error loading model: {e}")

    def record_audio(self, duration=5, filename=None):
        """آڈیو ریکارڈ کریں"""
        print(f"Recording audio for {duration} seconds...")

        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        frames = []
        for _ in range(0, int(self.rate / self.chunk_size * duration)):
            data = stream.read(self.chunk_size)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        audio_data = b''.join(frames)

        # اگر فائل نام دیا گیا ہو تو محفوظ کریں
        if filename:
            self.save_audio_file(audio_data, filename)

        print("Recording completed")
        return audio_data

    def save_audio_file(self, audio_data, filename):
        """آڈیو فائل محفوظ کریں"""
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
            wf.setframerate(self.rate)
            wf.writeframes(audio_data)

    def audio_to_numpy(self, audio_data):
        """آڈیو ڈیٹا کو NumPy ارے میں تبدیل کریں"""
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        return audio_array.astype(np.float32) / 32768.0

    def transcribe_audio(self, audio_data):
        """آڈیو کو ٹرانسکرائیب کریں"""
        if self.model is None:
            return "Model not loaded"

        try:
            start_time = time.time()

            # NumPy ارے میں تبدیل کریں
            if isinstance(audio_data, bytes):
                audio_array = self.audio_to_numpy(audio_data)
            else:
                audio_array = audio_data

            # ٹرانسکرائیب کریں
            result = self.model.transcribe(audio_array)

            end_time = time.time()
            transcription_time = end_time - start_time

            return {
                'text': result["text"],
                'time': transcription_time,
                'language': result.get("language", "unknown")
            }

        except Exception as e:
            print(f"Transcription error: {e}")
            return {
                'text': "",
                'time': 0,
                'error': str(e)
            }

    def test_single_recording(self, expected_text=None):
        """ایک ریکارڈنگ کو ٹیسٹ کریں"""
        print("\nStarting single recording test...")

        # آڈیو ریکارڈ کریں
        audio_data = self.record_audio(duration=5)

        # ٹرانسکرائیب کریں
        result = self.transcribe_audio(audio_data)

        print(f"Transcribed text: {result['text']}")
        print(f"Processing time: {result['time']:.2f}s")

        # اگر متوقع ٹیکسٹ دیا گیا ہو تو موازنہ کریں
        if expected_text:
            accuracy = self.calculate_accuracy(result['text'], expected_text)
            print(f"Expected: {expected_text}")
            print(f"Accuracy: {accuracy:.2f}%")

            test_result = {
                'expected': expected_text,
                'actual': result['text'],
                'accuracy': accuracy,
                'time': result['time'],
                'success': accuracy > 70  # 70% سے زیادہ ہونے پر کامیاب
            }
        else:
            test_result = {
                'actual': result['text'],
                'time': result['time'],
                'success': bool(result['text'].strip())
            }

        self.test_results.append(test_result)
        self.total_tests += 1

        if test_result['success']:
            self.successful_tests += 1

        return test_result

    def calculate_accuracy(self, actual, expected):
        """ٹیکسٹ کی ایکویسی کا حساب کریں"""
        if not actual or not expected:
            return 0.0

        actual_words = set(actual.lower().split())
        expected_words = set(expected.lower().split())

        common_words = actual_words.intersection(expected_words)
        total_expected_words = len(expected_words)

        if total_expected_words == 0:
            return 100.0 if len(actual_words) == 0 else 0.0

        accuracy = (len(common_words) / total_expected_words) * 100
        return min(accuracy, 100.0)  # 100% سے زیادہ نہ ہو

    def run_comprehensive_test(self, test_prompts: List[str]):
        """کمپری ہینسیو ٹیسٹ چلائیں"""
        print(f"\nStarting comprehensive test with {len(test_prompts)} prompts...")

        for i, prompt in enumerate(test_prompts):
            print(f"\nTest {i+1}/{len(test_prompts)}: Speak '{prompt}'")
            input("Press Enter when ready to speak...")

            result = self.test_single_recording(prompt)
            print(f"Result: {'✓' if result['success'] else '✗'}")

        # نتائج کو پرنٹ کریں
        self.print_test_summary()

    def print_test_summary(self):
        """ٹیسٹ کے نتائج کو پرنٹ کریں"""
        print("\n" + "="*50)
        print("VOICE RECOGNITION TEST SUMMARY")
        print("="*50)

        if self.total_tests > 0:
            success_rate = (self.successful_tests / self.total_tests) * 100
            avg_time = sum(r.get('time', 0) for r in self.test_results) / len(self.test_results) if self.test_results else 0

            print(f"Total tests: {self.total_tests}")
            print(f"Successful: {self.successful_tests}")
            print(f"Success rate: {success_rate:.2f}%")
            print(f"Average processing time: {avg_time:.2f}s")

            # اوسط ایکویسی
            successful_results = [r for r in self.test_results if 'accuracy' in r]
            if successful_results:
                avg_accuracy = sum(r['accuracy'] for r in successful_results) / len(successful_results)
                print(f"Average accuracy: {avg_accuracy:.2f}%")
        else:
            print("No tests were run")

    def test_different_noise_levels(self):
        """ مختلف نوائز لیولز پر ٹیسٹ کریں"""
        print("\nTesting under different noise conditions...")

        noise_conditions = [
            ("Quiet room", 5),
            ("Moderate noise", 5),
            ("Noisy environment", 5)
        ]

        for condition, duration in noise_conditions:
            print(f"\nTesting in: {condition}")
            input(f"Set up {condition} and press Enter...")

            audio_data = self.record_audio(duration=duration)
            result = self.transcribe_audio(audio_data)

            print(f"Transcribed: {result['text']}")
            print(f"Time: {result['time']:.2f}s")

    def cleanup(self):
        """ریسورسز کو صاف کریں"""
        self.audio.terminate()

def main():
    tester = VoiceRecognitionTester(model_size="base")

    try:
        # ٹیسٹ پروموٹس
        test_prompts = [
            "Hello robot please help me",
            "Pick up the red ball from the table",
            "Go to the kitchen and bring water",
            "Turn off the lights in the room",
            "What is the weather today"
        ]

        print("Voice Recognition Tester")
        print("Choose test type:")
        print("1. Single recording test")
        print("2. Comprehensive test")
        print("3. Noise level test")

        choice = input("Enter choice (1-3): ")

        if choice == "1":
            tester.test_single_recording()
        elif choice == "2":
            tester.run_comprehensive_test(test_prompts)
        elif choice == "3":
            tester.test_different_noise_levels()
        else:
            print("Invalid choice")

    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()
```

### اقدام 2: ROS 2 انٹیگریشن ٹیسٹ کریں
ایک فائل `voice_ros_tester.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import pyaudio
import numpy as np
import whisper
import torch
import json
import time

class VoiceROSTester(Node):
    def __init__(self):
        super().__init__('voice_ros_tester')

        # سبسکرائبرز
        self.voice_status_subscriber = self.create_subscription(
            String, 'voice_status', self.voice_status_callback, 10
        )
        self.text_command_subscriber = self.create_subscription(
            String, 'text_commands', self.text_command_callback, 10
        )

        # پبلیشرز
        self.voice_command_publisher = self.create_publisher(
            String, 'voice_commands', 10
        )
        self.test_status_publisher = self.create_publisher(
            String, 'test_status', 10
        )

        # ٹیسٹ متغیرات
        self.test_results = []
        self.current_test = None
        self.test_start_time = None

        # وائس کمانڈز کے لیے پیرامیٹرز
        self.test_commands = [
            "pick up the red ball",
            "go to the kitchen",
            "put the cup on the table",
            "turn off the lights"
        ]

        self.get_logger().info('Voice ROS Tester initialized')

    def voice_status_callback(self, msg):
        """وائس سٹیٹس کو ہینڈل کریں"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status', 'unknown')

            self.get_logger().info(f'Voice status: {status}')

            if self.current_test:
                self.record_test_result(status_data)

        except Exception as e:
            self.get_logger().error(f'Error processing voice status: {e}')

    def text_command_callback(self, msg):
        """ٹیکسٹ کمانڈ کو ہینڈل کریں"""
        try:
            command_data = json.loads(msg.data)
            text = command_data.get('text', '')

            self.get_logger().info(f'Received text command: {text}')

            if self.current_test:
                self.validate_test_result(text)

        except Exception as e:
            self.get_logger().error(f'Error processing text command: {e}')

    def run_ros_integration_test(self):
        """ROS انٹیگریشن ٹیسٹ چلائیں"""
        self.get_logger().info('Starting ROS integration test...')

        for i, command in enumerate(self.test_commands):
            self.get_logger().info(f'Testing command {i+1}/{len(self.test_commands)}: {command}')

            # ٹیسٹ شروع کریں
            self.start_test(command)

            # کمانڈ پبلش کریں
            self.publish_test_command(command)

            # 5 سیکنڈ تک نتیجہ انتظار کریں
            time.sleep(5)

            # ٹیسٹ ختم کریں
            self.end_test()

        self.print_test_summary()

    def start_test(self, command):
        """ٹیسٹ شروع کریں"""
        self.current_test = {
            'expected_command': command,
            'start_time': time.time(),
            'results': []
        }
        self.test_start_time = time.time()

    def publish_test_command(self, command):
        """ٹیسٹ کمانڈ پبلش کریں"""
        command_msg = String()

        command_data = {
            'text': command,
            'timestamp': self.get_clock().now().nanoseconds,
            'test_mode': True
        }

        command_msg.data = json.dumps(command_data)
        self.voice_command_publisher.publish(command_msg)

    def record_test_result(self, status_data):
        """ٹیسٹ کا نتیجہ ریکارڈ کریں"""
        if self.current_test:
            result = {
                'status': status_data,
                'timestamp': time.time()
            }
            self.current_test['results'].append(result)

    def validate_test_result(self, received_text):
        """ٹیسٹ کا نتیجہ والیڈیٹ کریں"""
        if self.current_test:
            expected = self.current_test['expected_command']
            accuracy = self.calculate_text_similarity(received_text.lower(), expected.lower())

            result = {
                'expected': expected,
                'received': received_text,
                'accuracy': accuracy,
                'success': accuracy > 70
            }

            self.test_results.append(result)

            self.get_logger().info(f'Test result - Expected: {expected}')
            self.get_logger().info(f'Test result - Received: {received_text}')
            self.get_logger().info(f'Test result - Accuracy: {accuracy:.2f}%')
            self.get_logger().info(f'Test result - Success: {result["success"]}')

    def calculate_text_similarity(self, text1, text2):
        """ٹیکسٹ کی مشابہت کا حساب کریں"""
        words1 = set(text1.split())
        words2 = set(text2.split())

        common_words = words1.intersection(words2)
        total_words = len(words1.union(words2))

        if total_words == 0:
            return 100.0 if len(words1) == 0 and len(words2) == 0 else 0.0

        similarity = (len(common_words) / total_words) * 100
        return min(similarity, 100.0)

    def end_test(self):
        """ٹیسٹ ختم کریں"""
        if self.current_test:
            self.current_test = None
            self.test_start_time = None

    def print_test_summary(self):
        """ٹیسٹ کا خلاصہ پرنٹ کریں"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('ROS INTEGRATION TEST SUMMARY')
        self.get_logger().info('='*50)

        if self.test_results:
            successful_tests = sum(1 for r in self.test_results if r['success'])
            total_tests = len(self.test_results)
            success_rate = (successful_tests / total_tests) * 100 if total_tests > 0 else 0

            avg_accuracy = sum(r['accuracy'] for r in self.test_results) / total_tests if total_tests > 0 else 0

            self.get_logger().info(f'Total tests: {total_tests}')
            self.get_logger().info(f'Successful tests: {successful_tests}')
            self.get_logger().info(f'Success rate: {success_rate:.2f}%')
            self.get_logger().info(f'Average accuracy: {avg_accuracy:.2f}%')

            for i, result in enumerate(self.test_results):
                status = '✓' if result['success'] else '✗'
                self.get_logger().info(f'{status} Test {i+1}: {result["accuracy"]:.2f}% - "{result["expected"]}"')
        else:
            self.get_logger().info('No test results to display')

def main(args=None):
    rclpy.init(args=args)
    tester = VoiceROSTester()

    try:
        # ٹیسٹ چلائیں
        tester.run_ros_integration_test()

        # چند سیکنڈ تک انتظار کریں
        time.sleep(2)

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 3: کارکردگی ٹیسٹنگ کریں
ایک فائل `performance_tester.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import pyaudio
import numpy as np
import whisper
import torch
import time
import json
from typing import Dict, List
import statistics

class VoicePerformanceTester:
    def __init__(self, model_size="base"):
        self.model_size = model_size
        self.model = None
        self.audio = pyaudio.PyAudio()

        # آڈیو پیرامیٹرز
        self.rate = 16000
        self.channels = 1
        self.chunk_size = 1024

        # کارکردگی ٹیسٹ متغیرات
        self.timing_results = []
        self.memory_usage = []

        # ماڈل لوڈ کریں
        self.load_model()

        print("Voice Performance Tester initialized")

    def load_model(self):
        """Whisper ماڈل لوڈ کریں"""
        try:
            print(f"Loading Whisper model: {self.model_size}")
            self.model = whisper.load_model(self.model_size)

            if torch.cuda.is_available():
                print("Using GPU for inference")
                self.model = self.model.cuda()
            else:
                print("Using CPU for inference")

        except Exception as e:
            print(f"Error loading model: {e}")

    def generate_test_audio(self, duration=3):
        """ٹیسٹ آڈیو جنریٹ کریں"""
        # سادہ ٹیسٹ آڈیو (اصل میں آپ اصل ڈیٹا استعمال کریں گے)
        samples = int(self.rate * duration)
        audio_data = np.random.normal(0, 0.1, samples).astype(np.float32)

        # NumPy سے bytes میں تبدیل کریں
        audio_bytes = (audio_data * 32767).astype(np.int16).tobytes()

        return audio_bytes

    def measure_transcription_performance(self, audio_data, iterations=5):
        """ٹرانسکرپشن کارکردگی کو ناپیں"""
        print(f"Measuring transcription performance over {iterations} iterations...")

        times = []

        for i in range(iterations):
            start_time = time.time()

            try:
                # NumPy ارے میں تبدیل کریں
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                audio_array = audio_array.astype(np.float32) / 32768.0

                # ٹرانسکرائیب کریں
                result = self.model.transcribe(audio_array)

                end_time = time.time()
                transcription_time = end_time - start_time

                times.append(transcription_time)

                print(f"Iteration {i+1}: {transcription_time:.3f}s")

            except Exception as e:
                print(f"Error in iteration {i+1}: {e}")

        return times

    def run_latency_test(self):
        """لیٹنسی ٹیسٹ چلائیں"""
        print("\nRunning latency test...")

        # ٹیسٹ آڈیو تیار کریں
        test_audio = self.generate_test_audio(duration=3)

        # کارکردگی ٹیسٹ چلائیں
        times = self.measure_transcription_performance(test_audio, iterations=10)

        if times:
            avg_time = statistics.mean(times)
            min_time = min(times)
            max_time = max(times)
            std_dev = statistics.stdev(times) if len(times) > 1 else 0

            print(f"\nLatency Test Results:")
            print(f"Average time: {avg_time:.3f}s")
            print(f"Min time: {min_time:.3f}s")
            print(f"Max time: {max_time:.3f}s")
            print(f"Standard deviation: {std_dev:.3f}s")
            print(f"FPS equivalent: {1/avg_time:.2f} FPS")

            return {
                'avg_time': avg_time,
                'min_time': min_time,
                'max_time': max_time,
                'std_dev': std_dev,
                'fps': 1/avg_time
            }

        return None

    def run_throughput_test(self, duration=30):
        """تھروپٹ ٹیسٹ چلائیں"""
        print(f"\nRunning throughput test for {duration} seconds...")

        start_time = time.time()
        processed_count = 0
        times = []

        while time.time() - start_time < duration:
            # ٹیسٹ آڈیو تیار کریں
            test_audio = self.generate_test_audio(duration=2)

            iteration_start = time.time()

            try:
                # NumPy ارے میں تبدیل کریں
                audio_array = np.frombuffer(test_audio, dtype=np.int16)
                audio_array = audio_array.astype(np.float32) / 32768.0

                # ٹرانسکرائیب کریں
                result = self.model.transcribe(audio_array)

                iteration_time = time.time() - iteration_start
                times.append(iteration_time)

                processed_count += 1

                print(f"Processed {processed_count} audio clips, time: {iteration_time:.3f}s", end='\r')

            except Exception as e:
                print(f"Error processing audio: {e}")

        total_time = time.time() - start_time
        avg_time = statistics.mean(times) if times else 0
        throughput = processed_count / total_time if total_time > 0 else 0

        print(f"\n\nThroughput Test Results:")
        print(f"Total processed: {processed_count}")
        print(f"Total time: {total_time:.2f}s")
        print(f"Average processing time: {avg_time:.3f}s")
        print(f"Throughput: {throughput:.2f} clips/second")

        return {
            'total_processed': processed_count,
            'total_time': total_time,
            'avg_time': avg_time,
            'throughput': throughput
        }

    def run_stress_test(self, iterations=100):
        """سٹریس ٹیسٹ چلائیں"""
        print(f"\nRunning stress test with {iterations} iterations...")

        times = []
        errors = 0

        for i in range(iterations):
            test_audio = self.generate_test_audio(duration=2)

            try:
                start_time = time.time()

                # NumPy ارے میں تبدیل کریں
                audio_array = np.frombuffer(test_audio, dtype=np.int16)
                audio_array = audio_array.astype(np.float32) / 32768.0

                # ٹرانسکرائیب کریں
                result = self.model.transcribe(audio_array)

                end_time = time.time()
                times.append(end_time - start_time)

                if i % 10 == 0:
                    print(f"Completed {i}/{iterations} iterations", end='\r')

            except Exception as e:
                errors += 1
                print(f"Error in iteration {i+1}: {e}")

        print(f"\n\nStress Test Results:")
        print(f"Total iterations: {iterations}")
        print(f"Successful: {iterations - errors}")
        print(f"Errors: {errors}")
        print(f"Success rate: {((iterations - errors) / iterations) * 100:.2f}%")

        if times:
            avg_time = statistics.mean(times)
            print(f"Average processing time: {avg_time:.3f}s")

        return {
            'total_iterations': iterations,
            'successful': iterations - errors,
            'errors': errors,
            'success_rate': ((iterations - errors) / iterations) * 100,
            'avg_time': statistics.mean(times) if times else 0
        }

    def print_performance_summary(self, latency_results, throughput_results, stress_results):
        """کارکردگی کا خلاصہ پرنٹ کریں"""
        print("\n" + "="*60)
        print("VOICE RECOGNITION PERFORMANCE SUMMARY")
        print("="*60)

        if latency_results:
            print(f"Latency Performance:")
            print(f"  Average processing time: {latency_results['avg_time']:.3f}s")
            print(f"  Min processing time: {latency_results['min_time']:.3f}s")
            print(f"  Max processing time: {latency_results['max_time']:.3f}s")
            print(f"  Throughput potential: {latency_results['fps']:.2f} FPS")

        if throughput_results:
            print(f"\nThroughput Performance:")
            print(f"  Processing rate: {throughput_results['throughput']:.2f} clips/second")
            print(f"  Average per clip: {throughput_results['avg_time']:.3f}s")

        if stress_results:
            print(f"\nStress Test Performance:")
            print(f"  Success rate: {stress_results['success_rate']:.2f}%")
            print(f"  Average processing time: {stress_results['avg_time']:.3f}s")

def main():
    tester = VoicePerformanceTester(model_size="base")

    try:
        print("Voice Recognition Performance Tester")
        print("Choose test type:")
        print("1. Latency test")
        print("2. Throughput test")
        print("3. Stress test")
        print("4. All tests")

        choice = input("Enter choice (1-4): ")

        latency_results = None
        throughput_results = None
        stress_results = None

        if choice == "1":
            latency_results = tester.run_latency_test()
        elif choice == "2":
            throughput_results = tester.run_throughput_test()
        elif choice == "3":
            stress_results = tester.run_stress_test()
        elif choice == "4":
            latency_results = tester.run_latency_test()
            throughput_results = tester.run_throughput_test()
            stress_results = tester.run_stress_test()
        else:
            print("Invalid choice")

        # خلاصہ پرنٹ کریں
        tester.print_performance_summary(latency_results, throughput_results, stress_results)

    finally:
        # ریسورسز کو صاف کریں
        tester.audio.terminate()

if __name__ == "__main__":
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- وائس ریکوگنیشن کو ٹیسٹ کرنے کے لیے ٹیسٹر تیار کیا
- ROS 2 انٹیگریشن ٹیسٹنگ کی
- کارکردگی ٹیسٹنگ کی
- لیٹنسی، تھروپٹ، اور سٹریس ٹیسٹ چلائے

یہ ٹیسٹنگ VLA سسٹم کے لیے وائس ریکوگنیشن کی کارکردگی کو یقینی بنانے کے لیے اہم ہے۔