---
sidebar_position: 16
title: "ورکشاپ 4.2: Whisper آپٹیمائزیشن"
---

# ورکشاپ 4.2: Whisper آپٹیمائزیشن

## مقصد
OpenAI Whisper ماڈل کی کارکردگی کو بہتر بنانا اور VLA سسٹم کے لیے ریئل ٹائم وائس ریکوگنیشن کو بہتر بنانا۔

## ضروریات
- Python 3.10+
- PyTorch 1.12+
- OpenAI Whisper
- CUDA 11.8+ (اگر GPU استعمال کر رہے ہیں)
- NumPy
- librosa

## ورکشاپ کے اقدامات

### اقدام 1: Whisper ماڈل کی آپٹیمائزیشن کو سمجھنا
ایک نیا فائل `whisper_optimization.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import whisper
import torch
import numpy as np
import time
import librosa
from typing import Dict, List, Tuple, Optional
import torch.nn as nn

class WhisperOptimizer:
    def __init__(self, model_size="base"):
        self.model_size = model_size
        self.model = None
        self.original_model = None

        # ماڈل کو لوڈ کریں
        self.load_model()

        print(f"Whisper optimizer initialized with {model_size} model")

    def load_model(self):
        """Whisper ماڈل لوڈ کریں"""
        try:
            print(f"Loading Whisper {self.model_size} model...")
            self.original_model = whisper.load_model(self.model_size)
            self.model = self.original_model

            # GPU استعمال کریں اگر دستیاب ہو
            if torch.cuda.is_available():
                print("Moving model to GPU...")
                self.model = self.model.cuda()
                print(f"GPU: {torch.cuda.get_device_name()}")
                print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.2f} GB")
            else:
                print("Using CPU")

        except Exception as e:
            print(f"Error loading model: {e}")

    def optimize_with_tensorrt(self):
        """TensorRT کے ذریعے ماڈل کو آپٹیمائز کریں"""
        try:
            import tensorrt as trt
            from torch2trt import torch2trt

            print("Optimizing with TensorRT...")

            # ٹیسٹ ان پٹ
            dummy_input = torch.randn(1, 80, 3000).cuda()  # 30 سیکنڈ کا آڈیو

            # TensorRT ماڈل تیار کریں
            model_trt = torch2trt(
                self.model,
                [dummy_input],
                fp16_mode=True,
                max_workspace_size=1 << 28  # 256 MB
            )

            self.model = model_trt
            print("TensorRT optimization completed")

        except ImportError:
            print("TensorRT not available, skipping optimization")
        except Exception as e:
            print(f"TensorRT optimization error: {e}")

    def quantize_model(self):
        """ماڈل کو کوئنٹائز کریں"""
        try:
            print("Quantizing model...")

            # ایوالویشن موڈ میں سیٹ کریں
            self.model.eval()

            # کوئنٹائزیشن کنفیگریشن
            quantized_model = torch.quantization.quantize_dynamic(
                self.model, {torch.nn.Linear}, dtype=torch.qint8
            )

            self.model = quantized_model
            print("Model quantization completed")

        except Exception as e:
            print(f"Quantization error: {e}")

    def prune_model(self, sparsity_ratio=0.2):
        """ماڈل کو پریون کریں"""
        try:
            import torch.nn.utils.prune as prune

            print(f"Pruning model with {sparsity_ratio*100}% sparsity...")

            # صرف لینئر لیئرز کو پریون کریں
            for name, module in self.model.named_modules():
                if isinstance(module, torch.nn.Linear):
                    prune.l1_unstructured(module, name='weight', amount=sparsity_ratio)

            print("Model pruning completed")

        except Exception as e:
            print(f"Pruning error: {e}")

    def compile_model(self):
        """PyTorch 2.0 کمپائل کا استعمال کریں"""
        try:
            print("Compiling model with torch.compile...")

            # PyTorch 2.0 کمپائل
            self.model = torch.compile(self.model, mode='reduce-overhead', fullgraph=True)
            print("Model compilation completed")

        except Exception as e:
            print(f"Compilation error: {e}")

    def benchmark_model(self, audio_data, num_runs=10):
        """ماڈل کی کارکردگی کو ٹیسٹ کریں"""
        print(f"Benchmarking model over {num_runs} runs...")

        times = []
        original_times = []

        # آڈیو ڈیٹا کو تیار کریں
        audio_tensor = self.preprocess_audio(audio_data)

        # آپٹیمائیزڈ ماڈل ٹیسٹ
        for i in range(num_runs):
            start_time = time.time()

            try:
                # کم سے کم چیزوں کو ٹرانسکرائیب کریں
                result = self.model.transcribe(
                    audio_tensor,
                    language='en',
                    without_timestamps=True
                )

                end_time = time.time()
                times.append(end_time - start_time)

                if i % 5 == 0:
                    print(f"Optimized run {i+1}/{num_runs}: {times[-1]:.3f}s")

            except Exception as e:
                print(f"Error in optimized run {i+1}: {e}")

        # اصل ماڈل کے ساتھ ٹیسٹ (اگر دستیاب ہو)
        if self.original_model is not None:
            for i in range(min(3, num_runs)):  # صرف 3 ٹیسٹس کے لیے
                start_time = time.time()

                try:
                    result = self.original_model.transcribe(
                        audio_tensor,
                        language='en',
                        without_timestamps=True
                    )

                    end_time = time.time()
                    original_times.append(end_time - start_time)

                    print(f"Original run {i+1}: {original_times[-1]:.3f}s")

                except Exception as e:
                    print(f"Error in original run {i+1}: {e}")

        return {
            'optimized_times': times,
            'original_times': original_times,
            'avg_optimized_time': sum(times) / len(times) if times else 0,
            'avg_original_time': sum(original_times) / len(original_times) if original_times else 0
        }

    def preprocess_audio(self, audio_data):
        """آڈیو کو ماڈل کے لیے پری پروسیس کریں"""
        if isinstance(audio_data, str):
            # فائل پاتھ سے لوڈ کریں
            audio, sr = librosa.load(audio_data, sr=16000)
        elif isinstance(audio_data, np.ndarray):
            audio = audio_data
        else:
            raise ValueError("Unsupported audio data type")

        # NumPy سے PyTorch ٹینسر
        audio_tensor = torch.from_numpy(audio).float()

        # GPU پر منتقل کریں اگر دستیاب ہو
        if torch.cuda.is_available():
            audio_tensor = audio_tensor.cuda()

        return audio_tensor

    def dynamic_batching(self, audio_segments: List[np.ndarray], max_batch_size=4):
        """ڈائینامک بیچنگ کا استعمال کریں"""
        print(f"Processing {len(audio_segments)} segments with dynamic batching...")

        results = []

        # بیچز میں تقسیم کریں
        for i in range(0, len(audio_segments), max_batch_size):
            batch = audio_segments[i:i + max_batch_size]

            # بیچ کو پروسیس کریں
            batch_start = time.time()
            for audio_segment in batch:
                audio_tensor = self.preprocess_audio(audio_segment)
                result = self.model.transcribe(audio_tensor)
                results.append(result)

            batch_time = time.time() - batch_start
            print(f"Processed batch {i//max_batch_size + 1} in {batch_time:.3f}s")

        return results

    def streaming_optimization(self, audio_stream_callback, chunk_duration=1.0):
        """اسٹریمنگ کے لیے آپٹیمائزیشن"""
        print(f"Setting up streaming with {chunk_duration}s chunks...")

        # کیش کو سیٹ کریں
        self.model.cache = {}

        def process_stream():
            results = []
            buffer = np.array([])

            for audio_chunk in audio_stream_callback():
                buffer = np.concatenate([buffer, audio_chunk])

                # اگر بفر کافی بڑا ہو تو پروسیس کریں
                if len(buffer) >= int(16000 * chunk_duration):
                    start_time = time.time()

                    # ٹرانسکرائیب کریں
                    audio_tensor = self.preprocess_audio(buffer[:int(16000 * chunk_duration)])
                    result = self.model.transcribe(audio_tensor)

                    process_time = time.time() - start_time
                    results.append((result, process_time))

                    # استعمال کیا ہوا حصہ ہٹائیں
                    buffer = buffer[int(16000 * chunk_duration):]

            return results

        return process_stream

    def memory_optimization(self):
        """میموری استعمال کو کم کریں"""
        try:
            print("Optimizing memory usage...")

            # گریڈیئنٹ چیک پوائنٹنگ
            for name, module in self.model.named_modules():
                if hasattr(module, 'gradient_checkpointing_enable'):
                    module.gradient_checkpointing_enable()

            # میموری کو کم کریں
            torch.backends.cudnn.benchmark = True

            print("Memory optimization completed")

        except Exception as e:
            print(f"Memory optimization error: {e}")

    def get_model_info(self):
        """ماڈل کی معلومات حاصل کریں"""
        if self.model is None:
            return {}

        # پیرامیٹرز کی تعداد
        total_params = sum(p.numel() for p in self.model.parameters())
        trainable_params = sum(p.numel() for p in self.model.parameters() if p.requires_grad)

        # ماڈل سائز (MB)
        param_size = sum(p.nelement() * p.element_size() for p in self.model.parameters())
        buffer_size = sum(b.nelement() * b.element_size() for b in self.model.buffers())
        model_size = (param_size + buffer_size) / 1024**2

        return {
            'total_parameters': total_params,
            'trainable_parameters': trainable_params,
            'model_size_mb': model_size,
            'device': next(self.model.parameters()).device
        }

def optimize_whisper_pipeline():
    """مکمل Whisper پائپ لائن کو آپٹیمائز کریں"""
    print("Starting Whisper pipeline optimization...")

    # آپٹیمائزر تیار کریں
    optimizer = WhisperOptimizer(model_size="base")

    # ماڈل کی معلومات حاصل کریں
    model_info = optimizer.get_model_info()
    print(f"Model info: {model_info}")

    # مختلف آپٹیمائزیشنز لاگو کریں
    print("\nApplying optimizations...")

    # کمپائل کریں
    optimizer.compile_model()

    # میموری آپٹیمائز کریں
    optimizer.memory_optimization()

    # ٹیسٹ ڈیٹا کے ساتھ ٹیسٹ کریں
    test_audio = np.random.randn(16000 * 5).astype(np.float32)  # 5 سیکنڈ کا ٹیسٹ آڈیو

    # کارکردگی ٹیسٹ
    benchmark_results = optimizer.benchmark_model(test_audio, num_runs=5)

    print(f"\nBenchmark Results:")
    print(f"Average optimized time: {benchmark_results['avg_optimized_time']:.3f}s")
    print(f"Average original time: {benchmark_results['avg_original_time']:.3f}s")

    # کتنی تیزی ہوئی؟
    if benchmark_results['avg_original_time'] > 0:
        speedup = benchmark_results['avg_original_time'] / benchmark_results['avg_optimized_time']
        print(f"Speedup: {speedup:.2f}x")

    return optimizer

def main():
    try:
        optimizer = optimize_whisper_pipeline()
        print("\nWhisper optimization completed successfully!")
    except Exception as e:
        print(f"Error during optimization: {e}")

if __name__ == "__main__":
    main()
```

### اقدام 2: ROS 2 انٹیگریشن کے ساتھ آپٹیمائزیشن
ایک فائل `whisper_ros_integration.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import whisper
import torch
import numpy as np
import time
from typing import Dict, Any
import threading
import queue

class OptimizedWhisperNode(Node):
    def __init__(self):
        super().__init__('optimized_whisper_node')

        # Whisper آپٹیمائزر
        self.whisper_optimizer = WhisperOptimizer(model_size="base")

        # کیش کے لیے متغیرات
        self.transcription_cache = {}
        self.cache_size_limit = 100

        # تھریڈنگ کے لیے
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_audio_queue)
        self.processing_active = True

        # سبسکرائبرز
        self.audio_subscriber = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10
        )

        # پبلیشرز
        self.text_publisher = self.create_publisher(
            String, 'transcribed_text', 10
        )
        self.status_publisher = self.create_publisher(
            String, 'whisper_status', 10
        )

        # ٹیمرز
        self.cache_cleanup_timer = self.create_timer(60.0, self.cleanup_cache)

        # شروع کریں
        self.processing_thread.start()
        self.get_logger().info('Optimized Whisper Node initialized')

    def audio_callback(self, msg: AudioData):
        """آڈیو ڈیٹا کو ہینڈل کریں"""
        try:
            # آڈیو کو NumPy ارے میں تبدیل کریں
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # کیو میں ڈالیں
            self.audio_queue.put({
                'audio_data': audio_array,
                'timestamp': msg.header.stamp
            })

            self.get_logger().debug(f'Audio queued: {len(audio_array)} samples')

        except Exception as e:
            self.get_logger().error(f'Audio callback error: {e}')

    def process_audio_queue(self):
        """آڈیو کو پروسیس کریں (سیپریٹ تھریڈ میں)"""
        while self.processing_active:
            try:
                # ڈیٹا حاصل کریں
                if not self.audio_queue.empty():
                    audio_item = self.audio_queue.get(timeout=0.1)

                    # کیش چیک کریں
                    audio_hash = hash(audio_item['audio_data'].tobytes())
                    if audio_hash in self.transcription_cache:
                        result = self.transcription_cache[audio_hash]
                        self.get_logger().debug('Cache hit')
                    else:
                        # ٹرانسکرائیب کریں
                        result = self.whisper_optimizer.model.transcribe(audio_item['audio_data'])
                        self.transcription_cache[audio_hash] = result

                        # کیش سائز چیک کریں
                        if len(self.transcription_cache) > self.cache_size_limit:
                            # پرانے ایٹمز کو ہٹائیں
                            oldest_key = next(iter(self.transcription_cache))
                            del self.transcription_cache[oldest_key]

                    # نتیجہ کو پبلش کریں
                    self.publish_transcription_result(result, audio_item['timestamp'])

                else:
                    time.sleep(0.01)  # 10ms delay

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Audio processing error: {e}')

    def publish_transcription_result(self, result: Dict[str, Any], timestamp):
        """ٹرانسکرپشن نتیجہ پبلش کریں"""
        text_msg = String()
        text_data = {
            'text': result.get('text', ''),
            'language': result.get('language', 'unknown'),
            'timestamp': timestamp,
            'confidence': result.get('confidence', 0.9)
        }
        text_msg.data = str(text_data)
        self.text_publisher.publish(text_msg)

    def cleanup_cache(self):
        """کیش کو صاف کریں"""
        current_time = time.time()
        self.get_logger().info(f'Cache size: {len(self.transcription_cache)}')

    def destroy_node(self):
        """نوڈ کو ڈسٹرائے کریں"""
        self.processing_active = False
        self.processing_thread.join()
        super().destroy_node()

class WhisperOptimizer:
    """Whisper ماڈل آپٹیمائزر (سادہ ورژن)"""
    def __init__(self, model_size="base"):
        self.model_size = model_size
        self.model = whisper.load_model(model_size)

        if torch.cuda.is_available():
            self.model = self.model.cuda()

    def transcribe_audio(self, audio_data):
        """آڈیو کو ٹرانسکرائیب کریں"""
        return self.model.transcribe(audio_data)

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedWhisperNode()

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

### اقدام 3: پرفارمنس ٹیسٹنگ کے لیے کوڈ
ایک فائل `whisper_benchmark.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import whisper
import torch
import numpy as np
import time
import statistics
from typing import Dict, List
import matplotlib.pyplot as plt

class WhisperBenchmark:
    def __init__(self, model_sizes=None):
        if model_sizes is None:
            self.model_sizes = ["tiny", "base", "small", "medium"]
        else:
            self.model_sizes = model_sizes

        self.results = {}

    def benchmark_single_model(self, model_size: str, audio_duration: float = 5.0, runs: int = 5):
        """ایک ماڈل کو ٹیسٹ کریں"""
        print(f"Benchmarking Whisper {model_size} model...")

        # ماڈل لوڈ کریں
        model = whisper.load_model(model_size)

        if torch.cuda.is_available():
            model = model.cuda()

        # ٹیسٹ آڈیو تیار کریں
        sample_rate = 16000
        samples = int(sample_rate * audio_duration)
        test_audio = np.random.randn(samples).astype(np.float32)

        times = []
        memory_usage = []

        for run in range(runs):
            start_time = time.time()

            try:
                # ٹرانسکرائیب کریں
                result = model.transcribe(test_audio)

                end_time = time.time()
                times.append(end_time - start_time)

                print(f"Run {run + 1}/{runs}: {times[-1]:.3f}s")

            except Exception as e:
                print(f"Error in run {run + 1}: {e}")

        # نتائج کو ذخیرہ کریں
        if times:
            self.results[model_size] = {
                'times': times,
                'avg_time': statistics.mean(times),
                'min_time': min(times),
                'max_time': max(times),
                'std_dev': statistics.stdev(times) if len(times) > 1 else 0,
                'throughput': audio_duration / statistics.mean(times) if statistics.mean(times) > 0 else 0
            }

        return self.results.get(model_size, {})

    def benchmark_all_models(self, audio_duration: float = 5.0, runs: int = 5):
        """تمام ماڈلز کو ٹیسٹ کریں"""
        print(f"Starting benchmark for {len(self.model_sizes)} models...")

        for model_size in self.model_sizes:
            self.benchmark_single_model(model_size, audio_duration, runs)

        return self.results

    def print_benchmark_summary(self):
        """ٹیسٹ کا خلاصہ پرنٹ کریں"""
        print("\n" + "="*70)
        print("WHISPER BENCHMARK SUMMARY")
        print("="*70)

        print(f"{'Model':<10} {'Avg Time':<10} {'Min Time':<10} {'Max Time':<10} {'Throughput':<12} {'Std Dev':<10}")
        print("-"*70)

        for model_size, result in self.results.items():
            avg_time = result['avg_time']
            min_time = result['min_time']
            max_time = result['max_time']
            throughput = result['throughput']
            std_dev = result['std_dev']

            print(f"{model_size:<10} {avg_time:<10.3f} {min_time:<10.3f} {max_time:<10.3f} {throughput:<12.2f} {std_dev:<10.3f}")

    def plot_results(self):
        """نتائج کو پلاٹ کریں"""
        if not self.results:
            print("No results to plot")
            return

        model_sizes = list(self.results.keys())
        avg_times = [self.results[ms]['avg_time'] for ms in model_sizes]
        throughputs = [self.results[ms]['throughput'] for ms in model_sizes]

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))

        # ایوریج ٹائم پلاٹ
        ax1.bar(model_sizes, avg_times)
        ax1.set_title('Average Transcription Time by Model Size')
        ax1.set_ylabel('Time (seconds)')
        ax1.tick_params(axis='x', rotation=45)

        # تھروپٹ پلاٹ
        ax2.bar(model_sizes, throughputs)
        ax2.set_title('Throughput (Real-time Factor) by Model Size')
        ax2.set_ylabel('Real-time Factor')
        ax2.tick_params(axis='x', rotation=45)

        plt.tight_layout()
        plt.savefig('/tmp/whisper_benchmark.png')
        print("Benchmark plot saved to /tmp/whisper_benchmark.png")

    def compare_optimizations(self, model_size: str = "base"):
        """ مختلف آپٹیمائزیشنز کا موازنہ کریں"""
        print(f"Comparing optimizations for {model_size} model...")

        audio_duration = 5.0
        runs = 5

        # اصل ماڈل
        original_model = whisper.load_model(model_size)
        if torch.cuda.is_available():
            original_model = original_model.cuda()

        sample_rate = 16000
        samples = int(sample_rate * audio_duration)
        test_audio = np.random.randn(samples).astype(np.float32)

        # اصل کارکردگی
        original_times = []
        for _ in range(runs):
            start = time.time()
            result = original_model.transcribe(test_audio)
            original_times.append(time.time() - start)

        print(f"Original model avg time: {statistics.mean(original_times):.3f}s")

        # کمپائل کردہ ماڈل
        try:
            compiled_model = torch.compile(original_model, mode='reduce-overhead', fullgraph=True)

            compiled_times = []
            for _ in range(runs):
                start = time.time()
                result = compiled_model.transcribe(test_audio)
                compiled_times.append(time.time() - start)

            print(f"Compiled model avg time: {statistics.mean(compiled_times):.3f}s")
            print(f"Speedup: {statistics.mean(original_times) / statistics.mean(compiled_times):.2f}x")

        except Exception as e:
            print(f"Compilation optimization error: {e}")

        # کوئنٹائزڈ ماڈل
        try:
            quantized_model = torch.quantization.quantize_dynamic(
                original_model, {torch.nn.Linear}, dtype=torch.qint8
            )

            quantized_times = []
            for _ in range(runs):
                start = time.time()
                result = quantized_model.transcribe(test_audio)
                quantized_times.append(time.time() - start)

            print(f"Quantized model avg time: {statistics.mean(quantized_times):.3f}s")
            print(f"Speedup: {statistics.mean(original_times) / statistics.mean(quantized_times):.2f}x")

        except Exception as e:
            print(f"Quantization optimization error: {e}")

def main():
    # بنچ مارک ٹیسٹ
    benchmark = WhisperBenchmark(model_sizes=["tiny", "base"])

    # ٹیسٹ چلائیں
    results = benchmark.benchmark_all_models(audio_duration=3.0, runs=3)

    # خلاصہ پرنٹ کریں
    benchmark.print_benchmark_summary()

    # موازنہ کریں
    benchmark.compare_optimizations("base")

    # نتائج کو پلاٹ کریں
    try:
        benchmark.plot_results()
    except ImportError:
        print("Matplotlib not available, skipping plot generation")

if __name__ == "__main__":
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- Whisper ماڈل کی مختلف آپٹیمائزیشن تکنیکس سیکھیں
- کمپائل، کوئنٹائزیشن، اور پریونگ کا استعمال کیا
- ROS 2 میں آپٹیمائیزڈ انٹیگریشن کیا
- کارکردگی ٹیسٹنگ اور بنچ مارکنگ کی

یہ آپٹیمائزیشنز VLA سسٹم کے لیے وائس ریکوگنیشن کی کارکردگی کو بہتر بنانے کے لیے اہم ہیں۔