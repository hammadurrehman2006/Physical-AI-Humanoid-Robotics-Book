# Exercise 2: Whisper Performance Configuration and Optimization

## Objective
Configure and optimize Whisper speech recognition performance for different environments and use cases in the Vision-Language-Action system.

## Prerequisites
- Completed previous exercises
- Working voice processing system
- OpenAI API key configured
- Understanding of Whisper model variants and parameters

## Exercise Steps

### Step 1: Understand Whisper Configuration Parameters

Create a file `whisper_configurator.py`:

```python
#!/usr/bin/env python3
"""
Whisper configuration and optimization tools
"""
import openai
import time
import json
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import numpy as np

@dataclass
class WhisperConfig:
    """Configuration parameters for Whisper recognition"""
    model: str = "whisper-1"
    language: str = "en"
    temperature: float = 0.0
    response_format: str = "json"
    timeout: int = 30
    max_audio_duration: int = 30  # seconds
    vad_enabled: bool = True
    vad_threshold: float = 0.3
    noise_reduction: bool = True
    noise_reduction_factor: float = 0.3
    preprocessing_enabled: bool = True
    buffer_size: int = 44100  # 1 second at 44.1kHz
    processing_interval: float = 0.1  # seconds

class WhisperPerformanceOptimizer:
    """Tools for optimizing Whisper performance"""

    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.config = WhisperConfig()
        self.performance_metrics = []

    def test_model_performance(self, audio_data: bytes, model: str = "whisper-1",
                             language: str = "en", temperature: float = 0.0) -> Dict:
        """Test performance of different Whisper models"""
        start_time = time.time()

        try:
            # Create a temporary file-like object
            import io
            from pydub import AudioSegment

            # Convert bytes to AudioSegment and then to WAV format
            audio_segment = AudioSegment(
                data=audio_data,
                sample_width=2,  # 16-bit
                frame_rate=44100,
                channels=1
            )

            wav_buffer = io.BytesIO()
            audio_segment.export(wav_buffer, format="wav")
            wav_buffer.seek(0)

            # Call Whisper API
            response = openai.Audio.transcribe(
                model=model,
                file=wav_buffer,
                language=language,
                response_format="verbose_json",
                temperature=temperature
            )

            end_time = time.time()
            processing_time = end_time - start_time

            result = {
                'model': model,
                'language': language,
                'temperature': temperature,
                'processing_time': processing_time,
                'text': response.get('text', ''),
                'segments': response.get('segments', []),
                'success': True,
                'error': None
            }

            return result

        except Exception as e:
            end_time = time.time()
            processing_time = end_time - start_time

            result = {
                'model': model,
                'language': language,
                'temperature': temperature,
                'processing_time': processing_time,
                'text': '',
                'segments': [],
                'success': False,
                'error': str(e)
            }

            return result

    def benchmark_models(self, audio_data: bytes) -> Dict:
        """Benchmark different Whisper models"""
        models = [
            "whisper-1"
        ]

        languages = ["en", "es", "fr"]  # Test with multiple languages
        temperatures = [0.0, 0.5, 1.0]

        results = {}

        for model in models:
            results[model] = {}
            for lang in languages:
                results[model][lang] = {}
                for temp in temperatures:
                    print(f"Testing {model} with {lang} at temp {temp}...")
                    result = self.test_model_performance(audio_data, model, lang, temp)
                    results[model][lang][temp] = result

        return results

    def optimize_for_environment(self, environment_type: str) -> WhisperConfig:
        """Get optimized configuration for specific environment"""
        config = WhisperConfig()

        if environment_type == "quiet_office":
            # Quiet environment - high sensitivity
            config.vad_threshold = 0.1
            config.noise_reduction_factor = 0.1
            config.temperature = 0.0
        elif environment_type == "noisy_office":
            # Moderate noise - balanced settings
            config.vad_threshold = 0.4
            config.noise_reduction_factor = 0.4
            config.temperature = 0.2
        elif environment_type == "industrial":
            # High noise - aggressive filtering
            config.vad_threshold = 0.7
            config.noise_reduction_factor = 0.6
            config.temperature = 0.3
        elif environment_type == "outdoor":
            # Variable conditions - adaptive settings
            config.vad_threshold = 0.5
            config.noise_reduction_factor = 0.5
            config.temperature = 0.25
        else:
            # Default settings
            config.vad_threshold = 0.3
            config.noise_reduction_factor = 0.3
            config.temperature = 0.0

        return config

    def adaptive_config(self, audio_level: float, noise_level: float) -> WhisperConfig:
        """Generate adaptive configuration based on audio characteristics"""
        config = WhisperConfig()

        # Adjust VAD threshold based on audio level
        if audio_level < 0.01:
            config.vad_threshold = 0.05  # Very quiet
        elif audio_level < 0.1:
            config.vad_threshold = 0.2   # Quiet
        elif audio_level < 0.5:
            config.vad_threshold = 0.3   # Normal
        else:
            config.vad_threshold = 0.6   # Loud

        # Adjust noise reduction based on noise level
        if noise_level < 0.05:
            config.noise_reduction_factor = 0.1  # Low noise
        elif noise_level < 0.2:
            config.noise_reduction_factor = 0.3  # Medium noise
        else:
            config.noise_reduction_factor = 0.5  # High noise

        # Adjust temperature based on clarity
        if noise_level < 0.1:
            config.temperature = 0.0  # Clear audio
        elif noise_level < 0.3:
            config.temperature = 0.2  # Some noise
        else:
            config.temperature = 0.4  # Noisy audio

        return config

    def evaluate_config_performance(self, audio_data: bytes, config: WhisperConfig) -> Dict:
        """Evaluate performance of a specific configuration"""
        start_time = time.time()

        try:
            import io
            from pydub import AudioSegment

            # Apply preprocessing based on config
            processed_audio = self.preprocess_audio(audio_data, config)

            # Convert to AudioSegment
            audio_segment = AudioSegment(
                data=processed_audio,
                sample_width=2,  # 16-bit
                frame_rate=44100,
                channels=1
            )

            wav_buffer = io.BytesIO()
            audio_segment.export(wav_buffer, format="wav")
            wav_buffer.seek(0)

            # Call Whisper API
            response = openai.Audio.transcribe(
                model=config.model,
                file=wav_buffer,
                language=config.language,
                response_format="verbose_json",
                temperature=config.temperature
            )

            end_time = time.time()
            processing_time = end_time - start_time

            result = {
                'config': config,
                'processing_time': processing_time,
                'text': response.get('text', ''),
                'segments': response.get('segments', []),
                'success': True,
                'error': None,
                'audio_quality_score': self.estimate_audio_quality(processed_audio)
            }

            return result

        except Exception as e:
            end_time = time.time()
            processing_time = end_time - start_time

            result = {
                'config': config,
                'processing_time': processing_time,
                'text': '',
                'segments': [],
                'success': False,
                'error': str(e),
                'audio_quality_score': 0.0
            }

            return result

    def preprocess_audio(self, audio_data: bytes, config: WhisperConfig) -> bytes:
        """Apply preprocessing based on configuration"""
        if not config.preprocessing_enabled:
            return audio_data

        # This is a simplified preprocessing function
        # In a real implementation, this would include:
        # - Noise reduction
        # - Voice activity detection
        # - Audio normalization
        # - Format conversion

        # For simulation, return the original data
        return audio_data

    def estimate_audio_quality(self, audio_data: bytes) -> float:
        """Estimate audio quality score (0.0 to 1.0)"""
        # This is a simplified quality estimation
        # In a real implementation, this would analyze:
        # - Signal-to-noise ratio
        # - Clarity metrics
        # - Distortion levels

        # For simulation, return a random score
        import random
        return random.uniform(0.5, 1.0)

    def find_optimal_config(self, audio_samples: List[bytes],
                           target_latency: float = 5.0,
                           min_quality: float = 0.7) -> Tuple[WhisperConfig, Dict]:
        """Find optimal configuration for given audio samples"""
        best_config = None
        best_performance = {}
        best_score = 0.0

        # Test different configurations
        configs_to_test = []

        # Generate different configurations
        for vad_thresh in [0.1, 0.2, 0.3, 0.4, 0.5]:
            for noise_factor in [0.1, 0.2, 0.3, 0.4, 0.5]:
                for temp in [0.0, 0.1, 0.2, 0.3]:
                    config = WhisperConfig()
                    config.vad_threshold = vad_thresh
                    config.noise_reduction_factor = noise_factor
                    config.temperature = temp
                    configs_to_test.append(config)

        print(f"Testing {len(configs_to_test)} configurations...")

        for i, config in enumerate(configs_to_test):
            total_latency = 0
            total_quality = 0
            success_count = 0

            for audio_sample in audio_samples:
                result = self.evaluate_config_performance(audio_sample, config)

                if result['success']:
                    total_latency += result['processing_time']
                    total_quality += result['audio_quality_score']
                    success_count += 1

            if success_count > 0:
                avg_latency = total_latency / success_count
                avg_quality = total_quality / success_count

                # Calculate score based on latency and quality
                latency_penalty = max(0, avg_latency - target_latency) / target_latency
                quality_bonus = avg_quality
                score = quality_bonus - latency_penalty

                if score > best_score and avg_quality >= min_quality:
                    best_score = score
                    best_config = config
                    best_performance = {
                        'avg_latency': avg_latency,
                        'avg_quality': avg_quality,
                        'success_rate': success_count / len(audio_samples),
                        'score': score
                    }

            if (i + 1) % 10 == 0:
                print(f"Tested {i + 1}/{len(configs_to_test)} configurations")

        return best_config, best_performance

# Configuration profiles for different scenarios
WHISPER_PROFILES = {
    'real_time': WhisperConfig(
        model="whisper-1",
        temperature=0.0,
        response_format="text",
        timeout=10,
        max_audio_duration=10,
        vad_threshold=0.3,
        noise_reduction_factor=0.3
    ),
    'accuracy': WhisperConfig(
        model="whisper-1",
        temperature=0.2,
        response_format="verbose_json",
        timeout=30,
        max_audio_duration=30,
        vad_threshold=0.4,
        noise_reduction_factor=0.4
    ),
    'low_latency': WhisperConfig(
        model="whisper-1",
        temperature=0.0,
        response_format="text",
        timeout=5,
        max_audio_duration=5,
        vad_threshold=0.5,
        noise_reduction_factor=0.2
    ),
    'noisy_environment': WhisperConfig(
        model="whisper-1",
        temperature=0.3,
        response_format="verbose_json",
        timeout=20,
        max_audio_duration=20,
        vad_threshold=0.6,
        noise_reduction_factor=0.5
    )
}

def main():
    """Example usage of the Whisper optimizer"""
    import os

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Please set OPENAI_API_KEY environment variable")
        return

    optimizer = WhisperPerformanceOptimizer(api_key)

    # Example: Get optimized config for different environments
    for env_type in ["quiet_office", "noisy_office", "industrial", "outdoor"]:
        config = optimizer.optimize_for_environment(env_type)
        print(f"\nOptimized config for {env_type}:")
        print(f"  VAD threshold: {config.vad_threshold}")
        print(f"  Noise reduction: {config.noise_reduction_factor}")
        print(f"  Temperature: {config.temperature}")

    # Example: Get predefined profile
    real_time_config = WHISPER_PROFILES['real_time']
    print(f"\nReal-time profile config:")
    print(f"  Model: {real_time_config.model}")
    print(f"  Timeout: {real_time_config.timeout}s")
    print(f"  VAD threshold: {real_time_config.vad_threshold}")

if __name__ == "__main__":
    main()
```

### Step 2: Create Configuration Testing Script

Create a file `test_whisper_config.py`:

```python
#!/usr/bin/env python3
"""
Whisper configuration testing script
Tests different configurations and measures performance
"""
import time
import json
import os
from typing import List, Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt

def create_test_audio(duration: float = 2.0, sample_rate: int = 44100) -> bytes:
    """Create test audio data for configuration testing"""
    # This creates a simple sine wave as test audio
    # In a real implementation, you'd use actual speech recordings

    t = np.linspace(0, duration, int(sample_rate * duration))
    # Create a combination of sine waves to simulate speech-like frequencies
    audio_data = (
        0.3 * np.sin(2 * np.pi * 440 * t) +  # A note
        0.2 * np.sin(2 * np.pi * 660 * t) +  # C note
        0.1 * np.sin(2 * np.pi * 880 * t)    # E note
    )

    # Add some noise to make it more realistic
    noise = np.random.normal(0, 0.1, len(audio_data))
    audio_data = audio_data + noise

    # Normalize to prevent clipping
    audio_data = audio_data / np.max(np.abs(audio_data))

    # Convert to 16-bit integers
    audio_data = (audio_data * 32767).astype(np.int16)

    return audio_data.tobytes()

def test_configuration_performance(optimizer, config, audio_data: bytes, test_name: str = "") -> Dict:
    """Test performance of a specific configuration"""
    print(f"Testing configuration: {test_name}")

    start_time = time.time()
    result = optimizer.evaluate_config_performance(audio_data, config)
    end_time = time.time()

    total_time = end_time - start_time

    performance = {
        'test_name': test_name,
        'config': config,
        'processing_time': result['processing_time'],
        'total_time': total_time,
        'success': result['success'],
        'quality_score': result['audio_quality_score'],
        'text_length': len(result['text']) if result['text'] else 0,
        'error': result['error']
    }

    return performance

def run_comprehensive_config_tests():
    """Run comprehensive configuration tests"""
    print("Starting Whisper Configuration Tests")
    print("=" * 50)

    # Get API key
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Please set OPENAI_API_KEY environment variable")
        return

    # Import optimizer
    try:
        from whisper_configurator import WhisperPerformanceOptimizer, WHISPER_PROFILES
        optimizer = WhisperPerformanceOptimizer(api_key)
    except ImportError:
        print("Whisper configurator not found")
        return

    # Create test audio
    print("Creating test audio...")
    test_audio = create_test_audio(duration=3.0)  # 3 seconds of test audio
    print(f"Test audio created: {len(test_audio)} bytes")

    # Test predefined profiles
    profile_results = []
    for profile_name, config in WHISPER_PROFILES.items():
        result = test_configuration_performance(
            optimizer, config, test_audio, f"Profile: {profile_name}"
        )
        profile_results.append(result)

    # Test custom configurations
    custom_configs = []

    # Test different VAD thresholds
    for threshold in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
        config = optimizer.config
        config.vad_threshold = threshold
        custom_configs.append((f"VAD {threshold}", config))

    # Test different noise reduction factors
    for factor in [0.1, 0.2, 0.3, 0.4, 0.5]:
        config = optimizer.config
        config.noise_reduction_factor = factor
        custom_configs.append((f"NR {factor}", config))

    # Test different temperatures
    for temp in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]:
        config = optimizer.config
        config.temperature = temp
        custom_configs.append((f"Temp {temp}", config))

    custom_results = []
    for name, config in custom_configs:
        result = test_configuration_performance(optimizer, config, test_audio, f"Custom: {name}")
        custom_results.append(result)

    # Combine all results
    all_results = profile_results + custom_results

    # Generate report
    generate_config_report(all_results)

    return all_results

def generate_config_report(results: List[Dict]):
    """Generate a report from configuration test results"""
    print("\n" + "=" * 50)
    print("WHISPER CONFIGURATION TEST REPORT")
    print("=" * 50)

    successful_results = [r for r in results if r['success']]
    failed_results = [r for r in results if not r['success']]

    print(f"Total tests: {len(results)}")
    print(f"Successful: {len(successful_results)}")
    print(f"Failed: {len(failed_results)}")

    if successful_results:
        # Calculate statistics
        processing_times = [r['processing_time'] for r in successful_results]
        quality_scores = [r['quality_score'] for r in successful_results]
        text_lengths = [r['text_length'] for r in successful_results]

        print(f"\nProcessing Time Statistics:")
        print(f"  Average: {np.mean(processing_times):.3f}s")
        print(f"  Min: {np.min(processing_times):.3f}s")
        print(f"  Max: {np.max(processing_times):.3f}s")
        print(f"  Std Dev: {np.std(processing_times):.3f}s")

        print(f"\nQuality Score Statistics:")
        print(f"  Average: {np.mean(quality_scores):.3f}")
        print(f"  Min: {np.min(quality_scores):.3f}")
        print(f"  Max: {np.max(quality_scores):.3f}")

        print(f"\nText Length Statistics:")
        print(f"  Average: {np.mean(text_lengths):.1f} characters")
        print(f"  Min: {np.min(text_lengths)} characters")
        print(f"  Max: {np.max(text_lengths)} characters")

        # Find best configurations by different metrics
        best_by_time = min(successful_results, key=lambda r: r['processing_time'])
        best_by_quality = max(successful_results, key=lambda r: r['quality_score'])
        best_by_balance = max(successful_results,
                             key=lambda r: r['quality_score'] / (r['processing_time'] + 0.1))

        print(f"\nBest by Processing Time:")
        print(f"  {best_by_time['test_name']}: {best_by_time['processing_time']:.3f}s")

        print(f"\nBest by Quality Score:")
        print(f"  {best_by_quality['test_name']}: {best_by_quality['quality_score']:.3f}")

        print(f"\nBest Balanced (Quality/Time):")
        print(f"  {best_by_balance['test_name']}: {best_by_balance['quality_score']:.3f}/{best_by_balance['processing_time']:.3f}s")

    if failed_results:
        print(f"\nFailed Tests:")
        for r in failed_results:
            print(f"  {r['test_name']}: {r['error']}")

    # Generate visualizations
    create_config_charts(successful_results)

def create_config_charts(results: List[Dict]):
    """Create charts to visualize configuration performance"""
    try:
        import matplotlib.pyplot as plt

        if not results:
            print("No successful results to chart")
            return

        # Extract data
        test_names = [r['test_name'] for r in results]
        processing_times = [r['processing_time'] for r in results]
        quality_scores = [r['quality_score'] for r in results]

        # Create subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        # Processing time by configuration
        axes[0, 0].bar(range(len(test_names)), processing_times)
        axes[0, 0].set_title('Processing Time by Configuration')
        axes[0, 0].set_xlabel('Configuration')
        axes[0, 0].set_ylabel('Processing Time (s)')
        axes[0, 0].tick_params(axis='x', rotation=45, labelsize=8)

        # Quality score by configuration
        axes[0, 1].bar(range(len(test_names)), quality_scores)
        axes[0, 1].set_title('Quality Score by Configuration')
        axes[0, 1].set_xlabel('Configuration')
        axes[0, 1].set_ylabel('Quality Score')
        axes[0, 1].tick_params(axis='x', rotation=45, labelsize=8)

        # Scatter plot: time vs quality
        axes[1, 0].scatter(processing_times, quality_scores, alpha=0.7)
        axes[1, 0].set_title('Processing Time vs Quality Score')
        axes[1, 0].set_xlabel('Processing Time (s)')
        axes[1, 0].set_ylabel('Quality Score')

        # Annotate points
        for i, txt in enumerate(test_names):
            if i < len(processing_times) and i < len(quality_scores):
                axes[1, 0].annotate(txt.split(': ')[-1][:10],
                                   (processing_times[i], quality_scores[i]),
                                   fontsize=6, alpha=0.7)

        # Performance efficiency (quality/time ratio)
        efficiency = [q/(t+0.001) for q, t in zip(quality_scores, processing_times)]  # Add small value to avoid division by zero
        axes[1, 1].bar(range(len(test_names)), efficiency)
        axes[1, 1].set_title('Performance Efficiency (Quality/Time)')
        axes[1, 1].set_xlabel('Configuration')
        axes[1, 1].set_ylabel('Efficiency (Q/T)')
        axes[1, 1].tick_params(axis='x', rotation=45, labelsize=8)

        plt.tight_layout()
        plt.savefig('whisper_config_performance.png', dpi=300, bbox_inches='tight')
        print("Configuration performance chart saved as 'whisper_config_performance.png'")

    except ImportError:
        print("Matplotlib not available, skipping chart generation")

def run_environment_optimization_test():
    """Test environment-specific optimizations"""
    print("\nRunning Environment Optimization Tests")
    print("=" * 50)

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Please set OPENAI_API_KEY environment variable")
        return

    try:
        from whisper_configurator import WhisperPerformanceOptimizer
        optimizer = WhisperPerformanceOptimizer(api_key)
    except ImportError:
        print("Whisper configurator not found")
        return

    # Test environment-specific configurations
    environments = ["quiet_office", "noisy_office", "industrial", "outdoor"]
    test_audio = create_test_audio(duration=2.0)

    results = []

    for env in environments:
        print(f"\nTesting environment: {env}")

        # Get optimized config for environment
        config = optimizer.optimize_for_environment(env)

        # Test the configuration
        result = test_configuration_performance(
            optimizer, config, test_audio, f"Env: {env}"
        )
        results.append(result)

        print(f"  VAD Threshold: {config.vad_threshold}")
        print(f"  Noise Reduction: {config.noise_reduction_factor}")
        print(f"  Temperature: {config.temperature}")
        print(f"  Processing Time: {result['processing_time']:.3f}s")
        print(f"  Quality Score: {result['quality_score']:.3f}")

    # Compare results
    print("\nEnvironment Comparison:")
    print("-" * 40)
    for r in results:
        env_name = r['test_name'].split(': ')[1]
        print(f"{env_name:12} | Time: {r['processing_time']:.3f}s | Quality: {r['quality_score']:.3f}")

    return results

def run_adaptive_config_test():
    """Test adaptive configuration based on audio characteristics"""
    print("\nRunning Adaptive Configuration Tests")
    print("=" * 50)

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Please set OPENAI_API_KEY environment variable")
        return

    try:
        from whisper_configurator import WhisperPerformanceOptimizer
        optimizer = WhisperPerformanceOptimizer(api_key)
    except ImportError:
        print("Whisper configurator not found")
        return

    # Test with different audio characteristics
    test_cases = [
        (0.005, 0.001, "Very quiet, low noise"),
        (0.05, 0.01, "Quiet, low noise"),
        (0.2, 0.05, "Normal, medium noise"),
        (0.6, 0.1, "Loud, medium noise"),
        (0.8, 0.3, "Very loud, high noise")
    ]

    results = []

    for audio_level, noise_level, description in test_cases:
        print(f"\nTesting: {description}")
        print(f"  Audio level: {audio_level}, Noise level: {noise_level}")

        # Get adaptive config
        config = optimizer.adaptive_config(audio_level, noise_level)

        # Create test audio with some variation
        test_audio = create_test_audio(duration=1.5)

        # Test the configuration
        result = test_configuration_performance(
            optimizer, config, test_audio, f"Adaptive: {description}"
        )
        results.append(result)

        print(f"  Applied config - VAD: {config.vad_threshold}, NR: {config.noise_reduction_factor}, Temp: {config.temperature}")
        print(f"  Result - Time: {result['processing_time']:.3f}s, Quality: {result['quality_score']:.3f}")

    return results

def main():
    """Main function for configuration testing"""
    print("Whisper Configuration and Optimization Tests")
    print("Choose test type:")
    print("1. Comprehensive configuration tests")
    print("2. Environment optimization tests")
    print("3. Adaptive configuration tests")
    print("4. All tests")

    choice = input("Enter choice (1-4): ").strip()

    if choice == "1":
        run_comprehensive_config_tests()
    elif choice == "2":
        run_environment_optimization_test()
    elif choice == "3":
        run_adaptive_config_test()
    elif choice == "4":
        run_comprehensive_config_tests()
        run_environment_optimization_test()
        run_adaptive_config_test()
    else:
        print("Invalid choice. Running comprehensive tests...")
        run_comprehensive_config_tests()

if __name__ == "__main__":
    main()
```

### Step 3: Create Advanced Configuration Script

Create a file `advanced_whisper_config.py`:

```python
#!/usr/bin/env python3
"""
Advanced Whisper configuration with real-time optimization
"""
import openai
import time
import threading
import queue
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
import numpy as np
import json

@dataclass
class RealTimeConfig:
    """Real-time configuration for Whisper optimization"""
    # Audio input parameters
    sample_rate: int = 44100
    channels: int = 1
    chunk_size: int = 1024

    # VAD (Voice Activity Detection) parameters
    vad_threshold: float = 0.3
    vad_window_size: int = 1024  # samples
    vad_hangover: int = 5  # frames after voice stops

    # Noise reduction parameters
    noise_reduction_enabled: bool = True
    noise_reduction_factor: float = 0.3
    noise_estimation_frames: int = 100

    # Whisper API parameters
    model: str = "whisper-1"
    language: str = "en"
    temperature: float = 0.0
    timeout: int = 30
    response_format: str = "text"

    # Performance parameters
    min_speech_duration: float = 0.5  # seconds
    max_speech_duration: float = 10.0  # seconds
    silence_threshold: float = 0.01
    processing_interval: float = 0.1  # seconds

    # Adaptive parameters
    enable_adaptive: bool = True
    target_latency: float = 5.0  # seconds
    min_quality_threshold: float = 0.7

class RealTimeWhisperOptimizer:
    """Real-time Whisper optimizer that adapts configuration based on performance"""

    def __init__(self, api_key: str, config: RealTimeConfig = None):
        openai.api_key = api_key
        self.config = config or RealTimeConfig()
        self.audio_buffer = np.array([])
        self.speech_buffer = np.array([])
        self.is_speech_active = False
        self.silence_counter = 0
        self.noise_profile = None
        self.frame_counter = 0
        self.performance_history = []

        # Stats for adaptive configuration
        self.avg_latency = 0.0
        self.avg_quality = 0.0
        self.success_count = 0

        # Threading for real-time processing
        self.processing_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.is_running = True
        self.processing_thread = threading.Thread(target=self._process_audio_stream)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def add_audio_chunk(self, audio_chunk: np.ndarray):
        """Add an audio chunk to the processing buffer"""
        self.processing_queue.put(audio_chunk)

    def get_result(self, timeout: float = 1.0) -> Optional[Dict]:
        """Get the next recognition result"""
        try:
            return self.result_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _process_audio_stream(self):
        """Process audio stream in real-time"""
        while self.is_running:
            try:
                audio_chunk = self.processing_queue.get(timeout=0.1)

                # Apply preprocessing
                processed_chunk = self._preprocess_audio(audio_chunk)

                # Detect voice activity
                voice_active = self._detect_voice_activity(processed_chunk)

                if voice_active:
                    # Add to speech buffer
                    self.speech_buffer = np.concatenate([self.speech_buffer, processed_chunk])

                    # Check if speech segment is long enough
                    speech_duration = len(self.speech_buffer) / self.config.sample_rate
                    if speech_duration >= self.config.min_speech_duration:
                        # Process the speech segment
                        self._process_speech_segment()
                else:
                    # Reset speech buffer if we've had silence for a while
                    self.silence_counter += 1
                    if self.silence_counter > self.config.vad_hangover:
                        if len(self.speech_buffer) > 0:
                            # Process any remaining speech
                            self._process_speech_segment()
                        self.speech_buffer = np.array([])
                        self.silence_counter = 0

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in audio processing: {e}")

    def _preprocess_audio(self, audio_chunk: np.ndarray) -> np.ndarray:
        """Apply preprocessing to audio chunk"""
        if self.config.noise_reduction_enabled:
            # Apply noise reduction if noise profile is available
            if self.noise_profile is not None:
                # Simplified noise reduction
                chunk_fft = np.fft.fft(audio_chunk)
                chunk_magnitude = np.abs(chunk_fft)
                chunk_phase = np.angle(chunk_fft)

                # Apply noise reduction
                reduced_magnitude = np.maximum(
                    chunk_magnitude - self.noise_profile * self.config.noise_reduction_factor,
                    0
                )

                # Reconstruct
                reduced_fft = reduced_magnitude * np.exp(1j * chunk_phase)
                audio_chunk = np.real(np.fft.ifft(reduced_fft)).astype(audio_chunk.dtype)

        return audio_chunk

    def _detect_voice_activity(self, audio_chunk: np.ndarray) -> bool:
        """Detect voice activity in audio chunk"""
        # Calculate energy
        energy = np.mean(audio_chunk ** 2)

        # Update noise profile (first few frames are assumed to be noise)
        if self.frame_counter < self.config.noise_estimation_frames:
            if self.noise_profile is None:
                self.noise_profile = energy
            else:
                # Exponential moving average
                self.noise_profile = 0.9 * self.noise_profile + 0.1 * energy

        self.frame_counter += 1

        # Voice activity if energy is above threshold
        return energy > self.config.vad_threshold

    def _process_speech_segment(self):
        """Process a complete speech segment"""
        if len(self.speech_buffer) == 0:
            return

        # Convert to bytes for Whisper API
        audio_bytes = self.speech_buffer.astype(np.int16).tobytes()

        # Transcribe with current configuration
        result = self._transcribe_with_config(audio_bytes)

        if result['success']:
            # Update performance statistics
            self._update_performance_stats(result)

            # Apply adaptive configuration if enabled
            if self.config.enable_adaptive:
                self._adjust_config_adaptively()

            # Put result in queue
            self.result_queue.put(result)

        # Clear speech buffer
        self.speech_buffer = np.array([])

    def _transcribe_with_config(self, audio_bytes: bytes) -> Dict:
        """Transcribe audio with current configuration"""
        start_time = time.time()

        try:
            import io
            from pydub import AudioSegment

            # Convert bytes to AudioSegment
            audio_segment = AudioSegment(
                data=audio_bytes,
                sample_width=2,  # 16-bit
                frame_rate=self.config.sample_rate,
                channels=self.config.channels
            )

            wav_buffer = io.BytesIO()
            audio_segment.export(wav_buffer, format="wav")
            wav_buffer.seek(0)

            # Call Whisper API
            response = openai.Audio.transcribe(
                model=self.config.model,
                file=wav_buffer,
                language=self.config.language,
                response_format=self.config.response_format,
                temperature=self.config.temperature
            )

            end_time = time.time()
            processing_time = end_time - start_time

            result = {
                'text': response if isinstance(response, str) else response.get('text', ''),
                'processing_time': processing_time,
                'config_used': self.config,
                'success': True,
                'error': None,
                'quality_estimate': self._estimate_quality(response)
            }

            return result

        except Exception as e:
            end_time = time.time()
            processing_time = end_time - start_time

            result = {
                'text': '',
                'processing_time': processing_time,
                'config_used': self.config,
                'success': False,
                'error': str(e),
                'quality_estimate': 0.0
            }

            return result

    def _estimate_quality(self, transcription: str) -> float:
        """Estimate quality of transcription"""
        if not transcription or not transcription.strip():
            return 0.0

        # Simple quality metrics
        text = transcription.strip().lower()

        # Length-based quality (longer text might be better)
        length_quality = min(1.0, len(text) / 100.0)

        # Check for common filler words that might indicate poor quality
        filler_words = ['um', 'uh', 'er', 'ah']
        filler_count = sum(1 for word in filler_words if word in text.split())
        filler_quality = max(0.0, 1.0 - (filler_count / max(1, len(text.split()))))

        # Combine metrics
        quality = 0.6 * length_quality + 0.4 * filler_quality
        return min(1.0, quality)

    def _update_performance_stats(self, result: Dict):
        """Update performance statistics"""
        if result['success']:
            # Update running averages
            self.success_count += 1
            self.avg_latency = (self.avg_latency * (self.success_count - 1) + result['processing_time']) / self.success_count
            self.avg_quality = (self.avg_quality * (self.success_count - 1) + result['quality_estimate']) / self.success_count

            # Store in history
            self.performance_history.append({
                'timestamp': time.time(),
                'latency': result['processing_time'],
                'quality': result['quality_estimate'],
                'config': self.config
            })

    def _adjust_config_adaptively(self):
        """Adjust configuration based on performance metrics"""
        if self.success_count == 0:
            return

        # Adjust based on latency vs target
        if self.avg_latency > self.config.target_latency:
            # If too slow, reduce quality requirements
            self.config.temperature = min(0.5, self.config.temperature + 0.1)
        else:
            # If fast enough, try to improve quality
            self.config.temperature = max(0.0, self.config.temperature - 0.05)

        # Adjust based on quality vs threshold
        if self.avg_quality < self.config.min_quality_threshold:
            # If quality too low, increase noise reduction
            self.config.noise_reduction_factor = min(0.7, self.config.noise_reduction_factor + 0.05)
            # Maybe try a different approach
            self.config.vad_threshold = max(0.1, self.config.vad_threshold - 0.05)
        else:
            # If quality is good, maybe reduce processing
            self.config.noise_reduction_factor = max(0.1, self.config.noise_reduction_factor - 0.02)

    def get_current_performance_metrics(self) -> Dict:
        """Get current performance metrics"""
        return {
            'avg_latency': self.avg_latency,
            'avg_quality': self.avg_quality,
            'success_count': self.success_count,
            'total_processed': len(self.performance_history),
            'current_config': self.config
        }

    def stop(self):
        """Stop the optimizer"""
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)

def main():
    """Example usage of real-time optimizer"""
    import os

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Please set OPENAI_API_KEY environment variable")
        return

    # Create optimizer with default config
    optimizer = RealTimeWhisperOptimizer(api_key)

    print("Real-time Whisper Optimizer started")
    print("Simulating audio input for 10 seconds...")

    # Simulate audio input (in real usage, this would come from microphone)
    import numpy as np

    sample_rate = 44100
    chunk_size = 1024
    duration = 10  # seconds

    for i in range(0, int(sample_rate * duration), chunk_size):
        # Create simulated audio chunk (in real usage, this would be from microphone)
        t = np.linspace(0, chunk_size/sample_rate, chunk_size)
        # Simulate some speech-like audio with occasional pauses
        if (i // chunk_size) % 8 < 6:  # Speak for 6 out of 8 chunks
            audio_chunk = (
                0.3 * np.sin(2 * np.pi * 440 * t) +  # Base frequency
                0.2 * np.sin(2 * np.pi * 660 * t) +  # Harmonic
                0.1 * np.random.normal(0, 0.1, len(t))  # Noise
            )
        else:
            # Silence
            audio_chunk = np.random.normal(0, 0.01, len(t))  # Just noise

        # Add to optimizer
        optimizer.add_audio_chunk(audio_chunk.astype(np.float32))

        # Check for results occasionally
        result = optimizer.get_result(timeout=0.01)
        if result and result['success']:
            print(f"Transcribed: '{result['text']}' (Latency: {result['processing_time']:.3f}s, Quality: {result['quality_estimate']:.3f})")

        # Small delay to simulate real-time processing
        time.sleep(0.001)

    # Get final metrics
    metrics = optimizer.get_current_performance_metrics()
    print(f"\nFinal Performance Metrics:")
    print(f"  Average Latency: {metrics['avg_latency']:.3f}s")
    print(f"  Average Quality: {metrics['avg_quality']:.3f}")
    print(f"  Successful Transcriptions: {metrics['success_count']}")

    # Stop optimizer
    optimizer.stop()

if __name__ == "__main__":
    main()
```

## Expected Outcomes

After completing this exercise, you should:

1. Understand different Whisper configuration parameters and their effects
2. Test various configuration profiles for different environments
3. Optimize configurations for specific performance requirements
4. Implement adaptive configuration that adjusts based on real-time performance
5. Measure and compare performance across different configurations

## Verification Steps

1. Test different Whisper models and parameters
2. Verify that environment-specific configurations work as expected
3. Confirm that adaptive configuration improves performance
4. Measure latency, quality, and resource usage
5. Validate that configurations meet target requirements

## Troubleshooting

- If API calls fail, verify your OpenAI API key is valid and has sufficient quota
- If configurations don't improve performance, check that preprocessing is working correctly
- If real-time processing is too slow, consider reducing audio quality requirements
- If quality scores are inaccurate, refine the quality estimation algorithm