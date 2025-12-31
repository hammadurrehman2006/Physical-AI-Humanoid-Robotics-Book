---
sidebar_position: 2
title: "ورکشاپ 4.2: آڈیو پری پروسیسنگ"
---

# ورکشاپ 4.2: آڈیو پری پروسیسنگ

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے آڈیو ڈیٹا کو پری پروسیس کرنا اور بہتر بنانا۔

## ضروریات
- Python 3.10+
- NumPy
- SciPy
- Librosa
- PyAudio
- ROS 2 ہمبل ہاکسبل

## ورکشاپ کے اقدامات

### اقدام 1: آڈیو پری پروسیسنگ کو سمجھنا
ایک نیا فائل `audio_preprocessing.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import numpy as np
import librosa
import scipy.signal as signal
from scipy import fft
from typing import Dict, List, Tuple, Optional
import soundfile as sf

class AudioPreprocessor:
    def __init__(self):
        self.sample_rate = 16000
        self.target_db = -20.0  # نارملائزیشن کے لیے ٹارگیٹ dB
        self.noise_threshold = 0.01  # نوائز ڈیٹیکشن کے لیے

    def load_audio(self, file_path: str) -> np.ndarray:
        """آڈیو فائل لوڈ کریں اور 16kHz پر ری سیمپل کریں"""
        audio, sr = librosa.load(file_path, sr=self.sample_rate)
        return audio

    def normalize_audio(self, audio: np.ndarray) -> np.ndarray:
        """آڈیو کو نارملائز کریں"""
        # RMS نارملائزیشن
        rms = np.sqrt(np.mean(audio**2))
        if rms == 0:
            return audio

        target_rms = 10**(self.target_db / 20.0)
        gain = target_rms / rms
        normalized_audio = audio * gain

        # کلپ نہ ہونے دیں
        normalized_audio = np.clip(normalized_audio, -1.0, 1.0)

        return normalized_audio

    def remove_dc_offset(self, audio: np.ndarray) -> np.ndarray:
        """DC آف سیٹ ہٹائیں"""
        return signal.detrend(audio, type='constant')

    def apply_high_pass_filter(self, audio: np.ndarray, cutoff_freq: float = 100.0) -> np.ndarray:
        """ہائی پاس فلٹر لاگو کریں"""
        nyquist = self.sample_rate / 2.0
        normalized_cutoff = cutoff_freq / nyquist

        # Butterworth فلٹر
        b, a = signal.butter(4, normalized_cutoff, btype='high', analog=False)
        filtered_audio = signal.filtfilt(b, a, audio)

        return filtered_audio

    def apply_low_pass_filter(self, audio: np.ndarray, cutoff_freq: float = 4000.0) -> np.ndarray:
        """لو پاس فلٹر لاگو کریں"""
        nyquist = self.sample_rate / 2.0
        normalized_cutoff = cutoff_freq / nyquist

        # Butterworth فلٹر
        b, a = signal.butter(4, normalized_cutoff, btype='low', analog=False)
        filtered_audio = signal.filtfilt(b, a, audio)

        return filtered_audio

    def noise_gate(self, audio: np.ndarray, threshold: float = 0.01, attack: float = 0.01, release: float = 0.1) -> np.ndarray:
        """نوائز گیٹ لاگو کریں"""
        # RMS ونڈو کا حساب
        frame_length = int(0.01 * self.sample_rate)  # 10ms فریم
        hop_length = frame_length // 4

        # اسپیچ/نوائز کا پتہ لگائیں
        frames = librosa.util.frame(audio, frame_length=frame_length, hop_length=hop_length)
        frame_rms = librosa.feature.rms(y=audio)[0]

        # گیٹ کے لیے ماسک
        gate_mask = frame_rms > threshold

        # انٹرپولیٹ کریں تاکہ اصل سیمپلز کے مطابق ہو
        gate_full = np.repeat(gate_mask, hop_length)[:len(audio)]
        if len(gate_full) < len(audio):
            gate_full = np.pad(gate_full, (0, len(audio) - len(gate_full)), mode='edge')

        # گیٹ لگائیں
        gated_audio = audio * gate_full

        return gated_audio

    def spectral_gate_denoise(self, audio: np.ndarray, n_fft: int = 1024, hop_length: int = 256) -> np.ndarray:
        """اسپیکٹرل گیٹ نوائز ریموال"""
        # STFT
        stft = librosa.stft(audio, n_fft=n_fft, hop_length=hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # نوائز کا تخمینہ (پہلے 0.5s کو نوائز سمجھیں)
        noise_frames = int(0.5 * self.sample_rate / hop_length)
        if noise_frames > magnitude.shape[1]:
            noise_frames = magnitude.shape[1]

        noise_profile = np.mean(magnitude[:, :noise_frames], axis=1, keepdims=True)

        # گیٹنگ تھریشولڈ
        threshold = noise_profile * 1.5  # 1.5x نوائز لیول

        # گیٹ لگائیں
        magnitude_gated = np.where(magnitude > threshold, magnitude, 0)

        # ISTFT
        stft_gated = magnitude_gated * np.exp(1j * phase)
        denoised_audio = librosa.istft(stft_gated, hop_length=hop_length, length=len(audio))

        return denoised_audio

    def vad_segmentation(self, audio: np.ndarray, threshold: float = 0.02) -> List[Dict]:
        """ووائس ایکٹیویٹی ڈیٹیکشن"""
        # ہر 10ms کے لیے RMS حساب کریں
        frame_length = int(0.01 * self.sample_rate)
        hop_length = frame_length // 2

        frames = librosa.util.frame(audio, frame_length=frame_length, hop_length=hop_length)
        frame_rms = librosa.feature.rms(y=audio)[0]

        # ووائس ایکٹیویٹی کا پتہ لگائیں
        voice_activity = frame_rms > threshold

        # سیگمینٹس تیار کریں
        segments = []
        current_start = None

        for i, is_voice in enumerate(voice_activity):
            time_start = i * hop_length / self.sample_rate

            if is_voice and current_start is None:
                # نیا سیگمینٹ شروع کریں
                current_start = time_start
            elif not is_voice and current_start is not None:
                # سیگمینٹ ختم کریں
                segments.append({
                    'start': current_start,
                    'end': time_start,
                    'duration': time_start - current_start
                })
                current_start = None

        # آخری سیگمینٹ کے لیے
        if current_start is not None:
            segments.append({
                'start': current_start,
                'end': len(audio) / self.sample_rate,
                'duration': (len(audio) / self.sample_rate) - current_start
            })

        return segments

    def apply_preprocessing_pipeline(self, audio: np.ndarray) -> Dict:
        """مکمل پری پروسیسنگ پائپ لائن لاگو کریں"""
        results = {
            'original': audio,
            'steps': []
        }

        # 1. DC آف سیٹ ہٹائیں
        step1 = self.remove_dc_offset(audio)
        results['steps'].append(('remove_dc_offset', step1))

        # 2. ہائی پاس فلٹر
        step2 = self.apply_high_pass_filter(step1)
        results['steps'].append(('high_pass_filter', step2))

        # 3. نوائز گیٹ
        step3 = self.noise_gate(step2)
        results['steps'].append(('noise_gate', step3))

        # 4. اسپیکٹرل گیٹ ڈی نوائز
        step4 = self.spectral_gate_denoise(step3)
        results['steps'].append(('spectral_gate_denoise', step4))

        # 5. نارملائز کریں
        step5 = self.normalize_audio(step4)
        results['steps'].append(('normalize', step5))

        results['processed'] = step5

        return results

    def get_audio_features(self, audio: np.ndarray) -> Dict:
        """آڈیو کے فیچرز حاصل کریں"""
        features = {}

        # ایمپلی ٹیوڈ فیچرز
        features['rms'] = np.sqrt(np.mean(audio**2))
        features['max_amplitude'] = np.max(np.abs(audio))
        features['zero_crossing_rate'] = np.sum(np.diff(np.sign(audio)) != 0) / len(audio)

        # اسپیکٹرل فیچرز
        try:
            # MFCCs
            mfccs = librosa.feature.mfcc(y=audio, sr=self.sample_rate, n_mfcc=13)
            features['mfcc_mean'] = np.mean(mfccs, axis=1).tolist()
            features['mfcc_std'] = np.std(mfccs, axis=1).tolist()

            # سپیکٹرل سینٹروئڈ
            spectral_centroids = librosa.feature.spectral_centroid(y=audio, sr=self.sample_rate)[0]
            features['spectral_centroid_mean'] = np.mean(spectral_centroids)
            features['spectral_centroid_std'] = np.std(spectral_centroids)

            # سپیکٹرل بینڈوڈتھ
            spectral_bandwidth = librosa.feature.spectral_bandwidth(y=audio, sr=self.sample_rate)[0]
            features['spectral_bandwidth_mean'] = np.mean(spectral_bandwidth)
            features['spectral_bandwidth_std'] = np.std(spectral_bandwidth)

        except Exception as e:
            print(f"Feature extraction error: {e}")

        return features

def preprocess_audio_file(input_file: str, output_file: str):
    """آڈیو فائل کو پری پروسیس کریں"""
    preprocessor = AudioPreprocessor()

    # فائل لوڈ کریں
    audio = preprocessor.load_audio(input_file)

    # پری پروسیسنگ لاگو کریں
    results = preprocessor.apply_preprocessing_pipeline(audio)

    # پروسیسڈ آڈیو محفوظ کریں
    sf.write(output_file, results['processed'], preprocessor.sample_rate)

    # فیچرز حاصل کریں
    features = preprocessor.get_audio_features(results['processed'])

    print(f"Audio preprocessing completed!")
    print(f"Original duration: {len(audio) / preprocessor.sample_rate:.2f}s")
    print(f"Processed duration: {len(results['processed']) / preprocessor.sample_rate:.2f}s")
    print(f"Features extracted: {list(features.keys())}")

    return results['processed'], features

def main():
    print("Audio Preprocessing Pipeline")

    # ٹیسٹ کے لیے جنرک آڈیو ڈیٹا تیار کریں (اصل میں فائل سے لوڈ کریں)
    sample_rate = 16000
    duration = 5  # 5 سیکنڈ
    t = np.linspace(0, duration, int(sample_rate * duration))

    # جنرک ٹیسٹ آڈیو (اصل میں آپ فائل سے لوڈ کریں گے)
    test_audio = np.sin(2 * np.pi * 440 * t)  # 440 Hz ٹون
    test_audio += 0.1 * np.random.randn(len(test_audio))  # نوائز شامل کریں

    # پری پروسیسنگ کلاس
    preprocessor = AudioPreprocessor()

    # پری پروسیسنگ لاگو کریں
    results = preprocessor.apply_preprocessing_pipeline(test_audio)

    # فیچرز حاصل کریں
    features = preprocessor.get_audio_features(results['processed'])

    print("Preprocessing completed successfully!")
    print(f"Features: {features}")

if __name__ == "__main__":
    main()
```

### اقدام 2: ROS 2 انٹیگریشن کے لیے پری پروسیسنگ نوڈ
ایک فائل `audio_preprocessing_node.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import numpy as np
import json
from audio_preprocessing import AudioPreprocessor  # ہماری کسٹم کلاس
from typing import Dict, Any

class AudioPreprocessingNode(Node):
    def __init__(self):
        super().__init__('audio_preprocessing_node')

        # آڈیو پری پروسیسر
        self.preprocessor = AudioPreprocessor()

        # سبسکرائبرز
        self.audio_subscriber = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10
        )

        # پبلیشرز
        self.preprocessed_audio_publisher = self.create_publisher(
            AudioData, 'preprocessed_audio', 10
        )
        self.audio_features_publisher = self.create_publisher(
            String, 'audio_features', 10
        )
        self.preprocessing_status_publisher = self.create_publisher(
            String, 'preprocessing_status', 10
        )

        self.get_logger().info('Audio Preprocessing Node initialized')

    def audio_callback(self, msg: AudioData):
        """آڈیو ڈیٹا کو پری پروسیس کریں"""
        try:
            # NumPy ارے میں تبدیل کریں
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # پری پروسیسنگ لاگو کریں
            results = self.preprocessor.apply_preprocessing_pipeline(audio_array)

            # پروسیسڈ آڈیو کو NumPy سے bytes میں تبدیل کریں
            processed_audio_bytes = (results['processed'] * 32767).astype(np.int16).tobytes()

            # پروسیسڈ آڈیو پبلش کریں
            processed_msg = AudioData()
            processed_msg.header = msg.header  # ہیڈر کو کاپی کریں
            processed_msg.data = processed_audio_bytes
            processed_msg.encoding = msg.encoding
            processed_msg.sample_rate = msg.sample_rate
            processed_msg.channels = msg.channels

            self.preprocessed_audio_publisher.publish(processed_msg)

            # فیچرز حاصل کریں اور پبلش کریں
            features = self.preprocessor.get_audio_features(results['processed'])
            self.publish_audio_features(features, msg.header.stamp)

            # اسٹیٹس پبلش کریں
            self.publish_preprocessing_status("success", msg.header.stamp)

            self.get_logger().info(f'Audio preprocessed: {len(audio_array)} samples -> {len(results["processed"])} samples')

        except Exception as e:
            self.get_logger().error(f'Audio preprocessing error: {e}')
            self.publish_preprocessing_status(f"error: {e}", self.get_clock().now().to_msg())

    def publish_audio_features(self, features: Dict, timestamp):
        """آڈیو فیچرز پبلش کریں"""
        features_msg = String()

        features_data = {
            'features': features,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9
        }

        features_msg.data = json.dumps(features_data)
        self.audio_features_publisher.publish(features_msg)

    def publish_preprocessing_status(self, status: str, timestamp):
        """پری پروسیسنگ اسٹیٹس پبلش کریں"""
        status_msg = String()

        status_data = {
            'status': status,
            'timestamp': timestamp.sec + timestamp.nanosec / 1e9
        }

        status_msg.data = json.dumps(status_data)
        self.preprocessing_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudioPreprocessingNode()

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

### اقدام 3: ایڈوانسڈ پری پروسیسنگ اور ٹیسٹنگ
ایک فائل `advanced_preprocessing.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import numpy as np
import librosa
import scipy.signal as signal
from scipy import fft
from typing import Dict, List, Tuple, Optional
import soundfile as sf
import matplotlib.pyplot as plt

class AdvancedAudioPreprocessor:
    def __init__(self):
        self.sample_rate = 16000
        self.frame_length = 2048  # STFT کے لیے
        self.hop_length = 512     # STFT کے لیے

    def adaptive_noise_reduction(self, audio: np.ndarray, noise_threshold_db: float = -30.0) -> np.ndarray:
        """ایڈاپٹیو نوائز ریموال"""
        # STFT
        stft = librosa.stft(audio, n_fft=self.frame_length, hop_length=self.hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # dB میں تبدیل کریں
        magnitude_db = librosa.amplitude_to_db(magnitude)

        # نوائز کا تخمینہ (کم ایمپلی ٹیوڈ والے فریکوئنسیز)
        noise_estimate = np.percentile(magnitude_db, 10, axis=1, keepdims=True)

        # ماسک تیار کریں
        mask = magnitude_db > (noise_estimate + 3)  # 3dB گیپ

        # ماسک کو ایمپلی ٹیوڈ پر لاگو کریں
        magnitude_cleaned = magnitude * mask

        # ISTFT
        stft_cleaned = magnitude_cleaned * np.exp(1j * phase)
        denoised_audio = librosa.istft(stft_cleaned, hop_length=self.hop_length, length=len(audio))

        return denoised_audio

    def voice_activity_detection(self, audio: np.ndarray, method: str = 'energy') -> List[bool]:
        """ووائس ایکٹیویٹی ڈیٹیکشن"""
        if method == 'energy':
            # اینرژی بیسڈ VAD
            frame_length = int(0.025 * self.sample_rate)  # 25ms فریم
            hop_length = int(0.01 * self.sample_rate)     # 10ms ہاپ

            frames = librosa.util.frame(audio, frame_length=frame_length, hop_length=hop_length)
            frame_energy = np.mean(frames**2, axis=0)

            # ایڈاپٹیو تھریشولڈ
            threshold = np.mean(frame_energy) * 0.1

            vad_result = frame_energy > threshold

        elif method == 'spectral':
            # سپیکٹرل بیسڈ VAD
            # زیرو کراسنگ ریٹ اور سپیکٹرل فلیٹنس کا استعمال
            frame_length = int(0.025 * self.sample_rate)
            hop_length = int(0.01 * self.sample_rate)

            zcr = librosa.feature.zero_crossing_rate(audio, frame_length=frame_length, hop_length=hop_length)[0]
            spectral_flatness = librosa.feature.spectral_flatness(y=audio, n_fft=frame_length, hop_length=hop_length)[0]

            # نارملائز کریں
            zcr_norm = (zcr - np.mean(zcr)) / (np.std(zcr) + 1e-8)
            sf_norm = (spectral_flatness - np.mean(spectral_flatness)) / (np.std(spectral_flatness) + 1e-8)

            # کمبائنڈ فیچر
            combined = zcr_norm + sf_norm
            threshold = np.mean(combined) * 0.5

            vad_result = combined > threshold

        else:
            raise ValueError(f"Unknown VAD method: {method}")

        return vad_result.tolist()

    def spectral_subtraction(self, audio: np.ndarray, alpha: float = 1.0, beta: float = 0.002) -> np.ndarray:
        """اسپیکٹرل سب ٹریکشن برائے نوائز ریموال"""
        # STFT
        stft = librosa.stft(audio, n_fft=self.frame_length, hop_length=self.hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # نوائز کا تخمینہ (پہلے 0.5s کو نوائز سمجھیں)
        noise_frames = int(0.5 * self.sample_rate / self.hop_length)
        if noise_frames > magnitude.shape[1]:
            noise_frames = magnitude.shape[1]

        noise_profile = np.mean(magnitude[:, :noise_frames], axis=1, keepdims=True)

        # اسپیکٹرل سب ٹریکشن فارمولہ
        magnitude_enhanced = np.maximum(magnitude - alpha * noise_profile, beta * noise_profile)

        # ISTFT
        stft_enhanced = magnitude_enhanced * np.exp(1j * phase)
        enhanced_audio = librosa.istft(stft_enhanced, hop_length=self.hop_length, length=len(audio))

        return enhanced_audio

    def automatic_gain_control(self, audio: np.ndarray, target_level: float = 0.5, attack: float = 0.01, release: float = 0.1) -> np.ndarray:
        """آٹومیٹک گین کنٹرول"""
        # RMS کا حساب
        frame_length = int(0.01 * self.sample_rate)  # 10ms
        hop_length = frame_length // 4

        frames = librosa.util.frame(audio, frame_length=frame_length, hop_length=hop_length)
        frame_rms = librosa.feature.rms(y=audio)[0]

        # ایٹیک اور ریلیز کے لیے کوائف
        attack_coeff = np.exp(-1 / (attack * self.sample_rate / hop_length))
        release_coeff = np.exp(-1 / (release * self.sample_rate / hop_length))

        # گین کیلکولیٹ کریں
        gain = np.ones_like(frame_rms)
        current_gain = 1.0

        for i, rms in enumerate(frame_rms):
            if rms > 0:
                desired_gain = target_level / rms
                if desired_gain < current_gain:
                    # ایٹیک
                    current_gain = current_gain * attack_coeff + desired_gain * (1 - attack_coeff)
                else:
                    # ریلیز
                    current_gain = current_gain * release_coeff + desired_gain * (1 - release_coeff)

                gain[i] = current_gain

        # انٹرپولیٹ کریں
        gain_full = np.repeat(gain, hop_length)[:len(audio)]
        if len(gain_full) < len(audio):
            gain_full = np.pad(gain_full, (0, len(audio) - len(gain_full)), mode='edge')

        # گین لگائیں
        agc_audio = audio * gain_full

        return agc_audio

    def plot_comparison(self, original: np.ndarray, processed: np.ndarray, title: str = "Audio Comparison"):
        """اصل اور پروسیسڈ آڈیو کا موازنہ پلاٹ کریں"""
        try:
            time_axis = np.linspace(0, len(original) / self.sample_rate, len(original))

            plt.figure(figsize=(15, 8))

            plt.subplot(2, 1, 1)
            plt.plot(time_axis[:len(original)//10], original[:len(original)//10])  # پہلے 10%
            plt.title(f'{title} - Original Audio')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Amplitude')
            plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.plot(time_axis[:len(processed)//10], processed[:len(processed)//10])  # پہلے 10%
            plt.title(f'{title} - Processed Audio')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Amplitude')
            plt.grid(True)

            plt.tight_layout()
            plt.savefig('/tmp/audio_comparison.png')
            print("Audio comparison plot saved to /tmp/audio_comparison.png")

        except ImportError:
            print("Matplotlib not available, skipping plot generation")

    def comprehensive_preprocessing(self, audio: np.ndarray) -> Dict:
        """مکمل پری پروسیسنگ پائپ لائن"""
        results = {
            'original': audio,
            'steps': {}
        }

        # 1. DC آف سیٹ ہٹائیں
        step1 = signal.detrend(audio, type='constant')
        results['steps']['dc_offset_removal'] = step1

        # 2. AGC
        step2 = self.automatic_gain_control(step1)
        results['steps']['agc'] = step2

        # 3. اسپیکٹرل سب ٹریکشن
        step3 = self.spectral_subtraction(step2)
        results['steps']['spectral_subtraction'] = step3

        # 4. ایڈاپٹیو نوائز ریموال
        step4 = self.adaptive_noise_reduction(step3)
        results['steps']['adaptive_denoising'] = step4

        # 5. فائنل نارملائزیشن
        final_audio = step4 / (np.max(np.abs(step4)) + 1e-8) * 0.8  # 0.8 کا پیک فیکٹر
        results['processed'] = final_audio

        # VAD کا نتیجہ
        vad_result = self.voice_activity_detection(final_audio)
        results['vad_result'] = vad_result

        return results

def main():
    print("Advanced Audio Preprocessing")

    # ٹیسٹ کے لیے جنرک آڈیو ڈیٹا
    sample_rate = 16000
    duration = 5  # 5 سیکنڈ
    t = np.linspace(0, duration, int(sample_rate * duration))

    # ٹیسٹ آڈیو تیار کریں (سگنل + نوائز)
    test_signal = 0.5 * np.sin(2 * np.pi * 440 * t)  # 440 Hz ٹون
    test_signal += 0.3 * np.sin(2 * np.pi * 880 * t)  # 880 Hz ٹون
    test_noise = 0.1 * np.random.randn(len(t))  # نوائز
    test_audio = test_signal + test_noise

    # ایڈوانسڈ پری پروسیسر
    advanced_preprocessor = AdvancedAudioPreprocessor()

    # مکمل پری پروسیسنگ
    results = advanced_preprocessor.comprehensive_preprocessing(test_audio)

    print(f"Comprehensive preprocessing completed!")
    print(f"Original audio length: {len(test_audio)} samples")
    print(f"Processed audio length: {len(results['processed'])} samples")
    print(f"VAD segments: {sum(results['vad_result'])} active frames out of {len(results['vad_result'])}")

    # موازنہ پلاٹ کریں
    advanced_preprocessor.plot_comparison(test_audio, results['processed'])

    # پروسیسڈ آڈیو کو محفوظ کریں
    try:
        sf.write('/tmp/processed_audio.wav', results['processed'], sample_rate)
        print("Processed audio saved to /tmp/processed_audio.wav")
    except:
        print("Could not save audio file (soundfile not available)")

if __name__ == "__main__":
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- آڈیو پری پروسیسنگ کے اہم طریقے سیکھے
- نوائز ریموال، VAD، AGC کا استعمال کیا
- ROS 2 میں پری پروسیسنگ نوڈ تیار کیا
- ایڈوانسڈ پری پروسیسنگ تکنیکس استعمال کیں

یہ پری پروسیسنگ VLA سسٹم کے لیے وائس کوالٹی کو بہتر بنانے کے لیے اہم ہے۔