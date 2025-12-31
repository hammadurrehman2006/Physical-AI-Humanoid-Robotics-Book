---
sidebar_position: 1
title: "وائس پروسیسنگ سیٹ اپ"
---

# وائس پروسیسنگ سیٹ اپ

## جائزہ

اس سیکشن میں، ہم اپنے وژن لینگویج ایکشن سسٹم کے لیے بنیادی وائس پروسیسنگ صلاحیتوں کو قائم کریں گے۔ اس میں سپیچ ریکوگنیشن، آڈیو پری پروسیسنگ، اور وائس کمانڈ تشریح کو مربوط کرنا شامل ہے تاکہ قدرتی ہیومن روبوٹ انٹرایکشن کو فعال کیا جا سکے۔

وائس پروسیسنگ ہیومن روبوٹ انٹرایکشن کا ایک اہم جزو ہے جو روبوٹس کو بولے گئے کمانڈز کو سمجھنے اور جواب دینے کے قابل بناتا ہے۔ یہ صلاحیت ہمیں انٹویٹو انٹرفیسز تخلیق کرنے کے قابل بناتی ہے جو صارفین کو روبوٹس کے ساتھ قدرتی طور پر انٹرایکٹ کرنے کی اجازت دیتے ہیں، بغیر کسی مخصوص انٹرفیس یا پروگرامنگ کے علم کی ضرورت کے۔

## سیکھنے کے اہداف

اس سیکشن کے اختتام تک، آپ کے اہل ہوگا::
- آڈیو ان پٹ اور پری پروسیسنگ پائپ لائنز قائم کریں
- وائس کمانڈ ریکوگنیشن کے لیے سپیچ ٹو ٹیکسٹ سروسز کو مربوط کریں
- وائس ایکٹیویٹی ڈیٹیکشن اور نوائز ریڈکشن کو کنفیگر کریں
- بنیادی وائس کمانڈ پارسنگ اور تشریح کو امپلیمنٹ کریں
- سیمولیٹڈ اور حقیقی ماحول میں وائس پروسیسنگ کی صلاحیتوں کو ٹیسٹ کریں

## شرائط

اس سیکشن کو شروع کرنے سے پہلے، یقین دہانی کریں کہ آپ کے پاس::
- Python 3.10+ انسٹال ہے
- آڈیو پروسیسنگ کے تصورات کا بنیادی اندراج
- ROS 2 Humble Hawksbill ماحول کنفیگر کیا گیا ہے
- مائیکروفون ہارڈویئر یا سیمولیٹڈ آڈیو ان پٹ تک رسائی
- سپیچ ریکوگنیشن سروسز کے لیے OpenAI API کلید

## روبوٹکس میں وائس پروسیسنگ کا تعارف

روبوٹکس میں وائس پروسیسنگ بولی گئی زبان کو عمل کے قابل کمانڈز میں تبدیل کرنے کا مسئلہ ہے جسے روبوٹ سمجھ سکتا اور انجام دے سکتا ہے۔ یہ ٹیکنالوجی قدرتی ہیومن روبوٹ انٹرایکشن کو فعال کرتی ہے، جس سے صارفین کو روبوٹس کے ساتھ ہر روز کی زبان کا استعمال کرتے ہوئے بات چیت کرنے کی اجازت ملتی ہے بجائے مخصوص انٹرفیسز کے۔

### وائس پروسیسنگ سسٹم کے کلیدی جزوات

وائس پروسیسنگ سسٹم عام طور پر شامل کرتا ہے::

1. **آڈیو ان پٹ**: مائیکروفونز کے ذریعے بولے گئے کمانڈز کو کیپچر کرنا
2. **پری پروسیسنگ**: آڈیو کوالٹی کو بہتر کرنا اور نوائز کو کم کرنا
3. **سپیچ ریکوگنیشن**: سپیچ کو ٹیکسٹ میں تبدیل کرنا
4. **نیچرل لینگویج اندراج**: ٹیکسٹ کے مطلب کی تشریح کرنا
5. **کمانڈ ایکزیکیوشن**: سمجھے گئے کمانڈز کو روبوٹ ایکشنز میں ترجمہ کرنا

### روبوٹکس ایپلیکیشنز میں فوائد

- **قدرتی انٹرایکشن**: صارفین ہر روز کی بولی گئی زبان کا استعمال کر کے بات چیت کر سکتے ہیں
- **ہینڈز فری آپریشن**: خاص طور پر ان منظرناموں میں قیمتی ہے جہاں دستی کنٹرول مشکل ہے
- **ایکسیسیبلٹی**: روبوٹکس کو مختلف صارف گروپس کے لیے زیادہ قابل رسائی بناتا ہے
- **کارکردگی**: تیز کمانڈ ان پٹ اور کام کی تکمیل کو فعال کرتا ہے

## OpenAI وہیسپر انٹیگریشن کا جائزہ

OpenAI وہیسپر ایک جدید خودکار سپیچ ریکوگنیشن (ASR) سسٹم ہے جسے 680,000 گھنٹے کے متعدد زبانوں اور متعدد کاموں کے نگرانی والے ڈیٹا سے تربیت دی گئی ہے جو ویب سے جمع کیا گیا تھا۔ یہ متعدد زبانوں اور آکوسٹک حالات میں مضبوط کارکردگی کا مظاہرہ کرتا ہے، جو اسے روبوٹکس ایپلیکیشنز کے لیے مثالی بناتا ہے۔

### روبوٹکس کے لیے وہیسپر کیوں؟

- **ملٹی لینگوئل سپورٹ**: متعدد زبانوں کی حمایت خود بخود
- **مضبوطی**: شور والے ماحول میں اچھا کام کرتا ہے
- **درستگی**: مختلف لہجوں میں اعلی ٹرانسکرپشن کی درستگی
- **کلاؤڈ بیسڈ**: مقامی ماڈلز کو برقرار رکھنے کی ضرورت نہیں
- **مسلسل اپ ڈیٹس**: OpenAI سے مسلسل بہتری

### وہیسپر ماڈل ویرینٹس

وہیسپر مختلف کارکردگی کے خصوصیات کے ساتھ کئی ماڈل سائز فراہم کرتا ہے::

| ماڈل | سائز | درکار VRAM | رشتہ دار رفتار | صرف انگریزی | متعدد زبانیں |
|-------|------|---------------|----------------|--------------|--------------|
| tiny  | 75 MB | ~1 GB | ~32x | 97.8% | 96.7% |
| base  | 145 MB | ~1 GB | ~16x | 96.8% | 95.0% |
| small | 465 MB | ~2 GB | ~6x | 95.0% | 93.8% |
| medium | 1.5 GB | ~5 GB | ~2x | 93.8% | 93.0% |
| large | 3.0 GB | ~10 GB | 1x | 95.7% | 95.0% |

روبوٹکس ایپلیکیشنز کے لیے، `base` یا `small` ماڈلز عام طور پر کارکردگی اور وسائل کے استعمال کا بہتر توازن فراہم کرتے ہیں۔

### وہیسپر API انٹیگریشن

وہیسپر API ایک کلاؤڈ بیسڈ حل فراہم کرتا ہے جس کے لیے مقامی ماڈل ڈیپلائمنٹ کی ضرورت نہیں ہوتی۔ یہ روبوٹکس ایپلیکیشنز کے لیے خاص طور پر فائدہ مند ہے جہاں کمپیوٹیشنل وسائل محدود ہو سکتے ہیں۔

```python
import openai

class WhisperSpeechRecognizer:
    def __init__(self, api_key):
        openai.api_key = api_key

    def transcribe_audio(self, audio_data, language="en", model="whisper-1"):
        """OpenAI وہیسپر API کا استعمال کرتے ہوئے آڈیو ٹرانسکرائب کریں"""
        import io
        from pydub import AudioSegment

        # آڈیو ڈیٹا کو WAV فارمیٹ میں تبدیل کریں
        audio_segment = AudioSegment(
            data=audio_data,
            sample_width=2,  # 16-bit
            frame_rate=44100,
            channels=1
        )

        # میموری میں WAV کو ایکسپورٹ کریں
        wav_buffer = io.BytesIO()
        audio_segment.export(wav_buffer, format="wav")
        wav_buffer.seek(0)

        # وہیسپر API کا استعمال کرتے ہوئے ٹرانسکرائب کریں
        response = openai.Audio.transcribe(
            model=model,
            file=wav_buffer,
            language=language,
            response_format="text"
        )

        return response.strip()
```

## وائس پروسیسنگ پائپ لائن کا قیام

ریل ٹائم روبوٹکس ایپلیکیشنز کے لیے ایک کارآمد وائس پروسیسنگ پائپ لائن تخلیق کرنا ضروری ہے۔ پائپ لائن کو کم سے کم تاخیر کے ساتھ آڈیو کیپچر، پری پروسیسنگ، ریکوگنیشن، اور کمانڈ پارسنگ کو ہینڈل کرنا چاہیے تاکہ روبوٹ کے رویے کو جواب دہ بنایا جا سکے۔

### پائپ لائن آرکیٹیکچر

وائس پروسیسنگ پائپ لائن کئی مربوط جزوات پر مشتمل ہے جو بولے گئے کمانڈز کو روبوٹ ایکشنز میں تبدیل کرنے کے لیے ایک ساتھ کام کرتے ہیں::

1. **آڈیو کیپچر**: ان پٹ ڈیوائسز سے جاری طور پر آڈیو کو کیپچر کرتا ہے
2. **بفر مینجمنٹ**: کارآمد پروسیسنگ کے لیے آڈیو چنکس کا نظم کرتا ہے
3. **پری پروسیسنگ**: نوائز ریڈکشن اور ایکوائزنگ لاگو کرتا ہے
4. **وائس ایکٹیویٹی ڈیٹیکشن**: سپیچ پر مشتمل سیگمنٹس کی شناخت کرتا ہے
5. **سپیچ ریکوگنیشن**: سپیچ کو ٹیکسٹ میں تبدیل کرتا ہے
6. **کمانڈ پارسنگ**: ٹیکسٹ کمانڈز کی تشریح کرتا ہے اور ارادہ نکالتا ہے
7. **ایکشن ایکزیکیوشن**: کمانڈز کو روبوٹ برتاؤ میں ترجمہ کرتا ہے

### پائپ لائن امپلیمنٹیشن

وائس پروسیسنگ پائپ لائن کا ایک جامع امپلیمنٹیشن یہاں ہے::

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from builtin_interfaces.msg import Time
import threading
import queue
import numpy as np
from dataclasses import dataclass
from typing import List, Optional
import re
import time

@dataclass
class VoiceCommand:
    intent: str
    entities: dict
    confidence: float
    original_text: str

class AudioPreprocessor:
    def __init__(self):
        self.sample_rate = 44100
        self.frame_length = 1024
        self.hop_length = 512

    def noise_reduction(self, audio_data):
        """اسپیکٹرل سب ٹریکشن کا استعمال کرتے ہوئے نوائز ریڈکشن لاگو کریں"""
        import librosa
        # فریکوینسی ڈومین میں تبدیل کریں
        stft = librosa.stft(audio_data, n_fft=self.frame_length, hop_length=self.hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # نوائز پروفائل کا اندازہ لگائیں (سادہ نقطہ نظر)
        noise_profile = np.mean(magnitude[:, :100], axis=1, keepdims=True)

        # اسپیکٹرل سب ٹریکشن لاگو کریں
        enhanced_magnitude = np.maximum(magnitude - noise_profile * 0.3, 0)

        # آڈیو کو دوبارہ تعمیر کریں
        enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
        enhanced_audio = librosa.istft(enhanced_stft, hop_length=self.hop_length)

        return enhanced_audio

    def voice_activity_detection(self, audio_data, threshold=0.01):
        """سادہ انرجی بیسڈ وائس ایکٹیویٹی ڈیٹیکشن"""
        frame_energy = np.array([
            np.mean(frame**2)
            for frame in self._frame_audio(audio_data, self.frame_length)
        ])

        # انرجی کو نارملائز کریں
        normalized_energy = (frame_energy - np.min(frame_energy)) / (
            np.max(frame_energy) - np.min(frame_energy) + 1e-8
        )

        # وائس ایکٹیویٹی کا پتہ لگائیں
        voice_active = normalized_energy > threshold
        return voice_active

    def _frame_audio(self, audio_data, frame_length):
        """آڈیو کو فریم میں تقسیم کریں"""
        frames = []
        for i in range(0, len(audio_data) - frame_length, self.hop_length):
            frames.append(audio_data[i:i + frame_length])
        return frames

class VoiceCommandParser:
    def __init__(self):
        self.command_patterns = {
            'move': [
                r'move\s+(?P<direction>\w+)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>\w+)?',
                r'go\s+(?P<direction>\w+)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>\w+)?',
                r'walk\s+(?P<direction>\w+)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>\w+)?'
            ],
            'grasp': [
                r'grasp\s+(?P<object>\w+)',
                r'pick\s+up\s+(?P<object>\w+)',
                r'grab\s+(?P<object>\w+)'
            ],
            'navigate': [
                r'go\s+to\s+(?P<location>\w+)',
                r'navigate\s+to\s+(?P<location>\w+)',
                r'move\s+to\s+(?P<location>\w+)'
            ],
            'stop': [
                r'stop',
                r'halt',
                r'pause'
            ]
        }

    def parse_command(self, text: str) -> Optional[VoiceCommand]:
        """ٹیکسٹ سے وائس کمانڈ کو پارس کریں"""
        text = text.lower().strip()

        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    entities = match.groupdict()
                    # پیٹرن میچ کے معیار کی بنیاد پر یقین دہانی کا حساب لگائیں
                    confidence = self._calculate_confidence(text, pattern)

                    return VoiceCommand(
                        intent=intent,
                        entities=entities,
                        confidence=confidence,
                        original_text=text
                    )

        return None

    def _calculate_confidence(self, text: str, pattern: str) -> float:
        """پیٹرن میچ کے لیے یقین کا اسکور حساب لگائیں"""
        # ٹیکسٹ کی لمبائی اور پیٹرن کی پیچیدگی کی بنیاد پر سادہ یقین کا حساب
        match_length = len(text)
        pattern_complexity = len(pattern)

        # طویل، زیادہ مخصوص میچز کے لیے زیادہ یقین
        confidence = min(0.9, 0.5 + (match_length / 100))
        return confidence

class VoiceProcessingPipeline:
    def __init__(self, api_key: str):
        self.preprocessor = AudioPreprocessor()
        self.speech_recognizer = WhisperSpeechRecognizer(api_key)
        self.command_parser = VoiceCommandParser()

        # آڈیو پروسیسنگ کی قطار
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # پروسیسنگ کی حالت
        self.buffer = np.array([])
        self.min_voice_duration = 0.5  # سیکنڈ
        self.max_buffer_duration = 5.0  # سیکنڈ
        self.voice_threshold = 0.01

    def process_audio_chunk(self, audio_chunk: np.ndarray):
        """پائپ لائن کے ذریعے ایک واحد آڈیو چنک کو پروسیس کریں"""
        # چنک کو بفر میں شامل کریں
        self.buffer = np.concatenate([self.buffer, audio_chunk])

        # چیک کریں کہ آیا بفر زیادہ لمبی مدت تک پہنچ گیا ہے
        buffer_duration = len(self.buffer) / self.preprocessor.sample_rate
        if buffer_duration > self.max_buffer_duration:
            # پروسیس کریں قبل اس کے کہ یہ بہت بڑا ہو جائے
            self._process_buffer()

    def _process_buffer(self):
        """موجودہ آڈیو بفر کو پروسیس کریں"""
        if len(self.buffer) == 0:
            return

        # آڈیو کو پری پروسیس کریں
        processed_audio = self.preprocessor.noise_reduction(self.buffer)

        # وائس ایکٹیویٹی کے لیے چیک کریں
        voice_active = self.preprocessor.voice_activity_detection(processed_audio)

        if any(voice_active):
            # سپیچ ریکوگنیشن کے لیے مناسب فارمیٹ میں تبدیل کریں
            int16_audio = (processed_audio * 32767).astype(np.int16)

            # ٹرانسکرائب اور پارس کریں
            try:
                text = self.speech_recognizer.transcribe_audio(int16_audio.tobytes())
                if text.strip():
                    command = self.command_parser.parse_command(text)
                    if command and command.confidence > 0.7:
                        self.result_queue.put(command)
            except Exception as e:
                print(f"سپیچ ریکوگنیشن کی خرابی: {e}")

        # پروسیسنگ کے بعد بفر کو صاف کریں
        self.buffer = np.array([])

    def get_processed_commands(self) -> List[VoiceCommand]:
        """نتائج کی قطار سے تمام پروسیسڈ کمانڈز حاصل کریں"""
        commands = []
        while not self.result_queue.empty():
            commands.append(self.result_queue.get())
        return commands

    def reset(self):
        """پائپ لائن کی حالت کو ری سیٹ کریں"""
        self.buffer = np.array([])
        while not self.audio_queue.empty():
            self.audio_queue.get()
        while not self.result_queue.empty():
            self.result_queue.get()
```

### پائپ لائن کی اصلاح

روبوٹکس ایپلیکیشنز میں بہترین کارکردگی کو یقینی بنانے کے لیے، درج ذیل اصلاحات پر غور کریں::

1. **بفر مینجمنٹ**: تاخیر اور کارآمدگی کو توازن میں رکھنے کے لیے مناسب بفر سائز استعمال کریں
2. **تھریڈنگ**: مین لوپ کو بلاک کرنے سے بچنے کے لیے علیحدہ تھریڈز میں آڈیو کو پروسیس کریں
3. **کیش**: جہاں ممکن ہو اکثر استعمال ہونے والے ماڈلز اور ڈیٹا کو کیش کریں
4. **ریسورس مینجمنٹ**: نظام کے اوور لوڈ کو روکنے کے لیے وسائل کے استعمال کو مانیٹر کریں اور محدود کریں

## آڈیو ان پٹ اور پری پروسیسنگ

آڈیو ان پٹ اور پری پروسیسنگ روبوٹکس میں کامیاب وائس پروسیسنگ کے لیے بنیادی ہیں۔ ان پٹ آڈیو کی کوالٹی سپیچ ریکوگنیشن اور کمانڈ تشریح کی درستگی کو براہ راست متاثر کرتی ہے۔

### آڈیو ان پٹ کنفیگریشن

روبوٹکس ایپلیکیشنز کے لیے، آڈیو ان پٹ مختلف ذرائع سے آ سکتا ہے::

- **بِلٹ ان مائیکروفونز**: روبوٹ پلیٹ فارم میں انٹیگریٹڈ
- **بیرونی مائیکروفون ایریز**: بہتر ہدایت اور نوائز ریڈکشن کے لیے
- **سیمولیٹڈ آڈیو**: ٹیسٹنگ اور ڈیولپمنٹ کے مقاصد کے لیے

### آڈیو پری پروسیسنگ ٹیکنیکس

حقیقی دنیا کے ماحول میں مضبوط وائس پروسیسنگ کے لیے مؤثر پری پروسیسنگ انتہائی ضروری ہے::

1. **نوائز ریڈکشن**: سپیچ کی وضاحت کو بہتر کرنے کے لیے پس منظر کا نوائز ہٹائیں
2. **ایکو کینسلیشن**: بند جگہوں میں ایکوسٹک فیڈ بیک کو ختم کریں
3. **وائس ایکٹیویٹی ڈیٹیکشن**: پروسیسنگ کے اوور ہیڈ کو کم کرنے کے لیے سپیچ سیگمنٹس کی شناخت کریں
4. **نارملائزیشن**: مستقل پروسیسنگ کے لیے آڈیو لیولز کو ایڈجسٹ کریں

```python
import pyaudio
import wave
import numpy as np
from scipy import signal

class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')
        self.publisher = self.create_publisher(AudioData, 'audio_stream', 10)

        # آڈیو پیرامیٹرز کنفیگر کریں
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100

        self.audio = pyaudio.PyAudio()

        # علیحدہ تھریڈ میں آڈیو کیپچر شروع کریں
        self.capture_thread = threading.Thread(target=self._capture_audio, daemon=True)
        self.capture_thread.start()

    def _capture_audio(self):
        """علیحدہ تھریڈ میں آڈیو کیپچر کریں"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.get_logger().info("آڈیو کیپچر شروع ہو گیا")

        while rclpy.ok():
            try:
                data = stream.read(self.chunk)
                audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                # آڈیو ڈیٹا شائع کریں
                msg = AudioData()
                msg.data = data
                msg.sample_rate = self.rate
                msg.channels = self.channels
                msg.encoding = '16BIT'
                self.publisher.publish(msg)

            except Exception as e:
                self.get_logger().error(f"آڈیو کیپچر کی خرابی: {e}")
                break

        stream.stop_stream()
        stream.close()

    def destroy_node(self):
        self.audio.terminate()
        super().destroy_node()
```

### پری پروسیسنگ پائپ لائن

پری پروسیسنگ پائپ لائن سپیچ ریکوگنیشن سے پہلے آڈیو کوالٹی کو بہتر بناتا ہے::

```python
class AdvancedAudioPreprocessor:
    def __init__(self):
        self.sample_rate = 44100
        self.frame_length = 1024
        self.hop_length = 512
        self.noise_buffer = []
        self.noise_buffer_size = 100

    def preprocess_audio(self, audio_data):
        """مکمل پری پروسیسنگ پائپ لائن"""
        # 1. نوائز ریڈکشن
        denoised = self.noise_reduction(audio_data)

        # 2. ایکو کینسلیشن (سادہ)
        echo_cancelled = self.echo_cancellation(denoised)

        # 3. نارملائزیشن
        normalized = self.normalize_audio(echo_cancelled)

        # 4. وائس ایکٹیویٹی ڈیٹیکشن
        vad_result = self.voice_activity_detection(normalized)

        return normalized, vad_result

    def noise_reduction(self, audio_data):
        """اسپیکٹرل سب ٹریکشن کا استعمال کرتے ہوئے اعلی نوائز ریڈکشن"""
        import librosa

        # STFT
        stft = librosa.stft(audio_data, n_fft=self.frame_length, hop_length=self.hop_length)
        magnitude = np.abs(stft)
        phase = np.angle(stft)

        # ضرورت ہونے پر نوائز پروفائل اپ ڈیٹ کریں
        if len(self.noise_buffer) < self.noise_buffer_size:
            self.noise_buffer.append(magnitude)
        else:
            self.noise_buffer.pop(0)
            self.noise_buffer.append(magnitude)

        # اوسط نوائز پروفائل کا حساب لگائیں
        if len(self.noise_buffer) > 0:
            noise_profile = np.mean(np.array(self.noise_buffer), axis=0)

            # وینر فلٹرنگ لاگو کریں
            enhanced_magnitude = (magnitude**2) / (magnitude + noise_profile + 1e-8)
            enhanced_magnitude = np.sqrt(enhanced_magnitude)

            # آڈیو کو دوبارہ تعمیر کریں
            enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
            enhanced_audio = librosa.istft(enhanced_stft, hop_length=self.hop_length)

            return enhanced_audio

        return audio_data

    def echo_cancellation(self, audio_data):
        """سادہ ایکو کینسلیشن"""
        # یہ ایک سادہ امپلیمنٹیشن ہے
        # عمل میں، زیادہ ترقی یافتہ الگورتھم جیسے NLMS استعمال کیے جائیں گے
        return audio_data

    def normalize_audio(self, audio_data):
        """آڈیو کو مستقل سطح پر نارملائز کریں"""
        # RMS کا حساب لگائیں
        rms = np.sqrt(np.mean(audio_data**2))

        # ہدف RMS
        target_rms = 0.1

        if rms > 0:
            gain = target_rms / rms
            normalized = audio_data * gain

            # کلیمپنگ سے بچنے کے لیے محدود کریں
            normalized = np.clip(normalized, -1.0, 1.0)
            return normalized

        return audio_data

    def voice_activity_detection(self, audio_data, threshold=0.01):
        """اعلی درجے کی وائس ایکٹیویٹی ڈیٹیکشن"""
        frame_energy = np.array([
            np.mean(frame**2)
            for frame in self._frame_audio(audio_data, self.frame_length)
        ])

        # جھوٹے مثبت کو کم کرنے کے لیے اسموتھنگ لاگو کریں
        smoothed_energy = np.convolve(frame_energy, np.ones(5)/5, mode='same')

        # انرجی کو نارملائز کریں
        normalized_energy = (smoothed_energy - np.min(smoothed_energy)) / (
            np.max(smoothed_energy) - np.min(smoothed_energy) + 1e-8
        )

        # ہسٹریسس کے ساتھ وائس ایکٹیویٹی کا پتہ لگائیں تاکہ ٹوگلنگ کم ہو
        voice_active = normalized_energy > threshold
        return voice_active

    def _frame_audio(self, audio_data, frame_length):
        """آڈیو کو فریم میں تقسیم کریں"""
        frames = []
        for i in range(0, len(audio_data) - frame_length, self.hop_length):
            frames.append(audio_data[i:i + frame_length])
        return frames
```

## سپیچ ریکوگنیشن کے بنیادیات

سپیچ ریکوگنیشن وہ مرکزی ٹیکنالوجی ہے جو بولی گئی زبان کو ٹیکسٹ میں تبدیل کرتی ہے۔ اس کے بنیادیات کو سمجھنا روبوٹکس میں مؤثر وائس پروسیسنگ سسٹم امپلیمنٹ کرنے کے لیے اہم ہے۔

### سپیچ ریکوگنیشن کیسے کام کرتا ہے

سپیچ ریکوگنیشن سسٹم عام طور پر ان اقدامات پر عمل کرتے ہیں::

1. **فیچر ایکسٹریکشن**: آڈیو سگنلز کو فیچرز میں تبدیل کریں جو سپیچ کے خصوصیات کی نمائندگی کرتے ہیں
2. **ایکوسٹک ماڈلنگ**: ایکوسٹک فیچرز کو فونیمز (بنیادی سپیچ ساؤنڈز) میں میپ کریں
3. **لینگویج ماڈلنگ**: زبان کے قواعد کا استعمال کریں تاکہ امکانی لفظ کی ترتیبات کا تعین کیا جا سکے
4. **ڈیکوڈنگ**: ایکوسٹک اور لینگویج ماڈلز کو جوڑ کر بہترین ٹیکسٹ آؤٹ پٹ پیدا کریں

### سپیچ ریکوگنیشن سسٹم کی اقسام

1. **ٹیمپلیٹ بیسڈ**: سپیچ کو محفوظ شدہ ٹیمپلیٹس کے ساتھ موازنہ کریں
2. **اسٹیٹسٹیکل**: اسٹیٹسٹیکل ماڈلز کا استعمال کریں تاکہ سپیچ پیٹرنز کو پہچانیں
3. **نیورل نیٹ ورک بیسڈ**: اینڈ ٹو اینڈ ریکوگنیشن کے لیے گہری سیکھنے کا استعمال کریں
4. **ہائبرڈ**: بہتر درستگی کے لیے متعدد نقطہ نظر کو جوڑیں

### وہیسپر مخصوص اہمیات

OpenAI وہیسپر ایک ٹرانسفارمر بیسڈ آرکیٹیکچر کا استعمال کرتا ہے جس میں کئی منفرد خصوصیات ہیں::

- **ملٹی لینگوئل تربیت**: متعدد زبانوں پر ایک ہی وقت میں تربیت
- **مضبوط فیچر ایکسٹریکشن**: مختلف ایکوسٹک حالات کو ہینڈل کرتا ہے
- **اینڈ ٹو اینڈ پروسیسنگ**: براہ راست آڈیو کو ٹیکسٹ میں تبدیل کر دیتا ہے
- **بڑے پیمانے پر تربیت**: وسیع تربیتی ڈیٹا کے فوائد

### کارکردگی کے عوامل

کئی عوامل روبوٹکس میں سپیچ ریکوگنیشن کی کارکردگی کو متاثر کرتے ہیں::

- **آڈیو کوالٹی**: صاف آڈیو کافی حد تک ریکوگنیشن کی درستگی کو بہتر بناتی ہے
- **پس منظر کا نوائز**: نوائز ریڈکشن پری پروسیسنگ ضروری ہے
- **اسپیکر کی دوری**: قریب کے اسپیکرز عام طور پر بہتر ریکوگنیشن فراہم کرتے ہیں
- **ایکوسٹک ماحول**: کمرے کی ایکوسٹک کارکردگی کو متاثر کر سکتی ہے
- **لینگویج ماڈل**: ڈومین مخصوص لینگویج ماڈل درستگی کو بہتر کر سکتے ہیں

## کنفیگریشن اختیارات اور پیرامیٹرز

مختلف ماحول اور ایپلیکیشنز میں بہترین کارکردگی کے لیے وائس پروسیسنگ پیرامیٹرز کی مناسب کنفیگریشن ضروری ہے۔

### آڈیو کنفیگریشن پیرامیٹرز

```yaml
# config/voice_processing.yaml
voice_processing:
  audio:
    sample_rate: 44100           # آڈیو سیمپلنگ کی شرح (Hz)
    channels: 1                  # آڈیو چینلز کی تعداد (مونو)
    chunk_size: 1024             # آڈیو بفر چنک سائز
    format: paInt16              # آڈیو فارمیٹ
    input_device_index: null     # مخصوص ان پٹ ڈیوائس (ڈیفالٹ کے لیے null)
    frame_duration: 0.023        # سیکنڈ میں آڈیو فریم کی مدت

  preprocessing:
    noise_reduction_enabled: true
    noise_reduction_factor: 0.3  # نوائز سب ٹریکشن کے لیے عنصر
    vad_threshold: 0.01          # وائس ایکٹیویٹی ڈیٹیکشن کی حد
    frame_length: 1024           # آڈیو فریم کی لمبائی
    hop_length: 512              # اوور لیپنگ فریم کے لیے ہاپ لمبائی
    normalization_enabled: true
    echo_cancellation_enabled: false

  recognition:
    service: "openai_whisper"    # اختیارات: "openai_whisper", "vosk_local"
    language: "en"               # ریکوگنیشن زبان
    model: "whisper-1"           # استعمال کرنے کے لیے وہیسپر ماڈل
    api_key: "${OPENAI_API_KEY}" # OpenAI API کلید
    model_path: "/path/to/vosk/model"  # مقامی ووسک ماڈلز کے لیے راستہ
    timeout: 30                  # سیکنڈ میں ریکوگنیشن ٹائم آؤٹ
    max_audio_duration: 30       # پروسیس کرنے کے لیے زیادہ سے زیادہ آڈیو کی مدت

  commands:
    confidence_threshold: 0.7    # کمانڈ قبول کرنے کے لیے کم از کم یقین
    max_audio_duration: 30       # زیادہ سے زیادہ آڈیو کی مدت (سیکنڈ)
    silence_duration: 500        # خاموشی کے لیے انتظار کی مدت (ملی سیکنڈ)
    command_timeout: 5           # کمانڈ مکمل ہونے کے لیے انتظار کا وقت
    retry_attempts: 3            # ناکام ریکوگنیشن کے لیے دوبارہ کوشش کی تعداد

  performance:
    buffer_size: 44100           # آڈیو بفر کا سائز (44.1kHz پر 1 سیکنڈ)
    processing_interval: 0.1     # پروسیسنگ سائیکل کے درمیان وقفہ (سیکنڈ)
    thread_count: 2              # پروسیسنگ تھریڈز کی تعداد
    memory_limit: "512MB"        # آڈیو پروسیسنگ کے لیے میموری حد
```

### پیرامیٹر ٹیوننگ ہدایات

1. **سیمپل ریٹ**: زیادہ سیمپل ریٹ بہتر کوالٹی فراہم کرتا ہے لیکن زیادہ پروسیسنگ طاقت کی ضرورت ہوتی ہے
2. **نوائز ریڈکشن**: ماحول کے نوائز کی سطحوں کے مطابق ایڈجسٹ کریں
3. **VAD حد**: کم ویلیوز آہستہ سپیچ کا پتہ لگاتی ہیں لیکن نوائز شامل کر سکتی ہیں
4. **یقین کی حد**: زیادہ ویلیوز جھوٹی مثبت کو کم کرتی ہیں لیکن درست کمانڈز چھوڑ سکتی ہیں

### ماحول کے مطابق کنفیگریشنز

مختلف ماحول کے لیے مختلف پیرامیٹر سیٹ کی ضرورت ہو سکتی ہے::

```python
# مختلف ماحول کے لیے کنفیگریشن کلاسز کی مثال
class IndoorConfig:
    """کنٹرول شدہ ایکوسٹکس والے اندر کے ماحول کے لیے کنفیگریشن"""
    sample_rate = 44100
    vad_threshold = 0.005  # خاموش اندر کے ماحول کے لیے کم حد
    noise_reduction_factor = 0.2
    confidence_threshold = 0.75

class OutdoorConfig:
    """متغیر نوائز والے باہر کے ماحول کے لیے کنفیگریشن"""
    sample_rate = 44100
    vad_threshold = 0.02   # باہر کے نوائز کے لیے زیادہ حد
    noise_reduction_factor = 0.4
    confidence_threshold = 0.8  # جھوٹی مثبت کو کم کرنے کے لیے زیادہ حد

class NoisyConfig:
    """اعلی نوائز والے صنعتی ماحول کے لیے کنفیگریشن"""
    sample_rate = 48000  # بہتر نوائز ہینڈلنگ کے لیے زیادہ سیمپل ریٹ
    vad_threshold = 0.05
    noise_reduction_factor = 0.5
    confidence_threshold = 0.85
```

## ٹیسٹنگ اور تصدیق کی طریقہ کار

جامع ٹیسٹنگ یقینی بناتی ہے کہ وائس پروسیسنگ سسٹم مختلف حالات اور منظرناموں میں قابل اعتماد طریقے سے کام کرتا ہے۔

### یونٹ ٹیسٹنگ

یونٹ ٹیسٹ وائس پروسیسنگ پائپ لائن کے انفرادی جزوات کی توثیق کرتے ہیں::

```python
import unittest
import numpy as np
from unittest.mock import Mock, patch

class TestVoiceProcessing(unittest.TestCase):
    def setUp(self):
        self.preprocessor = AudioPreprocessor()
        self.command_parser = VoiceCommandParser()

    def test_noise_reduction(self):
        """نوائز ریڈکشن فعالیت کا ٹیسٹ کریں"""
        # نوائز کے ساتھ ٹیسٹ آڈیو بنائیں
        clean_signal = np.sin(2 * np.pi * 440 * np.linspace(0, 1, 44100))
        noise = np.random.normal(0, 0.1, 44100)
        noisy_signal = clean_signal + noise

        enhanced_signal = self.preprocessor.noise_reduction(noisy_signal)

        # چیک کریں کہ نوائز کم ہو گیا ہے (SNR بہتر ہونا چاہیے)
        original_snr = np.mean(clean_signal**2) / np.mean((noisy_signal - clean_signal)**2)
        enhanced_snr = np.mean(clean_signal**2) / np.mean((enhanced_signal - clean_signal)**2)

        self.assertGreater(enhanced_snr, original_snr)

    def test_voice_activity_detection(self):
        """وائس ایکٹیویٹی ڈیٹیکشن کا ٹیسٹ کریں"""
        # خاموشی اور سپیچ کے ساتھ آڈیو بنائیں
        silence = np.zeros(44100)
        speech = np.sin(2 * np.pi * 440 * np.linspace(0, 1, 44100)) * 0.5
        combined = np.concatenate([silence, speech])

        vad_result = self.preprocessor.voice_activity_detection(combined)

        # پہلے نصف حصہ زیادہ تر غیر فعال ہونا چاہیے، دوسرا نصف فعال
        first_half_active = np.mean(vad_result[:len(vad_result)//2])
        second_half_active = np.mean(vad_result[len(vad_result)//2:])

        self.assertLess(first_half_active, second_half_active)

    def test_command_parsing(self):
        """وائس کمانڈ پارسنگ کا ٹیسٹ کریں"""
        test_commands = [
            ("move forward 2 meters", "move"),
            ("go to kitchen", "navigate"),
            ("pick up the red ball", "grasp"),
            ("stop immediately", "stop")
        ]

        for text, expected_intent in test_commands:
            command = self.command_parser.parse_command(text)
            self.assertIsNotNone(command)
            self.assertEqual(command.intent, expected_intent)

    def test_command_confidence(self):
        """کمانڈز کے لیے یقین کا حساب کا ٹیسٹ کریں"""
        command = self.command_parser.parse_command("move forward")
        self.assertIsNotNone(command)
        self.assertGreaterEqual(command.confidence, 0.5)
        self.assertLessEqual(command.confidence, 1.0)

    def test_invalid_command(self):
        """غلط کمانڈز کی پارسنگ کا ٹیسٹ کریں"""
        command = self.command_parser.parse_command("this is not a valid command")
        self.assertIsNone(command)
```

### انٹیگریشن ٹیسٹنگ

انٹیگریشن ٹیسٹ مکمل وائس پروسیسنگ پائپ لائن کی توثیق کرتے ہیں::

```python
class TestVoiceProcessingIntegration(unittest.TestCase):
    def setUp(self):
        # ٹیسٹ کے لیے مocker API کلید
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            self.pipeline = VoiceProcessingPipeline('test-key')

    def test_complete_pipeline(self):
        """مکمل وائس پروسیسنگ پائپ لائن کا ٹیسٹ کریں"""
        # ٹیسٹ آڈیو بنائیں (سادہ sine wave)
        sample_rate = 44100
        duration = 1.0  # سیکنڈ
        frequency = 440  # Hz
        t = np.linspace(0, duration, int(sample_rate * duration))
        test_audio = np.sin(2 * np.pi * frequency * t)

        # زیادہ حقیقت پسندانہ بنانے کے لیے کچھ نوائز شامل کریں
        noise = np.random.normal(0, 0.1, len(test_audio))
        noisy_audio = test_audio + noise

        # آڈیو کو پروسیس کریں
        self.pipeline.process_audio_chunk(noisy_audio)

        # چیک کریں کہ کمانڈز پروسیس کیے گئے تھے
        commands = self.pipeline.get_processed_commands()

        # پائپ لائن کو خرابی کے بغیر آڈیو کو ہینڈل کرنا چاہیے
        # نوٹ: یہ ٹیسٹ مصنوعی آڈیو کی وجہ سے مخصوص کمانڈز کی توقع نہیں کرتا
        self.assertIsInstance(commands, list)

    def test_pipeline_reset(self):
        """پائپ لائن ری سیٹ فعالیت کا ٹیسٹ کریں"""
        # پائپ لائن میں کچھ ڈیٹا شامل کریں
        test_audio = np.random.normal(0, 0.1, 44100)
        self.pipeline.process_audio_chunk(test_audio)

        # پائپ لائن کو ری سیٹ کریں
        self.pipeline.reset()

        # تصدیق کریں کہ بفر صاف کیا گیا ہے
        commands = self.pipeline.get_processed_commands()
        self.assertEqual(len(commands), 0)
        self.assertEqual(len(self.pipeline.buffer), 0)
```

### کارکردگی ٹیسٹنگ

کارکردگی کے ٹیسٹ یقینی بناتے ہیں کہ سسٹم ریل ٹائم تقاضوں کو پورا کرتا ہے::

```python
import time

class TestVoiceProcessingPerformance(unittest.TestCase):
    def setUp(self):
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            self.pipeline = VoiceProcessingPipeline('test-key')

    def test_processing_latency(self):
        """ٹیسٹ کریں کہ آیا آڈیو پروسیسنگ لیٹنسی کی ضروریات کو پورا کرتی ہے"""
        test_audio = np.random.normal(0, 0.1, 44100)  # 1 سیکنڈ کا آڈیو

        start_time = time.time()
        self.pipeline.process_audio_chunk(test_audio)
        end_time = time.time()

        processing_time = end_time - start_time
        max_allowed_time = 0.1  # ریل ٹائم پروسیسنگ کے لیے 100ms

        self.assertLess(processing_time, max_allowed_time,
               f"پروسیسنگ میں {processing_time:.3f}س لگا، {max_allowed_time}س سے تجاوز کر گیا")
```

    def test_buffer_management(self):
        """ٹیسٹ کریں کہ آیا آڈیو بفرز کو صحیح طریقے سے نظم کیا جاتا ہے"""
        # بفر مینجمنٹ کو ٹیسٹ کرنے کے لیے متعدد چنکس کو پروسیس کریں
        chunk_size = 1024
        for i in range(10):
            test_chunk = np.random.normal(0, 0.1, chunk_size)
            self.pipeline.process_audio_chunk(test_chunk)

        # تصدیق کریں کہ بفر لامحدود طور پر نہیں بڑھتا
        self.assertLess(len(self.pipeline.buffer), chunk_size * 20)  # چاہیے پروسیس ہو جائے

### تصدیق کی طریقہ کار

1. **آڈیو کوالٹی کی تصدیق**: تصدیق کریں کہ ان پٹ آڈیو معیار کی ضروریات کو پورا کرتا ہے
2. **ریکوگنیشن کی درستگی**: مختلف آڈیو حالات کے ساتھ ریکوگنیشن کی درستگی کو ٹیسٹ کریں
3. **کمانڈ ایکزیکیوشن**: تصدیق کریں کہ پہچانے گئے کمانڈز کو صحیح طریقے سے انجام دیا جاتا ہے
4. **کنارا کیس ہینڈلنگ**: غیر معمولی یا غیر متوقع ان پٹس کے ساتھ ٹیسٹ کریں

## عام مسائل اور ٹربل شوٹنگ

یہ سیکشن روبوٹکس میں وائس پروسیسنگ سسٹم امپلیمنٹ کرنے اور استعمال کرنے میں م遭遇 کیے جانے والے عام مسائل کو حل کرتا ہے۔

### آڈیو ان پٹ کے مسائل

**مسئلہ**: کوئی آڈیو ان پٹ کا پتہ نہیں چلا
- **سبب**: مائیکروفون مناسب طریقے سے منسلک یا کنفیگر نہیں ہے
- **حل**::
  1. جسمانی کنکشنز چیک کریں
  2. مائیکروفون کی اجازتیں تصدیق کریں
  3. `arecord -d 5 test.wav` کے ساتھ ٹیسٹ کریں
  4. سسٹم آڈیو سیٹنگز چیک کریں

**مسئلہ**: خراب آڈیو کوالٹی
- **سبب**: نوائز، کلپنگ، یا غلط آڈیو فارمیٹ
- **حل**::
  1. مائیکروفون گین سیٹنگز ایڈجسٹ کریں
  2. نوائز ریڈکشن پری پروسیسنگ استعمال کریں
  3. صحیح سیمپل ریٹ اور فارمیٹ کی تصدیق کریں
  4. آڈیو کلپنگ کے لیے چیک کریں

### ریکوگنیشن کے مسائل

**مسئلہ**: کم ریکوگنیشن کی درستگی
- **سبب**: پس منظر کا نوائز، خراب آڈیو کوالٹی، یا غلط زبان کی ترتیبات
- **حل**::
  1. آڈیو پری پروسیسنگ کو بہتر کریں
  2. نوائز ریڈکشن پیرامیٹرز ایڈجسٹ کریں
  3. زبان کی ترتیبات کی تصدیق کریں کہ بولی گئی زبان سے مماثل ہو
  4. ڈومین مخصوص زبان کے ماڈلز کا استعمال کریں

**مسئلہ**: ریکوگنیشن میں زیادہ لیٹنسی
- **سبب**: نیٹ ورک کی دیر، بڑے آڈیو بفرز، یا پروسیسنگ بُک مار
- **حل**::
  1. آڈیو بفر سائز کو اصلاح کریں
  2. نیٹ ورک سست ہونے پر مقامی ریکوگنیشن ماڈلز استعمال کریں
  3. اسینکرون پروسیسنگ لاگو کریں
  4. سسٹم ریسورس استعمال کو مانیٹر کریں

### کنفیگریشن کے مسائل

**مسئلہ**: پیرامیٹرز اثر نہیں ڈال رہے
- **سبب**: کنفیگریشن فائل لوڈ نہیں ہوئی یا پیرامیٹرز کو اوور رائیڈ کر دیا گیا
- **حل**::
  1. کنفیگریشن فائل کا راستہ اور فارمیٹ تصدیق کریں
  2. کوڈ میں پیرامیٹر اوور رائیڈ کے لیے چیک کریں
  3. پیرامیٹر ویلیوز کی تصدیق کے لیے لاگنگ شامل کریں
  4. کنفیگریشن تبدیلیوں کے بعد سسٹم کو دوبارہ شروع کریں

### کارکردگی کے مسائل

**مسئلہ**: زیادہ CPU استعمال
- **سبب**: غیر موثر پروسیسنگ، بڑے بفرز، یا زیادہ پری پروسیسنگ
- **حل**::
  1. آڈیو پروسیسنگ کی فریکوینسی کم کریں
  2. پری پروسیسنگ الگورتھم کو اصلاح کریں
  3. زیادہ کارآمد ڈیٹا سٹرکچر استعمال کریں
  4. ہارڈویئر ایکسیلریشن پر غور کریں

**مسئلہ**: میموری لیکس
- **سبب**: غیر جاری کردہ آڈیو بفرز یا پروسیسنگ آبجیکٹس
- **حل**::
  1. مناسب ریسورس کلین اپ لاگو کریں
  2. مناسب جگہوں پر کنٹیکسٹ مینیجرز استعمال کریں
  3. آپریشن کے دوران میموری استعمال کو مانیٹر کریں
  4. بفر سائز کی حدیں لاگو کریں

### ڈیبگنگ کی تکنیکس

```python
# مثال ڈیبگنگ یوٹیلیٹیز
class VoiceProcessingDebugger:
    def __init__(self):
        self.metrics = {
            'audio_chunks_processed': 0,
            'recognition_attempts': 0,
            'successful_recognitions': 0,
            'command_parsing_success': 0,
            'average_processing_time': 0
        }
        self.processing_times = []

    def log_audio_chunk(self, chunk_size):
        """آڈیو چنک پروسیسنگ لاگ کریں"""
        self.metrics['audio_chunks_processed'] += 1
        print(f"سائز کا آڈیو چنک پروسیس کیا گیا: {chunk_size}")

    def log_recognition_attempt(self, processing_time):
        """ریکوگنیشن کی کوشش لاگ کریں"""
        self.metrics['recognition_attempts'] += 1
        self.processing_times.append(processing_time)

        # اوسط پروسیسنگ ٹائم اپ ڈیٹ کریں
        if self.processing_times:
            avg_time = sum(self.processing_times) / len(self.processing_times)
            self.metrics['average_processing_time'] = avg_time

    def log_recognition_result(self, success):
        """ریکوگنیشن کا نتیجہ لاگ کریں"""
        if success:
            self.metrics['successful_recognitions'] += 1

    def log_command_parsing(self, success):
        """کمانڈ پارسنگ کا نتیجہ لاگ کریں"""
        if success:
            self.metrics['command_parsing_success'] += 1

    def get_metrics(self):
        """موجودہ میٹرکس حاصل کریں"""
        accuracy = (self.metrics['successful_recognitions'] /
                   max(self.metrics['recognition_attempts'], 1)) * 100
        parsing_success_rate = (self.metrics['command_parsing_success'] /
                               max(self.metrics['successful_recognitions'], 1)) * 100

        return {
            **self.metrics,
            'recognition_accuracy': accuracy,
            'command_parsing_success_rate': parsing_success_rate
        }

# پائپ لائن میں مثال استعمال
class DebuggableVoiceProcessingPipeline(VoiceProcessingPipeline):
    def __init__(self, api_key: str):
        super().__init__(api_key)
        self.debugger = VoiceProcessingDebugger()

    def process_audio_chunk(self, audio_chunk: np.ndarray):
        """ڈیبگنگ کے ساتھ آڈیو چنک پروسیس کریں"""
        import time

        start_time = time.time()
        self.debugger.log_audio_chunk(len(audio_chunk))

        # والدین میتھڈ کال کریں
        super().process_audio_chunk(audio_chunk)

        processing_time = time.time() - start_time
        self.debugger.log_recognition_attempt(processing_time)

    def get_debug_metrics(self):
        """ڈیبگنگ میٹرکس حاصل کریں"""
        return self.debugger.get_metrics()
```

### ٹربل شوٹنگ چیک لسٹ

اپنے وائس پروسیسنگ سسٹم کو ڈیپلوئے کرنے سے پہلے، اس چیک لسٹ کا استعمال کریں::

- [ ] آڈیو ان پٹ ڈیوائس مناسب طریقے سے منسلک اور کنفیگر کیا گیا
- [ ] مائیکروفون کی اجازتیں منظور کی گئیں
- [ ] OpenAI API کلید صحیح طریقے سے کنفیگر کی گئی (اگر وہیسپر API استعمال کر رہے ہیں)
- [ ] ماحول کے لیے آڈیو پری پروسیسنگ پیرامیٹرز ٹیون کیے گئے
- [ ] ریکوگنیشن یقین کی حدیں مناسب طریقے سے سیٹ کی گئیں
- [ ] کمانڈ پارسنگ پیٹرن امکانی کمانڈز کو احاطہ کرتے ہیں
- [ ] سسٹم کارکردگی ریل ٹائم تقاضوں کو پورا کرتی ہے
- [ ] تمام جزوات کے لیے خرابی کا انتظام لاگو کیا گیا
- [ ] ڈیبگنگ کے لیے لاگنگ فعال
- [ ] بیک اپ ریکوگنیشن طریقہ دستیاب (اگر ضرورت ہو)

## سیٹ اپ ہدایات

### انسٹالیشن کی ضروریات

```bash
# درکار پیکجز انسٹال کریں
pip install openai pyaudio scipy librosa pydub vosk numpy rclpy

# آڈیو پروسیسنگ کے لیے
sudo apt-get update
sudo apt-get install portaudio19-dev python3-pyaudio
```

### کنفیگریشن

اپنے وائس پروسیسنگ سسٹم کے لیے ایک کنفیگریشن فائل بنائیں::

```yaml
# config/voice_processing.yaml
voice_processing:
  audio:
    sample_rate: 44100
    channels: 1
    chunk_size: 1024
    format: paInt16

  preprocessing:
    noise_reduction_enabled: true
    vad_threshold: 0.01
    frame_length: 1024
    hop_length: 512

  recognition:
    service: "openai_whisper"  # یا "vosk_local"
    language: "en"
    api_key: "${OPENAI_API_KEY}"
    model_path: "/path/to/vosk/model"  # مقامی ریکوگنیشن کے لیے

  commands:
    confidence_threshold: 0.7
    max_audio_duration: 30  # سیکنڈ
    silence_duration: 500   # ملی سیکنڈ
```

### لانچ کنفیگریشن

```xml
<!-- launch/voice_processing.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="voice_processing_node" name="voice_processing">
    <param name="openai_api_key" value="$(var openai_api_key)"/>
  </node>
</launch>
```

## اگلے اقدامات

اگلے سیکشن میں، ہم کمانڈز کو پروسیس کرنے اور انہیں کمپیوٹر وژن کے ساتھ منسلک کرنے کے لیے زبان کی سمجھ بوجھ کو انٹیگریٹ کریں گے ایک مکمل ملٹی ماڈل سسٹم کے لیے۔

جاری رکھیں [لینگویج اندراج](../language-understanding/index.md).