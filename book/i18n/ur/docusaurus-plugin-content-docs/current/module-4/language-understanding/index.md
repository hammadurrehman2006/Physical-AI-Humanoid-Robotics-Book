---
sidebar_position: 2
title: "زبان کی سمجھ بوجھ"
---

# زبان کی سمجھ بوجھ

اس سیکشن میں، ہم اپنے وژن لینگویج ایکشن سسٹم کے زبان کی سمجھ بوجھ کے جزو کو ترقی دیں گے۔ یہ ماڈیول قدرتی زبان کے ان پٹ کو پروسیس کرتا ہے تاکہ معنی، ارادہ، اور عمل کے قابل کمانڈز نکالے جا سکیں جنہیں وژوئل معلومات کے ساتھ جوڑا جا سکے اور روبوٹ ایکشنز کے طور پر انجام دیا جا سکے۔

## جائزہ

زبان کی سمجھ بوجھ انسانی مواصلت اور روبوٹ ایکشن کے درمیان پل ہے۔ یہ سیکشن یہ احاطہ کرتا ہے::
- کمانڈ تشریح کے لیے قدرتی زبان کی پروسیسنگ
- صارف کے کمانڈز سے ارادہ اور ادارت نکالنا
- سیاقی سمجھ بوجھ اور سیمینٹک پارسنگ
- زمینی زبان کی سمجھ بوجھ کے لیے کمپیوٹر وژن کے ساتھ انٹیگریشن
- ابہام کے کمانڈز کے لیے خرابی کا انتظام اور یقین کا اسکورنگ

## سیکھنے کے اہداف

اس سیکشن کے اختتام تک، آپ کے اہل ہوگا::
- روبوٹ کمانڈ تشریح کے لیے قدرتی زبان کی پروسیسنگ پائپ لائنز امپلیمنٹ کریں
- صارف کے کمانڈز سے ارادہ اور ادارت مناسب یقین اسکورز کے ساتھ نکالیں
- ایسے سیاقی سمجھ بوجھ کے سسٹم تخلیق کریں جو روبوٹ کی حالت اور ماحول پر غور کریں
- زمینی تشریح کے لیے وژوئل ادراک کے ساتھ زبان کی سمجھ بوجھ کو انٹیگریٹ کریں
- ابہام یا غیر واضح زبان کے کمانڈز کو بخوبی ہینڈل کریں

## قدرتی زبان کی پروسیسنگ پائپ لائن

### ٹیکسٹ پری پروسیسنگ

قدرتی زبان کے کمانڈز کو پروسیس کرنے سے پہلے، ہمیں ان پٹ ٹیکسٹ کو صاف اور نارملائز کرنا ہوگا::

```python
import re
import string
from typing import List, Dict, Any
from dataclasses import dataclass

@dataclass
class ParsedCommand:
    intent: str
    entities: Dict[str, Any]
    confidence: float
    original_text: str
    processed_text: str

class TextPreprocessor:
    def __init__(self):
        self.stop_words = {
            'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
            'of', 'with', 'by', 'from', 'up', 'about', 'into', 'through', 'during',
            'before', 'after', 'above', 'below', 'between', 'among', 'out', 'off'
        }

    def preprocess(self, text: str) -> str:
        """ان پٹ ٹیکسٹ کو صاف اور نارملائز کریں"""
        # لوور کیس میں تبدیل کریں
        text = text.lower()

        # اضافی وائٹ سپیس ہٹائیں
        text = re.sub(r'\s+', ' ', text).strip()

        # نشان ہٹائیں (اعداد اور بنیادی کمانڈ لفظوں کے علاوہ)
        text = re.sub(r'[^\w\s]', ' ', text)

        # اسٹاپ ورڈز ہٹائیں (اختیاری، صاف پروسیسنگ کے لیے)
        words = text.split()
        filtered_words = [word for word in words if word not in self.stop_words]

        return ' '.join(filtered_words)

    def normalize_numbers(self, text: str) -> str:
        """نمبر کے الفاظ کو ہندسوں میں تبدیل کریں"""
        number_map = {
            'one': '1', 'two': '2', 'three': '3', 'four': '4', 'five': '5',
            'six': '6', 'seven': '7', 'eight': '8', 'nine': '9', 'ten': '10',
            'first': '1', 'second': '2', 'third': '3', 'fourth': '4', 'fifth': '5'
        }

        for word, digit in number_map.items():
            text = re.sub(r'\b' + word + r'\b', digit, text)

        return text
```

### ارادہ کیسٹیفکیشن

ارادہ کیسٹیفکیشن یہ طے کرتا ہے کہ صارف چاہتا ہے کہ روبوٹ کیا کرے::

```python
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.naive_bayes import MultinomialNB
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
import joblib

class IntentClassifier:
    def __init__(self):
        self.pipeline = Pipeline([
            ('tfidf', TfidfVectorizer(ngram_range=(1, 2), max_features=1000)),
            ('classifier', MultinomialNB())
        ])
        self.is_trained = False

        # عام روبوٹ ارادے کی وضاحت کریں
        self.intents = {
            'move': ['move', 'go', 'walk', 'navigate', 'drive', 'travel'],
            'grasp': ['grasp', 'pick', 'grab', 'take', 'hold', 'lift'],
            'place': ['place', 'put', 'set', 'drop', 'release'],
            'navigate': ['navigate', 'go to', 'move to', 'find', 'locate'],
            'identify': ['identify', 'find', 'show', 'point to', 'locate'],
            'follow': ['follow', 'come after', 'accompany', 'go behind'],
            'stop': ['stop', 'halt', 'pause', 'wait', 'freeze'],
            'greet': ['hello', 'hi', 'greet', 'wave', 'acknowledge'],
            'answer': ['what', 'how', 'why', 'when', 'where', 'question']
        }

    def train(self, training_data: List[tuple]):
        """لیبل والے مثالوں کے ساتھ ارادہ کلاسیفائر کو تربیت دیں"""
        texts, labels = zip(*training_data)
        self.pipeline.fit(texts, labels)
        self.is_trained = True

    def predict(self, text: str) -> tuple:
        """دیے گئے ٹیکسٹ کے لیے ارادہ اور یقین کی پیشن گوئی کریں"""
        if not self.is_trained:
            # تربیت نہ ہونے کی صورت میں کی ورڈ میچنگ کا استعمال کریں
            return self._keyword_match_intent(text)

        prediction = self.pipeline.predict([text])[0]
        probabilities = self.pipeline.predict_proba([text])[0]
        confidence = max(probabilities)

        return prediction, confidence

    def _keyword_match_intent(self, text: str) -> tuple:
        """واپسی کے لیے سادہ کی ورڈ بیسڈ ارادہ میچنگ"""
        text_lower = text.lower()

        best_match = 'unknown'
        best_score = 0

        for intent, keywords in self.intents.items():
            score = sum(1 for keyword in keywords if keyword in text_lower)
            if score > best_score:
                best_score = score
                best_match = intent

        confidence = min(0.9, best_score * 0.2)  # کی ورڈ میچز کی بنیاد پر یقین کو اسکیل کریں
        return best_match, confidence
```

### ادارت نکالنا

ادارت نکالنا کمانڈز میں ذکر کردہ مخصوص اشیاء، مقامات، اور پیرامیٹرز کی شناخت کرتا ہے::

```python
import re
from typing import Dict, List, Tuple

class EntityExtractor:
    def __init__(self):
        # ادارت کے پیٹرنز کی وضاحت کریں
        self.patterns = {
            'object': r'\b(red|blue|green|yellow|large|small|big|tiny|ball|cup|box|book|bottle|apple|orange|toy|object|item)\b',
            'location': r'\b(kitchen|bedroom|living room|office|hall|garage|garden|table|shelf|cabinet|counter)\b',
            'direction': r'\b(forward|backward|left|right|up|down|north|south|east|west)\b',
            'distance': r'\b(\d+(?:\.\d+)?)\s*(meter|foot|inch|cm|m)\b',
            'color': r'\b(red|blue|green|yellow|purple|orange|pink|black|white|brown|gray|grey)\b',
            'size': r'\b(small|large|big|tiny|medium|tall|short|wide|narrow)\b'
        }

    def extract_entities(self, text: str) -> Dict[str, List[str]]:
        """ریجیکس پیٹرنز کا استعمال کرتے ہوئے ٹیکسٹ سے ادارت نکالیں"""
        entities = {}

        for entity_type, pattern in self.patterns.items():
            matches = re.findall(pattern, text, re.IGNORECASE)
            if matches:
                entities[entity_type] = list(set(matches))  # ڈوپلیکیٹس ہٹائیں

        return entities

    def extract_quantities(self, text: str) -> Dict[str, float]:
        """ٹیکسٹ سے عددی مقداریں نکالیں"""
        quantities = {}

        # فاصلہ کی پیمائش
        distance_pattern = r'(\d+(?:\.\d+)?)\s*(meter|foot|inch|cm|m|ft)'
        distances = re.findall(distance_pattern, text, re.IGNORECASE)
        if distances:
            for value, unit in distances:
                # میٹر میں تبدیل کریں
                meters = float(value)
                if unit.lower() in ['foot', 'ft']:
                    meters *= 0.3048
                elif unit.lower() == 'cm':
                    meters *= 0.01
                elif unit.lower() == 'inch':
                    meters *= 0.0254
                quantities['distance_meters'] = meters

        # شمار کی مقداریں
        count_pattern = r'(\d+)\s+(?:times|times|repetitions?)'
        counts = re.findall(count_pattern, text)
        if counts:
            quantities['count'] = int(counts[0])

        return quantities
```

## سیاقی سمجھ بوجھ

### روبوٹ کی حالت کا انٹیگریشن

روبوٹ کی حالت کا انٹیگریشن کمانڈز کو الگ کرنے میں مدد کرتا ہے::

```python
from dataclasses import dataclass
from typing import Optional
import math

@dataclass
class RobotState:
    position: tuple  # (x, y, z)
    orientation: tuple  # (roll, pitch, yaw)
    battery_level: float
    gripper_state: str  # 'open', 'closed', 'holding'
    current_task: Optional[str] = None
    last_action: Optional[str] = None

class ContextualInterpreter:
    def __init__(self):
        self.robot_state = RobotState(
            position=(0, 0, 0),
            orientation=(0, 0, 0),
            battery_level=1.0,
            gripper_state='open'
        )
        self.object_locations = {}  # object_id -> (x, y, z)

    def update_robot_state(self, new_state: RobotState):
        """روبوٹ کی موجودہ حالت کو اپ ڈیٹ کریں"""
        self.robot_state = new_state

    def update_object_locations(self, locations: Dict[str, tuple]):
        """معلوم اشیاء کے مقامات کو اپ ڈیٹ کریں"""
        self.object_locations = {**self.object_locations, **locations}

    def resolve_references(self, entities: Dict[str, List[str]], text: str) -> Dict[str, List[str]]:
        """سیاق کا استعمال کرتے ہوئے ابہام کے حوالہ جات کو حل کریں"""
        resolved_entities = entities.copy()

        # سیاق کے مطابق "it"، "that"، "there" کو حل کریں
        if 'object' in entities:
            objects = entities['object']
            for obj in objects:
                # اگر یہ ایک رنگ ہے، تو اس رنگ کی اشیاء تلاش کریں
                if obj in ['red', 'blue', 'green', 'yellow']:
                    matching_objects = [
                        obj_id for obj_id, loc in self.object_locations.items()
                        if obj in obj_id.lower()
                    ]
                    if matching_objects:
                        resolved_entities['target_object'] = matching_objects
                        break

        # سپیشل حوالہ جات کو حل کریں
        if 'location' in entities:
            location = entities['location'][0]
            # اس مقام کے قریب ترین شے تلاش کریں
            closest_obj = self._find_closest_object_to_location(location)
            if closest_obj:
                resolved_entities['target_object'] = [closest_obj]

        return resolved_entities

    def _find_closest_object_to_location(self, location: str) -> Optional[str]:
        """دیے گئے مقام کے قریب ترین شے تلاش کریں"""
        # یہ مقامات کو کوآرڈینیٹس میں سیمینٹک میپنگ استعمال کرے گا
        # فی الحال، ایک مocker امپلیمنٹ لوٹائیں
        location_coords = {
            'kitchen': (2, 2, 0),
            'bedroom': (-2, 2, 0),
            'office': (0, -2, 0)
        }

        if location.lower() not in location_coords:
            return None

        target_pos = location_coords[location.lower()]
        closest_obj = None
        min_distance = float('inf')

        for obj_id, obj_pos in self.object_locations.items():
            dist = math.sqrt(
                (obj_pos[0] - target_pos[0])**2 +
                (obj_pos[1] - target_pos[1])**2 +
                (obj_pos[2] - target_pos[2])**2
            )
            if dist < min_distance:
                min_distance = dist
                closest_obj = obj_id

        return closest_obj
```

## سیمینٹک پارسر

### کمانڈ تشریح

سیمینٹک پارسر ارادہ کیسٹیفکیشن اور ادارت نکالنے کو جوڑتا ہے::

```python
class SemanticParser:
    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.entity_extractor = EntityExtractor()
        self.contextual_interpreter = ContextualInterpreter()

        # ارادہ کیسٹیفکیشن کے لیے تربیتی ڈیٹا (سادہ مثال)
        self.training_data = [
            ("move forward", "move"),
            ("go left", "move"),
            ("move backward", "move"),
            ("go to kitchen", "navigate"),
            ("navigate to table", "navigate"),
            ("find red ball", "identify"),
            ("pick up cup", "grasp"),
            ("grab the object", "grasp"),
            ("put down", "place"),
            ("place on shelf", "place"),
            ("stop moving", "stop"),
            ("what time is it", "answer")
        ]

        # کلاسیفائر کو تربیت دیں
        self.intent_classifier.train(self.training_data)

    def parse_command(self, text: str) -> ParsedCommand:
        """سٹرکچرڈ فارمیٹ میں قدرتی زبان کا کمانڈ پارس کریں"""
        preprocessor = TextPreprocessor()
        processed_text = preprocessor.preprocess(text)

        # ارادہ کیسٹیفکیشن
        intent, intent_confidence = self.intent_classifier.predict(processed_text)

        # ادارت نکالیں
        entities = self.entity_extractor.extract_entities(text)
        quantities = self.entity_extractor.extract_quantities(text)

        # ادارت اور مقداریں جوڑیں
        all_entities = {**entities, **quantities}

        # سیاقی حوالہ جات کو حل کریں
        resolved_entities = self.contextual_interpreter.resolve_references(all_entities, text)

        # مجموعی یقین کا حساب لگائیں
        confidence = intent_confidence * 0.7 + 0.3  # ادارت نکالنے کی بنیاد پر بوسٹ کریں

        return ParsedCommand(
            intent=intent,
            entities=resolved_entities,
            confidence=confidence,
            original_text=text,
            processed_text=processed_text
        )

    def update_context(self, robot_state: RobotState, object_locations: Dict[str, tuple]):
        """تشریح کے لیے سیاقی معلومات کو اپ ڈیٹ کریں"""
        self.contextual_interpreter.update_robot_state(robot_state)
        self.contextual_interpreter.update_object_locations(object_locations)
```

## زبان کی سمجھ بوجھ کا نوڈ

### مکمل ROS 2 امپلیمنٹیشن

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
from typing import Dict, Any
import json

class LanguageUnderstandingNode(Node):
    def __init__(self):
        super().__init__('language_understanding_node')

        # پبلشرز اور سبسکرائبرز
        self.command_pub = self.create_publisher(String, 'parsed_commands', 10)
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10
        )

        # سیمینٹک پارسر کو شروع کریں
        self.parser = SemanticParser()

        # روبوٹ کی حالت کا ٹریکنگ
        self.robot_state = RobotState(
            position=(0, 0, 0),
            orientation=(0, 0, 0),
            battery_level=1.0,
            gripper_state='open'
        )

        # اشیاء کے مقامات کا ٹریکنگ
        self.object_locations = {}

        self.get_logger().info("زبان کی سمجھ بوجھ کا نوڈ شروع ہو گیا")

    def voice_callback(self, msg: String):
        """آنے والے وائس کمانڈز کو پروسیس کریں"""
        try:
            # کمانڈ کو پارس کریں
            parsed_command = self.parser.parse_command(msg.data)

            # یقین کی حد چیک کریں
            if parsed_command.confidence < 0.5:
                self.get_logger().warn(
                    f"کم یقین والے کمانڈ: {parsed_command.original_text} "
                    f"(یقین: {parsed_command.confidence:.2f})"
                )
                return

            # کمانڈ میسج بنائیں
            command_msg = String()
            command_dict = {
                'intent': parsed_command.intent,
                'entities': parsed_command.entities,
                'confidence': parsed_command.confidence,
                'original_text': parsed_command.original_text
            }
            command_msg.data = json.dumps(command_dict)

            # پارسڈ کمانڈ شائع کریں
            self.command_pub.publish(command_msg)
            self.get_logger().info(
                f"پارسڈ کمانڈ: {parsed_command.intent} "
                f"یقین کے ساتھ {parsed_command.confidence:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"کمانڈ کو پارس کرنے میں خرابی: {e}")

    def update_robot_state(self, position: tuple, orientation: tuple,
                          battery: float, gripper_state: str):
        """سیاقی سمجھ بوجھ کے لیے روبوٹ کی حالت کو اپ ڈیٹ کریں"""
        self.robot_state = RobotState(
            position=position,
            orientation=orientation,
            battery_level=battery,
            gripper_state=gripper_state
        )
        self.parser.update_context(self.robot_state, self.object_locations)

    def update_object_locations(self, locations: Dict[str, tuple]):
        """معلوم اشیاء کے مقامات کو اپ ڈیٹ کریں"""
        self.object_locations = locations
        self.parser.update_context(self.robot_state, self.object_locations)

def main(args=None):
    rclpy.init(args=args)

    node = LanguageUnderstandingNode()

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

## اعلی درجے کی زبان کی سمجھ بوجھ

### ابہام کو ہینڈل کرنا

```python
class AmbiguityResolver:
    def __init__(self):
        self.ambiguity_patterns = {
            'multiple_objects': r'pick up the (red|blue|green) (ball|cup|box)',
            'spatial_reference': r'pick up (that|it|the one)',
            'temporal_reference': r'do (that|it) again'
        }

    def detect_ambiguity(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """پارسڈ کمانڈ میں ممکنہ ابہام کا پتہ لگائیں"""
        ambiguity_info = {
            'type': [],
            'details': [],
            'resolution_needed': False
        }

        # متعدد ممکنہ اشیاء کے لیے چیک کریں
        if 'object' in parsed_command.entities and len(parsed_command.entities['object']) > 1:
            ambiguity_info['type'].append('multiple_objects')
            ambiguity_info['details'].append(
                f"متعدد اشیاء کی شناخت: {parsed_command.entities['object']}"
            )
            ambiguity_info['resolution_needed'] = True

        # غیر واضح سپیشل حوالہ جات کے لیے چیک کریں
        vague_refs = ['that', 'it', 'the one', 'there', 'over there']
        if any(ref in parsed_command.original_text.lower() for ref in vague_refs):
            ambiguity_info['type'].append('spatial_reference')
            ambiguity_info['details'].append("غیر واضح سپیشل حوالہ کا پتہ چلا")
            ambiguity_info['resolution_needed'] = True

        return ambiguity_info

    def request_clarification(self, ambiguity_info: Dict[str, Any]) -> str:
        """ابہام کی قسم کی بنیاد پر وضاحت کی درخواست تیار کریں"""
        if not ambiguity_info['resolution_needed']:
            return ""

        clarification_requests = {
            'multiple_objects': "میں کون سی مخصوص شے سے بات چیت کرنا چاہوں گا؟",
            'spatial_reference': "کیا آپ اشارہ کر سکتے ہیں یا زیادہ مخصوص کر سکتے ہیں کہ آپ کون سی چیز کا مطلب ہے؟",
            'temporal_reference': "کیا آپ کمانڈ کو زیادہ مخصوص طور پر دہرائیں گے؟"
        }

        request = ""
        for amb_type in ambiguity_info['type']:
            if amb_type in clarification_requests:
                request += clarification_requests[amb_type] + " "

        return request.strip()
```

## ٹیسٹنگ اور تصدیق

### یونٹ ٹیسٹس

```python
import unittest
from unittest.mock import Mock

class TestLanguageUnderstanding(unittest.TestCase):
    def setUp(self):
        self.parser = SemanticParser()

    def test_intent_classification(self):
        """ارادہ کیسٹیفکیشن کی درستگی کا ٹیسٹ کریں"""
        test_cases = [
            ("move forward", "move"),
            ("go to kitchen", "navigate"),
            ("pick up the red ball", "grasp"),
            ("what time is it", "answer")
        ]

        for text, expected_intent in test_cases:
            result = self.parser.parse_command(text)
            self.assertEqual(result.intent, expected_intent)

    def test_entity_extraction(self):
        """کمانڈز سے ادارت نکالنے کا ٹیسٹ کریں"""
        text = "pick up the red ball from the kitchen"
        result = self.parser.parse_command(text)

        self.assertIn('object', result.entities)
        self.assertIn('location', result.entities)
        self.assertIn('red', result.entities['object'])
        self.assertIn('kitchen', result.entities['location'])

    def test_quantity_extraction(self):
        """عددی مقدار نکالنے کا ٹیسٹ کریں"""
        text = "move forward 2 meters"
        result = self.parser.parse_command(text)

        self.assertIn('distance_meters', result.entities)
        self.assertEqual(result.entities['distance_meters'], 2.0)

    def test_contextual_resolution(self):
        """سیاقی حوالہ حل کرنا کا ٹیسٹ کریں"""
        # مocker سیاق سیٹ اپ کریں
        robot_state = RobotState(
            position=(0, 0, 0),
            orientation=(0, 0, 0),
            battery_level=1.0,
            gripper_state='open'
        )

        object_locations = {
            'red_ball': (1, 1, 0),
            'blue_cup': (2, 2, 0),
            'green_box': (3, 3, 0)
        }

        self.parser.update_context(robot_state, object_locations)

        # رنگ بیسڈ اشیاء کے حل کا ٹیسٹ کریں
        text = "pick up the red one"
        result = self.parser.parse_command(text)

        # سیاق کی بنیاد پر red_ball کو حل کرنا چاہیے
        if 'target_object' in result.entities:
            self.assertIn('red_ball', result.entities['target_object'])

if __name__ == '__main__':
    unittest.main()
```

## کنفیگریشن اور سیٹ اپ

### کنفیگریشن فائل

```yaml
# config/language_understanding.yaml
language_understanding:
  preprocessing:
    remove_stop_words: true
    normalize_numbers: true
    min_confidence: 0.5

  classification:
    model_path: "/path/to/trained/model"
    feature_extractor: "tfidf"
    ngram_range: [1, 2]
    max_features: 1000

  entities:
    extract_objects: true
    extract_locations: true
    extract_quantities: true
    extract_colors: true
    extract_sizes: true

  contextual:
    enable_resolution: true
    ambiguity_detection: true
    clarification_threshold: 0.6
```

### لانچ فائل

```xml
<!-- launch/language_understanding.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="language_understanding_node" name="language_understanding">
    <param name="model_path" value="$(var model_path)"/>
    <param name="min_confidence" value="0.5"/>
  </node>
</launch>
```

## ٹربل شوٹنگ

### عام مسائل

1. **کم ارادہ کیسٹیفکیشن کی درستگی**
   - حل: مخصوص روبوٹ کمانڈز کے لیے مزید تربیتی مثالیں شامل کریں
   - چیک کریں: یقینی بنائیں کہ تربیتی ڈیٹا اصل استعمال کے کیسز کو احاطہ کرتا ہے

2. **غریب ادارت نکالنا**
   - حل: اپنے مخصوص ڈومین کے لیے ریجیکس پیٹرنز کو بڑھائیں
   - چیک کریں: یقینی بنائیں کہ ادارت پیٹرنز آپ کے کمانڈ لغت سے مماثل ہیں

3. **زیادہ ابہام کی شرح**
   - حل: بہتر سیاقی حل کو امپلیمنٹ کریں
   - چیک کریں: یقینی بنائیں کہ روبوٹ کی حالت اور اشیاء کے مقامات کا ڈیٹا موجود ہے

4. **کارکردگی کے مسائل**
   - حل: فیچر ایکسٹریکشن اور کلاسیفکیشن کو اصلاح کریں
   - چیک کریں: ریل ٹائم پروسیسنگ کے لیے ہلکے ماڈلز استعمال کریں

## اگلے اقدامات

اگلے سیکشن میں، ہم کمپیوٹر وژن سسٹم امپلیمنٹ کریں گے تاکہ ہمارے وژن لینگویج ایکشن فریم ورک کے وژوئل ادراک کے جزو کو فراہم کیا جا سکے۔ ہم نے جو زبان کی سمجھ بوجھ کا سسٹم تعمیر کیا ہے اسے وژوئل سیاق کے ساتھ بہتر بنایا جائے گا تاکہ زیادہ مضبوط کمانڈ تشریح تیار کی جا سکے۔

جاری رکھیں [کمپیوٹر وژن انٹیگریشن](../computer-vision/index.md).