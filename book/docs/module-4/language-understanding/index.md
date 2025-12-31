---
sidebar_position: 2
title: "Language Understanding"
---

# Language Understanding

In this section, we'll develop the language understanding component of our Vision-Language-Action system. This module processes natural language input to extract meaning, intent, and actionable commands that can be combined with visual information and executed as robot actions.

## Overview

Language understanding is the bridge between human communication and robot action. This section covers:
- Natural language processing for command interpretation
- Intent and entity extraction from user commands
- Contextual understanding and semantic parsing
- Integration with computer vision for grounded language understanding
- Error handling and confidence scoring for ambiguous commands

## Learning Objectives

By the end of this section, you will be able to:
- Implement natural language processing pipelines for robot command interpretation
- Extract intents and entities from user commands with appropriate confidence scores
- Create contextual understanding systems that consider robot state and environment
- Integrate language understanding with visual perception for grounded interpretation
- Handle ambiguous or unclear language commands gracefully

## Natural Language Processing Pipeline

### Text Preprocessing

Before processing natural language commands, we need to clean and normalize the input text:

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
        """Clean and normalize input text"""
        # Convert to lowercase
        text = text.lower()

        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text).strip()

        # Remove punctuation (except for numbers and basic command words)
        text = re.sub(r'[^\w\s]', ' ', text)

        # Remove stop words (optional, for cleaner processing)
        words = text.split()
        filtered_words = [word for word in words if word not in self.stop_words]

        return ' '.join(filtered_words)

    def normalize_numbers(self, text: str) -> str:
        """Convert number words to digits"""
        number_map = {
            'one': '1', 'two': '2', 'three': '3', 'four': '4', 'five': '5',
            'six': '6', 'seven': '7', 'eight': '8', 'nine': '9', 'ten': '10',
            'first': '1', 'second': '2', 'third': '3', 'fourth': '4', 'fifth': '5'
        }

        for word, digit in number_map.items():
            text = re.sub(r'\b' + word + r'\b', digit, text)

        return text
```

### Intent Classification

Intent classification determines what the user wants the robot to do:

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

        # Define common robot intents
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
        """Train the intent classifier with labeled examples"""
        texts, labels = zip(*training_data)
        self.pipeline.fit(texts, labels)
        self.is_trained = True

    def predict(self, text: str) -> tuple:
        """Predict intent and confidence for given text"""
        if not self.is_trained:
            # Fallback to keyword matching if not trained
            return self._keyword_match_intent(text)

        prediction = self.pipeline.predict([text])[0]
        probabilities = self.pipeline.predict_proba([text])[0]
        confidence = max(probabilities)

        return prediction, confidence

    def _keyword_match_intent(self, text: str) -> tuple:
        """Simple keyword-based intent matching as fallback"""
        text_lower = text.lower()

        best_match = 'unknown'
        best_score = 0

        for intent, keywords in self.intents.items():
            score = sum(1 for keyword in keywords if keyword in text_lower)
            if score > best_score:
                best_score = score
                best_match = intent

        confidence = min(0.9, best_score * 0.2)  # Scale confidence based on keyword matches
        return best_match, confidence
```

### Entity Extraction

Entity extraction identifies specific objects, locations, and parameters mentioned in commands:

```python
import re
from typing import Dict, List, Tuple

class EntityExtractor:
    def __init__(self):
        # Define entity patterns
        self.patterns = {
            'object': r'\b(red|blue|green|yellow|large|small|big|tiny|ball|cup|box|book|bottle|apple|orange|toy|object|item)\b',
            'location': r'\b(kitchen|bedroom|living room|office|hall|garage|garden|table|shelf|cabinet|counter)\b',
            'direction': r'\b(forward|backward|left|right|up|down|north|south|east|west)\b',
            'distance': r'\b(\d+(?:\.\d+)?)\s*(meter|foot|inch|cm|m)\b',
            'color': r'\b(red|blue|green|yellow|purple|orange|pink|black|white|brown|gray|grey)\b',
            'size': r'\b(small|large|big|tiny|medium|tall|short|wide|narrow)\b'
        }

    def extract_entities(self, text: str) -> Dict[str, List[str]]:
        """Extract entities from text using regex patterns"""
        entities = {}

        for entity_type, pattern in self.patterns.items():
            matches = re.findall(pattern, text, re.IGNORECASE)
            if matches:
                entities[entity_type] = list(set(matches))  # Remove duplicates

        return entities

    def extract_quantities(self, text: str) -> Dict[str, float]:
        """Extract numerical quantities from text"""
        quantities = {}

        # Distance measurements
        distance_pattern = r'(\d+(?:\.\d+)?)\s*(meter|foot|inch|cm|m|ft)'
        distances = re.findall(distance_pattern, text, re.IGNORECASE)
        if distances:
            for value, unit in distances:
                # Convert to meters
                meters = float(value)
                if unit.lower() in ['foot', 'ft']:
                    meters *= 0.3048
                elif unit.lower() == 'cm':
                    meters *= 0.01
                elif unit.lower() == 'inch':
                    meters *= 0.0254
                quantities['distance_meters'] = meters

        # Count quantities
        count_pattern = r'(\d+)\s+(?:times|times|repetitions?)'
        counts = re.findall(count_pattern, text)
        if counts:
            quantities['count'] = int(counts[0])

        return quantities
```

## Contextual Understanding

### Robot State Integration

Integrating robot state helps disambiguate commands:

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
        """Update the robot's current state"""
        self.robot_state = new_state

    def update_object_locations(self, locations: Dict[str, tuple]):
        """Update known object locations"""
        self.object_locations = {**self.object_locations, **locations}

    def resolve_references(self, entities: Dict[str, List[str]], text: str) -> Dict[str, List[str]]:
        """Resolve ambiguous references using context"""
        resolved_entities = entities.copy()

        # Resolve "it", "that", "there" based on context
        if 'object' in entities:
            objects = entities['object']
            for obj in objects:
                # If it's a color, find objects of that color
                if obj in ['red', 'blue', 'green', 'yellow']:
                    matching_objects = [
                        obj_id for obj_id, loc in self.object_locations.items()
                        if obj in obj_id.lower()
                    ]
                    if matching_objects:
                        resolved_entities['target_object'] = matching_objects
                        break

        # Resolve spatial references
        if 'location' in entities:
            location = entities['location'][0]
            # Find closest object at that location
            closest_obj = self._find_closest_object_to_location(location)
            if closest_obj:
                resolved_entities['target_object'] = [closest_obj]

        return resolved_entities

    def _find_closest_object_to_location(self, location: str) -> Optional[str]:
        """Find the closest object to a given location"""
        # This would use semantic mapping of locations to coordinates
        # For now, return a mock implementation
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

## Semantic Parser

### Command Interpretation

The semantic parser combines intent classification and entity extraction:

```python
class SemanticParser:
    def __init__(self):
        self.intent_classifier = IntentClassifier()
        self.entity_extractor = EntityExtractor()
        self.contextual_interpreter = ContextualInterpreter()

        # Training data for intent classification (simplified example)
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

        # Train the classifier
        self.intent_classifier.train(self.training_data)

    def parse_command(self, text: str) -> ParsedCommand:
        """Parse a natural language command into structured format"""
        preprocessor = TextPreprocessor()
        processed_text = preprocessor.preprocess(text)

        # Classify intent
        intent, intent_confidence = self.intent_classifier.predict(processed_text)

        # Extract entities
        entities = self.entity_extractor.extract_entities(text)
        quantities = self.entity_extractor.extract_quantities(text)

        # Combine entities and quantities
        all_entities = {**entities, **quantities}

        # Resolve contextual references
        resolved_entities = self.contextual_interpreter.resolve_references(all_entities, text)

        # Calculate overall confidence
        confidence = intent_confidence * 0.7 + 0.3  # Boost based on entity extraction

        return ParsedCommand(
            intent=intent,
            entities=resolved_entities,
            confidence=confidence,
            original_text=text,
            processed_text=processed_text
        )

    def update_context(self, robot_state: RobotState, object_locations: Dict[str, tuple]):
        """Update contextual information for interpretation"""
        self.contextual_interpreter.update_robot_state(robot_state)
        self.contextual_interpreter.update_object_locations(object_locations)
```

## Language Understanding Node

### Complete ROS 2 Implementation

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

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, 'parsed_commands', 10)
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10
        )

        # Initialize semantic parser
        self.parser = SemanticParser()

        # Robot state tracking
        self.robot_state = RobotState(
            position=(0, 0, 0),
            orientation=(0, 0, 0),
            battery_level=1.0,
            gripper_state='open'
        )

        # Object location tracking
        self.object_locations = {}

        self.get_logger().info("Language Understanding Node initialized")

    def voice_callback(self, msg: String):
        """Process incoming voice commands"""
        try:
            # Parse the command
            parsed_command = self.parser.parse_command(msg.data)

            # Check confidence threshold
            if parsed_command.confidence < 0.5:
                self.get_logger().warn(
                    f"Low confidence command: {parsed_command.original_text} "
                    f"(confidence: {parsed_command.confidence:.2f})"
                )
                return

            # Create command message
            command_msg = String()
            command_dict = {
                'intent': parsed_command.intent,
                'entities': parsed_command.entities,
                'confidence': parsed_command.confidence,
                'original_text': parsed_command.original_text
            }
            command_msg.data = json.dumps(command_dict)

            # Publish parsed command
            self.command_pub.publish(command_msg)
            self.get_logger().info(
                f"Parsed command: {parsed_command.intent} "
                f"with confidence {parsed_command.confidence:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error parsing command: {e}")

    def update_robot_state(self, position: tuple, orientation: tuple,
                          battery: float, gripper_state: str):
        """Update robot state for contextual understanding"""
        self.robot_state = RobotState(
            position=position,
            orientation=orientation,
            battery_level=battery,
            gripper_state=gripper_state
        )
        self.parser.update_context(self.robot_state, self.object_locations)

    def update_object_locations(self, locations: Dict[str, tuple]):
        """Update known object locations"""
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

## Advanced Language Understanding

### Handling Ambiguity

```python
class AmbiguityResolver:
    def __init__(self):
        self.ambiguity_patterns = {
            'multiple_objects': r'pick up the (red|blue|green) (ball|cup|box)',
            'spatial_reference': r'pick up (that|it|the one)',
            'temporal_reference': r'do (that|it) again'
        }

    def detect_ambiguity(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """Detect potential ambiguities in parsed command"""
        ambiguity_info = {
            'type': [],
            'details': [],
            'resolution_needed': False
        }

        # Check for multiple possible objects
        if 'object' in parsed_command.entities and len(parsed_command.entities['object']) > 1:
            ambiguity_info['type'].append('multiple_objects')
            ambiguity_info['details'].append(
                f"Multiple objects identified: {parsed_command.entities['object']}"
            )
            ambiguity_info['resolution_needed'] = True

        # Check for vague spatial references
        vague_refs = ['that', 'it', 'the one', 'there', 'over there']
        if any(ref in parsed_command.original_text.lower() for ref in vague_refs):
            ambiguity_info['type'].append('spatial_reference')
            ambiguity_info['details'].append("Vague spatial reference detected")
            ambiguity_info['resolution_needed'] = True

        return ambiguity_info

    def request_clarification(self, ambiguity_info: Dict[str, Any]) -> str:
        """Generate clarification request based on ambiguity type"""
        if not ambiguity_info['resolution_needed']:
            return ""

        clarification_requests = {
            'multiple_objects': "Which specific object would you like me to interact with?",
            'spatial_reference': "Could you point to or be more specific about which object you mean?",
            'temporal_reference': "Could you repeat the command more specifically?"
        }

        request = ""
        for amb_type in ambiguity_info['type']:
            if amb_type in clarification_requests:
                request += clarification_requests[amb_type] + " "

        return request.strip()
```

## Testing and Validation

### Unit Tests

```python
import unittest
from unittest.mock import Mock

class TestLanguageUnderstanding(unittest.TestCase):
    def setUp(self):
        self.parser = SemanticParser()

    def test_intent_classification(self):
        """Test intent classification accuracy"""
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
        """Test entity extraction from commands"""
        text = "pick up the red ball from the kitchen"
        result = self.parser.parse_command(text)

        self.assertIn('object', result.entities)
        self.assertIn('location', result.entities)
        self.assertIn('red', result.entities['object'])
        self.assertIn('kitchen', result.entities['location'])

    def test_quantity_extraction(self):
        """Test numerical quantity extraction"""
        text = "move forward 2 meters"
        result = self.parser.parse_command(text)

        self.assertIn('distance_meters', result.entities)
        self.assertEqual(result.entities['distance_meters'], 2.0)

    def test_contextual_resolution(self):
        """Test contextual reference resolution"""
        # Set up mock context
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

        # Test color-based object resolution
        text = "pick up the red one"
        result = self.parser.parse_command(text)

        # Should resolve to red_ball based on context
        if 'target_object' in result.entities:
            self.assertIn('red_ball', result.entities['target_object'])

if __name__ == '__main__':
    unittest.main()
```

## Configuration and Setup

### Configuration File

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

### Launch File

```xml
<!-- launch/language_understanding.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="language_understanding_node" name="language_understanding">
    <param name="model_path" value="$(var model_path)"/>
    <param name="min_confidence" value="0.5"/>
  </node>
</launch>
```

## Troubleshooting

### Common Issues

1. **Low Intent Classification Accuracy**
   - Solution: Add more training examples for specific robot commands
   - Check: Ensure training data covers actual use cases

2. **Poor Entity Extraction**
   - Solution: Expand regex patterns for your specific domain
   - Check: Verify entity patterns match your command vocabulary

3. **High Ambiguity Rates**
   - Solution: Implement better contextual resolution
   - Check: Ensure robot state and object location data is current

4. **Performance Issues**
   - Solution: Optimize feature extraction and classification
   - Check: Use lightweight models for real-time processing

## Next Steps

In the next section, we'll implement computer vision systems to provide the visual understanding component of our Vision-Language-Action framework. The language understanding system we've built will be enhanced with visual context to create more robust command interpretation.

Continue to [Computer Vision Integration](../computer-vision/index.md).