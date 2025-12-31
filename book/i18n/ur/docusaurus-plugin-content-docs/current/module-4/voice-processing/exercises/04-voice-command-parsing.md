# Exercise 1.4: Voice Command Parsing and Natural Language Understanding

## Objective
Implement voice command parsing to extract actionable intents and entities from transcribed speech for the Vision-Language-Action system.

## Prerequisites
- Completion of Exercises 1.1-1.3
- Python 3.10+
- Required libraries: regex, spacy (optional), nltk (optional)
- Transcribed text from speech recognition

## Exercise Steps

### Step 1: Set Up Command Parsing Environment
Create a new file `command_parser.py`:

```python
#!/usr/bin/env python3
import re
import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Any, Union
import numpy as np

@dataclass
class VoiceCommand:
    """Data structure for parsed voice commands"""
    intent: str
    entities: Dict[str, Any]
    confidence: float
    original_text: str
    processed_text: str

class VoiceCommandParser:
    """Parses transcribed voice commands into structured actions"""
    def __init__(self):
        self.command_patterns = {
            'move': [
                r'move\s+(?P<direction>forward|backward|left|right|up|down|north|south|east|west)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>meters|meter|m|cm|centimeters|steps)?',
                r'go\s+(?P<direction>forward|backward|left|right|north|south|east|west)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>meters|meter|m|cm|centimeters|steps)?',
                r'walk\s+(?P<direction>forward|backward|left|right|north|south|east|west)\s*(?P<distance>\d+\.?\d*)?\s*(?P<unit>meters|meter|m|cm|centimeters|steps)?',
                r'go\s+(?P<distance>\d+\.?\d*)\s*(?P<unit>meters|meter|m|cm|centimeters)\s+(?P<direction>forward|backward|left|right|north|south|east|west)'
            ],
            'grasp': [
                r'grasp\s+(?P<object>\w+(?:\s+\w+)*)',
                r'pick\s+up\s+(?P<object>\w+(?:\s+\w+)*)',
                r'grab\s+(?P<object>\w+(?:\s+\w+)*)',
                r'take\s+(?P<object>\w+(?:\s+\w+)*)',
                r'pick\s+(?P<object>\w+(?:\s+\w+)*)\s+up'
            ],
            'navigate': [
                r'go\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'navigate\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'move\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'go\s+(?P<location>\w+(?:\s+\w+)*)',
                r'head\s+to\s+(?P<location>\w+(?:\s+\w+)*)'
            ],
            'inspect': [
                r'look\s+at\s+(?P<object>\w+(?:\s+\w+)*)',
                r'inspect\s+(?P<object>\w+(?:\s+\w+)*)',
                r'check\s+(?P<object>\w+(?:\s+\w+)*)',
                r'observe\s+(?P<object>\w+(?:\s+\w+)*)',
                r'find\s+(?P<object>\w+(?:\s+\w+)*)'
            ],
            'stop': [
                r'stop',
                r'halt',
                r'pause',
                r'freeze',
                r'wait'
            ],
            'follow': [
                r'follow\s+(?P<target>\w+(?:\s+\w+)*)',
                r'come\s+with\s+(?P<target>\w+(?:\s+\w+)*)',
                r'accompany\s+(?P<target>\w+(?:\s+\w+)*)'
            ],
            'bring': [
                r'bring\s+(?P<object>\w+(?:\s+\w+)*)\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'take\s+(?P<object>\w+(?:\s+\w+)*)\s+to\s+(?P<location>\w+(?:\s+\w+)*)',
                r'carry\s+(?P<object>\w+(?:\s+\w+)*)\s+to\s+(?P<location>\w+(?:\s+\w+)*)'
            ]
        }

        # Entity extraction patterns
        self.entity_patterns = {
            'number': r'\d+\.?\d*',
            'distance': r'(\d+\.?\d*)\s*(meters|meter|m|cm|centimeters|steps)',
            'object': r'(red|blue|green|yellow|small|large|big|medium)\s+(\w+)',
            'location': r'(kitchen|living room|bedroom|office|bathroom|hallway|garage|garden|dining room)'
        }

        # Intent confidence weights
        self.intent_weights = {
            'move': 0.9,
            'grasp': 0.9,
            'navigate': 0.9,
            'inspect': 0.8,
            'stop': 0.95,
            'follow': 0.85,
            'bring': 0.85
        }

    def parse_command(self, text: str) -> Optional[VoiceCommand]:
        """Parse voice command from text"""
        if not text or not text.strip():
            return None

        original_text = text
        text = text.lower().strip()

        # Preprocess text
        processed_text = self._preprocess_text(text)

        # Try to match against each intent pattern
        best_match = None
        best_confidence = 0.0

        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, processed_text)
                if match:
                    entities = match.groupdict()

                    # Post-process entities
                    entities = self._post_process_entities(entities)

                    # Calculate confidence based on match quality
                    confidence = self._calculate_confidence(text, pattern, entities)

                    # Apply intent-specific weight
                    confidence *= self.intent_weights.get(intent, 0.8)

                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_match = VoiceCommand(
                            intent=intent,
                            entities=entities,
                            confidence=confidence,
                            original_text=original_text,
                            processed_text=processed_text
                        )

        # If no high-confidence match, try entity extraction
        if best_confidence < 0.5:
            fallback_result = self._fallback_parsing(processed_text, original_text)
            if fallback_result and fallback_result.confidence > best_confidence:
                best_match = fallback_result

        return best_match

    def _preprocess_text(self, text: str) -> str:
        """Preprocess text for better matching"""
        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text.strip())

        # Normalize common variations
        text = re.sub(r'\bstaight\b', 'forward', text)
        text = re.sub(r'\bgoto\b', 'go to', text)
        text = re.sub(r'\bwalk\s+to\b', 'go to', text)

        return text

    def _post_process_entities(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Post-process extracted entities"""
        processed_entities = {}

        for key, value in entities.items():
            if value is None:
                continue

            # Convert distance to numeric value
            if key == 'distance' and value:
                try:
                    processed_entities[key] = float(value)
                except ValueError:
                    processed_entities[key] = value
            # Process object names
            elif key == 'object' and value:
                processed_entities[key] = value.strip()
            # Process location names
            elif key == 'location' and value:
                processed_entities[key] = value.strip().replace(' ', '_')
            # Process direction names
            elif key == 'direction' and value:
                processed_entities[key] = value.strip()
            # Process target names
            elif key == 'target' and value:
                processed_entities[key] = value.strip()
            else:
                processed_entities[key] = value

        return processed_entities

    def _calculate_confidence(self, text: str, pattern: str, entities: Dict[str, Any]) -> float:
        """Calculate confidence score for pattern match"""
        # Base confidence on pattern match
        base_confidence = 0.7

        # Boost confidence if we have meaningful entities
        if entities:
            entity_count = len([v for v in entities.values() if v])
            base_confidence += entity_count * 0.1

        # Boost confidence for longer, more specific matches
        match_length = len(text)
        base_confidence = min(0.95, base_confidence + (match_length / 200))

        # Penalty for ambiguous matches
        if any(word in text for word in ['maybe', 'perhaps', 'possibly']):
            base_confidence *= 0.8

        return base_confidence

    def _fallback_parsing(self, text: str, original_text: str) -> Optional[VoiceCommand]:
        """Fallback parsing using entity extraction"""
        # Try to extract entities even without perfect pattern match
        entities = {}

        # Extract numbers
        number_matches = re.findall(self.entity_patterns['number'], text)
        if number_matches:
            entities['number'] = [float(n) for n in number_matches]

        # Extract objects with adjectives
        object_matches = re.findall(self.entity_patterns['object'], text)
        if object_matches:
            for adj, obj in object_matches:
                entities['object'] = f"{adj}_{obj}"

        # Extract known locations
        for location in ['kitchen', 'living room', 'bedroom', 'office', 'bathroom', 'hallway']:
            if location in text:
                entities['location'] = location.replace(' ', '_')
                break

        # Determine intent based on keywords
        intent = 'unknown'
        if any(word in text for word in ['move', 'go', 'walk', 'forward', 'backward', 'left', 'right']):
            intent = 'move'
        elif any(word in text for word in ['grasp', 'pick', 'grab', 'take', 'hold']):
            intent = 'grasp'
        elif any(word in text for word in ['go to', 'navigate', 'move to', 'head to']):
            intent = 'navigate'
        elif any(word in text for word in ['stop', 'halt', 'pause']):
            intent = 'stop'
        elif any(word in text for word in ['look', 'inspect', 'check', 'find']):
            intent = 'inspect'

        if intent != 'unknown':
            confidence = 0.4  # Lower confidence for fallback

            # Boost if we found entities
            if entities:
                confidence += len(entities) * 0.1

            return VoiceCommand(
                intent=intent,
                entities=entities,
                confidence=min(0.6, confidence),  # Cap fallback confidence
                original_text=original_text,
                processed_text=text
            )

        return None

    def batch_parse(self, texts: List[str]) -> List[Optional[VoiceCommand]]:
        """Parse multiple commands at once"""
        return [self.parse_command(text) for text in texts]

class AdvancedCommandParser(VoiceCommandParser):
    """Extended command parser with more sophisticated NLU capabilities"""
    def __init__(self):
        super().__init__()

        # Add more complex patterns for compound commands
        self.compound_patterns = [
            r'first\s+(?P<action1>.*?),?\s+then\s+(?P<action2>.*)',
            r'after\s+(?P<action1>.*?),?\s+(?P<action2>.*)',
            r'while\s+(?P<action1>.*?),?\s+(?P<action2>.*)'
        ]

        # Add temporal patterns
        self.temporal_patterns = {
            'repeat': r'(repeat|again|redo|do it again)',
            'delay': r'(wait|pause|delay)\s*(?P<time>\d+\.?\d*)?\s*(seconds|second|secs|sec|minutes|minute|min)?',
            'sequence': r'(first|second|third|next|then|afterwards|finally)'
        }

    def parse_compound_command(self, text: str) -> Optional[VoiceCommand]:
        """Parse compound commands with multiple actions"""
        for pattern in self.compound_patterns:
            match = re.search(pattern, text.lower())
            if match:
                groups = match.groupdict()

                # Parse individual actions
                action1 = self.parse_command(groups.get('action1', ''))
                action2 = self.parse_command(groups.get('action2', ''))

                if action1 and action2:
                    return VoiceCommand(
                        intent='compound',
                        entities={
                            'actions': [action1, action2],
                            'sequence': 'sequential'
                        },
                        confidence=0.85,
                        original_text=text,
                        processed_text=text.lower()
                    )

        return super().parse_command(text)

    def extract_temporal_info(self, text: str) -> Dict[str, Any]:
        """Extract temporal information from command"""
        temporal_info = {}

        for key, pattern in self.temporal_patterns.items():
            match = re.search(pattern, text.lower())
            if match:
                if key == 'delay' and 'time' in match.groupdict():
                    temporal_info[key] = float(match.group('time') or 1.0)
                else:
                    temporal_info[key] = match.group(0)

        return temporal_info