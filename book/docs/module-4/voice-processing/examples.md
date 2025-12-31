---
sidebar_position: 2
title: "Voice Processing Examples"
---

# Voice Processing Examples

## Overview

This document provides practical examples and use cases for voice processing in robotics applications. These examples demonstrate how to implement and use voice processing capabilities in real-world scenarios.

## Basic Voice Command Examples

### Simple Movement Commands

```python
# Example: Processing simple movement commands
import rospy
from std_msgs.msg import String

def process_movement_command(text):
    """
    Example: Process commands like "move forward", "turn left", etc.

    Sample inputs:
    - "move forward 2 meters"
    - "turn left 90 degrees"
    - "go backward slowly"
    """

    movement_patterns = {
        'forward': r'move\s+forward|go\s+forward|move\s+ahead|go\s+ahead',
        'backward': r'move\s+backward|go\s+backward|move\s+back|go\s+back',
        'left': r'turn\s+left|rotate\s+left|go\s+left|move\s+left',
        'right': r'turn\s+right|rotate\s+right|go\s+right|move\s+right',
        'up': r'move\s+up|go\s+up|lift',
        'down': r'move\s+down|go\s+down|lower'
    }

    for direction, pattern in movement_patterns.items():
        if re.search(pattern, text, re.IGNORECASE):
            # Extract distance if provided
            distance_match = re.search(r'(\d+\.?\d*)\s*(meters?|cm|inches?)', text)
            distance = float(distance_match.group(1)) if distance_match else 1.0

            return {
                'command': 'move',
                'direction': direction,
                'distance': distance,
                'unit': distance_match.group(2) if distance_match else 'meters'
            }

    return None

# ROS Node example
class VoiceMovementNode:
    def __init__(self):
        rospy.init_node('voice_movement_node')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.voice_sub = rospy.Subscriber('/voice_commands', String, self.voice_callback)

    def voice_callback(self, msg):
        command = process_movement_command(msg.data)
        if command:
            self.execute_movement(command)

    def execute_movement(self, command):
        twist = Twist()

        if command['direction'] == 'forward':
            twist.linear.x = 0.5  # m/s
        elif command['direction'] == 'backward':
            twist.linear.x = -0.5
        elif command['direction'] == 'left':
            twist.angular.z = 0.5  # rad/s
        elif command['direction'] == 'right':
            twist.angular.z = -0.5

        self.cmd_pub.publish(twist)
```

### Object Interaction Commands

```python
# Example: Processing object interaction commands
def process_object_command(text):
    """
    Example: Process commands like "pick up the red ball", "grasp the cup", etc.

    Sample inputs:
    - "pick up the red ball"
    - "grasp the blue cube"
    - "put down the object"
    """

    # Extract object properties
    color_pattern = r'(red|blue|green|yellow|black|white|purple|orange|pink)'
    shape_pattern = r'(ball|cube|box|cup|bottle|book|pen)'
    action_pattern = r'(pick up|grasp|grab|take|hold|put down|release|drop)'

    color_match = re.search(color_pattern, text, re.IGNORECASE)
    shape_match = re.search(shape_pattern, text, re.IGNORECASE)
    action_match = re.search(action_pattern, text, re.IGNORECASE)

    return {
        'action': action_match.group(1).lower() if action_match else None,
        'color': color_match.group(1).lower() if color_match else None,
        'shape': shape_match.group(1).lower() if shape_match else None,
        'raw_text': text
    }

# Example usage
def handle_object_interaction(text):
    command = process_object_command(text)

    if command['action'] in ['pick up', 'grasp', 'grab', 'take', 'hold']:
        # Find object in vision system
        target_object = find_object_by_properties(
            color=command['color'],
            shape=command['shape']
        )

        if target_object:
            return {
                'intent': 'grasp_object',
                'target': target_object,
                'action': 'grasp'
            }
        else:
            return {
                'intent': 'find_object',
                'color': command['color'],
                'shape': command['shape']
            }

    elif command['action'] in ['put down', 'release', 'drop']:
        return {
            'intent': 'release_object',
            'action': 'release'
        }

    return None
```

## Advanced Voice Processing Examples

### Multi-Step Command Sequences

```python
# Example: Processing multi-step commands
class MultiStepCommandProcessor:
    def __init__(self):
        self.command_queue = []
        self.current_step = 0

    def process_sequence_command(self, text):
        """
        Example: Process commands like "go to kitchen, then pick up the cup, then come back"
        """

        # Split by sequence indicators
        sequence_indicators = ['then', 'and then', 'after that', 'next']

        # Tokenize the command sequence
        commands = self.tokenize_sequence(text, sequence_indicators)

        processed_commands = []
        for cmd_text in commands:
            processed_cmd = self.parse_single_command(cmd_text.strip())
            if processed_cmd:
                processed_commands.append(processed_cmd)

        return processed_commands

    def tokenize_sequence(self, text, indicators):
        # Create regex pattern for sequence indicators
        pattern = '|'.join(re.escape(indicator) for indicator in indicators)
        commands = re.split(pattern, text, flags=re.IGNORECASE)
        return [cmd.strip() for cmd in commands if cmd.strip()]

    def parse_single_command(self, text):
        # Use existing command parsers
        movement_cmd = process_movement_command(text)
        object_cmd = process_object_command(text)
        navigation_cmd = process_navigation_command(text)

        # Return the most relevant command type
        if movement_cmd:
            return movement_cmd
        elif object_cmd and object_cmd['action']:
            return object_cmd
        elif navigation_cmd:
            return navigation_cmd

        return {'raw_text': text, 'type': 'unknown'}

# Example usage
processor = MultiStepCommandProcessor()

sequence_text = "Go to the kitchen, then pick up the red cup, then return to me"
commands = processor.process_sequence_command(sequence_text)

print("Parsed command sequence:")
for i, cmd in enumerate(commands):
    print(f"Step {i+1}: {cmd}")
```

### Context-Aware Voice Processing

```python
# Example: Context-aware voice processing
class ContextAwareVoiceProcessor:
    def __init__(self):
        self.context = {
            'location': 'starting_position',
            'objects_in_view': [],
            'last_action': None,
            'task_state': 'idle'
        }
        self.task_stack = []

    def update_context(self, **kwargs):
        """Update the current context"""
        self.context.update(kwargs)

    def process_contextual_command(self, text):
        """
        Process commands based on current context
        Examples:
        - "Do that again" (repeat last action)
        - "What did you see?" (query vision system)
        - "Tell me about the red object" (query specific object)
        """

        # Handle relative commands
        if 'again' in text.lower() or 'repeat' in text.lower():
            if self.context['last_action']:
                return self.context['last_action']

        # Handle context queries
        if 'what' in text.lower() and 'see' in text.lower():
            return {
                'intent': 'query_environment',
                'action': 'describe_objects'
            }

        # Handle object queries
        if 'tell me about' in text.lower() or 'describe' in text.lower():
            # Extract object reference
            obj_ref = self.extract_object_reference(text)
            return {
                'intent': 'describe_object',
                'target_object': obj_ref
            }

        # Handle location-based commands
        if 'here' in text.lower() or 'this' in text.lower():
            return self.process_command_with_location(text, self.context['location'])

        # Default processing
        return self.parse_command_with_context(text)

    def extract_object_reference(self, text):
        """Extract object reference from text"""
        # Look for demonstrative pronouns or color/shape combinations
        patterns = [
            r'this\s+(\w+)',      # "this ball"
            r'that\s+(\w+)',      # "that cup"
            r'the\s+(\w+)\s+(\w+)', # "the red ball"
            r'(\w+)\s+(\w+)\s+one'  # "red ball one"
        ]

        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                groups = match.groups()
                if len(groups) == 1:
                    return {'type': groups[0].lower()}
                elif len(groups) == 2:
                    return {'color': groups[0].lower(), 'type': groups[1].lower()}

        return None

# Example usage with context
context_processor = ContextAwareVoiceProcessor()
context_processor.update_context(
    location='kitchen',
    objects_in_view=['red cup', 'blue bowl', 'white plate'],
    last_action={'type': 'move', 'direction': 'forward', 'distance': 2.0}
)

command = context_processor.process_contextual_command("Do that again")
print(f"Contextual command result: {command}")
```

## Real-World Use Cases

### Home Assistant Robot

```python
# Example: Voice commands for a home assistant robot
class HomeAssistantVoiceProcessor:
    def __init__(self):
        self.known_locations = {
            'kitchen': [1.0, 2.0, 0.0],
            'living_room': [3.0, 1.0, 0.0],
            'bedroom': [5.0, 3.0, 0.0],
            'office': [2.0, 4.0, 0.0]
        }

        self.known_objects = {
            'red cup': {'type': 'cup', 'color': 'red', 'location': 'kitchen'},
            'blue bowl': {'type': 'bowl', 'color': 'blue', 'location': 'kitchen'},
            'white plate': {'type': 'plate', 'color': 'white', 'location': 'kitchen'}
        }

    def process_home_command(self, text):
        """
        Process home automation and assistance commands
        Examples:
        - "Go to the kitchen"
        - "Bring me the red cup"
        - "What's on the table?"
        - "Turn on the lights"
        """

        # Navigation commands
        nav_match = re.search(r'go to the (\w+)|navigate to (\w+)', text, re.IGNORECASE)
        if nav_match:
            location = (nav_match.group(1) or nav_match.group(2)).lower()
            if location in self.known_locations:
                return {
                    'intent': 'navigate',
                    'destination': location,
                    'coordinates': self.known_locations[location]
                }

        # Fetch object commands
        fetch_match = re.search(r'bring me the (\w+ \w+)|get the (\w+ \w+)', text, re.IGNORECASE)
        if fetch_match:
            obj_desc = (fetch_match.group(1) or fetch_match.group(2)).lower()
            if obj_desc in self.known_objects:
                obj_info = self.known_objects[obj_desc]
                return {
                    'intent': 'fetch_object',
                    'object': obj_desc,
                    'object_info': obj_info,
                    'pickup_location': obj_info['location']
                }

        # Environmental queries
        if 'what' in text.lower() and ('table' in text.lower() or 'counter' in text.lower()):
            location = self.infer_location_from_context(text)
            return {
                'intent': 'query_environment',
                'query_type': 'objects_on_surface',
                'surface': self.extract_surface(text),
                'location': location
            }

        return None

    def infer_location_from_context(self, text):
        """Infer location from context words"""
        location_indicators = {
            'kitchen': ['table', 'counter', 'fridge', 'oven', 'sink'],
            'living_room': ['sofa', 'couch', 'tv', 'coffee table'],
            'bedroom': ['bed', 'dresser', 'nightstand'],
            'office': ['desk', 'computer', 'chair']
        }

        text_lower = text.lower()
        for location, indicators in location_indicators.items():
            for indicator in indicators:
                if indicator in text_lower:
                    return location

        return 'current_location'  # Default to current location

    def extract_surface(self, text):
        """Extract surface type from text"""
        surfaces = ['table', 'counter', 'desk', 'shelf', 'couch', 'sofa', 'bed']
        for surface in surfaces:
            if surface in text.lower():
                return surface
        return 'unknown_surface'

# Example usage
home_processor = HomeAssistantVoiceProcessor()
command = home_processor.process_home_command("Bring me the red cup from the kitchen")
print(f"Home command result: {command}")
```

### Industrial Robot Voice Control

```python
# Example: Voice commands for industrial robots
class IndustrialVoiceProcessor:
    def __init__(self):
        self.work_cells = {
            'assembly_station_1': {'id': 1, 'components': ['motor', 'gear', 'housing']},
            'welding_station': {'id': 2, 'components': ['frame', 'joint', 'weld_points']},
            'quality_control': {'id': 3, 'components': ['sensor', 'camera', 'gauge']}
        }

        self.tools = {
            'screwdriver': {'id': 101, 'type': 'fastening'},
            'welder': {'id': 102, 'type': 'joining'},
            'inspector': {'id': 103, 'type': 'quality_check'}
        }

    def process_industrial_command(self, text):
        """
        Process industrial manufacturing commands
        Examples:
        - "Move to assembly station 1"
        - "Install the motor component"
        - "Run quality check on part 456"
        - "Switch to welding tool"
        """

        # Station navigation
        station_match = re.search(r'move to (\w+ \w+ \d+)|go to (\w+ \w+ \d+)', text, re.IGNORECASE)
        if station_match:
            station = (station_match.group(1) or station_match.group(2)).lower()
            if station in self.work_cells:
                return {
                    'intent': 'navigate_to_station',
                    'station': station,
                    'station_info': self.work_cells[station]
                }

        # Component installation
        install_match = re.search(r'install the (\w+ \w+)|assemble the (\w+ \w+)', text, re.IGNORECASE)
        if install_match:
            component = (install_match.group(1) or install_match.group(2)).lower()
            return {
                'intent': 'install_component',
                'component': component,
                'procedure': self.get_assembly_procedure(component)
            }

        # Quality control
        qc_match = re.search(r'run quality check|inspect|check quality', text, re.IGNORECASE)
        if qc_match:
            part_id = self.extract_part_id(text)
            return {
                'intent': 'quality_check',
                'part_id': part_id,
                'inspection_type': self.determine_inspection_type(text)
            }

        # Tool change
        tool_match = re.search(r'switch to (\w+ \w+)|change tool to (\w+ \w+)', text, re.IGNORECASE)
        if tool_match:
            tool = (tool_match.group(1) or tool_match.group(2)).lower()
            if tool in self.tools:
                return {
                    'intent': 'change_tool',
                    'tool': tool,
                    'tool_info': self.tools[tool]
                }

        return None

    def extract_part_id(self, text):
        """Extract part ID from text"""
        part_id_match = re.search(r'part (\d+)|component (\d+)', text, re.IGNORECASE)
        return part_id_match.group(1) if part_id_match else 'unknown'

    def determine_inspection_type(self, text):
        """Determine inspection type from text"""
        if 'dimension' in text.lower():
            return 'dimensional'
        elif 'surface' in text.lower() or 'visual' in text.lower():
            return 'visual'
        elif 'strength' in text.lower() or 'stress' in text.lower():
            return 'strength'
        else:
            return 'general'

# Example usage
industrial_processor = IndustrialVoiceProcessor()
command = industrial_processor.process_industrial_command("Move to assembly station 1 and install the motor component")
print(f"Industrial command result: {command}")
```

## Error Handling and Robustness

### Voice Recognition Error Recovery

```python
# Example: Error handling for voice recognition
class VoiceErrorRecovery:
    def __init__(self):
        self.recovery_strategies = [
            self.ask_for_repetition,
            self.use_context_clues,
            self.propose_alternatives
        ]

    def handle_recognition_error(self, original_text, confidence_score):
        """Handle low-confidence or unrecognized voice commands"""
        if confidence_score < 0.5:
            # Low confidence - use recovery strategies
            for strategy in self.recovery_strategies:
                recovery_result = strategy(original_text)
                if recovery_result:
                    return recovery_result

        return {'intent': 'unknown_command', 'original_text': original_text}

    def ask_for_repetition(self, original_text):
        """Ask user to repeat the command"""
        return {
            'intent': 'request_repeat',
            'message': f"I didn't understand '{original_text}'. Could you please repeat that?",
            'original_text': original_text
        }

    def use_context_clues(self, original_text):
        """Use context to interpret ambiguous commands"""
        # This would integrate with the context-aware processor
        # to make educated guesses based on current state
        pass

    def propose_alternatives(self, original_text):
        """Propose alternative interpretations"""
        # Generate possible interpretations based on similar-sounding commands
        possible_intents = self.find_similar_commands(original_text)
        if possible_intents:
            return {
                'intent': 'propose_alternatives',
                'alternatives': possible_intents,
                'original_text': original_text
            }

    def find_similar_commands(self, text):
        """Find commands similar to the unrecognized text"""
        # Implementation would use fuzzy matching or phonetic similarity
        # to find commands that sound similar to the input
        pass

# Example usage
error_recovery = VoiceErrorRecovery()
recovery_result = error_recovery.handle_recognition_error("unclear command text", 0.3)
print(f"Error recovery result: {recovery_result}")
```

## Performance Optimization Examples

### Real-time Voice Processing Pipeline

```python
# Example: Optimized real-time voice processing
import threading
import queue
import time

class RealTimeVoiceProcessor:
    def __init__(self):
        self.audio_queue = queue.Queue(maxsize=10)
        self.result_queue = queue.Queue(maxsize=5)
        self.is_running = False
        self.processing_thread = None

    def start_processing(self):
        """Start real-time voice processing"""
        self.is_running = True
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def stop_processing(self):
        """Stop real-time voice processing"""
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join()

    def _processing_loop(self):
        """Main processing loop"""
        while self.is_running:
            try:
                # Get audio chunk from queue
                audio_chunk = self.audio_queue.get(timeout=0.1)

                # Process audio (non-blocking)
                result = self.process_audio_chunk(audio_chunk)

                # Put result in output queue
                try:
                    self.result_queue.put_nowait(result)
                except queue.Full:
                    # Drop old result if queue is full
                    pass

            except queue.Empty:
                continue

    def process_audio_chunk(self, audio_chunk):
        """Process a single audio chunk efficiently"""
        # Quick preprocessing
        if self.is_voice_activity(audio_chunk):
            # More intensive processing only when voice detected
            text = self.speech_to_text(audio_chunk)
            if text and self.is_confident_enough(text):
                command = self.parse_command(text)
                return command

        return None

    def is_voice_activity(self, audio_chunk):
        """Quick voice activity detection"""
        # Simple energy-based detection
        energy = sum(audio_chunk**2) / len(audio_chunk)
        return energy > 0.001  # Adjust threshold as needed

    def is_confident_enough(self, text):
        """Check if recognition confidence is sufficient"""
        # Implementation would check confidence from STT service
        return len(text.strip()) > 2  # Basic check for meaningful text

# Example usage
real_time_processor = RealTimeVoiceProcessor()
real_time_processor.start_processing()

# In your main loop, add audio chunks to process
# audio_chunk = get_audio_input()
# real_time_processor.audio_queue.put(audio_chunk)

# Get results
try:
    result = real_time_processor.result_queue.get_nowait()
    if result:
        print(f"Processed command: {result}")
except queue.Empty:
    pass  # No results ready yet
```

These examples demonstrate practical implementations of voice processing for robotics applications, covering various scenarios from simple movement commands to complex multi-step interactions. The examples include error handling, context awareness, and performance optimization techniques that are essential for robust voice-controlled robotic systems.