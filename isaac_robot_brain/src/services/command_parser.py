"""
Command Parser Service

This module implements the command parsing service that extracts structured information
from natural language commands.
"""

import re
import logging
from typing import Dict, Any, List, Tuple
from enum import Enum

from ..models.voice_command import VoiceCommand


class CommandAction(Enum):
    """Enumeration of possible command actions"""
    PICK_UP = "pick_up"
    PUT_DOWN = "put_down"
    MOVE_TO = "move_to"
    GRAB = "grab"
    RELEASE = "release"
    POINT_TO = "point_to"
    FOLLOW = "follow"
    STOP = "stop"
    GO_TO = "go_to"
    BRING = "bring"
    GREET = "greet"
    SIT = "sit"
    STAND = "stand"
    WALK = "walk"
    RUN = "run"
    UNKNOWN = "unknown"


class CommandParser:
    """
    Command parsing service that extracts structured information from natural language commands.
    """

    def __init__(self):
        """Initialize the command parser service."""
        self.logger = logging.getLogger(__name__)

        # Define patterns for different command types
        self.patterns = {
            CommandAction.PICK_UP: [
                r'pick up the (\w+) (\w+)',
                r'grab the (\w+) (\w+)',
                r'take the (\w+) (\w+)',
                r'get the (\w+) (\w+)',
                r'lift the (\w+) (\w+)'
            ],
            CommandAction.MOVE_TO: [
                r'move to the (\w+)',
                r'go to the (\w+)',
                r'walk to the (\w+)',
                r'navigate to the (\w+)'
            ],
            CommandAction.POINT_TO: [
                r'point to the (\w+) (\w+)',
                r'point at the (\w+) (\w+)',
                r'indicate the (\w+) (\w+)'
            ],
            CommandAction.BRING: [
                r'bring me the (\w+) (\w+)',
                r'bring the (\w+) (\w+) to me',
                r'fetch the (\w+) (\w+)'
            ]
        }

        # Define common object types
        self.object_types = {
            "ball", "cube", "box", "cup", "bottle", "book", "phone", "tablet",
            "toy", "tool", "object", "item", "thing", "object"
        }

        # Define common colors
        self.colors = {
            "red", "blue", "green", "yellow", "black", "white", "orange",
            "purple", "pink", "brown", "gray", "grey"
        }

        # Define common locations
        self.locations = {
            "kitchen", "living room", "bedroom", "office", "table", "chair",
            "couch", "shelf", "cabinet", "door", "window", "floor", "wall"
        }

    def parse_command(self, voice_command: VoiceCommand) -> Dict[str, Any]:
        """
        Parse a voice command and extract structured information.

        Args:
            voice_command: The voice command to parse

        Returns:
            Dictionary containing parsed command information
        """
        if not voice_command.transcript:
            return {"action": CommandAction.UNKNOWN.value, "parameters": {}}

        transcript = voice_command.transcript.lower().strip()

        # Try to match patterns
        for action, pattern_list in self.patterns.items():
            for pattern in pattern_list:
                match = re.search(pattern, transcript)
                if match:
                    # Extract matched groups
                    groups = match.groups()
                    parameters = self._extract_parameters_from_groups(groups, transcript)
                    return {
                        "action": action.value,
                        "parameters": parameters,
                        "confidence": 0.9  # High confidence for pattern matches
                    }

        # If no pattern matches, use keyword-based parsing
        return self._keyword_parse(transcript)

    def _extract_parameters_from_groups(self, groups: tuple, transcript: str) -> Dict[str, Any]:
        """
        Extract parameters from regex match groups.

        Args:
            groups: Tuple of matched groups from regex
            transcript: Original transcript for additional context

        Returns:
            Dictionary of extracted parameters
        """
        parameters = {}

        # Handle different group configurations
        if len(groups) == 1:
            # Single parameter (e.g., location)
            param = groups[0]
            if param in self.locations:
                parameters["location"] = param
            else:
                parameters["target"] = param

        elif len(groups) >= 2:
            # Two or more parameters (e.g., color and object type)
            first_param, second_param = groups[0], groups[1]

            # Determine which is color and which is object type
            if first_param in self.colors and second_param in self.object_types:
                parameters["color"] = first_param
                parameters["object_type"] = second_param
            elif first_param in self.object_types and second_param in self.colors:
                parameters["color"] = second_param
                parameters["object_type"] = first_param
            elif first_param in self.object_types:
                parameters["object_type"] = first_param
                parameters["target"] = second_param
            else:
                parameters["object_type"] = first_param
                parameters["target"] = second_param

        # Add any additional context from the transcript
        additional_params = self._extract_additional_parameters(transcript)
        parameters.update(additional_params)

        return parameters

    def _keyword_parse(self, transcript: str) -> Dict[str, Any]:
        """
        Parse command using keyword matching when regex patterns don't match.

        Args:
            transcript: The command transcript to parse

        Returns:
            Dictionary containing parsed command information
        """
        # Identify action based on keywords
        action = self._identify_action(transcript)

        # Extract parameters using keyword matching
        parameters = self._extract_parameters_from_transcript(transcript)

        # Calculate confidence based on number of matches
        confidence = 0.5  # Base confidence
        if parameters:
            confidence += 0.3  # Increase for parameter matches
        if action != CommandAction.UNKNOWN:
            confidence += 0.2  # Increase for action identification

        return {
            "action": action.value,
            "parameters": parameters,
            "confidence": min(0.9, confidence)  # Cap confidence at 0.9 for keyword-based parsing
        }

    def _identify_action(self, transcript: str) -> CommandAction:
        """
        Identify the action based on keywords in the transcript.

        Args:
            transcript: The command transcript

        Returns:
            Identified command action
        """
        # Action keywords mapping
        action_keywords = {
            CommandAction.PICK_UP: ["pick up", "grab", "take", "get", "lift"],
            CommandAction.MOVE_TO: ["move to", "go to", "walk to", "navigate to", "move toward"],
            CommandAction.POINT_TO: ["point to", "point at", "indicate", "show me"],
            CommandAction.BRING: ["bring", "fetch", "get me", "bring me"],
            CommandAction.STOP: ["stop", "halt", "pause"],
            CommandAction.GREET: ["hello", "hi", "greet", "wave to"],
            CommandAction.SIT: ["sit", "sit down"],
            CommandAction.STAND: ["stand", "stand up"]
        }

        transcript_lower = transcript.lower()

        for action, keywords in action_keywords.items():
            for keyword in keywords:
                if keyword in transcript_lower:
                    return action

        return CommandAction.UNKNOWN

    def _extract_parameters_from_transcript(self, transcript: str) -> Dict[str, Any]:
        """
        Extract parameters from transcript using keyword matching.

        Args:
            transcript: The command transcript

        Returns:
            Dictionary of extracted parameters
        """
        parameters = {}
        words = transcript.lower().split()

        # Look for colors
        for color in self.colors:
            if color in transcript:
                parameters["color"] = color

        # Look for object types
        for obj_type in self.object_types:
            if obj_type in transcript:
                parameters["object_type"] = obj_type

        # Look for locations
        for location in self.locations:
            if location in transcript:
                parameters["location"] = location

        # Look for specific objects mentioned with "the"
        for i, word in enumerate(words):
            if word == "the" and i + 1 < len(words):
                next_word = words[i + 1]
                if next_word in self.object_types:
                    parameters["target_object"] = next_word
                elif next_word in self.colors:
                    parameters["target_color"] = next_word

        # Look for adjectives that might describe objects
        # This is a simplified approach - in practice, you'd use NLP for proper POS tagging
        for i, word in enumerate(words):
            if word in self.colors and i + 1 < len(words):
                next_word = words[i + 1]
                if next_word in self.object_types:
                    parameters["color"] = word
                    parameters["object_type"] = next_word

        return parameters

    def _extract_additional_parameters(self, transcript: str) -> Dict[str, Any]:
        """
        Extract additional parameters that might not be caught by basic parsing.

        Args:
            transcript: The command transcript

        Returns:
            Dictionary of additional parameters
        """
        additional_params = {}

        # Look for directional information
        directions = ["left", "right", "forward", "backward", "up", "down"]
        for direction in directions:
            if direction in transcript:
                additional_params["direction"] = direction

        # Look for distance information
        distance_pattern = r'(\d+(?:\.\d+)?)\s*(meters?|cm|inches?)'
        distance_match = re.search(distance_pattern, transcript)
        if distance_match:
            distance, unit = distance_match.groups()
            additional_params["distance"] = {"value": float(distance), "unit": unit}

        # Look for time information
        time_pattern = r'(\d+(?:\.\d+)?)\s*(seconds?|minutes?|hours?)'
        time_match = re.search(time_pattern, transcript)
        if time_match:
            time_val, time_unit = time_match.groups()
            additional_params["time"] = {"value": float(time_val), "unit": time_unit}

        return additional_params

    def validate_parsed_command(self, parsed_command: Dict[str, Any]) -> bool:
        """
        Validate the parsed command structure.

        Args:
            parsed_command: The parsed command dictionary

        Returns:
            True if valid, False otherwise
        """
        required_keys = ["action", "parameters", "confidence"]
        for key in required_keys:
            if key not in parsed_command:
                return False

        # Validate confidence range
        confidence = parsed_command.get("confidence", 0)
        if not 0.0 <= confidence <= 1.0:
            return False

        # Validate action is recognized
        try:
            CommandAction(parsed_command["action"])
        except ValueError:
            return False

        # Validate parameters is a dictionary
        if not isinstance(parsed_command["parameters"], dict):
            return False

        return True

    def enhance_with_context(self, parsed_command: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Enhance the parsed command with contextual information.

        Args:
            parsed_command: The parsed command to enhance
            context: Contextual information to use for enhancement

        Returns:
            Enhanced command dictionary
        """
        enhanced_command = parsed_command.copy()

        # Add contextual information to parameters
        if context:
            for key, value in context.items():
                if key not in enhanced_command["parameters"]:
                    enhanced_command["parameters"][key] = value

            # Adjust confidence based on context availability
            if "environment" in context or "user" in context:
                # Increase confidence when we have contextual information
                enhanced_command["confidence"] = min(1.0, enhanced_command["confidence"] + 0.1)

        return enhanced_command

    def get_command_template(self, action: CommandAction) -> Dict[str, Any]:
        """
        Get a template for a specific command action.

        Args:
            action: The command action to get a template for

        Returns:
            Template dictionary with required parameters
        """
        templates = {
            CommandAction.PICK_UP: {
                "required": ["object_type"],
                "optional": ["color", "location", "target_object"],
                "description": "Pick up an object"
            },
            CommandAction.MOVE_TO: {
                "required": ["location"],
                "optional": ["target", "direction"],
                "description": "Move to a location"
            },
            CommandAction.POINT_TO: {
                "required": ["target_object"],
                "optional": ["color", "location"],
                "description": "Point to an object"
            },
            CommandAction.BRING: {
                "required": ["object_type"],
                "optional": ["color", "target_location"],
                "description": "Bring an object to a location/person"
            }
        }

        return templates.get(action, {
            "required": [],
            "optional": [],
            "description": "Unknown command"
        })

    def parse_with_validation(self, voice_command: VoiceCommand) -> Tuple[Dict[str, Any], bool]:
        """
        Parse a command and return both the result and validation status.

        Args:
            voice_command: The voice command to parse

        Returns:
            Tuple of (parsed_command, is_valid)
        """
        parsed_command = self.parse_command(voice_command)
        is_valid = self.validate_parsed_command(parsed_command)
        return parsed_command, is_valid


# Example usage and testing
if __name__ == "__main__":
    # Test the command parser
    parser = CommandParser()

    # Test commands
    test_commands = [
        "Pick up the red ball",
        "Go to the kitchen",
        "Point to the blue cube",
        "Bring me the green bottle",
        "Move to the table",
        "Grab the yellow toy"
    ]

    for cmd_text in test_commands:
        # Create a voice command
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = cmd_text

        # Parse the command
        parsed = parser.parse_command(voice_cmd)
        is_valid = parser.validate_parsed_command(parsed)

        print(f"Command: {cmd_text}")
        print(f"Parsed: {parsed}")
        print(f"Valid: {is_valid}")
        print("---")

    # Test with context
    context = {
        "environment": {"objects": ["red ball", "blue cube"]},
        "user": {"name": "John", "location": "living room"}
    }

    voice_cmd = VoiceCommand()
    voice_cmd.transcript = "Pick up the ball"
    parsed = parser.parse_command(voice_cmd)
    enhanced = parser.enhance_with_context(parsed, context)

    print(f"Original: {parsed}")
    print(f"Enhanced with context: {enhanced}")