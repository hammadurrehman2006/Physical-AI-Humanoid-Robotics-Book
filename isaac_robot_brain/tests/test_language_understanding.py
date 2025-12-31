"""
Unit Tests for Language Understanding Components

This module contains unit tests for the language understanding functionality
including the NLU service and command parser.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import json

from src.services.nlu_service import NLUService, IntentType
from src.services.command_parser import CommandParser, CommandAction
from src.models.voice_command import VoiceCommand


class MockOpenAIClient:
    """Mock OpenAI client for testing."""

    class ChatCompletions:
        def create(self, model, messages, temperature, max_tokens, response_format):
            # Mock response for command interpretation
            mock_response = {
                "choices": [{
                    "message": {
                        "content": json.dumps({
                            "intent": "object_manipulation",
                            "parameters": {
                                "action": "pick_up",
                                "object": "red ball",
                                "color": "red",
                                "object_type": "ball"
                            }
                        })
                    }
                }]
            }
            return MagicMock(**{"choices": [MagicMock(message=MagicMock(content=json.dumps({
                "intent": "object_manipulation",
                "parameters": {
                    "action": "pick_up",
                    "object": "red ball",
                    "color": "red",
                    "object_type": "ball"
                }
            })))})

    def __init__(self):
        self.chat = MagicMock()
        self.chat.completions = self.ChatCompletions()


class TestNLUService(unittest.TestCase):
    """Unit tests for the NLU service."""

    @patch('src.services.nlu_service.OpenAI')
    def setUp(self, mock_openai):
        """Set up test fixtures."""
        # Mock the OpenAI client
        self.mock_client = MockOpenAIClient()
        mock_openai.return_value = self.mock_client

        self.nlu_service = NLUService(api_key="test_key")

    @patch('src.services.nlu_service.asyncio.get_event_loop')
    def test_interpret_command(self, mock_get_loop):
        """Test interpreting a voice command."""
        # Mock the event loop
        mock_loop = Mock()
        mock_get_loop.return_value = mock_loop
        mock_loop.run_until_complete.return_value = ("object_manipulation", {
            "action": "pick_up",
            "object": "red ball",
            "color": "red",
            "object_type": "ball"
        })

        # Create a test voice command
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Pick up the red ball"

        # Test interpretation
        intent, parameters = self.nlu_service.interpret_command_sync(voice_cmd)

        self.assertEqual(intent, "object_manipulation")
        self.assertEqual(parameters["action"], "pick_up")
        self.assertEqual(parameters["object"], "red ball")

    def test_classify_intent(self):
        """Test intent classification."""
        # Test navigation intent
        intent = self.nlu_service.classify_intent("Go to the kitchen")
        self.assertEqual(intent, IntentType.NAVIGATION)

        # Test object manipulation intent
        intent = self.nlu_service.classify_intent("Pick up the red ball")
        self.assertEqual(intent, IntentType.OBJECT_MANIPULATION)

        # Test information request intent
        intent = self.nlu_service.classify_intent("What is on the table?")
        self.assertEqual(intent, IntentType.INFORMATION_REQUEST)

        # Test gesture intent
        intent = self.nlu_service.classify_intent("Point to the red ball")
        self.assertEqual(intent, IntentType.GESTURE)

        # Test unknown intent
        intent = self.nlu_service.classify_intent("Random text")
        self.assertEqual(intent, IntentType.UNKNOWN)

    def test_extract_entities(self):
        """Test entity extraction."""
        transcript = "Pick up the red ball from the table"
        entities = self.nlu_service.extract_entities(transcript)

        self.assertIn("ball", entities["objects"])
        self.assertIn("table", entities["locations"])
        self.assertIn("red", entities["colors"])
        self.assertIn("pick up", entities["actions"])

    def test_get_command_complexity(self):
        """Test command complexity estimation."""
        # Simple command
        complexity = self.nlu_service.get_command_complexity("Pick up ball")
        self.assertGreaterEqual(complexity, 1)
        self.assertLessEqual(complexity, 5)

        # Complex command
        complexity = self.nlu_service.get_command_complexity("Go to the kitchen and pick up the red ball then bring it to me")
        self.assertGreaterEqual(complexity, 1)
        self.assertLessEqual(complexity, 5)


class TestCommandParser(unittest.TestCase):
    """Unit tests for the CommandParser service."""

    def setUp(self):
        """Set up test fixtures."""
        self.parser = CommandParser()

    def test_parse_pickup_command(self):
        """Test parsing a pickup command."""
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Pick up the red ball"

        result = self.parser.parse_command(voice_cmd)

        self.assertEqual(result["action"], CommandAction.PICK_UP.value)
        self.assertIn("object_type", result["parameters"])
        self.assertIn("color", result["parameters"])
        self.assertGreater(result["confidence"], 0.5)

    def test_parse_move_command(self):
        """Test parsing a move command."""
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Go to the kitchen"

        result = self.parser.parse_command(voice_cmd)

        self.assertEqual(result["action"], CommandAction.MOVE_TO.value)
        self.assertIn("location", result["parameters"])
        self.assertIn("kitchen", result["parameters"]["location"])

    def test_parse_point_command(self):
        """Test parsing a point command."""
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Point to the blue cube"

        result = self.parser.parse_command(voice_cmd)

        self.assertEqual(result["action"], CommandAction.POINT_TO.value)
        self.assertIn("object_type", result["parameters"])
        self.assertIn("color", result["parameters"])

    def test_parse_bring_command(self):
        """Test parsing a bring command."""
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Bring me the green bottle"

        result = self.parser.parse_command(voice_cmd)

        self.assertEqual(result["action"], CommandAction.BRING.value)
        self.assertIn("object_type", result["parameters"])
        self.assertIn("color", result["parameters"])

    def test_keyword_parse_unknown_command(self):
        """Test parsing an unknown command using keywords."""
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Do something with the object"

        result = self.parser.parse_command(voice_cmd)

        # Should return unknown action with some parameters
        self.assertIn("action", result)
        self.assertIn("parameters", result)

    def test_validate_parsed_command(self):
        """Test validating a parsed command."""
        valid_command = {
            "action": "pick_up",
            "parameters": {"object": "ball"},
            "confidence": 0.8
        }

        invalid_command = {
            "action": "invalid_action",
            "parameters": {"object": "ball"},
            "confidence": 1.5  # Invalid confidence
        }

        self.assertTrue(self.parser.validate_parsed_command(valid_command))
        self.assertFalse(self.parser.validate_parsed_command(invalid_command))

    def test_enhance_with_context(self):
        """Test enhancing a parsed command with context."""
        parsed_command = {
            "action": "pick_up",
            "parameters": {"object": "ball"},
            "confidence": 0.7
        }

        context = {
            "environment": {"objects": ["red ball", "blue cube"]},
            "user": {"name": "John"}
        }

        enhanced = self.parser.enhance_with_context(parsed_command, context)

        self.assertIn("environment", enhanced["parameters"])
        self.assertIn("user", enhanced["parameters"])
        self.assertGreater(enhanced["confidence"], parsed_command["confidence"])

    def test_get_command_template(self):
        """Test getting command templates."""
        template = self.parser.get_command_template(CommandAction.PICK_UP)

        self.assertIn("required", template)
        self.assertIn("optional", template)
        self.assertIn("description", template)
        self.assertEqual(template["description"], "Pick up an object")

    def test_parse_with_validation(self):
        """Test parsing with validation."""
        voice_cmd = VoiceCommand()
        voice_cmd.transcript = "Pick up the red ball"

        parsed, is_valid = self.parser.parse_with_validation(voice_cmd)

        self.assertIsInstance(parsed, dict)
        self.assertIsInstance(is_valid, bool)
        self.assertTrue(is_valid)


if __name__ == '__main__':
    unittest.main()