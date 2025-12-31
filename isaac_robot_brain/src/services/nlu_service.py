"""
Natural Language Understanding Service

This module implements the language understanding service with GPT integration
for command interpretation and intent extraction.
"""

import asyncio
import logging
import os
from typing import Dict, Any, List, Tuple, Optional
from enum import Enum

try:
    import openai
    from openai import OpenAI
except ImportError:
    OpenAI = None
    openai = None

from ..models.voice_command import VoiceCommand


class IntentType(Enum):
    """Enumeration of possible command intents"""
    OBJECT_MANIPULATION = "object_manipulation"
    NAVIGATION = "navigation"
    INFORMATION_REQUEST = "information_request"
    GESTURE = "gesture"
    SPEAK = "speak"
    SENSING = "sensing"
    UNKNOWN = "unknown"


class NLUService:
    """
    Natural Language Understanding service with GPT integration for command interpretation
    and intent extraction.
    """

    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-3.5-turbo"):
        """
        Initialize the NLU service.

        Args:
            api_key: OpenAI API key (if not provided, will use OPENAI_API_KEY environment variable)
            model: GPT model to use for language understanding
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key is required")

        self.model = model
        self.logger = logging.getLogger(__name__)

        if OpenAI is not None:
            self.client = OpenAI(api_key=self.api_key)
            self.logger.info(f"Initialized NLU service with model: {self.model}")
        else:
            raise RuntimeError("OpenAI library not available")

    async def interpret_command(self, voice_command: VoiceCommand) -> Tuple[str, Dict[str, Any]]:
        """
        Interpret a voice command using GPT to extract intent and parameters.

        Args:
            voice_command: The voice command to interpret

        Returns:
            Tuple of (intent, parameters)
        """
        if not voice_command.transcript:
            raise ValueError("Voice command must have a transcript")

        try:
            # Create a prompt for GPT to interpret the command
            prompt = self._create_interpretation_prompt(voice_command.transcript)

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Low temperature for consistent interpretation
                max_tokens=200,
                response_format={"type": "json_object"}  # Expect JSON response
            )

            interpretation = response.choices[0].message.content
            interpretation_data = self._parse_interpretation(interpretation)

            intent = interpretation_data.get("intent", "unknown")
            parameters = interpretation_data.get("parameters", {})

            self.logger.info(f"Interpreted command: '{voice_command.transcript}' -> intent: {intent}, params: {parameters}")
            return intent, parameters

        except Exception as e:
            self.logger.error(f"Error interpreting command '{voice_command.transcript}': {str(e)}")
            return "unknown", {}

    def _create_interpretation_prompt(self, transcript: str) -> str:
        """
        Create a prompt for command interpretation.

        Args:
            transcript: The voice command transcript

        Returns:
            Formatted prompt string
        """
        return f"""
        Analyze the following voice command and extract the intent and parameters.
        Return the response in JSON format with 'intent' and 'parameters' fields.

        Command: "{transcript}"

        Intent should be one of: object_manipulation, navigation, information_request, gesture, speak, sensing, unknown

        Parameters should include relevant information like objects, locations, colors, etc.

        Example response:
        {{
            "intent": "object_manipulation",
            "parameters": {{
                "action": "pick_up",
                "object": "red ball",
                "color": "red",
                "object_type": "ball"
            }}
        }}
        """

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt for the language model.

        Returns:
            System prompt string
        """
        return """
        You are a robot command interpreter. Your task is to understand human voice commands
        and extract the intent and relevant parameters. Focus on understanding the action
        the human wants the robot to perform and extract any relevant objects, locations,
        colors, or other parameters mentioned in the command. Return the response as JSON
        with 'intent' and 'parameters' fields.
        """

    def _parse_interpretation(self, interpretation: str) -> Dict[str, Any]:
        """
        Parse the interpretation response from the language model.

        Args:
            interpretation: Raw response from the language model

        Returns:
            Parsed interpretation data
        """
        import json
        try:
            # Try to parse as JSON
            data = json.loads(interpretation)
            return {
                "intent": data.get("intent", "unknown"),
                "parameters": data.get("parameters", {})
            }
        except json.JSONDecodeError:
            # If JSON parsing fails, return default values
            self.logger.warning(f"Could not parse interpretation as JSON: {interpretation}")
            return {"intent": "unknown", "parameters": {}}

    def interpret_command_sync(self, voice_command: VoiceCommand) -> Tuple[str, Dict[str, Any]]:
        """
        Synchronous version of interpret_command.

        Args:
            voice_command: The voice command to interpret

        Returns:
            Tuple of (intent, parameters)
        """
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        return loop.run_until_complete(self.interpret_command(voice_command))

    async def batch_interpret_commands(self, voice_commands: List[VoiceCommand]) -> List[Tuple[str, Dict[str, Any]]]:
        """
        Interpret multiple voice commands concurrently.

        Args:
            voice_commands: List of voice commands to interpret

        Returns:
            List of tuples (intent, parameters) for each command
        """
        tasks = [self.interpret_command(cmd) for cmd in voice_commands]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Handle any exceptions that occurred during processing
        processed_results = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                self.logger.error(f"Error interpreting command {i}: {str(result)}")
                processed_results.append(("unknown", {}))
            else:
                processed_results.append(result)

        return processed_results

    def extract_entities(self, transcript: str) -> Dict[str, Any]:
        """
        Extract named entities from the transcript.

        Args:
            transcript: The voice command transcript

        Returns:
            Dictionary of extracted entities
        """
        # This is a simplified entity extraction - in practice, you'd use more sophisticated NLP
        entities = {
            "objects": [],
            "locations": [],
            "colors": [],
            "actions": []
        }

        # Simple keyword-based extraction (this would be enhanced with proper NLP in production)
        transcript_lower = transcript.lower()

        # Common object keywords
        object_keywords = ["ball", "cup", "box", "table", "chair", "book", "cup", "bottle"]
        for obj in object_keywords:
            if obj in transcript_lower:
                entities["objects"].append(obj)

        # Common location keywords
        location_keywords = ["kitchen", "living room", "bedroom", "office", "table", "shelf", "cabinet"]
        for loc in location_keywords:
            if loc in transcript_lower:
                entities["locations"].append(loc)

        # Common color keywords
        color_keywords = ["red", "blue", "green", "yellow", "black", "white", "orange", "purple", "pink", "brown"]
        for color in color_keywords:
            if color in transcript_lower:
                entities["colors"].append(color)

        # Common action keywords
        action_keywords = ["pick up", "grab", "take", "move", "go to", "bring", "point to", "show"]
        for action in action_keywords:
            if action in transcript_lower:
                entities["actions"].append(action)

        return entities

    def classify_intent(self, transcript: str) -> IntentType:
        """
        Classify the intent of a transcript using simple keyword matching.

        Args:
            transcript: The voice command transcript

        Returns:
            Classified intent type
        """
        transcript_lower = transcript.lower()

        # Navigation intents
        navigation_keywords = ["go to", "move to", "navigate", "walk to", "go to the"]
        if any(keyword in transcript_lower for keyword in navigation_keywords):
            return IntentType.NAVIGATION

        # Object manipulation intents
        manipulation_keywords = ["pick up", "grab", "take", "get", "bring", "lift", "hold", "catch"]
        if any(keyword in transcript_lower for keyword in manipulation_keywords):
            return IntentType.OBJECT_MANIPULATION

        # Information request intents
        info_keywords = ["what is", "tell me", "how many", "where is", "describe"]
        if any(keyword in transcript_lower for keyword in info_keywords):
            return IntentType.INFORMATION_REQUEST

        # Gesture intents
        gesture_keywords = ["point to", "wave", "show", "indicate", "point at"]
        if any(keyword in transcript_lower for keyword in gesture_keywords):
            return IntentType.GESTURE

        # Default to unknown
        return IntentType.UNKNOWN

    async def refine_interpretation(self, voice_command: VoiceCommand, context: Dict[str, Any] = None) -> Tuple[str, Dict[str, Any]]:
        """
        Refine the interpretation based on additional context.

        Args:
            voice_command: The voice command to interpret
            context: Additional context information (environment, previous commands, etc.)

        Returns:
            Tuple of (refined_intent, refined_parameters)
        """
        intent, parameters = await self.interpret_command(voice_command)

        # If context is provided, refine the interpretation
        if context:
            # Create a refinement prompt that includes context
            refinement_prompt = self._create_refinement_prompt(voice_command.transcript, context)

            try:
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self._get_system_prompt()},
                        {"role": "user", "content": refinement_prompt}
                    ],
                    temperature=0.1,
                    max_tokens=200,
                    response_format={"type": "json_object"}
                )

                refinement = response.choices[0].message.content
                refinement_data = self._parse_interpretation(refinement)

                # Update intent and parameters with refined values
                refined_intent = refinement_data.get("intent", intent)
                refined_parameters = {**parameters, **refinement_data.get("parameters", {})}

                return refined_intent, refined_parameters

            except Exception as e:
                self.logger.error(f"Error refining interpretation: {str(e)}")
                # Return original interpretation if refinement fails
                return intent, parameters

        return intent, parameters

    def _create_refinement_prompt(self, transcript: str, context: Dict[str, Any]) -> str:
        """
        Create a prompt for refining interpretation with context.

        Args:
            transcript: The voice command transcript
            context: Additional context information

        Returns:
            Formatted refinement prompt
        """
        return f"""
        Analyze the following voice command in the context of the provided environment information.
        Return the response in JSON format with 'intent' and 'parameters' fields.

        Command: "{transcript}"

        Context: {context}

        Intent should be one of: object_manipulation, navigation, information_request, gesture, speak, sensing, unknown

        Parameters should include relevant information like objects, locations, colors, etc.
        Consider the context when interpreting ambiguous commands.
        """

    def get_command_complexity(self, transcript: str) -> int:
        """
        Estimate the complexity of a command based on its length and structure.

        Args:
            transcript: The voice command transcript

        Returns:
            Complexity level (1-5, where 5 is most complex)
        """
        # Simple complexity estimation based on command length and keywords
        complexity = 1  # Base complexity

        # Add complexity for longer commands
        word_count = len(transcript.split())
        if word_count > 10:
            complexity = min(5, complexity + 1)
        if word_count > 15:
            complexity = min(5, complexity + 1)

        # Add complexity for multi-step commands
        multi_step_indicators = ["and", "then", "after", "before", "while"]
        if any(indicator in transcript.lower() for indicator in multi_step_indicators):
            complexity = min(5, complexity + 1)

        # Add complexity for conditional commands
        conditional_indicators = ["if", "when", "unless"]
        if any(indicator in transcript.lower() for indicator in conditional_indicators):
            complexity = min(5, complexity + 1)

        return complexity


# Example usage and testing
if __name__ == "__main__":
    import os

    # Test the NLU service
    try:
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            print("OPENAI_API_KEY environment variable not set. Skipping NLU service test.")
        else:
            nlu_service = NLUService(api_key=api_key)

            # Test command
            test_command = VoiceCommand()
            test_command.transcript = "Pick up the red ball from the table"

            # Test interpretation
            intent, parameters = nlu_service.interpret_command_sync(test_command)
            print(f"Interpreted intent: {intent}")
            print(f"Parameters: {parameters}")

            # Test entity extraction
            entities = nlu_service.extract_entities(test_command.transcript)
            print(f"Extracted entities: {entities}")

            # Test intent classification
            classified_intent = nlu_service.classify_intent(test_command.transcript)
            print(f"Classified intent: {classified_intent}")

    except Exception as e:
        print(f"Error testing NLU service: {e}")