"""
Multi-Modal Fusion Service

This module implements the multi-modal fusion service that combines voice, vision,
and action data for coherent responses.
"""

import asyncio
import logging
from typing import Dict, Any, List, Optional, Tuple
from enum import Enum
import numpy as np
from dataclasses import dataclass

from ..models.voice_command import VoiceCommand
from ..models.visual_perception import VisualPerception
from ..models.action_plan import ActionPlan
from ..models.multi_modal_context import MultiModalContext


class FusionStrategy(Enum):
    """Enumeration of fusion strategies"""
    EARLY_FUSION = "early_fusion"
    LATE_FUSION = "late_fusion"
    INTERMEDIATE_FUSION = "intermediate_fusion"
    PROBABILISTIC_FUSION = "probabilistic_fusion"
    ATTENTION_BASED_FUSION = "attention_based_fusion"


class ConfidenceAggregationMethod(Enum):
    """Enumeration of confidence aggregation methods"""
    AVERAGE = "average"
    WEIGHTED_AVERAGE = "weighted_average"
    MAX = "max"
    PROBABILISTIC = "probabilistic"


@dataclass
class FusionResult:
    """Result of multi-modal fusion"""
    context: MultiModalContext
    confidence: float
    action_plan: Optional[ActionPlan] = None
    relevant_objects: List[Dict[str, Any]] = None


class MultiModalFusionService:
    """
    Multi-modal fusion service that combines voice, vision, and action data
    for coherent responses and decision making.
    """

    def __init__(self, default_strategy: FusionStrategy = FusionStrategy.PROBABILISTIC_FUSION):
        """
        Initialize the multi-modal fusion service.

        Args:
            default_strategy: Default fusion strategy to use
        """
        self.default_strategy = default_strategy
        self.logger = logging.getLogger(__name__)
        self.fusion_history = []

    def fuse_voice_and_vision(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception,
        strategy: Optional[FusionStrategy] = None
    ) -> FusionResult:
        """
        Fuse voice command and visual perception data.

        Args:
            voice_command: The voice command to fuse
            visual_perception: The visual perception data to fuse
            strategy: Fusion strategy to use (defaults to service default)

        Returns:
            FusionResult containing combined context and confidence
        """
        if strategy is None:
            strategy = self.default_strategy

        # Create initial multi-modal context
        context = MultiModalContext(
            voice_context={
                "transcript": voice_command.transcript,
                "intent": voice_command.intent,
                "confidence": voice_command.confidence,
                "parameters": voice_command.parameters
            },
            visual_context={
                "objects": [obj.__dict__ for obj in visual_perception.objects],
                "detection_accuracy": visual_perception.detection_accuracy,
                "camera_source": visual_perception.camera_source
            },
            fusion_strategy=strategy.value
        )

        # Apply the specified fusion strategy
        if strategy == FusionStrategy.EARLY_FUSION:
            result = self._early_fusion(voice_command, visual_perception, context)
        elif strategy == FusionStrategy.LATE_FUSION:
            result = self._late_fusion(voice_command, visual_perception, context)
        elif strategy == FusionStrategy.INTERMEDIATE_FUSION:
            result = self._intermediate_fusion(voice_command, visual_perception, context)
        elif strategy == FusionStrategy.PROBABILISTIC_FUSION:
            result = self._probabilistic_fusion(voice_command, visual_perception, context)
        elif strategy == FusionStrategy.ATTENTION_BASED_FUSION:
            result = self._attention_based_fusion(voice_command, visual_perception, context)
        else:
            result = self._probabilistic_fusion(voice_command, visual_perception, context)

        # Calculate overall confidence
        result.confidence = self._calculate_confidence(
            voice_command.confidence,
            visual_perception.detection_accuracy
        )

        # Update context confidence
        context.confidence_score = result.confidence

        # Store in history
        self.fusion_history.append({
            "timestamp": context.timestamp,
            "strategy": strategy.value,
            "confidence": result.confidence,
            "command_transcript": voice_command.transcript
        })

        return result

    def _early_fusion(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception,
        context: MultiModalContext
    ) -> FusionResult:
        """
        Perform early fusion by combining raw features before processing.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data
            context: The multi-modal context

        Returns:
            FusionResult
        """
        # In early fusion, we combine raw data representations
        # For this implementation, we'll focus on aligning voice commands with visual objects
        relevant_objects = self._find_relevant_objects(voice_command, visual_perception)

        # Update context with early fusion results
        context.update_visual_context({
            "relevant_objects": [obj.__dict__ for obj in relevant_objects],
            "fusion_method": "early"
        })

        return FusionResult(
            context=context,
            confidence=0.0,  # Will be calculated later
            relevant_objects=[obj.__dict__ for obj in relevant_objects]
        )

    def _late_fusion(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception,
        context: MultiModalContext
    ) -> FusionResult:
        """
        Perform late fusion by combining processed outputs.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data
            context: The multi-modal context

        Returns:
            FusionResult
        """
        # In late fusion, we combine the processed outputs of each modality
        relevant_objects = self._find_relevant_objects(voice_command, visual_perception)

        # Update context with late fusion results
        context.update_visual_context({
            "relevant_objects": [obj.__dict__ for obj in relevant_objects],
            "fusion_method": "late"
        })

        # Combine the processed results
        combined_intent = self._combine_intents(voice_command.intent, relevant_objects)
        context.update_voice_context({"combined_intent": combined_intent})

        return FusionResult(
            context=context,
            confidence=0.0,  # Will be calculated later
            relevant_objects=[obj.__dict__ for obj in relevant_objects]
        )

    def _intermediate_fusion(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception,
        context: MultiModalContext
    ) -> FusionResult:
        """
        Perform intermediate fusion by combining at feature level.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data
            context: The multi-modal context

        Returns:
            FusionResult
        """
        # Find relevant objects based on voice command
        relevant_objects = self._find_relevant_objects(voice_command, visual_perception)

        # Create intermediate representations
        voice_features = self._extract_voice_features(voice_command)
        visual_features = self._extract_visual_features(relevant_objects)

        # Combine features
        combined_features = self._combine_features(voice_features, visual_features)

        # Update context with intermediate fusion results
        context.update_visual_context({
            "relevant_objects": [obj.__dict__ for obj in relevant_objects],
            "fusion_method": "intermediate",
            "combined_features": combined_features
        })

        return FusionResult(
            context=context,
            confidence=0.0,  # Will be calculated later
            relevant_objects=[obj.__dict__ for obj in relevant_objects]
        )

    def _probabilistic_fusion(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception,
        context: MultiModalContext
    ) -> FusionResult:
        """
        Perform probabilistic fusion using Bayesian methods.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data
            context: The multi-modal context

        Returns:
            FusionResult
        """
        # Find relevant objects
        relevant_objects = self._find_relevant_objects(voice_command, visual_perception)

        # Calculate probabilities for each object being the target
        object_probabilities = self._calculate_object_probabilities(
            voice_command, relevant_objects
        )

        # Update context with probabilistic fusion results
        context.update_visual_context({
            "relevant_objects": [obj.__dict__ for obj in relevant_objects],
            "object_probabilities": object_probabilities,
            "fusion_method": "probabilistic"
        })

        return FusionResult(
            context=context,
            confidence=0.0,  # Will be calculated later
            relevant_objects=[obj.__dict__ for obj in relevant_objects]
        )

    def _attention_based_fusion(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception,
        context: MultiModalContext
    ) -> FusionResult:
        """
        Perform attention-based fusion focusing on relevant information.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data
            context: The multi-modal context

        Returns:
            FusionResult
        """
        # Find relevant objects
        relevant_objects = self._find_relevant_objects(voice_command, visual_perception)

        # Calculate attention weights based on relevance
        attention_weights = self._calculate_attention_weights(
            voice_command, relevant_objects
        )

        # Update context with attention-based fusion results
        context.update_visual_context({
            "relevant_objects": [obj.__dict__ for obj in relevant_objects],
            "attention_weights": attention_weights,
            "fusion_method": "attention_based"
        })

        return FusionResult(
            context=context,
            confidence=0.0,  # Will be calculated later
            relevant_objects=[obj.__dict__ for obj in relevant_objects]
        )

    def _find_relevant_objects(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception
    ) -> List[Any]:
        """
        Find objects in visual perception that match voice command parameters.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data

        Returns:
            List of relevant detected objects
        """
        if not voice_command.parameters:
            return visual_perception.objects

        relevant_objects = []
        command_params = voice_command.parameters

        for obj in visual_perception.objects:
            is_relevant = True

            # Check if object matches specified color
            if "color" in command_params:
                if command_params["color"].lower() not in obj.color.lower():
                    is_relevant = False

            # Check if object matches specified object type
            if "object_type" in command_params:
                if command_params["object_type"].lower() not in obj.class_name.lower():
                    is_relevant = False

            # Check if object matches specified object
            if "object" in command_params:
                object_desc = f"{obj.color} {obj.class_name}".lower()
                if command_params["object"].lower() not in object_desc:
                    is_relevant = False

            if is_relevant:
                relevant_objects.append(obj)

        return relevant_objects

    def _calculate_object_probabilities(
        self,
        voice_command: VoiceCommand,
        objects: List[Any]
    ) -> Dict[str, float]:
        """
        Calculate probabilities for each object being the target.

        Args:
            voice_command: The voice command
            objects: List of detected objects

        Returns:
            Dictionary mapping object IDs to probabilities
        """
        probabilities = {}

        for obj in objects:
            prob = 1.0  # Start with base probability

            # Increase probability if color matches
            if "color" in voice_command.parameters:
                if voice_command.parameters["color"].lower() in obj.color.lower():
                    prob *= 2.0

            # Increase probability if object type matches
            if "object_type" in voice_command.parameters:
                if voice_command.parameters["object_type"].lower() in obj.class_name.lower():
                    prob *= 2.0

            # Increase probability based on detection confidence
            prob *= obj.confidence

            # Normalize to ensure probabilities sum to 1 (or less)
            probabilities[obj.object_id] = min(1.0, prob)

        # Normalize probabilities so they sum to 1
        total_prob = sum(probabilities.values())
        if total_prob > 0:
            for obj_id in probabilities:
                probabilities[obj_id] /= total_prob

        return probabilities

    def _calculate_attention_weights(
        self,
        voice_command: VoiceCommand,
        objects: List[Any]
    ) -> Dict[str, float]:
        """
        Calculate attention weights for each object.

        Args:
            voice_command: The voice command
            objects: List of detected objects

        Returns:
            Dictionary mapping object IDs to attention weights
        """
        weights = {}

        for obj in objects:
            weight = 1.0  # Start with base weight

            # Increase weight if color matches
            if "color" in voice_command.parameters:
                if voice_command.parameters["color"].lower() in obj.color.lower():
                    weight *= 2.0

            # Increase weight if object type matches
            if "object_type" in voice_command.parameters:
                if voice_command.parameters["object_type"].lower() in obj.class_name.lower():
                    weight *= 2.0

            # Increase weight based on detection confidence
            weight *= obj.confidence

            weights[obj.object_id] = weight

        # Normalize weights
        total_weight = sum(weights.values())
        if total_weight > 0:
            for obj_id in weights:
                weights[obj_id] /= total_weight

        return weights

    def _extract_voice_features(self, voice_command: VoiceCommand) -> Dict[str, Any]:
        """
        Extract features from voice command.

        Args:
            voice_command: The voice command

        Returns:
            Dictionary of extracted features
        """
        features = {
            "transcript_length": len(voice_command.transcript or ""),
            "confidence": voice_command.confidence,
            "intent": voice_command.intent,
            "has_color": "color" in voice_command.parameters,
            "has_object_type": "object_type" in voice_command.parameters,
            "has_location": "location" in voice_command.parameters
        }

        return features

    def _extract_visual_features(self, objects: List[Any]) -> Dict[str, Any]:
        """
        Extract features from visual objects.

        Args:
            objects: List of detected objects

        Returns:
            Dictionary of extracted features
        """
        if not objects:
            return {"object_count": 0, "avg_confidence": 0.0}

        features = {
            "object_count": len(objects),
            "avg_confidence": np.mean([obj.confidence for obj in objects]) if objects else 0.0,
            "colors": list(set(obj.color for obj in objects)),
            "object_types": list(set(obj.class_name for obj in objects)),
            "avg_size": np.mean([np.prod(obj.size) for obj in objects]) if objects else 0.0
        }

        return features

    def _combine_features(self, voice_features: Dict[str, Any], visual_features: Dict[str, Any]) -> Dict[str, Any]:
        """
        Combine voice and visual features.

        Args:
            voice_features: Features extracted from voice
            visual_features: Features extracted from vision

        Returns:
            Combined features dictionary
        """
        combined = {
            "voice": voice_features,
            "visual": visual_features,
            "fusion_confidence": (voice_features["confidence"] + visual_features["avg_confidence"]) / 2
        }

        return combined

    def _combine_intents(self, voice_intent: Optional[str], relevant_objects: List[Any]) -> str:
        """
        Combine voice intent with visual information.

        Args:
            voice_intent: Intent from voice command
            relevant_objects: Relevant objects from visual perception

        Returns:
            Combined intent string
        """
        if not relevant_objects:
            return voice_intent or "unknown"

        # Create a combined intent that incorporates visual information
        obj_descriptions = [f"{obj.color} {obj.class_name}" for obj in relevant_objects[:3]]  # Limit to 3
        visual_info = ", ".join(obj_descriptions)

        if voice_intent:
            return f"{voice_intent} with {visual_info}"
        else:
            return f"act_on_objects: {visual_info}"

    def _calculate_confidence(self, voice_confidence: float, visual_confidence: float) -> float:
        """
        Calculate overall confidence from voice and visual confidences.

        Args:
            voice_confidence: Confidence from voice processing
            visual_confidence: Confidence from visual processing

        Returns:
            Combined confidence score
        """
        # Simple average for now - in practice, you might use more sophisticated methods
        return (voice_confidence + visual_confidence) / 2

    def generate_action_plan(
        self,
        fusion_result: FusionResult
    ) -> Optional[ActionPlan]:
        """
        Generate an action plan based on fusion result.

        Args:
            fusion_result: The result of multi-modal fusion

        Returns:
            Generated action plan, or None if not possible
        """
        context = fusion_result.context
        voice_context = context.voice_context
        visual_context = context.visual_context

        # Extract intent and parameters from voice context
        intent = voice_context.get("intent", "")
        parameters = voice_context.get("parameters", {})

        # Extract relevant objects from visual context
        relevant_objects = visual_context.get("relevant_objects", [])

        if not intent or not relevant_objects:
            self.logger.warning("Insufficient information to generate action plan")
            return None

        # Create actions based on intent and relevant objects
        actions = []

        # Determine action type based on intent
        if "pick" in intent.lower() or "grasp" in intent.lower() or "take" in intent.lower():
            for obj in relevant_objects:
                # Navigation action to approach the object
                nav_action = ActionPlan.__annotations__['actions'].__args__[0](  # This is a workaround
                    action_id=f"navigate_to_{obj.get('object_id', 'unknown')}",
                    type=type('ActionType', (), {'NAVIGATION': 'navigation'}).NAVIGATION,  # Mock ActionType
                    parameters={
                        "target_location": obj.get("position_3d", [0, 0, 0]),
                        "object_id": obj.get("object_id")
                    },
                    timeout=60.0,
                    preconditions=["robot_is_idle", "path_clear"],
                    postconditions=["robot_at_object"],
                    success_threshold=0.9
                )

                # Manipulation action to grasp the object
                manip_action = ActionPlan.__annotations__['actions'].__args__[0](
                    action_id=f"grasp_{obj.get('object_id', 'unknown')}",
                    type=type('ActionType', (), {'MANIPULATION': 'manipulation'}).MANIPULATION,
                    parameters={
                        "object_id": obj.get("object_id"),
                        "action": "grasp",
                        "object_pose": obj.get("position_3d", [0, 0, 0])
                    },
                    timeout=30.0,
                    preconditions=["robot_at_object", "gripper_open"],
                    postconditions=["object_grasped"],
                    success_threshold=0.85
                )

                actions.extend([nav_action, manip_action])

        elif "point" in intent.lower() or "show" in intent.lower():
            for obj in relevant_objects:
                gesture_action = ActionPlan.__annotations__['actions'].__args__[0](
                    action_id=f"point_to_{obj.get('object_id', 'unknown')}",
                    type=type('ActionType', (), {'GESTURE': 'gesture'}).GESTURE,
                    parameters={
                        "target_object": obj.get("object_id"),
                        "target_position": obj.get("position_3d", [0, 0, 0]),
                        "gesture_type": "point"
                    },
                    timeout=15.0,
                    preconditions=["robot_is_idle"],
                    postconditions=["gesture_completed"],
                    success_threshold=0.9
                )
                actions.append(gesture_action)

        # For this implementation, I'll create a simplified approach
        # since we can't easily reference the Action class from imports
        from ..models.action_plan import Action, ActionType

        # Create actual actions using the proper classes
        actual_actions = []
        if "pick" in intent.lower() or "grasp" in intent.lower() or "take" in intent.lower():
            for obj in relevant_objects:
                # Navigation action
                nav_action = Action(
                    type=ActionType.NAVIGATION,
                    parameters={
                        "target_location": "object_location",  # Would use actual position
                        "object_id": obj.get("object_id", ""),
                        "x": obj.get("position_3d", [0, 0, 0])[0] if obj.get("position_3d") else 0,
                        "y": obj.get("position_3d", [0, 0, 0])[1] if obj.get("position_3d") else 0,
                        "z": obj.get("position_3d", [0, 0, 0])[2] if obj.get("position_3d") else 0
                    },
                    timeout=60.0,
                    preconditions=["robot_is_idle", "path_clear"],
                    postconditions=["robot_at_object"],
                    success_threshold=0.9
                )

                # Manipulation action
                manip_action = Action(
                    type=ActionType.MANIPULATION,
                    parameters={
                        "object_id": obj.get("object_id", ""),
                        "action": "grasp",
                        "x": obj.get("position_3d", [0, 0, 0])[0] if obj.get("position_3d") else 0,
                        "y": obj.get("position_3d", [0, 0, 0])[1] if obj.get("position_3d") else 0,
                        "z": obj.get("position_3d", [0, 0, 0])[2] if obj.get("position_3d") else 0
                    },
                    timeout=30.0,
                    preconditions=["robot_at_object", "gripper_open"],
                    postconditions=["object_grasped"],
                    success_threshold=0.85
                )

                actual_actions.extend([nav_action, manip_action])

        elif "point" in intent.lower() or "show" in intent.lower():
            for obj in relevant_objects:
                gesture_action = Action(
                    type=ActionType.GESTURE,
                    parameters={
                        "target_object": obj.get("object_id", ""),
                        "x": obj.get("position_3d", [0, 0, 0])[0] if obj.get("position_3d") else 0,
                        "y": obj.get("position_3d", [0, 0, 0])[1] if obj.get("position_3d") else 0,
                        "z": obj.get("position_3d", [0, 0, 0])[2] if obj.get("position_3d") else 0,
                        "gesture_type": "point"
                    },
                    timeout=15.0,
                    preconditions=["robot_is_idle"],
                    postconditions=["gesture_completed"],
                    success_threshold=0.9
                )
                actual_actions.append(gesture_action)

        # Create action plan
        if actual_actions:
            action_plan = ActionPlan(
                command_id=voice_context.get("command_id", "unknown"),
                actions=actual_actions,
                priority=parameters.get("priority", 3),
                estimated_duration=len(actual_actions) * 30.0,
                required_resources=["navigation_system", "manipulator_arm"],
                success_criteria=["command_executed"],
                status=type('ActionStatus', (), {'PENDING': 'pending'}).PENDING  # Mock status
            )

            # Import the real status enum
            from ..models.action_plan import ActionStatus
            action_plan.status = ActionStatus.PENDING

            return action_plan

        return None

    def update_context_with_environment(
        self,
        context: MultiModalContext,
        environment_data: Dict[str, Any]
    ) -> MultiModalContext:
        """
        Update context with environment information.

        Args:
            context: The current multi-modal context
            environment_data: Environment information to add

        Returns:
            Updated context
        """
        context.update_environment_context(environment_data)
        return context

    def get_fusion_summary(self) -> Dict[str, Any]:
        """
        Get a summary of recent fusion activities.

        Returns:
            Summary dictionary
        """
        if not self.fusion_history:
            return {"total_fusions": 0}

        total_fusions = len(self.fusion_history)
        avg_confidence = np.mean([item["confidence"] for item in self.fusion_history])
        strategy_distribution = {}

        for item in self.fusion_history:
            strategy = item["strategy"]
            strategy_distribution[strategy] = strategy_distribution.get(strategy, 0) + 1

        return {
            "total_fusions": total_fusions,
            "average_confidence": avg_confidence,
            "strategy_distribution": strategy_distribution,
            "recent_commands": [item["command_transcript"] for item in self.fusion_history[-5:]]  # Last 5
        }

    def select_optimal_strategy(
        self,
        voice_command: VoiceCommand,
        visual_perception: VisualPerception
    ) -> FusionStrategy:
        """
        Select the optimal fusion strategy based on input characteristics.

        Args:
            voice_command: The voice command
            visual_perception: The visual perception data

        Returns:
            Optimal fusion strategy
        """
        # Simple heuristic for strategy selection:
        # - If voice confidence is high and visual confidence is high: use probabilistic
        # - If voice confidence is low: use attention-based to focus on visual
        # - If visual confidence is low: use attention-based to focus on voice
        # - For simple commands: use early fusion
        # - For complex commands: use late fusion

        voice_conf = voice_command.confidence
        visual_conf = visual_perception.detection_accuracy

        if voice_conf > 0.8 and visual_conf > 0.8:
            return FusionStrategy.PROBABILISTIC_FUSION
        elif voice_conf < 0.5:
            return FusionStrategy.ATTENTION_BASED_FUSION
        elif visual_conf < 0.5:
            return FusionStrategy.ATTENTION_BASED_FUSION
        elif len(voice_command.transcript or "") < 20:  # Simple command
            return FusionStrategy.EARLY_FUSION
        else:  # Complex command
            return FusionStrategy.LATE_FUSION


# Example usage and testing
if __name__ == "__main__":
    # Test the multi-modal fusion service
    from ..models.visual_perception import VisualPerception, DetectedObject
    from ..models.voice_command import VoiceCommand, VoiceCommandStatus

    # Create a fusion service
    fusion_service = MultiModalFusionService()

    # Create sample voice command
    voice_cmd = VoiceCommand(
        transcript="Pick up the red ball",
        confidence=0.92,
        intent="object_manipulation",
        parameters={
            "action": "pick_up",
            "object": "red ball",
            "color": "red",
            "object_type": "ball"
        },
        status=VoiceCommandStatus.PROCESSED
    )

    # Create sample visual perception
    red_ball = DetectedObject(
        class_name="ball",
        bounding_box=[100.0, 200.0, 50.0, 50.0],
        confidence=0.95,
        position_3d=[1.0, 2.0, 0.5],
        size=[0.1, 0.1, 0.1],
        color="red"
    )

    blue_cube = DetectedObject(
        class_name="cube",
        bounding_box=[300.0, 150.0, 60.0, 60.0],
        confidence=0.88,
        position_3d=[2.0, 1.0, 0.3],
        size=[0.15, 0.15, 0.15],
        color="blue"
    )

    visual_perception = VisualPerception(
        camera_source="front_camera",
        detection_accuracy=0.90,
        objects=[red_ball, blue_cube]
    )

    # Perform fusion with different strategies
    strategies = [
        FusionStrategy.EARLY_FUSION,
        FusionStrategy.LATE_FUSION,
        FusionStrategy.PROBABILISTIC_FUSION,
        FusionStrategy.ATTENTION_BASED_FUSION
    ]

    for strategy in strategies:
        print(f"\n--- Testing {strategy.value} ---")
        result = fusion_service.fuse_voice_and_vision(
            voice_cmd, visual_perception, strategy
        )

        print(f"Confidence: {result.confidence:.2f}")
        print(f"Relevant objects: {len(result.relevant_objects) if result.relevant_objects else 0}")

        # Generate action plan
        action_plan = fusion_service.generate_action_plan(result)
        if action_plan:
            print(f"Generated action plan with {len(action_plan.actions)} actions")
            for action in action_plan.actions:
                print(f"  - {action.type.value}: {action.parameters}")

    # Get fusion summary
    summary = fusion_service.get_fusion_summary()
    print(f"\nFusion Summary: {summary}")

    # Test strategy selection
    optimal_strategy = fusion_service.select_optimal_strategy(voice_cmd, visual_perception)
    print(f"\nOptimal strategy: {optimal_strategy.value}")