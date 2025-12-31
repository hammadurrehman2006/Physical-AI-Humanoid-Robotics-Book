---
sidebar_position: 5
title: "Multi-Modal Fusion"
---

# Multi-Modal Fusion

In this section, we'll implement the multi-modal fusion component that combines information from vision, language, and action systems into a coherent understanding and response. This is the heart of our Vision-Language-Action system where all modalities work together seamlessly.

## Overview

Multi-modal fusion is the integration layer that combines visual perception, language understanding, and action execution into a unified system. This section covers:
- Cross-modal attention mechanisms and fusion strategies
- Uncertainty handling and confidence propagation
- Temporal consistency and state tracking
- Grounded language understanding with visual context
- Coordinated action planning using multiple modalities

## Learning Objectives

By the end of this section, you will be able to:
- Implement cross-modal fusion mechanisms that combine vision, language, and action information
- Handle uncertainty and confidence across different modalities
- Maintain temporal consistency in multi-modal reasoning
- Create grounded language understanding systems that use visual context
- Design coordinated action planning using multi-modal information

## Cross-Modal Attention and Fusion

### Attention Mechanisms

Cross-modal attention allows different modalities to attend to relevant information in each other:

```python
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Dict, List, Tuple, Any

class CrossModalAttention(nn.Module):
    def __init__(self, vision_dim: int, language_dim: int, hidden_dim: int = 512):
        super().__init__()
        self.vision_dim = vision_dim
        self.language_dim = language_dim
        self.hidden_dim = hidden_dim

        # Linear projections for attention computation
        self.vision_proj = nn.Linear(vision_dim, hidden_dim)
        self.language_proj = nn.Linear(language_dim, hidden_dim)
        self.fusion_proj = nn.Linear(hidden_dim * 2, hidden_dim)

        # Attention weights
        self.attention_weights = nn.Linear(hidden_dim * 2, 1)

    def forward(self, vision_features: torch.Tensor,
                language_features: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Compute cross-modal attention between vision and language features

        Args:
            vision_features: [batch_size, num_regions, vision_dim]
            language_features: [batch_size, seq_len, language_dim]

        Returns:
            attended_vision: [batch_size, num_regions, hidden_dim]
            attended_language: [batch_size, seq_len, hidden_dim]
        """
        batch_size, num_regions, _ = vision_features.shape
        _, seq_len, _ = language_features.shape

        # Project features to common space
        vision_proj = self.vision_proj(vision_features)  # [B, R, H]
        lang_proj = self.language_proj(language_features)  # [B, S, H]

        # Compute attention weights between vision and language
        # Expand dimensions for cross-attention computation
        vision_expanded = vision_proj.unsqueeze(2).expand(-1, -1, seq_len, -1)  # [B, R, S, H]
        lang_expanded = lang_proj.unsqueeze(1).expand(-1, num_regions, -1, -1)  # [B, R, S, H]

        # Concatenate and compute attention
        combined = torch.cat([vision_expanded, lang_expanded], dim=-1)  # [B, R, S, 2*H]
        attention_scores = self.attention_weights(combined).squeeze(-1)  # [B, R, S]

        # Apply softmax to get attention weights
        vision_attention = F.softmax(attention_scores, dim=2)  # [B, R, S] - vision attends to language
        lang_attention = F.softmax(attention_scores, dim=1)   # [B, R, S] - language attends to vision

        # Compute attended features
        attended_vision = torch.bmm(vision_attention, lang_proj)  # [B, R, H]
        attended_language = torch.bmm(lang_attention.transpose(1, 2), vision_proj)  # [B, S, H]

        # Project to final representation
        attended_vision = self.fusion_proj(torch.cat([vision_proj, attended_vision], dim=-1))
        attended_language = self.fusion_proj(torch.cat([lang_proj, attended_language], dim=-1))

        return attended_vision, attended_language

class MultiModalFusion(nn.Module):
    def __init__(self, vision_dim: int, language_dim: int, action_dim: int, hidden_dim: int = 512):
        super().__init__()
        self.hidden_dim = hidden_dim

        # Cross-modal attention modules
        self.vision_language_attention = CrossModalAttention(vision_dim, language_dim, hidden_dim)
        self.vision_action_attention = CrossModalAttention(vision_dim, action_dim, hidden_dim)
        self.language_action_attention = CrossModalAttention(language_dim, action_dim, hidden_dim)

        # Final fusion layer
        self.fusion_layer = nn.Sequential(
            nn.Linear(hidden_dim * 3, hidden_dim),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dim, hidden_dim)
        )

    def forward(self, vision_features: torch.Tensor,
                language_features: torch.Tensor,
                action_features: torch.Tensor) -> torch.Tensor:
        """
        Fuse vision, language, and action features through cross-modal attention
        """
        # Apply cross-modal attention
        attended_vision_lang, attended_language_vision = self.vision_language_attention(
            vision_features, language_features
        )

        # Average pooled features for action attention
        avg_vision = attended_vision_lang.mean(dim=1)  # [B, H]
        avg_language = attended_language_vision.mean(dim=1)  # [B, H]

        # Repeat action features to match sequence length
        repeated_action = action_features.unsqueeze(1).expand(-1, avg_vision.size(0), -1)  # [B, H, A]

        # Apply attention with action modality
        attended_vision_action, attended_action_vision = self.vision_action_attention(
            avg_vision.unsqueeze(1), action_features.unsqueeze(1)
        )

        attended_language_action, attended_action_language = self.language_action_attention(
            avg_language.unsqueeze(1), action_features.unsqueeze(1)
        )

        # Combine all attended features
        fused_features = self.fusion_layer(
            torch.cat([
                attended_vision_action.squeeze(1),
                attended_language_action.squeeze(1),
                action_features
            ], dim=-1)
        )

        return fused_features
```

### Late Fusion Strategy

For systems where each modality is processed separately, we can use late fusion:

```python
class LateFusionModule(nn.Module):
    def __init__(self, input_dims: List[int], output_dim: int, fusion_type: str = 'concat'):
        super().__init__()
        self.fusion_type = fusion_type
        self.input_dims = input_dims
        self.output_dim = output_dim

        if fusion_type == 'concat':
            total_input_dim = sum(input_dims)
            self.fusion_layer = nn.Sequential(
                nn.Linear(total_input_dim, output_dim),
                nn.ReLU(),
                nn.Dropout(0.2),
                nn.Linear(output_dim, output_dim)
            )
        elif fusion_type == 'attention':
            # Attention-based fusion
            self.attention_weights = nn.Parameter(torch.randn(len(input_dims)))
            self.fusion_layer = nn.Linear(sum(input_dims), output_dim)

    def forward(self, modalities: List[torch.Tensor]) -> torch.Tensor:
        """Fuse multiple modalities using late fusion"""
        if self.fusion_type == 'concat':
            # Concatenate all modalities
            fused_input = torch.cat(modalities, dim=-1)
            return self.fusion_layer(fused_input)

        elif self.fusion_type == 'attention':
            # Compute attention weights for each modality
            weights = F.softmax(self.attention_weights, dim=0)

            # Weighted sum of modalities
            weighted_modalities = []
            for i, modality in enumerate(modalities):
                weighted_modalities.append(modality * weights[i])

            fused_input = torch.cat(weighted_modalities, dim=-1)
            return self.fusion_layer(fused_input)

        elif self.fusion_type == 'gated':
            # Gated fusion where each modality has learnable gate
            gates = []
            for i, modality in enumerate(modalities):
                gate = torch.sigmoid(torch.randn(modality.shape[-1], device=modality.device))
                gates.append(modality * gate)

            fused_input = torch.cat(gates, dim=-1)
            return self.fusion_layer(fused_input)
```

## Uncertainty and Confidence Handling

### Confidence Propagation

Handling uncertainty across modalities is crucial for robust VLA systems:

```python
class UncertaintyHandler:
    def __init__(self):
        self.confidence_thresholds = {
            'vision': 0.7,
            'language': 0.6,
            'action': 0.8
        }
        self.uncertainty_models = {}

    def propagate_confidence(self, modality_outputs: Dict[str, Any]) -> Dict[str, float]:
        """Propagate confidence scores across modalities"""
        confidences = {}

        for modality, output in modality_outputs.items():
            if 'confidence' in output:
                confidences[modality] = output['confidence']
            else:
                # Estimate confidence based on output properties
                confidences[modality] = self._estimate_confidence(modality, output)

        return confidences

    def _estimate_confidence(self, modality: str, output: Any) -> float:
        """Estimate confidence for a modality output"""
        if modality == 'vision':
            # For vision, confidence based on detection scores
            if isinstance(output, list) and output:
                avg_score = sum([obj.get('confidence', 0) for obj in output]) / len(output)
                return avg_score
        elif modality == 'language':
            # For language, confidence based on parsing confidence
            if isinstance(output, dict) and 'confidence' in output:
                return output['confidence']
        elif modality == 'action':
            # For action, confidence based on feasibility
            return 0.9  # Default high confidence for action planning

        return 0.5  # Default confidence

    def compute_fused_confidence(self, confidences: Dict[str, float],
                                weights: Dict[str, float] = None) -> float:
        """Compute fused confidence score"""
        if weights is None:
            # Equal weights by default
            weights = {mod: 1.0 for mod in confidences.keys()}

        total_weight = sum(weights.values())
        if total_weight == 0:
            return 0.0

        weighted_confidence = sum(
            confidences[mod] * weights[mod] for mod in confidences.keys()
        ) / total_weight

        return weighted_confidence

    def should_proceed(self, fused_confidence: float, modality_confidences: Dict[str, float]) -> bool:
        """Determine if system should proceed with execution"""
        # Check individual modality thresholds
        for mod, conf in modality_confidences.items():
            threshold = self.confidence_thresholds.get(mod, 0.5)
            if conf < threshold:
                return False

        # Check fused confidence
        return fused_confidence > 0.7
```

### Bayesian Fusion

Using probabilistic methods for uncertainty handling:

```python
import numpy as np
from scipy.stats import norm

class BayesianFusion:
    def __init__(self):
        self.priors = {
            'object_present': 0.5,  # Prior probability
            'command_correct': 0.5,
            'action_feasible': 0.8
        }

    def bayesian_update(self, prior: float, likelihood: float) -> float:
        """Update belief using Bayesian rule"""
        # P(H|D) = P(D|H) * P(H) / P(D)
        # Where P(D) = P(D|H) * P(H) + P(D|¬H) * P(¬H)

        evidence = likelihood * prior + (1 - likelihood) * (1 - prior)
        if evidence == 0:
            return prior  # Avoid division by zero

        posterior = (likelihood * prior) / evidence
        return posterior

    def fuse_multimodal_beliefs(self, beliefs: Dict[str, Dict[str, float]]) -> Dict[str, float]:
        """Fuse beliefs from multiple modalities using Bayesian approach"""
        fused_beliefs = {}

        for belief_type, modality_beliefs in beliefs.items():
            current_belief = self.priors.get(belief_type, 0.5)

            for modality, (likelihood, confidence) in modality_beliefs.items():
                # Weight the likelihood by confidence
                weighted_likelihood = likelihood * confidence + (1 - confidence) * 0.5  # neutral if low confidence
                current_belief = self.bayesian_update(current_belief, weighted_likelihood)

            fused_beliefs[belief_type] = current_belief

        return fused_beliefs

    def compute_uncertainty(self, belief: float) -> float:
        """Compute uncertainty from belief (entropy-based)"""
        if belief == 0 or belief == 1:
            return 0.0
        return -(belief * np.log2(belief) + (1 - belief) * np.log2(1 - belief))
```

## Temporal Consistency and State Tracking

### State Tracking System

Maintaining consistent state across time steps is essential for coherent behavior:

```python
from dataclasses import dataclass, field
from typing import Optional, List, Dict
import time

@dataclass
class RobotState:
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])  # quaternion
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gripper_state: str = 'open'  # 'open', 'closed', 'holding'
    battery_level: float = 1.0
    current_task: Optional[str] = None
    last_action: Optional[str] = None
    timestamp: float = field(default_factory=time.time)

@dataclass
class ObjectState:
    name: str
    position: List[float]
    confidence: float
    tracked: bool = True
    last_seen: float = field(default_factory=time.time)
    properties: Dict[str, Any] = field(default_factory=dict)

class StateTracker:
    def __init__(self):
        self.robot_state = RobotState()
        self.object_states = {}  # object_id -> ObjectState
        self.action_history = []
        self.language_context = []
        self.temporal_buffer_size = 10

    def update_robot_state(self, new_state: RobotState):
        """Update robot state with temporal smoothing"""
        # Apply temporal smoothing to prevent sudden changes
        alpha = 0.1  # smoothing factor

        self.robot_state.position = [
            alpha * new_state.position[i] + (1 - alpha) * self.robot_state.position[i]
            for i in range(3)
        ]

        self.robot_state.orientation = new_state.orientation  # Quaternions need special handling
        self.robot_state.velocity = new_state.velocity
        self.robot_state.gripper_state = new_state.gripper_state
        self.robot_state.battery_level = new_state.battery_level
        self.robot_state.current_task = new_state.current_task
        self.robot_state.last_action = new_state.last_action
        self.robot_state.timestamp = new_state.timestamp

    def update_object_states(self, detected_objects: List[Dict]):
        """Update object states with tracking"""
        current_time = time.time()

        for obj_data in detected_objects:
            obj_id = obj_data.get('name', 'unknown')
            position = obj_data.get('position', [0, 0, 0])
            confidence = obj_data.get('confidence', 0.0)
            properties = obj_data.get('properties', {})

            if obj_id in self.object_states:
                # Update existing object
                existing_obj = self.object_states[obj_id]
                # Apply temporal smoothing to position
                alpha = 0.2
                existing_obj.position = [
                    alpha * position[i] + (1 - alpha) * existing_obj.position[i]
                    for i in range(3)
                ]
                existing_obj.confidence = confidence
                existing_obj.last_seen = current_time
                existing_obj.properties.update(properties)
            else:
                # Add new object
                self.object_states[obj_id] = ObjectState(
                    name=obj_id,
                    position=position,
                    confidence=confidence,
                    properties=properties
                )

        # Mark objects not seen recently as potentially lost
        for obj_id, obj_state in list(self.object_states.items()):
            if current_time - obj_state.last_seen > 5.0:  # 5 seconds
                obj_state.tracked = False
                if current_time - obj_state.last_seen > 30.0:  # 30 seconds
                    del self.object_states[obj_id]  # Remove if not seen for 30 seconds

    def get_spatial_context(self) -> Dict[str, Any]:
        """Get current spatial context for language understanding"""
        spatial_context = {
            'robot_position': self.robot_state.position,
            'visible_objects': [
                {
                    'name': obj.name,
                    'position': obj.position,
                    'confidence': obj.confidence,
                    'properties': obj.properties
                }
                for obj in self.object_states.values()
                if obj.tracked
            ],
            'relationships': self._compute_spatial_relationships()
        }
        return spatial_context

    def _compute_spatial_relationships(self) -> List[Dict]:
        """Compute spatial relationships between objects"""
        relationships = []
        objects = list(self.object_states.values())

        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    pos1 = np.array(obj1.position)
                    pos2 = np.array(obj2.position)
                    distance = np.linalg.norm(pos1 - pos2)

                    # Determine relationship based on distance and relative position
                    if distance < 0.5:  # Objects are close
                        relationships.append({
                            'subject': obj1.name,
                            'predicate': 'near',
                            'object': obj2.name,
                            'distance': distance
                        })

                    # Check if obj1 is left of obj2
                    if pos1[0] < pos2[0] - 0.1:
                        relationships.append({
                            'subject': obj1.name,
                            'predicate': 'left_of',
                            'object': obj2.name
                        })

        return relationships

    def update_language_context(self, command: Dict[str, Any]):
        """Update language context with new command"""
        self.language_context.append({
            'command': command,
            'timestamp': time.time()
        })

        # Keep only recent context
        if len(self.language_context) > self.temporal_buffer_size:
            self.language_context = self.language_context[-self.temporal_buffer_size:]
```

## Grounded Language Understanding

### Visual Grounding Module

Connecting language to visual information for grounded understanding:

```python
class VisualGrounding:
    def __init__(self):
        self.object_reference_resolver = ObjectReferenceResolver()
        self.spatial_grounding = SpatialGrounder()

    def ground_language_command(self, command: Dict[str, Any],
                              visual_context: Dict[str, Any]) -> Dict[str, Any]:
        """Ground language command in visual context"""
        grounded_command = command.copy()

        # Resolve object references using visual context
        if 'entities' in command:
            resolved_entities = self.object_reference_resolver.resolve_references(
                command['entities'], visual_context
            )
            grounded_command['entities'] = resolved_entities

        # Ground spatial references
        if 'spatial_context' in command:
            grounded_spatial = self.spatial_grounding.ground_spatial_refs(
                command['spatial_context'], visual_context
            )
            grounded_command['spatial_context'] = grounded_spatial

        return grounded_command

class ObjectReferenceResolver:
    def __init__(self):
        self.color_map = {
            'red': ['red', 'crimson', 'scarlet'],
            'blue': ['blue', 'navy', 'azure'],
            'green': ['green', 'emerald', 'lime'],
            'yellow': ['yellow', 'gold', 'amber']
        }

    def resolve_references(self, entities: Dict[str, List[str]],
                         visual_context: Dict[str, Any]) -> Dict[str, Any]:
        """Resolve ambiguous object references using visual context"""
        resolved_entities = entities.copy()

        # Resolve color-based references
        if 'color' in entities and 'object' in entities:
            colors = entities['color']
            objects = entities['object']

            # Find objects matching color in visual context
            matching_objects = []
            for obj in visual_context.get('visible_objects', []):
                obj_name = obj.get('name', '').lower()
                obj_props = obj.get('properties', {})

                for color in colors:
                    # Check if object name contains color
                    if color in obj_name:
                        matching_objects.append(obj_name)
                    # Check object properties for color
                    elif obj_props.get('color', '').lower() == color:
                        matching_objects.append(obj_name)

            if matching_objects:
                resolved_entities['target_object'] = matching_objects
                resolved_entities['target_positions'] = [
                    obj['position'] for obj in visual_context['visible_objects']
                    if obj['name'] in matching_objects
                ]

        # Resolve spatial references
        if 'location' in entities:
            locations = entities['location']
            robot_pos = visual_context.get('robot_position', [0, 0, 0])

            # Find objects near specified locations
            for location in locations:
                nearby_objects = self._find_objects_near_location(
                    location, visual_context, robot_pos
                )
                if nearby_objects:
                    resolved_entities[f'near_{location}'] = nearby_objects

        return resolved_entities

    def _find_objects_near_location(self, location: str, visual_context: Dict[str, Any],
                                  robot_pos: List[float]) -> List[str]:
        """Find objects near a specified location"""
        # This would use semantic mapping in a real system
        # For now, use simple spatial reasoning
        location_map = {
            'table': [1.0, 0.0, 0.8],
            'shelf': [1.0, 0.0, 1.2],
            'kitchen': [2.0, 2.0, 0.0],
            'bedroom': [-2.0, 2.0, 0.0]
        }

        if location.lower() in location_map:
            location_pos = location_map[location.lower()]
            nearby_objects = []

            for obj in visual_context.get('visible_objects', []):
                obj_pos = obj.get('position', [0, 0, 0])
                distance = np.linalg.norm(
                    np.array(obj_pos) - np.array(location_pos)
                )

                if distance < 1.0:  # Within 1 meter
                    nearby_objects.append(obj['name'])

            return nearby_objects

        return []

class SpatialGrounder:
    def __init__(self):
        self.spatial_relations = {
            'left_of', 'right_of', 'above', 'below', 'near', 'far_from', 'on', 'in'
        }

    def ground_spatial_refs(self, spatial_context: Dict[str, Any],
                           visual_context: Dict[str, Any]) -> Dict[str, Any]:
        """Ground spatial references in command using visual context"""
        grounded_spatial = spatial_context.copy()

        # Compute spatial relationships from visual context
        relationships = visual_context.get('relationships', [])

        # Add robot-relative relationships
        robot_pos = visual_context.get('robot_position', [0, 0, 0])
        for obj in visual_context.get('visible_objects', []):
            obj_pos = obj.get('position', [0, 0, 0])

            # Relative to robot
            dx = obj_pos[0] - robot_pos[0]
            dy = obj_pos[1] - robot_pos[1]
            dz = obj_pos[2] - robot_pos[2]

            rel_info = {
                'distance': np.sqrt(dx*dx + dy*dy + dz*dz),
                'direction': [dx, dy, dz],
                'relative_position': self._get_relative_direction(dx, dy, dz)
            }

            if obj['name'] in grounded_spatial:
                grounded_spatial[obj['name']].update(rel_info)
            else:
                grounded_spatial[obj['name']] = rel_info

        return grounded_spatial

    def _get_relative_direction(self, dx: float, dy: float, dz: float) -> str:
        """Get relative direction from robot"""
        if abs(dx) > abs(dy) and abs(dx) > abs(dz):
            return 'left' if dx < 0 else 'right'
        elif abs(dy) > abs(dx) and abs(dy) > abs(dz):
            return 'front' if dy > 0 else 'back'
        else:
            return 'above' if dz > 0 else 'below'
```

## Multi-Modal Coordination

### Coordinated Action Planning

Coordinating actions based on multi-modal information:

```python
class CoordinatedActionPlanner:
    def __init__(self):
        self.uncertainty_handler = UncertaintyHandler()
        self.state_tracker = StateTracker()
        self.visual_grounding = VisualGrounding()

    def plan_coordinated_action(self, command: Dict[str, Any],
                              vision_data: Dict[str, Any],
                              language_data: Dict[str, Any]) -> Dict[str, Any]:
        """Plan coordinated action using multi-modal information"""

        # Ground language command in visual context
        grounded_command = self.visual_grounding.ground_language_command(
            language_data, vision_data
        )

        # Update state tracker
        self.state_tracker.update_language_context(grounded_command)

        # Get current spatial context
        spatial_context = self.state_tracker.get_spatial_context()

        # Compute confidences for each modality
        modality_outputs = {
            'vision': vision_data,
            'language': grounded_command,
            'action': {'feasibility': True}  # Placeholder
        }

        confidences = self.uncertainty_handler.propagate_confidence(modality_outputs)
        fused_confidence = self.uncertainty_handler.compute_fused_confidence(confidences)

        # Check if we should proceed
        if not self.uncertainty_handler.should_proceed(fused_confidence, confidences):
            return {
                'action': 'request_clarification',
                'confidence': fused_confidence,
                'reason': 'Low confidence in multi-modal understanding'
            }

        # Generate action plan based on grounded command
        action_plan = self._generate_action_plan(grounded_command, spatial_context)

        return {
            'action_plan': action_plan,
            'confidence': fused_confidence,
            'grounded_command': grounded_command,
            'spatial_context': spatial_context
        }

    def _generate_action_plan(self, grounded_command: Dict[str, Any],
                            spatial_context: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Generate action plan based on grounded command and context"""
        intent = grounded_command.get('intent', 'unknown')
        entities = grounded_command.get('entities', {})

        action_plan = []

        if intent == 'grasp':
            # Find target object position
            target_objects = entities.get('target_object', [])

            if target_objects:
                target_obj_name = target_objects[0]

                # Find object in spatial context
                target_obj = None
                for obj in spatial_context['visible_objects']:
                    if obj['name'] == target_obj_name:
                        target_obj = obj
                        break

                if target_obj:
                    # Add navigation action to go to object
                    action_plan.append({
                        'action_type': 'navigate',
                        'parameters': {
                            'target_position': target_obj['position'],
                            'approach_distance': 0.5
                        }
                    })

                    # Add manipulation action to grasp
                    action_plan.append({
                        'action_type': 'manipulation',
                        'parameters': {
                            'action': 'grasp',
                            'target_object': target_obj_name,
                            'position': target_obj['position']
                        }
                    })

        elif intent == 'navigate':
            # Handle navigation commands
            target_location = entities.get('location', ['default'])[0]

            # Use spatial context to find location coordinates
            location_coords = self._get_location_coordinates(target_location, spatial_context)

            if location_coords:
                action_plan.append({
                    'action_type': 'navigate',
                    'parameters': {
                        'target_position': location_coords
                    }
                })

        elif intent == 'identify':
            # Point to and identify object
            target_objects = entities.get('target_object', [])

            if target_objects:
                target_obj_name = target_objects[0]

                # Find object in spatial context
                target_obj = None
                for obj in spatial_context['visible_objects']:
                    if obj['name'] == target_obj_name:
                        target_obj = obj
                        break

                if target_obj:
                    action_plan.extend([
                        {
                            'action_type': 'gaze_control',
                            'parameters': {
                                'target_position': target_obj['position']
                            }
                        },
                        {
                            'action_type': 'manipulation',
                            'parameters': {
                                'action': 'point',
                                'target_position': target_obj['position']
                            }
                        }
                    ])

        return action_plan

    def _get_location_coordinates(self, location: str, spatial_context: Dict[str, Any]) -> List[float]:
        """Get coordinates for a named location"""
        # In a real system, this would use semantic mapping
        # For now, use predefined locations
        location_map = {
            'kitchen': [2.0, 2.0, 0.0],
            'bedroom': [-2.0, 2.0, 0.0],
            'office': [0.0, -2.0, 0.0],
            'living room': [2.0, -2.0, 0.0],
            'table': [1.0, 0.0, 0.8],  # Standard table height
            'shelf': [1.0, 0.0, 1.2],  # Standard shelf height
            'default': [0.0, 0.0, 0.0]
        }

        return location_map.get(location.lower(), location_map['default'])
```

## Multi-Modal Fusion Node

### Complete ROS 2 Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import json
import threading

class MultiModalFusionNode(Node):
    def __init__(self):
        super().__init__('multi_modal_fusion_node')

        # Publishers and subscribers
        self.fused_command_pub = self.create_publisher(String, 'fused_commands', 10)
        self.vision_sub = self.create_subscription(
            String, 'detected_objects', self.vision_callback, 10
        )
        self.language_sub = self.create_subscription(
            String, 'parsed_commands', self.language_callback, 10
        )
        self.robot_state_sub = self.create_subscription(
            String, 'robot_state', self.robot_state_callback, 10
        )

        # Initialize fusion components
        self.coordinated_planner = CoordinatedActionPlanner()
        self.uncertainty_handler = UncertaintyHandler()
        self.state_tracker = StateTracker()

        # Data buffers
        self.vision_data = None
        self.language_data = None
        self.robot_state_data = None

        # Threading for processing
        self.processing_lock = threading.Lock()
        self.fusion_timer = self.create_timer(0.1, self.fuse_and_publish)

        self.get_logger().info("Multi-Modal Fusion Node initialized")

    def vision_callback(self, msg: String):
        """Process vision data"""
        try:
            vision_data = json.loads(msg.data)
            with self.processing_lock:
                self.vision_data = vision_data
        except Exception as e:
            self.get_logger().error(f"Error processing vision data: {e}")

    def language_callback(self, msg: String):
        """Process language data"""
        try:
            language_data = json.loads(msg.data)
            with self.processing_lock:
                self.language_data = language_data
        except Exception as e:
            self.get_logger().error(f"Error processing language data: {e}")

    def robot_state_callback(self, msg: String):
        """Process robot state data"""
        try:
            robot_state = json.loads(msg.data)
            with self.processing_lock:
                self.robot_state_data = robot_state
        except Exception as e:
            self.get_logger().error(f"Error processing robot state: {e}")

    def fuse_and_publish(self):
        """Fuse modalities and publish result"""
        with self.processing_lock:
            if (self.vision_data is None or
                self.language_data is None or
                self.robot_state_data is None):
                return  # Wait for all data

            try:
                # Update state tracker with robot state
                robot_pos = self.robot_state_data.get('position', [0, 0, 0])
                robot_state_obj = RobotState(
                    position=robot_pos,
                    orientation=self.robot_state_data.get('orientation', [0, 0, 0, 1]),
                    battery_level=self.robot_state_data.get('battery_level', 1.0),
                    gripper_state=self.robot_state_data.get('gripper_state', 'open')
                )
                self.state_tracker.update_robot_state(robot_state_obj)

                # Update object states from vision data
                detected_objects = self.vision_data.get('objects', [])
                self.state_tracker.update_object_states(detected_objects)

                # Plan coordinated action
                fusion_result = self.coordinated_planner.plan_coordinated_action(
                    self.language_data,
                    self.vision_data,
                    self.language_data
                )

                # Publish fused command
                fused_msg = String()
                fused_msg.data = json.dumps(fusion_result)
                self.fused_command_pub.publish(fused_msg)

                self.get_logger().info(f"Fused command published with confidence: {fusion_result.get('confidence', 0):.2f}")

            except Exception as e:
                self.get_logger().error(f"Error in fusion processing: {e}")

    def get_current_context(self) -> Dict[str, Any]:
        """Get current multi-modal context"""
        with self.processing_lock:
            return {
                'vision': self.vision_data,
                'language': self.language_data,
                'robot_state': self.robot_state_data,
                'spatial_context': self.state_tracker.get_spatial_context()
            }

def main(args=None):
    rclpy.init(args=args)

    node = MultiModalFusionNode()

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

## Advanced Fusion Techniques

### Memory-Augmented Fusion

Incorporating memory for long-term reasoning:

```python
class MemoryAugmentedFusion:
    def __init__(self, memory_size: int = 100):
        self.episodic_memory = []  # Short-term memory
        self.semantic_memory = {}  # Long-term knowledge
        self.memory_size = memory_size

    def store_episode(self, state: Dict[str, Any], action: Dict[str, Any],
                     outcome: Dict[str, Any], confidence: float):
        """Store episode in episodic memory"""
        episode = {
            'state': state,
            'action': action,
            'outcome': outcome,
            'confidence': confidence,
            'timestamp': time.time()
        }

        self.episodic_memory.append(episode)

        # Keep memory size bounded
        if len(self.episodic_memory) > self.memory_size:
            self.episodic_memory.pop(0)

    def retrieve_similar_episodes(self, current_state: Dict[str, Any],
                               threshold: float = 0.7) -> List[Dict[str, Any]]:
        """Retrieve similar past episodes"""
        similar_episodes = []

        for episode in self.episodic_memory:
            similarity = self._compute_state_similarity(
                current_state, episode['state']
            )

            if similarity > threshold:
                similar_episodes.append({
                    'episode': episode,
                    'similarity': similarity
                })

        # Sort by similarity
        similar_episodes.sort(key=lambda x: x['similarity'], reverse=True)
        return similar_episodes

    def _compute_state_similarity(self, state1: Dict[str, Any],
                                state2: Dict[str, Any]) -> float:
        """Compute similarity between two states"""
        # Simple similarity based on object overlap
        objects1 = {obj['name'] for obj in state1.get('visible_objects', [])}
        objects2 = {obj['name'] for obj in state2.get('visible_objects', [])}

        if not objects1 and not objects2:
            return 1.0
        if not objects1 or not objects2:
            return 0.0

        intersection = objects1.intersection(objects2)
        union = objects1.union(objects2)

        return len(intersection) / len(union)  # Jaccard similarity

    def augment_fusion_with_memory(self, current_fusion: Dict[str, Any],
                                 current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Augment current fusion with memory-based reasoning"""
        # Retrieve similar past episodes
        similar_episodes = self.retrieve_similar_episodes(current_state)

        if similar_episodes:
            # Use most similar episode as reference
            best_episode = similar_episodes[0]['episode']

            # Enhance current fusion with past experience
            enhanced_fusion = current_fusion.copy()
            enhanced_fusion['past_experience'] = {
                'similar_episode': best_episode,
                'expected_outcome': best_episode['outcome'],
                'confidence_boost': similar_episodes[0]['similarity']
            }

            # Adjust confidence based on past success
            past_success = best_episode['outcome'].get('success', False)
            if past_success:
                enhanced_fusion['confidence'] = min(
                    1.0,
                    current_fusion.get('confidence', 0.5) +
                    similar_episodes[0]['similarity'] * 0.2
                )

            return enhanced_fusion

        return current_fusion
```

## Testing and Validation

### Unit Tests

```python
import unittest
from unittest.mock import Mock, patch

class TestMultiModalFusion(unittest.TestCase):
    def setUp(self):
        self.uncertainty_handler = UncertaintyHandler()
        self.state_tracker = StateTracker()
        self.visual_grounding = VisualGrounding()
        self.coordinated_planner = CoordinatedActionPlanner()

    def test_cross_modal_attention(self):
        """Test cross-modal attention mechanism"""
        # Create dummy features
        vision_features = torch.randn(1, 10, 512)  # [batch, regions, dim]
        language_features = torch.randn(1, 20, 512)  # [batch, seq_len, dim]
        action_features = torch.randn(1, 512)  # [batch, dim]

        attention_module = CrossModalAttention(512, 512, 256)
        attended_vision, attended_language = attention_module(
            vision_features, language_features
        )

        self.assertEqual(attended_vision.shape, (1, 10, 256))
        self.assertEqual(attended_language.shape, (1, 20, 256))

    def test_confidence_propagation(self):
        """Test confidence propagation across modalities"""
        modality_outputs = {
            'vision': [{'confidence': 0.8, 'name': 'object1'}],
            'language': {'confidence': 0.7, 'intent': 'grasp'},
            'action': {'feasibility': True}
        }

        confidences = self.uncertainty_handler.propagate_confidence(modality_outputs)

        self.assertIn('vision', confidences)
        self.assertIn('language', confidences)
        self.assertGreaterEqual(confidences['vision'], 0.7)
        self.assertGreaterEqual(confidences['language'], 0.6)

    def test_state_tracking(self):
        """Test state tracking functionality"""
        # Update robot state
        new_state = RobotState(
            position=[1.0, 2.0, 0.0],
            gripper_state='closed',
            battery_level=0.8
        )
        self.state_tracker.update_robot_state(new_state)

        # Check that state was updated
        self.assertEqual(self.state_tracker.robot_state.position, [1.0, 2.0, 0.0])
        self.assertEqual(self.state_tracker.robot_state.gripper_state, 'closed')

        # Update object states
        detected_objects = [
            {'name': 'red_ball', 'position': [1.5, 2.5, 0.5], 'confidence': 0.9}
        ]
        self.state_tracker.update_object_states(detected_objects)

        # Check that object was tracked
        self.assertIn('red_ball', self.state_tracker.object_states)
        tracked_obj = self.state_tracker.object_states['red_ball']
        self.assertEqual(tracked_obj.name, 'red_ball')
        self.assertGreaterEqual(tracked_obj.confidence, 0.8)

    def test_object_reference_resolution(self):
        """Test object reference resolution"""
        entities = {
            'color': ['red'],
            'object': ['ball']
        }

        visual_context = {
            'visible_objects': [
                {'name': 'red_ball', 'position': [1.0, 2.0, 0.5], 'properties': {'color': 'red'}},
                {'name': 'blue_cup', 'position': [2.0, 1.0, 0.6], 'properties': {'color': 'blue'}}
            ]
        }

        resolver = ObjectReferenceResolver()
        resolved_entities = resolver.resolve_references(entities, visual_context)

        self.assertIn('target_object', resolved_entities)
        self.assertIn('red_ball', resolved_entities['target_object'])

    def test_action_planning_with_context(self):
        """Test coordinated action planning"""
        command = {
            'intent': 'grasp',
            'entities': {'object': ['ball'], 'color': ['red']}
        }

        vision_data = {
            'objects': [
                {'name': 'red_ball', 'position': [1.0, 2.0, 0.5], 'confidence': 0.9}
            ],
            'relationships': []
        }

        language_data = command

        result = self.coordinated_planner.plan_coordinated_action(
            command, vision_data, language_data
        )

        self.assertIn('action_plan', result)
        self.assertGreater(len(result['action_plan']), 0)
        self.assertGreaterEqual(result.get('confidence', 0), 0.5)

    def test_memory_augmented_fusion(self):
        """Test memory-augmented fusion"""
        memory_fusion = MemoryAugmentedFusion()

        # Store a successful episode
        state = {'visible_objects': [{'name': 'red_ball'}]}
        action = {'type': 'grasp', 'target': 'red_ball'}
        outcome = {'success': True}

        memory_fusion.store_episode(state, action, outcome, 0.9)

        # Try to retrieve similar episode
        current_state = {'visible_objects': [{'name': 'red_ball'}]}
        similar_episodes = memory_fusion.retrieve_similar_episodes(current_state)

        self.assertGreater(len(similar_episodes), 0)
        self.assertTrue(similar_episodes[0]['similarity'] > 0.5)

if __name__ == '__main__':
    unittest.main()
```

## Configuration and Setup

### Configuration File

```yaml
# config/multi_modal_fusion.yaml
multi_modal_fusion:
  attention:
    hidden_dim: 512
    fusion_type: "cross_attention"
    dropout: 0.1

  uncertainty:
    confidence_thresholds:
      vision: 0.7
      language: 0.6
      action: 0.8
    propagation_method: "bayesian"

  temporal:
    state_buffer_size: 10
    object_tracking_timeout: 30.0
    smoothing_factor: 0.1

  memory:
    episodic_memory_size: 100
    similarity_threshold: 0.7
    memory_augmentation: true

  fusion:
    enable_grounding: true
    enable_coordination: true
    enable_temporal_consistency: true
    max_fusion_delay: 0.5
```

### Launch File

```xml
<!-- launch/multi_modal_fusion.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="multi_modal_fusion_node" name="multi_modal_fusion">
    <param name="fusion_rate" value="10"/>
    <param name="enable_memory_augmentation" value="true"/>
  </node>
</launch>
```

## Troubleshooting

### Common Issues

1. **Fusion Inconsistencies**
   - Solution: Implement proper temporal buffering and state synchronization
   - Check: Ensure all modalities are properly timestamped and synchronized

2. **Confidence Miscalibration**
   - Solution: Calibrate confidence thresholds based on empirical data
   - Check: Monitor confidence scores and adjust thresholds accordingly

3. **Memory Performance**
   - Solution: Optimize memory retrieval algorithms and limit memory size
   - Check: Monitor memory usage and processing time

4. **Grounding Failures**
   - Solution: Improve object detection and spatial reasoning
   - Check: Verify visual and spatial data quality

## Next Steps

In the next section, we'll integrate all components into a complete system and focus on testing, validation, and deployment considerations. The multi-modal fusion system we've built provides the foundation for coherent Vision-Language-Action behavior.

Continue to [Integration and Testing](../integration/index.md).