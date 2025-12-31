# Exercise 4.1: Action Planning and Representation

## Objective
Implement action planning and representation systems for the Vision-Language-Action system, enabling the robot to translate high-level commands into executable action sequences.

## Prerequisites
- Python 3.10+
- ROS 2 Humble Hawksbill installed
- Understanding of robot kinematics and dynamics
- Basic knowledge of motion planning concepts
- Completion of previous phases (Vision and Language)

## Exercise Steps

### Step 1: Set Up Action Planning Environment
Create a new file `action_planner.py`:

```python
#!/usr/bin/env python3
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import math
import json

class ActionType(Enum):
    """Enumeration of possible action types"""
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    GAZE_CONTROL = "gaze_control"
    SPEECH = "speech"
    INTERACTION = "interaction"

@dataclass
class Action:
    """Base action class"""
    action_type: ActionType
    parameters: Dict[str, Any]
    priority: int = 0
    timeout: float = 30.0  # seconds
    preconditions: List[str] = None
    effects: List[str] = None
    cost: float = 1.0

    def __post_init__(self):
        if self.preconditions is None:
            self.preconditions = []
        if self.effects is None:
            self.effects = []

@dataclass
class NavigationAction(Action):
    """Navigation action with path planning"""
    def __post_init__(self):
        self.action_type = ActionType.NAVIGATION
        if self.preconditions is None:
            self.preconditions = ['robot_is_operational', 'navigation_enabled']
        if self.effects is None:
            self.effects = ['robot_at_target']

@dataclass
class ManipulationAction(Action):
    """Manipulation action for robotic arms"""
    def __post_init__(self):
        self.action_type = ActionType.MANIPULATION
        if self.preconditions is None:
            self.preconditions = ['arm_free', 'reachable', 'object_detected']
        if self.effects is None:
            self.effects = ['manipulation_completed']

@dataclass
class GazeAction(Action):
    """Gaze control action for head/eye movements"""
    def __post_init__(self):
        self.action_type = ActionType.GAZE_CONTROL
        if self.preconditions is None:
            self.preconditions = ['head_operational']
        if self.effects is None:
            self.effects = ['gaze_at_target']

class ActionSpace:
    """Defines the action space for the robot"""
    def __init__(self):
        self.action_templates = self._initialize_action_templates()
        self.robot_capabilities = self._initialize_robot_capabilities()

    def _initialize_action_templates(self) -> Dict[str, Action]:
        """Initialize available action templates"""
        return {
            'move_to': NavigationAction(
                parameters={
                    'target_position': None,
                    'speed': 0.5,
                    'avoid_obstacles': True,
                    'orientation': None
                },
                preconditions=['robot_is_operational', 'navigation_enabled'],
                effects=['robot_at_target'],
                cost=1.0
            ),
            'grasp_object': ManipulationAction(
                parameters={
                    'object_id': None,
                    'object_position': None,
                    'gripper_force': 50,
                    'approach_direction': [0, 0, 1]  # default: approach from above
                },
                preconditions=['arm_free', 'reachable', 'object_detected'],
                effects=['object_grasped'],
                cost=2.0
            ),
            'place_object': ManipulationAction(
                parameters={
                    'target_position': None,
                    'orientation': None,
                    'release_force': 0
                },
                preconditions=['object_grasped'],
                effects=['object_placed', 'gripper_open'],
                cost=2.0
            ),
            'point_to': ManipulationAction(
                parameters={
                    'target_position': None,
                    'duration': 2.0,
                    'motion_type': 'pointing'
                },
                preconditions=['arm_free'],
                effects=['pointing_at_target'],
                cost=1.5
            ),
            'look_at': GazeAction(
                parameters={
                    'target_position': None,
                    'duration': 1.0
                },
                preconditions=['head_operational'],
                effects=['looking_at_target'],
                cost=0.5
            ),
            'speak': Action(
                action_type=ActionType.SPEECH,
                parameters={
                    'text': None,
                    'voice_type': 'default',
                    'volume': 0.7
                },
                preconditions=['speech_system_operational'],
                effects=['speech_delivered'],
                cost=0.3
            )
        }

    def _initialize_robot_capabilities(self) -> Dict[str, Any]:
        """Initialize robot capabilities"""
        return {
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'manipulator_dof': 6,
            'gripper_range': [0, 100],  # 0-100% force
            'workspace_bounds': {
                'x': [-1.0, 1.0],
                'y': [-1.0, 1.0],
                'z': [0.0, 1.5]
            },
            'navigation_enabled': True,
            'manipulation_enabled': True
        }

    def get_action_template(self, action_name: str) -> Optional[Action]:
        """Get action template by name"""
        return self.action_templates.get(action_name)

    def validate_action(self, action: Action) -> Tuple[bool, List[str]]:
        """Validate action against robot capabilities"""
        errors = []

        # Check if action type is supported
        if not self.robot_capabilities.get(f"{action.action_type.value}_enabled", True):
            errors.append(f"{action.action_type.value} is not enabled on this robot")

        # Validate specific action parameters
        if action.action_type == ActionType.NAVIGATION:
            target_pos = action.parameters.get('target_position')
            if target_pos is None:
                errors.append("Navigation action requires target_position")
            elif len(target_pos) < 2:
                errors.append("Navigation target_position must have at least x and y coordinates")

        elif action.action_type == ActionType.MANIPULATION:
            # Check workspace bounds for manipulation
            target_pos = action.parameters.get('target_position') or action.parameters.get('object_position')
            if target_pos and len(target_pos) >= 3:
                bounds = self.robot_capabilities['workspace_bounds']
                if (not bounds['x'][0] <= target_pos[0] <= bounds['x'][1] or
                    not bounds['y'][0] <= target_pos[1] <= bounds['y'][1] or
                    not bounds['z'][0] <= target_pos[2] <= bounds['z'][1]):
                    errors.append(f"Target position {target_pos} is outside workspace bounds")

        return len(errors) == 0, errors

class ActionPlanner:
    """Generates sequences of primitive actions to achieve high-level goals"""
    def __init__(self):
        self.action_space = ActionSpace()
        self.robot_state = self._initialize_robot_state()
        self.world_model = {}  # Will be updated with perception data

    def _initialize_robot_state(self) -> Dict[str, Any]:
        """Initialize robot state"""
        return {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],  # quaternion
            'arm_state': 'free',  # free, holding_object, busy
            'gripper_state': 'open',  # open, closed
            'battery_level': 100.0,
            'operational': True
        }

    def plan_action(self, high_level_command: Dict[str, Any]) -> List[Action]:
        """
        Plan primitive actions from high-level command

        Args:
            high_level_command: Dictionary containing intent and entities

        Returns:
            List of primitive actions to execute
        """
        intent = high_level_command.get('intent', '')
        entities = high_level_command.get('entities', {})
        original_text = high_level_command.get('original_text', '')

        actions = []

        if intent == 'navigate':
            actions.extend(self._create_navigation_sequence(entities))
        elif intent == 'grasp':
            actions.extend(self._create_grasp_sequence(entities))
        elif intent == 'place':
            actions.extend(self._create_place_sequence(entities))
        elif intent == 'move':
            actions.extend(self._create_move_sequence(entities))
        elif intent == 'inspect':
            actions.extend(self._create_inspection_sequence(entities))
        elif intent == 'bring':
            actions.extend(self._create_bring_sequence(entities))
        else:
            # Default: try to understand the command and create appropriate actions
            actions.extend(self._infer_actions_from_command(original_text, entities))

        return actions

    def _create_navigation_sequence(self, entities: Dict[str, Any]) -> List[Action]:
        """Create navigation sequence based on entities"""
        actions = []

        target_location = entities.get('location', ['default'])[0] if entities.get('location') else 'default'
        target_location = target_location.lower()

        # Map semantic locations to coordinates
        location_map = {
            'kitchen': [2.0, 2.0, 0.0],
            'bedroom': [-2.0, 2.0, 0.0],
            'office': [0.0, -2.0, 0.0],
            'living room': [2.0, -2.0, 0.0],
            'dining room': [-1.0, -1.0, 0.0],
            'default': [0.0, 0.0, 0.0]
        }

        target_position = location_map.get(target_location, location_map['default'])

        # Create navigation action
        nav_action = NavigationAction(
            parameters={
                'target_position': target_position,
                'speed': 0.5,
                'avoid_obstacles': True
            },
            priority=1
        )

        actions.append(nav_action)
        return actions

    def _create_grasp_sequence(self, entities: Dict[str, Any]) -> List[Action]:
        """Create grasp sequence based on entities"""
        actions = []

        # Get target object and position
        target_object = entities.get('object', entities.get('target_object', ['default']))[0]
        object_position = entities.get('object_position')

        # If position is not provided, we need to find it
        if not object_position:
            # In a real system, this would query the world model
            # For this exercise, we'll use a default position
            object_position = [1.0, 0.5, 0.1]

        # Move to object if not already close
        current_pos = self.robot_state['position']
        distance = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(current_pos[:2], object_position[:2]))
        )

        if distance > 0.5:  # If more than 0.5m away
            # Navigate close to object
            approach_pos = [
                object_position[0] - 0.3,  # Approach from front
                object_position[1],
                0.0  # Keep z at ground level for navigation
            ]

            nav_action = NavigationAction(
                parameters={
                    'target_position': approach_pos,
                    'speed': 0.3,
                    'avoid_obstacles': True
                },
                priority=2
            )
            actions.append(nav_action)

        # Manipulation to grasp the object
        grasp_action = ManipulationAction(
            parameters={
                'object_id': target_object,
                'object_position': object_position,
                'gripper_force': 40,
                'approach_direction': [0, 0, 1]  # Approach from above
            },
            priority=3
        )
        actions.append(grasp_action)

        return actions

    def _create_place_sequence(self, entities: Dict[str, Any]) -> List[Action]:
        """Create place sequence based on entities"""
        actions = []

        target_location = entities.get('location', ['table'])[0]
        location_positions = {
            'table': [1.0, 0.0, 0.8],  # Standard table height
            'shelf': [1.0, 0.0, 1.2],  # Standard shelf height
            'counter': [0.5, 0.0, 0.9],
            'floor': [0.0, 0.0, 0.1],
            'default': [0.0, 0.0, 0.8]
        }

        target_position = location_positions.get(target_location.lower(), location_positions['default'])

        place_action = ManipulationAction(
            parameters={
                'target_position': target_position,
                'release_force': 0,
                'orientation': [0, 0, 0, 1]  # Default orientation
            },
            priority=2
        )

        actions.append(place_action)
        return actions

    def _create_move_sequence(self, entities: Dict[str, Any]) -> List[Action]:
        """Create move sequence based on entities"""
        actions = []

        # Handle movement commands like "move forward", "go left", etc.
        direction = entities.get('direction', ['forward'])[0]
        distance = float(entities.get('distance', [1.0])[0]) if entities.get('distance') else 1.0

        # Get current position
        current_pos = self.robot_state['position']

        # Calculate target position based on direction
        direction_map = {
            'forward': [distance, 0, 0],
            'backward': [-distance, 0, 0],
            'left': [0, distance, 0],
            'right': [0, -distance, 0],
            'up': [0, 0, distance],
            'down': [0, 0, -distance]
        }

        offset = direction_map.get(direction.lower(), [distance, 0, 0])
        target_pos = [
            current_pos[0] + offset[0],
            current_pos[1] + offset[1],
            current_pos[2] + offset[2]
        ]

        nav_action = NavigationAction(
            parameters={
                'target_position': target_pos,
                'speed': 0.3,
                'avoid_obstacles': True
            },
            priority=1
        )

        actions.append(nav_action)
        return actions

    def _create_inspection_sequence(self, entities: Dict[str, Any]) -> List[Action]:
        """Create inspection sequence based on entities"""
        actions = []

        target_object = entities.get('object', ['default'])[0]

        # Look at the object
        gaze_action = GazeAction(
            parameters={
                'target_object': target_object,
                'duration': 2.0
            },
            priority=1
        )
        actions.append(gaze_action)

        # Point to the object (if manipulation is available)
        if self.action_space.robot_capabilities['manipulation_enabled']:
            point_action = ManipulationAction(
                parameters={
                    'target_object': target_object,
                    'motion_type': 'pointing',
                    'duration': 2.0
                },
                priority=1
            )
            actions.append(point_action)

        return actions

    def _create_bring_sequence(self, entities: Dict[str, Any]) -> List[Action]:
        """Create bring sequence based on entities"""
        actions = []

        # This involves going to get an object and bringing it to a location
        target_object = entities.get('object', ['default'])[0]
        destination = entities.get('location', ['default'])[0]

        # First, go to where the object is (in a real system, this would be from world model)
        # For this exercise, assume object is at [1, 0, 0]
        object_pos = [1.0, 0.0, 0.0]

        # Navigate to object
        nav_to_object = NavigationAction(
            parameters={
                'target_position': [object_pos[0] - 0.3, object_pos[1], 0.0],  # Approach
                'speed': 0.3,
                'avoid_obstacles': True
            },
            priority=2
        )
        actions.append(nav_to_object)

        # Grasp the object
        grasp_action = ManipulationAction(
            parameters={
                'object_id': target_object,
                'object_position': object_pos,
                'gripper_force': 40
            },
            priority=3
        )
        actions.append(grasp_action)

        # Navigate to destination
        location_map = {
            'kitchen': [2.0, 2.0, 0.0],
            'bedroom': [-2.0, 2.0, 0.0],
            'office': [0.0, -2.0, 0.0],
            'living room': [2.0, -2.0, 0.0],
            'default': [0.0, 0.0, 0.0]
        }

        dest_pos = location_map.get(destination.lower(), location_map['default'])

        nav_to_dest = NavigationAction(
            parameters={
                'target_position': dest_pos,
                'speed': 0.3,
                'avoid_obstacles': True
            },
            priority=2
        )
        actions.append(nav_to_dest)

        # Place the object
        place_action = ManipulationAction(
            parameters={
                'target_position': [dest_pos[0], dest_pos[1], 0.8],  # Place on surface
                'release_force': 0
            },
            priority=3
        )
        actions.append(place_action)

        return actions

    def _infer_actions_from_command(self, command_text: str, entities: Dict[str, Any]) -> List[Action]:
        """Infer actions from command text when intent is unclear"""
        actions = []

        # Simple keyword-based inference
        text_lower = command_text.lower()

        if any(word in text_lower for word in ['go', 'move', 'navigate', 'walk', 'drive']):
            # Assume navigation intent
            location = entities.get('location', ['default'])[0] if entities.get('location') else 'default'
            location_map = {'kitchen': [2.0, 2.0, 0.0], 'bedroom': [-2.0, 2.0, 0.0], 'default': [0.0, 0.0, 0.0]}
            target_pos = location_map.get(location.lower(), location_map['default'])

            actions.append(NavigationAction(
                parameters={'target_position': target_pos, 'speed': 0.5},
                priority=1
            ))

        elif any(word in text_lower for word in ['grasp', 'pick', 'grab', 'take', 'hold']):
            # Assume grasp intent
            obj_pos = entities.get('object_position', [1.0, 0.5, 0.1])
            actions.append(ManipulationAction(
                parameters={'object_position': obj_pos, 'gripper_force': 40},
                priority=2
            ))

        return actions

    def validate_action_sequence(self, actions: List[Action]) -> Tuple[bool, List[str]]:
        """Validate sequence of actions for consistency and feasibility"""
        errors = []

        for i, action in enumerate(actions):
            is_valid, action_errors = self.action_space.validate_action(action)
            if not is_valid:
                errors.extend([f"Action {i}: {error}" for error in action_errors])

        return len(errors) == 0, errors

    def optimize_action_sequence(self, actions: List[Action]) -> List[Action]:
        """Optimize action sequence for efficiency"""
        if not actions:
            return actions

        # Simple optimization: merge consecutive navigation actions
        optimized_actions = []
        i = 0

        while i < len(actions):
            current_action = actions[i]

            # If current action is navigation and next is also navigation, merge them
            if (current_action.action_type == ActionType.NAVIGATION and
                i + 1 < len(actions) and
                actions[i + 1].action_type == ActionType.NAVIGATION):

                # For simplicity, just keep the last navigation action in the sequence
                j = i
                while j + 1 < len(actions) and actions[j + 1].action_type == ActionType.NAVIGATION:
                    j += 1

                # Keep the final navigation target
                optimized_actions.append(actions[j])
                i = j + 1
            else:
                optimized_actions.append(current_action)
                i += 1

        return optimized_actions

def main():
    """Test the action planner"""
    print("Testing Action Planner...")

    planner = ActionPlanner()

    # Test different types of commands
    test_commands = [
        {
            'intent': 'navigate',
            'entities': {'location': ['kitchen']},
            'original_text': 'go to kitchen'
        },
        {
            'intent': 'grasp',
            'entities': {'object': ['red ball'], 'object_position': [1.0, 0.5, 0.1]},
            'original_text': 'grasp the red ball'
        },
        {
            'intent': 'place',
            'entities': {'location': ['table']},
            'original_text': 'place object on table'
        },
        {
            'intent': 'move',
            'entities': {'direction': ['forward'], 'distance': ['2']},
            'original_text': 'move forward 2 meters'
        },
        {
            'intent': 'bring',
            'entities': {'object': ['cup'], 'location': ['kitchen']},
            'original_text': 'bring the cup to kitchen'
        }
    ]

    for i, command in enumerate(test_commands):
        print(f"\n--- Test Command {i+1}: {command['original_text']} ---")

        actions = planner.plan_action(command)

        print(f"Planned {len(actions)} actions:")
        for j, action in enumerate(actions):
            print(f"  {j+1}. {action.action_type.value}: {action.parameters}")

        # Validate the action sequence
        is_valid, errors = planner.validate_action_sequence(actions)
        if not is_valid:
            print(f"Validation errors: {errors}")
        else:
            print("Action sequence is valid")

        # Optimize the sequence
        optimized_actions = planner.optimize_action_sequence(actions)
        print(f"Optimized to {len(optimized_actions)} actions")

if __name__ == "__main__":
    main()