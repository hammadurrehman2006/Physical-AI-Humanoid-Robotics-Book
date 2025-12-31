"""
Action Executor Service

This module implements the action execution service with ROS 2 integration
for executing physical actions through robotics action servers.
"""

import asyncio
import logging
from typing import Dict, Any, Optional, List, Tuple
from enum import Enum
import time
import uuid

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action_client import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import Pose, Point
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
except ImportError:
    rclpy = None
    Node = None
    ActionClient = None
    MultiThreadedExecutor = None
    Pose = None
    Point = None
    String = None
    JointState = None

from ..models.action_plan import ActionPlan, Action, ActionStatus, ActionType
from ..models.voice_command import VoiceCommand


class ActionExecutionStatus(Enum):
    """Enumeration of action execution statuses"""
    SUCCESS = "success"
    FAILED = "failed"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    IN_PROGRESS = "in_progress"


class ActionExecutor:
    """
    Action execution service with ROS 2 integration for executing physical actions
    through robotics action servers.
    """

    def __init__(self, ros_node: Optional['Node'] = None):
        """
        Initialize the action executor service.

        Args:
            ros_node: Optional ROS 2 node. If not provided, a new one will be created.
        """
        self.logger = logging.getLogger(__name__)
        self.ros_node = ros_node

        # If no ROS node is provided, initialize ROS
        if rclpy is not None and self.ros_node is None:
            rclpy.init()
            self.ros_node = ActionExecutorROSNode("action_executor_node")
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ros_node)
            self.logger.info("Initialized ROS 2 action executor node")

        # Store active action plans
        self.active_plans = {}
        self.action_results = {}

    def execute_action_plan(self, action_plan: ActionPlan) -> ActionPlan:
        """
        Execute an action plan synchronously.

        Args:
            action_plan: The action plan to execute

        Returns:
            Updated action plan with execution results
        """
        self.logger.info(f"Executing action plan: {action_plan.id}")
        action_plan.update_status(ActionStatus.EXECUTING)

        try:
            # Execute each action in the plan
            for action in action_plan.actions:
                self.logger.info(f"Executing action: {action.action_id} of type {action.type.value}")

                # Execute the action and get result
                result = self._execute_single_action(action, action_plan)
                action_plan.log_execution(action.action_id, result)

                # Check if action was successful
                if result != "success":
                    self.logger.warning(f"Action {action.action_id} failed with result: {result}")
                    action_plan.update_status(ActionStatus.FAILED)
                    break

            # If all actions completed successfully
            else:
                action_plan.update_status(ActionStatus.COMPLETED)
                self.logger.info(f"Action plan {action_plan.id} completed successfully")

        except Exception as e:
            self.logger.error(f"Error executing action plan {action_plan.id}: {str(e)}")
            action_plan.update_status(ActionStatus.FAILED)

        return action_plan

    async def execute_action_plan_async(self, action_plan: ActionPlan) -> ActionPlan:
        """
        Execute an action plan asynchronously.

        Args:
            action_plan: The action plan to execute

        Returns:
            Updated action plan with execution results
        """
        return self.execute_action_plan(action_plan)

    def _execute_single_action(self, action: Action, action_plan: ActionPlan) -> str:
        """
        Execute a single action based on its type.

        Args:
            action: The action to execute
            action_plan: The parent action plan

        Returns:
            Execution result string
        """
        try:
            # Check preconditions
            if not self._check_preconditions(action.preconditions):
                self.logger.warning(f"Preconditions not met for action {action.action_id}")
                return "preconditions_failed"

            # Execute based on action type
            if action.type == ActionType.NAVIGATION:
                result = self._execute_navigation_action(action)
            elif action.type == ActionType.MANIPULATION:
                result = self._execute_manipulation_action(action)
            elif action.type == ActionType.INTERACTION:
                result = self._execute_interaction_action(action)
            elif action.type == ActionType.GESTURE:
                result = self._execute_gesture_action(action)
            elif action.type == ActionType.SPEAK:
                result = self._execute_speak_action(action)
            elif action.type == ActionType.SENSING:
                result = self._execute_sensing_action(action)
            else:
                self.logger.error(f"Unknown action type: {action.type}")
                return "unknown_action_type"

            # Check postconditions
            if result == "success":
                if not self._check_postconditions(action.postconditions):
                    self.logger.warning(f"Postconditions not met for action {action.action_id}")
                    return "postconditions_failed"

            return result

        except Exception as e:
            self.logger.error(f"Error executing action {action.action_id}: {str(e)}")
            return "execution_error"

    def _check_preconditions(self, preconditions: List[str]) -> bool:
        """
        Check if all preconditions are met.

        Args:
            preconditions: List of precondition strings

        Returns:
            True if all preconditions are met, False otherwise
        """
        # In a real implementation, this would check robot state, sensor data, etc.
        # For now, we'll return True for all preconditions
        for precondition in preconditions:
            self.logger.debug(f"Checking precondition: {precondition}")
            # Here you would implement actual checks
            # For example: checking if robot is idle, if required resources are available, etc.

        return True

    def _check_postconditions(self, postconditions: List[str]) -> bool:
        """
        Check if all postconditions are met.

        Args:
            postconditions: List of postcondition strings

        Returns:
            True if all postconditions are met, False otherwise
        """
        # In a real implementation, this would check robot state after action execution
        # For now, we'll return True for all postconditions
        for postcondition in postconditions:
            self.logger.debug(f"Checking postcondition: {postcondition}")
            # Here you would implement actual checks
            # For example: checking if robot is at target location, if object is grasped, etc.

        return True

    def _execute_navigation_action(self, action: Action) -> str:
        """
        Execute a navigation action.

        Args:
            action: The navigation action to execute

        Returns:
            Execution result string
        """
        try:
            target_location = action.parameters.get("target_location")
            x = action.parameters.get("x", 0.0)
            y = action.parameters.get("y", 0.0)
            z = action.parameters.get("z", 0.0)

            self.logger.info(f"Navigating to location: {target_location} at ({x}, {y}, {z})")

            # In a real implementation, this would call navigation ROS services/actions
            # For now, we'll simulate navigation
            if rclpy is not None and self.ros_node is not None:
                # Call ROS navigation service
                nav_result = self.ros_node.navigate_to_pose(x, y, z)
                return "success" if nav_result else "navigation_failed"
            else:
                # Simulate navigation
                time.sleep(2)  # Simulate navigation time
                return "success"

        except Exception as e:
            self.logger.error(f"Navigation action failed: {str(e)}")
            return "navigation_failed"

    def _execute_manipulation_action(self, action: Action) -> str:
        """
        Execute a manipulation action.

        Args:
            action: The manipulation action to execute

        Returns:
            Execution result string
        """
        try:
            object_id = action.parameters.get("object_id")
            action_type = action.parameters.get("action", "grasp")
            object_pose = action.parameters.get("object_pose", {})

            self.logger.info(f"Manipulating object {object_id} with action {action_type}")

            # In a real implementation, this would call manipulation ROS services/actions
            # For now, we'll simulate manipulation
            if rclpy is not None and self.ros_node is not None:
                # Call ROS manipulation service
                manip_result = self.ros_node.manipulate_object(object_id, action_type, object_pose)
                return "success" if manip_result else "manipulation_failed"
            else:
                # Simulate manipulation
                time.sleep(3)  # Simulate manipulation time
                return "success"

        except Exception as e:
            self.logger.error(f"Manipulation action failed: {str(e)}")
            return "manipulation_failed"

    def _execute_interaction_action(self, action: Action) -> str:
        """
        Execute an interaction action.

        Args:
            action: The interaction action to execute

        Returns:
            Execution result string
        """
        try:
            interaction_type = action.parameters.get("interaction_type", "touch")
            target = action.parameters.get("target")

            self.logger.info(f"Performing interaction {interaction_type} with {target}")

            # In a real implementation, this would call interaction ROS services/actions
            # For now, we'll simulate interaction
            time.sleep(1)  # Simulate interaction time
            return "success"

        except Exception as e:
            self.logger.error(f"Interaction action failed: {str(e)}")
            return "interaction_failed"

    def _execute_gesture_action(self, action: Action) -> str:
        """
        Execute a gesture action.

        Args:
            action: The gesture action to execute

        Returns:
            Execution result string
        """
        try:
            gesture_type = action.parameters.get("gesture_type", "point")
            target = action.parameters.get("target", {})

            self.logger.info(f"Performing gesture {gesture_type} toward {target}")

            # In a real implementation, this would call gesture ROS services/actions
            # For now, we'll simulate gesture
            time.sleep(1)  # Simulate gesture time
            return "success"

        except Exception as e:
            self.logger.error(f"Gesture action failed: {str(e)}")
            return "gesture_failed"

    def _execute_speak_action(self, action: Action) -> str:
        """
        Execute a speak action.

        Args:
            action: The speak action to execute

        Returns:
            Execution result string
        """
        try:
            text = action.parameters.get("text", "")
            voice = action.parameters.get("voice", "default")

            self.logger.info(f"Speaking: '{text}' with voice {voice}")

            # In a real implementation, this would call TTS ROS services
            # For now, we'll simulate speech
            if rclpy is not None and self.ros_node is not None:
                # Call ROS TTS service
                tts_result = self.ros_node.speak_text(text, voice)
                return "success" if tts_result else "speak_failed"
            else:
                # Simulate speech
                print(f"Robot says: {text}")
                time.sleep(len(text.split()) * 0.3)  # Simulate speech time
                return "success"

        except Exception as e:
            self.logger.error(f"Speak action failed: {str(e)}")
            return "speak_failed"

    def _execute_sensing_action(self, action: Action) -> str:
        """
        Execute a sensing action.

        Args:
            action: The sensing action to execute

        Returns:
            Execution result string
        """
        try:
            sensor_type = action.parameters.get("sensor_type", "camera")
            target = action.parameters.get("target", "environment")

            self.logger.info(f"Sensing {target} using {sensor_type}")

            # In a real implementation, this would call sensing ROS services
            # For now, we'll simulate sensing
            if rclpy is not None and self.ros_node is not None:
                # Call ROS sensing service
                sensing_result = self.ros_node.sense_environment(sensor_type, target)
                return "success" if sensing_result else "sensing_failed"
            else:
                # Simulate sensing
                time.sleep(1)  # Simulate sensing time
                return "success"

        except Exception as e:
            self.logger.error(f"Sensing action failed: {str(e)}")
            return "sensing_failed"

    def cancel_action_plan(self, plan_id: str) -> bool:
        """
        Cancel an executing action plan.

        Args:
            plan_id: ID of the action plan to cancel

        Returns:
            True if successfully cancelled, False otherwise
        """
        try:
            if plan_id in self.active_plans:
                # In a real implementation, this would send cancellation requests to ROS actions
                action_plan = self.active_plans[plan_id]
                action_plan.update_status(ActionStatus.CANCELLED)
                del self.active_plans[plan_id]
                self.logger.info(f"Action plan {plan_id} cancelled")
                return True
            else:
                self.logger.warning(f"Action plan {plan_id} not found for cancellation")
                return False
        except Exception as e:
            self.logger.error(f"Error cancelling action plan {plan_id}: {str(e)}")
            return False

    def get_plan_status(self, plan_id: str) -> Optional[ActionStatus]:
        """
        Get the status of an action plan.

        Args:
            plan_id: ID of the action plan

        Returns:
            Status of the action plan, or None if not found
        """
        if plan_id in self.active_plans:
            return self.active_plans[plan_id].status
        elif plan_id in self.action_results:
            # If plan is completed, get status from results
            return self.action_results[plan_id].status
        else:
            return None

    def validate_action_plan(self, action_plan: ActionPlan) -> Tuple[bool, List[str]]:
        """
        Validate an action plan before execution.

        Args:
            action_plan: The action plan to validate

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        # Check if plan has actions
        if not action_plan.actions:
            errors.append("Action plan must contain at least one action")

        # Validate each action
        for i, action in enumerate(action_plan.actions):
            if action.type not in ActionType:
                errors.append(f"Action {i}: Invalid action type {action.type}")

            if action.timeout <= 0:
                errors.append(f"Action {i}: Timeout must be positive")

            if not 0.0 <= action.success_threshold <= 1.0:
                errors.append(f"Action {i}: Success threshold must be between 0.0 and 1.0")

        # Check resource availability
        # In a real implementation, this would check if required resources are available
        for resource in action_plan.required_resources:
            # Here you would check if resource is available
            pass

        is_valid = len(errors) == 0
        return is_valid, errors

    def execute_voice_command(self, voice_command: VoiceCommand) -> ActionPlan:
        """
        Execute a voice command by creating and executing an action plan.

        Args:
            voice_command: The voice command to execute

        Returns:
            Executed action plan
        """
        try:
            # Create an action plan from the voice command
            action_plan = self._create_action_plan_from_voice_command(voice_command)

            # Validate the plan
            is_valid, errors = self.validate_action_plan(action_plan)
            if not is_valid:
                self.logger.error(f"Invalid action plan: {errors}")
                action_plan.update_status(ActionStatus.FAILED)
                return action_plan

            # Execute the plan
            executed_plan = self.execute_action_plan(action_plan)
            return executed_plan

        except Exception as e:
            self.logger.error(f"Error executing voice command: {str(e)}")
            # Create a failed action plan
            failed_plan = ActionPlan(
                command_id=voice_command.id,
                status=ActionStatus.FAILED
            )
            return failed_plan

    def _create_action_plan_from_voice_command(self, voice_command: VoiceCommand) -> ActionPlan:
        """
        Create an action plan from a voice command.

        Args:
            voice_command: The voice command to convert

        Returns:
            Created action plan
        """
        # This would typically use the NLU service to interpret the command
        # and create appropriate actions based on the intent and parameters
        # For now, we'll create a simple plan based on mock interpretation

        # Create actions based on voice command parameters
        actions = []

        # Example: if the command is about picking up an object
        if voice_command.intent and "object" in voice_command.intent.lower():
            action = Action(
                type=ActionType.MANIPULATION,
                parameters=voice_command.parameters,
                timeout=30.0,
                preconditions=["robot_is_idle", "object_detected"],
                postconditions=["object_grasped"],
                success_threshold=0.9
            )
            actions.append(action)

        # Example: if the command is about navigation
        elif voice_command.intent and "navigate" in voice_command.intent.lower():
            action = Action(
                type=ActionType.NAVIGATION,
                parameters=voice_command.parameters,
                timeout=60.0,
                preconditions=["robot_is_idle", "path_clear"],
                postconditions=["robot_at_destination"],
                success_threshold=0.95
            )
            actions.append(action)

        # Create the action plan
        action_plan = ActionPlan(
            command_id=voice_command.id,
            actions=actions,
            priority=3,
            estimated_duration=len(actions) * 30.0,  # Rough estimate
            required_resources=["navigation_system", "manipulator_arm"] if actions else [],
            success_criteria=["command_executed"] if actions else [],
            status=ActionStatus.PENDING
        )

        return action_plan

    def cleanup(self):
        """
        Clean up resources used by the action executor.
        """
        if rclpy is not None and self.ros_node is not None:
            self.ros_node.destroy_node()
            rclpy.shutdown()
            self.logger.info("ROS 2 action executor cleaned up")


class ActionExecutorROSNode:
    """
    ROS 2 node for action execution services.
    This is a simplified implementation for demonstration purposes.
    """

    def __init__(self, node_name: str):
        if rclpy is not None:
            self.node = rclpy.create_node(node_name)
            self.logger = self.node.get_logger()
        else:
            self.node = None
            self.logger = logging.getLogger(__name__)

    def navigate_to_pose(self, x: float, y: float, z: float) -> bool:
        """
        Navigate to a specific pose.

        Args:
            x, y, z: Target coordinates

        Returns:
            True if successful, False otherwise
        """
        if self.node:
            self.logger.info(f"Navigating to pose ({x}, {y}, {z})")
            # In a real implementation, this would call navigation actions
            return True
        else:
            # Simulate navigation
            time.sleep(2)
            return True

    def manipulate_object(self, object_id: str, action_type: str, pose: Dict[str, Any]) -> bool:
        """
        Manipulate an object.

        Args:
            object_id: ID of the object to manipulate
            action_type: Type of manipulation action
            pose: Pose information for the object

        Returns:
            True if successful, False otherwise
        """
        if self.node:
            self.logger.info(f"Manipulating object {object_id} with action {action_type}")
            # In a real implementation, this would call manipulation actions
            return True
        else:
            # Simulate manipulation
            time.sleep(3)
            return True

    def speak_text(self, text: str, voice: str = "default") -> bool:
        """
        Speak text using text-to-speech.

        Args:
            text: Text to speak
            voice: Voice to use

        Returns:
            True if successful, False otherwise
        """
        if self.node:
            self.logger.info(f"Speaking: {text}")
            # In a real implementation, this would call TTS services
            return True
        else:
            # Simulate speech
            print(f"Robot says: {text}")
            return True

    def sense_environment(self, sensor_type: str, target: str) -> bool:
        """
        Sense the environment using specified sensor.

        Args:
            sensor_type: Type of sensor to use
            target: What to sense

        Returns:
            True if successful, False otherwise
        """
        if self.node:
            self.logger.info(f"Sensing {target} with {sensor_type}")
            # In a real implementation, this would call sensing services
            return True
        else:
            # Simulate sensing
            return True

    def destroy_node(self):
        """Destroy the ROS node."""
        if self.node:
            self.node.destroy_node()


# Example usage and testing
if __name__ == "__main__":
    # Test the action executor
    try:
        executor = ActionExecutor()

        # Create a sample action plan
        from ..models.action_plan import ActionPlan, Action, ActionType, ActionStatus

        # Create sample actions
        navigate_action = Action(
            type=ActionType.NAVIGATION,
            parameters={"target_location": "kitchen", "x": 2.0, "y": 1.5, "z": 0.0},
            timeout=60.0,
            preconditions=["robot_is_idle"],
            postconditions=["robot_at_kitchen"],
            success_threshold=0.95
        )

        grasp_action = Action(
            type=ActionType.MANIPULATION,
            parameters={"object_id": "red_cup", "action": "grasp"},
            timeout=30.0,
            preconditions=["robot_at_kitchen", "object_detected"],
            postconditions=["object_grasped"],
            success_threshold=0.9
        )

        # Create an action plan
        action_plan = ActionPlan(
            command_id="cmd_123",
            actions=[navigate_action, grasp_action],
            priority=4,
            estimated_duration=120.0,
            required_resources=["navigation_system", "manipulator_arm"],
            success_criteria=["object_grasped", "robot_returned"],
            status=ActionStatus.PENDING
        )

        # Validate the plan
        is_valid, errors = executor.validate_action_plan(action_plan)
        print(f"Plan is valid: {is_valid}")
        if errors:
            print(f"Validation errors: {errors}")

        if is_valid:
            # Execute the plan
            result_plan = executor.execute_action_plan(action_plan)
            print(f"Plan execution result: {result_plan.status.value}")
            print(f"Execution log: {result_plan.execution_log}")

        # Test with a voice command
        from ..models.voice_command import VoiceCommand

        voice_cmd = VoiceCommand(
            transcript="Go to the kitchen and pick up the red cup",
            intent="object_manipulation",
            parameters={"location": "kitchen", "object": "red cup", "action": "pick_up"}
        )

        voice_result = executor.execute_voice_command(voice_cmd)
        print(f"Voice command execution result: {voice_result.status.value}")

        # Clean up
        executor.cleanup()

    except Exception as e:
        print(f"Error testing action executor: {e}")