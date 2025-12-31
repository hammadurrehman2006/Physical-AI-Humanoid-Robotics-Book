"""
Action Executor ROS 2 Node

This module implements the ROS 2 node for action execution functionality,
handling action planning and execution through robotics action servers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
import threading
import asyncio
import logging
from typing import Optional, Dict, Any
import json

from ..services.action_executor import ActionExecutor
from ..models.action_plan import ActionPlan, Action, ActionStatus
from ..models.voice_command import VoiceCommand


class ActionExecutorNode(Node):
    """
    ROS 2 node for action execution functionality.
    Handles action planning and execution through robotics action servers.
    """

    def __init__(self):
        """Initialize the action executor node."""
        super().__init__('action_executor_node')

        # Setup logging
        self.logger = self.get_logger()

        # Initialize services
        self.action_executor = ActionExecutor(ros_node=self)

        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers
        self.action_plan_sub = self.create_subscription(
            String,
            'action_plan',
            self.action_plan_callback,
            qos_profile
        )

        self.voice_command_sub = self.create_subscription(
            String,
            'processed_voice_command',
            self.voice_command_callback,
            qos_profile
        )

        self.action_cancel_sub = self.create_subscription(
            String,
            'cancel_action',
            self.cancel_action_callback,
            qos_profile
        )

        # Create publishers
        self.action_status_pub = self.create_publisher(
            String,
            'action_status',
            qos_profile
        )

        self.execution_result_pub = self.create_publisher(
            String,
            'execution_result',
            qos_profile
        )

        self.feedback_pub = self.create_publisher(
            String,
            'action_feedback',
            qos_profile
        )

        # Setup parameters
        self.declare_parameter('max_concurrent_actions', 1)
        self.declare_parameter('default_timeout', 60.0)

        self.max_concurrent_actions = self.get_parameter('max_concurrent_actions').value
        self.default_timeout = self.get_parameter('default_timeout').value

        # Setup internal state
        self.active_plans = {}
        self.plan_queue = []
        self.execution_threads = []

        # Setup processing timer
        self.status_timer = self.create_timer(1.0, self.publish_status_updates)

        self.logger.info("Action Executor Node initialized")

    def action_plan_callback(self, msg: String):
        """
        Callback for action plan messages.

        Args:
            msg: String message containing action plan data (JSON format)
        """
        try:
            self.logger.info(f"Received action plan: {msg.data}")

            # Parse the action plan from JSON
            plan_data = json.loads(msg.data)

            # Create an ActionPlan object from the data
            action_plan = self._deserialize_action_plan(plan_data)

            # Add to processing queue
            self.plan_queue.append(action_plan)

            # Publish plan received confirmation
            status_msg = String()
            status_msg.data = f"plan_received:{action_plan.id}"
            self.action_status_pub.publish(status_msg)

        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding action plan JSON: {str(e)}")
            self._publish_error_status(f"json_error:{str(e)}")
        except Exception as e:
            self.logger.error(f"Error in action plan callback: {str(e)}")
            self._publish_error_status(f"callback_error:{str(e)}")

    def voice_command_callback(self, msg: String):
        """
        Callback for processed voice command messages.

        Args:
            msg: String message containing voice command data
        """
        try:
            self.logger.info(f"Received processed voice command: {msg.data}")

            # Parse the command data
            parts = msg.data.split(':')
            if len(parts) >= 2:
                transcript = parts[0]
                confidence_str = parts[1]

                try:
                    confidence = float(confidence_str)
                except ValueError:
                    confidence = 0.0

                # Create a voice command object
                voice_command = VoiceCommand(
                    transcript=transcript,
                    confidence=confidence
                )

                # Execute the voice command
                threading.Thread(
                    target=self._execute_voice_command_thread,
                    args=(voice_command,)
                ).start()

        except Exception as e:
            self.logger.error(f"Error in voice command callback: {str(e)}")

    def cancel_action_callback(self, msg: String):
        """
        Callback for action cancellation messages.

        Args:
            msg: String message containing plan ID to cancel
        """
        try:
            plan_id = msg.data
            self.logger.info(f"Received cancellation request for plan: {plan_id}")

            # Cancel the action plan
            success = self.action_executor.cancel_action_plan(plan_id)

            # Publish cancellation status
            status_msg = String()
            status_msg.data = f"cancel_response:{plan_id}:{success}"
            self.action_status_pub.publish(status_msg)

            if success:
                self.logger.info(f"Successfully cancelled plan: {plan_id}")
            else:
                self.logger.warning(f"Failed to cancel plan: {plan_id}")

        except Exception as e:
            self.logger.error(f"Error in cancel action callback: {str(e)}")

    def _deserialize_action_plan(self, plan_data: Dict[str, Any]) -> ActionPlan:
        """
        Deserialize action plan data from dictionary to ActionPlan object.

        Args:
            plan_data: Dictionary containing action plan data

        Returns:
            ActionPlan object
        """
        # Create actions from the data
        actions = []
        for action_data in plan_data.get('actions', []):
            action = Action(
                action_id=action_data.get('action_id', ''),
                type=action_data.get('type', 'manipulation'),  # Default to manipulation
                parameters=action_data.get('parameters', {}),
                timeout=action_data.get('timeout', 30.0),
                preconditions=action_data.get('preconditions', []),
                postconditions=action_data.get('postconditions', []),
                success_threshold=action_data.get('success_threshold', 0.9)
            )
            actions.append(action)

        # Create the action plan
        action_plan = ActionPlan(
            id=plan_data.get('id', ''),
            command_id=plan_data.get('command_id', ''),
            actions=actions,
            priority=plan_data.get('priority', 3),
            estimated_duration=plan_data.get('estimated_duration', 0.0),
            required_resources=plan_data.get('required_resources', []),
            success_criteria=plan_data.get('success_criteria', []),
            status=ActionStatus.PENDING  # Default to pending
        )

        return action_plan

    def _execute_voice_command_thread(self, voice_command: VoiceCommand):
        """
        Execute a voice command in a separate thread.

        Args:
            voice_command: The voice command to execute
        """
        try:
            self.logger.info(f"Executing voice command: {voice_command.transcript[:50]}...")

            # Execute the voice command using the action executor
            executed_plan = self.action_executor.execute_voice_command(voice_command)

            # Store the executed plan
            self.active_plans[executed_plan.id] = executed_plan

            # Publish execution result
            result_msg = String()
            result_msg.data = f"{executed_plan.id}:{executed_plan.status.value}:{len(executed_plan.actions)}"
            self.execution_result_pub.publish(result_msg)

            # Publish final status
            status_msg = String()
            status_msg.data = f"final_status:{executed_plan.id}:{executed_plan.status.value}"
            self.action_status_pub.publish(status_msg)

            self.logger.info(f"Voice command execution completed with status: {executed_plan.status.value}")

        except Exception as e:
            self.logger.error(f"Error executing voice command: {str(e)}")
            self._publish_error_status(f"execution_error:{str(e)}")

    def _execute_action_plan_thread(self, action_plan: ActionPlan):
        """
        Execute an action plan in a separate thread.

        Args:
            action_plan: The action plan to execute
        """
        try:
            self.logger.info(f"Executing action plan: {action_plan.id}")

            # Validate the action plan
            is_valid, errors = self.action_executor.validate_action_plan(action_plan)
            if not is_valid:
                self.logger.error(f"Invalid action plan {action_plan.id}: {errors}")
                self._publish_error_status(f"validation_error:{';'.join(errors)}")
                return

            # Store the plan as active
            self.active_plans[action_plan.id] = action_plan

            # Execute the action plan
            executed_plan = self.action_executor.execute_action_plan(action_plan)

            # Update stored plan with execution results
            self.active_plans[executed_plan.id] = executed_plan

            # Publish execution result
            result_msg = String()
            result_msg.data = f"{executed_plan.id}:{executed_plan.status.value}:{len(executed_plan.actions)}"
            self.execution_result_pub.publish(result_msg)

            # Publish final status
            status_msg = String()
            status_msg.data = f"final_status:{executed_plan.id}:{executed_plan.status.value}"
            self.action_status_pub.publish(status_msg)

            self.logger.info(f"Action plan {action_plan.id} execution completed with status: {executed_plan.status.value}")

        except Exception as e:
            self.logger.error(f"Error executing action plan {action_plan.id}: {str(e)}")
            self._publish_error_status(f"execution_error:{action_plan.id}:{str(e)}")

            # Update plan status to failed
            if action_plan.id in self.active_plans:
                self.active_plans[action_plan.id].update_status(ActionStatus.FAILED)

    def _publish_error_status(self, error_msg: str):
        """
        Publish an error status message.

        Args:
            error_msg: Error message to publish
        """
        status_msg = String()
        status_msg.data = f"error:{error_msg}"
        self.action_status_pub.publish(status_msg)

    def process_plan_queue(self):
        """
        Process action plans in the queue.
        """
        while self.plan_queue and len(self.active_plans) < self.max_concurrent_actions:
            action_plan = self.plan_queue.pop(0)

            # Start execution in a separate thread
            thread = threading.Thread(
                target=self._execute_action_plan_thread,
                args=(action_plan,)
            )
            thread.start()
            self.execution_threads.append(thread)

    def publish_status_updates(self):
        """
        Publish status updates for active plans.
        """
        for plan_id, plan in self.active_plans.items():
            if plan.status in [ActionStatus.EXECUTING, ActionStatus.PENDING, ActionStatus.VALIDATING]:
                status_msg = String()
                status_msg.data = f"active_status:{plan_id}:{plan.status.value}:{len(plan.execution_log)}"
                self.action_status_pub.publish(status_msg)

    def get_plan_status(self, plan_id: str) -> Optional[ActionStatus]:
        """
        Get the status of a specific action plan.

        Args:
            plan_id: ID of the action plan to check

        Returns:
            Status of the action plan, or None if not found
        """
        plan = self.active_plans.get(plan_id)
        return plan.status if plan else None

    def get_active_plan_count(self) -> int:
        """
        Get the count of active action plans.

        Returns:
            Number of active action plans
        """
        return len(self.active_plans)

    def cleanup(self):
        """
        Clean up resources before shutdown.
        """
        self.logger.info("Cleaning up action executor node...")

        # Cancel all active plans
        for plan_id in list(self.active_plans.keys()):
            self.action_executor.cancel_action_plan(plan_id)

        # Wait for execution threads to complete
        for thread in self.execution_threads:
            thread.join(timeout=5.0)  # Wait up to 5 seconds for each thread

        # Clean up action executor
        self.action_executor.cleanup()


def main(args=None):
    """
    Main function to run the action executor node.
    """
    rclpy.init(args=args)

    action_executor_node = ActionExecutorNode()

    try:
        # Process plan queue in the main thread
        while rclpy.ok():
            action_executor_node.process_plan_queue()
            rclpy.spin_once(action_executor_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        action_executor_node.cleanup()
        action_executor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()