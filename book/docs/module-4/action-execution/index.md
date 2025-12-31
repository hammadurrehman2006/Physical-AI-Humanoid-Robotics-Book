---
sidebar_position: 4
title: "Action Execution"
---

# Action Execution

In this section, we'll implement the action execution component of our Vision-Language-Action system. This module translates high-level commands from the language understanding and visual perception systems into specific robotic actions and movements.

## Overview

Action execution is the final component of our VLA system, where the robot physically responds to interpreted commands. This section covers:
- Action planning and trajectory generation
- Motor control and manipulation systems
- Action execution and monitoring
- Integration with perception systems for closed-loop control
- Safety and error handling in action execution

## Learning Objectives

By the end of this section, you will be able to:
- Implement action planning systems that generate executable trajectories
- Control robotic manipulators and navigation systems for task execution
- Monitor action execution with feedback and error handling
- Integrate perception systems for closed-loop action execution
- Implement safety mechanisms and recovery procedures

## Action Planning and Representation

### Action Space Definition

First, let's define the action space for our robot:

```python
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Any, Optional, Tuple
import numpy as np

class ActionType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    GAZE_CONTROL = "gaze_control"
    SPEECH = "speech"
    INTERACTION = "interaction"

@dataclass
class Action:
    action_type: ActionType
    parameters: Dict[str, Any]
    priority: int = 0
    timeout: float = 30.0  # seconds
    preconditions: List[str] = None
    effects: List[str] = None

@dataclass
class NavigationAction(Action):
    """Navigation action with path planning"""
    def __post_init__(self):
        self.action_type = ActionType.NAVIGATION

@dataclass
class ManipulationAction(Action):
    """Manipulation action for robotic arms"""
    def __post_init__(self):
        self.action_type = ActionType.MANIPULATION

@dataclass
class GazeAction(Action):
    """Gaze control action for head/eye movements"""
    def __post_init__(self):
        self.action_type = ActionType.GAZE_CONTROL
```

### Action Planner

The action planner generates sequences of primitive actions to achieve high-level goals:

```python
from abc import ABC, abstractmethod
from typing import Union
import time

class ActionPlanner:
    def __init__(self):
        self.robot_state = None
        self.world_model = None
        self.action_library = self._initialize_action_library()

    def _initialize_action_library(self) -> Dict[str, Action]:
        """Initialize available actions"""
        return {
            'move_to': NavigationAction(
                parameters={'target_position': None, 'speed': 0.5},
                preconditions=['robot_is_operational'],
                effects=['robot_at_target']
            ),
            'grasp_object': ManipulationAction(
                parameters={'object_id': None, 'position': None, 'gripper_force': 50},
                preconditions=['object_detected', 'arm_free', 'reachable'],
                effects=['object_grasped']
            ),
            'place_object': ManipulationAction(
                parameters={'position': None, 'orientation': None},
                preconditions=['object_grasped'],
                effects=['object_placed', 'gripper_open']
            ),
            'point_to': ManipulationAction(
                parameters={'target_position': None, 'duration': 2.0},
                preconditions=['arm_free'],
                effects=['pointing_at_target']
            ),
            'look_at': GazeAction(
                parameters={'target_position': None},
                preconditions=['head_operational'],
                effects=['looking_at_target']
            )
        }

    def plan_action(self, high_level_command: Dict[str, Any]) -> List[Action]:
        """Plan primitive actions from high-level command"""
        intent = high_level_command.get('intent', '')
        entities = high_level_command.get('entities', {})

        actions = []

        if intent == 'navigate':
            target_location = entities.get('location', ['default'])[0]
            actions.append(self._create_navigation_action(target_location))

        elif intent == 'grasp':
            target_object = entities.get('target_object', entities.get('object', ['default'])[0])
            object_position = entities.get('object_position', None)
            actions.extend(self._create_grasp_sequence(target_object, object_position))

        elif intent == 'place':
            target_location = entities.get('location', ['default'])[0]
            actions.append(self._create_place_action(target_location))

        elif intent == 'identify':
            target_object = entities.get('target_object', entities.get('object', ['default'])[0])
            actions.extend(self._create_identification_sequence(target_object))

        return actions

    def _create_navigation_action(self, target_location: str) -> NavigationAction:
        """Create navigation action for target location"""
        # In a real system, this would use semantic map
        # For now, we'll use predefined locations
        location_map = {
            'kitchen': [2.0, 2.0, 0.0],
            'bedroom': [-2.0, 2.0, 0.0],
            'office': [0.0, -2.0, 0.0],
            'living room': [2.0, -2.0, 0.0],
            'default': [0.0, 0.0, 0.0]
        }

        target_position = location_map.get(target_location.lower(), location_map['default'])

        return NavigationAction(
            parameters={
                'target_position': target_position,
                'speed': 0.5,
                'avoid_obstacles': True
            },
            priority=1
        )

    def _create_grasp_sequence(self, target_object: str, object_position: Optional[List[float]]) -> List[Action]:
        """Create sequence of actions for grasping an object"""
        actions = []

        if object_position:
            # Move to object
            actions.append(NavigationAction(
                parameters={
                    'target_position': [object_position[0], object_position[1], 0.0],  # Move to object location
                    'speed': 0.3,
                    'avoid_obstacles': True
                },
                priority=2
            ))

        # Approach and grasp
        actions.append(ManipulationAction(
            parameters={
                'object_id': target_object,
                'approach_distance': 0.1,
                'gripper_force': 30
            },
            priority=3
        ))

        return actions

    def _create_place_action(self, target_location: str) -> ManipulationAction:
        """Create action for placing an object"""
        location_positions = {
            'table': [1.0, 0.0, 0.8],  # Standard table height
            'shelf': [1.0, 0.0, 1.2],  # Standard shelf height
            'counter': [0.5, 0.0, 0.9],
            'default': [0.0, 0.0, 0.8]
        }

        position = location_positions.get(target_location.lower(), location_positions['default'])

        return ManipulationAction(
            parameters={
                'target_position': position,
                'release_force': 0,
                'orientation': [0, 0, 0]  # Default orientation
            },
            priority=2
        )

    def _create_identification_sequence(self, target_object: str) -> List[Action]:
        """Create sequence of actions for identifying an object"""
        actions = []

        # Point to the object
        actions.append(ManipulationAction(
            parameters={
                'target_object': target_object,
                'motion_type': 'pointing',
                'duration': 2.0
            },
            priority=1
        ))

        # Look at the object
        actions.append(GazeAction(
            parameters={
                'target_object': target_object
            },
            priority=1
        ))

        return actions
```

## Robot Control Systems

### Navigation Control

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
from tf2_ros import TransformException
import math

class NavigationController:
    def __init__(self, node: Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # Navigation action client
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # Publishers and subscribers
        self.cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = node.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = node.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Robot state
        self.current_pose = None
        self.current_velocity = None
        self.scan_data = None

        # Navigation parameters
        self.linear_vel_limit = 0.5
        self.angular_vel_limit = 1.0
        self.arrival_threshold = 0.2  # meters

    def odom_callback(self, msg: Odometry):
        """Update robot's current pose"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def scan_callback(self, msg: LaserScan):
        """Update laser scan data"""
        self.scan_data = msg

    def navigate_to_pose(self, target_pose: List[float]) -> bool:
        """Navigate to target pose using navigation2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target_pose[0]
        goal_msg.pose.pose.position.y = target_pose[1]
        goal_msg.pose.pose.position.z = target_pose[2] if len(target_pose) > 2 else 0.0

        # Set orientation to face forward
        goal_msg.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'Navigating to: {target_pose}')

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Navigation goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Navigation succeeded')
            return True
        else:
            self.node.get_logger().error(f'Navigation failed with status: {status}')
            return False

    def simple_move_to(self, target_x: float, target_y: float, speed: float = 0.3) -> bool:
        """Simple proportional controller for navigation"""
        rate = self.node.create_rate(10)  # 10 Hz

        while rclpy.ok():
            if not self.current_pose:
                continue

            # Calculate distance to target
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < self.arrival_threshold:
                # Stop the robot
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                return True

            # Calculate heading to target
            target_angle = math.atan2(dy, dx)
            current_angle = self._get_yaw_from_quaternion(self.current_pose.orientation)

            # Simple proportional control
            angular_error = target_angle - current_angle
            # Normalize angle to [-pi, pi]
            while angular_error > math.pi:
                angular_error -= 2 * math.pi
            while angular_error < -math.pi:
                angular_error += 2 * math.pi

            # Create velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = min(speed, distance * 0.5)  # Proportional to distance
            cmd_vel.angular.z = angular_error * 1.0  # Proportional to angular error

            # Apply velocity limits
            cmd_vel.linear.x = max(-self.linear_vel_limit, min(cmd_vel.linear.x, self.linear_vel_limit))
            cmd_vel.angular.z = max(-self.angular_vel_limit, min(cmd_vel.angular.z, self.angular_vel_limit))

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

    def _get_yaw_from_quaternion(self, quaternion) -> float:
        """Extract yaw angle from quaternion"""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
```

### Manipulation Control

```python
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import math

class ManipulationController:
    def __init__(self, node: Node):
        self.node = node

        # Publishers and subscribers
        self.joint_state_sub = node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.trajectory_pub = node.create_publisher(
            JointTrajectory, 'joint_trajectory', 10
        )

        # Joint trajectory action client for more complex motions
        self.trajectory_client = ActionClient(
            node, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory'
        )

        # Current joint states
        self.joint_names = []
        self.joint_positions = {}
        self.joint_velocities = {}

        # Manipulator parameters
        self.gripper_joint = 'gripper_joint'
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def joint_state_callback(self, msg: JointState):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

        self.joint_names = msg.name

    def grasp_object(self, object_position: List[float], gripper_force: int = 50) -> bool:
        """Execute grasping action"""
        try:
            # Plan approach trajectory
            approach_pose = self._calculate_approach_pose(object_position)

            # Move to approach position
            if not self.move_to_pose(approach_pose, speed=0.1):
                return False

            # Execute grasp
            return self.execute_grasp(gripper_force)

        except Exception as e:
            self.node.get_logger().error(f'Grasp failed: {e}')
            return False

    def place_object(self, target_position: List[float], orientation: List[float] = None) -> bool:
        """Execute placing action"""
        try:
            if orientation is None:
                orientation = [0, 0, 0, 1]  # Default orientation

            target_pose = Pose()
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            target_pose.orientation.x = orientation[0]
            target_pose.orientation.y = orientation[1]
            target_pose.orientation.z = orientation[2]
            target_pose.orientation.w = orientation[3]

            # Move to target position
            if not self.move_to_pose(target_pose, speed=0.1):
                return False

            # Release object
            return self.release_object()

        except Exception as e:
            self.node.get_logger().error(f'Place failed: {e}')
            return False

    def move_to_pose(self, target_pose: Pose, speed: float = 0.5) -> bool:
        """Move manipulator to target pose"""
        # In a real system, this would use inverse kinematics
        # For this example, we'll simulate the movement
        self.node.get_logger().info(f'Moving to pose: {target_pose}')

        # This would involve solving inverse kinematics and executing joint trajectory
        # For now, we'll return success
        return True

    def execute_grasp(self, force: int) -> bool:
        """Execute gripper grasp"""
        self.node.get_logger().info(f'Executing grasp with force: {force}')

        # Send gripper command
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint]

        point = JointTrajectoryPoint()
        # This is a simplified example - real grippers have different control methods
        point.positions = [force / 100.0]  # Scale force to joint position
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points = [point]

        self.trajectory_pub.publish(trajectory)
        self.node.get_logger().info('Grasp command published')

        return True

    def release_object(self) -> bool:
        """Release gripper"""
        self.node.get_logger().info('Releasing object')

        # Send gripper release command
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint]

        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Open gripper
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points = [point]

        self.trajectory_pub.publish(trajectory)
        self.node.get_logger().info('Release command published')

        return True

    def _calculate_approach_pose(self, object_position: List[float]) -> Pose:
        """Calculate approach pose before grasping"""
        approach_pose = Pose()
        approach_pose.position.x = object_position[0] - 0.1  # Approach from front
        approach_pose.position.y = object_position[1]
        approach_pose.position.z = object_position[2] + 0.1  # Approach from above
        approach_pose.orientation.w = 1.0  # Default orientation

        return approach_pose
```

## Action Execution Framework

### Action Executor

```python
import asyncio
from typing import Callable
from enum import Enum
import threading
import time

class ExecutionStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"

class ActionExecutor:
    def __init__(self, node: Node):
        self.node = node
        self.navigation_controller = NavigationController(node)
        self.manipulation_controller = ManipulationController(node)

        self.current_action = None
        self.execution_status = ExecutionStatus.PENDING
        self.execution_thread = None
        self.stop_execution = threading.Event()

    def execute_action(self, action: Action) -> bool:
        """Execute a single action"""
        self.current_action = action
        self.execution_status = ExecutionStatus.EXECUTING
        self.stop_execution.clear()

        try:
            if action.action_type == ActionType.NAVIGATION:
                return self._execute_navigation(action)
            elif action.action_type == ActionType.MANIPULATION:
                return self._execute_manipulation(action)
            elif action.action_type == ActionType.GAZE_CONTROL:
                return self._execute_gaze_control(action)
            else:
                self.node.get_logger().error(f'Unknown action type: {action.action_type}')
                return False

        except Exception as e:
            self.node.get_logger().error(f'Action execution failed: {e}')
            self.execution_status = ExecutionStatus.FAILED
            return False

    def execute_action_sequence(self, actions: List[Action]) -> bool:
        """Execute a sequence of actions"""
        for action in actions:
            if self.stop_execution.is_set():
                self.execution_status = ExecutionStatus.CANCELLED
                return False

            self.node.get_logger().info(f'Executing action: {action.action_type}')

            success = self.execute_action(action)
            if not success:
                self.node.get_logger().error(f'Action failed: {action.action_type}')
                return False

        self.execution_status = ExecutionStatus.SUCCEEDED
        return True

    def _execute_navigation(self, action: NavigationAction) -> bool:
        """Execute navigation action"""
        target_position = action.parameters.get('target_position')
        speed = action.parameters.get('speed', 0.5)
        avoid_obstacles = action.parameters.get('avoid_obstacles', True)

        if not target_position:
            self.node.get_logger().error('Navigation action missing target position')
            return False

        self.node.get_logger().info(f'Navigating to: {target_position}')

        # Use the navigation controller to execute the navigation
        success = self.navigation_controller.navigate_to_pose(target_position)

        if success:
            self.node.get_logger().info('Navigation completed successfully')
        else:
            self.node.get_logger().error('Navigation failed')

        return success

    def _execute_manipulation(self, action: ManipulationAction) -> bool:
        """Execute manipulation action"""
        action_type = action.parameters.get('action_type', 'custom')
        object_id = action.parameters.get('object_id')
        position = action.parameters.get('position')

        if action_type == 'grasp_object':
            object_position = action.parameters.get('position')
            gripper_force = action.parameters.get('gripper_force', 50)

            if not object_position:
                self.node.get_logger().error('Grasp action missing object position')
                return False

            return self.manipulation_controller.grasp_object(object_position, gripper_force)

        elif action_type == 'place_object':
            target_position = action.parameters.get('target_position')
            orientation = action.parameters.get('orientation')

            if not target_position:
                self.node.get_logger().error('Place action missing target position')
                return False

            return self.manipulation_controller.place_object(target_position, orientation)

        else:
            self.node.get_logger().info(f'Executing custom manipulation: {action_type}')
            # Implement other manipulation actions as needed
            return True

    def _execute_gaze_control(self, action: GazeAction) -> bool:
        """Execute gaze control action"""
        target_position = action.parameters.get('target_position')
        target_object = action.parameters.get('target_object')

        self.node.get_logger().info(f'Gaze control to: {target_position or target_object}')

        # In a real system, this would control head/eye movements
        # For now, we'll just log the action
        return True

    def cancel_execution(self):
        """Cancel current action execution"""
        self.stop_execution.set()
        self.execution_status = ExecutionStatus.CANCELLED
```

## Integration with Perception Systems

### Closed-Loop Control

```python
class ClosedLoopController:
    def __init__(self, node: Node):
        self.node = node
        self.action_executor = ActionExecutor(node)
        self.perception_handler = None  # Will be set externally
        self.robot_state = None

    def set_perception_handler(self, handler):
        """Set the perception handler for feedback"""
        self.perception_handler = handler

    def execute_with_feedback(self, action_sequence: List[Action],
                            max_attempts: int = 3) -> bool:
        """Execute actions with perception feedback and error recovery"""

        for attempt in range(max_attempts):
            self.node.get_logger().info(f'Execution attempt {attempt + 1}')

            success = self.action_executor.execute_action_sequence(action_sequence)

            if success:
                self.node.get_logger().info('Action sequence completed successfully')
                return True

            # Check for failure cause and potentially replan
            feedback = self._get_perception_feedback()
            recovery_action = self._generate_recovery_action(feedback, action_sequence)

            if recovery_action:
                self.node.get_logger().info('Executing recovery action')
                recovery_success = self.action_executor.execute_action(recovery_action)

                if recovery_success:
                    # Retry the original sequence
                    continue
                else:
                    self.node.get_logger().error('Recovery failed')
                    return False
            else:
                self.node.get_logger().error('No recovery action available')
                return False

        return False

    def _get_perception_feedback(self):
        """Get current perception feedback"""
        if self.perception_handler:
            return self.perception_handler.get_current_perception()
        return None

    def _generate_recovery_action(self, feedback, original_sequence):
        """Generate recovery action based on failure"""
        # This is a simplified example
        # In practice, this would involve more sophisticated error analysis

        if not feedback:
            return None

        # Example: if object not found, try to look around
        if 'object_missing' in feedback:
            return GazeAction(
                parameters={'action_type': 'search', 'search_pattern': 'spiral'}
            )

        # Example: if navigation failed due to obstacle, try alternative path
        if 'obstacle_detected' in feedback:
            return NavigationAction(
                parameters={'action_type': 'avoid_obstacle'}
            )

        return None

class AdaptiveActionPlanner:
    def __init__(self):
        self.action_planner = ActionPlanner()
        self.feedback_history = []

    def plan_with_context(self, high_level_command: Dict[str, Any],
                         context: Dict[str, Any] = None) -> List[Action]:
        """Plan actions considering context and feedback history"""

        # Adjust plan based on context
        adjusted_command = self._adjust_command_for_context(high_level_command, context)

        # Generate initial plan
        action_sequence = self.action_planner.plan_action(adjusted_command)

        # Adapt plan based on feedback history
        adapted_sequence = self._adapt_plan_for_feedback(action_sequence)

        return adapted_sequence

    def _adjust_command_for_context(self, command: Dict[str, Any],
                                  context: Dict[str, Any]) -> Dict[str, Any]:
        """Adjust command based on current context"""
        if not context:
            return command

        adjusted_command = command.copy()

        # Example: if object location is known more precisely, update command
        if 'object_locations' in context and 'object' in command.get('entities', {}):
            known_objects = context['object_locations']
            target_obj = command['entities']['object'][0]

            for obj_id, location in known_objects.items():
                if target_obj.lower() in obj_id.lower():
                    adjusted_command['entities']['object_position'] = location
                    break

        return adjusted_command

    def _adapt_plan_for_feedback(self, action_sequence: List[Action]) -> List[Action]:
        """Adapt action sequence based on feedback history"""
        # This would implement learning from past failures/successes
        # For now, return the original sequence
        return action_sequence

    def record_feedback(self, command: Dict[str, Any], result: bool, execution_time: float):
        """Record feedback for learning"""
        feedback = {
            'command': command,
            'result': result,
            'execution_time': execution_time,
            'timestamp': time.time()
        }
        self.feedback_history.append(feedback)
```

## Action Execution ROS Node

### Complete ROS 2 Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
import json

class ActionExecutionNode(Node):
    def __init__(self):
        super().__init__('action_execution_node')

        # Publishers and subscribers
        self.result_pub = self.create_publisher(String, 'action_results', 10)
        self.command_sub = self.create_subscription(
            String, 'parsed_commands', self.command_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, 'detected_objects', self.perception_callback, 10
        )

        # Initialize components
        self.action_planner = AdaptiveActionPlanner()
        self.action_executor = ActionExecutor(self)
        self.closed_loop_controller = ClosedLoopController(self)

        # Current state
        self.current_perception = None
        self.action_queue = []
        self.is_executing = False

        self.get_logger().info("Action Execution Node initialized")

    def command_callback(self, msg: String):
        """Process incoming commands"""
        try:
            command_data = json.loads(msg.data)

            # Get current context
            context = self._get_current_context()

            # Plan actions
            action_sequence = self.action_planner.plan_with_context(
                command_data, context
            )

            self.get_logger().info(
                f"Planned {len(action_sequence)} actions for command: {command_data.get('intent', 'unknown')}"
            )

            # Execute actions
            if not self.is_executing:
                self.is_executing = True
                success = self.closed_loop_controller.execute_with_feedback(action_sequence)

                # Publish result
                result_msg = String()
                result_msg.data = json.dumps({
                    'command_id': command_data.get('command_id', 'unknown'),
                    'success': success,
                    'timestamp': self.get_clock().now().to_msg()
                })
                self.result_pub.publish(result_msg)

                self.is_executing = False

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")

    def perception_callback(self, msg: String):
        """Update perception context"""
        try:
            perception_data = json.loads(msg.data)
            self.current_perception = perception_data
        except Exception as e:
            self.get_logger().error(f"Error processing perception: {e}")

    def _get_current_context(self) -> Dict[str, Any]:
        """Get current context for planning"""
        context = {}

        if self.current_perception:
            context['object_locations'] = {}
            for obj in self.current_perception.get('objects', []):
                obj_name = obj.get('name', 'unknown')
                position = obj.get('position', [0, 0, 0])
                if position:  # If position is available
                    context['object_locations'][obj_name] = position

        return context

def main(args=None):
    rclpy.init(args=args)

    node = ActionExecutionNode()

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

## Safety and Error Handling

### Safety Monitor

```python
class SafetyMonitor:
    def __init__(self, node: Node):
        self.node = node
        self.safety_limits = {
            'max_velocity': 0.5,
            'max_force': 100.0,
            'max_torque': 50.0,
            'collision_threshold': 0.1,
            'joint_limits': {
                'min': -2.0,
                'max': 2.0
            }
        }

        # Subscribe to safety-relevant topics
        self.joint_state_sub = node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.scan_sub = node.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        self.current_joint_states = None
        self.current_scan_data = None

    def joint_state_callback(self, msg: JointState):
        """Monitor joint states for safety"""
        self.current_joint_states = msg

    def scan_callback(self, msg: LaserScan):
        """Monitor scan data for collision detection"""
        self.current_scan_data = msg

    def check_safety(self, action: Action) -> bool:
        """Check if action is safe to execute"""
        # Check joint limits
        if not self._check_joint_limits():
            self.node.get_logger().warn('Joint limits violation detected')
            return False

        # Check collision risk
        if not self._check_collision_risk():
            self.node.get_logger().warn('Collision risk detected')
            return False

        # Check force/torque limits for manipulation
        if action.action_type == ActionType.MANIPULATION:
            if not self._check_manipulation_safety(action):
                self.node.get_logger().warn('Manipulation safety violation')
                return False

        return True

    def _check_joint_limits(self) -> bool:
        """Check if joints are within safe limits"""
        if not self.current_joint_states:
            return True  # No data, assume safe

        for position in self.current_joint_states.position:
            if (position < self.safety_limits['joint_limits']['min'] or
                position > self.safety_limits['joint_limits']['max']):
                return False

        return True

    def _check_collision_risk(self) -> bool:
        """Check for potential collisions"""
        if not self.current_scan_data:
            return True  # No data, assume safe

        # Check for obstacles within threshold
        min_distance = min([r for r in self.current_scan_data.ranges if r > 0], default=float('inf'))

        if min_distance < self.safety_limits['collision_threshold']:
            return False

        return True

    def _check_manipulation_safety(self, action: ManipulationAction) -> bool:
        """Check safety for manipulation actions"""
        force = action.parameters.get('gripper_force', 0)
        if force > self.safety_limits['max_force']:
            return False

        return True

class ActionExecutionManager:
    def __init__(self, node: Node):
        self.node = node
        self.action_executor = ActionExecutor(node)
        self.safety_monitor = SafetyMonitor(node)
        self.recovery_manager = RecoveryManager(node)

    def execute_safe_action(self, action: Action) -> bool:
        """Execute action with safety checks"""
        # Check safety before execution
        if not self.safety_monitor.check_safety(action):
            self.node.get_logger().error('Safety check failed, aborting action')
            return False

        # Execute action
        success = self.action_executor.execute_action(action)

        # If action failed, attempt recovery
        if not success:
            recovery_success = self.recovery_manager.attempt_recovery(action)
            if recovery_success:
                # Retry original action after recovery
                return self.action_executor.execute_action(action)

        return success

class RecoveryManager:
    def __init__(self, node: Node):
        self.node = node
        self.recovery_strategies = {
            'collision': self._handle_collision,
            'timeout': self._handle_timeout,
            'joint_limit': self._handle_joint_limit,
            'gripper_failure': self._handle_gripper_failure
        }

    def attempt_recovery(self, failed_action: Action) -> bool:
        """Attempt to recover from action failure"""
        # Determine failure type and apply appropriate recovery
        failure_type = self._analyze_failure(failed_action)

        if failure_type in self.recovery_strategies:
            return self.recovery_strategies[failure_type](failed_action)

        return False

    def _analyze_failure(self, action: Action) -> str:
        """Analyze what type of failure occurred"""
        # This would be implemented based on system feedback
        # For now, return a default failure type
        return 'timeout'  # Default to timeout for example

    def _handle_collision(self, action: Action) -> bool:
        """Handle collision failure"""
        self.node.get_logger().info('Attempting collision recovery')
        # Implement collision recovery logic
        return True

    def _handle_timeout(self, action: Action) -> bool:
        """Handle timeout failure"""
        self.node.get_logger().info('Attempting timeout recovery')
        # Implement timeout recovery logic
        return True

    def _handle_joint_limit(self, action: Action) -> bool:
        """Handle joint limit failure"""
        self.node.get_logger().info('Attempting joint limit recovery')
        # Implement joint limit recovery logic
        return True

    def _handle_gripper_failure(self, action: Action) -> bool:
        """Handle gripper failure"""
        self.node.get_logger().info('Attempting gripper failure recovery')
        # Implement gripper failure recovery logic
        return True
```

## Testing and Validation

### Unit Tests

```python
import unittest
from unittest.mock import Mock, MagicMock

class TestActionExecution(unittest.TestCase):
    def setUp(self):
        self.action_planner = ActionPlanner()
        self.navigation_controller = Mock()
        self.manipulation_controller = Mock()

    def test_action_planning(self):
        """Test action planning from high-level commands"""
        command = {
            'intent': 'navigate',
            'entities': {'location': ['kitchen']}
        }

        actions = self.action_planner.plan_action(command)

        self.assertIsInstance(actions, list)
        self.assertGreater(len(actions), 0)
        self.assertEqual(actions[0].action_type, ActionType.NAVIGATION)

    def test_grasp_sequence_planning(self):
        """Test grasp action sequence planning"""
        command = {
            'intent': 'grasp',
            'entities': {
                'target_object': ['red_ball'],
                'object_position': [1.0, 2.0, 0.5]
            }
        }

        actions = self.action_planner.plan_action(command)

        # Should include navigation and manipulation actions
        nav_actions = [a for a in actions if a.action_type == ActionType.NAVIGATION]
        manip_actions = [a for a in actions if a.action_type == ActionType.MANIPULATION]

        self.assertGreater(len(nav_actions), 0)
        self.assertGreater(len(manip_actions), 0)

    def test_action_executor_initialization(self):
        """Test action executor initialization"""
        node = Mock()
        executor = ActionExecutor(node)

        self.assertIsNotNone(executor.navigation_controller)
        self.assertIsNotNone(executor.manipulation_controller)
        self.assertEqual(executor.execution_status, ExecutionStatus.PENDING)

    def test_navigation_action_execution(self):
        """Test navigation action execution"""
        node = Mock()
        executor = ActionExecutor(node)

        # Mock the navigation controller
        executor.navigation_controller.navigate_to_pose = Mock(return_value=True)

        action = NavigationAction(
            parameters={'target_position': [1.0, 2.0, 0.0]}
        )

        success = executor.execute_action(action)

        self.assertTrue(success)
        executor.navigation_controller.navigate_to_pose.assert_called_once()

    def test_manipulation_action_execution(self):
        """Test manipulation action execution"""
        node = Mock()
        executor = ActionExecutor(node)

        # Mock the manipulation controller
        executor.manipulation_controller.grasp_object = Mock(return_value=True)

        action = ManipulationAction(
            parameters={
                'action_type': 'grasp_object',
                'position': [1.0, 2.0, 0.5],
                'gripper_force': 50
            }
        )

        success = executor.execute_action(action)

        self.assertTrue(success)
        executor.manipulation_controller.grasp_object.assert_called_once()

    def test_action_sequence_execution(self):
        """Test execution of action sequences"""
        node = Mock()
        executor = ActionExecutor(node)

        # Mock the controllers
        executor.navigation_controller.navigate_to_pose = Mock(return_value=True)
        executor.manipulation_controller.grasp_object = Mock(return_value=True)

        actions = [
            NavigationAction(parameters={'target_position': [1.0, 2.0, 0.0]}),
            ManipulationAction(parameters={'action_type': 'grasp_object', 'position': [1.0, 2.0, 0.5]})
        ]

        success = executor.execute_action_sequence(actions)

        self.assertTrue(success)

    def test_safety_monitor(self):
        """Test safety monitoring"""
        node = Mock()
        safety_monitor = SafetyMonitor(node)

        # Test joint limit check
        result = safety_monitor._check_joint_limits()
        self.assertTrue(result)  # Should return True when no data (safe assumption)

        # Test collision check
        result = safety_monitor._check_collision_risk()
        self.assertTrue(result)  # Should return True when no data (safe assumption)

if __name__ == '__main__':
    unittest.main()
```

## Configuration and Setup

### Configuration File

```yaml
# config/action_execution.yaml
action_execution:
  planning:
    max_action_sequence_length: 20
    default_timeout: 30.0
    replan_on_failure: true
    max_replan_attempts: 3

  navigation:
    default_speed: 0.5
    arrival_threshold: 0.2
    obstacle_avoidance: true
    max_velocity: 0.8

  manipulation:
    default_gripper_force: 50
    approach_distance: 0.1
    release_force: 0
    max_force: 100.0

  safety:
    enable_safety_monitor: true
    max_collision_threshold: 0.1
    joint_limit_margin: 0.1
    velocity_limit: 0.5

  execution:
    enable_feedback: true
    enable_recovery: true
    recovery_attempts: 3
    enable_adaptation: true
```

### Launch File

```xml
<!-- launch/action_execution.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="action_execution_node" name="action_execution">
    <param name="default_timeout" value="30.0"/>
    <param name="enable_safety_monitor" value="true"/>
  </node>
</launch>
```

## Troubleshooting

### Common Issues

1. **Action Execution Timeouts**
   - Solution: Increase timeout values or optimize action execution speed
   - Check: Ensure robot hardware is responsive and properly calibrated

2. **Navigation Failures**
   - Solution: Verify map quality and localization accuracy
   - Check: Ensure navigation stack is properly configured

3. **Manipulation Failures**
   - Solution: Check inverse kinematics solutions and joint limits
   - Check: Verify object detection accuracy and pose estimation

4. **Safety System Interference**
   - Solution: Adjust safety parameters appropriately
   - Check: Ensure safety thresholds are reasonable for your robot

## Next Steps

In the next section, we'll integrate all three components - vision, language, and action - into a cohesive multi-modal fusion system. The individual components we've built will be combined to create a complete Vision-Language-Action system capable of understanding natural language commands, perceiving the environment visually, and executing appropriate actions.

Continue to [Multi-Modal Fusion](../multi-modal-fusion/index.md).