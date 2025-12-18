---
title: Actions in Robotic Systems
description: Understanding ROS 2 actions for long-running tasks and goal management
sidebar_position: 4
---

# Actions in Robotic Systems

## Learning Objectives
- Understand the concept of ROS 2 actions and when to use them
- Implement action servers and clients for goal-oriented tasks
- Manage goal feedback and result reporting
- Handle action preemption and cancellation
- Create robust action-based robotic behaviors

## Introduction to ROS 2 Actions

Actions in ROS 2 provide a communication pattern specifically designed for long-running tasks that require feedback and the ability to be cancelled. Unlike services, which are synchronous and blocking, actions are asynchronous and provide continuous feedback during execution.

### When to Use Actions

Actions are ideal for tasks that:
- Take a significant amount of time to complete
- Require continuous feedback to the client
- Need to be cancellable or preemptable
- Have intermediate results during execution
- Represent goal-oriented behaviors

Examples include:
- Navigation to a specific location
- Robot arm motion planning and execution
- Object manipulation tasks
- Long-running data processing
- Calibration procedures

## Action Message Structure

Actions are composed of three message types:

1. **Goal**: Specifies what the action should do
2. **Feedback**: Provides intermediate status updates
3. **Result**: Contains the final outcome of the action

### Example Action Definition

```# Fibonacci.action
# Define a Fibonacci action that calculates a sequence

# Goal definition
int32 order

---
# Result definition
int32[] sequence

---
# Feedback definition
int32[] sequence
```

## Creating an Action Server

An action server handles incoming goals, executes them, and provides feedback and results.

### Basic Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server with callback group for reentrancy
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Fibonacci action server started')

    def goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        self.get_logger().info(f'Received goal request: {goal_request.order}')

        # Accept all goals for this example
        # In practice, you might reject goals that are impossible to achieve
        if goal_request.order > 0:
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Get the goal order
        order = goal_handle.request.order

        # Create feedback and result messages
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Initialize the Fibonacci sequence
        feedback_msg.sequence = [0, 1]

        # Start executing the action
        for i in range(1, order):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal was cancelled')
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # Update the sequence
            if i < len(feedback_msg.sequence):
                continue
            else:
                feedback_msg.sequence.append(
                    feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]
                )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Sleep to simulate work (in real applications, this would be actual work)
            from time import sleep
            sleep(0.5)

        # Check if goal was canceled during execution
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal was cancelled')
            result_msg.sequence = feedback_msg.sequence
            goal_handle.canceled()
            return result_msg

        # Goal completed successfully
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()

        self.get_logger().info(f'Goal succeeded with result: {result_msg.sequence}')
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()

    try:
        # Use MultiThreadedExecutor to handle callbacks properly
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating an Action Client

An action client sends goals to an action server and can monitor progress, receive feedback, and cancel goals.

### Basic Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        """Send a goal to the action server."""
        self.get_logger().info(f'Waiting for action server...')

        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send the goal and register callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callbacks for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        # Shutdown after receiving result
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    # Send a goal
    future = action_client.send_goal(5)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()

if __name__ == '__main__':
    main()
```

## Advanced Action Server Features

### Handling Multiple Concurrent Goals

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci
import threading
from concurrent.futures import ThreadPoolExecutor

class ConcurrentFibonacciActionServer(Node):
    def __init__(self):
        super().__init__('concurrent_fibonacci_action_server')

        # Thread pool for handling multiple goals concurrently
        self.executor_pool = ThreadPoolExecutor(max_workers=3)

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'concurrent_fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback
        )

        self.get_logger().info('Concurrent Fibonacci action server started')

    def goal_callback(self, goal_request):
        """Accept all goals."""
        self.get_logger().info(f'Received goal: {goal_request.order}')
        return rclpy.action.server.GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal in a separate thread."""
        self.get_logger().info('Executing goal in separate thread...')

        # Submit the execution to the thread pool
        future = self.executor_pool.submit(self.compute_fibonacci, goal_handle)
        result = future.result()

        return result

    def compute_fibonacci(self, goal_handle):
        """Compute Fibonacci sequence in a separate thread."""
        order = goal_handle.request.order
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Initialize sequence
        if order >= 1:
            feedback_msg.sequence = [0]
        if order >= 2:
            feedback_msg.sequence.append(1)

        # Compute the sequence
        for i in range(2, order):
            if goal_handle.is_cancel_requested:
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # Calculate next Fibonacci number
            next_num = feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
            feedback_msg.sequence.append(next_num)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Simulate work
            import time
            time.sleep(0.1)

        # Check for cancellation
        if goal_handle.is_cancel_requested:
            result_msg.sequence = feedback_msg.sequence
            goal_handle.canceled()
            return result_msg

        # Complete successfully
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    server = ConcurrentFibonacciActionServer()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.executor_pool.shutdown(wait=True)
        server.destroy_node()
        rclpy.shutdown()
```

## Navigation Action Example

A practical example showing how actions are used for robot navigation:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Define a custom navigation action (conceptual)
"""
# NavigateToPose.action
geometry_msgs/PoseStamped target_pose
---
geometry_msgs/PoseStamped final_pose
string message
---
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 progress_percentage
"""

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # In a real implementation, you'd use the actual NavigateToPose action
        # For this example, we'll use the Fibonacci action as a placeholder
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'navigate_to_pose',
            execute_callback=self.execute_navigation_callback,
            goal_callback=self.navigation_goal_callback,
            cancel_callback=self.navigation_cancel_callback
        )

        # Publishers for visualization
        self.path_pub = self.create_publisher(Path, 'current_path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'navigation_goal', 10)

        self.get_logger().info('Navigation action server started')

    def navigation_goal_callback(self, goal_request):
        """Validate navigation goal."""
        # Check if the target pose is valid and reachable
        # This would involve checking against a map, collision detection, etc.
        self.get_logger().info(f'Received navigation goal: {goal_request.order}')
        return GoalResponse.ACCEPT

    def navigation_cancel_callback(self, goal_handle):
        """Handle navigation cancellation."""
        self.get_logger().info('Navigation goal cancelled')
        return CancelResponse.ACCEPT

    async def execute_navigation_callback(self, goal_handle):
        """Execute navigation to the target pose."""
        self.get_logger().info('Starting navigation to target...')

        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Simulate navigation progress
        total_steps = goal_handle.request.order
        current_step = 0

        while current_step < total_steps:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Navigation cancelled by user')
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # Update progress
            current_step += 1
            progress = (current_step / total_steps) * 100

            # Create feedback
            feedback_msg.sequence.append(current_step)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Navigation progress: {progress:.1f}%')

            # Simulate movement
            import time
            time.sleep(1.0)

        # Navigation completed successfully
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()

        self.get_logger().info('Navigation completed successfully')
        return result_msg
```

## Action Client with Timeout and Retry Logic

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from example_interfaces.action import Fibonacci
import time

class RobustActionClient(Node):
    def __init__(self):
        super().__init__('robust_action_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        self.feedback_received = False

    def send_goal_with_timeout(self, order, timeout_seconds=30):
        """Send goal with timeout and return result."""
        self.get_logger().info(f'Waiting for action server (timeout: {timeout_seconds}s)...')

        # Wait for server with timeout
        if not self._action_client.wait_for_server(timeout_sec=timeout_seconds):
            self.get_logger().error('Action server not available')
            return None

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send goal
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Wait for goal response
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_seconds)

        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Failed to get goal handle')
            return None

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_seconds)

        result = result_future.result()
        if result:
            return result.result
        else:
            self.get_logger().error('Failed to get result')
            return None

    def feedback_callback(self, feedback_msg):
        """Handle feedback."""
        self.feedback_received = True
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def send_goal_with_retry(self, order, max_retries=3, timeout_seconds=30):
        """Send goal with retry logic."""
        for attempt in range(max_retries):
            self.get_logger().info(f'Attempt {attempt + 1} of {max_retries}')

            result = self.send_goal_with_timeout(order, timeout_seconds)

            if result is not None:
                self.get_logger().info(f'Success on attempt {attempt + 1}')
                return result
            else:
                self.get_logger().warn(f'Attempt {attempt + 1} failed')

                if attempt < max_retries - 1:  # Don't sleep after the last attempt
                    time.sleep(2)  # Wait before retry

        self.get_logger().error(f'Failed after {max_retries} attempts')
        return None

def main(args=None):
    rclpy.init(args=args)
    client = RobustActionClient()

    # Send goal with retry
    result = client.send_goal_with_retry(10, max_retries=3)

    if result:
        client.get_logger().info(f'Final result: {result.sequence}')
    else:
        client.get_logger().error('Action failed after all retries')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercise: Action-Based Robot Control

Create a complete action-based system for controlling a simulated robot arm.

### Exercise Requirements
1. Create an action server for robot arm movement
2. Implement goal validation and preemption
3. Create an action client with feedback monitoring
4. Demonstrate goal cancellation

### Python Implementation

```python
#!/usr/bin/env python3
"""
Action-Based Robot Control System
Demonstrates actions for robot arm control with feedback and cancellation
"""

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci  # Using Fibonacci as placeholder
import time
import threading
from enum import Enum

class ArmJointState(Enum):
    SHOULDER = 0
    ELBOW = 1
    WRIST = 2

class RobotArmActionServer(Node):
    def __init__(self):
        super().__init__('robot_arm_action_server')

        # Initialize arm joint positions
        self.joint_positions = {ArmJointState.SHOULDER: 0.0,
                               ArmJointState.ELBOW: 0.0,
                               ArmJointState.WRIST: 0.0}

        # Action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'move_arm',
            execute_callback=self.execute_move_arm_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Robot arm action server started')

    def goal_callback(self, goal_request):
        """Validate the move arm goal."""
        self.get_logger().info(f'Received move arm goal: order={goal_request.order}')

        # Validate that the goal is reasonable
        # In a real system, this would check joint limits, collisions, etc.
        if 1 <= goal_request.order <= 10:  # Reasonable range for this example
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f'Goal order {goal_request.order} is invalid')
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation."""
        self.get_logger().info('Move arm goal cancellation requested')
        return CancelResponse.ACCEPT

    async def execute_move_arm_callback(self, goal_handle):
        """Execute the robot arm movement."""
        self.get_logger().info('Starting robot arm movement...')

        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Simulate arm movement with multiple steps
        total_steps = goal_handle.request.order
        current_step = 0

        while current_step < total_steps:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Robot arm movement cancelled')
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # Simulate arm movement - update joint positions gradually
            for joint in ArmJointState:
                self.joint_positions[joint] += 0.1  # Increment joint position

            # Update feedback
            current_step += 1
            feedback_msg.sequence.append(current_step)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Log current state
            self.get_logger().info(
                f'Movement step {current_step}/{total_steps}, '
                f'Joint positions: {self.joint_positions}'
            )

            # Simulate time for physical movement
            time.sleep(0.5)

        # Check for cancellation one more time
        if goal_handle.is_cancel_requested:
            result_msg.sequence = feedback_msg.sequence
            goal_handle.canceled()
            return result_msg

        # Movement completed successfully
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()

        self.get_logger().info(f'Robot arm movement completed: {result_msg.sequence}')
        return result_msg

class RobotArmActionClient(Node):
    def __init__(self):
        super().__init__('robot_arm_action_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'move_arm'
        )

        self.current_feedback = None

    def send_arm_movement_goal(self, steps, send_cancel_after=None):
        """Send a goal to move the robot arm."""
        self.get_logger().info('Waiting for robot arm action server...')

        # Wait for server
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = steps

        # Send goal with feedback callback
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)
        goal_handle_future = future

        # Get the goal handle
        rclpy.spin_until_future_complete(self, goal_handle_future)
        goal_handle = goal_handle_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')

        # Optionally send cancel request after some time
        if send_cancel_after:
            def cancel_after_delay():
                time.sleep(send_cancel_after)
                self.get_logger().info('Sending cancel request...')
                cancel_future = goal_handle.cancel_goal_async()
                # Don't wait for cancel result to avoid blocking

            cancel_thread = threading.Thread(target=cancel_after_delay)
            cancel_thread.start()

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result:
            self.get_logger().info(f'Arm movement result: {result.result.sequence}')
            return result.result
        else:
            self.get_logger().error('Failed to get result')
            return None

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        self.get_logger().info(f'Goal response received: {goal_handle is not None}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the arm movement."""
        self.current_feedback = feedback_msg.feedback
        self.get_logger().info(f'Arm movement feedback: step {len(feedback_msg.feedback.sequence)}')

def run_robot_arm_demo():
    """Run the complete robot arm action demo."""
    rclpy.init()

    print("Starting Robot Arm Action Demo")
    print("=" * 40)

    # Create server and client nodes
    server = RobotArmActionServer()
    client = RobotArmActionClient()

    # Use multi-threaded executor to run both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)

    # Run nodes in background
    def spin_executor():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    import threading
    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    time.sleep(1)  # Give nodes time to start

    print("\n1. Testing successful arm movement:")
    result1 = client.send_arm_movement_goal(5)  # Should complete successfully
    print(f"Result 1: {'Success' if result1 else 'Failed'}")

    time.sleep(2)

    print("\n2. Testing arm movement cancellation:")
    result2 = client.send_arm_movement_goal(10, send_cancel_after=2.0)  # Should be cancelled
    print(f"Result 2: {'Success' if result2 else 'Cancelled/Failure'}")

    time.sleep(2)

    print("\n3. Testing invalid goal (should be rejected):")
    # Wait for server again since previous call might have ended
    client._action_client.wait_for_server()

    # Create and send invalid goal
    invalid_goal_msg = Fibonacci.Goal()
    invalid_goal_msg.order = 15  # Outside valid range [1, 10]

    future = client._action_client.send_goal_async(invalid_goal_msg)
    rclpy.spin_until_future_complete(client, future)

    goal_handle = future.result()
    if goal_handle:
        print(f"Invalid goal accepted: {goal_handle.accepted}")
    else:
        print("Failed to get goal handle for invalid goal")

    print("\nDemo completed!")
    server.destroy_node()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_robot_arm_demo()
```

## Troubleshooting Common Action Issues

### 1. Action Server Not Responding

```python
# Problem: Action server doesn't respond to goals
# Solution: Check server registration and callback execution

def verify_action_server_status(node, action_name):
    """Check if action server is properly registered."""
    # Get list of action servers
    action_servers = node.get_action_server_names_and_types()

    for name, types in action_servers:
        if name == action_name:
            print(f"Action server '{name}' found with types: {types}")
            return True

    print(f"Action server '{action_name}' not found")
    return False
```

### 2. Callback Execution Issues

```python
# Problem: Callbacks not executing properly
# Solution: Use appropriate callback groups

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ProperActionServer(Node):
    def __init__(self):
        super().__init__('proper_action_server')

        # Use mutually exclusive callback group if callbacks should not run concurrently
        cb_group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'proper_fibonacci',
            execute_callback=self.execute_callback,
            callback_group=cb_group
        )
```

### 3. Memory Management in Long-Running Actions

```python
# Problem: Memory leaks in long-running actions
# Solution: Proper resource management

import weakref

class MemoryManagedActionServer(Node):
    def __init__(self):
        super().__init__('memory_managed_server')

        self.active_goals = {}  # Keep track of active goals

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'memory_managed_fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """Track the goal."""
        return rclpy.action.server.GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute with proper cleanup."""
        goal_id = goal_handle.goal_id.uuid

        try:
            # Add to active goals
            self.active_goals[goal_id] = weakref.ref(goal_handle)

            # Perform the action work here
            result = await self.perform_action_work(goal_handle)
            return result
        finally:
            # Clean up reference
            if goal_id in self.active_goals:
                del self.active_goals[goal_id]

    async def perform_action_work(self, goal_handle):
        """Perform the actual action work."""
        # Implementation here
        result_msg = Fibonacci.Result()
        result_msg.sequence = [1, 1, 2, 3, 5]  # Example result
        goal_handle.succeed()
        return result_msg
```

## Resources for Further Learning

- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Actions-In-Python.html)
- [Action Design Guidelines](https://design.ros2.org/articles/actions.html)
- [Navigation2 Action Examples](https://navigation.ros.org/command_line_tools/docs/nav2_cli.html)
- "Programming Robots with ROS" by Morgan Quigley et al.

## Summary

Actions in ROS 2 provide a powerful communication pattern for long-running, goal-oriented tasks that require feedback and cancellation capabilities. They are essential for implementing complex robotic behaviors such as navigation, manipulation, and calibration. Understanding how to properly implement action servers and clients, handle feedback, and manage goal lifecycle is crucial for building responsive and robust robotic systems.