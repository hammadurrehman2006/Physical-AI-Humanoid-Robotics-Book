---
title: "Python-rclpy Bridge: Connecting Python to ROS 2"
description: Understanding how to use rclpy to connect Python applications to ROS 2
sidebar_position: 5
---

# Python-rclpy Bridge: Connecting Python to ROS 2

## Learning Objectives
- Understand the rclpy client library architecture
- Master the creation of ROS 2 nodes in Python
- Implement publishers, subscribers, services, and actions using rclpy
- Learn about parameter management and node lifecycle
- Create efficient Python-based ROS 2 applications

## Introduction to rclpy

rclpy is the Python client library for ROS 2 that provides the interface between Python applications and the ROS 2 middleware. It allows Python developers to create ROS 2 nodes, publishers, subscribers, services, and actions with a Pythonic API.

### rclpy Architecture

The rclpy library sits between your Python application and the underlying ROS 2 client library (rcl), which interfaces with the DDS middleware:

```
+------------------+    +----------+    +----------+    +------------+
| Python App     | -> | rclpy    | -> | rcl      | -> | DDS Impl   |
| (User Code)    |    | (Python) |    | (C)      |    | (FastDDS,  |
|                |    |          |    |          |    | CycloneDDS)|
+------------------+    +----------+    +----------+    +------------+
```

### Basic rclpy Concepts

```python
import rclpy
from rclpy.node import Node

class BasicRclpyNode(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('basic_rclpy_node')

        # Log messages using the node's logger
        self.get_logger().info('Basic rclpy node initialized')

        # Get the node's clock
        current_time = self.get_clock().now()
        self.get_logger().info(f'Current time: {current_time}')

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create the node
    node = BasicRclpyNode()

    try:
        # Spin the node to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Creation and Management

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Call the parent constructor with node name
        super().__init__('my_node')

        # Optional: specify namespace
        # super().__init__('my_node', namespace='robot1')

        # Node initialization code here
        self.get_logger().info('MyNode initialized')

    def __del__(self):
        # Cleanup code (though destroy_node() should be called explicitly)
        self.get_logger().info('MyNode destroyed')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Explicit cleanup
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Parameters

Parameters allow nodes to be configured at runtime:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values and types
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('use_camera', True)
        self.declare_parameter('sensor_topics', ['camera/image_raw', 'depth/image_raw'])

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.use_camera = self.get_parameter('use_camera').value
        self.sensor_topics = self.get_parameter('sensor_topics').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Use camera: {self.use_camera}')
        self.get_logger().info(f'Sensor topics: {self.sensor_topics}')

        # Set a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Handle parameter changes."""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Import needed for parameter callback
from rclpy.parameter_service import SetParametersResult
```

## Publishers and Subscribers

### Creating Publishers

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create publishers with different QoS settings
        self.string_pub = self.create_publisher(
            String,
            'string_topic',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.int_pub = self.create_publisher(
            Int32,
            'int_topic',
            10  # Using integer for simple QoS (depth=10)
        )

        self.float_pub = self.create_publisher(
            Float64,
            'sensor_data',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Create a timer to publish messages periodically
        self.counter = 0
        self.timer = self.create_timer(0.5, self.publish_messages)

        self.get_logger().info('Publisher node initialized')

    def publish_messages(self):
        """Publish messages to different topics."""
        # Publish string message
        string_msg = String()
        string_msg.data = f'Hello from Python: {self.counter}'
        self.string_pub.publish(string_msg)

        # Publish integer message
        int_msg = Int32()
        int_msg.data = self.counter
        self.int_pub.publish(int_msg)

        # Publish float message
        float_msg = Float64()
        float_msg.data = 3.14 * self.counter
        self.float_pub.publish(float_msg)

        self.counter += 1
        self.get_logger().info(f'Published messages with counter: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Creating Subscribers

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscribers for different message types
        self.string_sub = self.create_subscription(
            String,
            'string_topic',
            self.string_callback,
            10
        )

        self.int_sub = self.create_subscription(
            Int32,
            'int_topic',
            self.int_callback,
            10
        )

        self.float_sub = self.create_subscription(
            Float64,
            'sensor_data',
            self.float_callback,
            10
        )

        # Prevent unused variable warnings
        self.string_sub  # This keeps the subscription alive
        self.int_sub
        self.float_sub

        self.get_logger().info('Subscriber node initialized')

    def string_callback(self, msg):
        """Handle string messages."""
        self.get_logger().info(f'String message received: {msg.data}')

    def int_callback(self, msg):
        """Handle integer messages."""
        self.get_logger().info(f'Integer message received: {msg.data}')

    def float_callback(self, msg):
        """Handle float messages."""
        self.get_logger().info(f'Float message received: {msg.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Services and Actions with rclpy

### Creating Services

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, Trigger
from std_msgs.msg import String

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # Create a simple service
        self.add_srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        # Create a trigger service
        self.trigger_srv = self.create_service(
            Trigger,
            'robot_trigger',
            self.trigger_callback
        )

        # Create a custom service-like functionality using topics
        self.command_sub = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        self.get_logger().info('Service node initialized')

    def add_callback(self, request, response):
        """Handle add two ints service request."""
        response.sum = request.a + request.b
        self.get_logger().info(f'Calculated {request.a} + {request.b} = {response.sum}')
        return response

    def trigger_callback(self, request, response):
        """Handle trigger service request."""
        # Simulate some work
        import time
        time.sleep(0.1)

        response.success = True
        response.message = f'Trigger activated at {self.get_clock().now().to_msg()}'
        self.get_logger().info(f'Trigger service called: {response.message}')
        return response

    def command_callback(self, msg):
        """Handle command topic (alternative to services)."""
        self.get_logger().info(f'Command received: {msg.data}')
        # Process the command here

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Creating Action Servers with rclpy

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server_node')

        # Create action server with reentrant callback group
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci_action',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Action server node initialized')

    def goal_callback(self, goal_request):
        """Accept or reject goal requests."""
        self.get_logger().info(f'Received goal: {goal_request.order}')

        # Accept goals with reasonable order values
        if 1 <= goal_request.order <= 20:
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f'Rejected goal with order: {goal_request.order}')
            return rclpy.action.server.GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests."""
        self.get_logger().info('Goal cancellation requested')
        return rclpy.action.server.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the action goal."""
        self.get_logger().info('Executing goal...')

        # Initialize feedback and result
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Calculate Fibonacci sequence
        order = goal_handle.request.order
        sequence = [0, 1]

        # Send initial feedback if order > 2
        if order > 2:
            for i in range(2, order):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled during execution')
                    result_msg.sequence = sequence
                    goal_handle.canceled()
                    return result_msg

                # Calculate next Fibonacci number
                next_num = sequence[i-1] + sequence[i-2]
                sequence.append(next_num)

                # Send feedback
                feedback_msg.sequence = sequence.copy()
                goal_handle.publish_feedback(feedback_msg)

                # Simulate work
                import time
                time.sleep(0.2)

        # Check for cancellation before completing
        if goal_handle.is_cancel_requested:
            result_msg.sequence = sequence
            goal_handle.canceled()
            return result_msg

        # Complete successfully
        result_msg.sequence = sequence
        goal_handle.succeed()
        self.get_logger().info(f'Goal completed successfully: {result_msg.sequence}')

        return result_msg

def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced rclpy Features

### Timers and Callbacks

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'timer_messages', 10)

        # Create different types of timers
        self.rate_timer = self.create_timer(1.0, self.rate_callback)  # 1 Hz
        self.fast_timer = self.create_timer(0.1, self.fast_callback)  # 10 Hz
        self.slow_timer = self.create_timer(5.0, self.slow_callback)  # 0.2 Hz

        # Counter for messages
        self.counters = {'rate': 0, 'fast': 0, 'slow': 0}

        self.get_logger().info('Timer node initialized')

    def rate_callback(self):
        """Called at 1 Hz."""
        self.counters['rate'] += 1
        msg = String()
        msg.data = f'1Hz message #{self.counters["rate"]}'
        self.publisher.publish(msg)
        self.get_logger().info(f'1Hz: {msg.data}')

    def fast_callback(self):
        """Called at 10 Hz."""
        self.counters['fast'] += 1
        self.get_logger().info(f'10Hz tick #{self.counters["fast"]}')

    def slow_callback(self):
        """Called at 0.2 Hz."""
        self.counters['slow'] += 1
        self.get_logger().info(f'0.2Hz tick #{self.counters["slow"]}')

    def destroy_node(self):
        """Clean up timers."""
        # Timers are automatically destroyed when node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Client Libraries and Advanced Patterns

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from collections import deque

class AdvancedRobotNode(Node):
    def __init__(self):
        super().__init__('advanced_robot_node')

        # Publishers for different robot systems
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Store sensor data
        self.scan_data = None
        self.recent_commands = deque(maxlen=10)

        # Create a timer for robot control
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Advanced robot node initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        self.scan_data = msg
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} ranges')

    def control_loop(self):
        """Main robot control loop."""
        if self.scan_data is None:
            return

        # Simple obstacle avoidance
        safe_distance = 1.0
        min_range = min([r for r in self.scan_data.ranges if not math.isinf(r) and not math.isnan(r)], default=float('inf'))

        cmd_vel = Twist()

        if min_range < safe_distance:
            # Obstacle detected, turn away
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
            status = "Obstacle detected, turning"
        else:
            # Clear path, move forward
            cmd_vel.linear.x = 0.5  # Move forward
            cmd_vel.angular.z = 0.0
            status = "Moving forward"

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        self.recent_commands.append(cmd_vel)

        # Publish status
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Control: {status}, Lin: {cmd_vel.linear.x:.2f}, Ang: {cmd_vel.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Hands-on Exercise: Python-rclpy Integration

Create a comprehensive example that demonstrates various rclpy features working together.

### Exercise Requirements
1. Create a node with parameters, publishers, and subscribers
2. Implement service and action functionality
3. Demonstrate proper resource management
4. Show error handling and logging

### Python Implementation

```python
#!/usr/bin/env python3
"""
Comprehensive rclpy Integration Example
Demonstrates various rclpy features in a single application
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci

import time
import random
from enum import Enum
from collections import deque

class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    PROCESSING = 2
    ERROR = 3

class ComprehensiveRclpyNode(Node):
    def __init__(self):
        super().__init__('comprehensive_rclpy_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'rclpy_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_frequency', 10)
        self.declare_parameter('log_level', 'info')

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.sensor_frequency = self.get_parameter('sensor_frequency').value

        self.get_logger().info(f'Initialized {self.robot_name} with max speed {self.max_speed}')

        # Publishers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.sensor_pub = self.create_publisher(Float64, 'sensor_data', 10)
        self.log_pub = self.create_publisher(String, 'system_log', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'robot_commands', self.command_callback, 10)

        # Services
        self.calc_srv = self.create_service(
            AddTwoInts, 'calculate_sum', self.calculate_callback)

        # Action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'compute_fibonacci',
            execute_callback=self.fibonacci_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.fibonacci_goal_callback
        )

        # Internal state
        self.current_state = RobotState.IDLE
        self.sensor_timer = self.create_timer(
            1.0 / self.sensor_frequency, self.sensor_update)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Data storage
        self.sensor_history = deque(maxlen=100)
        self.command_history = deque(maxlen=50)

        self.get_logger().info('Comprehensive rclpy node initialized successfully')

    def command_callback(self, msg):
        """Handle robot commands."""
        command = msg.data.lower()
        self.command_history.append(command)

        self.get_logger().info(f'Received command: {command}')

        if command == 'start':
            self.current_state = RobotState.MOVING
            self.log_event(f'Start command received, state: {self.current_state.name}')
        elif command == 'stop':
            self.current_state = RobotState.IDLE
            self.log_event(f'Stop command received, state: {self.current_state.name}')
        elif command.startswith('speed'):
            try:
                parts = command.split()
                if len(parts) >= 2:
                    new_speed = float(parts[1])
                    if 0 <= new_speed <= self.max_speed * 2:  # Allow some overspeed
                        self.max_speed = new_speed
                        self.log_event(f'Speed changed to {new_speed}')
                    else:
                        self.get_logger().warn(f'Invalid speed: {new_speed}')
            except ValueError:
                self.get_logger().warn(f'Invalid speed command: {command}')

    def calculate_callback(self, request, response):
        """Handle calculation service request."""
        try:
            result = request.a + request.b
            response.sum = result
            self.log_event(f'Calculated {request.a} + {request.b} = {result}')
            self.get_logger().info(f'Service calculation: {request.a} + {request.b} = {result}')
        except Exception as e:
            self.get_logger().error(f'Calculation error: {e}')
            response.sum = 0  # Default value on error

        return response

    def fibonacci_goal_callback(self, goal_request):
        """Handle Fibonacci goal request."""
        self.get_logger().info(f'Fibonacci goal requested: order={goal_request.order}')

        # Validate goal
        if 1 <= goal_request.order <= 30:  # Reasonable limit
            return rclpy.action.server.GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f'Invalid Fibonacci order: {goal_request.order}')
            return rclpy.action.server.GoalResponse.REJECT

    async def fibonacci_callback(self, goal_handle):
        """Execute Fibonacci action."""
        self.get_logger().info('Executing Fibonacci action...')

        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        order = goal_handle.request.order
        sequence = []

        # Calculate Fibonacci sequence
        for i in range(order):
            try:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Fibonacci action cancelled')
                    result_msg.sequence = sequence
                    goal_handle.canceled()
                    return result_msg

                if i == 0:
                    sequence.append(0)
                elif i == 1:
                    sequence.append(1)
                else:
                    sequence.append(sequence[i-1] + sequence[i-2])

                # Publish feedback every few steps
                if i % 2 == 0 or i == order - 1:
                    feedback_msg.sequence = sequence.copy()
                    goal_handle.publish_feedback(feedback_msg)

                # Simulate computation time
                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f'Fibonacci calculation error at step {i}: {e}')
                result_msg.sequence = sequence
                goal_handle.abort()
                return result_msg

        # Complete successfully
        result_msg.sequence = sequence
        goal_handle.succeed()
        self.get_logger().info(f'Fibonacci completed: {sequence}')

        return result_msg

    def sensor_update(self):
        """Simulate sensor data updates."""
        try:
            # Generate simulated sensor data
            sensor_value = random.uniform(0.0, 10.0)

            # Publish sensor data
            sensor_msg = Float64()
            sensor_msg.data = sensor_value
            self.sensor_pub.publish(sensor_msg)

            # Store in history
            self.sensor_history.append(sensor_value)

            # Log if value is unusual
            if sensor_value > 8.0:
                self.log_event(f'High sensor reading: {sensor_value:.2f}')

        except Exception as e:
            self.get_logger().error(f'Sensor update error: {e}')
            self.current_state = RobotState.ERROR

    def publish_status(self):
        """Publish robot status."""
        try:
            status_msg = String()
            status_msg.data = f'{self.robot_name}: state={self.current_state.name}, speed={self.max_speed:.2f}, sensors={len(self.sensor_history)}'
            self.status_pub.publish(status_msg)

            # Update state based on conditions
            if self.current_state == RobotState.MOVING:
                # Simulate movement completion
                if random.random() < 0.1:  # 10% chance to return to idle
                    self.current_state = RobotState.IDLE
                    self.log_event('Movement completed')

        except Exception as e:
            self.get_logger().error(f'Status update error: {e}')

    def log_event(self, message):
        """Log an event to the system log topic."""
        try:
            log_msg = String()
            log_msg.data = f'[{self.get_clock().now().seconds_nanoseconds()}] {message}'
            self.log_pub.publish(log_msg)
        except Exception as e:
            self.get_logger().error(f'Log publication error: {e}')

def run_comprehensive_demo():
    """Run the comprehensive rclpy demo."""
    rclpy.init()

    print("Starting Comprehensive rclpy Demo")
    print("=" * 40)

    # Create the node
    node = ComprehensiveRclpyNode()

    # Create a client to test services (in a separate thread)
    import threading

    def test_services():
        """Test services from a separate thread."""
        time.sleep(2)  # Wait for node to initialize

        # Create a temporary node for client operations
        rclpy_client = rclpy.create_node('test_client')

        # Test service
        cli = rclpy_client.create_client(AddTwoInts, 'calculate_sum')
        while not cli.wait_for_service(timeout_sec=1.0):
            rclpy_client.get_logger().info('Service not available, waiting...')

        # Send request
        req = AddTwoInts.Request()
        req.a = 5
        req.b = 7
        future = cli.call_async(req)

        try:
            rclpy.spin_until_future_complete(rclpy_client, future, timeout_sec=5.0)
            response = future.result()
            if response:
                print(f"Service test result: 5 + 7 = {response.sum}")
        except Exception as e:
            print(f"Service call failed: {e}")

        # Send some commands via topic
        cmd_pub = rclpy_client.create_publisher(String, 'robot_commands', 10)
        time.sleep(1)

        commands = ['start', 'speed 2.5', 'stop']
        for cmd in commands:
            msg = String()
            msg.data = cmd
            cmd_pub.publish(msg)
            print(f"Sent command: {cmd}")
            time.sleep(1)

        rclpy_client.destroy_node()

    # Start service testing in background
    test_thread = threading.Thread(target=test_services, daemon=True)
    test_thread.start()

    try:
        # Use multi-threaded executor to handle all callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        print("Running comprehensive demo... Press Ctrl+C to stop")
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutting down comprehensive demo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_comprehensive_demo()
```

## Troubleshooting Common rclpy Issues

### 1. Import and Installation Issues

```python
# Problem: rclpy not found or import errors
# Solution: Verify installation and Python environment

def check_rclpy_installation():
    """Check if rclpy is properly installed."""
    try:
        import rclpy
        print(f"✓ rclpy version: {rclpy.__version__}")
        print(f"✓ rclpy location: {rclpy.__file__}")
        return True
    except ImportError as e:
        print(f"✗ rclpy import failed: {e}")
        print("Make sure ROS 2 is sourced and rclpy is installed")
        print("Source ROS 2: source /opt/ros/humble/setup.bash")
        return False

def check_ros_environment():
    """Check ROS 2 environment variables."""
    import os

    ros_distro = os.environ.get('ROS_DISTRO')
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', 'Not set')

    print(f"ROS_DISTRO: {ros_distro}")
    print(f"ROS_DOMAIN_ID: {ros_domain_id}")

    if not ros_distro or ros_distro != 'humble':
        print("Warning: ROS_DISTRO is not set to 'humble'")

    return bool(ros_distro)
```

### 2. Node Lifecycle and Resource Management

```python
# Problem: Memory leaks or resource not cleaned up
# Solution: Proper node destruction

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')

        # Keep references to all created objects
        self.publishers = []
        self.subscribers = []
        self.services = []
        self.action_servers = []
        self.timers = []

        # Create resources
        self._create_resources()

    def _create_resources(self):
        """Create all necessary resources."""
        # Publishers
        pub = self.create_publisher(String, 'test_topic', 10)
        self.publishers.append(pub)

        # Subscribers
        sub = self.create_subscription(String, 'test_topic', lambda msg: None, 10)
        self.subscribers.append(sub)

        # Timers
        timer = self.create_timer(1.0, lambda: None)
        self.timers.append(timer)

    def destroy_node(self):
        """Properly clean up all resources."""
        self.get_logger().info('Cleaning up resources...')

        # Destroy timers first
        for timer in self.timers:
            timer.destroy()

        # Destroy subscribers
        for sub in self.subscribers:
            sub.destroy()

        # Destroy publishers
        for pub in self.publishers:
            pub.destroy()

        # Clear references
        self.timers.clear()
        self.subscribers.clear()
        self.publishers.clear()

        # Call parent destroy
        super().destroy_node()
        self.get_logger().info('Node destroyed successfully')
```

### 3. Threading and Concurrency Issues

```python
# Problem: Threading issues with rclpy
# Solution: Use proper executors and callback groups

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ThreadSafeNode(Node):
    def __init__(self):
        super().__init__('thread_safe_node')

        # Use reentrant callback group for callbacks that might call other callbacks
        reentrant_group = ReentrantCallbackGroup()

        # Use mutually exclusive group for callbacks that shouldn't run concurrently
        exclusive_group = MutuallyExclusiveCallbackGroup()

        # Create action server with reentrant group
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'safe_fibonacci',
            execute_callback=self.safe_execute_callback,
            callback_group=reentrant_group
        )

        # Create service with exclusive group
        self._service = self.create_service(
            AddTwoInts,
            'safe_service',
            self.safe_service_callback,
            callback_group=exclusive_group
        )

    def safe_execute_callback(self, goal_handle):
        """Thread-safe action execution."""
        # Implementation here
        result_msg = Fibonacci.Result()
        result_msg.sequence = [1, 1, 2, 3, 5]  # Example
        goal_handle.succeed()
        return result_msg

    def safe_service_callback(self, request, response):
        """Thread-safe service callback."""
        response.sum = request.a + request.b
        return response
```

## Performance Optimization Tips

### Efficient Message Handling

```python
# Optimize message handling for high-frequency topics
class OptimizedNode(Node):
    def __init__(self):
        super().__init__('optimized_node')

        # Use appropriate QoS for high-frequency topics
        high_freq_qos = QoSProfile(
            depth=1,  # Keep only latest message
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.high_freq_sub = self.create_subscription(
            String, 'high_freq_topic', self.high_freq_callback, high_freq_qos)

    def high_freq_callback(self, msg):
        """Optimized callback for high-frequency messages."""
        # Process message efficiently
        # Avoid heavy computations in high-frequency callbacks
        pass
```

## Resources for Further Learning

- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Parameter Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-Python.html)
- [ROS 2 Quality of Service Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## Summary

The rclpy client library provides a comprehensive Python interface to ROS 2, enabling Python developers to create sophisticated robotic applications. Understanding node creation, parameter management, publisher/subscriber patterns, and service/action implementations is crucial for building effective ROS 2 systems. Proper resource management, error handling, and performance optimization ensure robust and efficient Python-based ROS 2 applications.