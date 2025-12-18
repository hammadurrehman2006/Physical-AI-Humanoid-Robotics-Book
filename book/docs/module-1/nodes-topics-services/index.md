---
title: ROS 2 Nodes, Topics, and Services
description: Understanding the fundamental communication patterns in ROS 2
sidebar_position: 3
---

# ROS 2 Nodes, Topics, and Services

## Learning Objectives
- Master the creation and management of ROS 2 nodes
- Understand the publisher-subscriber communication pattern
- Implement services for request-response communication
- Learn about message types and custom message definitions
- Create robust communication systems between nodes

## Introduction to ROS 2 Communication

ROS 2 provides three primary communication patterns that form the backbone of robotic applications:

1. **Nodes**: Execution units that perform computations
2. **Topics**: Asynchronous, many-to-many communication via publisher-subscriber pattern
3. **Services**: Synchronous request-response communication

These patterns work together to create distributed robotic systems where different components can communicate efficiently.

## Understanding ROS 2 Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node is a separate process that can contain publishers, subscribers, services, and other ROS entities.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class BasicNodeExample(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('basic_node_example')

        # Log a message
        self.get_logger().info('Basic node has been initialized')

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = BasicNodeExample()

    try:
        # Keep the node running
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

### Node Parameters

Nodes can accept parameters for configuration:

```python
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('update_rate', 10)
        self.declare_parameter('safety_enabled', True)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.update_rate = self.get_parameter('update_rate').value
        self.safety_enabled = self.get_parameter('safety_enabled').value

        self.get_logger().info(
            f'Initialized with: name={self.robot_name}, '
            f'rate={self.update_rate}Hz, safety={self.safety_enabled}'
        )

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
```

## Publisher-Subscriber Pattern (Topics)

The publisher-subscriber pattern enables asynchronous communication between nodes. Publishers send messages to topics, and subscribers receive messages from topics.

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a timer to publish messages periodically
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds
        self.i = 0

        self.get_logger().info('Talker node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i} at {time.time()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()
```

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10  # Queue size
        )
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()
```

### Complex Message Types

ROS 2 provides various message types for different data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Header
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan, Image
import numpy as np

class ComplexMessageNode(Node):
    def __init__(self):
        super().__init__('complex_message_node')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher for sensor data
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Publisher for multi-dimensional data
        self.array_pub = self.create_publisher(Float64MultiArray, 'joint_positions', 10)

        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_data)

        self.get_logger().info('Complex message node initialized')

    def publish_data(self):
        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        twist_msg.angular.z = 0.2  # Turn left at 0.2 rad/s
        self.cmd_vel_pub.publish(twist_msg)

        # Publish laser scan data
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = -np.pi / 2  # -90 degrees
        scan_msg.angle_max = np.pi / 2   # 90 degrees
        scan_msg.angle_increment = np.pi / 180  # 1 degree
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Simulate some range data
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [2.0 + 0.5 * np.sin(i * 0.1) for i in range(num_readings)]

        self.scan_pub.publish(scan_msg)

        # Publish joint positions
        array_msg = Float64MultiArray()
        array_msg.data = [0.1, 0.2, 0.3, 0.4, 0.5]  # Example joint angles
        self.array_pub.publish(array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ComplexMessageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Services - Request-Response Communication

Services provide synchronous communication where a client sends a request and waits for a response.

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server started')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service = MinimalService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()
```

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create a client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Asynchronously call the service
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    client = MinimalClient()

    # Send request
    future = client.send_request(1, 2)

    try:
        # Wait for response
        rclpy.spin_until_future_complete(client, future)
        response = future.result()
        client.get_logger().info(f'Result: {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()
```

## Custom Message Definitions

You can define custom message types for your specific application needs:

### Creating Custom Messages

First, create a `msg` directory in your package and define a custom message:

```# JointState.msg
# Custom message for joint state information
string name
float64 position
float64 velocity
float64 effort
time timestamp
```

Then use it in your nodes:

```python
import rclpy
from rclpy.node import Node
# Assuming you have a custom message package
# from your_robot_msgs.msg import JointState

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')

        # Create publisher for custom message
        # self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # For this example, we'll use a standard message
        self.data_pub = self.create_publisher(String, 'custom_data', 10)

        self.timer = self.create_timer(1.0, self.publish_custom_data)

    def publish_custom_data(self):
        # Simulate creating custom message data
        msg = String()
        msg.data = f'Custom data at {self.get_clock().now().to_msg()}'
        self.data_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomMessageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Communication Patterns

### Multiple Publishers and Subscribers

```python
class MultiCommunicationNode(Node):
    def __init__(self):
        super().__init__('multi_comm_node')

        # Multiple publishers
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_odom = self.create_publisher(String, 'odometry', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)

        # Multiple subscribers
        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel_input', self.cmd_callback, 10)
        self.sub_sensor = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)
        self.sub_control = self.create_subscription(
            String, 'control_commands', self.control_callback, 10)

        self.get_logger().info('Multi-communication node initialized')

    def cmd_callback(self, msg):
        self.get_logger().info(f'Received command: linear={msg.linear.x}, angular={msg.angular.z}')
        # Process command and possibly publish to other topics

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def control_callback(self, msg):
        self.get_logger().info(f'Received control command: {msg.data}')
```

### Publisher with Conditional Logic

```python
class ConditionalPublisherNode(Node):
    def __init__(self):
        super().__init__('conditional_publisher')

        self.pub = self.create_publisher(String, 'conditional_topic', 10)
        self.sub = self.create_subscription(String, 'trigger_topic', self.trigger_callback, 10)

        self.publishing_enabled = False
        self.publish_timer = self.create_timer(1.0, self.publish_if_enabled)

    def trigger_callback(self, msg):
        if msg.data == 'enable':
            self.publishing_enabled = True
            self.get_logger().info('Publishing enabled')
        elif msg.data == 'disable':
            self.publishing_enabled = False
            self.get_logger().info('Publishing disabled')

    def publish_if_enabled(self):
        if self.publishing_enabled:
            msg = String()
            msg.data = f'Conditional message at {self.get_clock().now().to_msg()}'
            self.pub.publish(msg)
```

## Hands-on Exercise: Communication System

Create a complete communication system with multiple nodes that demonstrate publisher-subscriber and service patterns.

### Exercise Requirements
1. Create a sensor node that publishes sensor data
2. Create a processing node that subscribes to sensor data and processes it
3. Create a service node that provides data analysis
4. Create a client node that uses the analysis service

### Python Implementation

```python
#!/usr/bin/env python3
"""
ROS 2 Communication System Exercise
Demonstrates publisher-subscriber and service patterns
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from example_interfaces.srv import Trigger
import random
import time
from collections import deque

class SensorNode(Node):
    """Node that simulates sensor data publishing"""
    def __init__(self):
        super().__init__('sensor_node')

        # Publisher for sensor data
        self.sensor_pub = self.create_publisher(Float32, 'sensor_data', 10)

        # Timer to publish sensor data periodically
        self.timer = self.create_timer(0.2, self.publish_sensor_data)

        self.get_logger().info('Sensor node started')

    def publish_sensor_data(self):
        # Simulate sensor reading with some noise
        msg = Float32()
        msg.data = random.uniform(0.0, 100.0)  # Simulate sensor value
        self.sensor_pub.publish(msg)
        self.get_logger().info(f'Published sensor data: {msg.data:.2f}')

class ProcessingNode(Node):
    """Node that processes sensor data and publishes results"""
    def __init__(self):
        super().__init__('processing_node')

        # Subscription to sensor data
        self.sensor_sub = self.create_subscription(
            Float32, 'sensor_data', self.sensor_callback, 10)

        # Publisher for processed data
        self.processed_pub = self.create_publisher(String, 'processed_data', 10)

        # Store recent values for averaging
        self.recent_values = deque(maxlen=5)

        self.get_logger().info('Processing node started')

    def sensor_callback(self, msg):
        # Add to recent values for averaging
        self.recent_values.append(msg.data)

        # Calculate average of recent values
        avg_value = sum(self.recent_values) / len(self.recent_values)

        # Create processed data message
        processed_msg = String()
        processed_msg.data = f'Raw: {msg.data:.2f}, Avg: {avg_value:.2f}, Count: {len(self.recent_values)}'

        self.processed_pub.publish(processed_msg)
        self.get_logger().info(f'Processed: {processed_msg.data}')

class AnalysisServiceNode(Node):
    """Node that provides data analysis service"""
    def __init__(self):
        super().__init__('analysis_service')

        # Create service
        self.srv = self.create_service(
            Trigger, 'analyze_data', self.analyze_callback)

        # Store historical data
        self.historical_data = deque(maxlen=100)

        # Subscription to processed data
        self.data_sub = self.create_subscription(
            String, 'processed_data', self.data_callback, 10)

        self.get_logger().info('Analysis service started')

    def data_callback(self, msg):
        """Store processed data for analysis"""
        try:
            # Extract numerical values from the processed message
            parts = msg.data.split(', ')
            raw_val = float(parts[0].split(': ')[1])
            self.historical_data.append(raw_val)
        except (ValueError, IndexError):
            self.get_logger().warn(f'Could not parse data: {msg.data}')

    def analyze_callback(self, request, response):
        """Provide analysis of stored data"""
        if not self.historical_data:
            response.success = False
            response.message = 'No data available for analysis'
            return response

        # Perform analysis
        avg = sum(self.historical_data) / len(self.historical_data)
        min_val = min(self.historical_data)
        max_val = max(self.historical_data)
        count = len(self.historical_data)

        response.success = True
        response.message = f'Analysis - Avg: {avg:.2f}, Min: {min_val:.2f}, Max: {max_val:.2f}, Count: {count}'

        self.get_logger().info(f'Analysis provided: {response.message}')
        return response

class ClientNode(Node):
    """Node that uses the analysis service"""
    def __init__(self):
        super().__init__('client_node')

        # Create subscription to processed data to monitor
        self.monitor_sub = self.create_subscription(
            String, 'processed_data', self.monitor_callback, 10)

        # Create service client
        self.cli = self.create_client(Trigger, 'analyze_data')

        # Timer to periodically request analysis
        self.timer = self.create_timer(5.0, self.request_analysis)

        self.get_logger().info('Client node started')

    def monitor_callback(self, msg):
        """Monitor processed data"""
        self.get_logger().info(f'Monitoring: {msg.data}')

    def request_analysis(self):
        """Request data analysis from service"""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Analysis service not available, waiting...')

        future = self.cli.call_async(Trigger.Request())
        future.add_done_callback(self.analysis_response_callback)

    def analysis_response_callback(self, future):
        """Handle analysis response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Analysis result: {response.message}')
            else:
                self.get_logger().warn(f'Analysis failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def run_communication_demo():
    """Run the complete communication system demo"""
    rclpy.init()

    print("Starting ROS 2 Communication System Demo")
    print("=" * 45)

    # Create all nodes
    sensor_node = SensorNode()
    processing_node = ProcessingNode()
    analysis_node = AnalysisServiceNode()
    client_node = ClientNode()

    try:
        print("Running communication system... Press Ctrl+C to stop")
        # Use multi-threaded executor to run all nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(sensor_node)
        executor.add_node(processing_node)
        executor.add_node(analysis_node)
        executor.add_node(client_node)

        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down communication system...")
    finally:
        sensor_node.destroy_node()
        processing_node.destroy_node()
        analysis_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_communication_demo()
```

## Troubleshooting Common Communication Issues

### 1. Topic Connection Issues

```python
# Problem: Nodes cannot connect to topics
# Solution: Verify topic names and QoS settings

def verify_topic_connection(node, topic_name, expected_type):
    """Verify that a topic is available and accessible"""
    # Get list of publishers for the topic
    publishers = node.get_publishers_info_by_topic(topic_name)
    subscribers = node.get_subscriptions_info_by_topic(topic_name)

    print(f"Topic '{topic_name}' info:")
    print(f"  Publishers: {len(publishers)}")
    print(f"  Subscribers: {len(subscribers)}")

    for pub in publishers:
        print(f"    Publisher: {pub.node_name}, QoS: {pub.qos_profile}")

    return len(publishers) > 0 or len(subscribers) > 0
```

### 2. Service Connection Issues

```python
# Problem: Service client cannot connect to service
# Solution: Check service availability and retry logic

import time
from rclpy.task import Future

def call_service_with_retry(client, request, max_retries=5, delay=1.0):
    """Call service with retry logic"""
    for attempt in range(max_retries):
        try:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(request)

                # Wait for response with timeout
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 5.0:  # 5 second timeout
                        raise TimeoutError("Service call timed out")
                    time.sleep(0.1)

                return future.result()
            else:
                print(f"Service not available, attempt {attempt + 1}/{max_retries}")
                time.sleep(delay)
        except Exception as e:
            print(f"Service call failed (attempt {attempt + 1}): {e}")
            if attempt == max_retries - 1:
                raise
            time.sleep(delay)

    raise RuntimeError(f"Failed to call service after {max_retries} attempts")
```

### 3. Message Serialization Issues

```python
# Problem: Issues with message serialization/deserialization
# Solution: Validate message content before publishing

def validate_message_content(msg):
    """Validate message content before publishing"""
    if hasattr(msg, 'data'):
        if isinstance(msg.data, (int, float)) and (msg.data != msg.data):  # Check for NaN
            print("Warning: Message contains NaN value")
            return False
        if isinstance(msg.data, str) and len(msg.data) > 10000:  # Arbitrary large string check
            print("Warning: Message string is very large")
            return False
    return True

def safe_publish(publisher, msg):
    """Safely publish a message with validation"""
    if validate_message_content(msg):
        publisher.publish(msg)
    else:
        print("Message validation failed, not publishing")
```

## Resources for Further Learning

- [ROS 2 Topics and Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Message Types](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [ROS 2 Quality of Service Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS 2 Node Development Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Node.html)

## For Educators

### Educational Goals for Instructors
- Teach students how to create robust ROS 2 nodes with proper error handling and cleanup
- Help students understand the differences between publisher-subscriber and service communication patterns
- Guide students in implementing practical robotic communication systems using real-world examples

### Assessment Rubric for Communication Patterns
Students should be able to:
- Create functional ROS 2 nodes with proper initialization and cleanup (25%)
- Implement publisher-subscriber communication with appropriate message types (35%)
- Create and use services for request-response communication (25%)
- Troubleshoot common communication issues (15%)

### Teaching Suggestions
- Start with simple publisher-subscriber examples before introducing services
- Use the communication system exercise to demonstrate how different patterns work together
- Emphasize the importance of proper node initialization and destruction
- Show real-world applications where each communication pattern is most appropriate

### Common Student Challenges
- Understanding the asynchronous nature of publisher-subscriber communication
- Managing node lifecycle and proper resource cleanup
- Distinguishing between when to use topics vs. services for different use cases
- Debugging communication issues between nodes

### Accommodation Options
- Provide starter code templates for basic node structures
- Offer additional debugging exercises for students struggling with communication concepts
- Include visual tools or simulators to help visualize message flow between nodes

## Summary

The publisher-subscriber and service patterns form the core communication mechanisms in ROS 2. Understanding how to properly implement nodes, topics, and services is essential for building distributed robotic systems. The flexibility of these patterns allows for creating complex, interconnected robotic applications while maintaining loose coupling between components.