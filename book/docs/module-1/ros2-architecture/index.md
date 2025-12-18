---
title: ROS 2 Architecture and Core Concepts
description: Understanding the fundamental architecture of ROS 2 and its core concepts
sidebar_position: 2
---

# ROS 2 Architecture and Core Concepts

## Learning Objectives
- Understand the fundamental architecture of ROS 2
- Identify the key components and their roles
- Learn about the differences between ROS 1 and ROS 2
- Recognize the DDS-based communication layer
- Appreciate the security and real-time capabilities

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is the next-generation robotics framework designed to address the limitations of ROS 1 while providing enhanced capabilities for modern robotics applications. Unlike ROS 1, which relied on a centralized master node, ROS 2 uses a decentralized architecture based on the Data Distribution Service (DDS) standard.

### Key Improvements Over ROS 1

1. **Decentralized Architecture**: No single point of failure
2. **Real-time Support**: Deterministic behavior for time-critical applications
3. **Security**: Built-in authentication and encryption
4. **Multi-robot Systems**: Native support for multi-robot coordination
5. **Professional Deployment**: Production-ready features

## Core Architecture Components

### 1. Nodes

Nodes are the fundamental execution units in ROS 2. Each node is a process that performs specific computations and communicates with other nodes:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

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

### 2. DDS (Data Distribution Service)

DDS is the middleware that provides the communication layer in ROS 2:

```python
# Understanding DDS QoS (Quality of Service) settings
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For sensor data (real-time, may lose some messages)
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For critical commands (must be delivered, keep history)
command_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

### 3. Communication Patterns

ROS 2 supports multiple communication patterns:

#### Publisher-Subscriber (Topics)
- Asynchronous, many-to-many communication
- Used for streaming data like sensor readings

#### Service-Client
- Synchronous request-response communication
- Used for actions that need confirmation

#### Action Server-Client
- Asynchronous with feedback and goal management
- Used for long-running tasks

## Understanding the Client Library Architecture

ROS 2 uses client libraries (rclpy for Python, rclcpp for C++) that interface with the underlying DDS implementation:

```python
# Architecture visualization
"""
+-------------------+    +------------------+    +------------------+
|   Application     |    |   Client Lib     |    |    DDS Impl      |
|   (User Code)     | -> |   (rclpy)        | -> |   (FastDDS, etc.)|
+-------------------+    +------------------+    +------------------+
| Node Definition   |    | Handle ROS 2     |    | DDS Entity       |
| Publishers        |    | Abstractions     |    | Management       |
| Subscribers       |    | Lifecycle        |    | Communication    |
| Services          |    | Management       |    | Protocols        |
+-------------------+    +------------------+    +------------------+
"""
```

### Lifecycle Nodes

ROS 2 introduces lifecycle nodes for better resource management:

```python
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleMinimalNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_minimal_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up lifecycle node')
        return TransitionCallbackReturn.SUCCESS
```

## Quality of Service (QoS) Settings

QoS settings allow fine-tuning of communication behavior:

```python
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class QoSDemoNode(Node):
    def __init__(self):
        super().__init__('qos_demo_node')

        # Different QoS profiles for different use cases

        # Sensor data: best effort, volatile
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Command data: reliable, persistent
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Status data: keep last, reliable
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publishers with different QoS
        self.sensor_pub = self.create_publisher(String, 'sensor_data', sensor_qos)
        self.command_pub = self.create_publisher(String, 'commands', command_qos)
        self.status_pub = self.create_publisher(String, 'status', status_qos)
```

## Namespaces and Compositions

ROS 2 supports hierarchical organization through namespaces:

```python
# Example of namespace usage
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self):
        # Initialize with namespace
        super().__init__('robot_controller', namespace='robot1')

        # Topic names will be: /robot1/joint_commands
        self.joint_pub = self.create_publisher(String, 'joint_commands', 10)

        # Service names will be: /robot1/get_robot_state
        self.srv = self.create_service(GetState, 'get_robot_state', self.state_callback)

def main(args=None):
    rclpy.init(args=args)

    # Multiple robots with different namespaces
    robot1 = RobotNode()
    robot2 = RobotNode()  # Will be under different namespace if specified

    try:
        rclpy.spin(robot1)
    except KeyboardInterrupt:
        pass
    finally:
        robot1.destroy_node()
        rclpy.shutdown()
```

## Security Architecture

ROS 2 includes built-in security features:

```python
# Security configuration example (conceptual)
"""
Security Files Structure:
robot_security/
├── identities/
│   ├── robot1.cert.pem      # Robot 1 certificate
│   ├── robot1.key.pem       # Robot 1 private key
│   └── ca.cert.pem          # Certificate authority
├── permissions/
│   ├── robot1_permissions.xml  # Robot 1 permissions
│   └── robot2_permissions.xml  # Robot 2 permissions
└── governance.xml              # Global security policy
"""
```

## Hands-on Exercise: Architecture Exploration

Create a simple ROS 2 system that demonstrates the core architectural concepts.

### Exercise Requirements
1. Create multiple nodes that communicate with different QoS settings
2. Implement a simple publisher-subscriber pattern
3. Demonstrate namespace usage
4. Show the node lifecycle

### Python Implementation

```python
#!/usr/bin/env python3
"""
ROS 2 Architecture Exploration
Demonstrates core architectural concepts
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class ArchitectureDemoNode(Node):
    def __init__(self, node_name, namespace=None):
        super().__init__(node_name, namespace=namespace)

        # Create publishers with different QoS settings
        self.best_effort_pub = self.create_publisher(
            String,
            'sensor_data',
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST
            )
        )

        self.reliable_pub = self.create_publisher(
            String,
            'command_data',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
        )

        # Create subscribers
        self.sensor_sub = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.command_sub = self.create_subscription(
            String,
            'command_data',
            self.command_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )
        )

        # Timer for publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info(f'Architecture Demo Node initialized in namespace: {namespace or "default"}')

    def sensor_callback(self, msg):
        self.get_logger().info(f'Sensor received: {msg.data}')

    def command_callback(self, msg):
        self.get_logger().info(f'Command received: {msg.data}')

    def timer_callback(self):
        # Publish sensor data (best effort)
        sensor_msg = String()
        sensor_msg.data = f'Sensor reading {self.counter} at {time.time()}'
        self.best_effort_pub.publish(sensor_msg)

        # Publish command data (reliable)
        command_msg = String()
        command_msg.data = f'Command {self.counter}'
        self.reliable_pub.publish(command_msg)

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    print("Starting ROS 2 Architecture Demo")
    print("=" * 40)

    # Create nodes in different namespaces
    node1 = ArchitectureDemoNode('demo_node_1', namespace='robot1')
    node2 = ArchitectureDemoNode('demo_node_2', namespace='robot2')

    try:
        print("Running nodes... Press Ctrl+C to stop")
        rclpy.spin_multi_threaded([node1, node2])
    except KeyboardInterrupt:
        print("\nShutting down nodes...")
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Common Architecture Issues

### 1. Node Discovery Issues

```python
# Problem: Nodes cannot find each other
# Solution: Check network configuration and domain IDs

import os

# Set ROS domain ID to avoid interference
os.environ['ROS_DOMAIN_ID'] = '42'  # Use specific domain ID

# Verify domain ID
import rclpy
print(f"ROS Domain ID: {os.environ.get('ROS_DOMAIN_ID', 'Not set')}")
```

### 2. QoS Compatibility Issues

```python
# Problem: Publishers and subscribers with incompatible QoS
# Solution: Ensure QoS compatibility

def check_qos_compatibility(publisher_qos, subscriber_qos):
    """
    Check if publisher and subscriber QoS settings are compatible
    """
    # Simplified compatibility check
    if (publisher_qos.reliability == ReliabilityPolicy.RELIABLE and
        subscriber_qos.reliability == ReliabilityPolicy.BEST_EFFORT):
        print("Warning: Reliable publisher with best-effort subscriber - may lose messages")

    return True
```

### 3. Resource Management

```python
# Problem: Memory leaks from unmanaged resources
# Solution: Properly destroy nodes and clean up resources

class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')
        self.timers = []
        self.subscribers = []
        self.publishers = []

    def destroy_node(self):
        # Clean up all resources
        for timer in self.timers:
            timer.destroy()
        for sub in self.subscribers:
            sub.destroy()
        for pub in self.publishers:
            pub.destroy()

        super().destroy_node()
```

## Resources for Further Learning

- [ROS 2 Design Documents](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/About-DDS/)
- [ROS 2 QoS Tutorials](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- "Programming Robots with ROS" by Morgan Quigley et al.

## For Educators

### Educational Goals for Instructors
- Understand how to teach ROS 2 architecture concepts to students with varying technical backgrounds
- Identify the key architectural differences between ROS 1 and ROS 2 that students need to grasp
- Develop effective hands-on exercises that demonstrate architectural concepts without overwhelming beginners

### Assessment Rubric for Architecture Understanding
Students should be able to:
- Explain the decentralized nature of ROS 2 vs. ROS 1's centralized architecture (40%)
- Implement basic nodes, topics, and services demonstrating communication patterns (30%)
- Apply appropriate QoS settings for different use cases (30%)

### Teaching Suggestions
- Use the architecture diagram visualization to help students understand the client library layer
- Emphasize the importance of QoS settings in real-world applications with examples
- Demonstrate namespace usage with multi-robot scenarios to show practical applications

### Common Student Challenges
- Understanding the concept of DDS as middleware and how it enables communication
- Grasping the significance of different QoS settings and when to use them
- Differentiating between publisher-subscriber, service-client, and action-client patterns

### Accommodation Options
- Provide pre-built ROS 2 packages for students with limited Linux experience
- Offer additional resources for students unfamiliar with distributed systems concepts
- Include visual aids and diagrams for different learning styles

## Summary

ROS 2's architecture represents a significant evolution from ROS 1, with a decentralized design based on DDS, enhanced security, real-time capabilities, and improved reliability. Understanding these architectural concepts is fundamental to building robust robotic systems. The QoS settings, namespaces, and lifecycle management features provide the flexibility needed for complex, real-world applications.