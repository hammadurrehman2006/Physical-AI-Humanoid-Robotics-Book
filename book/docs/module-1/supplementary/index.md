---
title: Module 1 Supplementary Materials
description: Additional resources, references, and advanced topics for Module 1
sidebar_position: 10
---

# Module 1 Supplementary Materials

This section contains additional resources, advanced topics, and reference materials that complement the core Module 1 content on the ROS 2 robotic nervous system. Use these materials to deepen your understanding of ROS 2 concepts and troubleshoot common issues you may encounter.

## Additional Resources

### Official ROS 2 Documentation
- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### Recommended Reading
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- "Effective Robotics Programming with ROS" by Anil Mahtani, Luis Sánchez Crespo, and Enrique Fernández Perdomo
- "ROS Robotics Projects" by Ramon Sanchez

### Online Resources
- [ROS Discourse Forum](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/questions/)
- [ROS Wiki](http://wiki.ros.org/)

## Advanced Topics

### Quality of Service (QoS) in Depth

Quality of Service settings allow you to configure how messages are delivered between nodes. Here are the main QoS policies:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example QoS configuration for sensor data
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Example QoS configuration for periodic status updates
status_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

### Advanced Parameter Handling

Parameters can be grouped and validated using parameter descriptors:

```python
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

# Declare a parameter with descriptor
descriptor = ParameterDescriptor(
    description='Maximum linear velocity for the robot',
    type=ParameterType.PARAMETER_DOUBLE,
    additional_constraints='Must be a positive value between 0.1 and 2.0',
    read_only=False
)

self.declare_parameter('max_linear_velocity', 0.5, descriptor)
```

### Custom Message Types

Creating custom message types for specialized data:

```python
# In your package's msg directory, create RobotStatus.msg:
# string robot_name
# float32 battery_level
# bool is_charging
# int32 error_code
# time last_update
```

## Troubleshooting Common Issues

### Node Communication Issues

**Problem**: Nodes cannot communicate with each other.
**Solutions**:
1. Check that nodes are on the same ROS domain ID
2. Verify that topic/service names match exactly
3. Ensure QoS settings are compatible between publisher and subscriber
4. Confirm that both nodes are running

### Parameter Configuration Issues

**Problem**: Parameters are not being loaded correctly.
**Solutions**:
1. Verify parameter names in launch files match those in the node
2. Check that parameter files are in the correct location
3. Ensure parameter file syntax is correct (YAML format)
4. Confirm that nodes are started after parameters are loaded

### Performance Issues

**Problem**: Robot system is running slowly or experiencing delays.
**Solutions**:
1. Reduce message frequency for high-bandwidth topics
2. Optimize QoS settings (use BEST_EFFORT for non-critical data)
3. Limit the number of callbacks running simultaneously
4. Profile your code to identify bottlenecks

## Best Practices

### Code Organization
- Use consistent naming conventions for topics, services, and parameters
- Group related functionality into logical nodes
- Separate business logic from ROS-specific code
- Use composition over inheritance when creating nodes

### Error Handling
- Always implement proper error handling in callbacks
- Use try-catch blocks for operations that may fail
- Implement graceful degradation when components fail
- Log errors with appropriate severity levels

### Testing
- Write unit tests for individual functions
- Create integration tests for node communication
- Use simulation environments for testing
- Implement continuous integration for automated testing

## Code Templates

### Basic Node Template

```python
#!/usr/bin/env python3
"""
Template for creating a basic ROS 2 node
"""

import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node_name')

        # Initialize node components here
        self.get_logger().info('Basic node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher-Subscriber Template

```python
#!/usr/bin/env python3
"""
Template for a node with publisher and subscriber
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create subscriber
        self.subscriber = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10
        )

        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)

    def callback(self, msg):
        """Handle incoming messages"""
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        """Publish messages periodically"""
        msg = String()
        msg.data = 'Hello from communication node'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Development Tools

### Essential ROS 2 Commands

```bash
# List all available topics
ros2 topic list

# Echo messages from a specific topic
ros2 topic echo /topic_name std_msgs/msg/String

# List all available services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty

# List all running nodes
ros2 node list

# View the graph of nodes and topics
rqt_graph

# Check parameter values
ros2 param list
ros2 param get /node_name parameter_name
```

### Debugging with rqt

The rqt suite provides various debugging tools:

- `rqt_graph`: Visualize the node graph
- `rqt_plot`: Plot numeric values over time
- `rqt_console`: Monitor log messages
- `rqt_bag`: Record and replay data
- `rqt_reconfigure`: Dynamically change parameters

## Glossary

- **Node**: A process that performs computation in ROS
- **Topic**: A named bus over which nodes exchange messages
- **Message**: A data packet sent between nodes over a topic
- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Service**: A synchronous request/response communication pattern
- **Action**: An asynchronous goal-based communication pattern
- **Parameter**: Configuration value accessible to a node
- **Launch File**: Configuration file that starts multiple nodes at once
- **Package**: A container for ROS functionality
- **QoS**: Quality of Service policies that define message delivery guarantees

## Frequently Asked Questions

**Q: How do I handle multiple robots in the same ROS network?**
A: Use different ROS_DOMAIN_ID values for each robot, or use namespaces to separate topics and services.

**Q: What's the difference between services and actions?**
A: Services are synchronous and should complete quickly, while actions are asynchronous and can take longer to complete, with feedback during execution.

**Q: How do I make my nodes more efficient?**
A: Use appropriate QoS settings, limit message frequency for high-bandwidth data, and implement proper error handling to avoid unnecessary retries.