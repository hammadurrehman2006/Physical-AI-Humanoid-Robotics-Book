#!/usr/bin/env python3
"""
ROS 2 Architecture Example

This example demonstrates the fundamental concepts of ROS 2 architecture:
- Nodes
- Topics
- Services
- Actions
- Parameters
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ArchitectureDemoNode(Node):
    """
    A demonstration node showing the fundamental ROS 2 architecture components
    """

    def __init__(self):
        super().__init__('architecture_demo_node')

        # 1. Publisher: Demonstrates the publisher-subscriber pattern
        self.publisher = self.create_publisher(
            String,
            'architecture_demo_topic',
            10
        )

        # 2. Subscriber: Demonstrates receiving messages
        self.subscription = self.create_subscription(
            String,
            'architecture_demo_topic',
            self.listener_callback,
            10
        )

        # 3. Service Server: Demonstrates request-response pattern
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # 4. Action Client: Demonstrates goal-feedback-result pattern
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        # 5. Parameters: Demonstrates parameter handling
        self.declare_parameter('demo_parameter', 'default_value')
        self.demo_param = self.get_parameter('demo_parameter').value

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

        self.get_logger().info(
            f'Architecture Demo Node initialized with parameter: {self.demo_param}'
        )

    def timer_callback(self):
        """Callback for the timer that publishes messages"""
        msg = String()
        msg.data = f'Hello ROS 2 Architecture! Message #{self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        """Callback for the subscriber that receives messages"""
        self.get_logger().info(f'Subscribed: "{msg.data}"')

    def add_two_ints_callback(self, request, response):
        """Callback for the service that adds two integers"""
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    """
    Main function demonstrating ROS 2 architecture components
    """
    rclpy.init(args=args)

    # Create the architecture demo node
    architecture_demo = ArchitectureDemoNode()

    # Create a MultiThreadedExecutor to handle multiple callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(architecture_demo)

    try:
        print("ROS 2 Architecture Demo Node is running...")
        print("This example demonstrates:")
        print("1. Publisher-Subscriber pattern (topics)")
        print("2. Request-Response pattern (services)")
        print("3. Goal-Feedback-Result pattern (actions)")
        print("4. Parameter handling")
        print("5. Node lifecycle management")
        print("\nCheck the console for published and subscribed messages.")
        print("Use 'ros2 service call /add_two_ints example_interfaces/AddTwoInts \"{a: 1, b: 2}\"' to test the service.")

        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down ROS 2 Architecture Demo Node...")
    finally:
        architecture_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()