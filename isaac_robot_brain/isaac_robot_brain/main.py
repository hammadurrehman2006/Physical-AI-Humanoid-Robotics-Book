#!/usr/bin/env python3
"""
Main entry point for the Isaac Robot Brain - Vision-Language-Action system.

This module initializes the core ROS 2 nodes for the vision-language-action pipeline.
"""

import rclpy
from rclpy.node import Node


class IsaacRobotBrainNode(Node):
    """Main ROS 2 node for the Isaac Robot Brain Vision-Language-Action system."""

    def __init__(self):
        """Initialize the Isaac Robot Brain node."""
        super().__init__('isaac_robot_brain_node')
        self.get_logger().info('Isaac Robot Brain node initialized')


def main(args=None):
    """Main entry point for the Isaac Robot Brain node."""
    rclpy.init(args=args)

    try:
        node = IsaacRobotBrainNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()