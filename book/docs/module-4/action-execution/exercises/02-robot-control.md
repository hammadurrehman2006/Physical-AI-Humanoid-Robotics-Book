# Exercise 4.2: Robot Control Systems Implementation

## Objective
Implement robot control systems for navigation and manipulation, enabling the Vision-Language-Action system to execute planned actions on the robot platform.

## Prerequisites
- Python 3.10+
- ROS 2 Humble Hawksbill installed
- Understanding of ROS 2 concepts (nodes, topics, actions)
- Completion of Exercise 4.1 (Action Planning)
- Robot simulation environment (Isaac Sim, Gazebo) or real robot hardware

## Exercise Steps

### Step 1: Set Up Robot Control Environment
Create a new file `robot_controller.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64MultiArray
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_ros import TransformException
import math
import time
from typing import List, Dict, Any, Optional, Tuple
import numpy as np

class NavigationController:
    """Controller for robot navigation and path following"""
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
        self.rotation_threshold = 0.1  # radians

        # For simple navigation controller
        self.is_navigating = False

    def odom_callback(self, msg: Odometry):
        """Update robot's current pose"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def scan_callback(self, msg: LaserScan):
        """Update laser scan data"""
        self.scan_data = msg

    def navigate_to_pose(self, target_pose: List[float], use_nav2: bool = True) -> bool:
        """
        Navigate to target pose using navigation2 or simple controller

        Args:
            target_pose: [x, y, z] or [x, y, z, roll, pitch, yaw]
            use_nav2: Whether to use Navigation2 stack or simple controller

        Returns:
            True if navigation succeeded, False otherwise
        """
        if use_nav2:
            return self._navigate_with_nav2(target_pose)
        else:
            return self._navigate_with_simple_controller(target_pose)

    def _navigate_with_nav2(self, target_pose: List[float]) -> bool:
        """Navigate using Navigation2 stack"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target_pose[0]
        goal_msg.pose.pose.position.y = target_pose[1]
        goal_msg.pose.pose.position.z = target_pose[2] if len(target_pose) > 2 else 0.0

        # Set orientation to face forward (if provided) or default
        if len(target_pose) >= 6:
            # Convert RPY to quaternion (simplified)
            roll, pitch, yaw = target_pose[3], target_pose[4], target_pose[5]
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            goal_msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
            goal_msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
            goal_msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
            goal_msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        else:
            # Default orientation (facing forward)
            goal_msg.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'Navigating to: {target_pose}')

        goal_handle_future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self.node, goal_handle_future)

        goal_handle = goal_handle_future.result()
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

    def _navigate_with_simple_controller(self, target_pose: List[float]) -> bool:
        """Navigate using simple proportional controller"""
        self.is_navigating = True
        rate = self.node.create_rate(10)  # 10 Hz

        target_x, target_y = target_pose[0], target_pose[1]

        while rclpy.ok() and self.is_navigating:
            if not self.current_pose:
                rate.sleep()
                continue

            # Calculate distance to target
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < self.arrival_threshold:
                # Stop the robot
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                self.is_navigating = False
                self.node.get_logger().info('Reached target position')
                return True

            # Check for obstacles
            if self._check_obstacle_ahead():
                self.node.get_logger().warn('Obstacle detected, stopping')
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                return False

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
            cmd_vel.linear.x = min(0.3, distance * 0.5)  # Proportional to distance
            cmd_vel.angular.z = angular_error * 1.0  # Proportional to angular error

            # Apply velocity limits
            cmd_vel.linear.x = max(-self.linear_vel_limit, min(cmd_vel.linear.x, self.linear_vel_limit))
            cmd_vel.angular.z = max(-self.angular_vel_limit, min(cmd_vel.angular.z, self.angular_vel_limit))

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

        self.is_navigating = False
        return False

    def _check_obstacle_ahead(self) -> bool:
        """Check if there's an obstacle directly ahead"""
        if not self.scan_data:
            return False

        # Check the front 30-degree sector
        front_indices = range(
            len(self.scan_data.ranges) // 2 - 15,
            len(self.scan_data.ranges) // 2 + 15
        )

        for i in front_indices:
            if 0 < i < len(self.scan_data.ranges):
                if self.scan_data.ranges[i] < 0.5:  # Obstacle within 0.5m
                    return True

        return False

    def _get_yaw_from_quaternion(self, quaternion) -> float:
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_navigation(self):
        """Stop current navigation"""
        self.is_navigating = False
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

class ManipulationController:
    """Controller for robotic manipulator and gripper"""
    def __init__(self, node: Node):
        self.node = node

        # Publishers and subscribers
        self.joint_state_sub = node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # For simple joint control (in simulation)
        self.joint_command_pub = node.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10
        )

        # Current joint states
        self.joint_names = []
        self.joint_positions = {}
        self.joint_velocities = {}

        # Manipulator parameters
        self.gripper_joint = 'gripper_joint'
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # For simulation purposes
        self.gripper_open_position = 0.0
        self.gripper_closed_position = 0.8

    def joint_state_callback(self, msg: JointState):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

        self.joint_names = msg.name

    def move_arm_to_position(self, joint_positions: List[float], duration: float = 5.0) -> bool:
        """Move arm to specified joint positions"""
        if len(joint_positions) != len(self.arm_joints):
            self.node.get_logger().error(f'Expected {len(self.arm_joints)} joint positions, got {len(joint_positions)}')
            return False

        # Create joint command message
        joint_cmd = Float64MultiArray()
        joint_cmd.data = joint_positions

        # Publish command
        self.joint_command_pub.publish(joint_cmd)

        self.node.get_logger().info(f'Moving arm to positions: {joint_positions}')

        # Wait for movement to complete (in simulation)
        time.sleep(duration)

        return True

    def move_arm_to_pose(self, target_pose: Pose) -> bool:
        """Move manipulator to target pose (requires inverse kinematics)"""
        self.node.get_logger().info(f'Moving arm to pose: {target_pose}')

        # In a real system, this would involve inverse kinematics
        # For this exercise, we'll simulate the movement

        # Calculate joint angles from pose (simplified)
        # In practice, you'd use a kinematics solver
        joint_positions = self._calculate_joint_positions_from_pose(target_pose)

        if joint_positions:
            return self.move_arm_to_position(joint_positions)
        else:
            self.node.get_logger().error('Could not calculate joint positions for target pose')
            return False

    def _calculate_joint_positions_from_pose(self, target_pose: Pose) -> Optional[List[float]]:
        """Calculate joint positions from end-effector pose (simplified)"""
        # This is a simplified inverse kinematics calculation
        # In a real system, you'd use a proper IK solver

        # For this exercise, return some default positions
        # In practice, you'd implement proper inverse kinematics
        try:
            # Extract target position
            x = target_pose.position.x
            y = target_pose.position.y
            z = target_pose.position.z

            # Simplified calculation based on position
            # This is just for demonstration purposes
            joint_positions = [
                math.atan2(y, x),  # Joint 1: shoulder pan
                math.atan2(z, math.sqrt(x*x + y*y)),  # Joint 2: shoulder lift
                0.0,  # Joint 3: elbow
                0.0,  # Joint 4: wrist 1
                0.0,  # Joint 5: wrist 2
                0.0   # Joint 6: wrist 3
            ]

            return joint_positions
        except:
            return None

    def grasp_object(self, object_position: List[float] = None, gripper_force: int = 50) -> bool:
        """Execute grasping action"""
        try:
            self.node.get_logger().info(f'Executing grasp with force: {gripper_force}')

            # Plan approach trajectory
            if object_position:
                approach_pose = self._calculate_approach_pose(object_position)

                # Move to approach position
                if not self.move_arm_to_pose(approach_pose):
                    return False

            # Close gripper
            success = self._control_gripper(closed=True, force=gripper_force)

            if success:
                self.node.get_logger().info('Grasp completed successfully')
            else:
                self.node.get_logger().error('Grasp failed')

            return success

        except Exception as e:
            self.node.get_logger().error(f'Grasp failed: {e}')
            return False

    def release_object(self) -> bool:
        """Release gripper"""
        try:
            self.node.get_logger().info('Releasing object')

            # Open gripper
            success = self._control_gripper(closed=False)

            if success:
                self.node.get_logger().info('Release completed successfully')
            else:
                self.node.get_logger().error('Release failed')

            return success

        except Exception as e:
            self.node.get_logger().error(f'Release failed: {e}')
            return False

    def _control_gripper(self, closed: bool, force: int = 50) -> bool:
        """Control gripper open/close"""
        # Create gripper command
        gripper_cmd = Float64MultiArray()

        if closed:
            # Close gripper (gripper position based on force)
            position = self.gripper_closed_position * (force / 100.0)
        else:
            # Open gripper
            position = self.gripper_open_position

        gripper_cmd.data = [position]

        # Publish gripper command
        self.joint_command_pub.publish(gripper_cmd)

        self.node.get_logger().info(f'Gripper command: {"close" if closed else "open"} at position {position}')

        # Wait for gripper to move
        time.sleep(1.0)

        return True

    def _calculate_approach_pose(self, object_position: List[float]) -> Pose:
        """Calculate approach pose before grasping"""
        approach_pose = Pose()

        # Approach from above the object
        approach_pose.position.x = object_position[0]
        approach_pose.position.y = object_position[1]
        approach_pose.position.z = object_position[2] + 0.2  # 20cm above object

        # Default orientation (facing down)
        approach_pose.orientation.w = 1.0  # Default orientation pointing down

        return approach_pose

    def move_to_place_position(self, target_position: List[float], orientation: List[float] = None) -> bool:
        """Move to position for placing object"""
        target_pose = Pose()
        target_pose.position.x = target_position[0]
        target_pose.position.y = target_position[1]
        target_pose.position.z = target_position[2]

        if orientation:
            # Apply specified orientation
            target_pose.orientation.x = orientation[0]
            target_pose.orientation.y = orientation[1]
            target_pose.orientation.z = orientation[2]
            target_pose.orientation.w = orientation[3]
        else:
            # Default orientation
            target_pose.orientation.w = 1.0

        return self.move_arm_to_pose(target_pose)

class GazeController:
    """Controller for robot gaze (head/eye movements)"""
    def __init__(self, node: Node):
        self.node = node

        # Publishers for head control
        self.head_pan_pub = node.create_publisher(Float64MultiArray, '/head_pan_controller/commands', 10)
        self.head_tilt_pub = node.create_publisher(Float64MultiArray, '/head_tilt_controller/commands', 10)

        # Current head state
        self.current_pan = 0.0
        self.current_tilt = 0.0

    def look_at_position(self, target_position: List[float]) -> bool:
        """Move head to look at a specific position"""
        try:
            # Calculate required pan and tilt angles
            dx = target_position[0] - 0  # Assuming robot is at origin
            dy = target_position[1] - 0
            dz = target_position[2] - 1.0  # Assuming head height of 1m

            # Calculate pan angle (horizontal)
            pan_angle = math.atan2(dy, dx)

            # Calculate tilt angle (vertical)
            horizontal_distance = math.sqrt(dx*dx + dy*dy)
            tilt_angle = math.atan2(dz, horizontal_distance)

            # Limit angles to reasonable ranges
            pan_angle = max(-1.57, min(1.57, pan_angle))  # -90 to 90 degrees
            tilt_angle = max(-0.785, min(0.785, tilt_angle))  # -45 to 45 degrees

            # Create and publish commands
            pan_cmd = Float64MultiArray()
            pan_cmd.data = [pan_angle]
            self.head_pan_pub.publish(pan_cmd)

            tilt_cmd = Float64MultiArray()
            tilt_cmd.data = [tilt_angle]
            self.head_tilt_pub.publish(tilt_cmd)

            self.node.get_logger().info(f'Looking at position {target_position}, pan: {pan_angle:.2f}, tilt: {tilt_angle:.2f}')

            # Update current state
            self.current_pan = pan_angle
            self.current_tilt = tilt_angle

            return True

        except Exception as e:
            self.node.get_logger().error(f'Gaze control failed: {e}')
            return False

    def center_gaze(self) -> bool:
        """Center the gaze (look straight ahead)"""
        try:
            center_cmd = Float64MultiArray()
            center_cmd.data = [0.0]  # Center position

            self.head_pan_pub.publish(center_cmd)
            self.head_tilt_pub.publish(center_cmd)

            self.current_pan = 0.0
            self.current_tilt = 0.0

            self.node.get_logger().info('Gaze centered')

            return True

        except Exception as e:
            self.node.get_logger().error(f'Gaze centering failed: {e}')
            return False

class RobotController:
    """Main robot controller that manages all control systems"""
    def __init__(self, node: Node):
        self.node = node
        self.navigation_controller = NavigationController(node)
        self.manipulation_controller = ManipulationController(node)
        self.gaze_controller = GazeController(node)

    def execute_navigation_action(self, target_position: List[float], use_nav2: bool = True) -> bool:
        """Execute navigation action"""
        return self.navigation_controller.navigate_to_pose(target_position, use_nav2)

    def execute_manipulation_action(self, action_params: Dict[str, Any]) -> bool:
        """Execute manipulation action based on parameters"""
        action_type = action_params.get('action_type', 'custom')

        if action_type == 'grasp_object':
            object_position = action_params.get('object_position', [1.0, 0.5, 0.1])
            gripper_force = action_params.get('gripper_force', 50)
            return self.manipulation_controller.grasp_object(object_position, gripper_force)

        elif action_type == 'release_object':
            return self.manipulation_controller.release_object()

        elif action_type == 'move_arm_to_pose':
            target_position = action_params.get('target_position', [0.5, 0.0, 0.8])
            target_pose = Pose()
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            target_pose.orientation.w = 1.0
            return self.manipulation_controller.move_arm_to_pose(target_pose)

        elif action_type == 'move_to_place_position':
            target_position = action_params.get('target_position', [0.5, 0.0, 0.8])
            orientation = action_params.get('orientation')
            return self.manipulation_controller.move_to_place_position(target_position, orientation)

        else:
            self.node.get_logger().warn(f'Unknown manipulation action type: {action_type}')
            return False

    def execute_gaze_action(self, action_params: Dict[str, Any]) -> bool:
        """Execute gaze control action"""
        action_type = action_params.get('action_type', 'look_at')

        if action_type == 'look_at':
            target_position = action_params.get('target_position', [1.0, 0.0, 0.0])
            return self.gaze_controller.look_at_position(target_position)

        elif action_type == 'center_gaze':
            return self.gaze_controller.center_gaze()

        else:
            self.node.get_logger().warn(f'Unknown gaze action type: {action_type}')
            return False

    def stop_all_motion(self):
        """Stop all robot motion"""
        self.navigation_controller.stop_navigation()
        # Add other stop commands as needed

def main():
    """Test the robot controller"""
    print("Testing Robot Controller...")

    # Initialize ROS 2
    rclpy.init()

    # Create a test node
    test_node = Node('robot_controller_test')

    # Create robot controller
    robot_controller = RobotController(test_node)

    print("Robot controller initialized")

    # Example navigation command
    print("\nTesting navigation to [1.0, 1.0, 0.0]...")
    nav_success = robot_controller.execute_navigation_action([1.0, 1.0, 0.0], use_nav2=False)
    print(f"Navigation result: {'Success' if nav_success else 'Failed'}")

    # Example manipulation command
    print("\nTesting grasp action...")
    grasp_params = {
        'action_type': 'grasp_object',
        'object_position': [1.0, 0.5, 0.1],
        'gripper_force': 50
    }
    grasp_success = robot_controller.execute_manipulation_action(grasp_params)
    print(f"Grasp result: {'Success' if grasp_success else 'Failed'}")

    # Example gaze command
    print("\nTesting gaze action...")
    gaze_params = {
        'action_type': 'look_at',
        'target_position': [1.0, 0.0, 0.5]
    }
    gaze_success = robot_controller.execute_gaze_action(gaze_params)
    print(f"Gaze result: {'Success' if gaze_success else 'Failed'}")

    # Shutdown
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()