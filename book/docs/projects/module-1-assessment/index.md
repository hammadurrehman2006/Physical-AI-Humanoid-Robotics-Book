---
title: Module 1 Assessment Project
description: Comprehensive project covering all ROS 2 fundamentals learned in Module 1
sidebar_position: 1
---

# Module 1 Assessment Project: ROS 2 Fundamentals Assessment

## Learning Objectives
- Implement a complete robotic system using ROS 2 concepts
- Demonstrate understanding of nodes, topics, services, and actions
- Create a functional robot controller with sensor integration
- Build and test a complete ROS 2 package
- Validate knowledge of launch files and parameters

## Educational Goals for Instructors

As an educator, this assessment project is designed to comprehensively evaluate students' understanding of ROS 2 fundamentals. This project serves as the capstone experience for Module 1, integrating all concepts learned throughout the module into a cohesive robotic system.

### Instructor's Guide to Assessment

This project provides multiple assessment opportunities to evaluate different aspects of student learning:

- **Implementation Skills**: Students demonstrate practical coding abilities in ROS 2
- **System Integration**: Understanding how different ROS 2 components work together
- **Problem-Solving**: Applying concepts to build a functioning robotic system
- **Best Practices**: Following ROS 2 development standards and conventions

### Expected Learning Outcomes

Upon completion of this assessment, students should demonstrate proficiency in:

1. **System Architecture**: Designing a complete ROS 2-based robotic system architecture
2. **Component Integration**: Connecting multiple nodes with proper communication patterns
3. **Configuration Management**: Using launch files and parameters effectively
4. **Safety Considerations**: Implementing safe operation protocols in robotic systems
5. **Debugging Skills**: Using ROS 2 tools to diagnose and fix system issues

### Assessment Rubric

Use the following rubric to evaluate student submissions:

| Component | Points | Evaluation Criteria |
|-----------|--------|-------------------|
| **Code Quality** | 20 | Well-structured, properly commented, following ROS 2 conventions |
| **Node Implementation** | 20 | Proper node structure, parameter declarations, lifecycle management |
| **Communication Patterns** | 25 | Correct use of topics, services, and actions with appropriate QoS settings |
| **System Integration** | 20 | All components work together seamlessly, proper error handling |
| **Documentation** | 15 | Clear README, inline comments, and setup instructions |

### Teaching Suggestions

For educators implementing this assessment:

- Provide starter code templates to help students focus on concepts rather than boilerplate
- Encourage iterative development: implement and test each component separately
- Recommend using ROS 2 visualization tools like `rqt_graph` and RViz for debugging
- Offer office hours or lab sessions for hands-on debugging assistance
- Consider providing sample test scenarios to validate system behavior

### Common Student Challenges

Be prepared to address these common issues students may encounter:

- **Package Dependencies**: Ensure all required dependencies are properly declared in `package.xml`
- **Parameter Configuration**: Students often struggle with parameter declaration and access
- **Coordinate Frames**: Understanding TF trees and frame transformations
- **Launch File Syntax**: Proper XML structure and substitution syntax
- **Node Lifecycle**: Understanding initialization and cleanup procedures

### Accommodation Options

For diverse learning needs:

- Provide partially completed code templates for students needing additional support
- Offer alternative assessment options for different learning styles
- Allow collaborative work for complex debugging challenges
- Provide additional time for students who need it

## Project Overview

The Module 1 Assessment Project requires you to build a complete robotic system that demonstrates all the concepts covered in Module 1. You will create a simulated robot that can navigate a simple environment while avoiding obstacles, using the ROS 2 architecture you've learned.

### Project Requirements

Your robot system must include:
1. A main controller node that orchestrates robot behavior
2. Sensor processing nodes for laser data
3. A navigation system using actions
4. Parameter management for configuration
5. Launch files to start the complete system
6. Proper URDF model for visualization

## Project Implementation

### 1. Robot Package Structure

First, create a new ROS 2 package for your project:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros module_1_assessment
```

### 2. URDF Model

Create a simple robot model in `urdf/robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="assessment_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Laser sensor -->
  <link name="laser_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Wheels (simplified) -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.1 -0.05" rpy="1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.1 -0.05" rpy="1.570796 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### 3. Main Robot Controller Node

Create the main controller in `module_1_assessment/robot_controller.py`:

```python
#!/usr/bin/env python3
"""
Main robot controller node for Module 1 Assessment
Implements complete robot behavior with navigation and obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from example_interfaces.action import NavigateToPose  # Using example interface for this assessment

import math
import time
from enum import Enum
from collections import deque

class RobotState(Enum):
    IDLE = 0
    NAVIGATING = 1
    AVOIDING_OBSTACLE = 2
    EMERGENCY_STOP = 3

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('robot_wheelbase', 0.2)

        # Get parameter values
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.robot_wheelbase = self.get_parameter('robot_wheelbase').value

        # Robot state
        self.current_state = RobotState.IDLE
        self.current_pose = Vector3()  # Simplified pose representation
        self.target_pose = None
        self.scan_data = None
        self.obstacle_detected = False
        self.emergency_stop_active = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Action server for navigation
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigate_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Timers
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Data history
        self.scan_history = deque(maxlen=5)

        self.get_logger().info('Robot Controller initialized successfully')
        self.get_logger().info(f'Parameters - Speed: {self.max_linear_speed}, Safety dist: {self.safety_distance}')

    def scan_callback(self, msg):
        """Handle laser scan data."""
        self.scan_data = msg
        self.scan_history.append(msg)

        # Check for obstacles
        if msg.ranges:
            # Get ranges in front of robot (simplified - just look at center ranges)
            center_ranges = msg.ranges[len(msg.ranges)//2-10:len(msg.ranges)//2+10]
            valid_ranges = [r for r in center_ranges if not math.isinf(r) and not math.isnan(r) and r > 0]

            if valid_ranges:
                min_range = min(valid_ranges)
                self.obstacle_detected = min_range < self.safety_distance
            else:
                self.obstacle_detected = False

            self.get_logger().debug(f'Min range: {min_range:.2f}, Obstacle: {self.obstacle_detected}')

    def goal_callback(self, goal_request):
        """Handle navigation goal requests."""
        self.get_logger().info(f'Received navigation goal: {goal_request}')
        # Accept all goals for this assessment
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle navigation cancellation requests."""
        self.get_logger().info('Navigation goal cancelled')
        return CancelResponse.ACCEPT

    async def execute_navigate_callback(self, goal_handle):
        """Execute navigation goal."""
        self.get_logger().info('Starting navigation task')

        # Extract target from goal (using example interface)
        # In a real implementation, you'd parse the actual NavigateToPose message
        target_x = 1.0  # Placeholder - in real implementation, extract from goal
        target_y = 1.0  # Placeholder - in real implementation, extract from goal

        self.target_pose = Vector3(x=target_x, y=target_y, z=0.0)
        self.current_state = RobotState.NAVIGATING

        feedback_msg = NavigateToPose.Feedback()
        result_msg = NavigateToPose.Result()

        # Navigation loop
        while self.current_state == RobotState.NAVIGATING:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.current_state = RobotState.IDLE
                result_msg.error_code = 1  # Canceled
                return result_msg

            # Simple navigation logic
            if self.reached_target():
                self.get_logger().info('Target reached!')
                self.current_state = RobotState.IDLE
                result_msg.error_code = 0  # Success
                goal_handle.succeed()
                return result_msg

            # Check for obstacles during navigation
            if self.obstacle_detected and not self.emergency_stop_active:
                self.get_logger().warn('Obstacle detected during navigation, avoiding...')
                self.current_state = RobotState.AVOIDING_OBSTACLE
                # Obstacle avoidance will be handled in control_loop
                # After obstacle is cleared, return to navigation

            # Publish feedback (simplified)
            feedback_msg.feedback = "Navigating to target"
            goal_handle.publish_feedback(feedback_msg)

            # Small delay to prevent busy waiting
            time.sleep(0.1)

        # If we exit the loop for other reasons
        result_msg.error_code = 2  # Aborted
        goal_handle.abort()
        return result_msg

    def reached_target(self):
        """Check if robot has reached the target."""
        if self.target_pose is None:
            return True  # No target, so "reached"

        # Simplified distance check
        distance = math.sqrt(
            (self.current_pose.x - self.target_pose.x)**2 +
            (self.current_pose.y - self.target_pose.y)**2
        )
        return distance < 0.2  # 20cm tolerance

    def control_loop(self):
        """Main control loop for robot behavior."""
        cmd_vel = Twist()

        if self.emergency_stop_active:
            # Emergency stop - stop all motion
            cmd_vel.linear = Vector3()
            cmd_vel.angular = Vector3()
            self.current_state = RobotState.EMERGENCY_STOP
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            # Simple obstacle avoidance behavior
            cmd_vel.linear = Vector3(x=0.0, y=0.0, z=0.0)
            cmd_vel.angular = Vector3(z=0.5)  # Turn right to avoid

            # Return to navigation when obstacle is cleared
            if not self.obstacle_detected:
                self.current_state = RobotState.NAVIGATING
        elif self.current_state == RobotState.NAVIGATING:
            # Navigate towards target
            if self.target_pose:
                # Simple proportional controller
                dx = self.target_pose.x - self.current_pose.x
                dy = self.target_pose.y - self.current_pose.y
                distance = math.sqrt(dx**2 + dy**2)

                if distance > 0.1:  # Not close enough to target
                    # Calculate desired angle
                    desired_angle = math.atan2(dy, dx)

                    # For this assessment, just move forward
                    cmd_vel.linear = Vector3(x=min(self.max_linear_speed, distance * 0.5))
                    cmd_vel.angular = Vector3(z=0.0)
                else:
                    # Close to target, stop
                    cmd_vel.linear = Vector3()
                    cmd_vel.angular = Vector3()
        else:
            # IDLE state - stop
            cmd_vel.linear = Vector3()
            cmd_vel.angular = Vector3()

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Update pose (simplified - in real implementation this would come from odometry)
        dt = 1.0 / self.control_frequency
        self.current_pose.x += cmd_vel.linear.x * dt
        self.current_pose.y += cmd_vel.linear.y * dt
        self.current_pose.z += cmd_vel.angular.z * dt  # Simplified rotation tracking

    def publish_status(self):
        """Publish robot status."""
        status_msg = String()
        status_msg.data = f'State: {self.current_state.name}, Obstacle: {self.obstacle_detected}, Emergency: {self.emergency_stop_active}'
        self.status_pub.publish(status_msg)

        # Publish simplified odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.current_pose.x
        odom_msg.pose.pose.position.y = self.current_pose.y
        odom_msg.pose.pose.position.z = 0.0
        # Simplified orientation
        from geometry_msgs.msg import Quaternion
        odom_msg.pose.pose.orientation = Quaternion(z=self.current_pose.z)
        self.odom_pub.publish(odom_msg)

    def emergency_stop(self):
        """Activate emergency stop."""
        self.emergency_stop_active = True
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def clear_emergency_stop(self):
        """Clear emergency stop."""
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop cleared')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robot Controller shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Sensor Processing Node

Create a sensor processing node in `module_1_assessment/sensor_processor.py`:

```python
#!/usr/bin/env python3
"""
Sensor processing node for Module 1 Assessment
Processes laser scan data and provides processed information
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point32

import math
from collections import deque

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Declare parameters
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('obstacle_threshold', 0.7)

        # Get parameter values
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        # Publishers
        self.obstacle_pub = self.create_publisher(String, 'obstacle_status', 10)
        self.min_range_pub = self.create_publisher(Float32, 'min_range', 10)
        self.front_range_pub = self.create_publisher(Float32, 'front_range', 10)
        self.obstacle_points_pub = self.create_publisher(String, 'obstacle_points', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Data storage
        self.scan_history = deque(maxlen=10)

        self.get_logger().info('Sensor Processor initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        self.scan_history.append(msg)

        # Process scan data
        if msg.ranges:
            # Calculate minimum range
            valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0]

            if valid_ranges:
                min_range = min(valid_ranges)
                min_range_msg = Float32()
                min_range_msg.data = min_range
                self.min_range_pub.publish(min_range_msg)

                # Calculate front range (center of scan)
                center_idx = len(msg.ranges) // 2
                front_range = msg.ranges[center_idx]
                front_range_msg = Float32()
                front_range_msg.data = front_range
                self.front_range_pub.publish(front_range_msg)

                # Determine obstacle status
                obstacle_status = String()
                if min_range < self.min_obstacle_distance:
                    obstacle_status.data = 'CRITICAL'
                elif min_range < self.obstacle_threshold:
                    obstacle_status.data = 'WARNING'
                else:
                    obstacle_status.data = 'CLEAR'

                self.obstacle_pub.publish(obstacle_status)

                # Find obstacle points
                obstacle_points = []
                angle_increment = msg.angle_increment
                angle_min = msg.angle_min

                for i, range_val in enumerate(msg.ranges):
                    if (not math.isinf(range_val) and not math.isnan(range_val) and
                        0 < range_val < self.obstacle_threshold):
                        angle = angle_min + i * angle_increment
                        x = range_val * math.cos(angle)
                        y = range_val * math.sin(angle)
                        obstacle_points.append(f"({x:.2f}, {y:.2f})")

                # Publish obstacle points as string (for this assessment)
                points_msg = String()
                points_msg.data = f"Obstacles: {len(obstacle_points)} points: {', '.join(obstacle_points[:5])}"  # Limit output
                self.obstacle_points_pub.publish(points_msg)

                self.get_logger().debug(f'Min range: {min_range:.2f}, Status: {obstacle_status.data}')
            else:
                # No valid ranges
                min_range_msg = Float32()
                min_range_msg.data = float('inf')
                self.min_range_pub.publish(min_range_msg)

                front_range_msg = Float32()
                front_range_msg.data = float('inf')
                self.front_range_pub.publish(front_range_msg)

                obstacle_status = String()
                obstacle_status.data = 'ERROR'
                self.obstacle_pub.publish(obstacle_status)

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sensor Processor shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Navigation Action Client (for testing)

Create a simple navigation client in `module_1_assessment/navigation_client.py`:

```python
#!/usr/bin/env python3
"""
Navigation client for Module 1 Assessment
Tests the navigation action server
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import NavigateToPose  # Using example interface for this assessment

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Navigation Client initialized')

    def send_goal(self, target_x=1.0, target_y=1.0):
        """Send a navigation goal."""
        self.get_logger().info('Waiting for action server...')

        # Wait for server
        self._action_client.wait_for_server()

        # Create goal (using example interface)
        goal_msg = NavigateToPose.Goal()
        # Note: In a real implementation, you'd set actual target coordinates
        # For this assessment, we'll use the action interface as provided

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')

def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()

    # Send a test goal
    future = client.send_goal(2.0, 2.0)  # Target coordinates

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Navigation Client shutting down...')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6. Launch File

Create a launch file in `launch/assessment_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='assessment_robot',
        description='Name of the robot'
    )

    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    # Get URDF path
    urdf_path = os.path.join(
        get_package_share_directory('module_1_assessment'),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': open(urdf_path).read()}
            ]
        ),

        # Joint state publisher (for fixed joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Main robot controller
        Node(
            package='module_1_assessment',
            executable='robot_controller',
            name='robot_controller',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'max_linear_speed': 0.5},
                {'max_angular_speed': 1.0},
                {'safety_distance': 0.5},
                {'control_frequency': 20.0}
            ],
            output='screen'
        ),

        # Sensor processor
        Node(
            package='module_1_assessment',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'scan_frequency': 10.0},
                {'min_obstacle_distance': 0.3},
                {'obstacle_threshold': 0.7}
            ],
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('module_1_assessment'),
                'rviz',
                'assessment_robot.rviz'
            )],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('launch_rviz', default='true')
            )
        )
    ])
```

### 7. Configuration File

Create a configuration file in `config/robot_params.yaml`:

```yaml
robot_controller:
  ros__parameters:
    # Navigation parameters
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    safety_distance: 0.5
    control_frequency: 20.0
    wheel_radius: 0.05
    robot_wheelbase: 0.2

    # Behavior parameters
    enable_obstacle_avoidance: true
    enable_navigation: true
    emergency_stop_distance: 0.2

sensor_processor:
  ros__parameters:
    scan_frequency: 10.0
    min_obstacle_distance: 0.3
    obstacle_threshold: 0.7
    enable_filtering: true
```

### 8. Setup Files

Update `setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'module_1_assessment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/assessment_robot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
        ('share/' + package_name + '/config', ['config/robot_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Module 1 Assessment - ROS 2 Fundamentals Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = module_1_assessment.robot_controller:main',
            'sensor_processor = module_1_assessment.sensor_processor:main',
            'navigation_client = module_1_assessment.navigation_client:main',
        ],
    },
)
```

### 9. Package.xml

Update `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>module_1_assessment</name>
  <version>0.0.0</version>
  <description>Module 1 Assessment Project - ROS 2 Fundamentals</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>example_interfaces</depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Testing the Implementation

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select module_1_assessment
source install/setup.bash
```

### 2. Run the Robot System

```bash
# Launch the complete robot system
ros2 launch module_1_assessment assessment_robot.launch.py

# In another terminal, send a navigation command
ros2 run module_1_assessment navigation_client
```

### 3. Monitor Topics

```bash
# Check available topics
ros2 topic list

# Monitor robot status
ros2 topic echo /robot_status std_msgs/msg/String

# Monitor commands sent to robot
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist

# Monitor sensor processing
ros2 topic echo /obstacle_status std_msgs/msg/String
```

## Evaluation Criteria

Your assessment project will be evaluated based on:

1. **Code Quality (25%)**: Clean, well-commented code following ROS 2 best practices
2. **Functionality (30%)**: All required components work as specified
3. **ROS 2 Concepts (25%)**: Proper use of nodes, topics, services, actions, and parameters
4. **System Integration (20%)**: Components work together cohesively in the complete system

### Specific Requirements Checklist:

- [ ] Robot controller node with proper parameter management
- [ ] Sensor processing node that analyzes laser data
- [ ] Action server for navigation tasks
- [ ] Launch file that starts the complete system
- [ ] Configuration file with parameters
- [ ] Proper URDF model for the robot
- [ ] Emergency stop functionality
- [ ] Obstacle detection and avoidance
- [ ] Clean, documented code
- [ ] Working demonstration of all features

## Resources for Success

- Review all Module 1 content on nodes, topics, services, actions, and parameters
- Ensure your launch files properly configure all nodes
- Test each component individually before integrating
- Use ROS 2 tools like `ros2 topic`, `ros2 service`, and `ros2 action` for debugging
- Validate your URDF model using `check_urdf` tool

## Troubleshooting

If you encounter issues:

1. Check that all dependencies are properly declared in `package.xml`
2. Verify that entry points are correctly defined in `setup.py`
3. Ensure parameter names match between code and configuration files
4. Use `rqt_graph` to visualize the node topology
5. Monitor console output for error messages

This assessment project demonstrates your comprehensive understanding of ROS 2 fundamentals by implementing a complete robotic system with navigation, sensing, and control capabilities.