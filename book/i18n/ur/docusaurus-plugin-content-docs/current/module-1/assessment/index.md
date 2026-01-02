---
title: ماڈیول 1 کا جائزہ
description: ماڈیول 1 میں سیکھے گئے ROS 2 کے تمام بنیادی اصولوں کا جامع جائزہ
sidebar_position: 9
---

# ماڈیول 1 کا جائزہ: ROS 2 کے بنیادی اصولوں کا امتحان

ماڈیول 1 کے تمام اسباق مکمل کرنے پر مبارکباد! یہ جائزہ آپ کے سیکھے گئے ROS 2 کے تصورات کی جانچ کرے گا، بشمول روبوٹک اعصابی نظام کا آرکیٹیکچر، نوڈ مواصلات، اور ROS 2 کے ساتھ Python کا انضمام۔

## سیکھنے کے مقاصد
- ROS 2 کے تصورات کا استعمال کرتے ہوئے ایک مکمل روبوٹک سسٹم نافذ کریں
- نوڈز، ٹاپکس، سروسز اور ایکشنز کی سمجھ کا مظاہرہ کریں
- سینسر انضمام کے ساتھ ایک فعال روبوٹ کنٹرولر بنائیں
- ایک مکمل ROS 2 پیکیج بنائیں اور ٹیسٹ کریں
- لانچ فائلوں اور پیرامیٹرز کے علم کی توثیق کریں

## جائزے کا جائزہ

ماڈیول 1 کے جائزے کے لیے آپ کو ایک مکمل روبوٹک سسٹم بنانے کی ضرورت ہے جو ماڈیول 1 میں شامل تمام تصورات کا مظاہرہ کرے۔ آپ ایک نقلی (simulated) روبوٹ بنائیں گے جو ROS 2 آرکیٹیکچر کا استعمال کرتے ہوئے رکاوٹوں سے بچتے ہوئے ایک سادہ ماحول میں نیویگیٹ کر سکے۔

### جائزے کی ضروریات

آپ کے روبوٹ سسٹم میں شامل ہونا چاہیے:
1. ایک مین کنٹرولر نوڈ جو روبوٹ کے رویے کو ترتیب دے
2. لیزر ڈیٹا کے لیے سینسر پروسیسنگ نوڈز
3. ایکشنز کا استعمال کرتے ہوئے نیویگیشن سسٹم
4. کنفیگریشن کے لیے پیرامیٹر مینجمنٹ
5. مکمل سسٹم شروع کرنے کے لیے لانچ فائلیں
6. وژولائزیشن کے لیے مناسب URDF ماڈل

## جائزے کا نفاذ

### 1. روبوٹ پیکیج کا ڈھانچہ

سب سے پہلے، اپنے اسیسمنٹ پروجیکٹ کے لیے ایک نیا ROS 2 پیکیج بنائیں:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros module_1_assessment
```

### 2. URDF ماڈل

`urdf/robot.urdf` میں ایک سادہ روبوٹ ماڈل بنائیں:

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

### 3. مین روبوٹ کنٹرولر نوڈ

`module_1_assessment/robot_controller.py` میں مین کنٹرولر بنائیں:

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

### 4. جائزہ کا روبک (Rubric)

اپنی جمع کرائی گئی اسائنمنٹ کا جائزہ لینے کے لیے درج ذیل روبک کا استعمال کریں:

| جزو | پوائنٹس | تشخیص کا معیار |
|-----------|--------|-------------------|
| **کوڈ کا معیار** | 20 | اچھی ساخت، مناسب تبصرے، ROS 2 کے کنونشنز کی پیروی |
| **نوڈ کا نفاذ** | 20 | مناسب نوڈ ڈھانچہ، پیرامیٹر ڈیکلریشن، لائف سائیکل مینجمنٹ |
| **مواصلاتی پیٹرنز** | 25 | ٹاپکس، سروسز، اور ایکشنز کا درست استعمال مناسب QoS سیٹنگز کے ساتھ |
| **سسٹم انضمام** | 20 | تمام اجزاء بغیر کسی رکاوٹ کے ایک ساتھ کام کرتے ہیں، مناسب غلطی کی ہینڈلنگ |
| **دستاویزات** | 15 | واضح README، ان لائن تبصرے، اور سیٹ اپ ہدایات |

## جائزے کی جمع آوری

اس جائزے کو مکمل کرنے کے لیے، آپ کو جمع کرانا ہوگا:

1. تمام سورس کوڈ فائلیں (نوڈز، لانچ فائلیں، کنفیگریشن)
2. URDF ماڈل فائل
3. پیکیج کنفیگریشن فائلیں
4. آپ کے نفاذ کے طریقہ کار کی وضاحت کرنے والی ایک مختصر رپورٹ
5. کامیاب ٹیسٹنگ کا ثبوت (اسکرین شاٹس، لاگز، یا ویڈیو)

آپ کا اسیسمنٹ پروجیکٹ نیویگیشن، سینسنگ، اور کنٹرول کی صلاحیتوں کے ساتھ ایک مکمل روبوٹک سسٹم نافذ کرکے ROS 2 کے بنیادی اصولوں کی آپ کی جامع سمجھ کا مظاہرہ کرتا ہے۔
