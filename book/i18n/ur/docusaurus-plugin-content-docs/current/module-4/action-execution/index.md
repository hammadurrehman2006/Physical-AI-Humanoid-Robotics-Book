---
sidebar_position: 4
title: "ایکشن ایکزیکیوشن"
---

# ایکشن ایکزیکیوشن

اس سیکشن میں، ہم اپنے وژن لینگویج ایکشن سسٹم کے ایکشن ایکزیکیوشن جزو کو امپلیمنٹ کریں گے۔ یہ ماڈیول زبان کی سمجھ بوجھ اور وژوئل پریسیپشن سسٹم سے بلند سطحی کمانڈز کو مخصوص روبوٹک ایکشنز اور موومنٹس میں تبدیل کرتا ہے۔

## جائزہ

ایکشن ایکزیکیوشن ہمارے VLA سسٹم کا آخری جزو ہے، جہاں روبوٹ فیزیکلی تشریح شدہ کمانڈز کا جواب دیتا ہے۔ یہ سیکشن یہ احاطہ کرتا ہے::
- ایکشن پلاننگ اور ٹریجکٹری جنریشن
- موٹر کنٹرول اور مینیپولیشن سسٹم
- ایکشن ایکزیکیوشن اور مانیٹرنگ
- بند حلقہ کنٹرول کے لیے پریسیپشن سسٹم کے ساتھ انٹیگریشن
- ایکشن ایکزیکیوشن میں سیفٹی اور خرابی کا انتظام

## سیکھنے کے اہداف

اس سیکشن کے اختتام تک، آپ کے اہل ہوگا::
- ایکشن پلاننگ سسٹم امپلیمنٹ کریں جو ایکزیکوٹبل ٹریجکٹریز تیار کرے
- ٹاسک ایکزیکیوشن کے لیے روبوٹک مینیپولیٹرز اور نیویگیشن سسٹم کنٹرول کریں
- فیڈ بیک اور خرابی کے انتظام کے ساتھ ایکشن ایکزیکیوشن کو مانیٹر کریں
- بند حلقہ ایکشن ایکزیکیوشن کے لیے پریسیپشن سسٹم کو انٹیگریٹ کریں
- سیفٹی مکینزم اور ریکوری طریق کار امپلیمنٹ کریں

## ایکشن پلاننگ اور نمائندگی

### ایکشن اسپیس کی تعریف

سب سے پہل، ہم اپنے روبوٹ کے لیے ایکشن اسپیس کی وضاحت کریں گے::

```python
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Any, Optional, Tuple
import numpy as np

class ActionType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    GAZE_CONTROL = "gaze_control"
    SPEECH = "speech"
    INTERACTION = "interaction"

@dataclass
class Action:
    action_type: ActionType
    parameters: Dict[str, Any]
    priority: int = 0
    timeout: float = 30.0  # seconds
    preconditions: List[str] = None
    effects: List[str] = None

@dataclass
class NavigationAction(Action):
    """راہ کی منصوبہ بندی کے ساتھ نیویگیشن ایکشن"""
    def __post_init__(self):
        self.action_type = ActionType.NAVIGATION

@dataclass
class ManipulationAction(Action):
    """روبوٹک آرمز کے لیے مینیپولیشن ایکشن"""
    def __post_init__(self):
        self.action_type = ActionType.MANIPULATION

@dataclass
class GazeAction(Action):
    """ہیڈ/آئی موومنٹس کے لیے گیز کنٹرول ایکشن"""
    def __post_init__(self):
        self.action_type = ActionType.GAZE_CONTROL
```

### ایکشن پلینر

ایکشن پلینر بلند سطحی مقاصد کو حاصل کرنے کے لیے بنیادی ایکشنز کی ترتیب تیار کرتا ہے::

```python
from abc import ABC, abstractmethod
from typing import Union
import time

class ActionPlanner:
    def __init__(self):
        self.robot_state = None
        self.world_model = None
        self.action_library = self._initialize_action_library()

    def _initialize_action_library(self) -> Dict[str, Action]:
        """دستیاب ایکشنز کو شروع کریں"""
        return {
            'move_to': NavigationAction(
                parameters={'target_position': None, 'speed': 0.5},
                preconditions=['robot_is_operational'],
                effects=['robot_at_target']
            ),
            'grasp_object': ManipulationAction(
                parameters={'object_id': None, 'position': None, 'gripper_force': 50},
                preconditions=['object_detected', 'arm_free', 'reachable'],
                effects=['object_grasped']
            ),
            'place_object': ManipulationAction(
                parameters={'position': None, 'orientation': None},
                preconditions=['object_grasped'],
                effects=['object_placed', 'gripper_open']
            ),
            'point_to': ManipulationAction(
                parameters={'target_position': None, 'duration': 2.0},
                preconditions=['arm_free'],
                effects=['pointing_at_target']
            ),
            'look_at': GazeAction(
                parameters={'target_position': None},
                preconditions=['head_operational'],
                effects=['looking_at_target']
            )
        }

    def plan_action(self, high_level_command: Dict[str, Any]) -> List[Action]:
        """بلند سطحی کمانڈ سے بنیادی ایکشنز کو منصوبہ بند کریں"""
        intent = high_level_command.get('intent', '')
        entities = high_level_command.get('entities', {})

        actions = []

        if intent == 'navigate':
            target_location = entities.get('location', ['default'])[0]
            actions.append(self._create_navigation_action(target_location))

        elif intent == 'grasp':
            target_object = entities.get('target_object', entities.get('object', ['default'])[0])
            object_position = entities.get('object_position', None)
            actions.extend(self._create_grasp_sequence(target_object, object_position))

        elif intent == 'place':
            target_location = entities.get('location', ['default'])[0]
            actions.append(self._create_place_action(target_location))

        elif intent == 'identify':
            target_object = entities.get('target_object', entities.get('object', ['default'])[0])
            actions.extend(self._create_identification_sequence(target_object))

        return actions

    def _create_navigation_action(self, target_location: str) -> NavigationAction:
        """ہدف مقام کے لیے نیویگیشن ایکشن تخلیق کریں"""
        # ایک حقیقی سسٹم میں، یہ سیمینٹک میپ کا استعمال کرے گا
        # فی الحال، ہم پہلے سے طے شدہ مقامات کا استعمال کریں گے
        location_map = {
            'kitchen': [2.0, 2.0, 0.0],
            'bedroom': [-2.0, 2.0, 0.0],
            'office': [0.0, -2.0, 0.0],
            'living room': [2.0, -2.0, 0.0],
            'default': [0.0, 0.0, 0.0]
        }

        target_position = location_map.get(target_location.lower(), location_map['default'])

        return NavigationAction(
            parameters={
                'target_position': target_position,
                'speed': 0.5,
                'avoid_obstacles': True
            },
            priority=1
        )

    def _create_grasp_sequence(self, target_object: str, object_position: Optional[List[float]]) -> List[Action]:
        """ایک چیز کو تھامنے کے لیے ایکشنز کی ترتیب تخلیق کریں"""
        actions = []

        if object_position:
            # چیز کی طرف جائیں
            actions.append(NavigationAction(
                parameters={
                    'target_position': [object_position[0], object_position[1], 0.0],  # چیز کے مقام پر جائیں
                    'speed': 0.3,
                    'avoid_obstacles': True
                },
                priority=2
            ))

        # قریب آئیں اور تھامیں
        actions.append(ManipulationAction(
            parameters={
                'object_id': target_object,
                'approach_distance': 0.1,
                'gripper_force': 30
            },
            priority=3
        ))

        return actions

    def _create_place_action(self, target_location: str) -> ManipulationAction:
        """چیز رکھنے کے لیے ایکشن تخلیق کریں"""
        location_positions = {
            'table': [1.0, 0.0, 0.8],  # معیاری ٹیبل کی اونچائی
            'shelf': [1.0, 0.0, 1.2],  # معیاری شیلف کی اونچائی
            'counter': [0.5, 0.0, 0.9],
            'default': [0.0, 0.0, 0.8]
        }

        position = location_positions.get(target_location.lower(), location_positions['default'])

        return ManipulationAction(
            parameters={
                'target_position': position,
                'release_force': 0,
                'orientation': [0, 0, 0]  # ڈیفالٹ اورینٹیشن
            },
            priority=2
        )

    def _create_identification_sequence(self, target_object: str) -> List[Action]:
        """چیز کی پہچان کے لیے ایکشنز کی ترتیب تخلیق کریں"""
        actions = []

        # چیز کی طرف اشارہ کریں
        actions.append(ManipulationAction(
            parameters={
                'target_object': target_object,
                'motion_type': 'pointing',
                'duration': 2.0
            },
            priority=1
        ))

        # چیز کو دیکھیں
        actions.append(GazeAction(
            parameters={
                'target_object': target_object
            },
            priority=1
        ))

        return actions
```

## روبوٹ کنٹرول سسٹم

### نیویگیشن کنٹرول

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
from tf2_ros import TransformException
import math

class NavigationController:
    def __init__(self, node: Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # نیویگیشن ایکشن کلائنٹ
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # پبلشرز اور سبسکرائبرز
        self.cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = node.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = node.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # روبوٹ کی حالت
        self.current_pose = None
        self.current_velocity = None
        self.scan_data = None

        # نیویگیشن پیرامیٹرز
        self.linear_vel_limit = 0.5
        self.angular_vel_limit = 1.0
        self.arrival_threshold = 0.2  # میٹر

    def odom_callback(self, msg: Odometry):
        """روبوٹ کا موجودہ پوز اپ ڈیٹ کریں"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def scan_callback(self, msg: LaserScan):
        """لیزر اسکین ڈیٹا اپ ڈیٹ کریں"""
        self.scan_data = msg

    def navigate_to_pose(self, target_pose: List[float]) -> bool:
        """نیویگیشن2 کا استعمال کرتے ہوئے ہدف پوز پر جائیں"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('نیویگیشن ایکشن سرور دستیاب نہیں')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target_pose[0]
        goal_msg.pose.pose.position.y = target_pose[1]
        goal_msg.pose.pose.position.z = target_pose[2] if len(target_pose) > 2 else 0.0

        # اورینٹیشن کو سامنے کی طرف کرنے کے لیے سیٹ کریں
        goal_msg.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'کوچل رہا ہے: {target_pose}')

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('نیویگیشن گول مسترد کر دیا گیا')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result().result
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('نیویگیشن کامیاب')
            return True
        else:
            self.node.get_logger().error(f'نیویگیشن ناکام، حیثیت: {status}')
            return False

    def simple_move_to(self, target_x: float, target_y: float, speed: float = 0.3) -> bool:
        """نیویگیشن کے لیے سادہ متناسب کنٹرولر"""
        rate = self.node.create_rate(10)  # 10 Hz

        while rclpy.ok():
            if not self.current_pose:
                continue

            # ہدف تک فاصلہ کا حساب لگائیں
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < self.arrival_threshold:
                # روبوٹ کو روکیں
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                return True

            # ہدف کی طرف سر کا حساب لگائیں
            target_angle = math.atan2(dy, dx)
            current_angle = self._get_yaw_from_quaternion(self.current_pose.orientation)

            # سادہ متناسب کنٹرول
            angular_error = target_angle - current_angle
            # زاویہ کو [-pi, pi] میں نارملائز کریں
            while angular_error > math.pi:
                angular_error -= 2 * math.pi
            while angular_error < -math.pi:
                angular_error += 2 * math.pi

            # ولسٹی کمانڈ تخلیق کریں
            cmd_vel = Twist()
            cmd_vel.linear.x = min(speed, distance * 0.5)  # فاصلے کے متناسب
            cmd_vel.angular.z = angular_error * 1.0  # زاویے کی خرابی کے متناسب

            # ولسٹی حدود لاگو کریں
            cmd_vel.linear.x = max(-self.linear_vel_limit, min(cmd_vel.linear.x, self.linear_vel_limit))
            cmd_vel.angular.z = max(-self.angular_vel_limit, min(cmd_vel.angular.z, self.angular_vel_limit))

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

    def _get_yaw_from_quaternion(self, quaternion) -> float:
        """کوائفنین سے yaw زاویہ نکالیں"""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
```

### مینیپولیشن کنٹرول

```python
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import math

class ManipulationController:
    def __init__(self, node: Node):
        self.node = node

        # پبلشرز اور سبسکرائبرز
        self.joint_state_sub = node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.trajectory_pub = node.create_publisher(
            JointTrajectory, 'joint_trajectory', 10
        )

        # زیادہ پیچیدہ موومنٹس کے لیے جوائنٹ ٹریجکٹری ایکشن کلائنٹ
        self.trajectory_client = ActionClient(
            node, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory'
        )

        # موجودہ جوائنٹ اسٹیٹس
        self.joint_names = []
        self.joint_positions = {}
        self.joint_velocities = {}

        # مینیپولیٹر پیرامیٹرز
        self.gripper_joint = 'gripper_joint'
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def joint_state_callback(self, msg: JointState):
        """موجودہ جوائنٹ اسٹیٹس اپ ڈیٹ کریں"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

        self.joint_names = msg.name

    def grasp_object(self, object_position: List[float], gripper_force: int = 50) -> bool:
        """گریسپ ایکشن انجام دیں"""
        try:
            # اپروچ ٹریجکٹری کو منصوبہ بند کریں
            approach_pose = self._calculate_approach_pose(object_position)

            # اپروچ پوزیشن پر جائیں
            if not self.move_to_pose(approach_pose, speed=0.1):
                return False

            # گریسپ انجام دیں
            return self.execute_grasp(gripper_force)

        except Exception as e:
            self.node.get_logger().error(f'گریسپ ناکام: {e}')
            return False

    def place_object(self, target_position: List[float], orientation: List[float] = None) -> bool:
        """چیز رکھنے کا ایکشن انجام دیں"""
        try:
            if orientation is None:
                orientation = [0, 0, 0, 1]  # ڈیفالٹ اورینٹیشن

            target_pose = Pose()
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            target_pose.orientation.x = orientation[0]
            target_pose.orientation.y = orientation[1]
            target_pose.orientation.z = orientation[2]
            target_pose.orientation.w = orientation[3]

            # ہدف پوزیشن پر جائیں
            if not self.move_to_pose(target_pose, speed=0.1):
                return False

            # چیز چھوڑ دیں
            return self.release_object()

        except Exception as e:
            self.node.get_logger().error(f'چیز رکھنا ناکام: {e}')
            return False

    def move_to_pose(self, target_pose: Pose, speed: float = 0.5) -> bool:
        """مینیپولیٹر کو ہدف پوز پر لے جائیں"""
        # ایک حقیقی سسٹم میں، یہ انورس کنیمیٹکس کا استعمال کرے گا
        # اس مثال کے لیے، ہم موومنٹ کو سیمولیٹ کریں گے
        self.node.get_logger().info(f'پوز پر جا رہا ہے: {target_pose}')

        # یہ انورس کنیمیٹکس حل کرے گا اور جوائنٹ ٹریجکٹری انجام دے گا
        # فی الحال، ہم کامیابی لوٹائیں گے
        return True

    def execute_grasp(self, force: int) -> bool:
        """گریپر گریسپ انجام دیں"""
        self.node.get_logger().info(f'گریسپ انجام دے رہا ہے، زور: {force}')

        # گریپر کمانڈ بھیجیں
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint]

        point = JointTrajectoryPoint()
        # یہ ایک سادہ مثال ہے - حقیقی گریپرز کے پاس مختلف کنٹرول طریقے ہیں
        point.positions = [force / 100.0]  # زور کو جوائنٹ پوزیشن میں اسکیل کریں
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points = [point]

        self.trajectory_pub.publish(trajectory)
        self.node.get_logger().info('گریسپ کمانڈ شائع کی گئی')

        return True

    def release_object(self) -> bool:
        """گریپر کو چھوڑ دیں"""
        self.node.get_logger().info('چیز چھوڑ رہا ہے')

        # گریپر ریلیز کمانڈ بھیجیں
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint]

        point = JointTrajectoryPoint()
        point.positions = [0.0]  # گریپر کھولیں
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points = [point]

        self.trajectory_pub.publish(trajectory)
        self.node.get_logger().info('ریلیز کمانڈ شائع کی گئی')

        return True

    def _calculate_approach_pose(self, object_position: List[float]) -> Pose:
        """گریسپ سے پہلے اپروچ پوز کا حساب لگائیں"""
        approach_pose = Pose()
        approach_pose.position.x = object_position[0] - 0.1  # سامنے سے قریب آئیں
        approach_pose.position.y = object_position[1]
        approach_pose.position.z = object_position[2] + 0.1  # اوپر سے قریب آئیں
        approach_pose.orientation.w = 1.0  # ڈیفالٹ اورینٹیشن

        return approach_pose
```

## ایکشن ایکزیکیوشن فریم ورک

### ایکشن ایکزیکیوٹر

```python
import asyncio
from typing import Callable
from enum import Enum
import threading
import time

class ExecutionStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"

class ActionExecutor:
    def __init__(self, node: Node):
        self.node = node
        self.navigation_controller = NavigationController(node)
        self.manipulation_controller = ManipulationController(node)

        self.current_action = None
        self.execution_status = ExecutionStatus.PENDING
        self.execution_thread = None
        self.stop_execution = threading.Event()

    def execute_action(self, action: Action) -> bool:
        """ایک ایکشن انجام دیں"""
        self.current_action = action
        self.execution_status = ExecutionStatus.EXECUTING
        self.stop_execution.clear()

        try:
            if action.action_type == ActionType.NAVIGATION:
                return self._execute_navigation(action)
            elif action.action_type == ActionType.MANIPULATION:
                return self._execute_manipulation(action)
            elif action.action_type == ActionType.GAZE_CONTROL:
                return self._execute_gaze_control(action)
            else:
                self.node.get_logger().error(f'نامعلوم ایکشن قسم: {action.action_type}')
                return False

        except Exception as e:
            self.node.get_logger().error(f'ایکشن ایکزیکیوشن ناکام: {e}')
            self.execution_status = ExecutionStatus.FAILED
            return False

    def execute_action_sequence(self, actions: List[Action]) -> bool:
        """ایکشنز کی ترتیب انجام دیں"""
        for action in actions:
            if self.stop_execution.is_set():
                self.execution_status = ExecutionStatus.CANCELLED
                return False

            self.node.get_logger().info(f'ایکشن انجام دے رہا ہے: {action.action_type}')

            success = self.execute_action(action)
            if not success:
                self.node.get_logger().error(f'ایکشن ناکام: {action.action_type}')
                return False

        self.execution_status = ExecutionStatus.SUCCEEDED
        return True

    def _execute_navigation(self, action: NavigationAction) -> bool:
        """نیویگیشن ایکشن انجام دیں"""
        target_position = action.parameters.get('target_position')
        speed = action.parameters.get('speed', 0.5)
        avoid_obstacles = action.parameters.get('avoid_obstacles', True)

        if not target_position:
            self.node.get_logger().error('نیویگیشن ایکشن میں ہدف پوزیشن غائب ہے')
            return False

        self.node.get_logger().info(f'کوچل رہا ہے: {target_position}')

        # نیویگیشن انجام دینے کے لیے نیویگیشن کنٹرولر کا استعمال کریں
        success = self.navigation_controller.navigate_to_pose(target_position)

        if success:
            self.node.get_logger().info('نیویگیشن کامیابی سے مکمل ہوا')
        else:
            self.node.get_logger().error('نیویگیشن ناکام')

        return success

    def _execute_manipulation(self, action: ManipulationAction) -> bool:
        """مینیپولیشن ایکشن انجام دیں"""
        action_type = action.parameters.get('action_type', 'custom')
        object_id = action.parameters.get('object_id')
        position = action.parameters.get('position')

        if action_type == 'grasp_object':
            object_position = action.parameters.get('position')
            gripper_force = action.parameters.get('gripper_force', 50)

            if not object_position:
                self.node.get_logger().error('گریسپ ایکشن میں چیز کی پوزیشن غائب ہے')
                return False

            return self.manipulation_controller.grasp_object(object_position, gripper_force)

        elif action_type == 'place_object':
            target_position = action.parameters.get('target_position')
            orientation = action.parameters.get('orientation')

            if not target_position:
                self.node.get_logger().error('چیز رکھنے کا ایکشن ہدف پوزیشن غائب ہے')
                return False

            return self.manipulation_controller.place_object(target_position, orientation)

        else:
            self.node.get_logger().info(f'کسٹم مینیپولیشن انجام دے رہا ہے: {action_type}')
            # دیگر مینیپولیشن ایکشنز کو ضرورت کے مطابق امپلیمنٹ کریں
            return True

    def _execute_gaze_control(self, action: GazeAction) -> bool:
        """گیز کنٹرول ایکشن انجام دیں"""
        target_position = action.parameters.get('target_position')
        target_object = action.parameters.get('target_object')

        self.node.get_logger().info(f'گیز کنٹرول: {target_position or target_object}')

        # ایک حقیقی سسٹم میں، یہ ہیڈ/آئی موومنٹس کو کنٹرول کرے گا
        # فی الحال، ہم صرف ایکشن کو لاگ کریں گے
        return True

    def cancel_execution(self):
        """موجودہ ایکشن ایکزیکیوشن منسوخ کریں"""
        self.stop_execution.set()
        self.execution_status = ExecutionStatus.CANCELLED
```

## پریسیپشن سسٹم کے ساتھ انٹیگریشن

### بند حلقہ کنٹرول

```python
class ClosedLoopController:
    def __init__(self, node: Node):
        self.node = node
        self.action_executor = ActionExecutor(node)
        self.perception_handler = None  # بیرونی طور پر سیٹ کیا جائے گا
        self.robot_state = None

    def set_perception_handler(self, handler):
        """فیڈ بیک کے لیے پریسیپشن ہینڈلر سیٹ کریں"""
        self.perception_handler = handler

    def execute_with_feedback(self, action_sequence: List[Action],
                            max_attempts: int = 3) -> bool:
        """پریسیپشن فیڈ بیک اور خرابی کی بازیافت کے ساتھ ایکشنز انجام دیں"""

        for attempt in range(max_attempts):
            self.node.get_logger().info(f'ایکزیکیوشن کوشش {attempt + 1}')

            success = self.action_executor.execute_action_sequence(action_sequence)

            if success:
                self.node.get_logger().info('ایکشن ترتیب کامیابی سے مکمل ہو گئی')
                return True

            # ناکامی کی وجہ چیک کریں اور شاید دوبارہ منصوبہ بند کریں
            feedback = self._get_perception_feedback()
            recovery_action = self._generate_recovery_action(feedback, action_sequence)

            if recovery_action:
                self.node.get_logger().info('ریکوری ایکشن انجام دے رہا ہے')
                recovery_success = self.action_executor.execute_action(recovery_action)

                if recovery_success:
                    # اصل ترتیب کو دوبارہ کوشش کریں
                    continue
                else:
                    self.node.get_logger().error('ریکوری ناکام')
                    return False
            else:
                self.node.get_logger().error('کوئی ریکوری ایکشن دستیاب نہیں')
                return False

        return False

    def _get_perception_feedback(self):
        """موجودہ پریسیپشن فیڈ بیک حاصل کریں"""
        if self.perception_handler:
            return self.perception_handler.get_current_perception()
        return None

    def _generate_recovery_action(self, feedback, original_sequence):
        """ناکامی کی بنیاد پر ریکوری ایکشن تخلیق کریں"""
        # یہ ایک سادہ مثال ہے
        # عمل میں، یہ زیادہ ترقی یافتہ خرابی کے تجزیہ کو شامل کرے گا

        if not feedback:
            return None

        # مثال: اگر چیز نہیں ملی، تو دیکھنے کی کوشش کریں
        if 'object_missing' in feedback:
            return GazeAction(
                parameters={'action_type': 'search', 'search_pattern': 'spiral'}
            )

        # مثال: اگر رکاوٹ کی وجہ سے نیویگیشن ناکام ہوا، تو متبادل راستہ کوشش کریں
        if 'obstacle_detected' in feedback:
            return NavigationAction(
                parameters={'action_type': 'avoid_obstacle'}
            )

        return None

class AdaptiveActionPlanner:
    def __init__(self):
        self.action_planner = ActionPlanner()
        self.feedback_history = []

    def plan_with_context(self, high_level_command: Dict[str, Any],
                         context: Dict[str, Any] = None) -> List[Action]:
        """سیاق اور فیڈ بیک تاریخ کو مدنظر رکھتے ہوئے ایکشنز منصوبہ بند کریں"""

        # سیاق کے مطابق کمانڈ کو ایڈجسٹ کریں
        adjusted_command = self._adjust_command_for_context(high_level_command, context)

        # ابتدائی منصوبہ تخلیق کریں
        action_sequence = self.action_planner.plan_action(adjusted_command)

        # فیڈ بیک تاریخ کے مطابق منصوبہ ایڈاپٹ کریں
        adapted_sequence = self._adapt_plan_for_feedback(action_sequence)

        return adapted_sequence

    def _adjust_command_for_context(self, command: Dict[str, Any],
                                  context: Dict[str, Any]) -> Dict[str, Any]:
        """موجودہ سیاق کے مطابق کمانڈ کو ایڈجسٹ کریں"""
        if not context:
            return command

        adjusted_command = command.copy()

        # مثال: اگر چیز کا مقام زیادہ درست طور پر معلوم ہے، تو کمانڈ اپ ڈیٹ کریں
        if 'object_locations' in context and 'object' in command.get('entities', {}):
            known_objects = context['object_locations']
            target_obj = command['entities']['object'][0]

            for obj_id, location in known_objects.items():
                if target_obj.lower() in obj_id.lower():
                    adjusted_command['entities']['object_position'] = location
                    break

        return adjusted_command

    def _adapt_plan_for_feedback(self, action_sequence: List[Action]) -> List[Action]:
        """فیڈ بیک تاریخ کے مطابق ایکشن ترتیب ایڈاپٹ کریں"""
        # یہ گذشتہ ناکامیوں/کامیابیوں سے سیکھنے کو امپلیمنٹ کرے گا
        # فی الحال، اصل ترتیب لوٹائیں
        return action_sequence

    def record_feedback(self, command: Dict[str, Any], result: bool, execution_time: float):
        """سیکھنے کے لیے فیڈ بیک ریکارڈ کریں"""
        feedback = {
            'command': command,
            'result': result,
            'execution_time': execution_time,
            'timestamp': time.time()
        }
        self.feedback_history.append(feedback)
```

## ایکشن ایکزیکیوشن ROS نوڈ

### مکمل ROS 2 امپلیمنٹیشن

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
import json

class ActionExecutionNode(Node):
    def __init__(self):
        super().__init__('action_execution_node')

        # پبلشرز اور سبسکرائبرز
        self.result_pub = self.create_publisher(String, 'action_results', 10)
        self.command_sub = self.create_subscription(
            String, 'parsed_commands', self.command_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, 'detected_objects', self.perception_callback, 10
        )

        # جزوات کو شروع کریں
        self.action_planner = AdaptiveActionPlanner()
        self.action_executor = ActionExecutor(self)
        self.closed_loop_controller = ClosedLoopController(self)

        # موجودہ حالت
        self.current_perception = None
        self.action_queue = []
        self.is_executing = False

        self.get_logger().info("ایکشن ایکزیکیوشن نوڈ شروع ہو گیا")

    def command_callback(self, msg: String):
        """آنے والے کمانڈز کو پروسیس کریں"""
        try:
            command_data = json.loads(msg.data)

            # موجودہ سیاق حاصل کریں
            context = self._get_current_context()

            # ایکشنز منصوبہ بند کریں
            action_sequence = self.action_planner.plan_with_context(
                command_data, context
            )

            self.get_logger().info(
                f"{len(action_sequence)} ایکشنز منصوبہ بند کیے گئے، کمانڈ کے لیے: {command_data.get('intent', 'unknown')}"
            )

            # ایکشنز انجام دیں
            if not self.is_executing:
                self.is_executing = True
                success = self.closed_loop_controller.execute_with_feedback(action_sequence)

                # نتیجہ شائع کریں
                result_msg = String()
                result_msg.data = json.dumps({
                    'command_id': command_data.get('command_id', 'unknown'),
                    'success': success,
                    'timestamp': self.get_clock().now().to_msg()
                })
                self.result_pub.publish(result_msg)

                self.is_executing = False

        except Exception as e:
            self.get_logger().error(f"کمانڈ پروسیس کرنے میں خرابی: {e}")

    def perception_callback(self, msg: String):
        """پریسیپشن سیاق اپ ڈیٹ کریں"""
        try:
            perception_data = json.loads(msg.data)
            self.current_perception = perception_data
        except Exception as e:
            self.get_logger().error(f"پریسیپشن پروسیس کرنے میں خرابی: {e}")

    def _get_current_context(self) -> Dict[str, Any]:
        """منصوبہ بندی کے لیے موجودہ سیاق حاصل کریں"""
        context = {}

        if self.current_perception:
            context['object_locations'] = {}
            for obj in self.current_perception.get('objects', []):
                obj_name = obj.get('name', 'unknown')
                position = obj.get('position', [0, 0, 0])
                if position:  # اگر پوزیشن دستیاب ہے
                    context['object_locations'][obj_name] = position

        return context

def main(args=None):
    rclpy.init(args=args)

    node = ActionExecutionNode()

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

## سیفٹی اور خرابی کا انتظام

### سیفٹی مانیٹر

```python
class SafetyMonitor:
    def __init__(self, node: Node):
        self.node = node
        self.safety_limits = {
            'max_velocity': 0.5,
            'max_force': 100.0,
            'max_torque': 50.0,
            'collision_threshold': 0.1,
            'joint_limits': {
                'min': -2.0,
                'max': 2.0
            }
        }

        # سیفٹی سے متعلق موضوعات کو سبسکرائب کریں
        self.joint_state_sub = node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.scan_sub = node.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        self.current_joint_states = None
        self.current_scan_data = None

    def joint_state_callback(self, msg: JointState):
        """جائنٹ اسٹیٹس کو سیفٹی کے لیے مانیٹر کریں"""
        self.current_joint_states = msg

    def scan_callback(self, msg: LaserScan):
        """کولیژن ڈیٹیکشن کے لیے اسکین ڈیٹا کو مانیٹر کریں"""
        self.current_scan_data = msg

    def check_safety(self, action: Action) -> bool:
        """چیک کریں کہ ایکشن انجام دینے کے لیے محفوظ ہے"""
        # جائنٹ حدود چیک کریں
        if not self._check_joint_limits():
            self.node.get_logger().warn('جائنٹ حدود کی خلاف ورزی کا پتہ چلا')
            return False

        # کولیژن خطرہ چیک کریں
        if not self._check_collision_risk():
            self.node.get_logger().warn('کولیژن خطرہ کا پتہ چلا')
            return False

        # مینیپولیشن کے لیے زور/ٹورک حدود چیک کریں
        if action.action_type == ActionType.MANIPULATION:
            if not self._check_manipulation_safety(action):
                self.node.get_logger().warn('مینیپولیشن سیفٹی خلاف ورزی')
                return False

        return True

    def _check_joint_limits(self) -> bool:
        """چیک کریں کہ جائنٹس محفوظ حدود کے اندر ہیں"""
        if not self.current_joint_states:
            return True  # کوئی ڈیٹا نہیں، محفوظ فرض کریں

        for position in self.current_joint_states.position:
            if (position < self.safety_limits['joint_limits']['min'] or
                position > self.safety_limits['joint_limits']['max']):
                return False

        return True

    def _check_collision_risk(self) -> bool:
        """موجودہ کولیژنز کے لیے چیک کریں"""
        if not self.current_scan_data:
            return True  # کوئی ڈیٹا نہیں، محفوظ فرض کریں

        # حد کے اندر رکاوٹوں کے لیے چیک کریں
        min_distance = min([r for r in self.current_scan_data.ranges if r > 0], default=float('inf'))

        if min_distance < self.safety_limits['collision_threshold']:
            return False

        return True

    def _check_manipulation_safety(self, action: ManipulationAction) -> bool:
        """مینیپولیشن ایکشنز کے لیے سیفٹی چیک کریں"""
        force = action.parameters.get('gripper_force', 0)
        if force > self.safety_limits['max_force']:
            return False

        return True

class ActionExecutionManager:
    def __init__(self, node: Node):
        self.node = node
        self.action_executor = ActionExecutor(node)
        self.safety_monitor = SafetyMonitor(node)
        self.recovery_manager = RecoveryManager(node)

    def execute_safe_action(self, action: Action) -> bool:
        """سیفٹی چیکس کے ساتھ ایکشن انجام دیں"""
        # ایکزیکیوشن سے پہلے سیفٹی چیک کریں
        if not self.safety_monitor.check_safety(action):
            self.node.get_logger().error('سیفٹی چیک ناکام، ایکشن منسوخ کر رہا ہے')
            return False

        # ایکشن انجام دیں
        success = self.action_executor.execute_action(action)

        # اگر ایکشن ناکام ہوا، تو ریکوری کی کوشش کریں
        if not success:
            recovery_success = self.recovery_manager.attempt_recovery(action)
            if recovery_success:
                # ریکوری کے بعد اصل ایکشن دوبارہ کوشش کریں
                return self.action_executor.execute_action(action)

        return success

class RecoveryManager:
    def __init__(self, node: Node):
        self.node = node
        self.recovery_strategies = {
            'collision': self._handle_collision,
            'timeout': self._handle_timeout,
            'joint_limit': self._handle_joint_limit,
            'gripper_failure': self._handle_gripper_failure
        }

    def attempt_recovery(self, failed_action: Action) -> bool:
        """ایکشن ناکامی سے بازیافت کی کوشش کریں"""
        # ناکامی کی قسم کا تعین کریں اور مناسب ریکوری لاگو کریں
        failure_type = self._analyze_failure(failed_action)

        if failure_type in self.recovery_strategies:
            return self.recovery_strategies[failure_type](failed_action)

        return False

    def _analyze_failure(self, action: Action) -> str:
        """تعین کریں کہ کیا قسم کی ناکامی ہوئی"""
        # یہ سسٹم فیڈ بیک کی بنیاد پر امپلیمنٹ کیا جائے گا
        # فی الحال، ایک ڈیفالٹ ناکامی قسم لوٹائیں
        return 'timeout'  # مثال کے لیے ڈیفالٹ کو ٹائم آؤٹ

    def _handle_collision(self, action: Action) -> bool:
        """کولیژن ناکامی کو ہینڈل کریں"""
        self.node.get_logger().info('کولیژن ریکوری کی کوشش کر رہا ہے')
        # کولیژن ریکوری لاگک کو امپلیمنٹ کریں
        return True

    def _handle_timeout(self, action: Action) -> bool:
        """ٹائم آؤٹ ناکامی کو ہینڈل کریں"""
        self.node.get_logger().info('ٹائم آؤٹ ریکوری کی کوشش کر رہا ہے')
        # ٹائم آؤٹ ریکوری لاگک کو امپلیمنٹ کریں
        return True

    def _handle_joint_limit(self, action: Action) -> bool:
        """جائنٹ حدود ناکامی کو ہینڈل کریں"""
        self.node.get_logger().info('جائنٹ حدود ریکوری کی کوشش کر رہا ہے')
        # جائنٹ حدود ریکوری لاگک کو امپلیمنٹ کریں
        return True

    def _handle_gripper_failure(self, action: Action) -> bool:
        """گریپر ناکامی کو ہینڈل کریں"""
        self.node.get_logger().info('گریپر ناکامی ریکوری کی کوشش کر رہا ہے')
        # گریپر ناکامی ریکوری لاگک کو امپلیمنٹ کریں
        return True
```

## ٹیسٹنگ اور تصدیق

### یونٹ ٹیسٹس

```python
import unittest
from unittest.mock import Mock, MagicMock

class TestActionExecution(unittest.TestCase):
    def setUp(self):
        self.action_planner = ActionPlanner()
        self.navigation_controller = Mock()
        self.manipulation_controller = Mock()

    def test_action_planning(self):
        """بلند سطحی کمانڈز سے ایکشن پلاننگ کا ٹیسٹ کریں"""
        command = {
            'intent': 'navigate',
            'entities': {'location': ['kitchen']}
        }

        actions = self.action_planner.plan_action(command)

        self.assertIsInstance(actions, list)
        self.assertGreater(len(actions), 0)
        self.assertEqual(actions[0].action_type, ActionType.NAVIGATION)

    def test_grasp_sequence_planning(self):
        """گریسپ ایکشن ترتیب منصوبہ بندی کا ٹیسٹ کریں"""
        command = {
            'intent': 'grasp',
            'entities': {
                'target_object': ['red_ball'],
                'object_position': [1.0, 2.0, 0.5]
            }
        }

        actions = self.action_planner.plan_action(command)

        # نیویگیشن اور مینیپولیشن ایکشنز پر مشتمل ہونا چاہیے
        nav_actions = [a for a in actions if a.action_type == ActionType.NAVIGATION]
        manip_actions = [a for a in actions if a.action_type == ActionType.MANIPULATION]

        self.assertGreater(len(nav_actions), 0)
        self.assertGreater(len(manip_actions), 0)

    def test_action_executor_initialization(self):
        """ایکشن ایکزیکیوٹر کی شروعات کا ٹیسٹ کریں"""
        node = Mock()
        executor = ActionExecutor(node)

        self.assertIsNotNone(executor.navigation_controller)
        self.assertIsNotNone(executor.manipulation_controller)
        self.assertEqual(executor.execution_status, ExecutionStatus.PENDING)

    def test_navigation_action_execution(self):
        """نیویگیشن ایکشن ایکزیکیوشن کا ٹیسٹ کریں"""
        node = Mock()
        executor = ActionExecutor(node)

        # نیویگیشن کنٹرولر کو مocker کریں
        executor.navigation_controller.navigate_to_pose = Mock(return_value=True)

        action = NavigationAction(
            parameters={'target_position': [1.0, 2.0, 0.0]}
        )

        success = executor.execute_action(action)

        self.assertTrue(success)
        executor.navigation_controller.navigate_to_pose.assert_called_once()

    def test_manipulation_action_execution(self):
        """مینیپولیشن ایکشن ایکزیکیوشن کا ٹیسٹ کریں"""
        node = Mock()
        executor = ActionExecutor(node)

        # مینیپولیشن کنٹرولر کو مocker کریں
        executor.manipulation_controller.grasp_object = Mock(return_value=True)

        action = ManipulationAction(
            parameters={
                'action_type': 'grasp_object',
                'position': [1.0, 2.0, 0.5],
                'gripper_force': 50
            }
        )

        success = executor.execute_action(action)

        self.assertTrue(success)
        executor.manipulation_controller.grasp_object.assert_called_once()

    def test_action_sequence_execution(self):
        """ایکشن ترتیب ایکزیکیوشن کا ٹیسٹ کریں"""
        node = Mock()
        executor = ActionExecutor(node)

        # کنٹرولرز کو مocker کریں
        executor.navigation_controller.navigate_to_pose = Mock(return_value=True)
        executor.manipulation_controller.grasp_object = Mock(return_value=True)

        actions = [
            NavigationAction(parameters={'target_position': [1.0, 2.0, 0.0]}),
            ManipulationAction(parameters={'action_type': 'grasp_object', 'position': [1.0, 2.0, 0.5]})
        ]

        success = executor.execute_action_sequence(actions)

        self.assertTrue(success)

    def test_safety_monitor(self):
        """سیفٹی مانیٹرنگ کا ٹیسٹ کریں"""
        node = Mock()
        safety_monitor = SafetyMonitor(node)

        # جائنٹ حدود چیک
        result = safety_monitor._check_joint_limits()
        self.assertTrue(result)  # کوئی ڈیٹا نہ ہونے پر سچ لوٹانا چاہیے (محفوظ فرض)

        # کولیژن چیک
        result = safety_monitor._check_collision_risk()
        self.assertTrue(result)  # کوئی ڈیٹا نہ ہونے پر سچ لوٹانا چاہیے (محفوظ فرض)

if __name__ == '__main__':
    unittest.main()
```

## کنفیگریشن اور سیٹ اپ

### کنفیگریشن فائل

```yaml
# config/action_execution.yaml
action_execution:
  planning:
    max_action_sequence_length: 20
    default_timeout: 30.0
    replan_on_failure: true
    max_replan_attempts: 3

  navigation:
    default_speed: 0.5
    arrival_threshold: 0.2
    obstacle_avoidance: true
    max_velocity: 0.8

  manipulation:
    default_gripper_force: 50
    approach_distance: 0.1
    release_force: 0
    max_force: 100.0

  safety:
    enable_safety_monitor: true
    max_collision_threshold: 0.1
    joint_limit_margin: 0.1
    velocity_limit: 0.5

  execution:
    enable_feedback: true
    enable_recovery: true
    recovery_attempts: 3
    enable_adaptation: true
```

### لانچ فائل

```xml
<!-- launch/action_execution.launch.xml -->
<launch>
  <node pkg="your_robot_package" exec="action_execution_node" name="action_execution">
    <param name="default_timeout" value="30.0"/>
    <param name="enable_safety_monitor" value="true"/>
  </node>
</launch>
```

## ٹربل شوٹنگ

### عام مسائل

1. **ایکشن ایکزیکیوشن ٹائم آؤٹس**
   - حل: ٹائم آؤٹ ویلیوز بڑھائیں یا ایکشن ایکزیکیوشن کی رفتار کو اصلاح کریں
   - چیک کریں: یقینی بنائیں کہ روبوٹ ہارڈویئر جوابدہ ہے اور مناسب طریقے سے کیلیبریٹ ہے

2. **نیویگیشن ناکامیاں**
   - حل: میپ کی کوالٹی اور لوکلائزیشن کی درستگی کی تصدیق کریں
   - چیک کریں: یقینی بنائیں کہ نیویگیشن اسٹیک مناسب طریقے سے کنفیگر ہے

3. **مینیپولیشن ناکامیاں**
   - حل: انورس کنیمیٹکس حل اور جائنٹ حدود چیک کریں
   - چیک کریں: چیز کے ڈیٹیکشن کی درستگی اور پوز ایسٹیمیشن کی تصدیق کریں

4. **سیفٹی سسٹم مداخلت**
   - حل: سیفٹی پیرامیٹرز مناسب طریقے سے ایڈجسٹ کریں
   - چیک کریں: یقینی بنائیں کہ سیفٹی تھریشولڈز آپ کے روبوٹ کے لیے مناسب ہیں

## اگلے اقدامات

اگلے سیکشن میں، ہم تمام تین جزوات - وژن، زبان، اور ایکشن - کو ایک مربوط ملٹی ماڈل فیوژن سسٹم میں انٹیگریٹ کریں گے۔ ہم نے جو انفرادی جزوات تعمیر کیے ہیں انہیں ایک ساتھ جوڑ کر ایک مکمل وژن لینگویج ایکشن سسٹم تخلیق کیا جائے گا جو قدرتی زبان کے کمانڈز کو سمجھنے، ماحول کو ویژوئلی تصور کرنے، اور مناسب ایکشنز انجام دینے کے قابل ہوگا۔

جاری رکھیں [ملٹی ماڈل فیوژن](../multi-modal-fusion/index.md).