---
sidebar_position: 2
title: "ورکشاپ 4.2: روبوٹ کنٹرول اور مینیپولیشن"
---

# ورکشاپ 4.2: روبوٹ کنٹرول اور مینیپولیشن

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے روبوٹ کنٹرول اور مینیپولیشن کے سسٹم کو نافذ کریں، جس سے روبوٹ کو مطلوبہ ایکشنز کو فزیکلی انجام دینے کی صلاحیت حاصل ہو۔

## ضروریات
- Python 3.10+
- ROS 2 ہمبل ہاکسبل نصب ہو
- MoveIt 2 نصب ہو
- URDF ماڈل کا علم
- کنٹرول تھیوری کا بنیادی علم
- پچھلا ورکشاپ مکمل ہو

## ورکشاپ کے اقدامات

### اقدام 1: روبوٹ کنٹرول ایریا کو نافذ کریں
ایک نیا فائل `robot_controller.py` تخلیق کریں:

```python
#!/usr/bin/env python3
from typing import Dict, Any, Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import json

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # سبسکرائبرز
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.action_command_subscriber = self.create_subscription(
            String, 'robot_actions', self.action_command_callback, 10
        )

        # پبلیشرز
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10
        )
        self.gripper_command_publisher = self.create_publisher(
            String, 'gripper_command', 10
        )
        self.robot_status_publisher = self.create_publisher(
            String, 'robot_status', 10
        )

        # روبوٹ کی حالت کو ٹریک کریں
        self.current_joint_positions = {}
        self.current_pose = None
        self.is_busy = False

        self.get_logger().info('Robot Controller Node initialized')

    def joint_state_callback(self, msg: JointState):
        """جائنٹ اسٹیٹس کو اپ ڈیٹ کریں"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def action_command_callback(self, msg):
        """روبوٹ ایکشن کمانڈز کو پروسیس کریں"""
        if self.is_busy:
            self.get_logger().warn('Robot is busy, ignoring new command')
            return

        try:
            command_data = json.loads(msg.data)
            action_type = command_data.get('action_type', '')

            self.is_busy = True
            self.get_logger().info(f'Executing robot action: {action_type}')

            success = self.execute_robot_action(command_data)

            # اسٹیٹس پبلش کریں
            status_msg = String()
            status_msg.data = json.dumps({
                'action_type': action_type,
                'success': success,
                'timestamp': self.get_clock().now().nanoseconds
            })
            self.robot_status_publisher.publish(status_msg)

            self.is_busy = False

        except Exception as e:
            self.get_logger().error(f'Error executing robot action: {e}')
            self.is_busy = False

    def execute_robot_action(self, command_data: Dict[str, Any]) -> bool:
        """روبوٹ ایکشن کو ایکزیکیوٹ کریں"""
        action_type = command_data.get('action_type', '')

        if action_type == 'move_to_pose':
            return self.execute_move_to_pose(command_data)
        elif action_type == 'pick_object':
            return self.execute_pick_object(command_data)
        elif action_type == 'place_object':
            return self.execute_place_object(command_data)
        elif action_type == 'move_joints':
            return self.execute_move_joints(command_data)
        elif action_type == 'open_gripper':
            return self.execute_gripper_action('open')
        elif action_type == 'close_gripper':
            return self.execute_gripper_action('close')
        else:
            self.get_logger().error(f'Unknown robot action: {action_type}')
            return False

    def execute_move_to_pose(self, command_data: Dict[str, Any]) -> bool:
        """پوز میں موو کریں"""
        target_pose_data = command_data.get('target_pose', {})
        if not target_pose_data:
            self.get_logger().error('Missing target_pose in move_to_pose command')
            return False

        # ہم یہاں IK سالور استعمال کریں گے
        target_pose = Pose()
        pos = target_pose_data.get('position', {})
        quat = target_pose_data.get('orientation', {})

        target_pose.position.x = pos.get('x', 0.0)
        target_pose.position.y = pos.get('y', 0.0)
        target_pose.position.z = pos.get('z', 0.0)
        target_pose.orientation.x = quat.get('x', 0.0)
        target_pose.orientation.y = quat.get('y', 0.0)
        target_pose.orientation.z = quat.get('z', 0.0)
        target_pose.orientation.w = quat.get('w', 1.0)

        # IK حل حاصل کریں (سادہ مثال کے لیے)
        joint_positions = self.calculate_ik_solution(target_pose)
        if joint_positions is None:
            self.get_logger().error('Failed to calculate IK solution')
            return False

        # جائنٹ ٹریجکٹری کو پبلش کریں
        return self.publish_joint_trajectory(joint_positions)

    def execute_pick_object(self, command_data: Dict[str, Any]) -> bool:
        """آبجیکٹ اٹھائیں"""
        object_pose_data = command_data.get('object_pose', {})
        if not object_pose_data:
            self.get_logger().error('Missing object_pose in pick_object command')
            return False

        # 1. اپروچ پوز پر جائیں
        approach_pose = self.calculate_approach_pose(object_pose_data)
        if not self.execute_move_to_pose({'target_pose': approach_pose}):
            return False

        # 2. گریپ پوز پر جائیں
        grasp_pose = object_pose_data
        if not self.execute_move_to_pose({'target_pose': grasp_pose}):
            return False

        # 3. گریپر کو بند کریں
        if not self.execute_gripper_action('close'):
            return False

        # 4. لفٹ کریں
        lift_pose = self.calculate_lift_pose(grasp_pose)
        if not self.execute_move_to_pose({'target_pose': lift_pose}):
            return False

        return True

    def execute_place_object(self, command_data: Dict[str, Any]) -> bool:
        """آبجیکٹ رکھیں"""
        target_pose_data = command_data.get('target_pose', {})
        if not target_pose_data:
            self.get_logger().error('Missing target_pose in place_object command')
            return False

        # 1. ٹارگٹ پوز پر جائیں
        if not self.execute_move_to_pose({'target_pose': target_pose_data}):
            return False

        # 2. گریپر کو کھولیں
        if not self.execute_gripper_action('open'):
            return False

        # 3. اپ ریٹر کریں
        current_pose = self.get_current_pose()
        if current_pose:
            retract_pose = self.calculate_retract_pose(current_pose)
            self.execute_move_to_pose({'target_pose': retract_pose})

        return True

    def execute_move_joints(self, command_data: Dict[str, Any]) -> bool:
        """جائنٹس کو موو کریں"""
        joint_positions = command_data.get('joint_positions', {})
        if not joint_positions:
            self.get_logger().error('Missing joint_positions in move_joints command')
            return False

        return self.publish_joint_trajectory(joint_positions)

    def execute_gripper_action(self, action: str) -> bool:
        """گریپر ایکشن انجام دیں"""
        command_msg = String()
        command_msg.data = json.dumps({
            'action': action,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.gripper_command_publisher.publish(command_msg)
        self.get_logger().info(f'Published gripper command: {action}')

        # وقت کے لیے انتظار کریں
        import time
        time.sleep(1.0)

        return True

    def calculate_ik_solution(self, target_pose: Pose) -> Optional[Dict[str, float]]:
        """IK حل حساب کریں (سادہ مثال)"""
        # یہ ایک جنرل IK حل نہیں ہے، صرف مثال کے لیے
        # اصل میں آپ MoveIt 2 کا استعمال کریں گے
        try:
            # سادہ کیلکولیشن (اصل میں MoveIt کا استعمال کریں)
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_positions = {name: 0.0 for name in joint_names}

            # یہاں اصل IK کیلکولیشن ہوگی
            # ہم صرف ایک جنرک حل واپس کر رہے ہیں
            return joint_positions
        except Exception:
            return None

    def calculate_approach_pose(self, object_pose: Dict[str, Any]) -> Dict[str, Any]:
        """اپروچ پوز کا حساب کریں"""
        approach_offset = 0.1  # 10cm اپروچ
        approach_pose = object_pose.copy()

        # ایکس ایکسس میں آبجیکٹ سے پیچھے کی طرف
        approach_pose['position']['x'] -= approach_offset

        return approach_pose

    def calculate_lift_pose(self, grasp_pose: Dict[str, Any]) -> Dict[str, Any]:
        """لیفٹ پوز کا حساب کریں"""
        lift_offset = 0.1  # 10cm اوپر
        lift_pose = grasp_pose.copy()

        # Z ایکسس میں اوپر کی طرف
        lift_pose['position']['z'] += lift_offset

        return lift_pose

    def calculate_retract_pose(self, current_pose: Dict[str, Any]) -> Dict[str, Any]:
        """ری ٹریکٹ پوز کا حساب کریں"""
        retract_offset = 0.1  # 10cm پیچھے
        retract_pose = current_pose.copy()

        # ایکس ایکسس میں پیچھے کی طرف
        retract_pose['position']['x'] -= retract_offset

        return retract_pose

    def get_current_pose(self) -> Optional[Dict[str, Any]]:
        """موجودہ پوز حاصل کریں"""
        # یہاں MoveIt کے ذریعے موجودہ پوز حاصل کی جائے گی
        # مثال کے لیے صرف جنرک پوز واپس کر رہے ہیں
        return {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }

    def publish_joint_trajectory(self, joint_positions: Dict[str, float]) -> bool:
        """جائنٹ ٹریجکٹری کو پبلش کریں"""
        try:
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = list(joint_positions.keys())

            point = JointTrajectoryPoint()
            point.positions = list(joint_positions.values())
            point.time_from_start.sec = 2  # 2 سیکنڈ میں پوزیشن تک جائیں
            point.time_from_start.nanosec = 0

            trajectory_msg.points = [point]

            self.joint_trajectory_publisher.publish(trajectory_msg)
            self.get_logger().info(f'Published joint trajectory: {joint_positions}')

            return True
        except Exception as e:
            self.get_logger().error(f'Error publishing joint trajectory: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    controller = RobotControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 2: MoveIt 2 انٹیگریشن کو نافذ کریں
ایک فائل `moveit_integration.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes, RobotState, Constraints
from moveit_msgs.srv import GetMotionPlan
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
import json
import time

class MoveItIntegrationNode(Node):
    def __init__(self):
        super().__init__('moveit_integration_node')

        # سبسکرائبرز
        self.motion_command_subscriber = self.create_subscription(
            String, 'motion_commands', self.motion_command_callback, 10
        )

        # سروس کلائنٹس
        self.get_motion_plan_client = self.create_client(
            GetMotionPlan, 'plan_kinematic_path'
        )

        # انتظار کریں کہ سروس دستیاب ہو
        while not self.get_motion_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Motion planning service not available, waiting...')

        self.get_logger().info('MoveIt Integration Node initialized')

    def motion_command_callback(self, msg):
        """موشن کمانڈز کو پروسیس کریں"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command_type', '')

            if command_type == 'plan_to_pose':
                success = self.plan_to_pose(command_data)
            elif command_type == 'plan_to_joints':
                success = self.plan_to_joints(command_data)
            else:
                self.get_logger().error(f'Unknown motion command: {command_type}')
                success = False

            self.get_logger().info(f'Motion planning result: {success}')

        except Exception as e:
            self.get_logger().error(f'Error in motion command: {e}')

    def plan_to_pose(self, command_data: Dict[str, Any]) -> bool:
        """پوز تک پلان تیار کریں"""
        target_pose_data = command_data.get('target_pose', {})
        if not target_pose_data:
            self.get_logger().error('Missing target_pose in plan_to_pose')
            return False

        # ٹارگٹ پوز تیار کریں
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = self.get_clock().now().to_msg()

        pos = target_pose_data.get('position', {})
        quat = target_pose_data.get('orientation', {})

        target_pose.pose.position.x = pos.get('x', 0.0)
        target_pose.pose.position.y = pos.get('y', 0.0)
        target_pose.pose.position.z = pos.get('z', 0.0)
        target_pose.pose.orientation.x = quat.get('x', 0.0)
        target_pose.pose.orientation.y = quat.get('y', 0.0)
        target_pose.pose.orientation.z = quat.get('z', 0.0)
        target_pose.pose.orientation.w = quat.get('w', 1.0)

        # پلان کی درخواست تیار کریں
        request = GetMotionPlan.Request()
        # یہاں مکمل درخواست تیار کی جائے گی
        # MoveIt 2 کے لیے مخصوص کوڈ ہوگا

        # اصل میں سروس کال کریں
        # future = self.get_motion_plan_client.call_async(request)
        # یہاں مستقبل کا نتیجہ ہینڈل کیا جائے گا

        # مثال کے لیے ہم صرف True واپس کر رہے ہیں
        return True

    def plan_to_joints(self, command_data: Dict[str, Any]) -> bool:
        """جائنٹس تک پلان تیار کریں"""
        target_joints = command_data.get('target_joints', {})
        if not target_joints:
            self.get_logger().error('Missing target_joints in plan_to_joints')
            return False

        # یہاں جائنٹ ویلیوز کے لیے پلان تیار کیا جائے گا
        # MoveIt 2 کے لیے مخصوص کوڈ ہوگا

        return True

def main(args=None):
    rclpy.init(args=args)
    moveit_node = MoveItIntegrationNode()

    try:
        rclpy.spin(moveit_node)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 3: کنٹرول سیفٹی اور مانیٹرنگ کو نافذ کریں
ایک فائل `control_safety.py` تخلیق کریں:

```python
#!/usr/bin/env python3
from typing import Dict, Any, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Temperature
from std_msgs.msg import String, Bool
from geometry_msgs.msg import WrenchStamped
import json
import numpy as np

class ControlSafetyNode(Node):
    def __init__(self):
        super().__init__('control_safety_node')

        # سبسکرائبرز
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.force_torque_subscriber = self.create_subscription(
            WrenchStamped, 'wrench_sensor', self.force_torque_callback, 10
        )
        self.temperature_subscriber = self.create_subscription(
            Temperature, 'temperature_sensors', self.temperature_callback, 10
        )
        self.emergency_stop_subscriber = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10
        )

        # پبلیشرز
        self.safety_status_publisher = self.create_publisher(
            String, 'safety_status', 10
        )
        self.emergency_stop_publisher = self.create_publisher(
            Bool, 'emergency_stop_command', 10
        )

        # سیفٹی پیرامیٹرز
        self.joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-2.0, 2.0),
            'joint3': (-3.14, 3.14),
            'joint4': (-2.0, 2.0),
            'joint5': (-3.14, 3.14),
            'joint6': (-2.0, 2.0)
        }

        self.max_temperature = 70.0  # Celsius
        self.max_force = 100.0  # Newtons
        self.max_torque = 50.0  # Nm

        # روبوٹ کی حالت کو ٹریک کریں
        self.current_joint_positions = {}
        self.current_temperatures = {}
        self.current_force_torque = None
        self.emergency_stop_active = False

        self.get_logger().info('Control Safety Node initialized')

    def joint_state_callback(self, msg: JointState):
        """جائنٹ اسٹیٹس کو چیک کریں"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

                # جائنٹ لیمٹس چیک کریں
                if name in self.joint_limits:
                    min_limit, max_limit = self.joint_limits[name]
                    if not (min_limit <= msg.position[i] <= max_limit):
                        self.get_logger().warn(f'Joint {name} out of limits: {msg.position[i]}')
                        self.trigger_safety_stop(f'Joint {name} out of limits')

    def force_torque_callback(self, msg: WrenchStamped):
        """فورس ٹورک کو چیک کریں"""
        force_magnitude = np.sqrt(
            msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2
        )
        torque_magnitude = np.sqrt(
            msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2
        )

        if force_magnitude > self.max_force or torque_magnitude > self.max_torque:
            self.get_logger().warn(f'Force/Torque limits exceeded: F={force_magnitude}, T={torque_magnitude}')
            self.trigger_safety_stop('Force/Torque limits exceeded')

        self.current_force_torque = msg

    def temperature_callback(self, msg: Temperature):
        """ٹیمپریچر کو چیک کریں"""
        sensor_name = msg.header.frame_id if msg.header.frame_id else 'unknown'
        self.current_temperatures[sensor_name] = msg.temperature

        if msg.temperature > self.max_temperature:
            self.get_logger().warn(f'Temperature limit exceeded: {msg.temperature}°C')
            self.trigger_safety_stop(f'Temperature limit exceeded: {sensor_name}')

    def emergency_stop_callback(self, msg: Bool):
        """ایمرجنسی سٹاپ کو ہینڈل کریں"""
        if msg.data and not self.emergency_stop_active:
            self.get_logger().info('Emergency stop activated')
            self.emergency_stop_active = True
            self.publish_emergency_stop(True)

    def trigger_safety_stop(self, reason: str):
        """سیفٹی سٹاپ ٹریگر کریں"""
        if not self.emergency_stop_active:
            self.get_logger().error(f'Safety stop triggered: {reason}')
            self.emergency_stop_active = True
            self.publish_emergency_stop(True)

            # سیفٹی اسٹیٹس پبلش کریں
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'safety_stop',
                'reason': reason,
                'timestamp': self.get_clock().now().nanoseconds
            })
            self.safety_status_publisher.publish(status_msg)

    def publish_emergency_stop(self, active: bool):
        """ایمرجنسی سٹاپ کمانڈ پبلش کریں"""
        stop_msg = Bool()
        stop_msg.data = active
        self.emergency_stop_publisher.publish(stop_msg)

    def reset_safety_system(self):
        """سیفٹی سسٹم کو ری سیٹ کریں"""
        if self.emergency_stop_active:
            self.get_logger().info('Safety system reset')
            self.emergency_stop_active = False
            self.publish_emergency_stop(False)

def main(args=None):
    rclpy.init(args=args)
    safety_node = ControlSafetyNode()

    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- روبوٹ کنٹرول سسٹم کو نافذ کیا
- MoveIt 2 انٹیگریشن کا ڈھانچہ تیار کیا
- سیفٹی اور مانیٹرنگ سسٹم کو نافذ کیا
- مختلف ایکشنز کو فزیکلی انجام دینے کے لیے کنٹرول لاجک تیار کی

یہ سسٹم وژن لینگویج ایکشن فریم ورک کے فزیکل ایکزیکیوشن کا حصہ ہے جہاں روبوٹ کے مینیپولیٹر اور نیویگیشن سسٹم کو کمانڈز کے مطابق کنٹرول کیا جاتا ہے۔