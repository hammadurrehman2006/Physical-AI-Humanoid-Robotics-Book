---
sidebar_position: 1
title: "ورکشاپ 4.1: ایکشن پلاننگ اور نمائندگی"
---

# ورکشاپ 4.1: ایکشن پلاننگ اور نمائندگی

## مقصد
وژن لینگویج ایکشن سسٹم کے لیے ایکشن پلاننگ اور نمائندگی کے سسٹم کو نافذ کریں، روبوٹ کو بلند سطحی کمانڈز کو ایکزیکوٹبل ایکشن سیکوئنس میں تبدیل کرنے کے قابل بنائیں۔

## ضروریات
- Python 3.10+
- ROS 2 ہمبل ہاکسبل نصب ہو
- روبوٹ کنیمیٹکس اور ڈائنا مکس کی سمجھ
- موشن پلاننگ کے تصورات کا بنیادی علم
- پچھلے مراحل (وژن اور لینگویج) مکمل ہوں

## ورکشاپ کے اقدامات

### اقدام 1: ایکشن پلاننگ کا ماحول تیار کریں
ایک نیا فائل `action_planner.py` تخلیق کریں:

```python
#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import json

@dataclass
class Action:
    """ایکشن کی نمائندگی کے لیے ڈیٹا کلاس"""
    action_type: str
    parameters: Dict[str, Any]
    priority: int = 1
    duration: Optional[float] = None

@dataclass
class ActionSequence:
    """ایکشن سیکوئنس کی نمائندگی کے لیے ڈیٹا کلاس"""
    actions: List[Action]
    sequence_id: str
    description: str

class ActionPlannerNode(Node):
    def __init__(self):
        super().__init__('action_planner_node')
        self.action_sequence_publisher = self.create_publisher(
            String, 'action_sequences', 10
        )
        self.command_subscription = self.create_subscription(
            String, 'high_level_commands', self.command_callback, 10
        )

        # ایکشن ٹیمپلیٹس کو ذخیرہ کریں
        self.action_templates = self.load_action_templates()

        self.get_logger().info('Action Planner Node initialized')

    def load_action_templates(self) -> Dict[str, Any]:
        """ایکشن ٹیمپلیٹس کو لوڈ کریں"""
        return {
            'move_to': {
                'required_params': ['target_pose'],
                'default_duration': 5.0
            },
            'pick_object': {
                'required_params': ['object_id', 'grasp_pose'],
                'default_duration': 10.0
            },
            'place_object': {
                'required_params': ['target_pose', 'object_id'],
                'default_duration': 8.0
            },
            'inspect_object': {
                'required_params': ['object_id', 'inspection_pose'],
                'default_duration': 6.0
            }
        }

    def command_callback(self, msg):
        """ہائی لیول کمانڈز کو پلاننگ کریں"""
        try:
            command_data = json.loads(msg.data)
            action_sequence = self.plan_action_sequence(command_data)

            # ایکشن سیکوئنس کو پبلش کریں
            sequence_msg = String()
            sequence_msg.data = json.dumps({
                'sequence_id': action_sequence.sequence_id,
                'actions': [
                    {
                        'action_type': action.action_type,
                        'parameters': action.parameters,
                        'priority': action.priority,
                        'duration': action.duration
                    } for action in action_sequence.actions
                ],
                'description': action_sequence.description
            })

            self.action_sequence_publisher.publish(sequence_msg)
            self.get_logger().info(f'Published action sequence: {action_sequence.sequence_id}')

        except Exception as e:
            self.get_logger().error(f'Error planning action sequence: {e}')

    def plan_action_sequence(self, command_data: Dict[str, Any]) -> ActionSequence:
        """کمانڈ ڈیٹا سے ایکشن سیکوئنس پلان کریں"""
        command_type = command_data.get('command_type', '')
        command_params = command_data.get('parameters', {})

        actions = []

        if command_type == 'move_and_pick':
            # روبوٹ کو ٹارگٹ پوزیشن تک لے جائیں
            move_action = Action(
                action_type='move_to',
                parameters={
                    'target_pose': command_params.get('navigation_target')
                },
                duration=5.0
            )
            actions.append(move_action)

            # آبجیکٹ کو اٹھائیں
            pick_action = Action(
                action_type='pick_object',
                parameters={
                    'object_id': command_params.get('object_id'),
                    'grasp_pose': command_params.get('grasp_pose')
                },
                duration=10.0
            )
            actions.append(pick_action)

        elif command_type == 'inspect_and_report':
            # آبجیکٹ کو انسپیکٹ کریں
            inspect_action = Action(
                action_type='inspect_object',
                parameters={
                    'object_id': command_params.get('object_id'),
                    'inspection_pose': command_params.get('inspection_pose')
                },
                duration=6.0
            )
            actions.append(inspect_action)

        elif command_type == 'move_place':
            # روبوٹ کو ٹارگٹ پوزیشن تک لے جائیں
            move_action = Action(
                action_type='move_to',
                parameters={
                    'target_pose': command_params.get('navigation_target')
                },
                duration=5.0
            )
            actions.append(move_action)

            # آبجیکٹ کو رکھیں
            place_action = Action(
                action_type='place_object',
                parameters={
                    'target_pose': command_params.get('placement_target'),
                    'object_id': command_params.get('object_id')
                },
                duration=8.0
            )
            actions.append(place_action)

        else:
            # ڈیفالٹ: کوئی بھی کمانڈ کو اسکیلیٹن ایکشن میں تبدیل کریں
            actions = self.create_skeleton_actions(command_type, command_params)

        return ActionSequence(
            actions=actions,
            sequence_id=f'seq_{command_type}_{int(self.get_clock().now().nanoseconds)}',
            description=f'Action sequence for {command_type}'
        )

    def create_skeleton_actions(self, command_type: str, params: Dict[str, Any]) -> List[Action]:
        """کمانڈ ٹائپ کے لیے اسکیلیٹن ایکشنز تخلیق کریں"""
        # یہ ایک جنرل فنکشن ہے جو نامعلوم کمانڈز کے لیے اسکیلیٹن ایکشنز تیار کرتا ہے
        skeleton_actions = []

        # کمانڈ کے پیرامیٹر کے اساس پر ایکشنز کو تخلیق کریں
        if 'target_pose' in params:
            move_action = Action(
                action_type='move_to',
                parameters={'target_pose': params['target_pose']},
                duration=5.0
            )
            skeleton_actions.append(move_action)

        # دیگر پیرامیٹرز کے لیے بھی اسی طرح کریں
        return skeleton_actions

def main(args=None):
    rclpy.init(args=args)
    action_planner = ActionPlannerNode()

    try:
        rclpy.spin(action_planner)
    except KeyboardInterrupt:
        pass
    finally:
        action_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 2: ایکشن پلاننگ کو ٹیسٹ کریں
ایک ٹیسٹ فائل `test_action_planner.py` تخلیق کریں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ActionPlannerTester(Node):
    def __init__(self):
        super().__init__('action_planner_tester')
        self.command_publisher = self.create_publisher(String, 'high_level_commands', 10)
        self.action_sequence_subscriber = self.create_subscription(
            String, 'action_sequences', self.action_sequence_callback, 10
        )
        self.test_results = []

    def action_sequence_callback(self, msg):
        """ایکشن سیکوئنس کو وصول کریں اور ٹیسٹ کریں"""
        try:
            sequence_data = json.loads(msg.data)
            self.get_logger().info(f'Received action sequence: {sequence_data["sequence_id"]}')
            self.test_results.append(sequence_data)
        except Exception as e:
            self.get_logger().error(f'Error processing action sequence: {e}')

    def test_move_and_pick(self):
        """موو اینڈ پک کمانڈ کو ٹیسٹ کریں"""
        command = {
            'command_type': 'move_and_pick',
            'parameters': {
                'navigation_target': {
                    'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'object_id': 'red_cube',
                'grasp_pose': {
                    'position': {'x': 1.1, 'y': 2.1, 'z': 0.1},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
        }

        cmd_msg = String()
        cmd_msg.data = json.dumps(command)
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info('Published move_and_pick command')

    def test_inspect_and_report(self):
        """انسپیکٹ اینڈ رپورٹ کمانڈ کو ٹیسٹ کریں"""
        command = {
            'command_type': 'inspect_and_report',
            'parameters': {
                'object_id': 'blue_sphere',
                'inspection_pose': {
                    'position': {'x': 0.5, 'y': 1.5, 'z': 0.2},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
        }

        cmd_msg = String()
        cmd_msg.data = json.dumps(command)
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info('Published inspect_and_report command')

def main(args=None):
    rclpy.init(args=args)
    tester = ActionPlannerTester()

    # چند ٹیسٹ کریں
    time.sleep(1)  # انتظار کریں کہ سبسکرائب کر لیا جائے
    tester.test_move_and_pick()
    time.sleep(2)
    tester.test_inspect_and_report()

    # 5 سیکنڈ تک انتظار کریں کہ نتائج موصول ہو جائیں
    time.sleep(5)

    tester.get_logger().info(f'Test completed. Received {len(tester.test_results)} action sequences')

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اقدام 3: ایکشن سیکوئنس ایکزیکیوٹر کو نافذ کریں
ایک فائل `action_executor.py` تخلیق کریں:

```python
#!/usr/bin/env python3
from typing import List, Dict, Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')
        self.action_sequence_subscriber = self.create_subscription(
            String, 'action_sequences', self.action_sequence_callback, 10
        )
        self.execution_status_publisher = self.create_publisher(
            String, 'execution_status', 10
        )

        self.current_action_sequence = None
        self.current_action_index = 0
        self.is_executing = False

        self.get_logger().info('Action Executor Node initialized')

    def action_sequence_callback(self, msg):
        """ایکشن سیکوئنس وصول کریں اور ایکزیکیوٹ کریں"""
        try:
            sequence_data = json.loads(msg.data)

            # موجودہ سیکوئنس کو کینسل کریں اگر چل رہا ہو
            if self.is_executing:
                self.cancel_current_execution()

            # نیا سیکوئنس تیار کریں
            self.current_action_sequence = sequence_data
            self.current_action_index = 0
            self.is_executing = True

            self.get_logger().info(f'Starting execution of sequence: {sequence_data["sequence_id"]}')

            # ایکزیکیوشن شروع کریں
            self.execute_sequence()

        except Exception as e:
            self.get_logger().error(f'Error processing action sequence: {e}')

    def execute_sequence(self):
        """سیکوئنس کے تمام ایکشنز کو ایکزیکیوٹ کریں"""
        if not self.current_action_sequence:
            return

        actions = self.current_action_sequence['actions']

        for i, action_data in enumerate(actions):
            if not self.is_executing:
                break

            self.current_action_index = i
            self.get_logger().info(f'Executing action {i+1}/{len(actions)}: {action_data["action_type"]}')

            # ایکشن کو ایکزیکیوٹ کریں
            success = self.execute_action(action_data)

            if not success:
                self.get_logger().error(f'Action execution failed: {action_data["action_type"]}')
                self.publish_execution_status('failed', f'Action failed: {action_data["action_type"]}')
                break

        if self.is_executing:
            self.get_logger().info('Action sequence completed successfully')
            self.publish_execution_status('completed', 'Sequence completed successfully')

        self.is_executing = False
        self.current_action_sequence = None

    def execute_action(self, action_data: Dict[str, Any]) -> bool:
        """ایک ایکشن کو ایکزیکیوٹ کریں"""
        action_type = action_data['action_type']
        parameters = action_data['parameters']
        duration = action_data.get('duration', 2.0)

        try:
            if action_type == 'move_to':
                return self.execute_move_to(parameters, duration)
            elif action_type == 'pick_object':
                return self.execute_pick_object(parameters, duration)
            elif action_type == 'place_object':
                return self.execute_place_object(parameters, duration)
            elif action_type == 'inspect_object':
                return self.execute_inspect_object(parameters, duration)
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                return self.execute_generic_action(action_type, parameters, duration)

        except Exception as e:
            self.get_logger().error(f'Error executing action {action_type}: {e}')
            return False

    def execute_move_to(self, params: Dict[str, Any], duration: float) -> bool:
        """موو ٹو ایکشن ایکزیکیوٹ کریں"""
        target_pose = params.get('target_pose')
        if not target_pose:
            self.get_logger().error('Missing target_pose in move_to action')
            return False

        self.get_logger().info(f'Moving to target: {target_pose}')

        # یہاں اصل نیویگیشن کوڈ ہوگا
        # وقت کے لیے سلیپ کریں
        time.sleep(duration)

        return True

    def execute_pick_object(self, params: Dict[str, Any], duration: float) -> bool:
        """پک آبجیکٹ ایکشن ایکزیکیوٹ کریں"""
        object_id = params.get('object_id')
        grasp_pose = params.get('grasp_pose')

        if not object_id or not grasp_pose:
            self.get_logger().error('Missing object_id or grasp_pose in pick_object action')
            return False

        self.get_logger().info(f'Picking object {object_id} at {grasp_pose}')

        # یہاں اصل مینیپولیشن کوڈ ہوگا
        time.sleep(duration)

        return True

    def execute_place_object(self, params: Dict[str, Any], duration: float) -> bool:
        """پلیس آبجیکٹ ایکشن ایکزیکیوٹ کریں"""
        target_pose = params.get('target_pose')
        object_id = params.get('object_id')

        if not target_pose or not object_id:
            self.get_logger().error('Missing target_pose or object_id in place_object action')
            return False

        self.get_logger().info(f'Placing object {object_id} at {target_pose}')

        # یہاں اصل مینیپولیشن کوڈ ہوگا
        time.sleep(duration)

        return True

    def execute_inspect_object(self, params: Dict[str, Any], duration: float) -> bool:
        """انسپیکٹ آبجیکٹ ایکشن ایکزیکیوٹ کریں"""
        object_id = params.get('object_id')
        inspection_pose = params.get('inspection_pose')

        if not object_id or not inspection_pose:
            self.get_logger().error('Missing object_id or inspection_pose in inspect_object action')
            return False

        self.get_logger().info(f'Inspecting object {object_id} from {inspection_pose}')

        # یہاں اصل وژن کوڈ ہوگا
        time.sleep(duration)

        return True

    def execute_generic_action(self, action_type: str, params: Dict[str, Any], duration: float) -> bool:
        """جنرک ایکشن ایکزیکیوٹ کریں"""
        self.get_logger().info(f'Executing generic action: {action_type} with params: {params}')
        time.sleep(duration)
        return True

    def cancel_current_execution(self):
        """موجودہ ایکزیکیوشن کو کینسل کریں"""
        self.get_logger().info('Cancelling current action sequence execution')
        self.is_executing = False

    def publish_execution_status(self, status: str, message: str):
        """ایکزیکیوشن کا اسٹیٹس پبلش کریں"""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': status,
            'message': message,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.execution_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    executor = ActionExecutorNode()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        executor.cancel_current_execution()
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خلاصہ

اس ورکشاپ میں، آپ نے:
- ایکشن پلاننگ کے لیے ڈیٹا سٹرکچر تخلیق کیا
- ہائی لیول کمانڈز کو ایکشن سیکوئنس میں تبدیل کرنے والے سسٹم کو نافذ کیا
- ایکشنز کو ایکزیکیوٹ کرنے کے لیے ایک ایکزیکیوٹر نوڈ تیار کیا
- مختلف اقسام کے ایکشنز کو ہینڈل کرنے کے لیے ماڈولر سٹرکچر ڈیزائن کیا

یہ سسٹم وژن لینگویج ایکشن فریم ورک کے لیے بنیاد فراہم کرتا ہے جہاں روبوٹ کو کمانڈز کو سمجھنے اور ان کو ایکشنز میں تبدیل کرنے کی صلاحیت حاصل ہو۔