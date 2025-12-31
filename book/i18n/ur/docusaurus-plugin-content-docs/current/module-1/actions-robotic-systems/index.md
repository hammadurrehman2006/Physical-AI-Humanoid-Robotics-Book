---
title: روبوٹک سسٹم میں ایکشنز
description: طویل مدتی کاموں اور گول مینجمنٹ کے لیے ROS 2 ایکشنز کو سمجھنا
sidebar_position: 4
---

# روبوٹک سسٹم میں ایکشنز

## سیکھنے کے اہداف
- ROS 2 ایکشنز کے تصور اور ان کے استعمال کا وقت سمجھیں
- گول-اورینٹڈ کاموں کے لیے ایکشن سرورز اور کلائنٹس نافذ کریں
- گول فیڈ بیک اور نتیجہ رپورٹنگ کا انتظام کریں
- ایکشن کا پریمپشن اور منسوخی کو سنبھالیں
- مضبوط ایکشن-مبنی روبوٹک رویے تیار کریں

## ROS 2 ایکشنز کا تعارف

ROS 2 میں ایکشنز ایک مواصلاتی نمونہ فراہم کرتے ہیں جو طویل مدتی کاموں کے لیے خاص طور پر ڈیزائن کیا گیا ہے جنہیں فیڈ بیک کی ضرورت ہوتی ہے اور منسوخ کیا جا سکتا ہے۔ سروسز کے برعکس، جو ہم وقت اور بلاکنگ ہوتی ہیں، ایکشنز غیر ہم وقت ہوتے ہیں اور انجام دہی کے دوران جاری فیڈ بیک فراہم کرتے ہیں۔

### ایکشنز کب استعمال کریں

ایکشنز ان کاموں کے لیے بہترین ہیں جو:
- مکمل ہونے میں کافی وقت لیتے ہیں
- کلائنٹ کو جاری فیڈ بیک کی ضرورت ہوتی ہے
- منسوخ یا پریمپٹ کرنے کے قابل ہونا چاہیے
- انجام دہی کے دوران بین الاقوامی نتائج رکھتے ہیں
- گول-اورینٹڈ رویے کی نمائندگی کرتے ہیں

مثالیں شامل ہیں:
- مخصوص مقام پر نیویگیشن
- روبوٹ آرم موشن پلاننگ اور انجام دہی
- آبجیکٹ مینیپولیشن کام
- طویل مدتی ڈیٹا پروسیسنگ
- کیلبریشن طریقے

## ایکشن میسج کی ساخت

ایکشنز تین میسج کی اقسام پر مشتمل ہوتے ہیں:

1. **گول**: یہ بتاتا ہے کہ ایکشن کیا کرنا چاہیے
2. **فیڈ بیک**: بین الاقوامی حیثیت کی اپ ڈیٹس فراہم کرتا ہے
3. **نتیجہ**: ایکشن کا حتمی نتیجہ رکھتا ہے

### مثال ایکشن کی تعریف

```# Fibonacci.action
# ایک فیبونیچی ایکشن کی تعریف کریں جو ایک ترتیب کا حساب لگاتا ہے

# گول کی تعریف
int32 order

---
# نتیجہ کی تعریف
int32[] sequence

---
# فیڈ بیک کی تعریف
int32[] sequence
```

## ایکشن سرور بنانا

ایک ایکشن سرور آنے والے گولز کو سنبھالتا ہے، انہیں انجام دیتا ہے، اور فیڈ بیک اور نتائج فراہم کرتا ہے۔

### بنیادی ایکشن سرور نافذ کاری

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        # کال بیک گروپ کے ساتھ ایکشن سرور بنائیں تاکہ دوبارہ داخل ہو سکے
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Fibonacci action server started')

    def goal_callback(self, goal_request):
        """گول کی درخواست کو قبول یا مسترد کریں۔"""
        self.get_logger().info(f'Received goal request: {goal_request.order}')

        # اس مثال کے لیے تمام گولز قبول کریں
        # عمل میں، آپ ان گولز کو مسترد کر سکتے ہیں جنہیں حاصل کرنا ناممکن ہو
        if goal_request.order > 0:
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """کلائنٹ کی درخواست کو قبول یا مسترد کریں ایکشن کو منسوخ کرنے کے لیے۔"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """گول کو انجام دیں۔"""
        self.get_logger().info('Executing goal...')

        # گول آرڈر حاصل کریں
        order = goal_handle.request.order

        # فیڈ بیک اور نتیجہ میسجز بنائیں
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # فیبونیچی ترتیب کو شروع کریں
        feedback_msg.sequence = [0, 1]

        # ایکشن کی انجام دہی شروع کریں
        for i in range(1, order):
            # دیکھیں کہ آیا منسوخی کی درخواست ہے
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal was cancelled')
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # ترتیب کو اپ ڈیٹ کریں
            if i < len(feedback_msg.sequence):
                continue
            else:
                feedback_msg.sequence.append(
                    feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]
                )

            # فیڈ بیک شائع کریں
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # کام کی شبیہہ بنانے کے لیے سلیپ کریں (حقیقی ایپلی کیشنز میں، یہ اصل کام ہوگا)
            from time import sleep
            sleep(0.5)

        # چیک کریں کہ آیا گول انجام دہی کے دوران منسوخ ہوا تھا
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal was cancelled')
            result_msg.sequence = feedback_msg.sequence
            goal_handle.canceled()
            return result_msg

        # گول کامیابی کے ساتھ مکمل ہوا
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()

        self.get_logger().info(f'Goal succeeded with result: {result_msg.sequence}')
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()

    try:
        # کال بیکس کو مناسب طریقے سے ہینڈل کرنے کے لیے MultiThreadedExecutor استعمال کریں
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ایکشن کلائنٹ بنانا

ایک ایکشن کلائنٹ ایکشن سرور کو گولز بھیجتا ہے اور پیشرفت کو نگرانی کر سکتا ہے، فیڈ بیک وصول کر سکتا ہے، اور گولز منسوخ کر سکتا ہے۔

### بنیادی ایکشن کلائنٹ نافذ کاری

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')

        # ایکشن کلائنٹ بنائیں
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        """ایکشن سرور کو ایک گول بھیجیں۔"""
        self.get_logger().info(f'Waiting for action server...')

        # ایکشن سرور کی دستیابی کا انتظار کریں
        self._action_client.wait_for_server()

        # ایک گول میسج بنائیں
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # گول بھیجیں اور کال بیکس رجسٹر کریں
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # گول جواب کے لیے کال بیکس شامل کریں
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """گول جواب کو ہینڈل کریں۔"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # نتیجہ کی درخواست کریں
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """ایکشن سرور سے فیڈ بیک کو ہینڈل کریں۔"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        """ایکشن سرور سے نتیجہ کو ہینڈل کریں۔"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        # نتیجہ وصول کرنے کے بعد شٹ ڈاؤن
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    # ایک گول بھیجیں
    future = action_client.send_goal(5)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()

if __name__ == '__main__':
    main()
```

## اعلی درجے کے ایکشن سرور خصوصیات

### متعدد ہم زمانہ گولز کو سنبھالنا

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci
import threading
from concurrent.futures import ThreadPoolExecutor

class ConcurrentFibonacciActionServer(Node):
    def __init__(self):
        super().__init__('concurrent_fibonacci_action_server')

        # متعدد گولز کو ہم زمانہ ہینڈل کرنے کے لیے تھریڈ پول
        self.executor_pool = ThreadPoolExecutor(max_workers=3)

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'concurrent_fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback
        )

        self.get_logger().info('Concurrent Fibonacci action server started')

    def goal_callback(self, goal_request):
        """تمام گولز قبول کریں۔"
        self.get_logger().info(f'Received goal: {goal_request.order}')
        return rclpy.action.server.GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """ایک الگ تھریڈ میں گول کو انجام دیں۔"
        self.get_logger().info('Executing goal in separate thread...')

        # تھریڈ پول میں انجام دہی جمع کرائیں
        future = self.executor_pool.submit(self.compute_fibonacci, goal_handle)
        result = future.result()

        return result

    def compute_fibonacci(self, goal_handle):
        """ایک الگ تھریڈ میں فیبونیچی ترتیب کا حساب لگائیں۔"
        order = goal_handle.request.order
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # ترتیب شروع کریں
        if order >= 1:
            feedback_msg.sequence = [0]
        if order >= 2:
            feedback_msg.sequence.append(1)

        # ترتیب کا حساب لگائیں
        for i in range(2, order):
            if goal_handle.is_cancel_requested:
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # اگلے فیبونیچی نمبر کا حساب لگائیں
            next_num = feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
            feedback_msg.sequence.append(next_num)

            # فیڈ بیک شائع کریں
            goal_handle.publish_feedback(feedback_msg)

            # کام کی شبیہہ بنائیں
            import time
            time.sleep(0.1)

        # منسوخی کے لیے چیک کریں
        if goal_handle.is_cancel_requested:
            result_msg.sequence = feedback_msg.sequence
            goal_handle.canceled()
            return result_msg

        # کامیابی کے ساتھ مکمل کریں
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    server = ConcurrentFibonacciActionServer()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.executor_pool.shutdown(wait=True)
        server.destroy_node()
        rclpy.shutdown()
```

## نیویگیشن ایکشن مثال

روبوٹ نیویگیشن کے لیے ایکشنز کے استعمال کو دکھانے والی ایک عملی مثال:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# ایک کسٹم نیویگیشن ایکشن کی تعریف کریں (تصوراتی)
"""
# NavigateToPose.action
geometry_msgs/PoseStamped target_pose
---
geometry_msgs/PoseStamped final_pose
string message
---
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 progress_percentage
"""

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # ایک حقیقی نافذ کاری میں، آپ اصل NavigateToPose ایکشن استعمال کریں گے
        # اس مثال کے لیے، ہم فیبونیچی ایکشن کو ایک پلیس ہولڈر کے طور پر استعمال کریں گے
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'navigate_to_pose',
            execute_callback=self.execute_navigation_callback,
            goal_callback=self.navigation_goal_callback,
            cancel_callback=self.navigation_cancel_callback
        )

        # وژولائزیشن کے لیے پبلشرز
        self.path_pub = self.create_publisher(Path, 'current_path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'navigation_goal', 10)

        self.get_logger().info('Navigation action server started')

    def navigation_goal_callback(self, goal_request):
        """نیویگیشن گول کی تصدیق کریں۔"
        # چیک کریں کہ ٹارگٹ پوز معتبر اور قابل رسائی ہے
        # یہ ایک میپ، کولیژن ڈیٹیکشن، وغیرہ کے خلاف چیک کرے گا
        self.get_logger().info(f'Received navigation goal: {goal_request.order}')
        return GoalResponse.ACCEPT

    def navigation_cancel_callback(self, goal_handle):
        """نیویگیشن منسوخی کو ہینڈل کریں۔"
        self.get_logger().info('Navigation goal cancelled')
        return CancelResponse.ACCEPT

    async def execute_navigation_callback(self, goal_handle):
        """ٹارگٹ پوز پر نیویگیشن کو انجام دیں۔"
        self.get_logger().info('Starting navigation to target...')

        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # نیویگیشن پیشرفت کی شبیہہ بنائیں
        total_steps = goal_handle.request.order
        current_step = 0

        while current_step < total_steps:
            # منسوخی کے لیے چیک کریں
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Navigation cancelled by user')
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # پیشرفت کو اپ ڈیٹ کریں
            current_step += 1
            progress = (current_step / total_steps) * 100

            # فیڈ بیک بنائیں
            feedback_msg.sequence.append(current_step)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Navigation progress: {progress:.1f}%')

            # حرکت کی شبیہہ بنائیں
            import time
            time.sleep(1.0)

        # نیویگیشن کامیابی کے ساتھ مکمل ہوا
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()

        self.get_logger().info('Navigation completed successfully')
        return result_msg
```

## ٹائم آؤٹ اور دوبارہ کوشش کا منطق کے ساتھ ایکشن کلائنٹ

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from example_interfaces.action import Fibonacci
import time

class RobustActionClient(Node):
    def __init__(self):
        super().__init__('robust_action_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        self.feedback_received = False

    def send_goal_with_timeout(self, order, timeout_seconds=30):
        """ٹائم آؤٹ کے ساتھ گول بھیجیں اور نتیجہ لوٹائیں۔"
        self.get_logger().info(f'Waiting for action server (timeout: {timeout_seconds}s)...')

        # ٹائم آؤٹ کے ساتھ سرور کا انتظار کریں
        if not self._action_client.wait_for_server(timeout_sec=timeout_seconds):
            self.get_logger().error('Action server not available')
            return None

        # گول بنائیں
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # گول بھیجیں
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # گول جواب کے لیے انتظار کریں
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_seconds)

        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Failed to get goal handle')
            return None

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        # نتیجہ کے لیے انتظار کریں
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_seconds)

        result = result_future.result()
        if result:
            return result.result
        else:
            self.get_logger().error('Failed to get result')
            return None

    def feedback_callback(self, feedback_msg):
        """فیڈ بیک کو ہینڈل کریں۔"
        self.feedback_received = True
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def send_goal_with_retry(self, order, max_retries=3, timeout_seconds=30):
        """دوبارہ کوشش کے منطق کے ساتھ گول بھیجیں۔"
        for attempt in range(max_retries):
            self.get_logger().info(f'Attempt {attempt + 1} of {max_retries}')

            result = self.send_goal_with_timeout(order, timeout_seconds)

            if result is not None:
                self.get_logger().info(f'Success on attempt {attempt + 1}')
                return result
            else:
                self.get_logger().warn(f'Attempt {attempt + 1} failed')

                if attempt < max_retries - 1:  # آخری کوشش کے بعد سلیپ نہ کریں
                    time.sleep(2)  # دوبارہ کوشش سے پہلے انتظار کریں

        self.get_logger().error(f'Failed after {max_retries} attempts')
        return None

def main(args=None):
    rclpy.init(args=args)
    client = RobustActionClient()

    # دوبارہ کوشش کے ساتھ گول بھیجیں
    result = client.send_goal_with_retry(10, max_retries=3)

    if result:
        client.get_logger().info(f'Final result: {result.sequence}')
    else:
        client.get_logger().error('Action failed after all retries')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ہاتھوں سے مشق: ایکشن-مبنی روبوٹ کنٹرول

ایک شبیہہ روبوٹ آرم کو کنٹرول کرنے کے لیے ایک مکمل ایکشن-مبنی سسٹم بنائیں۔

### مشق کی ضروریات
1. روبوٹ آرم حرکت کے لیے ایک ایکشن سرور بنائیں
2. گول کی تصدیق اور پریمپشن نافذ کریں
3. فیڈ بیک نگرانی کے ساتھ ایک ایکشن کلائنٹ بنائیں
4. گول منسوخی کا مظاہرہ کریں

### Python نافذ کاری

```python
#!/usr/bin/env python3
"""
ایکشن-مبنی روبوٹ کنٹرول سسٹم
فیڈ بیک اور منسوخی کے ساتھ روبوٹ آرم کنٹرول کے لیے ایکشنز کا مظاہرہ کرتا ہے
"""

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from example_interfaces.action import Fibonacci  # فیبونیچی کو پلیس ہولڈر کے طور پر استعمال کر رہا ہے
import time
import threading
from enum import Enum

class ArmJointState(Enum):
    SHOULDER = 0
    ELBOW = 1
    WRIST = 2

class RobotArmActionServer(Node):
    def __init__(self):
        super().__init__('robot_arm_action_server')

        # آرم جوائنٹ پوزیشنز شروع کریں
        self.joint_positions = {ArmJointState.SHOULDER: 0.0,
                               ArmJointState.ELBOW: 0.0,
                               ArmJointState.WRIST: 0.0}

        # ایکشن سرور
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'move_arm',
            execute_callback=self.execute_move_arm_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Robot arm action server started')

    def goal_callback(self, goal_request):
        """آرم چلنے کے گول کی تصدیق کریں۔"
        self.get_logger().info(f'Received move arm goal: order={goal_request.order}')

        # چیک کریں کہ گول مناسب ہے
        # ایک حقیقی سسٹم میں، یہ جوائنٹ حدود، کولیژن، وغیرہ کی چیک کرے گا
        if 1 <= goal_request.order <= 10:  # اس مثال کے لیے مناسب رینج
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f'Goal order {goal_request.order} is invalid')
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """گول منسوخی کو ہینڈل کریں۔"
        self.get_logger().info('Move arm goal cancellation requested')
        return CancelResponse.ACCEPT

    async def execute_move_arm_callback(self, goal_handle):
        """روبوٹ آرم حرکت کو انجام دیں۔"
        self.get_logger().info('Starting robot arm movement...')

        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # متعدد اسٹیپس کے ساتھ آرم حرکت کی شبیہہ بنائیں
        total_steps = goal_handle.request.order
        current_step = 0

        while current_step < total_steps:
            # منسوخی کے لیے چیک کریں
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Robot arm movement cancelled')
                result_msg.sequence = feedback_msg.sequence
                goal_handle.canceled()
                return result_msg

            # آرم حرکت کی شبیہہ بنائیں - جوائنٹ پوزیشنز کو تدریجی طور پر اپ ڈیٹ کریں
            for joint in ArmJointState:
                self.joint_positions[joint] += 0.1  # جوائنٹ پوزیشن اضافہ کریں

            # فیڈ بیک اپ ڈیٹ کریں
            current_step += 1
            feedback_msg.sequence.append(current_step)

            # فیڈ بیک شائع کریں
            goal_handle.publish_feedback(feedback_msg)

            # موجودہ حالت لاگ کریں
            self.get_logger().info(
                f'Movement step {current_step}/{total_steps}, '
                f'Joint positions: {self.joint_positions}'
            )

            # جسمانی حرکت کے لیے وقت کی شبیہہ بنائیں
            time.sleep(0.5)

        # ایک بار پھر منسوخی کے لیے چیک کریں
        if goal_handle.is_cancel_requested:
            result_msg.sequence = feedback_msg.sequence
            goal_handle.canceled()
            return result_msg

        # حرکت کامیابی کے ساتھ مکمل ہو گئی
        result_msg.sequence = feedback_msg.sequence
        goal_handle.succeed()

        self.get_logger().info(f'Robot arm movement completed: {result_msg.sequence}')
        return result_msg

class RobotArmActionClient(Node):
    def __init__(self):
        super().__init__('robot_arm_action_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'move_arm'
        )

        self.current_feedback = None

    def send_arm_movement_goal(self, steps, send_cancel_after=None):
        """روبوٹ آرم کو چلنے کا گول بھیجیں۔"
        self.get_logger().info('Waiting for robot arm action server...')

        # سرور کا انتظار کریں
        self._action_client.wait_for_server()

        # گول بنائیں
        goal_msg = Fibonacci.Goal()
        goal_msg.order = steps

        # فیڈ بیک کال بیک کے ساتھ گول بھیجیں
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)
        goal_handle_future = future

        # گول ہینڈل حاصل کریں
        rclpy.spin_until_future_complete(self, goal_handle_future)
        goal_handle = goal_handle_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')

        # اختیاری طور پر کچھ وقت کے بعد منسوخی کی درخواست بھیجیں
        if send_cancel_after:
            def cancel_after_delay():
                time.sleep(send_cancel_after)
                self.get_logger().info('Sending cancel request...')
                cancel_future = goal_handle.cancel_goal_async()
                # منسوخی نتیجہ کے انتظار میں بلاک ہونے سے بچنے کے لیے انتظار نہ کریں

            cancel_thread = threading.Thread(target=cancel_after_delay)
            cancel_thread.start()

        # نتیجہ کے لیے انتظار کریں
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result:
            self.get_logger().info(f'Arm movement result: {result.result.sequence}')
            return result.result
        else:
            self.get_logger().error('Failed to get result')
            return None

    def goal_response_callback(self, future):
        """گول جواب کو ہینڈل کریں۔"
        goal_handle = future.result()
        self.get_logger().info(f'Goal response received: {goal_handle is not None}')

    def feedback_callback(self, feedback_msg):
        """آرم حرکت سے فیڈ بیک کو ہینڈل کریں۔"
        self.current_feedback = feedback_msg.feedback
        self.get_logger().info(f'Arm movement feedback: step {len(feedback_msg.feedback.sequence)}')

def run_robot_arm_demo():
    """مکمل روبوٹ آرم ایکشن ڈیمو چلائیں۔"
    rclpy.init()

    print("Starting Robot Arm Action Demo")
    print("=" * 40)

    # سرور اور کلائنٹ نوڈس بنائیں
    server = RobotArmActionServer()
    client = RobotArmActionClient()

    # دونوں نوڈس چلانے کے لیے متعدد تھریڈڈ ایگزیکیوٹر استعمال کریں
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)

    # نوڈس کو پس منظر میں چلائیں
    def spin_executor():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    import threading
    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    time.sleep(1)  # نوڈس کو شروع ہونے کا وقت دیں

    print("\n1. کامیاب آرم حرکت کی جانچ:")
    result1 = client.send_arm_movement_goal(5)  # کامیابی کے ساتھ مکمل ہونا چاہیے
    print(f"Result 1: {'Success' if result1 else 'Failed'}")

    time.sleep(2)

    print("\n2. آرم حرکت منسوخی کی جانچ:")
    result2 = client.send_arm_movement_goal(10, send_cancel_after=2.0)  # منسوخ ہونا چاہیے
    print(f"Result 2: {'Success' if result2 else 'Cancelled/Failure'}")

    time.sleep(2)

    print("\n3. غلط گول کی جانچ (مسترد ہونا چاہیے):"
    # پچھلا کال ختم ہو سکتا ہے اس لیے دوبارہ سرور کا انتظار کریں
    client._action_client.wait_for_server()

    # غلط گول بنائیں اور بھیجیں
    invalid_goal_msg = Fibonacci.Goal()
    invalid_goal_msg.order = 15  # جائز رینج [1, 10] سے باہر

    future = client._action_client.send_goal_async(invalid_goal_msg)
    rclpy.spin_until_future_complete(client, future)

    goal_handle = future.result()
    if goal_handle:
        print(f"Invalid goal accepted: {goal_handle.accepted}")
    else:
        print("Failed to get goal handle for invalid goal")

    print("\nDemo completed!")
    server.destroy_node()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_robot_arm_demo()
```

## عام ایکشن مسائل کا حل

### 1. ایکشن سرور جواب نہیں دے رہا

```python
# مسئلہ: ایکشن سرور گولز کا جواب نہیں دے رہا
# حل: سرور رجسٹریشن اور کال بیک انجام دہی چیک کریں

def verify_action_server_status(node, action_name):
    """چیک کریں کہ آیا ایکشن سرور مناسب طریقے سے رجسٹر ہے۔"
    # ایکشن سرورز کی فہرست حاصل کریں
    action_servers = node.get_action_server_names_and_types()

    for name, types in action_servers:
        if name == action_name:
            print(f"Action server '{name}' found with types: {types}")
            return True

    print(f"Action server '{action_name}' not found")
    return False
```

### 2. کال بیک انجام دہی کے مسائل

```python
# مسئلہ: کال بیکس مناسب طریقے سے انجام نہیں دے رہے
# حل: مناسب کال بیک گروپس استعمال کریں

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ProperActionServer(Node):
    def __init__(self):
        super().__init__('proper_action_server')

        # کال بیکس کو ہم وقت نہ چلنے دینے کے لیے باہمی انحصار کال بیک گروپ استعمال کریں
        cb_group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'proper_fibonacci',
            execute_callback=self.execute_callback,
            callback_group=cb_group
        )
```

### 3. طویل مدتی ایکشنز میں میموری مینجمنٹ

```python
# مسئلہ: طویل مدتی ایکشنز میں میموری لیکس
# حل: مناسب وسائل کا انتظام

import weakref

class MemoryManagedActionServer(Node):
    def __init__(self):
        super().__init__('memory_managed_server')

        self.active_goals = {}  # فعال گولز کا نوٹ رکھیں

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'memory_managed_fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """گول کو ٹریک کریں۔"
        return rclpy.action.server.GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """ مناسب صاف کاری کے ساتھ انجام دیں۔"
        goal_id = goal_handle.goal_id.uuid

        try:
            # فعال گولز میں شامل کریں
            self.active_goals[goal_id] = weakref.ref(goal_handle)

            # ایکشن کا کام یہاں انجام دیں
            result = await self.perform_action_work(goal_handle)
            return result
        finally:
            # حوالہ صاف کریں
            if goal_id in self.active_goals:
                del self.active_goals[goal_id]

    async def perform_action_work(self, goal_handle):
        """اصل ایکشن کام انجام دیں۔"
        # یہاں نافذ کاری
        result_msg = Fibonacci.Result()
        result_msg.sequence = [1, 1, 2, 3, 5]  # مثال نتیجہ
        goal_handle.succeed()
        return result_msg
```

## مزید سیکھنے کے لیے وسائل

- [ROS 2 ایکشنز دستاویزات](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Actions-In-Python.html)
- [ایکشن ڈیزائن ہدایات](https://design.ros2.org/articles/actions.html)
- [نیویگیشن2 ایکشن ایکسز](https://navigation.ros.org/command_line_tools/docs/nav2_cli.html)
- "Programming Robots with ROS" by Morgan Quigley وغیرہ

## خلاصہ

ROS 2 میں ایکشنز طویل مدتی، گول-اورینٹڈ کاموں کے لیے ایک طاقتور مواصلاتی نمونہ فراہم کرتے ہیں جنہیں فیڈ بیک اور منسوخی کی صلاحیتیں کی ضرورت ہوتی ہے۔ وہ نیویگیشن، مینیپولیشن، اور کیلبریشن جیسے پیچیدہ روبوٹک رویوں کو نافذ کرنے کے لیے ضروری ہیں۔ ایکشن سرورز اور کلائنٹس کو مناسب طریقے سے نافذ کرنا، فیڈ بیک کو سنبھالنا، اور گول لائف سائیکل کا انتظام کرنا جواب دہ اور مضبوط روبوٹک سسٹم بنانے کے لیے اہم ہے۔