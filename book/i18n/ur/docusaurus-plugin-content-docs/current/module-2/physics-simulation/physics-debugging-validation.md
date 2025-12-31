---
sidebar_position: 4
---

# فزکس ڈیبگنگ اور توثیق کی تکنیکیں

یہ ٹیوٹوریل گیزبو میں فزکس سیمولیشنز کو ڈیبگ کرنے اور توثیق کرنے کی اہم تکنیکوں کو دیکھتا ہے کہ آپ کے روبوٹ ماڈلز حقیقی طور پر کیسے سلوک کرتے ہیں۔

## فزکس توثیق کے بنیادیات

### فزکس سیمولیشن کو سمجھنا

گیزبو میں فزکس سیمولیشن میں شامل ہیں:
- **کولیژن ڈیٹیکشن**: یہ تعین کرنا کہ کب اشیاء کاٹتی ہیں
- **کنٹیکٹ ریزولوشن**: جب اشیاء چھوتی ہیں تو قوتیں کیسے محسوب کی جاتی ہیں
- **انٹیگریشن**: وقت کے ساتھ پوزیشنز اور ویلوسٹیز کو اپ ڈیٹ کرنا
- **کنٹرینٹس**: جوائنٹ ریلیشن شپس اور حدود کو برقرار رکھنا

### عام فزکس کے مسائل

1. **اشیاء سطحوں کے ذریعے گرتی ہیں** - عام طور پر کولیژن جیومیٹریز یا غلط انرشل خصوصیات کے فقدان کی وجہ سے
2. **غیر مستحکم سیمولیشنز** - اکثر غلط ٹائم اسٹیپس یا غیر حقیقی پیرامیٹرز کی وجہ سے
3. **جِٹری حرکتیں** - زیادہ فریکوینسی کے اوسیلیشنز یا عددی خامیوں کی وجہ سے نتیجہ دے سکتا ہے
4. **اشیاء کے درمیان گھسناوٹ** - غیر کافی سالور اسٹیپس یا خراب میش کوالٹی کی نشاندہی کر سکتا ہے

## توثیق کی تکنیکیں

### 1. وژول توثیق

کولیژن شکلوں اور کنٹیکٹ پوائنٹس کو دیکھنے کے لیے فزکس وژولائزیشن کو فعال کریں:

```bash
# کولیژن وژولائزیشن کے ساتھ گیزبو لانچ کریں
gz sim --render-engine ogre2

# جی یو آئی میں، فعال کریں:
# - View -> Transparent Models
# - View -> Wireframe
# - View -> Contacts
```

### 2. عددی توثیق

#### انرشل خصوصیات چیک کریں
```bash
# تصدیق کریں کہ آپ کے URDF میں مناسب انرشل ویلیوز ہیں
check_urdf your_robot.urdf

# مرکز کا دھرنا لنک کی جسمانی حدود کے اندر ہونا چاہیے
# انرشیا ویلیوز مثبت اور مناسب ہونی چاہئیں
```

#### فزکس پیرامیٹرز کی توثیق کریں
```xml
<!-- آپ کی ورلڈ فائل میں -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- استحکام کے لیے چھوٹا -->
  <real_time_factor>1.0</real_time_factor>  <!-- حقیقی وقت کے لیے 1.0 -->
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>  <!-- استحکام کے لیے بڑھائیں -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

### 3. برتاؤ کی توثیق

#### گریویٹی ٹیسٹ
گریویٹی کام کر رہی ہے کی تصدیق کے لیے ایک سادہ ٹیسٹ بنائیں:
```xml
<!-- gravity_test.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="gravity_test">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="falling_sphere">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

لانچ کریں اور تصدیق کریں کہ گولہ تقریباً 9.8 میٹر/سیکنڈ² کے ساتھ گرتا ہے۔

#### کولیژن ٹیسٹ
سادہ سیٹ اپ کے ساتھ کولیژن ڈیٹیکشن کو ٹیسٹ کریں:
```xml
<!-- collision_test.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="collision_test">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- سٹیٹک رکاوٹ -->
    <model name="wall">
      <pose>1 0 1 0 0 0</pose>
      <static>true</static>  <!-- حرکت نہیں کرے گا -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.1 2 2</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- حرکت کرتی ہوئی چیز -->
    <model name="moving_sphere">
      <pose>0 0 1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## ڈیبگنگ ٹولز اور کمانڈز

### گیزبو کمانڈ لائن ٹولز

```bash
# ماڈل اسٹیٹس چیک کریں
gz model -m

# تفصیلی ماڈل کی معلومات حاصل کریں
gz model -m your_model_name -i

# تمام ٹاپکس کی فہرست
gz topic -l

# فزکس اپ ڈیٹس کو مانیٹر کریں
gz topic -e /world/default/stats
```

### فزکس پیرامیٹر ٹیوننگ

#### ٹائم اسٹیپ تجزیہ
```bash
# وربس فزکس آؤٹ پٹ کے ساتھ لانچ کریں
gz sim -v 4 your_world.sdf
```

#### سالور پیرامیٹر ایڈجسٹمنٹ
```xml
<physics type="ode">
  <!-- محتاط ویلیوز کے ساتھ شروع کریں -->
  <max_step_size>0.001</max_step_size>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- اگر غیر مستحکم ہو تو بڑھائیں -->
      <sor>1.0</sor>     <!-- زیادہ استحکام کے لیے کم -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>     <!-- کنٹرینٹ فورس مکسنگ -->
      <erp>0.2</erp>     <!-- ایرر ریڈکشن پیرامیٹر -->
    </constraints>
  </ode>
</physics>
```

### انرشل خصوصیت کی توثیق

#### مناسب انرشیا کا حساب

عام شکلوں کے لیے:

**باکس (ماس m، ابعاد x، y، z):**
- Ixx = m*(y² + z²)/12
- Iyy = m*(x² + z²)/12
- Izz = m*(x² + y²)/12

**سلنڈر (ماس m، ریڈیس r، لمبائی l):**
- Ixx = m*(3*r² + l²)/12
- Iyy = m*(3*r² + l²)/12
- Izz = m*r²/2

**سپیئر (ماس m، ریڈیس r):**
- Ixx = Iyy = Izz = 2*m*r²/5

### مثال کی توثیق اسکرپٹ

فزکس برتاؤ کی توثیق کے لیے ایک پائی تھون اسکرپٹ بنائیں:

```python
#!/usr/bin/env python3
"""
گیزبو سیمولیشنز کے لیے فزکس توثیق اسکرپٹ
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkStates
import numpy as np

class PhysicsValidator(Node):
    def __init__(self):
        super().__init__('physics_validator')
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.listener_callback,
            10)
        self.subscription  # غیر استعمال شدہ متغیر کی وارننگ کو روکنے کے لیے

        # توثیق پیرامیٹرز سیٹ اپ کریں
        self.gravity = 9.81  # میٹر/سیکنڈ²
        self.test_start_time = self.get_clock().now()

    def listener_callback(self, msg):
        # مثال: تصدیق کریں کہ ایک مفت گرتی ہوئی چیز g کے ساتھ تیز ہو رہی ہے
        try:
            # پیغام میں ٹیسٹ آبجیکٹ تلاش کریں
            obj_index = msg.name.index('falling_sphere::link')
            obj_position = msg.pose[obj_index].position
            obj_velocity = msg.twist[obj_index].linear

            # مفت گرنے کے لیے متوقع پوزیشن کا حساب لگائیں
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.test_start_time).nanoseconds / 1e9

            expected_position = -0.5 * self.gravity * elapsed_time**2

            # تصدیق کریں کہ اصل پوزیشن متوقع سے مماثل ہے
            if abs(obj_position.z - expected_position) > 0.1:  # 10سینٹی میٹر ٹولرینس
                self.get_logger().warn(f'گریویٹی توثیق ناکام: متوقع {expected_position}, حاصل {obj_position.z}')
            else:
                self.get_logger().info(f'گریویٹی توثیق کامیاب: {obj_position.z} بمقابلہ {expected_position}')

        except ValueError:
            # پیغام میں آبجیکٹ نہیں ملا
            pass

def main(args=None):
    rclpy.init(args=args)
    validator = PhysicsValidator()

    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## عام ڈیبگنگ منظرنامے

### منظر نامہ 1: روبوٹ زمین کے ذریعے گر رہا ہے

**علامات**: روبوٹ زمینی سطح کے ذریعے گر رہا ہے
**حل**:
1. چیک کریں کہ زمینی سطح ماڈل لوڈ ہو گیا ہے
2. یقینی بنائیں کہ تمام لنکس کے لیے کولیژن جیومیٹریز کی وضاحت کی گئی ہے
3. یقینی بنائیں کہ ماس ویلیوز مثبت ہیں
4. چیک کریں کہ انرشل اوریجن کولیژن جیومیٹری کے اندر ہے

### منظر نامہ 2: روبوٹ جِٹر یا وائبریٹ کر رہا ہے

**علامات**: روبوٹ جگہ پر تیزی سے آویش کر رہا ہے
**حل**:
1. فزکس سیٹنگز میں `max_step_size` کم کریں
2. سالور اسٹیپس بڑھائیں
3. اوور لیپنگ کولیژن جیومیٹریز کے لیے چیک کریں
4. جوائنٹ حدود اور سختی کی تصدیق کریں

### منظر نامہ 3: غیر مستحکم ملٹی لنک روبوٹ

**علامات**: متعدد لنکس والے روبوٹ کو غیر مستحکم ہو جاتا ہے
**حل**:
1. ایک سادہ ماڈل کے ساتھ شروع کریں اور تدریج سے پیچیدگی شامل کریں
2. یقینی بنائیں کہ تمام جوائنٹس کے پاس مناسب حدود ہیں
3. چیک کریں کہ انرشل خصوصیات حقیقی ہیں
4. مناسب جگہوں پر بہت سخت ریوولوٹ جوائنٹس کے بجائے فکسڈ جوائنٹس استعمال کریں

## کارکردگی کی بہتری

### فزکس کارکردگی کے نکات

1. **آسان کولیژن جیومیٹریز استعمال کریں**:
   - کمپلیکس میشز کی جگہ باکسز، سpheres، اور سلنڈرز استعمال کریں
   - میشز کے لیے `<approximate_as_box>true</approximate_as_box>` استعمال کریں

2. **سالور پیرامیٹرز کو بہتر بنائیں**:
   - محتاط سیٹنگز کے ساتھ شروع کریں اور کارکردگی کے لیے بہتر بنائیں
   - اپنی ضروریات کی بنیاد پر صحت اور رفتار کا توازن قائم کریں

3. **جہاں ممکن ہو اپ ڈیٹ ریٹس کم کریں**:
   - کم اہم سیمولیشنز کے لیے فزکس اپ ڈیٹ ریٹ کم کریں
   - مناسب سینسر اپ ڈیٹ ریٹس استعمال کریں

### مثال کے طور پر بہتر فزکس سیٹنگز

```xml
<physics type="ode">
  <!-- کارکردگی کے اہم ایپلی کیشنز کے لیے -->
  <max_step_size>0.01</max_step_size>  <!-- رفتار کے لیے بڑا -->
  <real_time_update_rate>100</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>20</iters>  <!-- استحکام اور رفتار کے درمیان توازن -->
    </solver>
  </ode>
</physics>
```

## توثیق چیک لسٹ

اپنی فزکس سیمولیشن کو مکمل سمجھنے سے پہلے:

- [ ] اشیاء سطحوں پر مستحکم طور پر آرام کرتی ہیں
- [ ] گریویٹی متوقع تیزی پیدا کرتی ہے (9.8 میٹر/سیکنڈ²)
- [ ] کولیژن کا پتہ چلتا ہے اور مناسب طریقے سے حل کیا جاتا ہے
- [ ] روبوٹ جوائنٹس متوقع حدود کے اندر کام کرتے ہیں
- [ ] اشیاء کے درمیان کوئی غیر متوقع گھسناوٹ نہیں ہے
- [ ] سیمولیشن قابل قبول حقیقی وقت کے عامل پر چلتا ہے
- [ ] انرشل خصوصیات جسمانی طور پر حقیقی ہیں
- [ ] ماس اور مرکز کا دھرنا درست طریقے سے پوزیشن کیا گیا ہے

## مسئلہ حل کرنے کے وسائل

### عام خامی کے پیغامات

- **"ODE Message 3: body not finite"**: عام طور پر عددی عدم استحکام یا غلط پیرامیٹرز کی نشاندہی کرتا ہے
- **"Contact not finite"**: اکثر بہت چھوٹے یا صفر ماس ویلیوز کی وجہ سے
- **"Joint limits exceeded"**: سالور کے مسائل یا خراب جوائنٹ پیرامیٹرز کی نشاندہی کر سکتا ہے

### مدد حاصل کرنا

- [گیزبو دستاویزات](http://gazebosim.org/tutorials?cat=physics) چیک کریں
- اپنی SDF فائلز کی توثیق کے لیے `gz sdf -k` استعمال کریں
- مسائل کو الگ کرنے کے لیے سادہ ماڈلز کے ساتھ ٹیسٹ کریں

## اگلے اقدامات

اپنی فزکس سیمولیشن کی توثیق کے بعد:

1. [ماحول کی ماڈلنگ](./environment-modeling.md) کے ساتھ جاری رکھیں تاکہ پیچیدہ سیمولیشن دنیا تخلیق کی جا سکے
2. ادراک کی صلاحیتوں کو شامل کرنے کے لیے [سینسر سیمولیشن](../sensor-simulation/lidar-simulation.md) کا استعما ل کریں
3. ہائی فائیڈلٹی وژولائزیشن کے لیے [یونٹی انٹیگریشن](../unity-integration/unity-setup.md) کے بارے میں سیکھیں