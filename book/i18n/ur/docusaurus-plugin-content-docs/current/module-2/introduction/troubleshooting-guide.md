---
sidebar_position: 5
---

# عام گیزبو مسائل کا حل

یہ گائیڈ وہ حل فراہم کرتا ہے جو آپ کو گیزبو سیمولیشنز کو سیٹ اپ کرنے اور چلانے کے دوران ممکنہ مسائل کا سامنا کرنا پڑ سکتا ہے۔

## عام انسٹالیشن مسائل

### وابستگیاں غائب ہیں

اگر آپ گیزبو انسٹال کرنے کے دوران خامیوں کا سامنا کرتے ہیں:

```bash
# پیکیج لسٹ اپ ڈیٹ کریں
sudo apt update

# ضروری وابستگیاں انسٹال کریں
sudo apt install libglu1-mesa freeglut3-dev mesa-common-dev

# ROS 2 گیزبو پیکیجز انسٹال کریں
sudo apt install ros-humble-gazebo-*
```

### گرافکس ڈرائیور مسائل

اگر گیزبو گرافکس کی خرابیوں کے ساتھ لانچ کرنے میں ناکام ہو جاتا ہے:

1. **اپنے گرافکس ڈرائیورز چیک کریں**:
   ```bash
   # موجودہ گرافکس ڈرائیور چیک کریں
   sudo lshw -c display

   # تجویز کردہ ڈرائیورز انسٹال کریں
   sudo ubuntu-drivers autoinstall
   ```

2. **سافٹ ویئر رینڈرنگ کی کوشش کریں**:
   ```bash
   # انٹیل گرافکس یا ورچوئل مشینز استعمال کرتے وقت
   export MESA_GL_VERSION_OVERRIDE=3.3
   gz sim
   ```

## عام رن ٹائم مسائل

### سیمولیشن عدم استحکام

اگر آپ کی سیمولیشن عدم مستحکم ہے یا اشیاء غیر متوقع طور پر رویہ اختیار کر رہی ہیں:

1. **فزکس پیرامیٹرز ایڈجسٹ کریں**:
   ```bash
   # آپ کی ورلڈ فائل میں، max_step_size کم کریں
   <max_step_size>0.001</max_step_size>
   ```

2. **اپنے یوآرڈی ایف/ایس ڈی ایف فائلوں میں انرشل خصوصیات کی تصدیق کریں**:
   - یقینی بنائیں کہ تمام لنکس کے پاس مناسب ماس ویلیوز ہیں
   - چیک کریں کہ انرشیا ویلیوز مثبت اور مناسب ہیں

### کارکردگی کے مسائل

اگر سیمولیشن سست چل رہی ہے:

1. **اپ ڈیٹ ریٹ کم کریں**:
   ```xml
   <update_rate>100</update_rate>  <!-- بجائے 1000 کے -->
   ```

2. **کولیژن جیومیٹریز کو سادہ بنائیں**:
   - میشز کے بجائے سادہ شکلیں (باکسز، سpheres، سلنڈرز) استعمال کریں
   - وژول میشز میں مثلثوں کی تعداد کم کریں

### سینسر کے مسائل

اگر سینسرز ڈیٹا پبلش نہیں کر رہے ہیں:

1. **سینسر کنفیگریشن چیک کریں**:
   - سینسر کا نام اور قسم کی تصدیق کریں
   - یقینی بنائیں کہ `always_on` کو `1` پر سیٹ کیا گیا ہے
   - چیک کریں کہ اپ ڈیٹ ریٹ مناسب ہے

2. **ROS 2 انٹرفیسز کی توثیق کریں**:
   ```bash
   # دستیاب ٹاپکس کی فہرست
   ros2 topic list

   # چیک کریں کہ آیا سینسر ٹاپک موجود ہے
   ros2 topic echo /your_sensor_topic
   ```

## ROS 2 انٹیگریشن کے مسائل

### پلگ ان لوڈنگ کی خامیاں

اگر گیزبو پلگ ان لوڈ کرنے میں ناکام ہو جاتے ہیں:

1. **پلگ ان لائبریری راستے کی تصدیق کریں**:
   ```bash
   # چیک کریں کہ آیا پلگ ان موجود ہے
   find /usr/lib -name "*gazebo_ros*"
   ```

2. **ROS 2 ماحول کی چیک کریں**:
   ```bash
   # ROS 2 سیٹ اپ سورس کریں
   source /opt/ros/humble/setup.bash
   ```

### کنٹرول انٹرفیس کے مسائل

اگر روبوٹ کنٹرول کام نہیں کر رہا ہے:

1. **URDF میں ٹرانسمیشن کنفیگریشن کی تصدیق کریں**:
   ```xml
   <transmission name="wheel_trans">
     <type>transmission_interface/SimpleTransmission</type>
     <joint name="wheel_joint">
       <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
     </joint>
   </transmission>
   ```

2. **کنٹرول کنفیگریشن فائل چیک کریں**:
   - یقینی بنائیں کہ YAML کنٹرول کنفیگ URDF سے مماثل ہے
   - یقینی بنائیں کہ جوائنٹ نام بالکل مماثل ہیں

## ڈیبگنگ کے نکات

### وربس لاگنگ فعال کریں

```bash
# وربس آؤٹ پٹ کے ساتھ لانچ کریں
gz sim -v 4 your_world.sdf
```

### ماڈل اسپاون چیک کریں

```bash
# اسپاون کردہ ماڈلز کی فہرست
gz model -m

# ماڈل پوز چیک کریں
gz model -m your_model_name -i
```

### SDF فائلوں کی توثیق کریں

```bash
# SDF سینٹیکس چیک کریں
gz sdf -k your_file.sdf

# URDF کو کنورٹ اور چیک کریں
gz sdf -p your_robot.urdf
```

## اپنے سیٹ اپ کی جانچ

اپنی گیزبو انسٹالیشن کی تصدیق کے لیے ایک سادہ ٹیسٹ بنائیں:

1. **ایک مختصر ورلڈ فائل بنائیں** (`test_world.sdf`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.10">
     <world name="test_world">
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>
       <model name="box">
         <pose>0 0 0.5 0 0 0</pose>
         <link name="box_link">
           <visual name="visual">
             <geometry>
               <box><size>1 1 1</size></box>
             </geometry>
           </visual>
           <collision name="collision">
             <geometry>
               <box><size>1 1 1</size></box>
             </geometry>
           </collision>
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
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **ٹیسٹ ورلڈ لانچ کریں**:
   ```bash
   gz sim -r test_world.sdf
   ```

3. **چیک کریں کہ باکس ظاہر ہوتا ہے اور فزکس کا جواب دیتا ہے**۔

## مدد حاصل کرنا

اگر آپ کو یہاں پر نہیں سنبھالے گئے مسائل کا سامنا ہوتا ہے:

1. **گیزبو دستاویزات چیک کریں**: [gazebosim.org](https://gazebosim.org/)
2. **ROS Answers**: [answers.ros.org](https://answers.ros.org)
3. **گیزبو کمیونٹی فورم**: [community.gazebosim.org](https://community.gazebosim.org)

## اگلے اقدامات

جب آپ کوائف سیٹ اپ کے مسائل حل کر لیں، تو [یوآرڈی ایف اور ایس ڈی ایف روبوٹ کی تفصیل کے فارمیٹس](../urdf-sdf-formats/urdf-basics.md) کے بارے میں سیکھنے کے لیے جاری رکھیں۔