---
title: ضروریات اور سیٹ اپ
description: فزیکل ای آئی اور ہیومنوڈ روبوٹکس کی ترقی کے لیے ضروری تقاضے اور سیٹ اپ کی کارروائیاں
sidebar_position: 7
---

# ضروریات اور سیٹ اپ

## سیکھنے کے اہداف
- فزیکل ای آئی کی ترقی کے لیے سافٹ ویئر اور ہارڈ ویئر کی ضروریات سمجھیں
- ROS 2 اور ہیومنوڈ روبوٹکس کے لیے ترقی کا ماحول سیٹ اپ کرنا سیکھیں
- ضروری ٹولز اور انحصار کی ترتیب کریں
- بنیادی ٹیسٹس کے ساتھ سیٹ اپ کی توثیق کریں

## سافٹ ویئر ضروریات

### آپریٹنگ سسٹم کی ضروریات

ROS 2 ہمبول ہاکسبل کے ساتھ فزیکل ای آئی کی ترقی کے لیے لینکس بیسڈ آپریٹنگ سسٹم کی ضرورت ہوتی ہے۔ اوبنٹو 22.04 ایل ٹی ایس تجویز کردہ تقسیم ہے۔

#### سسٹم کی ضروریات:
- **او ایس**: اوبنٹو 22.04 ایل ٹی ایس (جمی جیلی فش) یا بعد کا
- **آرکیٹیکچر**: 64-بٹ (ای ایم ڈی 64)
- **ریم**: کم از کم 8 جی بی (16 جی بی تجویز کردہ)
- **اسٹوریج**: کم از کم 50 جی بی خالی جگہ
- **پروسیسر**: ملٹی کور پروسیسر (انٹیل آئی5 یا اس کے برابر تجویز کردہ)

### پائی تھون کی ضروریات

ROS 2 ہمبول ہاکسبل کی مطابقت کے لیے پائی تھون 3.10+ کی ضرورت ہے:

```bash
# پائی تھون ورژن چیک کریں
python3 --version

# اگر پائی تھون 3.10+ انسٹال نہیں ہے:
sudo apt update
sudo apt install python3.10 python3.10-dev python3.10-venv python3-pip
```

### ROS 2 ہمبول ہاکسبل انسٹالیشن

ROS 2 ہمبول ہاکسبل ایل ٹی ایس ورژن ہے جو اس کوریکولم کے لیے ضروری ہے:

```bash
# ROS 2 جی پی جی کلید شامل کریں
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS 2 ریپوزیٹری شامل کریں
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 ہمبول پیکجز انسٹال کریں
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge ros-humble-tf2-py ros-humble-tf2-ros ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager
```

### اضافی انحصار

```bash
# اضافی پائی تھون پیکجز انسٹال کریں
pip3 install rclpy transforms3d numpy matplotlib opencv-python

# ترقی کے ٹولز انسٹال کریں
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# سیمولیشن ٹولز انسٹال کریں
sudo apt install gazebo libgazebo-dev
```

## ترقی کا ماحول سیٹ اپ

### ورک اسپیس کریشن

فزیکل ای آئی کی ترقی کے لیے ایک ROS 2 ورک اسپیس بنائیں:

```bash
# ورک اسپیس ڈائریکٹری بنائیں
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# ROS 2 ماحول ماخذ کریں
source /opt/ros/humble/setup.bash

# ورک اسپیس کو بلڈ کریں (یہاں تک کہ یہ خالی ہے)
colcon build --symlink-install

# بیش آر سی میں ماخذ شامل کریں
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### پائی تھون ورچوئل اینوائرمنٹ سیٹ اپ

صرف پائی تھون کی ترقی اور تجربہ کاری کے لیے:

```bash
# ورچوئل اینوائرمنٹ بنائیں
cd ~/physical_ai_projects
python3 -m venv physical_ai_env
source physical_ai_env/bin/activate

# ضروری پیکجز انسٹال کریں
pip install numpy scipy matplotlib opencv-python transforms3d pygame

# ROS 2 پائی تھون کلائنٹ لائبریری انسٹال کریں
pip install ros2cli

# ضروریات کی فائل بنائیں
pip freeze > requirements.txt
```

## آئی ڈی ای اور ترقی کے ٹولز

### ویژول اسٹوڈیو کوڈ سیٹ اپ

ROS 2 کی ترقی کے لیے وی ایس کوڈ تجویز کردہ ہے:

```bash
# وی ایس کوڈ انسٹال کریں
wget -qO - https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# ROS 2 ایکسٹینشنز انسٹال کریں
code --install-extension ms-iot.vscode-ros
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
```

### ضروری وی ایس کوڈ ایکسٹینشنز

- **ROS**: ROS 2 زبان کی حمایت اور ٹولز فراہم کرتا ہے
- **پائی تھون**: آفیشل پائی تھون ایکسٹینشن کے ساتھ انٹلی سینس
- **سی/سی++**: سی++ ROS 2 نوڈس کے لیے
- **ڈوکر**: کنٹینرائزڈ ترقی کے لیے
- **گٹ لینس**: بہتر گٹ کی صلاحیتیں

## سیمولیشن ماحول سیٹ اپ

### گزیبو انسٹالیشن اور کنفیگریشن

گزیبو ROS 2 کے لیے معیاری سیمولیشن ماحول ہے:

```bash
# گزیبو گارڈن انسٹال کریں (ROS 2 ہمبول کے لیے تجویز کردہ)
sudo apt install ros-humble-gazebo-* ros-humble-ign-*

# گزیبو انسٹالیشن کی ٹیسٹ کریں
gz sim --version
```

### یو آر ڈی ایف اور زیکرو سیٹ اپ

ہیومنوڈ روبوٹ ماڈلنگ کے لیے:

```bash
# یو آر ڈی ایف ٹولز انسٹال کریں
sudo apt install ros-humble-urdf ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# یو آر ڈی ایف ٹولز کی ٹیسٹ کریں
ros2 pkg create --build-type ament_python test_urdf --dependencies urdf xacro
```

## نظریاتی بنیاد: ماحول سیٹ اپ کے تصورات

فزیکل ای آئی ترقی کے ماحول کے اجزاء اور ضروریات کو سمجھنا کامیاب پروجیکٹ نافذ کاری کے لیے اہم ہے۔ یہ نظریاتی بنیاد وہ اہم عناصر کو احاطہ کرتی ہے جو ایک مضبوط ترقی کا سیٹ اپ تیار کرنے کے لیے ضروری ہیں۔

### بنیادی سسٹم کی ضروریات

ایک فزیکل ای آئی ترقی کا ماحول کئی بنیادی اجزاء کا تقاضا کرتا ہے:

- **آپریٹنگ سسٹم**: لینکس بیسڈ سسٹم (اوبنٹو 22.04 ایل ٹی ایس تجویز کردہ) ROS 2 اور سیمولیشن ٹولز کے ساتھ مطابقت فراہم کرتا ہے
- **پائی تھون ماحول**: پائی تھون 3.10+ ROS 2 ہمبول ہاکسبل اور متعلقہ لائبریریز کے ساتھ مطابقت کو یقینی بناتا ہے
- **ROS 2 فریم ورک**: روبوٹ آپریٹنگ سسٹم 2 روبوٹکس کی ترقی کے لیے مڈل ویئر اور ٹولز فراہم کرتا ہے
- **سیمولیشن ماحول**: گزیبو اور متعلقہ ٹولز جسمانی ہارڈ ویئر کے بغیر ٹیسٹنگ اور توثیق کے لیے

### ترقی ٹول چین کے اجزاء

مکمل ترقی ٹول چین کئی مربوط عناصر پر مشتمل ہے:

- **بلڈ سسٹم**: ROS 2 پیکجز اور ورک اسپیسز کو بلڈ کرنے کے لیے کولکون
- **ورژن کنٹرول**: تعاونی ترقی کے لیے مناسب کنفیگریشن کے ساتھ گٹ
- **آئی ڈی ای انضمام**: مؤثر ترقی کے لیے ROS 2 ایکسٹینشنز کے ساتھ کوڈ ایڈیٹر
- **پیکج مینجمنٹ**: سسٹم لیول (ایپٹ) اور پائی تھون لیول (پیپ) دونوں کے پیکج مینیجرز

### توثیق اور ٹیسٹنگ فریم ورک

مناسب ماحول کی توثیق تمام اجزاء کے ساتھ کام کرنے کو یقینی بناتی ہے:

- **انحصار کی تصدیق**: تمام ضروری پیکجز اور لائبریریز انسٹال ہیں یا نہیں چیک کرنا
- **رَن ٹائم ماحول**: ROS 2 ماحولیاتی متغیرات مناسب طریقے سے سیٹ ہیں یا نہیں تصدیق کرنا
- **بنیادی فعالیت کے ٹیسٹ**: مواصلت کی توثیق کے لیے سادہ پبلشر/سبسکرائبر ٹیسٹ
- **سیمولیشن انضمام**: یقینی بنانا کہ سیمولیشن ٹولز مناسب طریقے سے لانچ اور کنٹرول کیے جا سکتے ہیں

## ترقی ورک فلو سیٹ اپ

### ROS پروجیکٹس کے لیے گٹ کنفیگریشن

```bash
# ROS ترقی کے لیے گٹ کنفیگر کریں
git config --global user.name "آپ کا نام"
git config --global user.email "your.email@example.com"
git config --global core.precomposeunicode true

# گٹ ہکس ROS پروجیکٹس کے لیے سیٹ کریں (اختیاری)
# ہکس کے لیے ایک ٹیمپلیٹ ڈائریکٹری بنائیں
mkdir -p ~/.git_template/hooks

# ROS پیکج فارمیٹ چیک کرنے کے لیے مثال پری کمٹ ہک
cat > ~/.git_template/hooks/pre-commit << 'EOF'
#!/bin/bash
# ROS پیکجز کے لیے پری کمٹ ہک

# چیک کریں کہ کیا کوئی package.xml فائلیں تبدیل کی گئی ہیں
if git diff --cached --name-only | grep -q "package.xml"; then
    echo "package.xml فارمیٹ چیک کر رہا ہے..."
    # ضرورت کے مطابق ROS پیکج کی توثیق شامل کریں
fi

exit 0
EOF

chmod +x ~/.git_template/hooks/pre-commit

# ٹیمپلیٹ موجودہ ریپوزیٹریز پر لاگو کریں
git config --global init.templatedir '~/.git_template'
```

### وی ایس کوڈ ورک اسپیس کنفیگریشن

ROS 2 ترقی کے لیے ایک `.vscode/settings.json` فائل بنائیں:

```json
{
    "python.defaultInterpreterPath": "~/physical_ai_env/bin/python",
    "python.terminal.activateEnvironment": true,
    "ros.distro": "humble",
    "files.associations": {
        "*.msg": "yaml",
        "*.srv": "yaml",
        "*.action": "yaml"
    },
    "cmake.configureOnOpen": true,
    "C_Cpp.default.compilerPath": "/usr/bin/gcc",
    "C_Cpp.default.cStandard": "c17",
    "C_Cpp.default.cppStandard": "c++17"
}
```

## عام سیٹ اپ مسائل کا حل

### 1. ROS 2 ماحول ماخذ نہیں کیا گیا

```bash
# مسئلہ: کمانڈ 'ros2' نہیں ملا
# حل: ROS 2 ماحول ماخذ کریں
source /opt/ros/humble/setup.bash

# مستقل بنانے کے لیے، ~/.bashrc میں شامل کریں:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. گزیبو کے لیے اجازت کے مسائل

```bash
# مسئلہ: گزیبو اجازت کے مسائل کی وجہ سے شروع نہیں ہوتا
# حل: صارف کی اجازت چیک کریں اور گزیبو کنفیگریشن ری سیٹ کریں
rm -rf ~/.gazebo
# گزیبو دوبارہ شروع کریں
```

### 3. پائی تھون پیکج امپورٹ کے ایرر

```bash
# مسئلہ: rclpy یا دیگر ROS پیکجز کے لیے امپورٹ ایرر
# حل: یقینی بنائیں کہ پائی تھون پیکجز صحیح ماحول میں انسٹال ہیں
pip3 install -U rclpy

# یا انسٹال کرنے سے پہلے ROS ماحول ماخذ کریں
source /opt/ros/humble/setup.bash
pip3 install rclpy
```

### 4. کولکون بلڈ کی ناکامیاں

```bash
# مسئلہ: کولکون بلڈ ضائع شدہ انحصار کی وجہ سے ناکام ہوتا ہے
# حل: ضائع شدہ انحصار انسٹال کریں
sudo apt update
sudo apt upgrade
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## کنٹینرائزڈ ترقی سیٹ اپ (اختیاری)

مطابق ترقی کے ماحول کے لیے، ڈوکر استعمال کرنا غور کریں:

```dockerfile
# فزیکل ای آئی ترقی کے لیے ڈوکرفائل
FROM osrf/ros:humble-desktop

# اضافی انحصار انسٹال کریں
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    vim \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# پائی تھون پیکجز انسٹال کریں
RUN pip3 install --upgrade pip
RUN pip3 install numpy scipy matplotlib opencv-python transforms3d

# ورک اسپیس سیٹ اپ کریں
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# ROS ماحول ماخذ کریں
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
```

کنٹینر کو بلڈ اور چلائیں:

```bash
# کنٹینر بلڈ کریں
docker build -t physical-ai-dev .

# ہوسٹ ایکس11 تک رسائی کے ساتھ جی یو آئی ایپلی کیشنز کے لیے چلائیں
xhost +local:docker
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    --name physical-ai-container \
    physical-ai-dev
```

## مزید سیکھنے کے لیے وسائل

- [ROS 2 ہمبول انسٹالیشن گائیڈ](https://docs.ros.org/en/humble/Installation.html)
- [اوبنٹو انسٹالیشن گائیڈ](https://ubuntu.com/tutorials/install-ubuntu-desktop)
- [پائی تھون ورچوئل اینوائرمنٹس گائیڈ](https://docs.python.org/3/tutorial/venv.html)
- [وی ایس کوڈ ROS ایکسٹینشن دستاویزات](https://github.com/ms-iot/vscode-ros)

## خلاصہ

فزیکل ای آئی کی ترقی کے لیے ایک مناسب ترقی کا ماحول سیٹ اپ کرنا انتہائی اہم ہے۔ اس میں ROS 2 ہمبول ہاکسبل انسٹال کرنا، پائی تھون 3.10+ کنفیگر کرنا، گزیبو جیسے سیمولیشن ٹولز سیٹ اپ کرنا، اور ایک مناسب ورک اسپیس بنانا شامل ہے۔ توثیق کے ٹیسٹ یقینی بناتے ہیں کہ تمام اجزاء صحیح طریقے سے ایک دوسرے کے ساتھ کام کرتے ہیں قبل از ترقی شروع کرنے کے۔ ایک اچھی طرح کنفیگر کردہ ماحول ترقی کے دوران وقت کی بچت کرے گا اور عام مسائل سے بچے گا۔