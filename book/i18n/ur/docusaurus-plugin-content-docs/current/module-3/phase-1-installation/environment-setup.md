---
sidebar_position: 9
---

# Isaac Sim ماحول سیٹ اپ

یہ حصہ ہیومنوائڈ روبوٹکس ترقی کے لیے NVIDIA Isaac Sim ماحول کے سیٹ اپ اور کنفیگریشن کو احاطہ کرتا ہے۔ ہم انسٹالیشن کے عمل، سسٹم کنفیگریشن، اور ابتدائی سیٹ اپ اقدامات سے گزریں گے۔

## سسٹم کی تیاری

Isaac Sim انسٹال کرنے سے پہلے، یقینی بنائیں کہ آپ کا سسٹم مناسب طریقے سے تیار ہے:

### سسٹم پیکجز اپ ڈیٹ کریں
```bash
sudo apt update && sudo apt upgrade -y
```

### ضروری انحصار انسٹال کریں
```bash
sudo apt install build-essential cmake git python3-dev python3-pip python3-venv
```

### GPU اور ڈرائیور انسٹالیشن کی تصدیق کریں
```bash
nvidia-smi
```

آپ کو اپنے NVIDIA GPU کو ڈرائیور ورژن کے ساتھ فہرست میں دیکھنا چاہیے۔ Isaac Sim کو جدید NVIDIA GPU کی ضرورت ہوتی ہے جس میں تازہ ترین ڈرائیورز انسٹال ہوں۔

## Isaac Sim انسٹالیشن کے اختیارات

Isaac Sim کو مختلف طریقوں سے انسٹال کیا جا سکتا ہے آپ کی ضروریات کے مطابق:

### اختیار 1: Isaac Sim Omniverse ایپ (مبتدیوں کے لیے تجویز کردہ)

1. **Isaac Sim ڈاؤن لوڈ کریں**:
   - NVIDIA ڈویلپر ویب سائٹ پر جائیں
   - Isaac Sim Omniverse ایپ ڈاؤن لوڈ کریں
   - یہ ایک مکمل، پیش کنفیگر ماحول فراہم کرتا ہے

2. **Omniverse لانچر کے ذریعے انسٹال کریں**:
   - Omniverse لانچر انسٹال کریں
   - Isaac Sim انسٹال کرنے کے لیے لانچر کا استعمال کریں
   - لانچر انحصار اور اپ ڈیٹس کا انتظام کرتا ہے

### اختیار 2: Isaac Sim Docker (ترقی کے لیے تجویز کردہ)

یہ اختیار زیادہ سے زیادہ کنٹرول فراہم کرتا ہے اور ترقی کے کام کے لیے مناسب ہے:

1. **Docker انسٹال کریں** (اگر پہلے سے انسٹال نہ ہو):
```bash
sudo apt install docker.io
sudo usermod -aG docker $USER
```
   - گروپ کی تبدیلیوں کے اثر میں آنے کے لیے لاگ آؤٹ کریں اور دوبارہ لاگ ان کریں

2. **NVIDIA کنٹینر ٹول کٹ انسٹال کریں**:
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker
```

3. **Isaac Sim Docker امیج حاصل کریں**:
```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
```

### اختیار 3: Isaac Sim Isaac ROS ترقی ماحول کے ذریعے

ROS 2 کے ساتھ گہرا انضمام کے لیے:

1. **ROS 2 Humble Hawksbill انسٹال کریں** (اگر پہلے سے انسٹال نہ ہو):
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

2. **Isaac ROS پیکجز انسٹال کریں**:
```bash
sudo apt install ros-humble-isaac-ros-* ros-humble-nitros-*
```

## Docker-مبنی انسٹالیشن (تجویز کردہ)

اس کورس کے لیے، ہم Docker-مبنی انسٹالیشن کی تجویز کرتے ہیں کیونکہ یہ فراہم کرتا ہے:

- علیحدہ ماحول
- مسلسل ترقی کا تجربہ
- آسان ورژن مینجمنٹ
- صاف انسٹالیشن

### Docker کے ساتھ Isaac Sim کا سیٹ اپ

1. **ایک پروجیکٹ ڈائرکٹری بنائیں**:
```bash
mkdir ~/isaac_sim_project
cd ~/isaac_sim_project
```

2. **ایک docker-compose.yml فائل بنائیں**:
```yaml
version: '3.8'
services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:2023.1.1
    environment:
      - "NVIDIA_VISIBLE_DEVICES=all"
      - "NVIDIA_DRIVER_CAPABILITIES=all"
      - "QT_X11_NO_MITSHM=1"
      - "DISPLAY=${DISPLAY}"
      - "XAUTHORITY=/tmp/.docker.xauth"
    volumes:
      - /tmp/.X11-unix:/tmp/.x11-unix:rw
      - /tmp/.docker.xauth:/tmp/.docker.xauth:rw
      - ./workspace:/workspace:rw
      - ./isaac-sim-cache:/isaac-sim/cache:rw
    network_mode: host
    stdin_open: true
    tty: true
    runtime: nvidia
    privileged: true
```

3. **Isaac Sim شروع کریں**:
```bash
xhost +local:docker
docker-compose up -d
```

## تصدیق اور ابتدائی کنفیگریشن

### انسٹالیشن کی جانچ

1. **Isaac Sim لانچ کریں** (اگر Docker استعمال کر رہے ہیں):
```bash
docker exec -it isaac_sim_project_isaac-sim_1 bash
```

2. **ایک بنیادی ٹیسٹ چلائیں**:
```bash
cd /isaac-sim
python3 -c "import omni; print('Isaac Sim Python API is working!')"
```

### ابتدائی کنفیگریشن

1. **اپنا ورک سپیس سیٹ کریں**:
```bash
mkdir -p ~/isaac_sim_workspace/{scenes,robots,scripts,configs}
```

2. **ماحولیاتی متغیرات کنفیگر کریں** (~/.bashrc میں شامل کریں):
```bash
export ISAAC_SIM_PATH=/isaac-sim  # یا آپ کی Isaac Sim انسٹالیشن کا راستہ
export NVIDIA_OMNIVERSE_LOGGING_LEVEL=0
export ISAACSIM_PYTHON_EXE=/isaac-sim/python.sh
```

3. **ایک بنیادی ٹیسٹ اسکرپٹ بنائیں** تاکہ ہر چیز کام کرتی ہو کی تصدیق ہو:
```python
#!/usr/bin/env python3
"""
Basic Isaac Sim test script
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Dunia ko shuru karen
my_world = World(stage_units_in_meters=1.0)

# Ek saday robot zayed karen (is example mein Franka Panda)
assets_root_path = get_assets_root_path()
if assets_root_path is not None:
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/Franka"
    )

# Reset aur step dunia
my_world.reset()
for i in range(100):
    my_world.step(render=True)

print("Isaac Sim test successfully mukammal ho gaya!")
```

اسے اپنے ورک سپیس میں `test_isaac_sim.py` کے نام سے محفوظ کریں۔

## عام مسائل کو حل کرنا

### GPU تک رسائی کے مسائل
- یقینی بنائیں کہ آپ کا صارف docker گروپ میں ہے
- یقینی بنائیں کہ nvidia-container-toolkit انسٹال اور کنفیگر ہے
- یقینی بنائیں کہ NVIDIA ڈرائیور مناسب طریقے سے انسٹال ہے

### ڈسپلے کے مسائل
- یقینی بنائیں کہ X11 فارورڈنگ مناسب طریقے سے کنفیگر ہے
- سرخیل سسٹم کے لیے، VNC یا اس جیسے حل استعمال کرنا غور کریں
- یقینی بنائیں کہ DISPLAY ماحولیاتی متغیر صحیح طریقے سے سیٹ ہے

### میموری کے مسائل
- Isaac Sim میموری-زیادہ استعمال کرتا ہے؛ یقینی بنائیں کہ آپ کے پاس کافی RAM ہے
- `nvidia-smi` کے ساتھ GPU میموری استعمال کی نگرانی کریں
- اگر میموری کم ہو رہی ہو تو سیمولیشن کی پیچیدگی کم کرنا غور کریں

## اگلے اقدامات

جب آپ کا Isaac Sim ماحول مناسب طریقے سے سیٹ ہو جائے، تو آپ کر سکتے ہیں:

1. تعمیر شدہ مثالیں اور ٹیوٹوریلز دریافت کریں
2. اپنا پہلا سیمولیشن سین بنائیں
3. ہیومنوائڈ روبوٹکس کے لیے روبوٹ ماڈلز کنفیگر کریں
4. تاثرات کے کاموں کے لیے سینسر کنفیگریشنز سیٹ کریں

اگلا حصہ ہیومنوائڈ روبوٹس کے ساتھ اپنا پہلا تصویر نما سیمولیشن ماحول تخلیق کرنے کو احاطہ کرے گا۔