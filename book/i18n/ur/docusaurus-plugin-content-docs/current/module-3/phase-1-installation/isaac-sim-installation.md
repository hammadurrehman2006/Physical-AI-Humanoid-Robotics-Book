---
sidebar_position: 3
---

# Isaac Sim انسٹالیشن گائیڈ

یہ جامع گائیڈ آپ کو ہیومنوائڈ روبوٹکس ترقی کے لیے NVIDIA Isaac Sim انسٹال کرنے کے عمل سے گزارتا ہے۔ ہم انسٹالیشن کا عمل، کنفیگریشن اقدامات، اور توثیق کی کارروائیوں کو احاطہ کریں گے۔

## انسٹالیشن کی ضروریات

Isaac Sim انسٹال کرنے سے پہلے، اس بات کو یقینی بنائیں کہ آپ کے پاس ہے:

- RTX 3080 یا بہتر کے ساتھ NVIDIA GPU (RTX 4090 تجویز کردہ)
- NVIDIA GPU ڈرائیورز انسٹال (ورژن 520 یا بعد کا)
- CUDA 11.8 یا بعد میں انسٹال
- Ubuntu 22.04 LTS
- کم از کم 100GB مفت ڈسک سپیس
- 32GB+ سسٹم RAM (64GB+ تجویز کردہ)

### ضروریات کی تصدیق کریں

ان کمانڈز کو چلائیں تاکہ یقینی بنایا جا سکے کہ آپ کا سسٹم تیار ہے:

```bash
# GPU چیک کریں
nvidia-smi

# CUDA ورژن چیک کریں
nvcc --version

# دستیاب ڈسک سپیس چیک کریں
df -h $HOME

# سسٹم میموری چیک کریں
free -h

# ROS 2 انسٹالیشن چیک کریں
source /opt/ros/humble/setup.bash
ros2 --version
```

## انسٹالیشن کے طریقے

Isaac Sim کو متعدد طریقوں سے انسٹال کیا جا سکتا ہے۔ ہم دو عام ترین طریقوں کو احاطہ کریں گے:

### طریقہ 1: Omniverse لانچر (مبتدیوں کے لیے تجویز کردہ)

1. **Omniverse لانچر ڈاؤن لوڈ کریں**
   - NVIDIA Omniverse ویب سائٹ پر جائیں
   - Omniverse لانچر ڈاؤن لوڈ اور انسٹال کریں
   - یہ Isaac Sim کے لیے ایک صارف دوست انٹرفیس فراہم کرتا ہے

2. **لینچر کے ذریعے Isaac Sim انسٹال کریں**
   - Omniverse لانچر لانچ کریں
   - ایپ کیٹلاگ میں Isaac Sim تلاش کریں
   - "Install" پر کلک کریں اور ہدایات پر عمل کریں
   - لانچر تمام انحصار کو ہینڈل کرے گا

3. **Isaac Sim لانچ کریں**
   - Isaac Sim شروع کرنے کے لیے Omniverse لانچر استعمال کریں
   - اپنے NVIDIA ڈویلپر اکاؤنٹ کے ساتھ سائن ان کریں
   - Isaac Sim ایک ڈیفالٹ سین کے ساتھ شروع ہوگا

### طریقہ 2: Docker انسٹالیشن (ڈویلپرز کے لیے تجویز کردہ)

یہ طریقہ بہتر کنٹرول فراہم کرتا ہے اور ترقی کے کام کے لیے مناسب ہے:

1. **Docker اور NVIDIA کنٹینر ٹول کٹ انسٹال کریں**
```bash
# Docker انسٹال کریں
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER

# NVIDIA کنٹینر ٹول کٹ انسٹال کریں
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker
```

2. **Isaac Sim Docker امیج حاصل کریں**
```bash
# تازہ ترین Isaac Sim امیج حاصل کریں
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# تصدیق کریں کہ امیج حاصل کیا گیا تھا
docker images | grep isaac-sim
```

3. **Docker کمپوز فائل بنائیں**
`docker-compose.yml` نام کی فائل بنائیں:

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

4. **Isaac Sim لانچ کریں**
```bash
# ضروری ڈائرکٹریز بنائیں
mkdir -p workspace isaac-sim-cache

# X11 تصدیق سیٹ اپ کریں
xhost +local:docker

# Isaac Sim شروع کریں
docker-compose up -d
```

## کنفیگریشن اور سیٹ اپ

### اپنا ورک سپیس سیٹ کرنا

1. **ایک مخصوص ورک سپیس ڈائرکٹری بنائیں**:
```bash
mkdir -p ~/isaac_sim_workspace
cd ~/isaac_sim_workspace
mkdir -p scenes robots scripts configs logs
```

2. **ماحولیاتی متغیرات کنفیگر کریں** (`~/.bashrc` میں شامل کریں):
```bash
# Isaac Sim ماحولیاتی متغیرات
export ISAAC_SIM_PATH=/path/to/your/isaac-sim-installation  # اپنی انسٹالیشن کے لیے ایڈجسٹ کریں
export NVIDIA_OMNIVERSE_LOGGING_LEVEL=0
export ISAACSIM_PYTHON_EXE=/isaac-sim/python.sh

# PATH میں شامل کریں
export PATH=$ISAAC_SIM_PATH/python.sh:$PATH
```

3. **اپنے شیل کنفیگریشن کو دوبارہ لوڈ کریں**:
```bash
source ~/.bashrc
```

### ابتدائی کنفیگریشن فائلیں

ایک بنیادی کنفیگریشن فائل `config/isaac_sim_config.yaml` بنائیں:

```yaml
# ہیومنوائڈ روبوٹکس کے لیے Isaac Sim کنفیگریشن
simulation:
  stage_units_in_meters: 1.0
  rendering_frequency: 60
  physics_frequency: 600
  enable_scene_cache: true

physics:
  solver_type: 0  # 0: TGS, 1: PGSP
  solver_position_iteration_count: 8
  solver_velocity_iteration_count: 2
  gpu_max_particle_contacts: 10240
  gpu_max_contact_pairs: 1024000

rendering:
  resolution:
    width: 1920
    height: 1080
  enable_livestreaming: false
  livestream_resolution:
    width: 1280
    height: 720

robot:
  default_end_effector: "panda_hand"
  default_gripper: "panda_finger"
  joint_damping: 0.1

sensors:
  camera:
    resolution:
      width: 640
      height: 480
    fov: 60
  lidar:
    samples: 1000000
    rotation_frequency: 10
    channels: 16

ai:
  reinforcement_learning:
    max_episode_length: 1000
    batch_size: 256
    learning_rate: 0.001
```

## انسٹالیشن کی توثیق

### بنیادی کارکردگی کا ٹیسٹ

1. **Isaac Sim شروع کریں** (اپنے منتخب کردہ طریقے کا استعمال کریں)

2. **ایک سادہ ٹیسٹ اسکرپٹ چلائیں**:
`test_basic.py` بنائیں:

```python
#!/usr/bin/env python3
"""
Beniadi Isaac Sim kari ke tajraibi
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid

# Dunia ko shuru karen
my_world = World(stage_units_in_meters=1.0)

# Zameen ka satah zayed karen
create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

# Cube zayed karen
my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=[0, 0, 1.0],
        size=0.1,
        mass=0.1
    )
)

# Reset aur step dunia
my_world.reset()
for i in range(100):
    my_world.step(render=True)

print("Basic Isaac Sim test completed successfully!")
print("Cube position after simulation:", my_world.scene.get_object("cube").get_world_pose()[0])
```

3. **ٹیسٹ چلائیں**:
```bash
python3 test_basic.py
```

### ہیومنوائڈ روبوٹ ٹیسٹ

ہیومنوائڈ روبوٹ کے ساتھ ایک مزید ترقی یافتہ ٹیسٹ بنائیں:

```python
#!/usr/bin/env python3
"""
Isaac Sim mein humanoid robot ka tajraibi
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.robots import Robot

# Dunia ko shuru karen
my_world = World(stage_units_in_meters=1.0)

# Assets root path hasil karen
assets_root_path = get_assets_root_path()
if assets_root_path is not None:
    # Ek humanoid robot zayed karen (ek saday robot ka istemal placeholder ke tor par)
    # Asli humanoid robots ke liye, aapko mojooda URDF/USD files darjat karne ki zaroorat ho sakti hai
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd",
        prim_path="/World/Robot"
    )

    # Ek saday humanoid jaisi structure banayen
    robot = Robot(prim_path="/World/Robot", name="test_robot")

# Reset aur step dunia
my_world.reset()
for i in range(200):
    my_world.step(render=True)

    # Saday control loop (placeholder)
    if i == 100:
        print(f"Robot position at step {i}: {robot.get_world_pose()[0]}")

print("Humanoid robot test completed successfully!")
```

## انسٹالیشن کے مسائل کو حل کرنا

### عام انسٹالیشن کے مسائل اور حل

1. **GPU کا پتہ نہیں چل رہا**
   - یقینی بنائیں کہ NVIDIA ڈرائیورز انسٹال ہیں: `nvidia-smi`
   - CUDA انسٹالیشن چیک کریں: `nvcc --version`
   - یقینی بنائیں کہ صارف Docker گروپ میں ہے (Docker انسٹالیشنز کے لیے)

2. **Docker اجازت سے انکار**
   - صارف کو docker گروپ میں شامل کریں: `sudo usermod -aG docker $USER`
   - لاگ آؤٹ اور دوبارہ لاگ ان کریں
   - ٹیسٹ: `docker run hello-world`

3. **ڈسپلے/رینڈرنگ کے مسائل**
   - یقینی بنائیں کہ X11 فارورڈنگ ترتیب دی گئی ہے
   - Docker کے لیے: `xhost +local:docker`
   - DISPLAY ماحولیاتی متغیر چیک کریں

4. **میموری کے مسائل**
   - غیر ضروری ایپلی کیشنز بند کریں
   - GPU میموری مانیٹر کریں: `nvidia-smi`
   - سیمولیشن کی پیچیدگی کم کرنے پر غور کریں

5. **نیٹ ورک/ڈاؤن لوڈ کے مسائل**
   - انٹرنیٹ کنکشن چیک کریں
   - NVIDIA کنٹینر رجسٹری تک رسائی کی تصدیق کریں
   - کارپوریٹ نیٹ ورکس کے لیے، پراکسی ترتیبات چیک کریں

## پوسٹ-انسٹالیشن سیٹ اپ

### پروجیکٹ ٹیمپلیٹس بنانا

ایک پروجیکٹ ٹیمپلیٹ ڈائرکٹری ڈھانچہ بنائیں:

```bash
mkdir -p ~/isaac_sim_projects/humanoid_robot
cd ~/isaac_sim_projects/humanoid_robot

# معیاری ڈائرکٹریز بنائیں
mkdir -p scenes robots configs scripts logs data

# ایک بنیادی پروجیکٹ کنفیگریشن بنائیں
cat > project_config.yaml << EOF
project:
  name: "Humanoid Robot Simulation"
  version: "1.0.0"
  description: "Humanoid robot simulation and control project"

simulation:
  default_scene: "scenes/empty.usd"
  physics_frequency: 600
  rendering_frequency: 60

robot:
  model_path: "robots/humanoid.usd"
  initial_position: [0, 0, 0.8]
  initial_orientation: [0, 0, 0, 1]

sensors:
  enabled:
    - "camera_left"
    - "camera_right"
    - "lidar"
    - "imu"
    - "force_torque"

ai:
  reinforcement_learning:
    enabled: true
    algorithm: "ppo"
    max_episodes: 10000
EOF
```

### ترقی کے ماحول کو سیٹ کرنا

1. **Python مجازی ماحول**:
```bash
cd ~/isaac_sim_workspace
python3 -m venv isaac_sim_env
source isaac_sim_env/bin/activate
pip install --upgrade pip
```

2. **Isaac Sim Python انحصار انسٹال کریں**:
```bash
# Isaac Sim Python API انسٹال کریں
pip install omni-isaac-gym-py

# روبوٹکس کے لیے اضافی انحصار انسٹال کریں
pip install numpy scipy matplotlib transforms3d
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install stable-baselines3[extra]
```

## اگلے اقدامات

کامیاب انسٹالیشن کے بعد:

1. **Isaac Sim مثالیں دریافت کریں**: صلاحیات کو سمجھنے کے لیے تعمیر شدہ مثالوں کو براؤز کریں
2. **اپنا پہلا روبوٹ درآمد کریں**: روبوٹ ماڈلز (URDF/USD) درآمد کرنا سیکھیں
3. **سینسرز کنفیگر کریں**: کیمرے، LiDAR، اور دیگر سینسرز سیٹ کریں
4. **کنٹرول سسٹم لاگو کریں**: روبوٹ کنٹرول الگورتھم ترقی دیں
5. **سیمولیشن سناریوز تخلیق کریں**: ٹیسٹنگ کے لیے پیچیدہ ماحول بنائیں

اگلا حصہ ہیومنوائڈ روبوٹس کے لیے فوٹو ریلزم سیمولیشن ماحول تخلیق کرنے کو احاطہ کرتا ہے۔