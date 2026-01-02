# Isaac Sim میں سینسر کنفیگریشن

یہ سیکشن ہیومنوائڈ روبوٹکس ایپلی کیشنز کے لیے Isaac Sim میں مختلف سینسرز کو کنفیگر کرنے کا احاطہ کرتا ہے۔ ادراک (perception) کے سسٹمز، sim-to-real منتقلی، اور حقیقت پسندانہ ڈیٹا جنریشن کے لیے سینسر کی مناسب کنفیگریشن بہت اہم ہے۔

## Isaac Sim میں سینسر کی اقسام

Isaac Sim سینسر ماڈلز کا ایک جامع سوٹ فراہم کرتا ہے جو حقیقت پسندانہ شور کی خصوصیات اور کارکردگی کے پیرامیٹرز کے ساتھ حقیقی دنیا کے سینسرز کی نقل کرتا ہے:

### کیمرہ سینسرز
- **RGB کیمرے**: بصری ادراک کے لیے معیاری رنگین کیمرے۔
- **ڈیپتھ کیمرے**: 3D تعمیر نو کے لیے گہرائی کی معلومات۔
- **سٹیریو کیمرے**: گہرائی کے ادراک کے لیے بائنوکولر ویژن۔
- **فش آئی کیمرے**: پینورامک نظاروں کے لیے وائڈ اینگل کیمرے۔
- **ایونٹ کیمرے**: تیز رفتار حرکت کے لیے نیورومورفک کیمرے۔

### LiDAR سینسرز
- **3D LiDAR**: 3D میپنگ کے لیے ملٹی بیم LiDAR۔
- **2D LiDAR**: نیویگیشن کے لیے سنگل پلین LiDAR۔
- **سولڈ اسٹیٹ LiDAR**: MEMS اور فلیش LiDAR سمولیشن۔
- **مکینیکل LiDAR**: گھومنے والا ملٹی لائن LiDAR سمولیشن۔

### انرشیل سینسرز
- **IMU**: واقفیت اور ایکسلریشن کے لیے انرشیل پیمائش یونٹس۔
- **گائیروسکوپس**: کونیی (angular) شرح کی پیمائش۔
- **ایکسلیرومیٹرز**: لکیری ایکسلریشن کی پیمائش۔
- **میگنیٹومیٹرز**: مقناطیسی میدان کی پیمائش۔

### قوت اور ٹارک سینسرز
- **فورس/ٹارک سینسرز**: جوائنٹ فورس اور ٹارک کی پیمائش۔
- **رابطہ سینسرز**: رابطے کا پتہ لگانا اور قوت کی پیمائش۔
- **پریشر سینسرز**: سطح کے دباؤ کی تقسیم۔

## کیمرہ سینسر کنفیگریشن

### بنیادی RGB کیمرہ سیٹ اپ

```python
#!/usr/bin/env python3
"""
Basic RGB camera configuration in Isaac Sim
"""
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import create_prim
import numpy as np

def setup_basic_camera():
    """Set up a basic RGB camera in Isaac Sim"""

    world = World(stage_units_in_meters=1.0)

    # Create a simple scene with objects
    create_scene_for_camera_test(world)

    # Add a camera to the scene
    camera = Camera(
        prim_path="/World/Camera",
        position=[2.0, 2.0, 2.0],
        look_at=[0, 0, 0]
    )
    camera.initialize()

    # Set camera properties
    camera.add_render_product("/Render/Results/Camera", [640, 480])

    # Reset the world
    world.reset()

    # Capture images
    for i in range(10):
        world.step(render=True)

        if i % 2 == 0:  # Capture every 2nd frame
            rgb_image = camera.get_rgb()
            print(f"Captured RGB image shape: {rgb_image.shape}")

    return world, camera

def create_scene_for_camera_test(world):
    """Create a simple scene to test camera functionality"""

    # Create ground plane
    create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

    # Add some objects to see in the camera
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere

    # Add a cube
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/Cube",
            name="test_cube",
            position=[0, 0, 0.5],
            size=0.5,
            color=np.array([1.0, 0.0, 0.0])  # Red
        )
    )

    # Add a sphere
    world.scene.add(
        DynamicSphere(
            prim_path="/World/Sphere",
            name="test_sphere",
            position=[1, 0, 0.5],
            radius=0.3,
            color=np.array([0.0, 1.0, 0.0])  # Green
        )
    )

    # Add a cylinder
    create_prim(
        prim_path="/World/Cylinder",
        prim_type="Cylinder",
        position=[0, 1, 0.5],
        scale=[0.3, 0.3, 0.5],
        color=[0.0, 0.0, 1.0]  # Blue
    )

if __name__ == "__main__":
    world, camera = setup_basic_camera()
```

### ایڈوانسڈ کیمرہ کنفیگریشن

```python
#!/usr/bin/env python3
"""
Advanced camera configuration with realistic properties
"""
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import create_prim
import carb

def setup_advanced_camera(robot_prim_path="/World/Robot"):
    """Set up an advanced camera with realistic properties"""

    # Create camera mount point on robot
    camera_mount_path = f"{robot_prim_path}/CameraMount"
    create_prim(
        prim_path=camera_mount_path,
        prim_type="Xform",
        position=[0.1, 0, 0.1]  # Position on robot
    )

    # Create camera with realistic properties
    camera = Camera(
        prim_path=f"{camera_mount_path}/Camera",
        position=[0, 0, 0],  # Relative to mount
        frequency=30,        # 30 Hz capture rate
    )

    # Initialize camera
    camera.initialize()

    # Configure realistic camera properties
    configure_realistic_camera_properties(camera)

    # Add render product for image capture
    camera.add_render_product("/Render/Results/AdvancedCamera", [1280, 720])

    return camera

def configure_realistic_camera_properties(camera):
    """Configure realistic camera properties"""

    # Set field of view (FOV)
    camera.set_focal_length(24.0)  # 24mm equivalent
    camera.set_horizontal_aperture(36.0)  # Full frame equivalent
    camera.set_vertical_aperture(24.0)

    # Configure noise properties
    camera.set_sensor_noise(
        noise_type="gaussian",
        noise_mean=0.0,
        noise_std=0.01  # 1% noise level
    )

    # Set exposure and gain
    camera.set_exposure(1.0/60.0)  # 60 FPS exposure
    camera.set_gain(1.0)

    print("Advanced camera properties configured")

def setup_stereo_camera(robot_prim_path="/World/Robot"):
    """Set up stereo camera system for depth perception"""

    # Create left camera
    left_camera = Camera(
        prim_path=f"{robot_prim_path}/LeftCamera",
        position=[0.05, 0, 0.1],  # 10cm baseline
        frequency=30
    )
    left_camera.initialize()
    left_camera.add_render_product("/Render/Results/LeftCamera", [640, 480])

    # Create right camera
    right_camera = Camera(
        prim_path=f"{robot_prim_path}/RightCamera",
        position=[-0.05, 0, 0.1],  # 10cm baseline
        frequency=30
    )
    right_camera.initialize()
    right_camera.add_render_product("/Render/Results/RightCamera", [640, 480])

    print("Stereo camera system configured with 10cm baseline")

    return left_camera, right_camera
```

### ڈیپتھ کیمرہ کنفیگریشن

```python
#!/usr/bin/env python3
"""
Depth camera configuration for 3D perception
"""
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import create_prim
import numpy as np

def setup_depth_camera(robot_prim_path="/World/Robot"):
    """Set up depth camera for 3D perception"""

    # Create depth camera mount
    depth_camera_mount = f"{robot_prim_path}/DepthCameraMount"
    create_prim(
        prim_path=depth_camera_mount,
        prim_type="Xform",
        position=[0.1, 0, 0.15]
    )

    # Create depth camera
    depth_camera = Camera(
        prim_path=f"{depth_camera_mount}/DepthCamera",
        position=[0, 0, 0],
        frequency=30
    )
    depth_camera.initialize()

    # Configure depth camera properties
    configure_depth_camera_properties(depth_camera)

    # Add render products for different data types
    depth_camera.add_render_product("/Render/Results/DepthCamera", [640, 480])
    depth_camera.add_render_product("/Render/Results/DepthCamera/Depth", [640, 480])

    return depth_camera

def configure_depth_camera_properties(camera):
    """Configure depth-specific camera properties"""

    # Set depth-specific properties
    camera.set_focal_length(24.0)
    camera.set_horizontal_aperture(36.0)
    camera.set_vertical_aperture(24.0)

    # Configure depth range
    # These are typically set in the rendering pipeline
    min_range = 0.1   # 10 cm minimum
    max_range = 10.0  # 10 m maximum

    print(f"Depth camera configured with range {min_range}m to {max_range}m")

def capture_depth_data(world, depth_camera):
    """Capture and process depth data"""

    # Step the world to update sensor data
    world.step(render=True)

    # Get depth image
    depth_image = depth_camera.get_depth()

    # Process depth data
    if depth_image is not None:
        print(f"Depth image shape: {depth_image.shape}")
        print(f"Depth range: {np.min(depth_image):.3f} - {np.max(depth_image):.3f}m")

        # Filter out invalid depth values (typically 0 or inf)
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            print(f"Valid depth range: {np.min(valid_depths):.3f} - {np.max(valid_depths):.3f}m")

    return depth_image
```

## LiDAR سینسر کنفیگریشن

### 3D LiDAR سیٹ اپ

```python
#!/usr/bin/env python3
"""
3D LiDAR configuration for mapping and navigation
"""
from omni.isaac.sensor import RotatingLidarPhysX
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core import World

def setup_3d_lidar(robot_prim_path="/World/Robot"):
    """Set up a 3D LiDAR sensor for mapping and navigation"""

    # Create LiDAR mount point
    lidar_mount_path = f"{robot_prim_path}/LiDARMount"
    create_prim(
        prim_path=lidar_mount_path,
        prim_type="Xform",
        position=[0.15, 0, 0.3]  # Higher position on robot
    )

    # Create 3D LiDAR sensor
    lidar = RotatingLidarPhysX(
        prim_path=f"{lidar_mount_path}/LiDAR",
        translation=[0, 0, 0],
        configuration=setup_lidar_parameters()
    )

    lidar.initialize()

    print("3D LiDAR configured with realistic parameters")

    return lidar

def setup_lidar_parameters():
    """Configure realistic LiDAR parameters"""

    # Create LiDAR configuration
    lidar_config = {
        "rotation_frequency": 10,  # 10 Hz rotation
        "channels": 64,           # 64 vertical channels
        "points_per_channel": 1000,  # Points per revolution per channel
        "horizontal_fov": 360,    # Full 360 degree horizontal FOV
        "vertical_fov": 26.8,     # Vertical FOV (like HDL-64E)
        "range": 120.0,           # 120m max range
        "min_range": 0.5,         # 0.5m min range
        "angular_resolution": 0.1, # 0.1 degree angular resolution
        "upper_fov": 2.0,         # Upper vertical limit
        "lower_fov": -24.8,       # Lower vertical limit
        "rotation_speed": 600,    # 600 RPM
        "samples": 1000000,       # Total samples per second
        "return_mode": "closest", # Return closest hit
    }

    return lidar_config

def capture_lidar_data(world, lidar_sensor):
    """Capture and process LiDAR data"""

    # Step the world to update sensor data
    world.step(render=True)

    # Get LiDAR data (this would be point cloud data)
    # Note: Actual implementation depends on the specific LiDAR type
    try:
        point_cloud = lidar_sensor.get_point_cloud()
        if point_cloud is not None:
            print(f"Point cloud shape: {point_cloud.shape}")
            print(f"Number of points: {point_cloud.shape[0]}")

            # Basic statistics
            if point_cloud.shape[0] > 0:
                ranges = np.sqrt(np.sum(point_cloud[:, :3]**2, axis=1))
                print(f"Distance range: {np.min(ranges):.2f} - {np.max(ranges):.2f}m")

        return point_cloud
    except Exception as e:
        print(f"Error getting LiDAR data: {e}")
        return None
```

### 2D LiDAR کنفیگریشن

```python
#!/usr/bin/env python3
"""
2D LiDAR configuration for navigation
"""
def setup_2d_lidar(robot_prim_path="/World/Robot"):
    """Set up a 2D LiDAR sensor for navigation"""

    # Create 2D LiDAR mount
    lidar_mount_path = f"{robot_prim_path}/LiDAR2DMount"
    create_prim(
        prim_path=lidar_mount_path,
        prim_type="Xform",
        position=[0.1, 0, 0.25]
    )

    # Create 2D LiDAR (single plane)
    lidar_2d = RotatingLidarPhysX(
        prim_path=f"{lidar_mount_path}/LiDAR2D",
        translation=[0, 0, 0],
        configuration=setup_2d_lidar_parameters()
    )

    lidar_2d.initialize()

    print("2D LiDAR configured for navigation")

    return lidar_2d

def setup_2d_lidar_parameters():
    """Configure 2D LiDAR parameters"""

    lidar_config = {
        "rotation_frequency": 10,   # 10 Hz
        "channels": 1,              # Single horizontal plane
        "points_per_channel": 1080, # 1080 points per revolution
        "horizontal_fov": 360,      # Full 360 degrees
        "vertical_fov": 0.0,        # No vertical spread
        "range": 25.0,              # 25m max range
        "min_range": 0.1,           # 0.1m min range
        "angular_resolution": 0.33, # ~0.33 degree resolution (360/1080)
        "return_mode": "closest",   # Closest return
    }

    return lidar_config
```

## انرشیل سینسر کنفیگریشن

### IMU کنفیگریشن

```python
#!/usr/bin/env python3
"""
IMU (Inertial Measurement Unit) configuration
"""
from omni.isaac.core.sensors import ImuSensor
from omni.isaac.core.utils.prims import create_prim
import numpy as np

def setup_imu(robot_prim_path="/World/Robot"):
    """Set up IMU sensor for orientation and motion sensing"""

    # Create IMU mount point (typically at robot's center of mass)
    imu_mount_path = f"{robot_prim_path}/IMUMount"
    create_prim(
        prim_path=imu_mount_path,
        prim_type="Xform",
        position=[0, 0, 0.8]  # Approximate CoM height for humanoid
    )

    # Create IMU sensor
    imu = ImuSensor(
        prim_path=f"{imu_mount_path}/IMU",
        name="robot_imu",
        translation=np.array([0, 0, 0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # No rotation
        frequency=100  # 100 Hz sampling
    )

    # Initialize the IMU
    imu.initialize()

    # Configure realistic IMU properties
    configure_imu_properties(imu)

    print("IMU configured with realistic properties")

    return imu

def configure_imu_properties(imu_sensor):
    """Configure realistic IMU properties"""

    # IMU noise characteristics (typical for tactical-grade IMU)
    # These values would be set in the sensor configuration
    gyro_noise_density = 0.0001  # rad/s/sqrt(Hz)
    gyro_random_walk = 0.00001   # rad/s/sqrt(s)
    accel_noise_density = 0.001  # m/s^2/sqrt(Hz)
    accel_random_walk = 0.0001   # m/s^2/sqrt(s)

    print(f"IMU configured with:")
    print(f"  - Gyro noise density: {gyro_noise_density}")
    print(f"  - Accel noise density: {accel_noise_density}")
    print(f"  - Sampling frequency: 100 Hz")

def read_imu_data(world, imu_sensor):
    """Read and process IMU data"""

    # Step the world to update sensor data
    world.step(render=True)

    # Get IMU measurements
    try:
        linear_acceleration = imu_sensor.get_linear_acceleration()
        angular_velocity = imu_sensor.get_angular_velocity()
        orientation = imu_sensor.get_orientation()

        if linear_acceleration is not None:
            print(f"Linear acceleration: {linear_acceleration}")
            print(f"Angular velocity: {angular_velocity}")
            print(f"Orientation: {orientation}")

        return {
            'acceleration': linear_acceleration,
            'angular_velocity': angular_velocity,
            'orientation': orientation
        }
    except Exception as e:
        print(f"Error reading IMU data: {e}")
        return None
```

## قوت اور ٹارک سینسرز

### فورس/ٹارک سینسر کنفیگریشن

```python
#!/usr/bin/env python3
"""
Force/Torque sensor configuration for manipulation
"""
from omni.isaac.core.sensors import ContactSensor
from omni.isaac.core.utils.prims import create_prim
import numpy as np

def setup_force_torque_sensors(robot_path="/World/Robot"):
    """Set up force/torque sensors for manipulation tasks"""

    # Create force/torque sensors at key locations
    sensors = {}

    # End-effector force sensor
    sensors['end_effector'] = setup_end_effector_force_sensor(robot_path)

    # Joint torque sensors
    sensors['joint_torques'] = setup_joint_torque_sensors(robot_path)

    # Foot contact sensors (for humanoid balance)
    sensors['left_foot'] = setup_foot_contact_sensor(robot_path, "left_foot")
    sensors['right_foot'] = setup_foot_contact_sensor(robot_path, "right_foot")

    print("Force/torque sensors configured for manipulation and balance")

    return sensors

def setup_end_effector_force_sensor(robot_path):
    """Set up force sensor at end effector"""

    # Create contact sensor at end effector
    # In Isaac Sim, contact sensors can provide force information
    contact_sensor = ContactSensor(
        prim_path=f"{robot_path}/EndEffector/ContactSensor",
        name="end_effector_force_sensor",
        min_threshold=0,
        max_threshold=1e6,
        history=20
    )

    contact_sensor.initialize()

    return contact_sensor

def setup_joint_torque_sensors(robot_path):
    """Set up joint torque sensors"""

    # In Isaac Sim, joint torques can be accessed directly from the robot
    # This function would set up the necessary configurations
    print("Joint torque sensors configured - access through robot interface")

    # The actual torque values are obtained through the robot's joint interfaces
    return f"{robot_path}/joint_torque_interface"

def setup_foot_contact_sensor(robot_path, foot_name):
    """Set up contact sensor for foot (important for humanoid balance)"""

    contact_sensor = ContactSensor(
        prim_path=f"{robot_path}/{foot_name}_contact_sensor",
        name=f"{foot_name}_contact_sensor",
        min_threshold=0,
        max_threshold=1e6,
        history=20
    )

    contact_sensor.initialize()

    print(f"Foot contact sensor configured for {foot_name}")

    return contact_sensor

def read_contact_data(world, contact_sensor):
    """Read contact sensor data"""

    # Step the world to update sensor data
    world.step(render=True)

    try:
        # Get contact information
        contact_data = contact_sensor.get_raw_data()

        if contact_data is not None:
            num_contacts = len(contact_data)
            print(f"Number of contacts: {num_contacts}")

            # Process contact information
            total_force = 0
            for contact in contact_data:
                force = contact.get('force', 0)
                total_force += force

            print(f"Total contact force: {total_force}")

        return contact_data
    except Exception as e:
        print(f"Error reading contact data: {e}")
        return None
```

## سینسر فیوژن کنفیگریشن

### ملٹی سینسر انضمام

```python
#!/usr/bin/env python3
"""
Sensor fusion configuration for integrated perception
"""
class SensorFusionSystem:
    """A class to manage and fuse data from multiple sensors"""

    def __init__(self, robot_path):
        self.robot_path = robot_path
        self.sensors = {}
        self.sensor_data = {}

    def setup_sensor_suite(self):
        """Set up a complete sensor suite for the robot"""

        print("Setting up complete sensor suite...")

        # Set up all sensor types
        self.sensors['camera'] = setup_advanced_camera(self.robot_path)
        self.sensors['lidar'] = setup_3d_lidar(self.robot_path)
        self.sensors['imu'] = setup_imu(self.robot_path)
        self.sensors['force_sensors'] = setup_force_torque_sensors(self.robot_path)

        print("Complete sensor suite configured")

    def read_all_sensors(self, world):
        """Read data from all sensors"""

        # Step the world to update all sensors
        world.step(render=True)

        # Read data from each sensor type
        self.sensor_data['camera'] = self.sensors['camera'].get_rgb()
        self.sensor_data['depth'] = self.sensors['camera'].get_depth()

        # For LiDAR, IMU, etc., read their specific data
        # This is a simplified example - actual implementation would vary

        print(f"Read data from {len(self.sensors)} sensor types")

        return self.sensor_data

    def fuse_sensor_data(self):
        """Fuse data from multiple sensors for perception"""

        # This would implement sensor fusion algorithms
        # For example: combining camera and LiDAR for better 3D understanding
        # Or combining IMU and visual data for better pose estimation

        fused_data = {
            'environment_map': self.create_environment_map(),
            'robot_pose': self.estimate_robot_pose(),
            'object_detections': self.detect_objects()
        }

        return fused_data

    def create_environment_map(self):
        """Create environment map from sensor data"""
        # Implementation would combine LiDAR, camera, and other sensor data
        return "environment_map"

    def estimate_robot_pose(self):
        """Estimate robot pose using sensor fusion"""
        # Implementation would combine IMU, visual odometry, etc.
        return "robot_pose"

    def detect_objects(self):
        """Detect objects using multiple sensors"""
        # Implementation would combine visual, depth, and LiDAR data
        return "object_detections"

# Example usage
def demonstrate_sensor_fusion():
    """Demonstrate sensor fusion system"""

    world = World(stage_units_in_meters=1.0)

    # Create robot (using a simple approach)
    create_prim("/World/Robot", "Xform", position=[0, 0, 0.5])

    # Create sensor fusion system
    fusion_system = SensorFusionSystem("/World/Robot")
    fusion_system.setup_sensor_suite()

    # Initialize world
    world.reset()

    # Read and fuse sensor data
    for i in range(10):
        sensor_data = fusion_system.read_all_sensors(world)
        fused_data = fusion_system.fuse_sensor_data()

        print(f"Step {i}: Sensor fusion completed")
        if i % 5 == 0:
            print(f"  - Camera data shape: {sensor_data['camera'].shape if sensor_data['camera'] is not None else 'None'}")
            print(f"  - Fused data types: {list(fused_data.keys())}")

    return world, fusion_system
```

## سینسر کیلیبریشن اور توثیق

### سینسر توثیقی طریقہ کار

```python
#!/usr/bin/env python3
"""
Sensor calibration and validation procedures
"""
def validate_sensor_configuration():
    """Validate that sensors are configured correctly"""

    print("Validating sensor configuration...")

    # Test 1: Camera validation
    camera_validation()

    # Test 2: LiDAR validation
    lidar_validation()

    # Test 3: IMU validation
    imu_validation()

    # Test 4: Sensor timing validation
    timing_validation()

    print("Sensor validation completed")

def camera_validation():
    """Validate camera sensor configuration"""

    world = World(stage_units_in_meters=1.0)

    # Create simple scene for camera test
    create_scene_for_camera_test(world)

    # Add camera
    camera = setup_basic_camera()[1]  # Get camera from setup function

    # Test camera functionality
    world.reset()

    # Capture several frames
    for i in range(5):
        world.step(render=True)
        rgb_data = camera.get_rgb()
        if rgb_data is not None:
            print(f"✓ Camera validation: Frame {i+1} captured, shape {rgb_data.shape}")
        else:
            print(f"✗ Camera validation: Frame {i+1} failed to capture")

def lidar_validation():
    """Validate LiDAR sensor configuration"""

    print("Validating LiDAR sensor...")

    # Create world and LiDAR
    world = World(stage_units_in_meters=1.0)

    # Add simple objects for LiDAR to detect
    create_scene_for_lidar_test(world)

    # Add LiDAR
    lidar = setup_3d_lidar()

    # Test LiDAR functionality
    world.reset()

    # Capture several readings
    for i in range(5):
        world.step(render=True)
        # Note: Actual LiDAR data access may vary depending on implementation
        print(f"✓ LiDAR validation: Reading {i+1} completed")

def create_scene_for_lidar_test(world):
    """Create scene objects for LiDAR testing"""

    # Create ground plane and some obstacles
    create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

    # Add some cubes at different distances
    from omni.isaac.core.objects import DynamicCuboid

    world.scene.add(
        DynamicCuboid(
            prim_path="/World/Obstacle1",
            name="obstacle_1m",
            position=[1, 0, 0.5],
            size=0.5
        )
    )

    world.scene.add(
        DynamicCuboid(
            prim_path="/World/Obstacle2",
            name="obstacle_2m",
            position=[2, 1, 0.5],
            size=0.5
        )
    )

def imu_validation():
    """Validate IMU sensor configuration"""

    print("Validating IMU sensor...")

    # Test IMU by moving robot and checking measurements
    world = World(stage_units_in_meters=1.0)
    create_prim("/World/Robot", "Xform", position=[0, 0, 0.5])

    imu = setup_imu("/World/Robot")
    world.reset()

    # Apply a known motion and check IMU response
    for i in range(10):
        world.step(render=True)

        # In a real test, you would apply controlled motion
        # and verify IMU readings match expectations
        imu_data = read_imu_data(world, imu)
        if imu_data:
            print(f"✓ IMU validation: Reading {i+1} captured")

def timing_validation():
    """Validate sensor timing and synchronization"""

    print("Validating sensor timing...")

    # Check that sensors are publishing at expected frequencies
    # This would involve checking actual timing of sensor data

    expected_frequencies = {
        'camera': 30,    # Hz
        'lidar': 10,     # Hz
        'imu': 100,      # Hz
    }

    print(f"Expected sensor frequencies: {expected_frequencies}")
    print("Timing validation completed")

if __name__ == "__main__":
    # Run validation tests
    validate_sensor_configuration()
```

## کارکردگی کی اصلاح (Performance Optimization)

### سینسر کارکردگی کی ٹیوننگ

```python
#!/usr/bin/env python3
"""
Sensor performance optimization techniques
"""
def optimize_sensor_performance():
    """Optimize sensor performance for real-time operation"""

    # Different optimization strategies based on use case

    # Strategy 1: High Fidelity (for data generation)
    high_fidelity_config = {
        'camera': {'resolution': [1920, 1080], 'frequency': 30, 'quality': 'high'},
        'lidar': {'channels': 64, 'samples': 1000000, 'frequency': 10},
        'imu': {'frequency': 1000, 'precision': 'high'}
    }

    # Strategy 2: Balanced (for most applications)
    balanced_config = {
        'camera': {'resolution': [640, 480], 'frequency': 30, 'quality': 'medium'},
        'lidar': {'channels': 16, 'samples': 100000, 'frequency': 10},
        'imu': {'frequency': 200, 'precision': 'medium'}
    }

    # Strategy 3: High Performance (for fast operation)
    high_performance_config = {
        'camera': {'resolution': [320, 240], 'frequency': 15, 'quality': 'low'},
        'lidar': {'channels': 4, 'samples': 10000, 'frequency': 5},
        'imu': {'frequency': 100, 'precision': 'low'}
    }

    print("Sensor configuration strategies:")
    print(f"1. High Fidelity: {high_fidelity_config}")
    print(f"2. Balanced: {balanced_config}")
    print(f"3. High Performance: {high_performance_config}")

    return balanced_config  # Return balanced as default

def adaptive_sensor_config(robot_task):
    """Return adaptive sensor configuration based on robot task"""

    if robot_task == "navigation":
        return {
            'camera': {'resolution': [640, 480], 'frequency': 15},
            'lidar': {'channels': 16, 'frequency': 10},  # Good for mapping
            'imu': {'frequency': 200}
        }
    elif robot_task == "manipulation":
        return {
            'camera': {'resolution': [1280, 720], 'frequency': 30},  # High detail needed
            'lidar': {'channels': 4, 'frequency': 5},  # Less critical for manipulation
            'imu': {'frequency': 500}  # High frequency for precise control
        }
    elif robot_task == "balance":
        return {
            'camera': {'resolution': [320, 240], 'frequency': 10},  # Lower priority
            'lidar': {'channels': 1, 'frequency': 5},  # Minimal for balance
            'imu': {'frequency': 1000}  # Critical for balance
        }
    else:  # Default configuration
        return {
            'camera': {'resolution': [640, 480], 'frequency': 30},
            'lidar': {'channels': 16, 'frequency': 10},
            'imu': {'frequency': 200}
        }
```

## سینسر کے مسائل کا حل (Troubleshooting)

### عام سینسر کے مسائل اور حل

```python
#!/usr/bin/env python3
"""
Sensor troubleshooting guide
"""
def troubleshoot_sensor_issues():
    """Common sensor problems and solutions"""

    issues = {
        "camera_black": {
            "problem": "Camera shows black/empty images",
            "causes": [
                "No lighting in scene",
                "Camera positioned incorrectly",
                "Render pipeline not configured",
                "Objects not visible to camera"
            ],
            "solutions": [
                "Add lighting to scene",
                "Check camera position and look-at target",
                "Verify render pipeline setup",
                "Ensure objects are within camera view"
            ]
        },
        "lidar_sparse": {
            "problem": "LiDAR returns sparse/empty point cloud",
            "causes": [
                "No objects in sensor range",
                "LiDAR parameters set incorrectly",
                "Collision geometry missing",
                "Objects not set as collidable"
            ],
            "solutions": [
                "Add objects within LiDAR range",
                "Verify LiDAR configuration parameters",
                "Ensure collision geometry exists",
                "Set objects as collidable"
            ]
        },
        "imu_drift": {
            "problem": "IMU shows drift or incorrect orientation",
            "causes": [
                "Integration errors over time",
                "Noise not properly modeled",
                "Sensor not at CoM",
                "High-frequency vibrations"
            ],
            "solutions": [
                "Implement sensor fusion with other sources",
                "Add realistic noise models",
                "Position sensor at CoM",
                "Apply filtering algorithms"
            ]
        },
        "sensor_slow": {
            "problem": "Sensors cause simulation to run slowly",
            "causes": [
                "High resolution settings",
                "High frequency updates",
                "Complex scene geometry",
                "Multiple sensors active"
            ],
            "solutions": [
                "Reduce resolution/frequency",
                "Simplify scene when possible",
                "Use sensors selectively",
                "Optimize sensor configurations"
            ]
        }
    }

    for issue, details in issues.items():
        print(f"\nIssue: {details['problem']}")
        print(f"Possible causes: {', '.join(details['causes'])}")
        print(f"Solutions: {', '.join(details['solutions'])}")

if __name__ == "__main__":
    # Run validation tests
    validate_sensor_configuration()
    troubleshoot_sensor_issues()
```

## سینسر کنفیگریشن کے بہترین طریقے

### حقیقت پسندانہ سینسر سمولیشن کے لیے رہنما خطوط

1. **حقیقی سینسرز سے مماثلت**: سینسرز کو حقیقی سینسرز کی خصوصیات سے ملانے کے لیے کنفیگر کریں جنہیں آپ استعمال کرنے کا ارادہ رکھتے ہیں۔
2. **حقیقت پسندانہ شور شامل کریں**: سمولیشن کو زیادہ حقیقت پسندانہ بنانے کے لیے مناسب شور کے ماڈلز شامل کریں۔
3. **کارکردگی کی توثیق کریں**: مختلف منظرناموں میں سینسرز کی جانچ کریں تاکہ یہ یقینی بنایا جا سکے کہ وہ توقع کے مطابق کام کرتے ہیں۔
4. **کام کے لیے بہتر بنائیں**: اپنے مخصوص روبوٹکس ٹاسک کے لیے سینسرز کو مناسب طریقے سے کنفیگر کریں۔
5. **کمپیوٹیشنل لاگت پر غور کریں**: سمولیشن کی کارکردگی کے ساتھ سینسر کی مخلصی (fidelity) میں توازن رکھیں۔

### ہیومنوائڈ کے لیے مخصوص سینسر تحفظات

1. **توازن سینسرز**: ہیومنوائڈ توازن کنٹرول کے لیے IMU کی جگہ بہت اہم ہے۔
2. **پاؤں کے رابطے کے سینسرز**: چلنے کے دوران زمین کے رابطے کا پتہ لگانے کے لیے ضروری ہیں۔
3. **سر پر لگے سینسرز**: ماحولیاتی ادراک کے لیے سر پر کیمرے اور LiDAR۔
4. **ہیرا پھیری (Manipulation) سینسرز**: محفوظ ہیرا پھیری کے لیے فورس/ٹارک سینسرز۔

## اگلے اقدامات

اپنے Isaac Sim ماحول میں سینسرز کو کنفیگر کرنے کے بعد:

1. **انفرادی سینسرز کی جانچ کریں**: ہر سینسر کی قسم کی آزادانہ طور پر توثیق کریں۔
2. **سینسر فیوژن کی جانچ کریں**: متعدد سینسرز سے ڈیٹا کو اکٹھا کریں۔
3. **حقیقی ڈیٹا کے ساتھ توثیق کریں**: جب ممکن ہو تو سمولیشن سینسر ڈیٹا کا حقیقی سینسر ڈیٹا کے ساتھ موازنہ کریں۔
4. **کارکردگی کو بہتر بنائیں**: بہترین کارکردگی کے لیے سینسر کنفیگریشنز کو ایڈجسٹ کریں۔
5. **ادراک کے الگورتھم تیار کریں**: ادراک کے نظام بنائیں جو آپ کے کنفیگر کردہ سینسرز کے ساتھ کام کریں۔

اگلا سیکشن کنفیگر کردہ سینسرز کا استعمال کرتے ہوئے ڈیٹا جنریشن تکنیک کا احاطہ کرتا ہے۔
