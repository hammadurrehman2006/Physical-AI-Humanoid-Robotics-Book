# Data Generation in Isaac Sim

This section covers synthetic data generation techniques using Isaac Sim for training perception models and developing AI systems for humanoid robotics. Synthetic data generation is a critical component of modern robotics development, enabling training of perception systems without requiring extensive real-world data collection.

## Overview of Synthetic Data Generation

### Why Synthetic Data?

Synthetic data generation in Isaac Sim provides several key advantages:

- **Safety**: Train perception models without risking real robots or environments
- **Cost-effectiveness**: Generate large datasets without expensive data collection
- **Control**: Create specific scenarios and edge cases on demand
- **Variety**: Generate diverse environmental conditions quickly
- **Annotation**: Automatically generate perfect ground truth labels
- **Scalability**: Generate datasets of any size needed for training

### Types of Synthetic Data

Isaac Sim can generate various types of data:

- **RGB Images**: Color images for visual perception
- **Depth Maps**: Depth information for 3D understanding
- **Point Clouds**: 3D point cloud data from LiDAR simulation
- **Semantic Segmentation**: Pixel-level semantic labels
- **Instance Segmentation**: Object instance labels
- **Bounding Boxes**: 2D and 3D bounding box annotations
- **Pose Data**: Object and robot pose information
- **Sensor Fusion Data**: Combined data from multiple sensors

## Setting Up Data Generation Pipeline

### Basic Data Generation Framework

```python
#!/usr/bin/env python3
"""
Basic synthetic data generation framework
"""
import os
import numpy as np
import cv2
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import create_prim
from PIL import Image
import json

class IsaacSimDataGenerator:
    """A class to manage synthetic data generation in Isaac Sim"""

    def __init__(self, output_dir="synthetic_data", scene_config=None):
        self.output_dir = output_dir
        self.scene_config = scene_config or {}
        self.world = None
        self.cameras = {}
        self.data_counter = 0

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "metadata"), exist_ok=True)

    def setup_world(self):
        """Set up the Isaac Sim world for data generation"""

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0/60.0,
            rendering_dt=1.0/60.0
        )

        # Create basic scene elements
        self.setup_scene()

        # Set up cameras for data capture
        self.setup_cameras()

        # Initialize the world
        self.world.reset()

    def setup_scene(self):
        """Set up the scene for data generation"""

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Add objects for data generation
        self.add_random_objects()

    def add_random_objects(self):
        """Add random objects to the scene for data generation"""

        from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder

        # Add various objects for training
        for i in range(5):
            obj_type = np.random.choice(["cube", "sphere", "cylinder"])
            position = [np.random.uniform(-2, 2), np.random.uniform(-2, 2), 0.5]

            if obj_type == "cube":
                self.world.scene.add(
                    DynamicCuboid(
                        prim_path=f"/World/Object_{i}",
                        name=f"object_{i}",
                        position=position,
                        size=np.random.uniform(0.2, 0.5),
                        color=np.random.rand(3)
                    )
                )
            elif obj_type == "sphere":
                self.world.scene.add(
                    DynamicSphere(
                        prim_path=f"/World/Object_{i}",
                        name=f"object_{i}",
                        position=position,
                        radius=np.random.uniform(0.2, 0.4),
                        color=np.random.rand(3)
                    )
                )
            elif obj_type == "cylinder":
                self.world.scene.add(
                    DynamicCylinder(
                        prim_path=f"/World/Object_{i}",
                        name=f"object_{i}",
                        position=position,
                        radius=np.random.uniform(0.2, 0.4),
                        height=np.random.uniform(0.3, 0.6),
                        color=np.random.rand(3)
                    )
                )

    def setup_cameras(self):
        """Set up cameras for data capture"""

        # Main camera
        main_camera = Camera(
            prim_path="/World/MainCamera",
            position=[3, 3, 2],
            look_at=[0, 0, 0],
            frequency=30
        )
        main_camera.initialize()
        main_camera.add_render_product("/Render/Results/MainCamera", [640, 480])

        self.cameras["main"] = main_camera

        # Secondary camera (different angle)
        secondary_camera = Camera(
            prim_path="/World/SecondaryCamera",
            position=[-3, 2, 2],
            look_at=[0, 0, 0],
            frequency=30
        )
        secondary_camera.initialize()
        secondary_camera.add_render_product("/Render/Results/SecondaryCamera", [640, 480])

        self.cameras["secondary"] = secondary_camera

    def capture_data_frame(self):
        """Capture a single frame of synthetic data"""

        # Step the world to update all sensors
        self.world.step(render=True)

        # Capture data from all cameras
        frame_data = {}
        for cam_name, camera in self.cameras.items():
            # Get RGB image
            rgb_image = camera.get_rgb()
            if rgb_image is not None:
                frame_data[f"{cam_name}_rgb"] = rgb_image

            # Get depth image
            depth_image = camera.get_depth()
            if depth_image is not None:
                frame_data[f"{cam_name}_depth"] = depth_image

        # Generate metadata
        metadata = self.generate_metadata()
        frame_data["metadata"] = metadata

        return frame_data

    def generate_metadata(self):
        """Generate metadata for the current frame"""

        # Get robot and object poses
        metadata = {
            "frame_id": self.data_counter,
            "timestamp": self.world.current_time_step_index * self.world.get_physics_dt(),
            "objects": [],
            "camera_poses": {}
        }

        # Add camera poses
        for cam_name, camera in self.cameras.items():
            pose = camera.get_world_pose()
            metadata["camera_poses"][cam_name] = {
                "position": pose[0].tolist(),
                "orientation": pose[1].tolist()
            }

        return metadata

    def save_data_frame(self, frame_data):
        """Save the captured data frame to disk"""

        frame_id = f"{self.data_counter:06d}"

        # Save RGB images
        for key, data in frame_data.items():
            if key.endswith("_rgb"):
                cam_name = key.replace("_rgb", "")
                image_path = os.path.join(self.output_dir, "images", f"{frame_id}_{cam_name}.png")

                # Convert to PIL Image and save
                if isinstance(data, np.ndarray):
                    # Ensure data is in the right format (0-255, uint8)
                    if data.dtype == np.float32 or data.dtype == np.float64:
                        data = np.clip(data * 255, 0, 255).astype(np.uint8)

                    img = Image.fromarray(data)
                    img.save(image_path)

            elif key.endswith("_depth"):
                cam_name = key.replace("_depth", "")
                depth_path = os.path.join(self.output_dir, "depth", f"{frame_id}_{cam_name}.npy")

                # Save depth as numpy array
                np.save(depth_path, data)

        # Save metadata
        metadata_path = os.path.join(self.output_dir, "metadata", f"{frame_id}.json")
        with open(metadata_path, 'w') as f:
            json.dump(frame_data.get("metadata", {}), f, indent=2)

        self.data_counter += 1

    def generate_dataset(self, num_frames=100):
        """Generate a complete dataset"""

        print(f"Generating dataset with {num_frames} frames...")

        for i in range(num_frames):
            # Capture a frame
            frame_data = self.capture_data_frame()

            # Save the frame
            self.save_data_frame(frame_data)

            # Print progress
            if (i + 1) % 10 == 0:
                print(f"Generated {i + 1}/{num_frames} frames")

        print(f"Dataset generation completed! {self.data_counter} frames saved to {self.output_dir}")

# Example usage
def run_data_generation_example():
    """Run an example data generation session"""

    # Create data generator
    generator = IsaacSimDataGenerator(
        output_dir="synthetic_robotics_data",
        scene_config={"num_objects": 5, "scene_size": 4.0}
    )

    # Set up the world
    generator.setup_world()

    # Generate a small dataset
    generator.generate_dataset(num_frames=50)

    return generator

if __name__ == "__main__":
    generator = run_data_generation_example()
```

## Domain Randomization

### Randomizing Scene Elements

Domain randomization is crucial for generating diverse synthetic data that can transfer to the real world:

```python
#!/usr/bin/env python3
"""
Domain randomization techniques for synthetic data generation
"""
import random
import numpy as np
from pxr import Gf, UsdLux
import omni.usd

class DomainRandomizer:
    """Class to handle domain randomization in Isaac Sim"""

    def __init__(self, world):
        self.world = world
        self.stage = omni.usd.get_context().get_stage()

    def randomize_lighting(self):
        """Randomize lighting conditions in the scene"""

        # Randomize dome light (environment lighting)
        dome_light = self.stage.GetPrimAtPath("/World/DomeLight")
        if dome_light.IsValid():
            # Randomize intensity (0.5 to 3.0)
            intensity = random.uniform(0.5, 3.0)
            dome_light.GetAttribute("inputs:intensity").Set(intensity)

            # Randomize color temperature (4000K to 8000K)
            color_temp = random.uniform(4000, 8000)
            rgb = self.color_temperature_to_rgb(color_temp)
            dome_light.GetAttribute("inputs:color").Set(Gf.Vec3f(*rgb))

        # Randomize directional lights
        self.randomize_directional_lights()

    def randomize_directional_lights(self):
        """Randomize directional lights in the scene"""

        # Look for existing directional lights
        for light_name in ["KeyLight", "FillLight", "BackLight"]:
            light_path = f"/World/{light_name}"
            light_prim = self.stage.GetPrimAtPath(light_path)

            if light_prim.IsValid():
                # Randomize intensity
                intensity = random.uniform(500, 1500)
                light_prim.GetAttribute("inputs:intensity").Set(intensity)

                # Randomize direction
                # Randomize rotation
                rot_x = random.uniform(-90, 90)
                rot_y = random.uniform(0, 360)
                rot_z = random.uniform(-30, 30)

                # Apply rotations (this is simplified - in practice you'd use proper transforms)
                # light_prim.GetAttribute("xformOp:rotateX").Set(rot_x)
                # light_prim.GetAttribute("xformOp:rotateY").Set(rot_y)
                # light_prim.GetAttribute("xformOp:rotateZ").Set(rot_z)

    def color_temperature_to_rgb(self, temp_k):
        """Convert color temperature in Kelvin to RGB"""
        temp_k = max(1000, min(40000, temp_k)) / 100.0

        if temp_k <= 66:
            red = 255
            green = temp_k
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            red = temp_k - 60
            red = 329.698727446 * (red ** -0.1332047592)
            green = temp_k - 60
            green = 288.1221695283 * (green ** -0.0755148492)

        if temp_k >= 66:
            blue = 255
        elif temp_k <= 19:
            blue = 0
        else:
            blue = temp_k - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307

        return (
            max(0, min(255, red)) / 255.0,
            max(0, min(255, green)) / 255.0,
            max(0, min(255, blue)) / 255.0
        )

    def randomize_materials(self):
        """Randomize material properties in the scene"""

        # This would involve iterating through materials and randomizing their properties
        # For now, we'll just outline the approach

        # 1. Randomize diffuse colors
        # 2. Randomize roughness
        # 3. Randomize metallic properties
        # 4. Randomize textures (if applicable)

        print("Materials randomized")

    def randomize_object_appearances(self):
        """Randomize object appearances in the scene"""

        # Randomize object colors, textures, and materials
        # This would involve accessing all objects in the scene and modifying their visual properties

        # Example for a specific object:
        # obj_prim = self.stage.GetPrimAtPath("/World/Object_0")
        # if obj_prim.IsValid():
        #     # Randomize color
        #     color = (random.random(), random.random(), random.random())
        #     obj_prim.GetAttribute("inputs:diffuse_color").Set(Gf.Vec3f(*color))

        print("Object appearances randomized")

    def randomize_object_positions(self):
        """Randomize object positions in the scene"""

        # For each object in the scene, apply random position variations
        # This would require identifying all objects in the scene

        print("Object positions randomized")

    def randomize_camera_parameters(self):
        """Randomize camera parameters for synthetic data diversity"""

        # Randomize camera positions, orientations, and intrinsic parameters
        # This would involve modifying camera properties in the USD stage

        print("Camera parameters randomized")

    def apply_randomization(self):
        """Apply all domain randomization techniques"""

        self.randomize_lighting()
        self.randomize_materials()
        self.randomize_object_appearances()
        self.randomize_object_positions()
        self.randomize_camera_parameters()

        print("Domain randomization applied successfully")
```

### Advanced Domain Randomization

```python
#!/usr/bin/env python3
"""
Advanced domain randomization techniques
"""
class AdvancedDomainRandomizer(DomainRandomizer):
    """Advanced domain randomization with more sophisticated techniques"""

    def __init__(self, world):
        super().__init__(world)
        self.randomization_config = self.get_default_config()

    def get_default_config(self):
        """Get default randomization configuration"""

        config = {
            "lighting": {
                "intensity_range": (0.5, 3.0),
                "color_temperature_range": (4000, 8000),
                "num_lights_range": (1, 4)
            },
            "materials": {
                "roughness_range": (0.0, 1.0),
                "metallic_range": (0.0, 1.0),
                "albedo_range": (0.0, 1.0)
            },
            "objects": {
                "position_jitter": 0.5,
                "rotation_jitter": 45.0,
                "scale_range": (0.8, 1.2)
            },
            "camera": {
                "position_jitter": 0.3,
                "rotation_jitter": 10.0,
                "fov_range": (30, 90)
            },
            "environment": {
                "fog_density_range": (0.0, 0.1),
                "background_options": ["outdoor", "indoor", "laboratory"]
            }
        }

        return config

    def randomize_environment(self):
        """Randomize environmental conditions"""

        # Randomize fog and atmospheric effects
        fog_density = random.uniform(
            self.randomization_config["environment"]["fog_density_range"][0],
            self.randomization_config["environment"]["fog_density_range"][1]
        )

        # Randomize background environment
        background_type = random.choice(
            self.randomization_config["environment"]["background_options"]
        )

        print(f"Environment randomized: fog={fog_density}, background={background_type}")

    def randomize_textures(self):
        """Randomize textures and surface patterns"""

        # This would involve applying random textures or procedural patterns
        # to objects in the scene
        print("Textures randomized")

    def randomize_physics_properties(self):
        """Randomize physics properties for more realistic simulation"""

        # Randomize friction coefficients
        # Randomize restitution (bounciness)
        # Randomize mass properties within reasonable ranges
        print("Physics properties randomized")

    def temporal_randomization(self):
        """Apply randomization that changes over time"""

        # Randomize properties that change gradually over the sequence
        # This helps create more natural-looking variations
        print("Temporal randomization applied")

    def apply_advanced_randomization(self):
        """Apply all advanced domain randomization techniques"""

        self.randomize_lighting()
        self.randomize_materials()
        self.randomize_object_appearances()
        self.randomize_object_positions()
        self.randomize_camera_parameters()
        self.randomize_environment()
        self.randomize_textures()
        self.randomize_physics_properties()
        self.temporal_randomization()

        print("Advanced domain randomization applied successfully")
```

## Semantic Segmentation Data Generation

### Generating Semantic Segmentation Labels

```python
#!/usr/bin/env python3
"""
Semantic segmentation data generation
"""
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.semantics import add_semantics
import omni.kit.commands
import omni.usd

class SemanticSegmentationGenerator:
    """Generate semantic segmentation data in Isaac Sim"""

    def __init__(self, world, output_dir="semantic_data"):
        self.world = world
        self.output_dir = output_dir
        self.camera = None
        self.semantic_labels = {}

        # Create output directories
        os.makedirs(os.path.join(output_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "semantic"), exist_ok=True)

    def setup_semantic_camera(self):
        """Set up camera for semantic segmentation capture"""

        self.camera = Camera(
            prim_path="/World/SemanticCamera",
            position=[3, 3, 2],
            look_at=[0, 0, 0],
            frequency=10  # Lower frequency for semantic data
        )
        self.camera.initialize()

        # Add render product for RGB
        self.camera.add_render_product("/Render/Results/SemanticRGB", [640, 480])

        # Add render product for semantic segmentation
        self.camera.add_render_product("/Render/Results/SemanticSegmentation", [640, 480])

    def assign_semantic_labels(self):
        """Assign semantic labels to objects in the scene"""

        # This is a simplified example - in practice, you would assign labels
        # to specific objects in your scene

        # Example: Assign labels to objects
        stage = omni.usd.get_context().get_stage()

        # Get all prims in the scene and assign semantic labels
        for prim in stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder"]:
                # Assign semantic label based on object type
                if prim.GetTypeName() == "Cube":
                    self._assign_semantic_label(prim, "obstacle")
                elif prim.GetTypeName() == "Sphere":
                    self._assign_semantic_label(prim, "ball")
                elif prim.GetTypeName() == "Cylinder":
                    self._assign_semantic_label(prim, "cylinder")

    def _assign_semantic_label(self, prim, label):
        """Assign a semantic label to a prim"""

        # Add semantic schema to the prim
        add_semantics(prim, "class", label)

        # Store label for later use
        self.semantic_labels[prim.GetPath().pathString] = label

    def capture_semantic_data(self):
        """Capture semantic segmentation data"""

        # Step the world
        self.world.step(render=True)

        # Get RGB image
        rgb_image = self.camera.get_rgb()

        # Get semantic segmentation
        # Note: Actual implementation may vary depending on Isaac Sim version
        try:
            semantic_data = self.camera.get_semantic_segmentation()
        except:
            semantic_data = None
            print("Semantic segmentation not available in this Isaac Sim version")

        return rgb_image, semantic_data

    def save_semantic_data(self, rgb_image, semantic_data, frame_id):
        """Save semantic segmentation data"""

        if rgb_image is not None:
            # Save RGB image
            rgb_path = os.path.join(self.output_dir, "rgb", f"{frame_id:06d}.png")
            if isinstance(rgb_image, np.ndarray):
                if rgb_image.dtype == np.float32 or rgb_image.dtype == np.float64:
                    rgb_image = np.clip(rgb_image * 255, 0, 255).astype(np.uint8)
                img = Image.fromarray(rgb_image)
                img.save(rgb_path)

        if semantic_data is not None:
            # Save semantic segmentation
            semantic_path = os.path.join(self.output_dir, "semantic", f"{frame_id:06d}.png")
            if isinstance(semantic_data, np.ndarray):
                # Convert semantic data to proper format
                semantic_img = Image.fromarray(semantic_data.astype(np.uint8))
                semantic_img.save(semantic_path)

    def generate_semantic_dataset(self, num_frames=100):
        """Generate a semantic segmentation dataset"""

        print(f"Generating semantic segmentation dataset with {num_frames} frames...")

        for i in range(num_frames):
            # Capture semantic data
            rgb_image, semantic_data = self.capture_semantic_data()

            # Save the data
            self.save_semantic_data(rgb_image, semantic_data, i)

            # Print progress
            if (i + 1) % 10 == 0:
                print(f"Generated {i + 1}/{num_frames} semantic frames")

        print(f"Semantic dataset generation completed!")
```

## Instance Segmentation Data Generation

### Generating Instance Segmentation Labels

```python
#!/usr/bin/env python3
"""
Instance segmentation data generation
"""
class InstanceSegmentationGenerator:
    """Generate instance segmentation data in Isaac Sim"""

    def __init__(self, world, output_dir="instance_data"):
        self.world = world
        self.output_dir = output_dir
        self.camera = None
        self.instance_labels = {}

        # Create output directories
        os.makedirs(os.path.join(output_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "instance"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "masks"), exist_ok=True)

    def setup_instance_camera(self):
        """Set up camera for instance segmentation capture"""

        self.camera = Camera(
            prim_path="/World/InstanceCamera",
            position=[3, 3, 2],
            look_at=[0, 0, 0],
            frequency=10
        )
        self.camera.initialize()

        # Add render product for RGB
        self.camera.add_render_product("/Render/Results/InstanceRGB", [640, 480])

        # Add render product for instance segmentation
        self.camera.add_render_product("/Render/Results/InstanceSegmentation", [640, 480])

    def assign_instance_labels(self):
        """Assign unique instance labels to objects"""

        # Assign unique IDs to each object instance
        instance_id = 1
        stage = omni.usd.get_context().get_stage()

        for prim in stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder"]:
                # Assign unique instance ID
                add_semantics(prim, "instance", f"instance_{instance_id}")
                self.instance_labels[prim.GetPath().pathString] = instance_id
                instance_id += 1

    def capture_instance_data(self):
        """Capture instance segmentation data"""

        # Step the world
        self.world.step(render=True)

        # Get RGB image
        rgb_image = self.camera.get_rgb()

        # Get instance segmentation data
        try:
            instance_data = self.camera.get_instance_segmentation()
        except:
            instance_data = None
            print("Instance segmentation not available in this Isaac Sim version")

        return rgb_image, instance_data

    def generate_instance_masks(self, instance_data):
        """Generate individual masks for each instance"""

        if instance_data is None:
            return []

        unique_instances = np.unique(instance_data)
        masks = []

        for instance_id in unique_instances:
            if instance_id == 0:  # Skip background
                continue

            # Create binary mask for this instance
            mask = (instance_data == instance_id).astype(np.uint8) * 255
            masks.append((instance_id, mask))

        return masks

    def save_instance_data(self, rgb_image, instance_data, masks, frame_id):
        """Save instance segmentation data"""

        if rgb_image is not None:
            # Save RGB image
            rgb_path = os.path.join(self.output_dir, "rgb", f"{frame_id:06d}.png")
            if isinstance(rgb_image, np.ndarray):
                if rgb_image.dtype == np.float32 or rgb_image.dtype == np.float64:
                    rgb_image = np.clip(rgb_image * 255, 0, 255).astype(np.uint8)
                img = Image.fromarray(rgb_image)
                img.save(rgb_path)

        if instance_data is not None:
            # Save instance segmentation
            instance_path = os.path.join(self.output_dir, "instance", f"{frame_id:06d}.png")
            if isinstance(instance_data, np.ndarray):
                instance_img = Image.fromarray(instance_data.astype(np.uint8))
                instance_img.save(instance_path)

        # Save individual masks
        for instance_id, mask in masks:
            mask_path = os.path.join(self.output_dir, "masks", f"{frame_id:06d}_instance_{instance_id}.png")
            mask_img = Image.fromarray(mask)
            mask_img.save(mask_path)

    def generate_instance_dataset(self, num_frames=100):
        """Generate an instance segmentation dataset"""

        print(f"Generating instance segmentation dataset with {num_frames} frames...")

        for i in range(num_frames):
            # Capture instance data
            rgb_image, instance_data = self.capture_instance_data()

            # Generate masks
            masks = self.generate_instance_masks(instance_data)

            # Save the data
            self.save_instance_data(rgb_image, instance_data, masks, i)

            # Print progress
            if (i + 1) % 10 == 0:
                print(f"Generated {i + 1}/{num_frames} instance frames")

        print(f"Instance dataset generation completed!")
```

## 3D Object Detection Data Generation

### Generating 3D Detection Labels

```python
#!/usr/bin/env python3
"""
3D object detection data generation
"""
from omni.isaac.sensor import RotatingLidarPhysX
import json

class Detection3DGenerator:
    """Generate 3D object detection data in Isaac Sim"""

    def __init__(self, world, output_dir="detection_3d_data"):
        self.world = world
        self.output_dir = output_dir
        self.rgb_camera = None
        self.lidar = None

        # Create output directories
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "pointclouds"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)

    def setup_sensors(self):
        """Set up sensors for 3D detection data generation"""

        # RGB camera
        self.rgb_camera = Camera(
            prim_path="/World/RGBCamera",
            position=[3, 3, 2],
            look_at=[0, 0, 0],
            frequency=10
        )
        self.rgb_camera.initialize()
        self.rgb_camera.add_render_product("/Render/Results/RGBCamera", [640, 480])

        # LiDAR for 3D data
        self.lidar = RotatingLidarPhysX(
            prim_path="/World/LiDAR",
            translation=[0, 0, 1.5],  # Mount on robot
            configuration={
                "rotation_frequency": 10,
                "channels": 16,
                "points_per_channel": 1000,
                "horizontal_fov": 360,
                "vertical_fov": 30,
                "range": 25.0,
                "min_range": 0.1,
                "angular_resolution": 0.18,
            }
        )
        self.lidar.initialize()

    def get_object_poses(self):
        """Get poses of all objects in the scene for ground truth"""

        # This would involve getting the pose of each object in the scene
        # For now, we'll simulate this with a simplified approach

        object_poses = []
        stage = omni.usd.get_context().get_stage()

        for prim in stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder"]:
                # Get object pose
                try:
                    # This is a simplified approach - actual implementation
                    # would use Isaac Sim's pose APIs
                    position = [0, 0, 0]  # Placeholder
                    rotation = [0, 0, 0, 1]  # Placeholder (quaternion)

                    obj_info = {
                        "name": prim.GetName(),
                        "type": prim.GetTypeName(),
                        "position": position,
                        "rotation": rotation,
                        "size": [1.0, 1.0, 1.0]  # Placeholder
                    }
                    object_poses.append(obj_info)
                except:
                    continue

        return object_poses

    def capture_detection_data(self):
        """Capture RGB and LiDAR data for 3D detection"""

        # Step the world
        self.world.step(render=True)

        # Get RGB image
        rgb_image = self.rgb_camera.get_rgb()

        # Get LiDAR point cloud
        try:
            point_cloud = self.lidar.get_point_cloud()
        except:
            point_cloud = None
            print("LiDAR data not available")

        # Get object poses for ground truth
        object_poses = self.get_object_poses()

        return rgb_image, point_cloud, object_poses

    def save_detection_data(self, rgb_image, point_cloud, object_poses, frame_id):
        """Save 3D detection data"""

        if rgb_image is not None:
            # Save RGB image
            image_path = os.path.join(self.output_dir, "images", f"{frame_id:06d}.png")
            if isinstance(rgb_image, np.ndarray):
                if rgb_image.dtype == np.float32 or rgb_image.dtype == np.float64:
                    rgb_image = np.clip(rgb_image * 255, 0, 255).astype(np.uint8)
                img = Image.fromarray(rgb_image)
                img.save(image_path)

        if point_cloud is not None:
            # Save point cloud as numpy array
            pc_path = os.path.join(self.output_dir, "pointclouds", f"{frame_id:06d}.npy")
            np.save(pc_path, point_cloud)

        # Save object poses as labels
        label_path = os.path.join(self.output_dir, "labels", f"{frame_id:06d}.json")
        with open(label_path, 'w') as f:
            json.dump({
                "frame_id": frame_id,
                "objects": object_poses,
                "timestamp": self.world.current_time_step_index * self.world.get_physics_dt()
            }, f, indent=2)

    def generate_detection_dataset(self, num_frames=100):
        """Generate a 3D detection dataset"""

        print(f"Generating 3D detection dataset with {num_frames} frames...")

        for i in range(num_frames):
            # Capture detection data
            rgb_image, point_cloud, object_poses = self.capture_detection_data()

            # Save the data
            self.save_detection_data(rgb_image, point_cloud, object_poses, i)

            # Print progress
            if (i + 1) % 10 == 0:
                print(f"Generated {i + 1}/{num_frames} detection frames")

        print(f"3D detection dataset generation completed!")
```

## Data Quality and Validation

### Validating Synthetic Data Quality

```python
#!/usr/bin/env python3
"""
Data quality validation for synthetic datasets
"""
import matplotlib.pyplot as plt

class DataQualityValidator:
    """Validate the quality of synthetic data"""

    def __init__(self, data_dir):
        self.data_dir = data_dir

    def validate_image_quality(self, num_samples=10):
        """Validate RGB image quality"""

        image_dir = os.path.join(self.data_dir, "images")
        if not os.path.exists(image_dir):
            print("Image directory not found")
            return

        image_files = [f for f in os.listdir(image_dir) if f.endswith(('.png', '.jpg', '.jpeg'))]
        sample_files = np.random.choice(image_files, min(num_samples, len(image_files)), replace=False)

        print(f"Validating {len(sample_files)} sample images...")

        for file in sample_files:
            img_path = os.path.join(image_dir, file)
            img = cv2.imread(img_path)

            if img is None:
                print(f"❌ Failed to load image: {file}")
                continue

            # Check basic image properties
            height, width, channels = img.shape
            mean_intensity = np.mean(img)

            print(f"✅ {file}: {width}x{height}, {channels} channels, mean={mean_intensity:.2f}")

    def validate_depth_data(self, num_samples=10):
        """Validate depth data quality"""

        depth_dir = os.path.join(self.data_dir, "depth")
        if not os.path.exists(depth_dir):
            print("Depth directory not found")
            return

        depth_files = [f for f in os.listdir(depth_dir) if f.endswith('.npy')]
        sample_files = np.random.choice(depth_files, min(num_samples, len(depth_files)), replace=False)

        print(f"Validating {len(sample_files)} sample depth maps...")

        for file in sample_files:
            depth_path = os.path.join(depth_dir, file)
            depth_data = np.load(depth_path)

            if depth_data is None:
                print(f"❌ Failed to load depth: {file}")
                continue

            # Check depth properties
            valid_depths = depth_data[depth_data > 0]
            if len(valid_depths) > 0:
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                mean_depth = np.mean(valid_depths)

                print(f"✅ {file}: range={min_depth:.2f}-{max_depth:.2f}m, mean={mean_depth:.2f}m")
            else:
                print(f"❌ {file}: No valid depth values")

    def validate_semantic_data(self, num_samples=10):
        """Validate semantic segmentation data"""

        semantic_dir = os.path.join(self.data_dir, "semantic")
        if not os.path.exists(semantic_dir):
            print("Semantic directory not found")
            return

        semantic_files = [f for f in os.listdir(semantic_dir) if f.endswith(('.png', '.jpg', '.jpeg'))]
        sample_files = np.random.choice(semantic_files, min(num_samples, len(semantic_files)), replace=False)

        print(f"Validating {len(sample_files)} sample semantic masks...")

        for file in sample_files:
            mask_path = os.path.join(semantic_dir, file)
            mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

            if mask is None:
                print(f"❌ Failed to load semantic mask: {file}")
                continue

            # Check semantic properties
            unique_labels = np.unique(mask)
            num_classes = len(unique_labels)

            print(f"✅ {file}: {num_classes} classes found - {unique_labels}")

    def validate_point_clouds(self, num_samples=10):
        """Validate point cloud data"""

        pc_dir = os.path.join(self.data_dir, "pointclouds")
        if not os.path.exists(pc_dir):
            print("Point cloud directory not found")
            return

        pc_files = [f for f in os.listdir(pc_dir) if f.endswith('.npy')]
        sample_files = np.random.choice(pc_files, min(num_samples, len(pc_files)), replace=False)

        print(f"Validating {len(sample_files)} sample point clouds...")

        for file in sample_files:
            pc_path = os.path.join(pc_dir, file)
            pc_data = np.load(pc_path)

            if pc_data is None:
                print(f"❌ Failed to load point cloud: {file}")
                continue

            # Check point cloud properties
            num_points = pc_data.shape[0] if len(pc_data.shape) > 1 else 0
            if num_points > 0:
                x_range = np.max(pc_data[:, 0]) - np.min(pc_data[:, 0])
                y_range = np.max(pc_data[:, 1]) - np.min(pc_data[:, 1])
                z_range = np.max(pc_data[:, 2]) - np.min(pc_data[:, 2])

                print(f"✅ {file}: {num_points} points, ranges: X={x_range:.2f}, Y={y_range:.2f}, Z={z_range:.2f}")
            else:
                print(f"❌ {file}: No valid points")

    def run_complete_validation(self):
        """Run complete validation of synthetic dataset"""

        print("Starting complete dataset validation...")
        print("=" * 50)

        self.validate_image_quality()
        print()
        self.validate_depth_data()
        print()
        self.validate_semantic_data()
        print()
        self.validate_point_clouds()

        print("=" * 50)
        print("Dataset validation completed!")
```

## Performance Optimization

### Optimizing Data Generation Speed

```python
#!/usr/bin/env python3
"""
Performance optimization for synthetic data generation
"""
class OptimizedDataGenerator:
    """Optimized data generator for faster synthetic data generation"""

    def __init__(self, world):
        self.world = world
        self.optimization_config = self.get_optimization_config()

    def get_optimization_config(self):
        """Get optimization configuration"""

        config = {
            "render_resolution": [640, 480],  # Lower resolution for speed
            "capture_frequency": 10,  # Hz - lower frequency for more processing time
            "parallel_capture": True,  # Capture multiple sensor types in parallel
            "batch_processing": 10,  # Process multiple frames at once
            "memory_management": {
                "preallocate_buffers": True,
                "reuse_objects": True,
                "streaming": True
            },
            "scene_complexity": {
                "max_objects": 20,
                "simplified_geometry": True,
                "level_of_detail": "low"
            }
        }

        return config

    def optimize_rendering(self):
        """Optimize rendering settings for speed"""

        # Reduce rendering quality for faster generation
        # This might involve:
        # - Lowering anti-aliasing
        # - Reducing shadow quality
        # - Simplifying materials
        # - Reducing lighting complexity

        print("Rendering optimized for speed")

    def optimize_physics(self):
        """Optimize physics simulation for data generation"""

        # For data generation, we might not need full physics accuracy
        physics_context = self.world.scene.get_physics_context()

        # Reduce solver iterations if accuracy allows
        # physics_context.set_position_iteration_count(4)  # Lower than normal
        # physics_context.set_velocity_iteration_count(2)  # Lower than normal

        print("Physics optimized for speed")

    def batch_scene_generation(self, num_scenes=5):
        """Generate multiple scene variations in batch"""

        print(f"Generating {num_scenes} scene variations in batch...")

        scenes = []
        for i in range(num_scenes):
            # Create scene variation
            scene_config = self.generate_scene_variation()
            scenes.append(scene_config)

        return scenes

    def generate_scene_variation(self):
        """Generate a single scene variation"""

        # Randomize scene elements
        variation = {
            "lighting": self.randomize_lighting_config(),
            "objects": self.randomize_object_config(),
            "materials": self.randomize_material_config()
        }

        return variation

    def parallel_data_capture(self, cameras):
        """Capture data from multiple cameras in parallel"""

        # In a real implementation, this would use threading or async operations
        # to capture from multiple sensors simultaneously

        captured_data = {}
        for cam_name, camera in cameras.items():
            # Capture data from each camera
            rgb = camera.get_rgb()
            depth = camera.get_depth()

            captured_data[cam_name] = {
                "rgb": rgb,
                "depth": depth
            }

        return captured_data

    def streaming_data_generation(self, total_frames=1000, batch_size=50):
        """Generate data in streaming fashion to manage memory"""

        print(f"Streaming generation: {total_frames} frames in batches of {batch_size}")

        for batch_start in range(0, total_frames, batch_size):
            batch_end = min(batch_start + batch_size, total_frames)

            print(f"Generating frames {batch_start} to {batch_end}...")

            # Generate batch of data
            for i in range(batch_start, batch_end):
                # Capture single frame
                frame_data = self.capture_single_frame()

                # Process and save frame
                self.save_frame(frame_data, i)

            print(f"Completed batch {batch_start // batch_size + 1}")

    def capture_single_frame(self):
        """Capture a single frame of data"""

        # Step the world
        self.world.step(render=True)

        # Capture from all sensors
        frame_data = {
            "timestamp": self.world.current_time_step_index,
            "sensors": {}
        }

        return frame_data

    def save_frame(self, frame_data, frame_id):
        """Save a single frame"""

        # In a real implementation, this would save the frame to disk
        # with proper file naming and organization
        pass

def benchmark_data_generation(generator, num_frames=100):
    """Benchmark data generation performance"""

    import time

    print(f"Benchmarking data generation for {num_frames} frames...")

    start_time = time.time()

    # Generate frames
    for i in range(num_frames):
        frame_data = generator.capture_single_frame()
        generator.save_frame(frame_data, i)

        if (i + 1) % 25 == 0:
            elapsed = time.time() - start_time
            fps = (i + 1) / elapsed
            print(f"  {i + 1}/{num_frames} frames, {fps:.2f} FPS")

    end_time = time.time()
    total_time = end_time - start_time
    avg_fps = num_frames / total_time

    print(f"Generation completed in {total_time:.2f}s, average {avg_fps:.2f} FPS")

if __name__ == "__main__":
    # Example of using the optimized generator
    print("Optimized data generation techniques:")

    # Show configuration options
    opt_gen = OptimizedDataGenerator(None)  # Pass actual world when used
    print(f"Optimization config: {opt_gen.optimization_config}")
```

## Best Practices for Data Generation

### Guidelines for High-Quality Synthetic Data

1. **Realistic Domain Randomization**: Apply domain randomization that reflects real-world variation
2. **Proper Annotation**: Ensure all generated data has accurate ground truth labels
3. **Validation**: Regularly validate synthetic data quality against real data
4. **Diversity**: Generate diverse scenarios to improve model generalization
5. **Consistency**: Maintain consistency between different sensor modalities
6. **Performance**: Optimize for generation speed without sacrificing quality

### Humanoid-Specific Considerations

1. **Human-like Environments**: Generate data in environments humans typically encounter
2. **Social Scenarios**: Include scenarios with human interaction
3. **Dynamic Objects**: Include moving objects that humanoid robots might encounter
4. **Balance Scenarios**: Generate data relevant to humanoid balance and locomotion

## Next Steps

After implementing synthetic data generation:

1. **Train Perception Models**: Use generated data to train computer vision models
2. **Validate Performance**: Test trained models on real-world data
3. **Iterate and Improve**: Refine data generation based on model performance
4. **Scale Generation**: Generate larger datasets for more complex models
5. **Quality Assurance**: Implement automated quality checks for generated data

The next section covers domain randomization techniques in more detail.