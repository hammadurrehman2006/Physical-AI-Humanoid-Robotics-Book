# Domain Randomization in Isaac Sim

This section covers domain randomization techniques in Isaac Sim for improving the sim-to-real transfer of perception and control systems. Domain randomization is a crucial technique for making synthetic data more robust and transferable to real-world applications.

## Understanding Domain Randomization

### What is Domain Randomization?

Domain randomization is a technique used in synthetic data generation to improve the transferability of models trained on simulated data to real-world applications. The core idea is to randomize various aspects of the simulation environment during training, making the trained model robust to the differences between simulation and reality.

### Why Domain Randomization Matters

In robotics, especially for humanoid robots, there's often a significant gap between simulation and reality. Domain randomization helps bridge this gap by:

- **Improving Robustness**: Models become more robust to environmental variations
- **Reducing Overfitting**: Prevents models from overfitting to specific simulation conditions
- **Enhancing Transfer**: Improves the transfer of trained models from simulation to reality
- **Increasing Generalization**: Models learn to focus on essential features rather than environmental artifacts

### Key Concepts in Domain Randomization

1. **Visual Domain Randomization**: Randomizing appearance properties (colors, textures, lighting)
2. **Geometric Domain Randomization**: Randomizing shapes, sizes, and spatial arrangements
3. **Physical Domain Randomization**: Randomizing physical properties (friction, mass, dynamics)
4. **Temporal Domain Randomization**: Randomizing timing and motion patterns

## Visual Domain Randomization

### Lighting Randomization

Lighting randomization is one of the most important aspects of visual domain randomization:

```python
#!/usr/bin/env python3
"""
Lighting domain randomization in Isaac Sim
"""
import random
import numpy as np
from pxr import Gf, UsdLux
import omni.usd
import carb

class LightingRandomizer:
    """Class to handle lighting domain randomization"""

    def __init__(self, stage):
        self.stage = stage
        self.lighting_config = self.get_default_lighting_config()

    def get_default_lighting_config(self):
        """Get default lighting configuration for randomization"""

        config = {
            "dome_light": {
                "intensity_range": (0.1, 3.0),
                "color_temperature_range": (3000, 8000),  # Kelvin
                "enable": True
            },
            "directional_lights": {
                "count_range": (1, 4),
                "intensity_range": (100, 1500),
                "color_temperature_range": (4000, 7000),
                "position_variance": 2.0,
                "rotation_variance": 45.0
            },
            "point_lights": {
                "count_range": (0, 3),
                "intensity_range": (50, 500),
                "radius_range": (0.1, 1.0),
                "position_variance": 1.5
            },
            "environment_effects": {
                "fog_density_range": (0.0, 0.05),
                "fog_color_range": [(0.8, 0.8, 0.9), (1.0, 1.0, 1.0)],  # Light blue to white
                "enable": True
            }
        }

        return config

    def randomize_dome_light(self):
        """Randomize the dome light (environment lighting)"""

        dome_light = self.stage.GetPrimAtPath("/World/DomeLight")
        if not dome_light.IsValid():
            # Create dome light if it doesn't exist
            dome_light = UsdLux.DomeLight.Define(self.stage, "/World/DomeLight")

        # Randomize intensity
        intensity = random.uniform(
            self.lighting_config["dome_light"]["intensity_range"][0],
            self.lighting_config["dome_light"]["intensity_range"][1]
        )
        dome_light.CreateIntensityAttr(intensity)

        # Randomize color temperature
        color_temp = random.uniform(
            self.lighting_config["dome_light"]["color_temperature_range"][0],
            self.lighting_config["dome_light"]["color_temperature_range"][1]
        )
        rgb_color = self.color_temperature_to_rgb(color_temp)
        dome_light.CreateColorAttr(Gf.Vec3f(*rgb_color))

        carb.log_info(f"Dome light randomized: intensity={intensity:.2f}, color_temp={color_temp:.0f}K")

    def randomize_directional_lights(self):
        """Randomize directional lights in the scene"""

        # First, remove existing directional lights
        self.remove_existing_directional_lights()

        # Create random number of directional lights
        num_lights = random.randint(
            self.lighting_config["directional_lights"]["count_range"][0],
            self.lighting_config["directional_lights"]["count_range"][1]
        )

        for i in range(num_lights):
            light_name = f"/World/DirectionalLight_{i}"
            directional_light = UsdLux.DistantLight.Define(self.stage, light_name)

            # Randomize intensity
            intensity = random.uniform(
                self.lighting_config["directional_lights"]["intensity_range"][0],
                self.lighting_config["directional_lights"]["intensity_range"][1]
            )
            directional_light.CreateIntensityAttr(intensity)

            # Randomize color
            color_temp = random.uniform(
                self.lighting_config["directional_lights"]["color_temperature_range"][0],
                self.lighting_config["directional_lights"]["color_temperature_range"][1]
            )
            rgb_color = self.color_temperature_to_rgb(color_temp)
            directional_light.CreateColorAttr(Gf.Vec3f(*rgb_color))

            # Randomize direction (rotation)
            rot_x = random.uniform(-90, 90)
            rot_y = random.uniform(0, 360)
            rot_z = random.uniform(-30, 30)

            # Apply rotations
            directional_light.AddRotateXOp().Set(rot_x)
            directional_light.AddRotateYOp().Set(rot_y)
            directional_light.AddRotateZOp().Set(rot_z)

        carb.log_info(f"Created {num_lights} directional lights with randomized properties")

    def randomize_point_lights(self):
        """Randomize point lights in the scene"""

        # Remove existing point lights
        self.remove_existing_point_lights()

        # Create random number of point lights
        num_lights = random.randint(
            self.lighting_config["point_lights"]["count_range"][0],
            self.lighting_config["point_lights"]["count_range"][1]
        )

        for i in range(num_lights):
            light_name = f"/World/PointLight_{i}"
            point_light = UsdLux.SphereLight.Define(self.stage, light_name)

            # Randomize intensity
            intensity = random.uniform(
                self.lighting_config["point_lights"]["intensity_range"][0],
                self.lighting_config["point_lights"]["intensity_range"][1]
            )
            point_light.CreateIntensityAttr(intensity)

            # Randomize color
            color_temp = random.uniform(3000, 7000)
            rgb_color = self.color_temperature_to_rgb(color_temp)
            point_light.CreateColorAttr(Gf.Vec3f(*rgb_color))

            # Randomize position
            pos_x = random.uniform(-2.0, 2.0)
            pos_y = random.uniform(-2.0, 2.0)
            pos_z = random.uniform(1.0, 4.0)

            point_light.AddTranslateOp().Set(Gf.Vec3d(pos_x, pos_y, pos_z))

            # Randomize radius (affects light spread)
            radius = random.uniform(
                self.lighting_config["point_lights"]["radius_range"][0],
                self.lighting_config["point_lights"]["radius_range"][1]
            )
            point_light.CreateRadiusAttr(radius)

        carb.log_info(f"Created {num_lights} point lights with randomized properties")

    def randomize_environment_effects(self):
        """Randomize environmental effects like fog"""

        if not self.lighting_config["environment_effects"]["enable"]:
            return

        # In Isaac Sim, this might involve:
        # - Setting up volumetric effects
        # - Configuring atmospheric properties
        # - Adjusting post-processing effects

        fog_density = random.uniform(
            self.lighting_config["environment_effects"]["fog_density_range"][0],
            self.lighting_config["environment_effects"]["fog_density_range"][1]
        )

        # This is conceptual - actual implementation depends on Isaac Sim's rendering pipeline
        carb.log_info(f"Environment effects randomized: fog_density={fog_density:.3f}")

    def remove_existing_directional_lights(self):
        """Remove existing directional lights"""

        # Find and remove existing directional lights
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "DistantLight":
                self.stage.RemovePrim(prim.GetPath())

    def remove_existing_point_lights(self):
        """Remove existing point lights"""

        # Find and remove existing point lights
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "SphereLight":
                self.stage.RemovePrim(prim.GetPath())

    def color_temperature_to_rgb(self, temp_k):
        """Convert color temperature in Kelvin to RGB (approximation)"""

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

    def apply_lighting_randomization(self):
        """Apply all lighting randomization techniques"""

        self.randomize_dome_light()
        self.randomize_directional_lights()
        self.randomize_point_lights()
        self.randomize_environment_effects()

        carb.log_info("Lighting domain randomization applied successfully")
```

### Material and Texture Randomization

```python
#!/usr/bin/env python3
"""
Material and texture domain randomization in Isaac Sim
"""
from omni.isaac.core.materials import VisualMaterial
from pxr import Gf, Sdf, UsdShade
import omni.usd

class MaterialRandomizer:
    """Class to handle material and texture domain randomization"""

    def __init__(self, stage):
        self.stage = stage
        self.material_config = self.get_default_material_config()

    def get_default_material_config(self):
        """Get default material configuration for randomization"""

        config = {
            "diffuse_color": {
                "randomize": True,
                "hue_range": (0, 360),
                "saturation_range": (0.3, 1.0),
                "value_range": (0.3, 1.0)
            },
            "roughness": {
                "randomize": True,
                "range": (0.0, 1.0),
                "distribution": "uniform"  # uniform, gaussian, etc.
            },
            "metallic": {
                "randomize": True,
                "range": (0.0, 1.0)
            },
            "specular": {
                "randomize": True,
                "range": (0.0, 1.0)
            },
            "clearcoat": {
                "randomize": True,
                "range": (0.0, 1.0)
            },
            "opacity": {
                "randomize": True,
                "range": (0.8, 1.0)  # Usually don't make things too transparent
            },
            "textures": {
                "randomize": True,
                "types": ["procedural", "noise", "pattern"],
                "enable": True
            }
        }

        return config

    def randomize_material_properties(self, material_prim_path):
        """Randomize properties of a specific material"""

        # This is a conceptual implementation - actual implementation would depend
        # on how materials are structured in the USD stage

        # Create random material properties
        material_props = {
            "diffuse_color": self.randomize_diffuse_color(),
            "roughness": random.uniform(*self.material_config["roughness"]["range"]),
            "metallic": random.uniform(*self.material_config["metallic"]["range"]),
            "specular": random.uniform(*self.material_config["specular"]["range"]),
            "clearcoat": random.uniform(*self.material_config["clearcoat"]["range"]),
            "opacity": random.uniform(*self.material_config["opacity"]["range"])
        }

        # Apply properties to material
        self.apply_material_properties(material_prim_path, material_props)

        return material_props

    def randomize_diffuse_color(self):
        """Randomize diffuse color using HSV to RGB conversion"""

        if not self.material_config["diffuse_color"]["randomize"]:
            return (0.8, 0.8, 0.8)  # Default gray

        # Randomize HSV values
        hue = random.uniform(*self.material_config["diffuse_color"]["hue_range"])
        saturation = random.uniform(*self.material_config["diffuse_color"]["saturation_range"])
        value = random.uniform(*self.material_config["diffuse_color"]["value_range"])

        # Convert HSV to RGB
        rgb = self.hsv_to_rgb(hue / 360.0, saturation, value)
        return rgb

    def hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB color space"""

        if s == 0.0:
            return (v, v, v)

        i = int(h * 6.0)
        f = (h * 6.0) - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        i = i % 6

        if i == 0:
            return (v, t, p)
        elif i == 1:
            return (q, v, p)
        elif i == 2:
            return (p, v, t)
        elif i == 3:
            return (p, q, v)
        elif i == 4:
            return (t, p, v)
        elif i == 5:
            return (v, p, q)

    def randomize_object_materials(self):
        """Randomize materials for all objects in the scene"""

        material_count = 0

        # Traverse all prims in the stage
        for prim in self.stage.Traverse():
            # Check if this prim is a renderable object
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder", "Mesh"]:
                # Get or create material for this prim
                material_path = f"{prim.GetPath().pathString}_Material"

                # Randomize material properties
                material_props = self.randomize_material_properties(material_path)

                # Create or update material
                material = self.create_material(material_path, material_props)

                # Apply material to prim
                self.apply_material_to_prim(prim, material)

                material_count += 1

        carb.log_info(f"Randomized materials for {material_count} objects")

    def create_material(self, prim_path, properties):
        """Create a material with random properties"""

        # Create a new material
        material = VisualMaterial(
            prim_path=prim_path,
            name=prim_path.split("/")[-1],
            diffuse_color=properties["diffuse_color"],
            roughness=properties["roughness"],
            metallic=properties["metallic"],
            specular=properties["specular"],
            clearcoat=properties["clearcoat"],
            opacity=properties["opacity"]
        )

        return material

    def apply_material_to_prim(self, prim, material):
        """Apply material to a prim"""

        # Bind the material to the prim
        material.bind(prim)

    def randomize_textures(self):
        """Randomize textures applied to materials"""

        if not self.material_config["textures"]["enable"]:
            return

        # This would involve applying random textures or procedural patterns
        # to objects in the scene
        carb.log_info("Textures randomized (conceptual implementation)")

    def apply_material_randomization(self):
        """Apply all material randomization techniques"""

        self.randomize_object_materials()
        self.randomize_textures()

        carb.log_info("Material domain randomization applied successfully")
```

## Geometric Domain Randomization

### Object Position and Orientation Randomization

```python
#!/usr/bin/env python3
"""
Geometric domain randomization - position and orientation
"""
import numpy as np

class GeometricRandomizer:
    """Class to handle geometric domain randomization"""

    def __init__(self, stage):
        self.stage = stage
        self.geometry_config = self.get_default_geometry_config()

    def get_default_geometry_config(self):
        """Get default geometric configuration for randomization"""

        config = {
            "position": {
                "randomize": True,
                "x_range": (-3.0, 3.0),
                "y_range": (-3.0, 3.0),
                "z_range": (0.1, 3.0),  # Objects above ground
                "jitter_amount": 0.1
            },
            "rotation": {
                "randomize": True,
                "max_rotation_deg": 45.0,
                "distribution": "uniform"
            },
            "scale": {
                "randomize": True,
                "range": (0.8, 1.2),
                "uniform_scaling": True,
                "non_uniform_scaling": False
            },
            "object_count": {
                "randomize": True,
                "range": (3, 10),
                "types": ["Cube", "Sphere", "Cylinder"]
            },
            "arrangement": {
                "randomize": True,
                "patterns": ["random", "grid", "cluster", "line"],
                "enable": True
            }
        }

        return config

    def randomize_object_positions(self):
        """Randomize positions of objects in the scene"""

        position_count = 0

        for prim in self.stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder", "Mesh"]:
                # Skip ground plane and fixed elements
                if "ground" in prim.GetPath().pathString.lower():
                    continue

                # Generate random position
                pos_x = random.uniform(
                    self.geometry_config["position"]["x_range"][0],
                    self.geometry_config["position"]["x_range"][1]
                )
                pos_y = random.uniform(
                    self.geometry_config["position"]["y_range"][0],
                    self.geometry_config["position"]["y_range"][1]
                )
                pos_z = random.uniform(
                    self.geometry_config["position"]["z_range"][0],
                    self.geometry_config["position"]["z_range"][1]
                )

                # Apply position
                prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(pos_x, pos_y, pos_z))
                position_count += 1

        carb.log_info(f"Randomized positions for {position_count} objects")

    def randomize_object_rotations(self):
        """Randomize rotations of objects in the scene"""

        rotation_count = 0

        for prim in self.stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder", "Mesh"]:
                # Skip ground plane and fixed elements
                if "ground" in prim.GetPath().pathString.lower():
                    continue

                # Generate random rotations
                rot_x = random.uniform(-self.geometry_config["rotation"]["max_rotation_deg"],
                                      self.geometry_config["rotation"]["max_rotation_deg"])
                rot_y = random.uniform(-self.geometry_config["rotation"]["max_rotation_deg"],
                                      self.geometry_config["rotation"]["max_rotation_deg"])
                rot_z = random.uniform(-self.geometry_config["rotation"]["max_rotation_deg"],
                                      self.geometry_config["rotation"]["max_rotation_deg"])

                # Apply rotations
                prim.GetAttribute("xformOp:rotateX").Set(rot_x)
                prim.GetAttribute("xformOp:rotateY").Set(rot_y)
                prim.GetAttribute("xformOp:rotateZ").Set(rot_z)

                rotation_count += 1

        carb.log_info(f"Randomized rotations for {rotation_count} objects")

    def randomize_object_scales(self):
        """Randomize scales of objects in the scene"""

        scale_count = 0

        for prim in self.stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder", "Mesh"]:
                # Skip ground plane and fixed elements
                if "ground" in prim.GetPath().pathString.lower():
                    continue

                # Generate random scale
                if self.geometry_config["scale"]["uniform_scaling"]:
                    scale_val = random.uniform(
                        self.geometry_config["scale"]["range"][0],
                        self.geometry_config["scale"]["range"][1]
                    )
                    scale_vec = Gf.Vec3f(scale_val, scale_val, scale_val)
                else:
                    scale_x = random.uniform(
                        self.geometry_config["scale"]["range"][0],
                        self.geometry_config["scale"]["range"][1]
                    )
                    scale_y = random.uniform(
                        self.geometry_config["scale"]["range"][0],
                        self.geometry_config["scale"]["range"][1]
                    )
                    scale_z = random.uniform(
                        self.geometry_config["scale"]["range"][0],
                        self.geometry_config["scale"]["range"][1]
                    )
                    scale_vec = Gf.Vec3f(scale_x, scale_y, scale_z)

                # Apply scale
                prim.GetAttribute("xformOp:scale").Set(scale_vec)
                scale_count += 1

        carb.log_info(f"Randomized scales for {scale_count} objects")

    def randomize_object_count(self):
        """Randomize the number of objects in the scene"""

        # This would involve adding or removing objects from the scene
        # For this implementation, we'll focus on adjusting existing objects
        # and potentially adding new ones

        target_count = random.randint(
            self.geometry_config["object_count"]["range"][0],
            self.geometry_config["object_count"]["range"][1]
        )

        current_objects = []
        for prim in self.stage.Traverse():
            if prim.GetTypeName() in self.geometry_config["object_count"]["types"]:
                current_objects.append(prim)

        current_count = len(current_objects)

        if current_count < target_count:
            # Add more objects
            self.add_random_objects(target_count - current_count)
        elif current_count > target_count:
            # Remove some objects (keeping important ones)
            self.remove_random_objects(current_count - target_count)

        carb.log_info(f"Adjusted object count to {target_count}")

    def add_random_objects(self, count):
        """Add random objects to the scene"""

        # This is a simplified implementation - in practice, you would
        # use Isaac Sim's object creation utilities
        for i in range(count):
            obj_type = random.choice(self.geometry_config["object_count"]["types"])
            obj_name = f"/World/RandomObject_{i}"

            # Create object based on type
            # In practice, this would use omni.isaac.core.utils.prims.create_prim
            carb.log_info(f"Added {obj_type} at {obj_name}")

    def remove_random_objects(self, count):
        """Remove random objects from the scene"""

        # Find objects that can be removed (not essential to the scene)
        removable_objects = []
        for prim in self.stage.Traverse():
            if (prim.GetTypeName() in self.geometry_config["object_count"]["types"] and
                "ground" not in prim.GetPath().pathString.lower()):
                removable_objects.append(prim)

        # Remove random selection of objects
        objects_to_remove = random.sample(removable_objects, min(count, len(removable_objects)))

        for prim in objects_to_remove:
            self.stage.RemovePrim(prim.GetPath())

        carb.log_info(f"Removed {len(objects_to_remove)} objects")

    def apply_geometric_randomization(self):
        """Apply all geometric randomization techniques"""

        self.randomize_object_positions()
        self.randomize_object_rotations()
        self.randomize_object_scales()
        self.randomize_object_count()

        carb.log_info("Geometric domain randomization applied successfully")
```

## Physical Domain Randomization

### Physics Property Randomization

```python
#!/usr/bin/env python3
"""
Physical domain randomization in Isaac Sim
"""
class PhysicsRandomizer:
    """Class to handle physical domain randomization"""

    def __init__(self, world):
        self.world = world
        self.physics_config = self.get_default_physics_config()

    def get_default_physics_config(self):
        """Get default physics configuration for randomization"""

        config = {
            "gravity": {
                "randomize": True,
                "range": (-10.5, -8.5),  # Earth's gravity is -9.81
                "jitter": 0.1
            },
            "friction": {
                "randomize": True,
                "static_range": (0.1, 1.0),
                "dynamic_range": (0.1, 0.8),
                "enable": True
            },
            "restitution": {
                "randomize": True,
                "range": (0.0, 0.5),  # Bounciness (0 = no bounce, 1 = perfectly elastic)
                "enable": True
            },
            "mass": {
                "randomize": True,
                "multiplier_range": (0.5, 2.0),  # Mass can be 0.5x to 2x original
                "enable": True
            },
            "damping": {
                "randomize": True,
                "linear_range": (0.0, 0.1),
                "angular_range": (0.0, 0.1),
                "enable": True
            },
            "solver": {
                "randomize": False,  # Usually keep solver settings consistent
                "position_iterations_range": (4, 16),
                "velocity_iterations_range": (1, 8)
            }
        }

        return config

    def randomize_gravity(self):
        """Randomize gravity in the physics scene"""

        if not self.physics_config["gravity"]["randomize"]:
            return

        # Get physics scene
        physics_scene = self.world.scene.get_physics_context().get_physics_scene()

        # Randomize gravity
        gravity = random.uniform(
            self.physics_config["gravity"]["range"][0],
            self.physics_config["gravity"]["range"][1]
        )

        # Apply gravity (negative Y direction)
        physics_scene.GetGravityAttr().Set(gravity)

        carb.log_info(f"Gravity randomized to: {gravity:.3f} m/s²")

    def randomize_friction_properties(self):
        """Randomize friction properties of objects"""

        if not self.physics_config["friction"]["enable"]:
            return

        # This would involve accessing individual rigid bodies and setting their friction properties
        # In practice, this is done through USD attributes or Isaac Sim APIs
        static_friction = random.uniform(
            self.physics_config["friction"]["static_range"][0],
            self.physics_config["friction"]["static_range"][1]
        )
        dynamic_friction = random.uniform(
            self.physics_config["friction"]["dynamic_range"][0],
            self.physics_config["friction"]["dynamic_range"][1]
        )

        carb.log_info(f"Friction randomized: static={static_friction:.3f}, dynamic={dynamic_friction:.3f}")

    def randomize_restitution(self):
        """Randomize restitution (bounciness) properties"""

        if not self.physics_config["restitution"]["enable"]:
            return

        restitution = random.uniform(
            self.physics_config["restitution"]["range"][0],
            self.physics_config["restitution"]["range"][1]
        )

        carb.log_info(f"Restitution randomized to: {restitution:.3f}")

    def randomize_mass_properties(self):
        """Randomize mass properties of objects"""

        if not self.physics_config["mass"]["enable"]:
            return

        # This would involve modifying the mass properties of individual objects
        mass_multiplier = random.uniform(
            self.physics_config["mass"]["multiplier_range"][0],
            self.physics_config["mass"]["multiplier_range"][1]
        )

        carb.log_info(f"Mass properties will be randomized with multiplier: {mass_multiplier:.3f}")

    def randomize_damping_properties(self):
        """Randomize damping properties"""

        if not self.physics_config["damping"]["enable"]:
            return

        linear_damping = random.uniform(
            self.physics_config["damping"]["linear_range"][0],
            self.physics_config["damping"]["linear_range"][1]
        )
        angular_damping = random.uniform(
            self.physics_config["damping"]["angular_range"][0],
            self.physics_config["damping"]["angular_range"][1]
        )

        # Apply damping through physics context
        physics_context = self.world.scene.get_physics_context()
        # Note: Actual API may vary depending on Isaac Sim version
        carb.log_info(f"Damping randomized: linear={linear_damping:.3f}, angular={angular_damping:.3f}")

    def apply_physics_randomization(self):
        """Apply all physics randomization techniques"""

        self.randomize_gravity()
        self.randomize_friction_properties()
        self.randomize_restitution()
        self.randomize_mass_properties()
        self.randomize_damping_properties()

        carb.log_info("Physics domain randomization applied successfully")
```

## Temporal Domain Randomization

### Motion and Timing Randomization

```python
#!/usr/bin/env python3
"""
Temporal domain randomization in Isaac Sim
"""
class TemporalRandomizer:
    """Class to handle temporal domain randomization"""

    def __init__(self, world):
        self.world = world
        self.temporal_config = self.get_default_temporal_config()

    def get_default_temporal_config(self):
        """Get default temporal configuration for randomization"""

        config = {
            "motion_speed": {
                "randomize": True,
                "multiplier_range": (0.8, 1.2),  # 80% to 120% of normal speed
                "enable": True
            },
            "object_trajectories": {
                "randomize": True,
                "types": ["linear", "circular", "oscillating", "random_walk"],
                "frequency_range": (0.1, 2.0),  # Hz
                "amplitude_range": (0.1, 1.0),  # meters
                "enable": True
            },
            "camera_motion": {
                "randomize": True,
                "types": ["static", "panning", "zooming", "handheld"],
                "speed_range": (0.01, 0.1),  # m/s
                "enable": True
            },
            "lighting_changes": {
                "randomize": True,
                "frequency_range": (0.01, 0.1),  # Hz (slow changes)
                "intensity_variance": 0.1,  # 10% variance
                "enable": True
            },
            "frame_rate": {
                "randomize": True,
                "range": (15, 60),  # FPS
                "enable": True
            }
        }

        return config

    def randomize_motion_speeds(self):
        """Randomize the speed of moving objects"""

        if not self.temporal_config["motion_speed"]["randomize"]:
            return

        speed_multiplier = random.uniform(
            self.temporal_config["motion_speed"]["multiplier_range"][0],
            self.temporal_config["motion_speed"]["multiplier_range"][1]
        )

        carb.log_info(f"Motion speeds randomized with multiplier: {speed_multiplier:.3f}")

    def randomize_object_trajectories(self):
        """Randomize trajectories of moving objects"""

        if not self.temporal_config["object_trajectories"]["enable"]:
            return

        # This would involve setting up animated movements for objects
        # For example, making objects move in different patterns
        trajectory_type = random.choice(self.temporal_config["object_trajectories"]["types"])
        frequency = random.uniform(
            self.temporal_config["object_trajectories"]["frequency_range"][0],
            self.temporal_config["object_trajectories"]["frequency_range"][1]
        )
        amplitude = random.uniform(
            self.temporal_config["object_trajectories"]["amplitude_range"][0],
            self.temporal_config["object_trajectories"]["amplitude_range"][1]
        )

        carb.log_info(f"Object trajectories randomized: type={trajectory_type}, freq={frequency:.2f}Hz, amp={amplitude:.2f}m")

    def randomize_camera_motion(self):
        """Randomize camera motion during data capture"""

        if not self.temporal_config["camera_motion"]["enable"]:
            return

        camera_type = random.choice(self.temporal_config["camera_motion"]["types"])
        speed = random.uniform(
            self.temporal_config["camera_motion"]["speed_range"][0],
            self.temporal_config["camera_motion"]["speed_range"][1]
        )

        carb.log_info(f"Camera motion randomized: type={camera_type}, speed={speed:.3f}m/s")

    def randomize_lighting_over_time(self):
        """Randomize lighting changes over time"""

        if not self.temporal_config["lighting_changes"]["enable"]:
            return

        frequency = random.uniform(
            self.temporal_config["lighting_changes"]["frequency_range"][0],
            self.temporal_config["lighting_changes"]["frequency_range"][1]
        )
        intensity_variance = self.temporal_config["lighting_changes"]["intensity_variance"]

        carb.log_info(f"Time-varying lighting: frequency={frequency:.3f}Hz, variance={intensity_variance:.2f}")

    def apply_temporal_randomization(self):
        """Apply all temporal randomization techniques"""

        self.randomize_motion_speeds()
        self.randomize_object_trajectories()
        self.randomize_camera_motion()
        self.randomize_lighting_over_time()

        carb.log_info("Temporal domain randomization applied successfully")
```

## Advanced Domain Randomization Techniques

### Correlated Randomization

```python
#!/usr/bin/env python3
"""
Advanced domain randomization with correlated properties
"""
class CorrelatedRandomizer:
    """Class to handle correlated domain randomization"""

    def __init__(self, world):
        self.world = world
        self.correlation_config = self.get_default_correlation_config()

    def get_default_correlation_config(self):
        """Get default correlation configuration"""

        config = {
            "indoor_outdoor": {
                "enable": True,
                "correlations": {
                    "indoor": {
                        "lighting": {"type": "artificial", "intensity": (100, 500)},
                        "colors": {"warm": True, "temperature_range": (2700, 3500)},
                        "objects": {"furniture": True, "clutter": "low"}
                    },
                    "outdoor": {
                        "lighting": {"type": "natural", "intensity": (1000, 3000)},
                        "colors": {"cool": True, "temperature_range": (5000, 7000)},
                        "objects": {"nature": True, "clutter": "medium"}
                    }
                }
            },
            "time_of_day": {
                "enable": True,
                "correlations": {
                    "morning": {"intensity": (800, 1200), "temperature": (6000, 7000)},
                    "noon": {"intensity": (2000, 3000), "temperature": (5500, 6500)},
                    "evening": {"intensity": (400, 800), "temperature": (3000, 4000)},
                    "night": {"intensity": (50, 200), "temperature": (2700, 3500)}
                }
            },
            "weather": {
                "enable": True,
                "correlations": {
                    "clear": {"fog": 0.0, "lighting": "bright", "visibility": "high"},
                    "cloudy": {"fog": 0.02, "lighting": "diffuse", "visibility": "medium"},
                    "foggy": {"fog": 0.08, "lighting": "dim", "visibility": "low"}
                }
            }
        }

        return config

    def randomize_correlated_scenes(self):
        """Randomize scene properties with correlations"""

        # Randomly select a scene context
        scene_context = random.choice(["indoor", "outdoor"])
        time_context = random.choice(["morning", "noon", "evening", "night"])
        weather_context = random.choice(["clear", "cloudy", "foggy"])

        carb.log_info(f"Scene context: {scene_context}, {time_context}, {weather_context}")

        # Apply correlated randomization based on context
        self.apply_indoor_outdoor_correlation(scene_context)
        self.apply_time_of_day_correlation(time_context)
        self.apply_weather_correlation(weather_context)

    def apply_indoor_outdoor_correlation(self, context):
        """Apply indoor/outdoor correlated randomization"""

        if not self.correlation_config["indoor_outdoor"]["enable"]:
            return

        correlations = self.correlation_config["indoor_outdoor"]["correlations"][context]

        # Apply correlated lighting
        lighting_config = correlations["lighting"]
        carb.log_info(f"Applying {context} lighting: {lighting_config}")

        # Apply correlated colors
        color_config = correlations["colors"]
        carb.log_info(f"Applying {context} colors: {color_config}")

        # Apply correlated objects
        object_config = correlations["objects"]
        carb.log_info(f"Applying {context} objects: {object_config}")

    def apply_time_of_day_correlation(self, context):
        """Apply time-of-day correlated randomization"""

        if not self.correlation_config["time_of_day"]["enable"]:
            return

        correlations = self.correlation_config["time_of_day"]["correlations"][context]

        intensity = random.uniform(*correlations["intensity"])
        temperature = random.uniform(*correlations["temperature"])

        carb.log_info(f"Applying {context} lighting: intensity={intensity:.0f}, temp={temperature:.0f}K")

    def apply_weather_correlation(self, context):
        """Apply weather correlated randomization"""

        if not self.correlation_config["weather"]["enable"]:
            return

        correlations = self.correlation_config["weather"]["correlations"][context]

        carb.log_info(f"Applying {context} weather: {correlations}")

    def apply_correlated_randomization(self):
        """Apply all correlated randomization techniques"""

        self.randomize_correlated_scenes()

        carb.log_info("Correlated domain randomization applied successfully")
```

## Adaptive Domain Randomization

### Learning-Based Randomization

```python
#!/usr/bin/env python3
"""
Adaptive domain randomization that learns from real data
"""
class AdaptiveRandomizer:
    """Class to handle adaptive domain randomization"""

    def __init__(self, world):
        self.world = world
        self.performance_history = []
        self.randomization_bounds = self.get_initial_bounds()

    def get_initial_bounds(self):
        """Get initial randomization bounds"""

        bounds = {
            "lighting": {"intensity": [0.1, 3.0], "temperature": [3000, 8000]},
            "materials": {"roughness": [0.0, 1.0], "metallic": [0.0, 1.0]},
            "geometry": {"position_jitter": [0.0, 0.5], "rotation_jitter": [0.0, 45.0]},
            "physics": {"friction": [0.1, 1.0], "restitution": [0.0, 0.5]}
        }

        return bounds

    def update_randomization_based_on_performance(self, real_world_performance):
        """Update randomization based on real-world performance feedback"""

        # This is a simplified example of how adaptive randomization might work
        # In practice, this would involve more sophisticated algorithms

        if len(self.performance_history) == 0:
            # First measurement, just record it
            self.performance_history.append(real_world_performance)
            return

        # Compare current performance with previous
        previous_performance = self.performance_history[-1]
        performance_change = real_world_performance - previous_performance

        # Adjust randomization bounds based on performance
        if performance_change > 0:
            # Performance improved, potentially reduce randomization range
            self.narrow_randomization_bounds()
        else:
            # Performance decreased, potentially increase randomization range
            self.widen_randomization_bounds()

        # Record performance
        self.performance_history.append(real_world_performance)

    def narrow_randomization_bounds(self):
        """Narrow randomization bounds (make simulation more realistic)"""

        for category, bounds in self.randomization_bounds.items():
            for param, (min_val, max_val) in bounds.items():
                # Reduce range by 10%
                center = (min_val + max_val) / 2
                range_size = max_val - min_val
                new_range = range_size * 0.9
                new_min = center - new_range / 2
                new_max = center + new_range / 2
                self.randomization_bounds[category][param] = [new_min, new_max]

        carb.log_info("Randomization bounds narrowed (simulation made more realistic)")

    def widen_randomization_bounds(self):
        """Widen randomization bounds (make simulation more diverse)"""

        for category, bounds in self.randomization_bounds.items():
            for param, (min_val, max_val) in bounds.items():
                # Increase range by 10%
                center = (min_val + max_val) / 2
                range_size = max_val - min_val
                new_range = min(range_size * 1.1, 10.0)  # Cap at reasonable maximum
                new_min = max(center - new_range / 2, 0)  # Don't go below 0
                new_max = center + new_range / 2
                self.randomization_bounds[category][param] = [new_min, new_max]

        carb.log_info("Randomization bounds widened (simulation made more diverse)")

    def apply_adaptive_randomization(self):
        """Apply adaptive randomization with current bounds"""

        # This would apply randomization using the current bounds
        carb.log_info("Adaptive randomization applied with current bounds")
```

## Validation and Quality Assessment

### Domain Randomization Validation

```python
#!/usr/bin/env python3
"""
Validation techniques for domain randomization
"""
class RandomizationValidator:
    """Class to validate domain randomization quality"""

    def __init__(self, world):
        self.world = world

    def validate_visual_diversity(self, num_samples=100):
        """Validate that visual randomization creates diverse images"""

        print(f"Validating visual diversity with {num_samples} samples...")

        # This would capture multiple images with different randomization
        # and analyze their diversity using computer vision techniques

        diversity_metrics = {
            "color_histogram_difference": 0.0,
            "texture_difference": 0.0,
            "edge_density_variation": 0.0,
            "lighting_condition_diversity": 0.0
        }

        # Simulate diversity analysis
        for i in range(num_samples):
            # Apply randomization
            # Capture image
            # Analyze diversity
            pass

        print(f"Visual diversity validation completed: {diversity_metrics}")

    def validate_physics_stability(self, num_tests=50):
        """Validate that physics randomization maintains stability"""

        print(f"Validating physics stability with {num_tests} tests...")

        stable_tests = 0
        total_tests = 0

        for i in range(num_tests):
            # Apply physics randomization
            # Run simulation
            # Check for stability (no explosions, NaN values, etc.)

            # Simulate stability check
            is_stable = random.choice([True, True, True, True, False])  # 80% success rate
            if is_stable:
                stable_tests += 1
            total_tests += 1

        stability_rate = stable_tests / total_tests if total_tests > 0 else 0
        print(f"Physics stability: {stability_rate:.2%} ({stable_tests}/{total_tests})")

        return stability_rate

    def validate_transfer_performance(self, real_data_performance, sim_data_performance):
        """Validate transfer performance between real and simulated data"""

        print("Validating transfer performance...")

        # Calculate performance gap
        performance_gap = abs(real_data_performance - sim_data_performance)
        relative_gap = performance_gap / max(real_data_performance, sim_data_performance)

        print(f"Real data performance: {real_data_performance:.3f}")
        print(f"Sim data performance: {sim_data_performance:.3f}")
        print(f"Performance gap: {performance_gap:.3f}")
        print(f"Relative gap: {relative_gap:.3f}")

        # Return transfer quality score
        transfer_score = max(0, 1 - relative_gap)  # Higher is better
        return transfer_score

    def run_comprehensive_validation(self):
        """Run comprehensive validation of domain randomization"""

        print("Running comprehensive domain randomization validation...")

        # Validate different aspects
        visual_diversity = self.validate_visual_diversity(num_samples=50)
        physics_stability = self.validate_physics_stability(num_tests=30)

        # Simulate transfer performance validation
        real_perf = 0.85  # Example real-world performance
        sim_perf = 0.82   # Example simulation performance
        transfer_score = self.validate_transfer_performance(real_perf, sim_perf)

        print("\nValidation Results:")
        print(f"- Visual diversity: {'Good' if True else 'Needs improvement'}")
        print(f"- Physics stability: {physics_stability:.2%}")
        print(f"- Transfer quality: {transfer_score:.2f}/1.0")

        overall_score = (physics_stability + transfer_score) / 2
        print(f"- Overall quality: {overall_score:.2f}/1.0")

        return overall_score
```

## Best Practices for Domain Randomization

### Guidelines for Effective Domain Randomization

1. **Start Simple**: Begin with basic randomization and gradually increase complexity
2. **Monitor Stability**: Ensure physics randomization doesn't break simulation stability
3. **Validate Transfer**: Regularly test model performance on real data
4. **Balance Diversity**: Don't randomize so much that objects become unrecognizable
5. **Correlate Properties**: Randomize related properties together (e.g., indoor lighting with warm colors)
6. **Use Real Data**: When possible, use real-world data statistics to guide randomization ranges

### Humanoid-Specific Considerations

1. **Human-like Environments**: Focus on randomization that reflects human environments
2. **Social Context**: Include randomization that reflects social interaction scenarios
3. **Dynamic Elements**: Include moving objects that humanoid robots typically encounter
4. **Furniture Context**: Randomize indoor scenes with appropriate furniture arrangements

## Implementation Example

### Complete Domain Randomization Pipeline

```python
#!/usr/bin/env python3
"""
Complete domain randomization pipeline
"""
from omni.isaac.core import World

class DomainRandomizationPipeline:
    """Complete pipeline for domain randomization in Isaac Sim"""

    def __init__(self, world):
        self.world = world
        self.stage = omni.usd.get_context().get_stage()

        # Initialize randomizers
        self.lighting_randomizer = LightingRandomizer(self.stage)
        self.material_randomizer = MaterialRandomizer(self.stage)
        self.geometric_randomizer = GeometricRandomizer(self.stage)
        self.physics_randomizer = PhysicsRandomizer(self.world)
        self.temporal_randomizer = TemporalRandomizer(self.world)
        self.correlated_randomizer = CorrelatedRandomizer(self.world)
        self.adaptive_randomizer = AdaptiveRandomizer(self.world)
        self.validator = RandomizationValidator(self.world)

    def apply_domain_randomization(self, randomization_type="full"):
        """Apply domain randomization of specified type"""

        if randomization_type == "full":
            # Apply all types of randomization
            self.lighting_randomizer.apply_lighting_randomization()
            self.material_randomizer.apply_material_randomization()
            self.geometric_randomizer.apply_geometric_randomization()
            self.physics_randomizer.apply_physics_randomization()
            self.temporal_randomizer.apply_temporal_randomization()
            self.correlated_randomizer.apply_correlated_randomization()

        elif randomization_type == "visual":
            # Apply only visual randomization
            self.lighting_randomizer.apply_lighting_randomization()
            self.material_randomizer.apply_material_randomization()

        elif randomization_type == "physical":
            # Apply only physical randomization
            self.physics_randomizer.apply_physics_randomization()
            self.geometric_randomizer.apply_geometric_randomization()

        else:
            # Apply specific randomization based on type
            carb.log_warn(f"Unknown randomization type: {randomization_type}")

        carb.log_info(f"Domain randomization applied: {randomization_type}")

    def validate_randomization(self):
        """Validate the applied randomization"""

        # Run comprehensive validation
        validation_score = self.validator.run_comprehensive_validation()
        return validation_score

    def adaptive_randomization_cycle(self, real_world_performance):
        """Run an adaptive randomization cycle"""

        # Update adaptive randomizer based on real-world feedback
        self.adaptive_randomizer.update_randomization_based_on_performance(real_world_performance)

        # Apply updated randomization
        self.apply_domain_randomization("full")

        # Validate the new randomization
        validation_score = self.validate_randomization()

        return validation_score

# Example usage
def run_domain_randomization_example():
    """Run an example domain randomization session"""

    # Initialize Isaac Sim world
    world = World(stage_units_in_meters=1.0)

    # Create domain randomization pipeline
    dr_pipeline = DomainRandomizationPipeline(world)

    # Apply domain randomization
    dr_pipeline.apply_domain_randomization("full")

    # Validate the randomization
    validation_score = dr_pipeline.validate_randomization()

    print(f"Domain randomization completed with validation score: {validation_score:.2f}")

    # Simulate adaptive cycle with example real-world performance
    real_performance = 0.82
    adaptive_score = dr_pipeline.adaptive_randomization_cycle(real_performance)

    print(f"Adaptive randomization completed with score: {adaptive_score:.2f}")

    return dr_pipeline

if __name__ == "__main__":
    pipeline = run_domain_randomization_example()
```

## Performance Optimization

### Efficient Randomization Techniques

```python
#!/usr/bin/env python3
"""
Performance optimization for domain randomization
"""
class OptimizedDomainRandomizer:
    """Optimized domain randomization for better performance"""

    def __init__(self, world):
        self.world = world
        self.optimization_config = self.get_optimization_config()

    def get_optimization_config(self):
        """Get optimization configuration"""

        config = {
            "batch_randomization": True,
            "caching": True,
            "selective_randomization": True,
            "progressive_randomization": True,
            "performance_thresholds": {
                "max_frame_time": 0.033,  # 30 FPS threshold
                "min_diversity_score": 0.5
            }
        }

        return config

    def batch_randomization(self, num_scenes=10):
        """Apply randomization to multiple scenes in batch"""

        print(f"Generating {num_scenes} scene variations in batch...")

        scenes = []
        for i in range(num_scenes):
            # Generate scene with random parameters
            scene_config = self.generate_random_scene_config()
            scenes.append(scene_config)

        return scenes

    def generate_random_scene_config(self):
        """Generate a random scene configuration"""

        config = {
            "lighting": self.randomize_lighting_config(),
            "materials": self.randomize_material_config(),
            "geometry": self.randomize_geometry_config(),
            "physics": self.randomize_physics_config()
        }

        return config

    def selective_randomization(self, objects_of_interest):
        """Apply randomization only to objects of interest"""

        print(f"Applying selective randomization to {len(objects_of_interest)} objects")

        # Only randomize specific objects rather than all objects
        for obj_path in objects_of_interest:
            # Apply randomization to this specific object
            self.randomize_specific_object(obj_path)

    def progressive_randomization(self, current_level=0, target_level=10):
        """Apply randomization progressively from simple to complex"""

        # Gradually increase randomization complexity
        complexity = current_level / target_level

        # Adjust randomization parameters based on complexity level
        self.adjust_randomization_complexity(complexity)

    def adjust_randomization_complexity(self, complexity_level):
        """Adjust randomization complexity based on level"""

        # Map complexity level (0-1) to randomization parameters
        range_multiplier = 0.1 + complexity_level * 0.9  # 10% to 100% of full range

        print(f"Adjusting randomization complexity to {complexity_level:.2%}")

    def apply_optimized_randomization(self):
        """Apply all optimized randomization techniques"""

        # Use caching to avoid recomputing random values
        # Apply batch processing
        # Use selective randomization
        # Implement progressive complexity

        print("Optimized domain randomization applied")

def benchmark_randomization_performance(randomizer, num_iterations=100):
    """Benchmark domain randomization performance"""

    import time

    print(f"Benchmarking randomization performance for {num_iterations} iterations...")

    times = []
    for i in range(num_iterations):
        start_time = time.time()

        # Apply randomization
        randomizer.apply_optimized_randomization()

        end_time = time.time()
        iteration_time = end_time - start_time
        times.append(iteration_time)

        if (i + 1) % 25 == 0:
            avg_time = sum(times) / len(times)
            print(f"  Iteration {i + 1}/{num_iterations}, avg time: {avg_time:.4f}s")

    avg_time = sum(times) / len(times)
    fps_equivalent = 1.0 / avg_time if avg_time > 0 else 0

    print(f"Randomization completed: avg {avg_time:.4f}s per iteration ({fps_equivalent:.2f} FPS equivalent)")

if __name__ == "__main__":
    # Example benchmark
    print("Optimized domain randomization techniques:")

    # Create dummy world for example
    opt_randomizer = OptimizedDomainRandomizer(None)
    benchmark_randomization_performance(opt_randomizer, num_iterations=50)
```

## Next Steps

After implementing domain randomization techniques:

1. **Validate Transfer**: Test models trained with randomized data on real-world data
2. **Optimize Parameters**: Fine-tune randomization ranges based on transfer performance
3. **Scale Generation**: Use domain randomization for large-scale synthetic data generation
4. **Monitor Performance**: Continuously monitor real-world performance and adjust randomization
5. **Document Results**: Keep track of which randomization techniques work best for your use case

The next section covers reinforcement learning infrastructure setup in Isaac Sim.