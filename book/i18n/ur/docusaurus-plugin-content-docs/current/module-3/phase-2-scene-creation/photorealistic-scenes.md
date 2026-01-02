# Isaac Sim میں فوٹو ریلسٹک سینز بنانا

یہ سیکشن ہیومنوائڈ روبوٹکس ایپلی کیشنز کے لیے Isaac Sim میں فوٹو ریلسٹک سینز بنانے کا احاطہ کرتا ہے۔ ہم دریافت کریں گے کہ حقیقت پسندانہ ماحول کیسے بنایا جائے جو ادراک (perception) کی تربیت اور sim-to-real منتقلی کے لیے ضروری بصری مخلصی (visual fidelity) فراہم کرے۔

## Isaac Sim میں فوٹو ریلسٹک رینڈرنگ کو سمجھنا

Isaac Sim جدید ترین فوٹو ریلسٹک رینڈرنگ کی صلاحیتیں فراہم کرنے کے لیے NVIDIA کے Omniverse پلیٹ فارم کا فائدہ اٹھاتا ہے۔ یہ اس کے لیے ضروری ہے:

- **ادراک کی تربیت (Perception Training)**: مصنوعی ڈیٹا تیار کرنا جو حقیقی دنیا کے حالات سے قریب سے میل کھاتا ہو۔
- **Sim-to-Real منتقلی**: اس بات کو یقینی بنانا کہ نقالی (simulation) میں تیار کردہ الگورتھم حقیقی دنیا میں کام کرتے ہیں۔
- **ہیومنوائڈ تعامل**: سماجی روبوٹکس ایپلی کیشنز کے لیے حقیقت پسندانہ ماحول بنانا۔

### رینڈرنگ کی کلیدی ٹیکنالوجیز

1. **RTX Ray Tracing**: درست لائٹنگ اور عکاسی (reflections) کے لیے ریئل ٹائم رے ٹریسنگ۔
2. **Physically-Based Materials**: حقیقت پسندانہ سطح کی خصوصیات کے لیے PBR میٹریلز۔
3. **ایڈوانسڈ لائٹنگ**: حقیقت پسندانہ فال آف اور سائے کے ساتھ متعدد روشنی کے ذرائع۔
4. **Volumetric Effects**: ماحولیاتی اثرات جیسے دھند اور والیومیٹرک لائٹنگ۔

## Isaac Sim میں سین آرکیٹیکچر

### USD (یونیورسل سین ڈیسکرپشن) فارمیٹ

Isaac Sim اپنے مقامی سین فارمیٹ کے طور پر USD کا استعمال کرتا ہے۔ USD فراہم کرتا ہے:

- **توسیع پذیری (Scalability)**: لاکھوں پولی گونز کے ساتھ پیچیدہ سینز کو ہینڈل کرنا۔
- **ماڈیولریٹی**: مختلف اثاثوں اور اجزاء کو بغیر کسی رکاوٹ کے جوڑنا۔
- **اینیمیشن**: پیچیدہ کردار کی اینیمیشنز کے لیے سپورٹ۔
- **سمولیشن**: طبیعیات کے سمولیشن ڈیٹا کے ساتھ انضمام۔

### سین کا درجہ بندی (Hierarchy)

ایک عام Isaac Sim سین اس ڈھانچے کی پیروی کرتا ہے:

```
/World
├── /World/Environment
│   ├── /World/Environment/GroundPlane
│   ├── /World/Environment/Buildings
│   ├── /World/Environment/Objects
│   └── /World/Environment/Lights
├── /World/Robots
│   ├── /World/Robots/HumanoidRobot
│   └── /World/Robots/SupportingRobots
├── /World/Sensors
│   ├── /World/Sensors/Cameras
│   ├── /World/Sensors/LiDAR
│   └── /World/Sensors/IMU
└── /World/Physics
    ├── /World/Physics/WorldSettings
    └── /World/Physics/Materials
```

## اپنا پہلا فوٹو ریلسٹک سین بنانا

### بنیادی سین سیٹ اپ

آئیے فوٹو ریلسٹک عناصر کے ساتھ ایک سادہ انڈور ماحول بنائیں:

```python
#!/usr/bin/env python3
"""
Creating a photorealistic indoor scene in Isaac Sim
"""
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.utils.semantics import add_semantics
from omni.isaac.core.materials import VisualMaterial
from pxr import Gf, UsdLux, UsdGeom
import carb

def create_photorealistic_indoor_scene():
    # Initialize the world
    my_world = World(stage_units_in_meters=1.0)

    # Create the scene root
    create_prim("/World", "Xform")

    # Create a room environment
    create_indoor_environment()

    # Add realistic lighting
    setup_realistic_lighting()

    # Add photorealistic materials
    apply_photorealistic_materials()

    return my_world

def create_indoor_environment():
    """Create a basic indoor environment"""

    # Create room walls
    create_prim(
        prim_path="/World/Room",
        prim_type="Xform",
        position=[0, 0, 0]
    )

    # Floor
    create_prim(
        prim_path="/World/Room/Floor",
        prim_type="Plane",
        position=[0, 0, 0],
        scale=[10, 10, 1],
        orientation=[0.707, 0, 0, 0.707]  # Rotate to be horizontal
    )

    # Walls
    wall_thickness = 0.1
    room_size = 8.0

    # Back wall
    create_prim(
        prim_path="/World/Room/BackWall",
        prim_type="Cube",
        position=[0, room_size/2, 2.0],
        scale=[room_size, wall_thickness, 4.0]
    )

    # Front wall (with opening for door)
    create_prim(
        prim_path="/World/Room/FrontWall",
        prim_type="Cube",
        position=[0, -room_size/2, 2.0],
        scale=[room_size, wall_thickness, 4.0]
    )

    # Left wall
    create_prim(
        prim_path="/World/Room/LeftWall",
        prim_type="Cube",
        position=[-room_size/2, 0, 2.0],
        scale=[wall_thickness, room_size, 4.0]
    )

    # Right wall
    create_prim(
        prim_path="/World/Room/RightWall",
        prim_type="Cube",
        position=[room_size/2, 0, 2.0],
        scale=[wall_thickness, room_size, 4.0]
    )

    # Ceiling
    create_prim(
        prim_path="/World/Room/Ceiling",
        prim_type="Cube",
        position=[0, 0, 4.0],
        scale=[room_size, room_size, wall_thickness]
    )

def setup_realistic_lighting():
    """Set up realistic indoor lighting"""

    # Create dome light (environment light)
    dome_light = UsdLux.DomeLight.Define(omni.usd.get_context().get_stage(), "/World/DomeLight")
    dome_light.CreateIntensityAttr(2.0)
    dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0))  # Slightly blue-white

    # Create key light (main light source)
    key_light = UsdLux.DistantLight.Define(omni.usd.get_context().get_stage(), "/World/KeyLight")
    key_light.CreateIntensityAttr(1000.0)
    key_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.9))
    key_light.AddRotateXOp().Set(-45.0)
    key_light.AddRotateYOp().Set(30.0)

    # Create fill light (softens shadows)
    fill_light = UsdLux.DistantLight.Define(omni.usd.get_context().get_stage(), "/World/FillLight")
    fill_light.CreateIntensityAttr(300.0)
    fill_light.CreateColorAttr(Gf.Vec3f(0.8, 0.8, 1.0))
    fill_light.AddRotateXOp().Set(-20.0)
    fill_light.AddRotateYOp().Set(-120.0)

def apply_photorealistic_materials():
    """Apply photorealistic materials to scene objects"""

    # Create a realistic floor material
    floor_material = VisualMaterial(
        prim_path="/World/Materials/FloorMaterial",
        name="floor_material",
        diffuse_color=(0.8, 0.8, 0.8),
        roughness=0.2,
        metallic=0.0,
        clearcoat=0.0,
        opacity=1.0
    )

    # Apply material to floor
    omni.usd.get_context().get_stage().GetPrimAtPath("/World/Room/Floor").GetAppliedSchema().Bind(floor_material.prim)

def add_furniture_and_objects():
    """Add photorealistic furniture and objects to the scene"""

    # Add a table
    create_prim(
        prim_path="/World/Room/Table",
        prim_type="Cube",
        position=[2.0, 1.0, 0.8],
        scale=[1.5, 0.8, 0.8]
    )

    # Add a chair
    create_prim(
        prim_path="/World/Room/Chair",
        prim_type="Cylinder",
        position=[2.0, -1.0, 0.4],
        scale=[0.4, 0.4, 0.8]
    )

    # Add objects on the table
    create_prim(
        prim_path="/World/Room/Object1",
        prim_type="Sphere",
        position=[2.0, 1.0, 1.2],
        scale=[0.15, 0.15, 0.15]
    )

def run_scene_creation_example():
    """Run the complete scene creation example"""
    print("Creating photorealistic indoor scene...")

    world = create_photorealistic_indoor_scene()
    add_furniture_and_objects()

    # Reset and step the world to see the scene
    world.reset()

    print("Scene created successfully!")
    print("You can now view the scene in Isaac Sim")

    # Run for a few steps to ensure everything is loaded
    for i in range(10):
        world.step(render=True)

    return world

if __name__ == "__main__":
    run_scene_creation_example()
```

## اعلی درجے کی سین تخلیق کی تکنیکیں

### Isaac Sim لائبریری سے اثاثوں کا استعمال

Isaac Sim اثاثوں کی ایک بھرپور لائبریری کے ساتھ آتا ہے۔ انہیں استعمال کرنے کا طریقہ یہاں ہے:

```python
#!/usr/bin/env python3
"""
Using Isaac Sim's built-in asset library for photorealistic scenes
"""
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
import carb

def create_scene_with_library_assets():
    """Create a scene using Isaac Sim's built-in assets"""

    # Initialize the world
    my_world = World(stage_units_in_meters=1.0)

    # Get the assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets. Ensure Isaac Sim is properly installed.")
        return None

    print(f"Assets root path: {assets_root_path}")

    # Create a kitchen scene using library assets
    create_kitchen_environment(assets_root_path)

    # Create an office scene using library assets
    # create_office_environment(assets_root_path)

    my_world.reset()
    return my_world

def create_kitchen_environment(assets_root_path):
    """Create a photorealistic kitchen environment"""

    # Add kitchen environment
    kitchen_path = assets_root_path + "/Isaac/Environments/Simple_Rooms/simple_kitchen.usd"
    add_reference_to_stage(
        usd_path=kitchen_path,
        prim_path="/World/kitchen"
    )

    # Add a simple robot to the scene
    robot_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
    add_reference_to_stage(
        usd_path=robot_path,
        prim_path="/World/Robot"
    )

def create_office_environment(assets_root_path):
    """Create a photorealistic office environment"""

    # Add office environment
    office_path = assets_root_path + "/Isaac/Environments/Simple_Rooms/simple_office.usd"
    add_reference_to_stage(
        usd_path=office_path,
        prim_path="/World/office"
    )
```

## میٹریل اور ٹیکسچر سسٹمز

### فزیکلی بیسڈ رینڈرنگ (PBR) میٹریلز

Isaac Sim مکمل PBR میٹریل ورک فلو کو سپورٹ کرتا ہے:

```python
#!/usr/bin/env python3
"""
Creating and applying PBR materials in Isaac Sim
"""
from omni.isaac.core.materials import VisualMaterial
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.usd

def create_realistic_materials():
    """Create various realistic materials for the scene"""

    stage = omni.usd.get_context().get_stage()

    # Create a realistic wood material
    wood_material = VisualMaterial(
        prim_path="/World/Materials/WoodMaterial",
        name="wood_material",
        diffuse_color=(0.8, 0.6, 0.4),  # Warm wood color
        roughness=0.3,
        metallic=0.1,
        clearcoat=0.8,
        opacity=1.0
    )

    # Create a realistic metal material
    metal_material = VisualMaterial(
        prim_path="/World/Materials/MetalMaterial",
        name="metal_material",
        diffuse_color=(0.7, 0.7, 0.8),
        roughness=0.1,
        metallic=0.9,
        clearcoat=0.5,
        opacity=1.0
    )

    # Create a realistic fabric material
    fabric_material = VisualMaterial(
        prim_path="/World/Materials/FabricMaterial",
        name="fabric_material",
        diffuse_color=(0.9, 0.7, 0.8),  # Soft pink fabric
        roughness=0.7,
        metallic=0.0,
        clearcoat=0.0,
        opacity=1.0
    )

    # Apply materials to objects
    # Example: Apply wood material to a table
    # table_prim = stage.GetPrimAtPath("/World/Room/Table")
    # if table_prim.IsValid():
    #     wood_material.bind(table_prim)

    return wood_material, metal_material, fabric_material
```

## فوٹو ریلزم کے لیے لائٹنگ تکنیک

### ایڈوانسڈ لائٹنگ سیٹ اپ

```python
#!/usr/bin/env python3
"""
Advanced lighting techniques for photorealistic scenes
"""
from pxr import UsdLux, Gf, UsdGeom
import omni.usd

def setup_advanced_lighting():
    """Set up advanced lighting for photorealistic rendering"""

    stage = omni.usd.get_context().get_stage()

    # Create an HDRI environment (dome light with texture)
    dome_light = UsdLux.DomeLight.Define(stage, "/World/HDRI_Environment")
    dome_light.CreateIntensityAttr(1.5)
    dome_light.CreateTextureFileAttr("path/to/hdri/environment.hdr")
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    # Create area lights for soft lighting
    key_area_light = UsdLux.RectLight.Define(stage, "/World/KeyAreaLight")
    key_area_light.CreateIntensityAttr(800.0)
    key_area_light.CreateWidthAttr(2.0)
    key_area_light.CreateHeightAttr(2.0)
    key_area_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.9))
    key_area_light.AddTranslateOp().Set(Gf.Vec3d(3.0, 2.0, 3.0))
    key_area_light.AddRotateXOp().Set(-60.0)

    # Create rim light for depth
    rim_light = UsdLux.DistantLight.Define(stage, "/World/RimLight")
    rim_light.CreateIntensityAttr(500.0)
    rim_light.CreateColorAttr(Gf.Vec3f(0.8, 0.8, 1.0))
    rim_light.AddRotateXOp().Set(20.0)
    rim_light.AddRotateYOp().Set(150.0)

    # Add volumetric effects
    render_product = UsdGeom.Scope.Define(stage, "/Render/Volumetrics")
    # Configure volumetric lighting effects
```

## مصنوعی ڈیٹا کے لیے ڈومین رینڈمائزیشن

### سین کے عناصر کو رینڈمائز کرنا

مصنوعی ڈیٹا جنریشن کے لیے، ڈومین رینڈمائزیشن بہت اہم ہے:

```python
#!/usr/bin/env python3
"""
Domain randomization techniques for synthetic data generation
"""
import random
import numpy as np
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.materials import VisualMaterial

def apply_domain_randomization():
    """Apply domain randomization to scene elements"""

    # Randomize lighting conditions
    randomize_lighting()

    # Randomize material properties
    randomize_materials()

    # Randomize object positions
    randomize_object_positions()

    # Randomize camera parameters
    randomize_camera_parameters()

def randomize_lighting():
    """Randomize lighting conditions"""
    stage = omni.usd.get_context().get_stage()

    # Randomize dome light intensity and color
    dome_light = stage.GetPrimAtPath("/World/DomeLight")
    if dome_light.IsValid():
        intensity = random.uniform(0.5, 3.0)
        color_temp = random.uniform(5000, 8000)  # Kelvin
        dome_light.GetAttribute("inputs:intensity").Set(intensity)

        # Convert color temperature to RGB approximation
        rgb = color_temperature_to_rgb(color_temp)
        dome_light.GetAttribute("inputs:color").Set(Gf.Vec3f(*rgb))

def color_temperature_to_rgb(temp_k):
    """Convert color temperature in Kelvin to RGB"""
    temp_k = max(1000, min(40000, temp_k)) / 100.0

    if temp_k <= 66:
        red = 255
        green = temp_k
        green = 99.4708025861 * math.log(green) - 161.1195681661
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
        blue = 138.5177312231 * math.log(blue) - 305.0447927307

    return (
        max(0, min(255, red)) / 255.0,
        max(0, min(255, green)) / 255.0,
        max(0, min(255, blue)) / 255.0
    )

def randomize_materials():
    """Randomize material properties for domain randomization"""
    # In practice, you would iterate through materials and randomize their properties
    # This is a simplified example
    pass

def randomize_object_positions():
    """Randomize object positions within the scene"""
    # In practice, you would iterate through objects and apply random position variations
    # This is a simplified example
    pass

def randomize_camera_parameters():
    """Randomize camera parameters for synthetic data"""
    # In practice, you would access camera objects and randomize their properties
    # This is a simplified example
    pass
```

## کارکردگی کے لیے سین کی اصلاح (Optimization)

### معیار اور کارکردگی میں توازن

فوٹو ریلسٹک سینز بنانے کے لیے بصری معیار اور کارکردگی میں توازن کی ضرورت ہوتی ہے:

```python
#!/usr/bin/env python3
"""
Scene optimization techniques for balancing quality and performance
"""
def optimize_scene_for_performance():
    """Optimize scene for better performance while maintaining quality"""

    # Use Level of Detail (LOD) for distant objects
    setup_lod_system()

    # Implement occlusion culling
    setup_occlusion_culling()

    # Use texture streaming
    setup_texture_streaming()

    # Optimize geometry complexity
    optimize_geometry()

def setup_lod_system():
    """Set up Level of Detail system"""
    # Create multiple levels of detail for complex objects
    # This is typically handled through USD schemas
    pass

def setup_occlusion_culling():
    """Set up occlusion culling for hidden objects"""
    # Configure rendering settings to skip hidden objects
    pass

def setup_texture_streaming():
    """Set up texture streaming for large scenes"""
    # Configure texture streaming settings
    pass

def optimize_geometry():
    """Optimize geometry complexity"""
    # Reduce polygon count for distant objects
    # Use simpler collision meshes
    pass
```

## فوٹو ریلسٹک سینز کے لیے بہترین طریقے

### ڈیزائن کے اصول

1. **حقیقی دنیا کی تصاویر کا حوالہ دیں**: لائٹنگ اور میٹریلز کے حوالے کے طور پر حقیقی تصاویر کا استعمال کریں۔
2. **مستقل اسکیل (Scale)**: تمام اشیاء کے لیے مناسب حقیقی دنیا کے اسکیل کو برقرار رکھیں۔
3. **طبیعی طور پر درست میٹریلز**: حقیقت پسندانہ خصوصیات کے ساتھ میٹریلز کا استعمال کریں۔
4. **مناسب لائٹنگ سیٹ اپ**: ایک سے زیادہ روشنی کے ذرائع استعمال کریں جو حقیقی دنیا کی لائٹنگ کی نقل کریں۔
5. **ٹیکسچر کا معیار**: مناسب PBR خصوصیات کے ساتھ ہائی ریزولیوشن ٹیکسچرز استعمال کریں۔

### کارکردگی کے رہنما خطوط

1. **فریم ریٹ کی نگرانی کریں**: انٹرایکٹو ڈویلپمنٹ کے لیے رینڈرنگ کو 30+ FPS پر رکھیں۔
2. **پیچیدہ سینز کو بہتر بنائیں**: دہرائی جانے والی اشیاء کے لیے instancing کا استعمال کریں۔
3. **میموری کے استعمال کا نظم کریں**: سمولیشن کے دوران GPU اور سسٹم میموری کی نگرانی کریں۔
4. **مناسب ریزولیوشنز استعمال کریں**: ٹیکسچر ریزولیوشن اور کارکردگی کی ضروریات میں توازن رکھیں۔

## اگلے اقدامات

اپنے فوٹو ریلسٹک سینز بنانے کے بعد:

1. **روبوٹ ماڈلز کو ضم کریں**: اپنے سینز میں ہیومنوائڈ روبوٹس شامل کریں۔
2. **سینسرز کنفیگر کریں**: سین میں کیمرے، LiDAR، اور دیگر سینسرز ترتیب دیں۔
3. **طبیعیات کی جانچ کریں**: تصدیق کریں کہ اشیاء حقیقت پسندانہ طور پر تعامل کرتی ہیں۔
4. **ادراک کے نظام کی توثیق کریں**: ٹیسٹ کریں کہ مصنوعی ڈیٹا حقیقی دنیا کی توقعات سے میل کھاتا ہے۔
5. **تربیت کے لیے بہتر بنائیں**: مصنوعی ڈیٹا جنریشن کے لیے سینز تیار کریں۔

اگلا سیکشن آپ کے فوٹو ریلسٹک سینز میں روبوٹ ماڈلز کو ضم کرنے کا احاطہ کرتا ہے۔
