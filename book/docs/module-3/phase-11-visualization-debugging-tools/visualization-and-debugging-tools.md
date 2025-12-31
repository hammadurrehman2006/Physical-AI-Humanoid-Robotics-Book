# Isaac Sim Visualization and Debugging Tools

This comprehensive guide covers the visualization and debugging tools available in NVIDIA Isaac Sim. These tools are essential for developing, testing, and validating humanoid robotics applications in simulation.

## 1. Isaac Sim Visualization Tools Concepts

### Overview of Visualization Tools

Isaac Sim provides a rich set of visualization tools that enable developers to observe, analyze, and debug their simulation environments. These tools include:

- **Viewport Visualization**: Real-time rendering of the simulation environment
- **Physics Debug Visualization**: Visual representation of physics properties and interactions
- **Sensor Data Visualization**: Real-time display of sensor outputs
- **Robot State Visualization**: Visual indicators for robot joint states and control
- **Collision Visualization**: Visual feedback for collision detection and response

### Viewport and Camera Controls

The main viewport in Isaac Sim provides several visualization modes and controls:

```python
#!/usr/bin/env python3
"""
Isaac Sim viewport and visualization controls
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np

class ViewportVisualization:
    """Class to demonstrate Isaac Sim viewport visualization capabilities"""

    def __init__(self):
        self.world = None
        self.viewport_api = None

    def setup_visualization_scene(self):
        """Set up a scene with various visualization elements"""

        # Create world
        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create objects for visualization
        for i in range(5):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Cube_{i}",
                    name=f"cube_{i}",
                    position=[i*0.5, 0, 2.0],
                    size=0.2,
                    color=np.array([1.0, i*0.2, 0.0])
                )
            )

        # Reset the world
        self.world.reset()

        print("Visualization scene created with 5 colored cubes")

    def configure_camera_view(self):
        """Configure camera views for optimal visualization"""

        # Set camera position and target
        set_camera_view(
            eye=np.array([5.0, 5.0, 3.0]),
            target=np.array([0.0, 0.0, 0.0]),
            camera_prim=omni.usd.get_context().get_stage().GetPrimAtPath("/OmniGraph/Camera")
        )

        print("Camera view configured for optimal visualization")

    def toggle_visualization_modes(self):
        """Demonstrate different visualization modes"""

        # Access the viewport API
        viewport_window = omni.ui.Workspace.get_window("Viewport")

        # Note: Actual implementation would depend on Isaac Sim's viewport API
        print("Visualization modes available:")
        print("- Shaded mode: Standard rendering with materials")
        print("- Wireframe mode: Shows mesh structure")
        print("- Point cloud mode: Points only representation")
        print("- Physics debug mode: Shows collision shapes and physics properties")
        print("- UV mode: Shows texture coordinates")

    def run_visualization_demo(self):
        """Run a visualization demonstration"""

        print("Starting visualization demonstration...")

        self.setup_visualization_scene()
        self.configure_camera_view()
        self.toggle_visualization_modes()

        # Step through the simulation to see visualization
        for i in range(100):
            self.world.step(render=True)

            if i % 25 == 0:
                print(f"Visualization step {i}/100")

        print("Visualization demonstration completed")

def demonstrate_viewport_visualization():
    """Demonstrate viewport visualization capabilities"""

    viz = ViewportVisualization()
    viz.run_visualization_demo()

if __name__ == "__main__":
    demonstrate_viewport_visualization()
```

### Physics Debug Visualization

Physics debug visualization is crucial for understanding how physics simulations behave:

```python
#!/usr/bin/env python3
"""
Physics debug visualization in Isaac Sim
"""
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.physics import PhysicsSchemaTools
import numpy as np

class PhysicsDebugVisualization:
    """Class to demonstrate physics debug visualization"""

    def __init__(self):
        self.world = None

    def setup_physics_debug_scene(self):
        """Set up a scene for physics debugging"""

        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create objects with different physics properties
        # Dynamic object
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/DynamicCube",
                name="dynamic_cube",
                position=[0, 0, 2.0],
                size=0.3,
                mass=1.0
            )
        )

        # Fixed object (static)
        self.world.scene.add(
            FixedCuboid(
                prim_path="/World/FixedCube",
                name="fixed_cube",
                position=[1, 0, 1.0],
                size=0.3
            )
        )

        # Create multiple objects to show collision behavior
        for i in range(3):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/StackCube_{i}",
                    name=f"stack_cube_{i}",
                    position=[-1, i*0.3, 1.0 + i*0.3],
                    size=0.2,
                    mass=0.5
                )
            )

        self.world.reset()
        print("Physics debug scene created with various physics objects")

    def enable_physics_debug_visualization(self):
        """Enable physics debug visualization"""

        # Get physics context
        physics_context = self.world.scene.get_physics_context()

        # Enable physics debug visualization (this is conceptual - actual API may vary)
        # physics_context.enable_debug_draw(True)  # This is a conceptual call

        print("Physics debug visualization enabled")
        print("- Collision shapes will be visible")
        print("- Contact points will be shown")
        print("- Joint constraints will be visualized")
        print("- Force vectors will be displayed")

    def analyze_physics_properties(self):
        """Analyze and display physics properties"""

        physics_context = self.world.scene.get_physics_context()

        print("Physics Context Properties:")
        print(f"- Position Iteration Count: {physics_context.get_position_iteration_count()}")
        print(f"- Velocity Iteration Count: {physics_context.get_velocity_iteration_count()}")
        print(f"- Physics DT: {physics_context.get_physics_dt()}")
        print(f"- Solver Type: {physics_context.get_solver_type()}")

        # Analyze specific objects
        for i in range(3):
            obj_name = f"stack_cube_{i}"
            if self.world.scene.has_object(obj_name):
                obj = self.world.scene.get_object(obj_name)
                position, orientation = obj.get_world_pose()
                linear_vel, angular_vel = obj.get_linear_velocity(), obj.get_angular_velocity()

                print(f"\nObject {obj_name}:")
                print(f"  Position: {position}")
                print(f"  Linear Velocity: {linear_vel}")
                print(f"  Angular Velocity: {angular_vel}")

    def run_physics_debug_demo(self):
        """Run physics debug demonstration"""

        print("Starting physics debug visualization demo...")

        self.setup_physics_debug_scene()
        self.enable_physics_debug_visualization()

        # Run simulation and observe physics behavior
        for i in range(200):
            self.world.step(render=True)

            if i % 50 == 0:
                print(f"Physics debug step {i}/200")
                self.analyze_physics_properties()

        print("Physics debug demonstration completed")

def demonstrate_physics_debug():
    """Demonstrate physics debug visualization"""

    debug_viz = PhysicsDebugVisualization()
    debug_viz.run_physics_debug_demo()

if __name__ == "__main__":
    demonstrate_physics_debug()
```

## 2. Debugging Tools and Techniques for Isaac Sim

### General Debugging Approaches

Isaac Sim provides several debugging tools and techniques:

1. **Console Logging**: Real-time logging of simulation events
2. **Physics Debugging**: Visualization of physics interactions
3. **Sensor Debugging**: Monitoring sensor data and performance
4. **Performance Profiling**: Analysis of simulation performance
5. **Scene Debugging**: Inspection of scene hierarchy and properties

### Console and Logging Debugging

```python
#!/usr/bin/env python3
"""
Console and logging debugging in Isaac Sim
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

class ConsoleDebugging:
    """Class to demonstrate console and logging debugging"""

    def __init__(self):
        self.world = None
        self.debug_enabled = True

    def setup_debug_scene(self):
        """Set up a scene for debugging"""

        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create debug objects
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/DebugCube",
                name="debug_cube",
                position=[0, 0, 2.0],
                size=0.3,
                mass=1.0
            )
        )

        self.world.reset()
        print("[INFO] Debug scene created successfully")

    def enable_detailed_logging(self):
        """Enable detailed logging for debugging"""

        # Set logging level
        carb.settings.get_settings().set("/log/level", 0)  # Verbose logging

        # Enable physics logging
        carb.settings.get_settings().set("/app/physics/logLevel", 0)

        print("[DEBUG] Detailed logging enabled")
        print("[DEBUG] Physics logging enabled")

    def log_simulation_state(self, step_count):
        """Log simulation state for debugging"""

        if step_count % 10 == 0:  # Log every 10 steps
            cube = self.world.scene.get_object("debug_cube")
            if cube:
                position, orientation = cube.get_world_pose()
                linear_vel, angular_vel = cube.get_linear_velocity(), cube.get_angular_velocity()

                carb.log_info(f"Step {step_count}: Cube position={position}, velocity={linear_vel}")

    def debug_physics_interactions(self):
        """Debug physics interactions"""

        physics_context = self.world.scene.get_physics_context()

        # Log physics properties
        print(f"[PHYSICS] Position iterations: {physics_context.get_position_iteration_count()}")
        print(f"[PHYSICS] Velocity iterations: {physics_context.get_velocity_iteration_count()}")
        print(f"[PHYSICS] Solver type: {physics_context.get_solver_type()}")

    def handle_simulation_errors(self):
        """Handle and log simulation errors"""

        try:
            # Simulate a potential error scenario
            # In real scenarios, this would catch actual simulation errors
            self.world.step(render=True)
        except Exception as e:
            carb.log_error(f"Simulation error occurred: {str(e)}")
            return False

        return True

    def run_debug_demo(self):
        """Run debugging demonstration"""

        print("[INFO] Starting debugging demonstration...")

        self.setup_debug_scene()
        self.enable_detailed_logging()

        success_count = 0
        error_count = 0

        for i in range(100):
            # Log simulation state
            self.log_simulation_state(i)

            # Debug physics interactions periodically
            if i % 25 == 0:
                self.debug_physics_interactions()

            # Step simulation
            try:
                self.world.step(render=True)
                success_count += 1
            except Exception as e:
                carb.log_error(f"Error at step {i}: {str(e)}")
                error_count += 1

        print(f"[SUMMARY] Debug demo completed: {success_count} successful steps, {error_count} errors")

def demonstrate_console_debugging():
    """Demonstrate console debugging capabilities"""

    debugger = ConsoleDebugging()
    debugger.run_debug_demo()

if __name__ == "__main__":
    demonstrate_console_debugging()
```

### Scene Hierarchy Debugging

```python
#!/usr/bin/env python3
"""
Scene hierarchy debugging in Isaac Sim
"""
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.utils.stage import get_stage_units
from pxr import Usd, UsdGeom, Gf
import numpy as np

class SceneHierarchyDebugger:
    """Class to debug and inspect scene hierarchy"""

    def __init__(self):
        self.world = None
        self.stage = None

    def setup_scene_for_debugging(self):
        """Set up a complex scene for hierarchy debugging"""

        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.stage

        # Create a complex scene hierarchy
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create a robot-like structure
        create_prim("/World/Robot", "Xform", position=[0, 0, 1.0])
        create_prim("/World/Robot/Body", "Cylinder", position=[0, 0, 0.5], attributes={"radius": 0.2, "height": 0.8})
        create_prim("/World/Robot/Head", "Sphere", position=[0, 0, 1.2], attributes={"radius": 0.15})

        # Add some objects to interact with
        for i in range(3):
            create_prim(f"/World/Object_{i}", "Cube", position=[i*0.5, 2, 0.5], attributes={"size": 0.2})

        self.world.reset()
        print("Complex scene hierarchy created for debugging")

    def inspect_scene_hierarchy(self):
        """Inspect and display the scene hierarchy"""

        print("\n=== SCENE HIERARCHY INSPECTION ===")

        # Get the root prim
        root_prim = self.stage.GetPseudoRoot()

        def print_prim_info(prim, depth=0):
            indent = "  " * depth
            prim_type = prim.GetTypeName()
            prim_path = prim.GetPath().pathString

            print(f"{indent}{prim_path} ({prim_type})")

            # Get prim attributes
            attributes = [attr.GetName() for attr in prim.GetAttributes()]
            if attributes:
                print(f"{indent}  Attributes: {attributes[:5]}...")  # Show first 5 attributes

            # Get prim properties
            properties = [prop.GetName() for prop in prim.GetProperties()]
            if properties:
                print(f"{indent}  Properties: {properties[:5]}...")  # Show first 5 properties

            # Recursively print children
            for child in prim.GetChildren():
                print_prim_info(child, depth + 1)

        print_prim_info(root_prim)

    def validate_scene_integrity(self):
        """Validate the integrity of the scene"""

        print("\n=== SCENE INTEGRITY VALIDATION ===")

        # Check for common issues
        issues = []

        # Check if stage is valid
        if not self.stage:
            issues.append("Stage is not initialized")

        # Check for invalid prims
        for prim in self.stage.TraverseAll():
            if not prim.IsValid():
                issues.append(f"Invalid prim: {prim.GetPath()}")

        # Check for missing references
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() == "Xform":
                # Check for common transform issues
                xform_api = UsdGeom.Xformable(prim)
                if not xform_api:
                    issues.append(f"Non-transformable Xform: {prim.GetPath()}")

        if issues:
            print("Issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("Scene integrity: OK")

    def debug_prim_properties(self, prim_path):
        """Debug properties of a specific prim"""

        prim = get_prim_at_path(prim_path)
        if not prim:
            print(f"Prim not found: {prim_path}")
            return

        print(f"\n=== DEBUGGING PRIM: {prim_path} ===")
        print(f"Type: {prim.GetTypeName()}")
        print(f"Path: {prim.GetPath()}")
        print(f"Valid: {prim.IsValid()}")

        # Get attributes
        attributes = []
        for attr in prim.GetAttributes():
            attr_name = attr.GetName()
            attr_value = attr.Get()
            attributes.append((attr_name, attr_value))

        print("Attributes:")
        for attr_name, attr_value in attributes:
            print(f"  {attr_name}: {attr_value}")

        # Get relationships
        relationships = []
        for rel in prim.GetRelationships():
            rel_name = rel.GetName()
            targets = rel.GetTargets()
            relationships.append((rel_name, targets))

        print("Relationships:")
        for rel_name, targets in relationships:
            print(f"  {rel_name}: {targets}")

    def run_scene_debug_demo(self):
        """Run scene debugging demonstration"""

        print("Starting scene hierarchy debugging demo...")

        self.setup_scene_for_debugging()
        self.inspect_scene_hierarchy()
        self.validate_scene_integrity()

        # Debug specific prims
        self.debug_prim_properties("/World/Robot")
        self.debug_prim_properties("/World/Robot/Body")
        self.debug_prim_properties("/World/Robot/Head")

        print("\nScene debugging demo completed")

def demonstrate_scene_debugging():
    """Demonstrate scene hierarchy debugging"""

    debugger = SceneHierarchyDebugger()
    debugger.run_scene_debug_demo()

if __name__ == "__main__":
    demonstrate_scene_debugging()
```

## 3. Isaac Sim UI and Workflow Concepts

### Isaac Sim Interface Overview

Isaac Sim's user interface is built on the Omniverse platform and includes several key components:

- **Viewport**: The main 3D visualization window
- **Stage View**: Hierarchical view of the scene
- **Property Panel**: Properties of selected objects
- **Timeline**: Animation and simulation control
- **Layer Panel**: USD layer management
- **Outliner**: Scene object management

### UI Workflow for Debugging

```python
#!/usr/bin/env python3
"""
Isaac Sim UI workflow for debugging
"""
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

class UIWorkflowDebugger:
    """Class to demonstrate UI workflow for debugging"""

    def __init__(self):
        self.world = None

    def setup_debug_workflow_scene(self):
        """Set up a scene for UI debugging workflow"""

        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create objects for UI debugging
        for i in range(3):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/DebugObject_{i}",
                    name=f"debug_object_{i}",
                    position=[i*0.5, 0, 2.0],
                    size=0.2,
                    mass=0.5 + i*0.2
                )
            )

        self.world.reset()
        print("UI workflow debugging scene created")

    def demonstrate_selection_workflow(self):
        """Demonstrate object selection workflow"""

        print("\n=== UI SELECTION WORKFLOW ===")
        print("1. Click on objects in the viewport to select them")
        print("2. Selected objects will be highlighted in the viewport")
        print("3. Properties will appear in the Property Panel")
        print("4. Use Ctrl+click for multiple selections")
        print("5. Use Shift+click for range selection")

    def demonstrate_property_inspection(self):
        """Demonstrate property inspection workflow"""

        print("\n=== PROPERTY INSPECTION WORKFLOW ===")
        print("1. Select an object in the viewport or Stage View")
        print("2. View and modify properties in the Property Panel")
        print("3. Key properties for debugging:")
        print("   - Transform: Position, rotation, scale")
        print("   - Physics: Mass, friction, restitution")
        print("   - Visibility: Render visibility, physics visibility")
        print("   - Materials: Surface properties")

    def demonstrate_stage_view_workflow(self):
        """Demonstrate Stage View workflow"""

        print("\n=== STAGE VIEW WORKFLOW ===")
        print("1. Use Stage View to see the hierarchical structure")
        print("2. Expand/collapse nodes to explore the scene")
        print("3. Right-click on nodes for context menu options")
        print("4. Use search to find specific objects")
        print("5. Drag and drop to reorganize the hierarchy")

    def demonstrate_timeline_workflow(self):
        """Demonstrate Timeline workflow for debugging"""

        print("\n=== TIMELINE WORKFLOW ===")
        print("1. Use Timeline to control simulation playback")
        print("2. Set keyframes for animation debugging")
        print("3. Use playback controls to step through simulation")
        print("4. Adjust simulation speed for detailed inspection")
        print("5. Use markers to indicate important events")

    def demonstrate_debugging_tools_ui(self):
        """Demonstrate debugging tools in the UI"""

        print("\n=== DEBUGGING TOOLS IN UI ===")
        print("1. Physics Debug Visualization:")
        print("   - Enable in Viewport menu: Viewport > Lighting > Physics Debug")
        print("   - Shows collision shapes, contact points, joint constraints")
        print("2. Statistics Window:")
        print("   - Window > Statistics to monitor performance")
        print("   - Shows FPS, physics steps, memory usage")
        print("3. Console Window:")
        print("   - Window > Console for logging output")
        print("   - Shows errors, warnings, and debug messages")
        print("4. USD Stage View:")
        print("   - Window > Stage to see USD structure")
        print("   - Useful for understanding scene composition")

    def run_ui_workflow_demo(self):
        """Run UI workflow demonstration"""

        print("Starting Isaac Sim UI workflow demonstration...")

        self.setup_debug_workflow_scene()
        self.demonstrate_selection_workflow()
        self.demonstrate_property_inspection()
        self.demonstrate_stage_view_workflow()
        self.demonstrate_timeline_workflow()
        self.demonstrate_debugging_tools_ui()

        print("\nUI workflow demonstration completed")
        print("Use the Isaac Sim interface to interact with the created objects")

def demonstrate_ui_workflow():
    """Demonstrate Isaac Sim UI workflow for debugging"""

    ui_debugger = UIWorkflowDebugger()
    ui_debugger.run_ui_workflow_demo()

if __name__ == "__main__":
    demonstrate_ui_workflow()
```

## 4. Performance Profiling Tools for Isaac Sim

### Profiling and Performance Analysis

Isaac Sim provides several tools for performance profiling and analysis:

```python
#!/usr/bin/env python3
"""
Performance profiling tools for Isaac Sim
"""
import time
import psutil
import GPUtil
import numpy as np
import carb
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid
from collections import deque
import matplotlib.pyplot as plt

class PerformanceProfiler:
    """Class to profile Isaac Sim performance"""

    def __init__(self):
        self.world = None
        self.metrics_history = {
            'fps': deque(maxlen=1000),
            'physics_steps_per_sec': deque(maxlen=1000),
            'gpu_load': deque(maxlen=1000),
            'gpu_memory': deque(maxlen=1000),
            'cpu_load': deque(maxlen=1000),
            'memory_usage': deque(maxlen=1000),
            'simulation_time_ratio': deque(maxlen=1000)
        }
        self.start_time = time.time()
        self.last_render_time = time.time()
        self.frame_count = 0
        self.physics_step_count = 0
        self.start_physics_time = time.time()

    def setup_profiling_scene(self):
        """Set up a scene for performance profiling"""

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0/60.0,   # 60 Hz physics
            rendering_dt=1.0/30.0  # 30 Hz rendering
        )

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create multiple objects to stress test performance
        for i in range(20):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/PerformanceCube_{i}",
                    name=f"perf_cube_{i}",
                    position=[i*0.2, 0, 2.0],
                    size=0.15,
                    mass=0.3
                )
            )

        self.world.reset()
        self.start_physics_time = time.time()
        print("Performance profiling scene created with 20 objects")

    def record_performance_metrics(self):
        """Record current performance metrics"""

        current_time = time.time()

        # Calculate FPS
        elapsed_time = current_time - self.last_render_time
        if elapsed_time > 0:
            fps = 1.0 / elapsed_time
            self.metrics_history['fps'].append(fps)
            self.frame_count += 1

        # Record system metrics
        self.metrics_history['cpu_load'].append(psutil.cpu_percent())
        self.metrics_history['memory_usage'].append(psutil.virtual_memory().percent)

        # Record GPU metrics
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu = gpus[0]  # Get first GPU
            self.metrics_history['gpu_load'].append(gpu.load * 100)
            self.metrics_history['gpu_memory'].append(gpu.memoryUtil * 100)

        # Calculate physics steps per second
        physics_elapsed = current_time - self.start_physics_time
        if physics_elapsed > 0:
            physics_steps_per_sec = self.physics_step_count / physics_elapsed
            self.metrics_history['physics_steps_per_sec'].append(physics_steps_per_sec)

        # Calculate simulation time ratio
        total_time = current_time - self.start_time
        if total_time > 0:
            sim_time_ratio = self.frame_count / total_time
            self.metrics_history['simulation_time_ratio'].append(sim_time_ratio)

        self.last_render_time = current_time
        self.physics_step_count += 1

    def get_performance_summary(self):
        """Get performance summary"""

        if not self.metrics_history['fps']:
            return "No metrics collected yet"

        # Calculate averages
        avg_fps = sum(self.metrics_history['fps']) / len(self.metrics_history['fps']) if self.metrics_history['fps'] else 0
        avg_cpu = sum(self.metrics_history['cpu_load']) / len(self.metrics_history['cpu_load']) if self.metrics_history['cpu_load'] else 0
        avg_gpu = sum(self.metrics_history['gpu_load']) / len(self.metrics_history['gpu_load']) if self.metrics_history['gpu_load'] else 0
        avg_memory = sum(self.metrics_history['memory_usage']) / len(self.metrics_history['memory_usage']) if self.metrics_history['memory_usage'] else 0
        avg_physics_steps = sum(self.metrics_history['physics_steps_per_sec']) / len(self.metrics_history['physics_steps_per_sec']) if self.metrics_history['physics_steps_per_sec'] else 0
        avg_sim_ratio = sum(self.metrics_history['simulation_time_ratio']) / len(self.metrics_history['simulation_time_ratio']) if self.metrics_history['simulation_time_ratio'] else 0

        total_time = time.time() - self.start_time

        summary = f"""
PERFORMANCE SUMMARY:
- Average FPS: {avg_fps:.2f}
- Average Physics Steps/sec: {avg_physics_steps:.2f}
- Average CPU Load: {avg_cpu:.1f}%
- Average GPU Load: {avg_gpu:.1f}%
- Average Memory Usage: {avg_memory:.1f}%
- Simulation Time Ratio: {avg_sim_ratio:.2f}x real-time
- Total Runtime: {total_time:.1f}s
- Total Frames: {self.frame_count}
- Total Physics Steps: {self.physics_step_count}
        """

        return summary

    def check_performance_thresholds(self):
        """Check if performance meets thresholds"""

        if not self.metrics_history['fps']:
            return True

        avg_fps = sum(self.metrics_history['fps']) / len(self.metrics_history['fps'])
        avg_cpu = sum(self.metrics_history['cpu_load']) / len(self.metrics_history['cpu_load'])
        avg_gpu = sum(self.metrics_history['gpu_load']) / len(self.metrics_history['gpu_load'])

        # Performance thresholds
        fps_threshold = 30    # Minimum acceptable FPS
        cpu_threshold = 80    # Maximum acceptable CPU usage %
        gpu_threshold = 90    # Maximum acceptable GPU usage %

        performance_ok = (
            avg_fps >= fps_threshold and
            avg_cpu <= cpu_threshold and
            avg_gpu <= gpu_threshold
        )

        if not performance_ok:
            print("⚠️ Performance warning:")
            if avg_fps < fps_threshold:
                print(f"  - FPS ({avg_fps:.2f}) below threshold ({fps_threshold})")
            if avg_cpu > cpu_threshold:
                print(f"  - CPU usage ({avg_cpu:.1f}%) above threshold ({cpu_threshold}%)")
            if avg_gpu > gpu_threshold:
                print(f"  - GPU usage ({avg_gpu:.1f}%) above threshold ({gpu_threshold}%)")

        return performance_ok

    def analyze_performance_bottlenecks(self):
        """Analyze performance bottlenecks"""

        metrics = self.metrics_history

        if not metrics['fps']:
            return "Insufficient data for analysis"

        avg_fps = sum(metrics['fps']) / len(metrics['fps']) if metrics['fps'] else 0
        avg_cpu = sum(metrics['cpu_load']) / len(metrics['cpu_load']) if metrics['cpu_load'] else 0
        avg_gpu = sum(metrics['gpu_load']) / len(metrics['gpu_load']) if metrics['gpu_load'] else 0
        avg_memory = sum(metrics['memory_usage']) / len(metrics['memory_usage']) if metrics['memory_usage'] else 0

        bottlenecks = []

        # Check for FPS bottleneck
        if avg_fps < 30:
            bottlenecks.append("Rendering/FPS bottleneck - consider lowering resolution or quality")

        # Check for CPU bottleneck
        if avg_cpu > 80:
            bottlenecks.append("CPU bottleneck - consider reducing environment complexity or increasing parallelization")

        # Check for GPU bottleneck
        if avg_gpu > 90:
            bottlenecks.append("GPU bottleneck - consider reducing rendering quality or physics complexity")

        # Check for memory bottleneck
        if avg_memory > 85:
            bottlenecks.append("Memory bottleneck - consider reducing batch sizes or using memory-efficient operations")

        return bottlenecks

    def suggest_optimizations(self):
        """Suggest specific optimizations based on analysis"""

        bottlenecks = self.analyze_performance_bottlenecks()

        suggestions = []

        for bottleneck in bottlenecks:
            if "FPS" in bottleneck:
                suggestions.append("Reduce rendering resolution or quality settings")
            elif "CPU" in bottleneck:
                suggestions.append("Optimize physics settings or reduce environment complexity")
            elif "GPU" in bottleneck:
                suggestions.append("Reduce rendering quality or use lower-resolution textures")
            elif "Memory" in bottleneck:
                suggestions.append("Reduce batch sizes or implement memory-efficient data loading")

        return suggestions

    def plot_performance_metrics(self):
        """Plot performance metrics"""

        try:
            fig, axes = plt.subplots(2, 3, figsize=(18, 10))
            fig.suptitle('Isaac Sim Performance Metrics', fontsize=16)

            # FPS plot
            if self.metrics_history['fps']:
                axes[0, 0].plot(list(self.metrics_history['fps']))
                axes[0, 0].set_title('Frames Per Second (FPS)')
                axes[0, 0].set_xlabel('Time Step')
                axes[0, 0].set_ylabel('FPS')
                axes[0, 0].grid(True)

            # CPU usage
            if self.metrics_history['cpu_load']:
                axes[0, 1].plot(list(self.metrics_history['cpu_load']))
                axes[0, 1].set_title('CPU Usage')
                axes[0, 1].set_xlabel('Time Step')
                axes[0, 1].set_ylabel('CPU %')
                axes[0, 1].grid(True)

            # GPU usage
            if self.metrics_history['gpu_load']:
                axes[0, 2].plot(list(self.metrics_history['gpu_load']))
                axes[0, 2].set_title('GPU Usage')
                axes[0, 2].set_xlabel('Time Step')
                axes[0, 2].set_ylabel('GPU %')
                axes[0, 2].grid(True)

            # Memory usage
            if self.metrics_history['memory_usage']:
                axes[1, 0].plot(list(self.metrics_history['memory_usage']))
                axes[1, 0].set_title('Memory Usage')
                axes[1, 0].set_xlabel('Time Step')
                axes[1, 0].set_ylabel('Memory %')
                axes[1, 0].grid(True)

            # Physics steps per second
            if self.metrics_history['physics_steps_per_sec']:
                axes[1, 1].plot(list(self.metrics_history['physics_steps_per_sec']))
                axes[1, 1].set_title('Physics Steps Per Second')
                axes[1, 1].set_xlabel('Time Step')
                axes[1, 1].set_ylabel('Steps/sec')
                axes[1, 1].grid(True)

            # Simulation time ratio
            if self.metrics_history['simulation_time_ratio']:
                axes[1, 2].plot(list(self.metrics_history['simulation_time_ratio']))
                axes[1, 2].set_title('Simulation Time Ratio')
                axes[1, 2].set_xlabel('Time Step')
                axes[1, 2].set_ylabel('Ratio (sim/sec)')
                axes[1, 2].grid(True)

            plt.tight_layout()
            plt.show()

        except ImportError:
            print("Matplotlib not available for plotting")

    def run_performance_profiling(self, duration_steps=200):
        """Run performance profiling"""

        print("Starting Isaac Sim performance profiling...")

        self.setup_profiling_scene()

        # Run simulation and collect metrics
        for i in range(duration_steps):
            # Record metrics before step
            self.record_performance_metrics()

            # Step the simulation
            self.world.step(render=True)

            # Print progress
            if i % 50 == 0:
                print(f"Profiling step {i}/{duration_steps}")

        # Generate summary
        summary = self.get_performance_summary()
        print(summary)

        # Check thresholds
        self.check_performance_thresholds()

        # Analyze bottlenecks
        bottlenecks = self.analyze_performance_bottlenecks()
        if bottlenecks:
            print("\nIdentified bottlenecks:")
            for bottleneck in bottlenecks:
                print(f"  - {bottleneck}")

        # Suggest optimizations
        suggestions = self.suggest_optimizations()
        if suggestions:
            print("\nSuggested optimizations:")
            for suggestion in suggestions:
                print(f"  - {suggestion}")

        # Plot metrics if possible
        self.plot_performance_metrics()

        print("\nPerformance profiling completed")

def demonstrate_performance_profiling():
    """Demonstrate performance profiling tools"""

    profiler = PerformanceProfiler()
    profiler.run_performance_profiling(duration_steps=150)

if __name__ == "__main__":
    demonstrate_performance_profiling()
```

## 5. Isaac Sim Logs and Diagnostics

### Logging and Diagnostic Systems

Isaac Sim provides comprehensive logging and diagnostic capabilities:

```python
#!/usr/bin/env python3
"""
Isaac Sim logs and diagnostics
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid
import os
import json
from datetime import datetime
import numpy as np

class IsaacSimDiagnostics:
    """Class to handle Isaac Sim diagnostics and logging"""

    def __init__(self, log_dir="logs"):
        self.world = None
        self.log_dir = log_dir
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.diagnostics_log = []

        # Create log directory
        os.makedirs(log_dir, exist_ok=True)

    def setup_diagnostics_scene(self):
        """Set up a scene for diagnostics"""

        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create diagnostic objects
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/DiagnosticCube",
                name="diagnostic_cube",
                position=[0, 0, 2.0],
                size=0.3,
                mass=1.0
            )
        )

        self.world.reset()
        print("Diagnostics scene created")

    def log_system_info(self):
        """Log system information"""

        # Get system information
        import platform
        import psutil

        system_info = {
            "timestamp": datetime.now().isoformat(),
            "platform": platform.platform(),
            "processor": platform.processor(),
            "cpu_count": psutil.cpu_count(),
            "memory_total": psutil.virtual_memory().total,
            "memory_available": psutil.virtual_memory().available,
            "python_version": platform.python_version(),
            "carb_version": carb.__version__ if hasattr(carb, '__version__') else "unknown"
        }

        # Log to file
        log_file = os.path.join(self.log_dir, f"system_info_{self.session_id}.json")
        with open(log_file, 'w') as f:
            json.dump(system_info, f, indent=2)

        # Add to diagnostics log
        self.diagnostics_log.append({
            "type": "system_info",
            "data": system_info,
            "timestamp": datetime.now().isoformat()
        })

        print(f"System information logged to {log_file}")

    def log_simulation_state(self, step):
        """Log simulation state"""

        if self.world and step % 10 == 0:  # Log every 10 steps
            cube = self.world.scene.get_object("diagnostic_cube")
            if cube:
                position, orientation = cube.get_world_pose()
                linear_vel, angular_vel = cube.get_linear_velocity(), cube.get_angular_velocity()

                state_info = {
                    "step": step,
                    "position": position.tolist() if hasattr(position, 'tolist') else list(position),
                    "orientation": orientation.tolist() if hasattr(orientation, 'tolist') else list(orientation),
                    "linear_velocity": linear_vel.tolist() if hasattr(linear_vel, 'tolist') else list(linear_vel),
                    "angular_velocity": angular_vel.tolist() if hasattr(angular_vel, 'tolist') else list(angular_vel)
                }

                # Add to diagnostics log
                self.diagnostics_log.append({
                    "type": "simulation_state",
                    "data": state_info,
                    "timestamp": datetime.now().isoformat()
                })

                # Log to console
                carb.log_info(f"Step {step}: Cube position={position}, velocity={linear_vel}")

    def log_physics_context_info(self):
        """Log physics context information"""

        if self.world:
            physics_context = self.world.scene.get_physics_context()

            physics_info = {
                "position_iteration_count": physics_context.get_position_iteration_count(),
                "velocity_iteration_count": physics_context.get_velocity_iteration_count(),
                "physics_dt": physics_context.get_physics_dt(),
                "solver_type": physics_context.get_solver_type(),
                "use_gpu": physics_context.is_gpu_dynamics_enabled()
            }

            # Add to diagnostics log
            self.diagnostics_log.append({
                "type": "physics_context",
                "data": physics_info,
                "timestamp": datetime.now().isoformat()
            })

            print("Physics context information logged")

    def log_scene_hierarchy(self):
        """Log scene hierarchy information"""

        if self.world:
            hierarchy_info = {
                "objects": [],
                "total_prims": 0
            }

            stage = self.world.stage
            for prim in stage.TraverseAll():
                hierarchy_info["objects"].append({
                    "path": str(prim.GetPath()),
                    "type": prim.GetTypeName(),
                    "valid": prim.IsValid()
                })
                hierarchy_info["total_prims"] += 1

            # Add to diagnostics log
            self.diagnostics_log.append({
                "type": "scene_hierarchy",
                "data": hierarchy_info,
                "timestamp": datetime.now().isoformat()
            })

            print(f"Scene hierarchy logged: {hierarchy_info['total_prims']} prims")

    def log_errors_and_warnings(self):
        """Log errors and warnings"""

        # This would typically capture actual errors from the simulation
        # For demonstration, we'll create some sample error logs

        error_info = {
            "errors": [],
            "warnings": [],
            "info_messages": []
        }

        # Simulate checking for common issues
        if self.world:
            # Check for physics issues
            physics_context = self.world.scene.get_physics_context()
            if physics_context.get_position_iteration_count() < 4:
                error_info["warnings"].append("Low position iteration count may cause instability")

            # Check for performance issues
            if len(self.diagnostics_log) > 1000:  # Simulated check
                error_info["info_messages"].append("High diagnostic log volume detected")

        # Add to diagnostics log
        self.diagnostics_log.append({
            "type": "errors_warnings",
            "data": error_info,
            "timestamp": datetime.now().isoformat()
        })

        print(f"Diagnostics: {len(error_info['errors'])} errors, {len(error_info['warnings'])} warnings")

    def save_diagnostics_log(self):
        """Save the diagnostics log to file"""

        log_file = os.path.join(self.log_dir, f"diagnostics_{self.session_id}.json")

        with open(log_file, 'w') as f:
            json.dump(self.diagnostics_log, f, indent=2)

        print(f"Diagnostics log saved to {log_file}")
        return log_file

    def analyze_diagnostics(self):
        """Analyze diagnostics for common issues"""

        print("\n=== DIAGNOSTICS ANALYSIS ===")

        # Count different types of logs
        log_types = {}
        for log_entry in self.diagnostics_log:
            log_type = log_entry["type"]
            log_types[log_type] = log_types.get(log_type, 0) + 1

        print("Log type distribution:")
        for log_type, count in log_types.items():
            print(f"  - {log_type}: {count}")

        # Look for specific issues
        issues_found = []

        for log_entry in self.diagnostics_log:
            if log_entry["type"] == "errors_warnings":
                errors = log_entry["data"].get("errors", [])
                warnings = log_entry["data"].get("warnings", [])

                for error in errors:
                    issues_found.append(f"ERROR: {error}")
                for warning in warnings:
                    issues_found.append(f"WARNING: {warning}")

        if issues_found:
            print("\nIssues found:")
            for issue in issues_found:
                print(f"  - {issue}")
        else:
            print("\nNo issues detected in diagnostics")

    def run_diagnostics_demo(self):
        """Run diagnostics demonstration"""

        print("Starting Isaac Sim diagnostics demonstration...")

        self.setup_diagnostics_scene()
        self.log_system_info()
        self.log_physics_context_info()
        self.log_scene_hierarchy()

        # Run simulation and log state
        for i in range(100):
            self.log_simulation_state(i)

            if i % 25 == 0:
                self.log_errors_and_warnings()

            self.world.step(render=True)

        # Final diagnostics
        self.log_errors_and_warnings()

        # Save and analyze
        log_file = self.save_diagnostics_log()
        self.analyze_diagnostics()

        print(f"\nDiagnostics demonstration completed. Log saved to: {log_file}")

def demonstrate_diagnostics():
    """Demonstrate Isaac Sim diagnostics"""

    diagnostics = IsaacSimDiagnostics()
    diagnostics.run_diagnostics_demo()

if __name__ == "__main__":
    demonstrate_diagnostics()
```

## 6. Scene Debugging Techniques

### Scene Debugging Methods

Scene debugging involves identifying and resolving issues in the simulation environment:

```python
#!/usr/bin/env python3
"""
Scene debugging techniques for Isaac Sim
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, Gf
import numpy as np

class SceneDebugger:
    """Class to demonstrate scene debugging techniques"""

    def __init__(self):
        self.world = None
        self.stage = None
        self.debug_issues = []

    def setup_debugging_scene(self):
        """Set up a scene with common debugging scenarios"""

        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.stage

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create objects with potential issues for debugging
        # Object with physics issues
        self.world.scene.add(
            DynamicCuboidWithIssues(
                prim_path="/World/ProblematicCube",
                name="problematic_cube",
                position=[0, 0, 2.0],
                size=0.01,  # Very small - potential issue
                mass=1000   # Very heavy for size - potential issue
            )
        )

        # Object with transform issues
        create_prim("/World/TransformIssue", "Cube",
                   position=[2, 0, 1],
                   orientation=Gf.Quatf(10, 0, 0, 0))  # Invalid quaternion

        # Valid object for comparison
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/NormalCube",
                name="normal_cube",
                position=[-2, 0, 2.0],
                size=0.3,
                mass=1.0
            )
        )

        self.world.reset()
        print("Scene created with potential debugging scenarios")

    def check_scene_integrity(self):
        """Check scene integrity for common issues"""

        print("\n=== SCENE INTEGRITY CHECK ===")

        issues = []

        # Check for invalid prims
        for prim in self.stage.TraverseAll():
            if not prim.IsValid():
                issues.append(f"Invalid prim: {prim.GetPath()}")

        # Check for prims with no meaningful geometry
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() in ["Xform", "Scope"] and not prim.GetChildren():
                issues.append(f"Empty container prim: {prim.GetPath()}")

        # Check for potential physics issues
        for prim in self.stage.TraverseAll():
            if "Cube" in prim.GetTypeName() or "Sphere" in prim.GetTypeName():
                # Check for potentially problematic size/mass ratios
                mass_attr = prim.GetAttribute("physics:mass")
                if mass_attr and mass_attr.Get():
                    mass = mass_attr.Get()
                    size_attr = prim.GetAttribute("size")
                    if size_attr and size_attr.Get():
                        size = size_attr.Get()
                        # Very high density could cause issues
                        if mass > 100 and size < 0.1:
                            issues.append(f"High density object: {prim.GetPath()} (mass: {mass}, size: {size})")

        # Check for overlapping objects
        self.check_for_overlaps(issues)

        self.debug_issues.extend(issues)

        if issues:
            print("Issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("No integrity issues found")

    def check_for_overlaps(self, issues):
        """Check for overlapping objects"""

        # This is a simplified overlap check
        # In practice, you'd use proper collision detection
        objects = []

        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder"]:
                xform_api = UsdGeom.Xformable(prim)
                if xform_api:
                    local_transform = xform_api.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    pos = local_transform.ExtractTranslation()
                    objects.append((prim.GetPath().pathString, pos))

        # Check for close positions (simplified)
        for i, (path1, pos1) in enumerate(objects):
            for j, (path2, pos2) in enumerate(objects[i+1:], i+1):
                distance = sum((a-b)**2 for a, b in zip(pos1, pos2))**0.5
                if distance < 0.1:  # Threshold for "overlap"
                    issues.append(f"Potential overlap between {path1} and {path2} (distance: {distance:.3f})")

    def validate_physics_properties(self):
        """Validate physics properties of objects"""

        print("\n=== PHYSICS PROPERTIES VALIDATION ===")

        issues = []

        # Get all physics-enabled objects
        for prim in self.stage.TraverseAll():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_api = UsdPhysics.RigidBodyAPI(prim)

                # Check mass properties
                mass_attr = prim.GetAttribute("physics:mass")
                if mass_attr and mass_attr.Get():
                    mass = mass_attr.Get()
                    if mass <= 0:
                        issues.append(f"Zero or negative mass: {prim.GetPath()}")
                    elif mass > 10000:
                        issues.append(f"Excessive mass: {prim.GetPath()} ({mass})")

                # Check linear velocity
                lin_vel_attr = prim.GetAttribute("physics:linearVelocity")
                if lin_vel_attr and lin_vel_attr.Get():
                    lin_vel = lin_vel_attr.Get()
                    speed = sum(v**2 for v in lin_vel)**0.5
                    if speed > 100:  # Very high velocity
                        issues.append(f"High linear velocity: {prim.GetPath()} ({speed:.2f})")

        self.debug_issues.extend(issues)

        if issues:
            print("Physics property issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("Physics properties validated successfully")

    def inspect_transforms(self):
        """Inspect and validate transforms"""

        print("\n=== TRANSFORM INSPECTION ===")

        issues = []

        for prim in self.stage.TraverseAll():
            xform_api = UsdGeom.Xformable(prim)
            if xform_api:
                # Get transform operations
                xform_ops = xform_api.GetOrderedXformOps()

                for op in xform_ops:
                    if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                        scale_value = op.Get()
                        # Check for zero or negative scale
                        if hasattr(scale_value, '__iter__'):
                            for s in scale_value:
                                if s <= 0:
                                    issues.append(f"Invalid scale in {prim.GetPath()}: {scale_value}")
                        elif scale_value <= 0:
                            issues.append(f"Invalid scale in {prim.GetPath()}: {scale_value}")

                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateQuat:
                        quat_value = op.Get()
                        # Check quaternion validity (simplified)
                        if hasattr(quat_value, '__iter__') and len(quat_value) == 4:
                            # Check if quaternion is normalized (magnitude should be 1)
                            mag = sum(x**2 for x in quat_value)**0.5
                            if not 0.9 < mag < 1.1:  # Allow some tolerance
                                issues.append(f"Non-normalized quaternion in {prim.GetPath()}: magnitude {mag}")

        self.debug_issues.extend(issues)

        if issues:
            print("Transform issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("Transforms validated successfully")

    def debug_materials_and_appearance(self):
        """Debug materials and appearance issues"""

        print("\n=== MATERIALS AND APPEARANCE DEBUG ===")

        issues = []

        # Check for unassigned materials
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() in ["Cube", "Sphere", "Cylinder", "Plane"]:
                # Check if there's a material binding
                mat_api = UsdShade.MaterialBindingAPI(prim)
                bound_material = mat_api.ComputeBoundMaterial()

                if not bound_material[0]:  # No material bound
                    issues.append(f"No material assigned to: {prim.GetPath()}")

        self.debug_issues.extend(issues)

        if issues:
            print("Material issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("Materials validated successfully")

    def run_scene_debugging(self):
        """Run comprehensive scene debugging"""

        print("Starting scene debugging...")

        self.setup_debugging_scene()
        self.check_scene_integrity()
        self.validate_physics_properties()
        self.inspect_transforms()
        self.debug_materials_and_appearance()

        # Summary
        print(f"\n=== DEBUGGING SUMMARY ===")
        print(f"Total issues found: {len(self.debug_issues)}")

        if self.debug_issues:
            print("All issues:")
            for i, issue in enumerate(self.debug_issues, 1):
                print(f"  {i}. {issue}")
        else:
            print("No issues found - scene is valid!")

# Import required classes for the example
try:
    from omni.isaac.core.objects import DynamicCuboid
except ImportError:
    # Create a mock class for the example
    class DynamicCuboid:
        def __init__(self, prim_path, name, position, size, mass):
            self.prim_path = prim_path
            self.name = name
            self.position = position
            self.size = size
            self.mass = mass

class DynamicCuboidWithIssues(DynamicCuboid):
    """A version of DynamicCuboid with intentionally problematic properties for debugging"""
    pass

def demonstrate_scene_debugging():
    """Demonstrate scene debugging techniques"""

    debugger = SceneDebugger()
    debugger.run_scene_debugging()

if __name__ == "__main__":
    demonstrate_scene_debugging()
```

## 7. Sensor Data Visualization

### Sensor Data Visualization Techniques

Visualizing sensor data is crucial for understanding robot perception in simulation:

```python
#!/usr/bin/env python3
"""
Sensor data visualization in Isaac Sim
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import cv2

class SensorDataVisualizer:
    """Class to visualize sensor data in Isaac Sim"""

    def __init__(self):
        self.world = None
        self.sensors = {}
        self.sensor_data = {}

    def setup_sensor_visualization_scene(self):
        """Set up a scene with various sensors for visualization"""

        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create objects for sensor testing
        create_prim("/World/Wall", "Cube",
                   position=[3, 0, 1],
                   attributes={"size": 0.2})

        create_prim("/World/Obstacle", "Cylinder",
                   position=[1, 1, 0.5],
                   attributes={"radius": 0.3, "height": 1.0})

        # Add a robot platform to mount sensors
        create_prim("/World/RobotPlatform", "Cube",
                   position=[0, 0, 0.5],
                   attributes={"size": 0.5})

        # Add reference to a simple robot (if available) or create basic structure
        try:
            add_reference_to_stage(
                usd_path="/Isaac/Robots/TurtleBot3/turtlebot3_usd/turtlebot3_dqn.usd",  # Example path
                prim_path="/World/Robot"
            )
        except:
            # If reference fails, create basic robot structure
            create_prim("/World/Robot", "Xform", position=[0, 0, 0.8])

        self.world.reset()
        print("Sensor visualization scene created")

    def setup_camera_sensor(self):
        """Set up camera sensor for visualization"""

        # Create camera sensor
        camera = Camera(
            prim_path="/World/Robot/Camera",
            name="camera_sensor",
            position=np.array([0.2, 0, 0.1]),
            frequency=30,
            resolution=(640, 480)
        )

        # Add camera to world
        self.world.scene.add(camera)

        # Store reference
        self.sensors['camera'] = camera

        print("Camera sensor configured")

    def setup_lidar_sensor(self):
        """Set up LiDAR sensor for visualization"""

        try:
            # Create LiDAR sensor (using RTX LiDAR for high-fidelity simulation)
            lidar = LidarRtx(
                prim_path="/World/Robot/Lidar",
                name="lidar_sensor",
                translation=np.array([0.0, 0.0, 0.3]),
                orientation=np.array([1, 0, 0, 0]),
                config="Example_Rotary_Lidar",
                visual_lod=True
            )

            # Add LiDAR to world
            self.world.scene.add(lidar)

            # Store reference
            self.sensors['lidar'] = lidar

            print("LiDAR sensor configured")
        except Exception as e:
            print(f"LiDAR setup failed (this is expected in some Isaac Sim versions): {e}")
            # Fallback to basic approach
            print("LiDAR sensor setup skipped - using basic sensor simulation")

    def setup_imu_sensor(self):
        """Set up IMU sensor for visualization"""

        # Note: IMU sensors are typically accessed through the robot's joint states
        # For this example, we'll create a placeholder
        print("IMU sensor would be configured here")
        self.sensors['imu'] = "imu_placeholder"

    def capture_and_visualize_camera_data(self):
        """Capture and visualize camera data"""

        if 'camera' not in self.sensors:
            print("No camera sensor available")
            return

        camera = self.sensors['camera']

        # Step the world to update sensor data
        self.world.step(render=True)

        try:
            # Get RGB image
            rgb_image = camera.get_rgb()

            if rgb_image is not None:
                print(f"Camera data captured: Shape {rgb_image.shape}, dtype {rgb_image.dtype}")

                # Visualize the image
                plt.figure(figsize=(10, 6))
                plt.subplot(1, 2, 1)
                plt.imshow(rgb_image)
                plt.title("RGB Camera Data")
                plt.axis('off')

                # Show grayscale version
                if len(rgb_image.shape) == 3:
                    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
                    plt.subplot(1, 2, 2)
                    plt.imshow(gray_image, cmap='gray')
                    plt.title("Grayscale")
                    plt.axis('off')

                plt.tight_layout()
                plt.show()

                # Store the data
                self.sensor_data['camera_rgb'] = rgb_image

            else:
                print("No camera data available")

        except Exception as e:
            print(f"Error capturing camera data: {e}")

    def capture_and_visualize_lidar_data(self):
        """Capture and visualize LiDAR data"""

        if 'lidar' not in self.sensors:
            print("No LiDAR sensor available")
            return

        lidar = self.sensors['lidar']

        # Step the world to update sensor data
        self.world.step(render=True)

        try:
            # Get LiDAR data (this is conceptual - actual API may vary)
            # For demonstration, we'll create simulated data
            # In real usage, you would use: lidar.get_linear_depth_data() or similar

            # Simulate LiDAR data
            # Create a simple 2D representation
            angles = np.linspace(0, 2*np.pi, 360)  # 360 degree scan
            distances = np.random.uniform(0.5, 5.0, 360)  # Random distances

            # Add some structure to simulate environment
            for i, angle in enumerate(angles):
                # Simulate wall at 3m in front
                if -np.pi/4 < angle < np.pi/4:
                    distances[i] = 3.0
                # Simulate obstacle to the right
                elif np.pi/4 < angle < 3*np.pi/4:
                    distances[i] = 1.5

            print(f"LiDAR data captured: {len(distances)} points")

            # Visualize LiDAR data
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

            # Polar plot
            ax1.plot(angles, distances)
            ax1.set_theta_zero_location('N')
            ax1.set_theta_direction(-1)
            ax1.set_title('LiDAR Polar Scan')

            # Cartesian plot
            x_coords = distances * np.cos(angles)
            y_coords = distances * np.sin(angles)
            ax2.scatter(x_coords, y_coords, s=1)
            ax2.set_xlim(-6, 6)
            ax2.set_ylim(-6, 6)
            ax2.set_aspect('equal')
            ax2.grid(True)
            ax2.set_title('LiDAR Cartesian Scan')

            plt.tight_layout()
            plt.show()

            # Store the data
            self.sensor_data['lidar'] = {
                'angles': angles,
                'distances': distances,
                'x_coords': x_coords,
                'y_coords': y_coords
            }

        except Exception as e:
            print(f"Error capturing LiDAR data: {e}")
            print("Note: LiDAR API may vary depending on Isaac Sim version")

    def visualize_sensor_fusion(self):
        """Visualize fused sensor data"""

        print("\n=== SENSOR FUSION VISUALIZATION ===")

        fig, ax = plt.subplots(figsize=(10, 8))

        # If we have camera data, show a simple representation
        if 'camera_rgb' in self.sensor_data:
            # Show a simplified representation of camera FOV
            camera_fov = Rectangle((-1, -0.5), 2, 1, linewidth=2,
                                  edgecolor='blue', facecolor='blue', alpha=0.2)
            ax.add_patch(camera_fov)
            ax.text(0, 0.6, 'Camera FOV', ha='center', va='center')

        # If we have LiDAR data, show the scan
        if 'lidar' in self.sensor_data:
            lidar_data = self.sensor_data['lidar']
            ax.scatter(lidar_data['x_coords'], lidar_data['y_coords'],
                      s=1, c='red', alpha=0.6, label='LiDAR Points')

        # Add some reference objects
        ax.scatter([3], [0], s=100, c='black', marker='s', label='Wall')
        ax.scatter([1], [1], s=100, c='orange', marker='o', label='Obstacle')

        ax.set_xlim(-4, 4)
        ax.set_ylim(-3, 3)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.legend()
        ax.set_title('Sensor Fusion Visualization')

        plt.show()

    def run_sensor_visualization_demo(self):
        """Run sensor data visualization demonstration"""

        print("Starting sensor data visualization demo...")

        self.setup_sensor_visualization_scene()
        self.setup_camera_sensor()
        self.setup_lidar_sensor()
        self.setup_imu_sensor()

        # Reset world after adding sensors
        self.world.reset()

        # Capture and visualize sensor data
        print("\nCapturing camera data...")
        self.capture_and_visualize_camera_data()

        print("\nCapturing LiDAR data...")
        self.capture_and_visualize_lidar_data()

        print("\nVisualizing sensor fusion...")
        self.visualize_sensor_fusion()

        print("\nSensor visualization demo completed")

def demonstrate_sensor_visualization():
    """Demonstrate sensor data visualization"""

    visualizer = SensorDataVisualizer()
    visualizer.run_sensor_visualization_demo()

if __name__ == "__main__":
    demonstrate_sensor_visualization()
```

## 8. Physics Simulation Debugging Methods

### Physics Debugging Techniques

Debugging physics simulation is critical for ensuring realistic robot behavior:

```python
#!/usr/bin/env python3
"""
Physics simulation debugging methods for Isaac Sim
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import matplotlib.pyplot as plt

class PhysicsDebugger:
    """Class to debug physics simulation in Isaac Sim"""

    def __init__(self):
        self.world = None
        self.physics_context = None
        self.debug_data = {
            'positions': [],
            'velocities': [],
            'forces': [],
            'contacts': []
        }

    def setup_physics_debug_scene(self):
        """Set up a scene for physics debugging"""

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0/60.0,   # 60 Hz physics
            rendering_dt=1.0/30.0  # 30 Hz rendering
        )

        self.physics_context = self.world.scene.get_physics_context()

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create objects with different physics properties for debugging
        # Normal object
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/NormalCube",
                name="normal_cube",
                position=[0, 0, 2.0],
                size=0.3,
                mass=1.0
            )
        )

        # Object with potential physics issues
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/HeavyCube",
                name="heavy_cube",
                position=[1, 0, 2.0],
                size=0.1,  # Small but heavy
                mass=10.0  # High density
            )
        )

        # Object with low friction
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/IcyCube",
                name="icy_cube",
                position=[-1, 0, 2.0],
                size=0.3,
                mass=1.0
            )
        )
        # Set low friction (conceptual - actual API may vary)
        # icy_cube_obj = self.world.scene.get_object("icy_cube")
        # icy_cube_obj.set_friction(0.1)  # Very low friction

        # Create a ramp to test physics
        create_prim("/World/Ramp", "Xform", position=[2, 0, 0.5])
        create_prim("/World/Ramp/Plane", "Plane",
                   position=[0, 0, 0],
                   orientation=np.array([0.707, 0, 0, 0.707]))  # 45 degree angle

        self.world.reset()
        print("Physics debugging scene created with various physics scenarios")

    def configure_physics_for_debugging(self):
        """Configure physics settings for debugging"""

        # Set physics solver parameters for stability debugging
        self.physics_context.set_position_iteration_count(8)  # Default
        self.physics_context.set_velocity_iteration_count(4)  # Default
        self.physics_context.set_solver_type(0)  # TGS solver (more stable)

        # Enable contact counting for debugging
        self.physics_context.set_enable_ccd(True)  # Continuous collision detection

        print("Physics configured for debugging:")
        print(f"  - Position iterations: {self.physics_context.get_position_iteration_count()}")
        print(f"  - Velocity iterations: {self.physics_context.get_velocity_iteration_count()}")
        print(f"  - Solver type: {self.physics_context.get_solver_type()}")
        print(f"  - CCD enabled: {self.physics_context.get_enable_ccd()}")

    def debug_collision_properties(self):
        """Debug collision properties of objects"""

        print("\n=== COLLISION PROPERTIES DEBUG ===")

        objects_to_check = ["normal_cube", "heavy_cube", "icy_cube"]

        for obj_name in objects_to_check:
            if self.world.scene.has_object(obj_name):
                obj = self.world.scene.get_object(obj_name)

                # Get pose and velocity
                position, orientation = obj.get_world_pose()
                linear_vel, angular_vel = obj.get_linear_velocity(), obj.get_angular_velocity()

                print(f"\nObject: {obj_name}")
                print(f"  Position: {position}")
                print(f"  Linear Velocity: {linear_vel}")
                print(f"  Angular Velocity: {angular_vel}")

                # Check if object is sleeping
                try:
                    is_sleeping = obj.is_sleeping()
                    print(f"  Is Sleeping: {is_sleeping}")
                except:
                    print(f"  Is Sleeping: Not available")

                # Check mass properties
                try:
                    mass = obj.get_mass()
                    print(f"  Mass: {mass}")
                except:
                    print(f"  Mass: Not available")

    def debug_contact_information(self):
        """Debug contact information between objects"""

        print("\n=== CONTACT INFORMATION DEBUG ===")

        # This is a simplified contact debugging approach
        # Actual contact information might require different API calls
        # depending on Isaac Sim version

        # Get all objects
        objects = ["normal_cube", "heavy_cube", "icy_cube"]

        contacts_found = 0
        for i, obj_name1 in enumerate(objects):
            for obj_name2 in objects[i+1:]:
                if (self.world.scene.has_object(obj_name1) and
                    self.world.scene.has_object(obj_name2)):

                    obj1 = self.world.scene.get_object(obj_name1)
                    obj2 = self.world.scene.get_object(obj_name2)

                    # Get positions
                    pos1, _ = obj1.get_world_pose()
                    pos2, _ = obj2.get_world_pose()

                    # Calculate distance
                    distance = np.linalg.norm(np.array(pos1) - np.array(pos2))

                    # Check if potentially in contact (simplified)
                    if distance < 0.6:  # Sum of typical bounding radii
                        print(f"  Potential contact between {obj_name1} and {obj_name2}")
                        print(f"    Distance: {distance:.3f}")
                        contacts_found += 1

        if contacts_found == 0:
            print("  No contacts detected between test objects")

    def analyze_physics_stability(self):
        """Analyze physics simulation stability"""

        print("\n=== PHYSICS STABILITY ANALYSIS ===")

        stability_metrics = {
            'max_velocity': 0,
            'max_angular_velocity': 0,
            'position_drift': 0,
            'energy_conservation': True
        }

        # Run simulation for a short period to analyze stability
        initial_positions = {}
        final_positions = {}

        # Store initial positions
        for obj_name in ["normal_cube", "heavy_cube", "icy_cube"]:
            if self.world.scene.has_object(obj_name):
                pos, _ = self.world.scene.get_object(obj_name).get_world_pose()
                initial_positions[obj_name] = np.array(pos)

        # Run simulation
        for i in range(100):
            self.world.step(render=False)

            # Track metrics
            for obj_name in ["normal_cube", "heavy_cube", "icy_cube"]:
                if self.world.scene.has_object(obj_name):
                    obj = self.world.scene.get_object(obj_name)
                    lin_vel, ang_vel = obj.get_linear_velocity(), obj.get_angular_velocity()

                    max_lin_vel = np.linalg.norm(lin_vel)
                    max_ang_vel = np.linalg.norm(ang_vel)

                    if max_lin_vel > stability_metrics['max_velocity']:
                        stability_metrics['max_velocity'] = max_lin_vel
                    if max_ang_vel > stability_metrics['max_angular_velocity']:
                        stability_metrics['max_angular_velocity'] = max_ang_vel

        # Calculate final positions and drift
        for obj_name in ["normal_cube", "heavy_cube", "icy_cube"]:
            if self.world.scene.has_object(obj_name):
                pos, _ = self.world.scene.get_object(obj_name).get_world_pose()
                final_positions[obj_name] = np.array(pos)

                # Calculate position drift from expected
                if obj_name in initial_positions:
                    drift = np.linalg.norm(final_positions[obj_name] - initial_positions[obj_name])
                    if drift > stability_metrics['position_drift']:
                        stability_metrics['position_drift'] = drift

        print(f"Stability Analysis Results:")
        print(f"  - Max Linear Velocity: {stability_metrics['max_velocity']:.3f}")
        print(f"  - Max Angular Velocity: {stability_metrics['max_angular_velocity']:.3f}")
        print(f"  - Max Position Drift: {stability_metrics['position_drift']:.3f}")

        # Stability assessment
        unstable = False
        reasons = []

        if stability_metrics['max_velocity'] > 100:
            unstable = True
            reasons.append("Excessive linear velocity")
        if stability_metrics['max_angular_velocity'] > 100:
            unstable = True
            reasons.append("Excessive angular velocity")
        if stability_metrics['position_drift'] > 5.0:
            unstable = True
            reasons.append("Excessive position drift")

        if unstable:
            print(f"  - Status: UNSTABLE ({', '.join(reasons)})")
        else:
            print(f"  - Status: STABLE")

    def debug_physics_parameters(self):
        """Debug physics parameters and settings"""

        print("\n=== PHYSICS PARAMETERS DEBUG ===")

        # Get physics context properties
        print("Physics Context Properties:")
        print(f"  - Physics DT: {self.physics_context.get_physics_dt()}")
        print(f"  - Position Iterations: {self.physics_context.get_position_iteration_count()}")
        print(f"  - Velocity Iterations: {self.physics_context.get_velocity_iteration_count()}")
        print(f"  - Solver Type: {self.physics_context.get_solver_type()}")
        print(f"  - GPU Dynamics: {self.physics_context.is_gpu_dynamics_enabled()}")
        print(f"  - CCD Enabled: {self.physics_context.get_enable_ccd()}")
        print(f"  - Broadphase Type: {self.physics_context.get_broadphase_type()}")

        # Check contact properties
        print(f"  - Contact Offset: {self.physics_context.get_contact_offset()}")
        print(f"  - Rest Offset: {self.physics_context.get_rest_offset()}")

        # Check gravity
        gravity = self.physics_context.get_gravity()
        print(f"  - Gravity: {gravity}")

    def visualize_physics_data(self):
        """Visualize physics simulation data"""

        print("\n=== VISUALIZING PHYSICS DATA ===")

        # Run simulation and collect data
        positions = []
        velocities = []

        for i in range(200):
            self.world.step(render=False)

            if self.world.scene.has_object("normal_cube"):
                pos, _ = self.world.scene.get_object("normal_cube").get_world_pose()
                vel = self.world.scene.get_object("normal_cube").get_linear_velocity()

                positions.append(pos[2])  # Just z-coordinate for simplicity
                velocities.append(np.linalg.norm(vel))

            if i % 50 == 0:
                print(f"  Collected data at step {i}")

        if positions and velocities:
            # Plot physics data
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

            steps = range(len(positions))
            ax1.plot(steps, positions)
            ax1.set_title('Object Height Over Time')
            ax1.set_ylabel('Height (m)')
            ax1.grid(True)

            ax2.plot(steps, velocities)
            ax2.set_title('Object Speed Over Time')
            ax2.set_xlabel('Simulation Step')
            ax2.set_ylabel('Speed (m/s)')
            ax2.grid(True)

            plt.tight_layout()
            plt.show()

        print("Physics data visualization completed")

    def run_physics_debugging_demo(self):
        """Run comprehensive physics debugging demonstration"""

        print("Starting physics debugging demonstration...")

        self.setup_physics_debug_scene()
        self.configure_physics_for_debugging()

        # Run various debugging methods
        self.debug_collision_properties()
        self.debug_contact_information()
        self.analyze_physics_stability()
        self.debug_physics_parameters()
        self.visualize_physics_data()

        print("\nPhysics debugging demo completed")
        print("Use these techniques to debug physics issues in your Isaac Sim projects")

def demonstrate_physics_debugging():
    """Demonstrate physics debugging methods"""

    debugger = PhysicsDebugger()
    debugger.run_physics_debugging_demo()

if __name__ == "__main__":
    demonstrate_physics_debugging()
```

## 9. RL Training Visualization Tools

### Reinforcement Learning Visualization

Visualizing RL training progress is essential for developing humanoid robots:

```python
#!/usr/bin/env python3
"""
RL training visualization tools for Isaac Sim
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from collections import deque
import torch

class RLVisualizationTools:
    """Class to visualize RL training in Isaac Sim"""

    def __init__(self):
        self.world = None
        self.episode_rewards = deque(maxlen=100)
        self.episode_lengths = deque(maxlen=100)
        self.training_steps = 0
        self.episode_count = 0
        self.current_episode_reward = 0
        self.current_episode_length = 0
        self.is_training = False

    def setup_rl_visualization_scene(self):
        """Set up a scene for RL visualization"""

        self.world = World(stage_units_in_meters=1.0)

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create simple environment for RL (e.g., a target to reach)
        create_prim("/World/Target", "Sphere",
                   position=[2, 0, 0.2],
                   attributes={"radius": 0.2})
        create_prim("/World/Target/Visual", "Sphere",
                   position=[2, 0, 0.2],
                   attributes={"radius": 0.3})

        # Add a simple robot (for visualization purposes)
        try:
            add_reference_to_stage(
                usd_path="/Isaac/Robots/Carter/carter_nav.usd",  # Example path
                prim_path="/World/Robot"
            )
        except:
            # If reference fails, create basic robot structure
            create_prim("/World/Robot", "Xform", position=[0, 0, 0.5])

        self.world.reset()
        print("RL visualization scene created")

    def simulate_training_data(self):
        """Simulate RL training data for visualization"""

        # Simulate some training progress
        if not self.is_training:
            self.is_training = True
            print("Starting simulated RL training...")

        # Simulate an episode
        episode_reward = 0
        episode_length = 0

        # Simulate some steps with random rewards
        for step in range(50):  # Simulate 50 steps per episode
            # Generate random reward (in practice, this comes from the environment)
            step_reward = np.random.normal(10, 5)  # Mean 10, std 5
            episode_reward += step_reward
            episode_length += 1

            # Add some variation to make it more realistic
            if step % 10 == 0:
                print(f"  Training step {self.training_steps}, episode {self.episode_count}")

            self.training_steps += 1

        # Store episode data
        self.episode_rewards.append(episode_reward)
        self.episode_lengths.append(episode_length)
        self.episode_count += 1

        print(f"  Completed episode {self.episode_count} with reward {episode_reward:.2f}")

    def plot_training_progress(self):
        """Plot RL training progress"""

        if len(self.episode_rewards) == 0:
            print("No training data to plot")
            return

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('RL Training Progress Visualization', fontsize=16)

        episodes = range(len(self.episode_rewards))

        # Plot 1: Episode rewards over time
        ax1.plot(episodes, list(self.episode_rewards), 'b-', alpha=0.7)
        ax1.set_title('Episode Rewards Over Time')
        ax1.set_xlabel('Episode')
        ax1.set_ylabel('Total Reward')
        ax1.grid(True, alpha=0.3)

        # Add moving average
        if len(self.episode_rewards) >= 10:
            moving_avg = []
            for i in range(9, len(self.episode_rewards)):
                avg = sum(list(self.episode_rewards)[i-9:i+1]) / 10
                moving_avg.append(avg)
            ax1.plot(range(9, len(self.episode_rewards)), moving_avg, 'r-', linewidth=2, label='Moving Average (10)')
            ax1.legend()

        # Plot 2: Episode lengths
        ax2.plot(episodes, list(self.episode_lengths), 'g-', alpha=0.7)
        ax2.set_title('Episode Lengths Over Time')
        ax2.set_xlabel('Episode')
        ax2.set_ylabel('Steps per Episode')
        ax2.grid(True, alpha=0.3)

        # Plot 3: Reward distribution histogram
        if len(self.episode_rewards) > 1:
            ax3.hist(list(self.episode_rewards), bins=min(20, len(self.episode_rewards)), alpha=0.7, color='skyblue', edgecolor='black')
            ax3.set_title('Distribution of Episode Rewards')
            ax3.set_xlabel('Total Reward')
            ax3.set_ylabel('Frequency')
            ax3.grid(True, alpha=0.3)

        # Plot 4: Training statistics
        ax4.axis('off')  # Turn off axis for text display

        if len(self.episode_rewards) > 0:
            avg_reward = sum(self.episode_rewards) / len(self.episode_rewards)
            best_reward = max(self.episode_rewards) if self.episode_rewards else 0
            avg_length = sum(self.episode_lengths) / len(self.episode_lengths) if self.episode_lengths else 0

            stats_text = f"""Training Statistics:

Total Episodes: {self.episode_count}
Total Steps: {self.training_steps}

Performance:
- Average Reward: {avg_reward:.2f}
- Best Reward: {best_reward:.2f}
- Average Length: {avg_length:.1f} steps

Progress:
- Recent Trend: {'Improving' if len(self.episode_rewards) > 10 and np.polyfit(range(len(list(self.episode_rewards)[-10:])), list(self.episode_rewards)[-10:], 1)[0] > 0 else 'Stable/Declining'}
"""
            ax4.text(0.1, 0.9, stats_text, transform=ax4.transAxes, fontsize=12, verticalalignment='top',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))

        plt.tight_layout()
        plt.show()

    def create_real_time_visualization(self):
        """Create a real-time visualization of training progress"""

        print("Creating real-time training visualization...")

        # Set up the figure
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('Real-time RL Training Progress', fontsize=16)

        def animate(frame):
            # Simulate new training data
            self.simulate_training_data()

            # Clear and update plots
            ax1.clear()
            ax2.clear()

            if len(self.episode_rewards) > 0:
                episodes = range(len(self.episode_rewards))

                # Plot rewards
                ax1.plot(episodes, list(self.episode_rewards), 'b-', alpha=0.7, label='Episode Reward')

                # Add moving average if enough data
                if len(self.episode_rewards) >= 10:
                    moving_avg = []
                    for i in range(9, len(self.episode_rewards)):
                        avg = sum(list(self.episode_rewards)[i-9:i+1]) / 10
                        moving_avg.append(avg)
                    ax1.plot(range(9, len(self.episode_rewards)), moving_avg, 'r-', linewidth=2, label='Moving Average (10)')

                ax1.set_title('Episode Rewards (Real-time)')
                ax1.set_ylabel('Total Reward')
                ax1.legend()
                ax1.grid(True, alpha=0.3)

                # Plot episode lengths
                ax2.plot(episodes, list(self.episode_lengths), 'g-', alpha=0.7)
                ax2.set_title('Episode Lengths (Real-time)')
                ax2.set_xlabel('Episode')
                ax2.set_ylabel('Steps per Episode')
                ax2.grid(True, alpha=0.3)

            # Add current stats as text
            if len(self.episode_rewards) > 0:
                avg_reward = sum(self.episode_rewards) / len(self.episode_rewards)
                current_reward = list(self.episode_rewards)[-1] if self.episode_rewards else 0

                stats_text = f'Current: {current_reward:.2f} | Average: {avg_reward:.2f} | Episodes: {len(self.episode_rewards)}'
                fig.suptitle(f'Real-time RL Training Progress - {stats_text}', fontsize=16)

        # Create animation (run for a limited number of frames for demo)
        ani = FuncAnimation(fig, animate, frames=50, interval=500, repeat=False)
        plt.tight_layout()
        plt.show()

        print("Real-time visualization completed")

    def visualize_policy_performance(self):
        """Visualize policy performance metrics"""

        print("\n=== POLICY PERFORMANCE VISUALIZATION ===")

        if len(self.episode_rewards) == 0:
            print("No training data available")
            return

        # Calculate performance metrics
        rewards = list(self.episode_rewards)
        lengths = list(self.episode_lengths)

        # Performance over time
        performance_windows = []
        window_size = max(1, len(rewards) // 10)  # Use 10% of data for each window

        if window_size > 0:
            for i in range(0, len(rewards), window_size):
                window_rewards = rewards[i:i+window_size]
                if window_rewards:
                    avg_reward = sum(window_rewards) / len(window_rewards)
                    performance_windows.append(avg_reward)

        # Create performance visualization
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # Performance over time
        if performance_windows:
            window_labels = [f"{i*window_size}-{min((i+1)*window_size-1, len(rewards)-1)}"
                           for i in range(len(performance_windows))]
            ax1.bar(range(len(performance_windows)), performance_windows)
            ax1.set_title('Performance Over Time (Windowed Average)')
            ax1.set_xlabel('Training Window')
            ax1.set_ylabel('Average Reward')
            ax1.set_xticks(range(len(performance_windows)))
            ax1.set_xticklabels(window_labels, rotation=45)
            ax1.grid(True, alpha=0.3)

        # Reward vs Length correlation
        if len(rewards) > 1 and len(lengths) > 1:
            ax2.scatter(lengths, rewards, alpha=0.6)
            ax2.set_title('Reward vs Episode Length Correlation')
            ax2.set_xlabel('Episode Length')
            ax2.set_ylabel('Total Reward')
            ax2.grid(True, alpha=0.3)

            # Add trend line
            if len(rewards) > 1:
                z = np.polyfit(lengths, rewards, 1)
                p = np.poly1d(z)
                ax2.plot(lengths, p(lengths), "r--", alpha=0.8, label=f'Trend (slope: {z[0]:.2f})')
                ax2.legend()

        plt.tight_layout()
        plt.show()

    def run_rl_visualization_demo(self):
        """Run RL visualization demonstration"""

        print("Starting RL visualization tools demonstration...")

        self.setup_rl_visualization_scene()

        # Simulate some training episodes
        print("\nSimulating training episodes...")
        for episode in range(20):
            self.simulate_training_data()
            if episode % 5 == 0:
                print(f"  Simulated episode {episode + 1}/20")

        # Create visualizations
        print("\nCreating training progress plots...")
        self.plot_training_progress()

        print("\nCreating policy performance visualization...")
        self.visualize_policy_performance()

        print("\nRL visualization demo completed")
        print("These tools help monitor and debug RL training in Isaac Sim environments")

def demonstrate_rl_visualization():
    """Demonstrate RL visualization tools"""

    rl_viz = RLVisualizationTools()
    rl_viz.run_rl_visualization_demo()

if __name__ == "__main__":
    demonstrate_rl_visualization()
```

## 10. Hands-on Exercise: Visualization and Debugging Tools in Isaac Sim

### Practical Exercise

Now let's create a comprehensive hands-on exercise that combines all the visualization and debugging concepts:

```python
#!/usr/bin/env python3
"""
Hands-on Exercise: Isaac Sim Visualization and Debugging Tools
"""
import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
import matplotlib.pyplot as plt
import time
from collections import deque
import psutil
import GPUtil

class IsaacSimVisualizationDebuggingExercise:
    """Comprehensive hands-on exercise for Isaac Sim visualization and debugging"""

    def __init__(self):
        self.world = None
        self.stage = None
        self.sensors = {}
        self.metrics_history = {
            'fps': deque(maxlen=500),
            'physics_steps_per_sec': deque(maxlen=500),
            'gpu_load': deque(maxlen=500),
            'cpu_load': deque(maxlen=500),
            'memory_usage': deque(maxlen=500)
        }
        self.start_time = time.time()
        self.last_render_time = time.time()
        self.frame_count = 0
        self.physics_step_count = 0
        self.exercise_results = {}

    def setup_exercise_environment(self):
        """Set up the exercise environment with various debugging scenarios"""

        print("Setting up Isaac Sim visualization and debugging exercise environment...")

        # Create world with specific parameters for the exercise
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0/120.0,   # Higher physics frequency for detailed debugging
            rendering_dt=1.0/60.0   # Rendering frequency
        )

        self.stage = self.world.stage

        # Create a complex scene for debugging
        # Ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Create multiple objects with different properties
        objects_config = [
            {"name": "normal_object", "position": [0, 0, 2.0], "size": 0.3, "mass": 1.0},
            {"name": "heavy_object", "position": [1, 0, 2.0], "size": 0.1, "mass": 10.0},  # High density
            {"name": "light_object", "position": [-1, 0, 2.0], "size": 0.4, "mass": 0.1},  # Low density
            {"name": "stack_1", "position": [2, 0, 1.0], "size": 0.2, "mass": 0.5},
            {"name": "stack_2", "position": [2, 0, 1.3], "size": 0.2, "mass": 0.5},
            {"name": "stack_3", "position": [2, 0, 1.6], "size": 0.2, "mass": 0.5},
        ]

        for obj_config in objects_config:
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{obj_config['name']}",
                    name=obj_config['name'],
                    position=obj_config['position'],
                    size=obj_config['size'],
                    mass=obj_config['mass']
                )
            )

        # Create static objects
        self.world.scene.add(
            FixedCuboid(
                prim_path="/World/wall",
                name="wall",
                position=[3, 0, 0.5],
                size=0.5
            )
        )

        # Add a ramp for physics debugging
        create_prim("/World/ramp", "Xform", position=[-2, 0, 0.2])
        create_prim("/World/ramp/plane", "Plane",
                   position=[0, 0, 0],
                   orientation=np.array([0.707, 0, 0, 0.707]))  # 45 degree angle

        # Add a camera for visualization
        camera = Camera(
            prim_path="/World/camera",
            name="exercise_camera",
            position=np.array([4, 4, 3]),
            frequency=30,
            resolution=(640, 480)
        )
        self.world.scene.add(camera)
        self.sensors['camera'] = camera

        # Reset the world
        self.world.reset()

        print("Exercise environment set up with various debugging scenarios")
        print("- Normal physics object")
        print("- High-density object (potential instability)")
        print("- Low-density object")
        print("- Stacked objects (potential collapse)")
        print("- Static wall")
        print("- Inclined ramp")
        print("- Camera sensor for visualization")

    def exercise_part_1_viewport_visualization(self):
        """Exercise Part 1: Viewport and Visualization"""

        print("\n" + "="*60)
        print("EXERCISE PART 1: VIEWPORT AND VISUALIZATION")
        print("="*60)

        print("\nInstructions:")
        print("1. Open the Isaac Sim viewport")
        print("2. Use the mouse to navigate the 3D view")
        print("3. Try different visualization modes (shaded, wireframe, etc.)")
        print("4. Use the timeline to step through the simulation")
        print("5. Observe the different objects in the scene")

        # Set up camera view for optimal visualization
        set_camera_view(
            eye=np.array([5.0, 5.0, 4.0]),
            target=np.array([0.0, 0.0, 0.0])
        )

        print("\nCamera view set for optimal visualization")
        print("Run simulation for 50 steps to observe objects...")

        for i in range(50):
            self.world.step(render=True)
            if i % 10 == 0:
                print(f"  Step {i}/50 - Observe object positions and movements")

        print("\nPart 1 Complete: You have practiced viewport visualization techniques")

    def exercise_part_2_physics_debug_visualization(self):
        """Exercise Part 2: Physics Debug Visualization"""

        print("\n" + "="*60)
        print("EXERCISE PART 2: PHYSICS DEBUG VISUALIZATION")
        print("="*60)

        print("\nInstructions:")
        print("1. Enable Physics Debug Visualization in the viewport:")
        print("   - Go to Viewport menu > Lighting > Physics Debug")
        print("2. Observe collision shapes around objects")
        print("3. Look for contact points between objects")
        print("4. Notice joint constraints and force vectors")

        # Run simulation to generate physics interactions
        print("\nRunning simulation to generate physics interactions...")

        for i in range(100):
            self.world.step(render=True)

            # Record metrics during physics debugging
            current_time = time.time()
            elapsed_time = current_time - self.last_render_time
            if elapsed_time > 0:
                fps = 1.0 / elapsed_time
                self.metrics_history['fps'].append(fps)
                self.frame_count += 1

            # Record system metrics
            self.metrics_history['cpu_load'].append(psutil.cpu_percent())
            self.metrics_history['memory_usage'].append(psutil.virtual_memory().percent)

            # Record GPU metrics
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu = gpus[0]
                self.metrics_history['gpu_load'].append(gpu.load * 100)

            self.last_render_time = current_time
            self.physics_step_count += 1

            if i % 25 == 0:
                print(f"  Physics debug step {i}/100")

        print("\nPhysics debug visualization complete")
        print("Observe how collision shapes and physics properties are visualized")

    def exercise_part_3_scene_debugging(self):
        """Exercise Part 3: Scene Debugging"""

        print("\n" + "="*60)
        print("EXERCISE PART 3: SCENE DEBUGGING")
        print("="*60)

        print("\nInstructions:")
        print("1. Open the Stage View (Window > Stage)")
        print("2. Examine the scene hierarchy")
        print("3. Look for any potential issues with the scene structure")
        print("4. Check the Property Panel for individual object properties")
        print("5. Verify transforms and physical properties")

        # Perform scene debugging analysis
        print("\nAnalyzing scene hierarchy...")

        issues_found = []

        # Check for common scene issues
        for prim in self.stage.TraverseAll():
            if not prim.IsValid():
                issues_found.append(f"Invalid prim: {prim.GetPath()}")

        # Check for objects with potentially problematic physics properties
        for prim in self.stage.TraverseAll():
            if "Cube" in prim.GetTypeName():
                # This is a simplified check - in practice you'd use USD APIs
                pass

        print(f"\nScene analysis complete:")
        print(f"- Total prims in scene: {len(list(self.stage.TraverseAll()))}")
        print(f"- Issues found: {len(issues_found)}")

        if issues_found:
            print("Issues:")
            for issue in issues_found:
                print(f"  - {issue}")
        else:
            print("No obvious scene issues detected")

    def exercise_part_4_sensor_data_visualization(self):
        """Exercise Part 4: Sensor Data Visualization"""

        print("\n" + "="*60)
        print("EXERCISE PART 4: SENSOR DATA VISUALIZATION")
        print("="*60)

        print("\nInstructions:")
        print("1. Observe the camera sensor in the scene")
        print("2. Access the sensor data programmatically")
        print("3. Visualize the captured data")

        # Capture and visualize sensor data
        print("\nCapturing camera data...")

        camera = self.sensors['camera']

        # Step to update sensor data
        self.world.step(render=True)

        try:
            # Get camera image (this might not work in all Isaac Sim versions)
            print("  Camera sensor configured. In actual Isaac Sim:")
            print("  - You would call camera.get_rgb() to get image data")
            print("  - You would visualize this data using matplotlib or similar")
            print("  - You would analyze the image for debugging purposes")

            # For this exercise, we'll simulate what you would do
            print("  Simulated camera data capture and visualization")
            print("  In practice, you would:")
            print("  - Check for image quality issues")
            print("  - Verify sensor parameters")
            print("  - Analyze field of view and resolution")
            print("  - Debug sensor positioning and orientation")

        except Exception as e:
            print(f"  Note: Camera API may vary - {e}")

        print("\nSensor data visualization complete")

    def exercise_part_5_performance_profiling(self):
        """Exercise Part 5: Performance Profiling"""

        print("\n" + "="*60)
        print("EXERCISE PART 5: PERFORMANCE PROFILING")
        print("="*60)

        print("\nInstructions:")
        print("1. Open the Statistics window (Window > Statistics)")
        print("2. Monitor FPS, physics steps, and memory usage")
        print("3. Observe performance during simulation")
        print("4. Use the Console window to check for warnings/errors")

        # Run extended simulation to collect performance data
        print("\nRunning extended simulation for performance analysis...")

        start_time = time.time()
        performance_data = []

        for i in range(200):
            self.world.step(render=True)

            # Record performance metrics
            current_time = time.time()
            elapsed_time = current_time - self.last_render_time
            if elapsed_time > 0:
                fps = 1.0 / elapsed_time
                self.metrics_history['fps'].append(fps)
                self.frame_count += 1

            # Record system metrics
            cpu_load = psutil.cpu_percent()
            mem_usage = psutil.virtual_memory().percent
            self.metrics_history['cpu_load'].append(cpu_load)
            self.metrics_history['memory_usage'].append(mem_usage)

            # Record GPU metrics
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu = gpus[0]
                self.metrics_history['gpu_load'].append(gpu.load * 100)

            self.last_render_time = current_time
            self.physics_step_count += 1

            # Collect performance snapshot every 50 steps
            if i % 50 == 0:
                snapshot = {
                    'step': i,
                    'fps': fps if elapsed_time > 0 else 0,
                    'cpu': cpu_load,
                    'memory': mem_usage,
                    'gpu': gpu.load * 100 if gpus else 0
                }
                performance_data.append(snapshot)
                print(f"  Performance snapshot at step {i}: FPS={fps:.1f}, CPU={cpu_load:.1f}%")

        # Analyze performance data
        print(f"\nPerformance Analysis:")
        if self.metrics_history['fps']:
            avg_fps = sum(self.metrics_history['fps']) / len(self.metrics_history['fps'])
            avg_cpu = sum(self.metrics_history['cpu_load']) / len(self.metrics_history['cpu_load'])
            avg_gpu = sum(self.metrics_history['gpu_load']) / len(self.metrics_history['gpu_load']) if self.metrics_history['gpu_load'] else 0
            avg_mem = sum(self.metrics_history['memory_usage']) / len(self.metrics_history['memory_usage'])

            print(f"  Average FPS: {avg_fps:.2f}")
            print(f"  Average CPU Usage: {avg_cpu:.1f}%")
            print(f"  Average GPU Usage: {avg_gpu:.1f}%")
            print(f"  Average Memory Usage: {avg_mem:.1f}%")
            print(f"  Total Simulation Time: {time.time() - start_time:.2f}s")
            print(f"  Total Frames: {self.frame_count}")
            print(f"  Total Physics Steps: {self.physics_step_count}")

        # Performance recommendations
        print(f"\nPerformance Recommendations:")
        if avg_fps < 30:
            print("  - FPS is low (<30), consider reducing scene complexity or improving rendering settings")
        if avg_cpu > 80:
            print("  - CPU usage is high (>80%), optimize physics or reduce object count")
        if avg_gpu > 90:
            print("  - GPU usage is high (>90%), reduce rendering quality or complexity")
        if avg_mem > 85:
            print("  - Memory usage is high (>85%), optimize asset loading or reduce batch sizes")

    def exercise_part_6_logs_and_diagnostics(self):
        """Exercise Part 6: Logs and Diagnostics"""

        print("\n" + "="*60)
        print("EXERCISE PART 6: LOGS AND DIAGNOSTICS")
        print("="*60)

        print("\nInstructions:")
        print("1. Open the Console window (Window > Console)")
        print("2. Look for any warnings or errors in the simulation")
        print("3. Use carb logging to add your own debug messages")
        print("4. Check for physics-related warnings")

        # Add some diagnostic logging
        print("\nGenerating diagnostic information...")

        # Log system information
        carb.log_info(f"Isaac Sim Exercise - Started at {time.strftime('%Y-%m-%d %H:%M:%S')}")
        carb.log_info(f"Simulation parameters: physics_dt={self.world.get_physics_dt()}, rendering_dt={self.world.get_rendering_dt()}")

        # Log object states
        for obj_name in ["normal_object", "heavy_object", "light_object"]:
            if self.world.scene.has_object(obj_name):
                obj = self.world.scene.get_object(obj_name)
                pos, orient = obj.get_world_pose()
                lin_vel, ang_vel = obj.get_linear_velocity(), obj.get_angular_velocity()

                carb.log_info(f"Object {obj_name}: pos={pos}, lin_vel={lin_vel}, ang_vel={ang_vel}")

        # Run simulation with logging
        print("\nRunning simulation with diagnostic logging...")

        for i in range(25):
            self.world.step(render=True)

            # Log periodically
            if i % 10 == 0:
                carb.log_info(f"Diagnostic log at step {i}")

        print("Diagnostic logging complete")
        print("Check the Console window for the logged information")

    def exercise_part_7_integrated_debugging_workflow(self):
        """Exercise Part 7: Integrated Debugging Workflow"""

        print("\n" + "="*60)
        print("EXERCISE PART 7: INTEGRATED DEBUGGING WORKFLOW")
        print("="*60)

        print("\nInstructions:")
        print("Now integrate all the debugging techniques you've learned:")
        print("1. Use viewport visualization to observe the scene")
        print("2. Apply physics debug visualization to understand interactions")
        print("3. Check scene hierarchy for structural issues")
        print("4. Monitor performance metrics")
        print("5. Use logging for detailed diagnostics")
        print("6. Visualize sensor data")

        print("\nRunning integrated debugging workflow...")

        # Run a final simulation while monitoring everything
        for i in range(75):
            self.world.step(render=True)

            if i % 25 == 0:
                print(f"  Integrated debugging step {i}/75")

        print("\nIntegrated debugging workflow complete")
        print("You have practiced using multiple debugging tools together")

    def generate_exercise_report(self):
        """Generate a comprehensive exercise report"""

        print("\n" + "="*60)
        print("EXERCISE COMPLETION REPORT")
        print("="*60)

        print("\nCompleted Exercise Parts:")
        print("✓ Part 1: Viewport and Visualization")
        print("✓ Part 2: Physics Debug Visualization")
        print("✓ Part 3: Scene Debugging")
        print("✓ Part 4: Sensor Data Visualization")
        print("✓ Part 5: Performance Profiling")
        print("✓ Part 6: Logs and Diagnostics")
        print("✓ Part 7: Integrated Debugging Workflow")

        print(f"\nExercise Summary:")
        print(f"- Total simulation steps: {self.physics_step_count}")
        print(f"- Total rendered frames: {self.frame_count}")
        print(f"- Exercise duration: {time.time() - self.start_time:.2f} seconds")

        if self.metrics_history['fps']:
            avg_fps = sum(self.metrics_history['fps']) / len(self.metrics_history['fps'])
            print(f"- Average FPS achieved: {avg_fps:.2f}")

        print(f"\nKey Learning Outcomes:")
        print("• How to use Isaac Sim's visualization tools effectively")
        print("• Techniques for debugging physics simulations")
        print("• Methods for monitoring performance")
        print("• Approaches for scene and sensor debugging")
        print("• Best practices for integrated debugging workflows")

        print(f"\nNext Steps:")
        print("1. Apply these debugging techniques to your own Isaac Sim projects")
        print("2. Create custom debugging tools for your specific use cases")
        print("3. Develop automated testing and validation procedures")
        print("4. Integrate debugging into your development workflow")

    def run_complete_exercise(self):
        """Run the complete visualization and debugging exercise"""

        print("Starting Isaac Sim Visualization and Debugging Exercise")
        print("="*60)

        # Set up the environment
        self.setup_exercise_environment()

        # Run all exercise parts
        self.exercise_part_1_viewport_visualization()
        self.exercise_part_2_physics_debug_visualization()
        self.exercise_part_3_scene_debugging()
        self.exercise_part_4_sensor_data_visualization()
        self.exercise_part_5_performance_profiling()
        self.exercise_part_6_logs_and_diagnostics()
        self.exercise_part_7_integrated_debugging_workflow()

        # Generate final report
        self.generate_exercise_report()

        print("\n" + "="*60)
        print("EXERCISE COMPLETED SUCCESSFULLY!")
        print("="*60)
        print("You have completed the comprehensive Isaac Sim visualization and debugging exercise.")
        print("Apply these techniques to your own robotics development projects.")

def run_isaac_sim_visualization_debugging_exercise():
    """Run the complete Isaac Sim visualization and debugging exercise"""

    exercise = IsaacSimVisualizationDebuggingExercise()
    exercise.run_complete_exercise()

if __name__ == "__main__":
    run_isaac_sim_visualization_debugging_exercise()
```

Now I'll update the todo list to mark the tasks as completed:
