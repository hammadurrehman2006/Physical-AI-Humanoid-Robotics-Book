---
sidebar_position: 4
---

# Physics Debugging and Validation Techniques

This tutorial covers essential techniques for debugging physics simulations and validating that your robot models behave realistically in Gazebo.

## Physics Validation Fundamentals

### Understanding Physics Simulation

Physics simulation in Gazebo involves:
- **Collision Detection**: Determining when objects intersect
- **Contact Resolution**: Calculating forces when objects touch
- **Integration**: Updating positions and velocities over time
- **Constraints**: Maintaining joint relationships and limits

### Common Physics Issues

1. **Objects falling through surfaces** - Usually due to missing collision geometries or incorrect inertial properties
2. **Unstable simulations** - Often caused by improper time steps or unrealistic parameters
3. **Jittery movements** - Can result from high-frequency oscillations or numerical errors
4. **Penetration between objects** - May indicate insufficient solver iterations or bad mesh quality

## Validation Techniques

### 1. Visual Validation

Enable physics visualization to observe collision shapes and contact points:

```bash
# Launch Gazebo with collision visualization
gz sim --render-engine ogre2

# In the GUI, enable:
# - View -> Transparent Models
# - View -> Wireframe
# - View -> Contacts
```

### 2. Numerical Validation

#### Check Inertial Properties
```bash
# Verify that your URDF has proper inertial values
check_urdf your_robot.urdf

# The center of mass should be within the physical bounds of the link
# Inertia values should be positive and reasonable
```

#### Validate Physics Parameters
```xml
<!-- In your world file -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller for stability -->
  <real_time_factor>1.0</real_time_factor>  <!-- 1.0 for real-time -->
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>  <!-- Increase for stability -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

### 3. Behavioral Validation

#### Gravity Test
Create a simple test to verify gravity is working:
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

Launch and verify the sphere falls at approximately 9.8 m/s².

#### Collision Test
Test collision detection with a simple setup:
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

    <!-- Static obstacle -->
    <model name="wall">
      <pose>1 0 1 0 0 0</pose>
      <static>true</static>  <!-- Won't move -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.1 2 2</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Moving object -->
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

## Debugging Tools and Commands

### Gazebo Command Line Tools

```bash
# Check model states
gz model -m

# Get detailed model info
gz model -m your_model_name -i

# List all topics
gz topic -l

# Monitor physics updates
gz topic -e /world/default/stats
```

### Physics Parameter Tuning

#### Time Step Analysis
```bash
# Launch with verbose physics output
gz sim -v 4 your_world.sdf
```

#### Solver Parameter Adjustment
```xml
<physics type="ode">
  <!-- Start with conservative values -->
  <max_step_size>0.001</max_step_size>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- Increase if unstable -->
      <sor>1.0</sor>     <!-- Lower for more stability -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>     <!-- Constraint Force Mixing -->
      <erp>0.2</erp>     <!-- Error Reduction Parameter -->
    </constraints>
  </ode>
</physics>
```

### Inertial Property Validation

#### Calculating Proper Inertias
For common shapes:

**Box (mass m, dimensions x, y, z):**
- Ixx = m*(y² + z²)/12
- Iyy = m*(x² + z²)/12
- Izz = m*(x² + y²)/12

**Cylinder (mass m, radius r, length l):**
- Ixx = m*(3*r² + l²)/12
- Iyy = m*(3*r² + l²)/12
- Izz = m*r²/2

**Sphere (mass m, radius r):**
- Ixx = Iyy = Izz = 2*m*r²/5

### Example Validation Script

Create a Python script to validate physics behavior:

```python
#!/usr/bin/env python3
"""
Physics validation script for Gazebo simulations
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
        self.subscription  # prevent unused variable warning

        # Setup validation parameters
        self.gravity = 9.81  # m/s²
        self.test_start_time = self.get_clock().now()

    def listener_callback(self, msg):
        # Example: Validate that a free-falling object accelerates at g
        try:
            # Find the test object in the message
            obj_index = msg.name.index('falling_sphere::link')
            obj_position = msg.pose[obj_index].position
            obj_velocity = msg.twist[obj_index].linear

            # Calculate expected position for free fall
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.test_start_time).nanoseconds / 1e9

            expected_position = -0.5 * self.gravity * elapsed_time**2

            # Validate that actual position matches expected
            if abs(obj_position.z - expected_position) > 0.1:  # 10cm tolerance
                self.get_logger().warn(f'Gravity validation failed: expected {expected_position}, got {obj_position.z}')
            else:
                self.get_logger().info(f'Gravity validation passed: {obj_position.z} vs {expected_position}')

        except ValueError:
            # Object not found in message
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

## Common Debugging Scenarios

### Scenario 1: Robot Falling Through Ground

**Symptoms**: Robot falls through the ground plane
**Solutions**:
1. Check that the ground plane model is loaded
2. Verify collision geometries are defined for all links
3. Ensure mass values are positive
4. Check that inertial origin is within the collision geometry

### Scenario 2: Robot Jittering or Vibrating

**Symptoms**: Robot oscillates rapidly in place
**Solutions**:
1. Reduce `max_step_size` in physics settings
2. Increase solver iterations
3. Check for overlapping collision geometries
4. Verify joint limits and stiffness

### Scenario 3: Unstable Multi-Link Robot

**Symptoms**: Robot with multiple links becomes unstable
**Solutions**:
1. Start with a simpler model and add complexity gradually
2. Ensure all joints have proper limits
3. Check that inertial properties are realistic
4. Use fixed joints instead of very stiff revolute joints where appropriate

## Performance Optimization

### Physics Performance Tips

1. **Use simpler collision geometries**:
   - Replace complex meshes with boxes, spheres, and cylinders
   - Use `<approximate_as_box>true</approximate_as_box>` for meshes

2. **Optimize solver parameters**:
   - Start with conservative settings and optimize for performance
   - Balance accuracy vs. speed based on your needs

3. **Reduce update rates** where possible:
   - Lower physics update rate for less critical simulations
   - Use appropriate sensor update rates

### Example Optimized Physics Settings

```xml
<physics type="ode">
  <!-- For performance-critical applications -->
  <max_step_size>0.01</max_step_size>  <!-- Larger for speed -->
  <real_time_update_rate>100</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>20</iters>  <!-- Balance between stability and speed -->
    </solver>
  </ode>
</physics>
```

## Validation Checklist

Before considering your physics simulation complete:

- [ ] Objects rest stably on surfaces
- [ ] Gravity produces expected acceleration (9.8 m/s²)
- [ ] Collisions are detected and resolved properly
- [ ] Robot joints behave within expected limits
- [ ] No unexpected penetrations between objects
- [ ] Simulation runs at acceptable real-time factor
- [ ] Inertial properties are physically realistic
- [ ] Mass and center of mass are correctly positioned

## Troubleshooting Resources

### Common Error Messages

- **"ODE Message 3: body not finite"**: Usually indicates numerical instability or invalid parameters
- **"Contact not finite"**: Often caused by very small or zero mass values
- **"Joint limits exceeded"**: May indicate solver issues or bad joint parameters

### Getting Help

- Check the [Gazebo documentation](http://gazebosim.org/tutorials?cat=physics)
- Use `gz sdf -k` to validate your SDF files
- Test with simplified models to isolate issues

## Next Steps

After validating your physics simulation:

1. Continue to [Environment Modeling](./environment-modeling.md) to create complex simulation worlds
2. Explore [Sensor Simulation](../sensor-simulation/lidar-simulation.md) to add perception capabilities
3. Learn about [Unity Integration](../unity-integration/unity-setup.md) for high-fidelity visualization