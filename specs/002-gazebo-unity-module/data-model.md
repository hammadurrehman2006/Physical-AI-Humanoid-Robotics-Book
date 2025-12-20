# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

## Key Entities

### Simulation Environment
- **name**: String (required) - Unique identifier for the simulation environment
- **description**: String (optional) - Human-readable description of the environment
- **gazebo_version**: String (required) - Compatible Gazebo version
- **world_file**: String (required) - Path to the Gazebo world file
- **physics_properties**: Object (required) - Gravity, friction, and other physics parameters
- **models**: Array of Model references - Robot and object models in the environment
- **sensors**: Array of Sensor references - Sensors configured in the environment

### Robot Model
- **name**: String (required) - Unique identifier for the robot model
- **format**: Enum ['URDF', 'SDF'] (required) - Format of the robot description
- **file_path**: String (required) - Path to the robot description file
- **links**: Array of Link objects - Physical links of the robot
- **joints**: Array of Joint objects - Joints connecting the links
- **sensors**: Array of Sensor references - Sensors attached to the robot
- **actuators**: Array of Actuator references - Actuators for robot movement

### Link
- **name**: String (required) - Unique identifier for the link
- **visual**: Object (optional) - Visual properties (mesh, color, etc.)
- **collision**: Object (required) - Collision properties (shape, size)
- **inertial**: Object (required) - Mass, center of mass, and inertia properties
- **geometry**: Object (required) - Geometric shape and dimensions

### Joint
- **name**: String (required) - Unique identifier for the joint
- **type**: Enum ['revolute', 'prismatic', 'fixed', 'continuous', 'floating', 'planar'] (required) - Type of joint
- **parent**: String (required) - Name of parent link
- **child**: String (required) - Name of child link
- **axis**: Object (required) - Axis of rotation or translation
- **limits**: Object (optional) - Joint limits (position, velocity, effort)

### Sensor
- **name**: String (required) - Unique identifier for the sensor
- **type**: Enum ['lidar', 'camera', 'depth_camera', 'imu', 'gps', 'force_torque'] (required) - Type of sensor
- **parent_link**: String (required) - Link to which the sensor is attached
- **pose**: Object (required) - Position and orientation relative to parent link
- **parameters**: Object (required) - Sensor-specific parameters (range, resolution, etc.)
- **topic**: String (required) - ROS topic for sensor data output

### Unity Scene
- **name**: String (required) - Unique identifier for the Unity scene
- **file_path**: String (required) - Path to the .unity scene file
- **gazebo_equivalent**: String (optional) - Corresponding Gazebo world file
- **robot_models**: Array of Robot Model references - Robots included in the scene
- **environment_objects**: Array of Object references - Static objects in the scene
- **visualization_settings**: Object (required) - Rendering and display settings

### Assessment Project
- **title**: String (required) - Name of the assessment project
- **description**: String (required) - Detailed description of the project
- **requirements**: Array of String (required) - Specific requirements for completion
- **evaluation_criteria**: Array of Object (required) - Criteria for grading/assessment
- **estimated_duration**: Number (required) - Time needed to complete in hours
- **prerequisites**: Array of String (required) - Prerequisites for starting the project
- **deliverables**: Array of String (required) - Required deliverables for submission

## State Transitions

### Simulation Environment States
- **created** → **configured**: When all models and sensors are added
- **configured** → **running**: When the simulation is started
- **running** → **paused**: When the simulation is paused
- **paused** → **running**: When the simulation is resumed
- **running** → **stopped**: When the simulation is terminated
- **configured** → **validated**: When the simulation passes validation tests

### Assessment Project States
- **planned** → **in_progress**: When learner starts working on the project
- **in_progress** → **completed**: When learner finishes the project
- **completed** → **evaluated**: When project is graded by instructor/automated system
- **evaluated** → **archived**: When project record is archived