# Isaac Sim میں تربیتی ماحول (Training Environments)

یہ سیکشن Isaac Sim میں ری انفورسمنٹ لرننگ کے لیے تربیتی ماحول بنانے اور ترتیب دینے کا احاطہ کرتا ہے، خاص طور پر ہیومنوائڈ روبوٹ کنٹرول کے لیے۔ تربیتی ماحول مضبوط پالیسیاں تیار کرنے کے لیے اہم ہیں جو سمولیشن سے حقیقت میں منتقل ہو سکتی ہیں۔

## ماحولیاتی ڈیزائن کے اصول

### موثر تربیتی ماحول بنانا

ایک اچھی طرح سے ڈیزائن کردہ تربیتی ماحول کو کئی کلیدی اصولوں میں توازن رکھنا چاہیے:

1. **تنوع**: پالیسی کی عمومیت (generalization) کو بہتر بنانے کے لیے مختلف منظرنامے شامل کریں۔
2. **پیشرفت**: سادہ شروع کریں اور آہستہ آہستہ پیچیدگی بڑھائیں۔
3. **حقیقت پسندی**: حقیقت پسندانہ طبیعیات اور سینسر ماڈلز شامل کریں۔
4. **حفاظت**: تربیت کے دوران محفوظ تلاش (exploration) کو یقینی بنائیں۔
5. **کارکردگی**: معیار کو قربان کیے بغیر تیز تربیت کے لیے بہتر بنائیں۔

### ماحولیاتی زمرے

تربیتی ماحول کو درج ذیل میں درجہ بندی کیا جا سکتا ہے:

- **بنیادی مہارتوں کے ماحول**: توازن جیسی بنیادی مہارتوں پر توجہ مرکوز کریں۔
- **ٹاسک مخصوص ماحول**: مخصوص کاموں جیسے چلنے یا ہیرا پھیری کو ہدف بنائیں۔
- **چیلنج ماحول**: پیچیدہ رکاوٹیں اور منظرنامے شامل کریں۔
- **منتقلی کے ماحول**: sim-to-real منتقلی کو جانچنے کے لیے ڈیزائن کیا گیا ہے۔

## بنیادی توازن کی تربیت کا ماحول

### اسٹیشنری بیلنس ماحول

```python
#!/usr/bin/env python3
"""
Basic stationary balance training environment
"""
import torch
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from pxr import Gf

class StationaryBalanceEnv:
    """Environment for training stationary balance control"""

    def __init__(self, num_envs=1, device="cuda", robot_name="A1", max_episode_length=500):
        self.num_envs = num_envs
        self.device = device
        self.robot_name = robot_name
        self.max_episode_length = max_episode_length

        # Create Isaac Sim world
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0/60.0,
            rendering_dt=1.0/60.0
        )

        # Environment parameters
        self.dt = 1/60.0
        self.action_scale = 0.5
        self.base_init_state = [0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 1.0]  # pos(xyz) + quat(wxyz)
        self.default_dof_pos = np.zeros(12)  # Default joint positions for A1 robot
        self.default_dof_vel = np.zeros(12)  # Default joint velocities

        # Reward parameters
        self.lin_vel_scale = 0.1
        self.ang_vel_scale = 0.1
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.1
        self.action_scale = 0.5
        self.cosmetic_factor = 0.5  # Bonus for good behavior

        # Robot dimensions
        self.base_mass = 10.0  # kg
        self.leg_mass = 1.0   # kg per leg

        # Initialize environment
        self._create_envs()
        self.world.reset()

        # Get robot information
        self.robot = Robot(
            prim_path=f"/World/envs/env_0/Robot_0",
            name=f"robot_0"
        )

        # Set up DOF properties
        self._setup_dof_properties()

        # Initialize buffers
        self.reset_idx = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        self.progress_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.randomize_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)

        # Get initial states
        self.base_quat = self.robot.get_world_quat()
        self.base_euler = torch_utils.quat_to_euler_xyz(self.base_quat)
        self.base_lin_vel = self.robot.get_linear_velocity()
        self.base_ang_vel = self.robot.get_angular_velocity()
        self.dof_pos = self.robot.get_joint_positions()
        self.dof_vel = self.robot.get_joint_velocities()

    def _create_envs(self):
        """Create multiple parallel environments"""

        print(f"Creating {self.num_envs} parallel environments for balance training...")

        # Create ground plane
        create_prim("/World/defaultGroundPlane", "Plane", position=[0, 0, 0])

        # Environment spacing
        spacing = 2.0
        env_lower = [-spacing, -spacing, 0.0]
        env_upper = [spacing, spacing, spacing]

        for i in range(self.num_envs):
            # Create environment
            env_path = f"/World/envs/env_{i}"
            env = self.world.scene.add_environment(prim_path=env_path)

            # Add robot to environment
            robot_path = f"{env_path}/Robot_{i}"
            self._add_robot_to_env(robot_path, i)

    def _add_robot_to_env(self, robot_path, env_id):
        """Add robot to a specific environment"""

        # Load robot asset
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            robot_usd_path = assets_root_path + "/Isaac/Robots/Unitree/A1/a1.usd"

            add_reference_to_stage(
                usd_path=robot_usd_path,
                prim_path=robot_path
            )

            print(f"Added robot to environment {env_id}")
        else:
            print("Could not find assets root path")

    def _setup_dof_properties(self):
        """Set up DOF (Degree of Freedom) properties"""

        # Get DOF properties
        robot_dof_props = self.robot.get_dof_properties()

        # Set drive mode and stiffness/damping
        for i in range(len(robot_dof_props)):
            robot_dof_props['driveMode'][i] = 1  # Position drive
            robot_dof_props['stiffness'][i] = 80.0
            robot_dof_props['damping'][i] = 1.0

        # Apply DOF properties
        self.robot.set_dof_properties(robot_dof_props)

    def reset(self, env_ids=None):
        """Reset the environment"""

        if env_ids is None:
            env_ids = self.reset_idx

        # Randomize initial states
        num_resets = len(env_ids)

        # Randomize base positions
        base_pos = torch.zeros((num_resets, 3), device=self.device)
        base_pos[:, 2] = 0.6  # Height

        # Randomize base orientations
        base_quat = torch.zeros((num_resets, 4), device=self.device)
        base_quat[:, 3] = 1.0  # Identity quaternion

        # Randomize DOF positions and velocities
        dof_pos = torch.zeros((num_resets, 12), device=self.device)
        dof_vel = torch.zeros((num_resets, 12), device=self.device)

        # Set the states
        self.robot.set_world_poses(positions=base_pos, orientations=base_quat, indices=env_ids)
        self.robot.set_joint_positions(positions=dof_pos, indices=env_ids)
        self.robot.set_joint_velocities(velocities=dof_vel, indices=env_ids)

        # Reset progress buffer
        self.progress_buf[env_ids] = 0

        # Get initial observations
        obs = self._compute_observations(env_ids)

        return obs

    def step(self, actions):
        """Execute one step in the environment"""

        # Apply actions to robots
        self._apply_actions(actions)

        # Step the world simulation
        self.world.step(render=True)

        # Compute observations
        obs = self._compute_observations()

        # Compute rewards
        rewards = self._compute_rewards()

        # Compute dones
        dones = self._compute_dones()

        # Update progress buffer
        self.progress_buf += 1

        # Reset environments that are done
        reset_env_ids = dones.nonzero(as_tuple=False).flatten()
        if len(reset_env_ids) > 0:
            self.reset(reset_env_ids)

        # Info dictionary
        info = {}

        return obs, rewards, dones, info

    def _apply_actions(self, actions):
        """Apply actions to the robots"""

        # Scale actions
        scaled_actions = actions * self.action_scale

        # Add default joint positions
        target_pos = self.default_dof_pos + scaled_actions.cpu().numpy()

        # Apply positions
        self.robot.set_joint_positions(positions=torch.from_numpy(target_pos).to(self.device))

    def _compute_observations(self, env_ids=None):
        """Compute observations for the environment"""

        # Get current states
        base_quat = self.robot.get_world_quat()
        base_euler = torch_utils.quat_to_euler_xyz(base_quat)
        base_lin_vel = self.robot.get_linear_velocity()
        base_ang_vel = self.robot.get_angular_velocity()
        dof_pos = self.robot.get_joint_positions()
        dof_vel = self.robot.get_joint_velocities()

        # Normalize observations
        obs = torch.cat([
            base_lin_vel * self.lin_vel_scale,           # 3
            base_ang_vel * self.ang_vel_scale,           # 3
            base_euler,                                  # 3
            dof_pos * self.dof_pos_scale,               # 12
            dof_vel * self.dof_vel_scale,               # 12
            self.default_dof_pos                        # 12 (desired positions)
        ], dim=-1)

        return obs

    def _compute_rewards(self):
        """Compute rewards for the current state"""

        # Get current states
        base_quat = self.robot.get_world_quat()
        base_euler = torch_utils.quat_to_euler_xyz(base_quat)
        base_lin_vel = self.robot.get_linear_velocity()
        base_ang_vel = self.robot.get_angular_velocity()
        dof_pos = self.robot.get_joint_positions()
        dof_vel = self.robot.get_joint_velocities()

        # Reward for staying upright
        up_reward = torch.cos(base_euler[:, 1])  # pitch
        up_reward = torch.clamp(up_reward, min=0.0, max=1.0)

        # Reward for low velocity (stability)
        vel_reward = torch.exp(-torch.sqrt(torch.sum(base_lin_vel**2, dim=1) +
                                         torch.sum(base_ang_vel**2, dim=1)))

        # Penalty for joint limits
        joint_deviation_penalty = torch.sum((dof_pos - torch.from_numpy(self.default_dof_pos).to(self.device))**2, dim=1)

        # Penalty for joint velocity
        joint_vel_penalty = torch.sum(dof_vel**2, dim=1)

        # Combine rewards
        total_reward = (up_reward * 2.0 +
                       vel_reward * 1.0 -
                       joint_deviation_penalty * 0.01 -
                       joint_vel_penalty * 0.001)

        return total_reward

    def _compute_dones(self):
        """Compute done flags for environments"""

        # Get current states
        base_pos = self.robot.get_world_positions()
        base_euler = torch_utils.quat_to_euler_xyz(self.robot.get_world_quat())

        # Check for termination conditions
        # Too low height
        too_low = base_pos[:, 2] < 0.3
        # Too tilted
        too_tilted = torch.abs(base_euler[:, 1]) > 1.0  # pitch
        # Too tilted in roll
        too_tilted = too_tilted | (torch.abs(base_euler[:, 0]) > 1.0)
        # Too far from origin
        too_far = torch.norm(base_pos[:, :2], dim=1) > 1.0

        # Maximum episode length
        max_length = self.progress_buf >= self.max_episode_length

        # Combine all termination conditions
        dones = too_low | too_tilted | too_far | max_length

        return dones

    def close(self):
        """Close the environment"""

        self.world.clear()

def create_balance_environment():
    """Create and return a stationary balance environment"""

    print("Creating stationary balance training environment...")

    env = StationaryBalanceEnv(num_envs=1, device="cuda" if torch.cuda.is_available() else "cpu")

    print("Balance environment created successfully")
    return env

if __name__ == "__main__":
    # Test the balance environment
    env = create_balance_environment()

    # Reset environment
    obs = env.reset()
    print(f"Initial observation shape: {obs.shape}")

    # Run a few steps with random actions
    for i in range(20):
        # Generate random actions
        actions = torch.randn((1, 12)) * 0.1
        obs, rewards, dones, info = env.step(actions)

        print(f"Step {i}: reward = {rewards.item():.3f}, done = {dones.item()}")

    env.close()
    print("Balance environment test completed")
```

## چلنے کی تربیت کا ماحول

### نقل و حرکت (Locomotion) کی تربیت کا ماحول

```python
#!/usr/bin/env python3
"""
Walking/locomotion training environment
"""
import torch
import numpy as np

class WalkingLocomotionEnv(StationaryBalanceEnv):
    """Environment for training walking locomotion"""

    def __init__(self, num_envs=1, device="cuda", robot_name="A1", max_episode_length=1000):
        # Call parent constructor
        super().__init__(num_envs, device, robot_name, max_episode_length)

        # Additional walking-specific parameters
        self.target_velocity = 0.5  # m/s
        self.velocity_command_range = [-0.8, 0.8]  # Forward/backward range
        self.lateral_velocity_range = [-0.2, 0.2]   # Side-to-side range
        self.angular_velocity_range = [-0.5, 0.5]   # Turning range

        # Walking reward parameters
        self.linear_vel_reward_weight = 1.0
        self.lateral_vel_penalty_weight = 0.1
        self.angular_vel_penalty_weight = 0.1
        self.base_height_reward_weight = 0.5
        self.energy_penalty_weight = 0.01
        self.action_smoothness_weight = 0.01

        # Initialize velocity commands
        self.velocity_commands = torch.zeros((self.num_envs, 3), device=self.device)

    def reset(self, env_ids=None):
        """Reset the environment with new velocity commands"""

        if env_ids is None:
            env_ids = self.reset_idx

        # Generate new velocity commands for reset environments
        if len(env_ids) > 0:
            self.velocity_commands[env_ids, 0] = torch.rand(len(env_ids), device=self.device) * \
                                                (self.velocity_command_range[1] - self.velocity_command_range[0]) + \
                                                self.velocity_command_range[0]
            self.velocity_commands[env_ids, 1] = torch.rand(len(env_ids), device=self.device) * \
                                                (self.lateral_velocity_range[1] - self.lateral_velocity_range[0]) + \
                                                self.lateral_velocity_range[0]
            self.velocity_commands[env_ids, 2] = torch.rand(len(env_ids), device=self.device) * \
                                                (self.angular_velocity_range[1] - self.angular_velocity_range[0]) + \
                                                self.angular_velocity_range[0]

        # Call parent reset
        return super().reset(env_ids)

    def _compute_observations(self, env_ids=None):
        """Compute observations for walking environment"""

        # Get current states
        base_quat = self.robot.get_world_quat()
        base_euler = torch_utils.quat_to_euler_xyz(base_quat)
        base_lin_vel = self.robot.get_linear_velocity()
        base_ang_vel = self.robot.get_angular_velocity()
        dof_pos = self.robot.get_joint_positions()
        dof_vel = self.robot.get_joint_velocities()

        # Get current velocity commands
        commands = self.velocity_commands

        # Normalize observations
        obs = torch.cat([
            base_lin_vel[:, 0:2] * self.lin_vel_scale,  # Forward and lateral linear velocity
            base_ang_vel[:, 2:3] * self.ang_vel_scale,  # Yaw angular velocity
            base_euler,                                  # Base orientation
            dof_pos * self.dof_pos_scale,               # Joint positions
            dof_vel * self.dof_vel_scale,               # Joint velocities
            commands * torch.tensor([2.0, 4.0, 2.0], device=self.device)  # Commands (scaled)
        ], dim=-1)

        return obs

    def _compute_rewards(self):
        """Compute rewards for walking behavior"""

        # Get current states
        base_pos = self.robot.get_world_positions()
        base_quat = self.robot.get_world_quat()
        base_euler = torch_utils.quat_to_euler_xyz(base_quat)
        base_lin_vel = self.robot.get_linear_velocity()
        base_ang_vel = self.robot.get_angular_velocity()
        dof_pos = self.robot.get_joint_positions()
        dof_vel = self.robot.get_joint_velocities()
        dof_efforts = self.robot.get_applied_joint_efforts()

        # Get current commands
        commands = self.velocity_commands

        # Reward for tracking forward velocity
        forward_vel_error = torch.abs(base_lin_vel[:, 0] - commands[:, 0])
        forward_vel_reward = torch.exp(-forward_vel_error / 0.2)

        # Penalty for lateral velocity (when not commanded)
        lateral_vel_error = torch.abs(base_lin_vel[:, 1] - commands[:, 1])
        lateral_vel_penalty = torch.exp(-lateral_vel_error / 0.1)

        # Penalty for angular velocity (when not commanded)
        angular_vel_error = torch.abs(base_ang_vel[:, 2] - commands[:, 2])
        angular_vel_penalty = torch.exp(-angular_vel_error / 0.1)

        # Reward for base height (keeping robot upright)
        base_height_error = torch.abs(base_pos[:, 2] - 0.45)  # Desired height
        base_height_reward = torch.exp(-base_height_error)

        # Penalty for joint efforts (energy efficiency)
        energy_penalty = torch.sum(torch.abs(dof_efforts), dim=1) * self.energy_penalty_weight

        # Penalty for joint position deviation
        joint_deviation_penalty = torch.sum((dof_pos - torch.from_numpy(self.default_dof_pos).to(self.device))**2, dim=1)

        # Combine all rewards and penalties
        total_reward = (forward_vel_reward * self.linear_vel_reward_weight +
                       lateral_vel_penalty * self.lateral_vel_penalty_weight +
                       angular_vel_penalty * self.angular_vel_penalty_weight +
                       base_height_reward * self.base_height_reward_weight -
                       energy_penalty -
                       joint_deviation_penalty * 0.01)

        return total_reward

    def update_command(self, env_ids, new_commands):
        """Update velocity commands for specific environments"""

        self.velocity_commands[env_ids] = new_commands

def create_locomotion_environment():
    """Create and return a locomotion training environment"""

    print("Creating locomotion training environment...")

    env = WalkingLocomotionEnv(num_envs=1, device="cuda" if torch.cuda.is_available() else "cpu")

    print("Locomotion environment created successfully")
    return env

if __name__ == "__main__":
    # Test the locomotion environment
    env = create_locomotion_environment()

    # Reset environment
    obs = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Initial velocity command: {env.velocity_commands[0]}")

    # Run a few steps with small random actions
    for i in range(20):
        # Generate small random actions to test walking
        actions = torch.randn((1, 12)) * 0.05
        obs, rewards, dones, info = env.step(actions)

        print(f"Step {i}: reward = {rewards.item():.3f}, vel_x = {env.robot.get_linear_velocity()[0, 0]:.3f}")

    env.close()
    print("Locomotion environment test completed")
```

## رکاوٹ نیویگیشن ماحول

### رکاوٹوں کے ساتھ نیویگیشن

```python
#!/usr/bin/env python3
"""
Obstacle navigation training environment
"""
from omni.isaac.core.objects import DynamicCuboid
import random

class ObstacleNavigationEnv(WalkingLocomotionEnv):
    """Environment for training navigation with obstacles"""

    def __init__(self, num_envs=1, device="cuda", robot_name="A1", max_episode_length=1500):
        # Call parent constructor
        super().__init__(num_envs, device, robot_name, max_episode_length)

        # Obstacle parameters
        self.num_obstacles_per_env = 10
        self.obstacle_size_range = [0.1, 0.5]  # meters
        self.obstacle_height_range = [0.1, 1.0]  # meters
        self.obstacle_position_range = [-3.0, 3.0, -2.0, 2.0]  # x_min, x_max, y_min, y_max
        self.goal_distance = 5.0  # meters
        self.collision_penalty = -1.0

        # Navigation reward parameters
        self.goal_reward_weight = 5.0
        self.collision_penalty_weight = 10.0
        self.progress_reward_weight = 1.0

        # Initialize obstacles
        self._create_obstacles()

        # Initialize goal positions
        self.goal_positions = torch.zeros((self.num_envs, 3), device=self.device)
        self._generate_goals()

    def _create_obstacles(self):
        """Create obstacles in the environment"""

        print(f"Creating {self.num_obstacles_per_env} obstacles per environment...")

        for env_id in range(self.num_envs):
            env_path = f"/World/envs/env_{env_id}"

            for obs_id in range(self.num_obstacles_per_env):
                # Randomize obstacle properties
                size = random.uniform(*self.obstacle_size_range)
                height = random.uniform(*self.obstacle_height_range)
                pos_x = random.uniform(self.obstacle_position_range[0], self.obstacle_position_range[1])
                pos_y = random.uniform(self.obstacle_position_range[2], self.obstacle_position_range[3])

                obstacle_path = f"{env_path}/Obstacle_{obs_id}"

                # Add obstacle to environment
                self.world.scene.add(
                    DynamicCuboid(
                        prim_path=obstacle_path,
                        name=f"obstacle_{env_id}_{obs_id}",
                        position=[pos_x, pos_y, height/2],  # Place on ground
                        size=size,
                        color=torch.tensor([0.5, 0.5, 0.5])  # Gray color
                    )
                )

    def _generate_goals(self):
        """Generate goal positions for each environment"""

        for env_id in range(self.num_envs):
            # Generate goal in random direction but fixed distance
            angle = random.uniform(0, 2 * np.pi)
            goal_x = self.goal_distance * np.cos(angle)
            goal_y = self.goal_distance * np.sin(angle)

            self.goal_positions[env_id] = torch.tensor([goal_x, goal_y, 0.5], device=self.device)

    def reset(self, env_ids=None):
        """Reset the environment with new obstacles and goals"""

        if env_ids is None:
            env_ids = self.reset_idx

        # Generate new goals for reset environments
        if len(env_ids) > 0:
            for env_id in env_ids:
                angle = random.uniform(0, 2 * np.pi)
                goal_x = self.goal_distance * np.cos(angle)
                goal_y = self.goal_distance * np.sin(angle)

                self.goal_positions[env_id] = torch.tensor([goal_x, goal_y, 0.5], device=self.device)

        # Call parent reset
        return super().reset(env_ids)

    def _compute_observations(self, env_ids=None):
        """Compute observations for navigation environment"""

        # Get current states from parent
        base_obs = super()._compute_observations(env_ids)

        # Get robot position
        robot_pos = self.robot.get_world_positions()

        # Calculate relative goal position
        relative_goal_pos = self.goal_positions - robot_pos
        distance_to_goal = torch.norm(relative_goal_pos[:, :2], dim=1, keepdim=True)
        direction_to_goal = relative_goal_pos / (distance_to_goal + 1e-8)  # Avoid division by zero

        # Combine base observations with navigation-specific observations
        obs = torch.cat([
            base_obs,
            relative_goal_pos[:, :2],  # 2D relative goal position
            distance_to_goal,          # Distance to goal
            direction_to_goal[:, :2]   # Direction to goal
        ], dim=-1)

        return obs

    def _compute_rewards(self):
        """Compute rewards for navigation behavior"""

        # Get current states
        base_pos = self.robot.get_world_positions()
        base_lin_vel = self.robot.get_linear_velocity()
        dof_efforts = self.robot.get_applied_joint_efforts()

        # Calculate distance to goal
        distance_to_goal = torch.norm(self.goal_positions[:, :2] - base_pos[:, :2], dim=1)

        # Calculate progress towards goal
        progress = self.progress_buf.float() * self.dt  # Time-based progress
        distance_progress = torch.clamp(self.goal_distance - distance_to_goal, min=0.0)

        # Navigation reward - reward for getting closer to goal
        goal_reward = (self.goal_distance - distance_to_goal) / self.goal_distance
        goal_reward = torch.clamp(goal_reward, min=0.0)

        # Progress reward - reward for making forward progress
        progress_reward = distance_progress * self.progress_reward_weight

        # Energy efficiency penalty
        energy_penalty = torch.sum(torch.abs(dof_efforts), dim=1) * self.energy_penalty_weight

        # Combine rewards
        total_reward = (goal_reward * self.goal_reward_weight +
                       progress_reward +
                       super()._compute_rewards() * 0.1 -  # Base locomotion reward with lower weight
                       energy_penalty)

        return total_reward

    def _compute_dones(self):
        """Compute done flags for navigation environment"""

        # Get current states
        base_pos = self.robot.get_world_positions()
        base_euler = torch_utils.quat_to_euler_xyz(self.robot.get_world_quat())

        # Calculate distance to goal
        distance_to_goal = torch.norm(self.goal_positions[:, :2] - base_pos[:, :2], dim=1)

        # Check for termination conditions
        # Too low height (robot fell)
        too_low = base_pos[:, 2] < 0.3
        # Too tilted
        too_tilted = (torch.abs(base_euler[:, 1]) > 1.0) | (torch.abs(base_euler[:, 0]) > 1.0)
        # Reached goal
        reached_goal = distance_to_goal < 0.5
        # Too far from start (maybe went in wrong direction)
        too_far = torch.norm(base_pos[:, :2], dim=1) > 10.0
        # Maximum episode length
        max_length = self.progress_buf >= self.max_episode_length

        # Combine all termination conditions
        dones = too_low | too_tilted | reached_goal | too_far | max_length

        return dones

def create_navigation_environment():
    """Create and return a navigation training environment"""

    print("Creating navigation with obstacles training environment...")

    env = ObstacleNavigationEnv(num_envs=1, device="cuda" if torch.cuda.is_available() else "cpu")

    print("Navigation environment created successfully")
    return env

if __name__ == "__main__":
    # Test the navigation environment
    env = create_navigation_environment()

    # Reset environment
    obs = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Goal position: {env.goal_positions[0]}")

    # Run a few steps
    for i in range(50):
        # Generate small random actions to test navigation
        actions = torch.randn((1, 12)) * 0.05
        obs, rewards, dones, info = env.step(actions)

        robot_pos = env.robot.get_world_positions()
        distance_to_goal = torch.norm(env.goal_positions[0, :2] - robot_pos[0, :2]).item()

        print(f"Step {i}: reward = {rewards.item():.3f}, dist_to_goal = {distance_to_goal:.3f}")

    env.close()
    print("Navigation environment test completed")
```

## پیچیدہ خطوں کا ماحول (Complex Terrain Environment)

### کچے خطوں پر نیویگیشن

```python
#!/usr/bin/env python3
"""
Complex terrain training environment
"""
from omni.isaac.core.objects import FixedCuboid, FixedSphere
from omni.isaac.core.utils.prims import create_prim
import math

class ComplexTerrainEnv(ObstacleNavigationEnv):
    """Environment for training on complex terrain"""

    def __init__(self, num_envs=1, device="cuda", robot_name="A1", max_episode_length=2000):
        # Call parent constructor
        super().__init__(num_envs, device, robot_name, max_episode_length)

        # Terrain parameters
        self.terrain_types = ["flat", "stepping_stones", "gap", "slope", "rough"]
        self.terrain_difficulty_range = [0.0, 1.0]  # 0.0 = easy, 1.0 = hard
        self.terrain_size = 8.0  # meters

        # Initialize terrain
        self._create_complex_terrain()

    def _create_complex_terrain(self):
        """Create complex terrain with different challenges"""

        print("Creating complex terrain environments...")

        for env_id in range(self.num_envs):
            env_path = f"/World/envs/env_{env_id}"

            # Select terrain type based on difficulty
            difficulty = random.uniform(*self.terrain_difficulty_range)
            terrain_type = self._select_terrain_type(difficulty)

            print(f"Creating {terrain_type} terrain for env {env_id} (difficulty: {difficulty:.2f})")

            # Create terrain based on selected type
            if terrain_type == "flat":
                self._create_flat_terrain(env_path, env_id)
            elif terrain_type == "stepping_stones":
                self._create_stepping_stones_terrain(env_path, env_id, difficulty)
            elif terrain_type == "gap":
                self._create_gap_terrain(env_path, env_id, difficulty)
            elif terrain_type == "slope":
                self._create_slope_terrain(env_path, env_id, difficulty)
            elif terrain_type == "rough":
                self._create_rough_terrain(env_path, env_id, difficulty)

    def _select_terrain_type(self, difficulty):
        """Select terrain type based on difficulty level"""

        if difficulty < 0.2:
            return "flat"
        elif difficulty < 0.4:
            return "slope"
        elif difficulty < 0.6:
            return "stepping_stones"
        elif difficulty < 0.8:
            return "rough"
        else:
            return random.choice(["gap", "rough", "stepping_stones"])

    def _create_flat_terrain(self, env_path, env_id):
        """Create flat terrain"""

        # Use the default ground plane
        pass

    def _create_stepping_stones_terrain(self, env_path, env_id, difficulty):
        """Create stepping stones terrain"""

        # Calculate number and size of stones based on difficulty
        num_stones = int(5 + difficulty * 15)
        stone_size = 0.3 - difficulty * 0.15
        stone_height = 0.1
        gap_size = 0.1 + difficulty * 0.3

        for i in range(num_stones):
            x_pos = -self.terrain_size/2 + (i + 0.5) * (self.terrain_size / num_stones)
            y_pos = random.uniform(-1.0, 1.0)

            stone_path = f"{env_path}/Stone_{i}"

            # Create stone
            self.world.scene.add(
                FixedCuboid(
                    prim_path=stone_path,
                    name=f"stone_{env_id}_{i}",
                    position=[x_pos, y_pos, stone_height/2],
                    size=[stone_size, stone_size, stone_height],
                    color=torch.tensor([0.7, 0.7, 0.7])
                )
            )

    def _create_gap_terrain(self, env_path, env_id, difficulty):
        """Create gap terrain"""

        # Calculate gap parameters based on difficulty
        num_gaps = int(1 + difficulty * 3)
        gap_width = 0.2 + difficulty * 0.6
        platform_width = 1.0 - difficulty * 0.5

        x_start = -self.terrain_size/4
        for i in range(num_gaps):
            # Create platform before gap
            platform_path = f"{env_path}/Platform_Before_Gap_{i}"
            self.world.scene.add(
                FixedCuboid(
                    prim_path=platform_path,
                    name=f"platform_before_gap_{env_id}_{i}",
                    position=[x_start, 0, 0.05],
                    size=[platform_width, 2.0, 0.1],
                    color=torch.tensor([0.5, 0.5, 0.5])
                )
            )

            # Move to after gap
            x_start += platform_width + gap_width

            # Create platform after gap
            platform_path = f"{env_path}/Platform_After_Gap_{i}"
            self.world.scene.add(
                FixedCuboid(
                    prim_path=platform_path,
                    name=f"platform_after_gap_{env_id}_{i}",
                    position=[x_start, 0, 0.05],
                    size=[platform_width, 2.0, 0.1],
                    color=torch.tensor([0.5, 0.5, 0.5])
                )
            )

            x_start += platform_width

    def _create_slope_terrain(self, env_path, env_id, difficulty):
        """Create sloped terrain"""

        # Calculate slope angle based on difficulty
        slope_angle = math.radians(difficulty * 20)  # 0 to 20 degrees
        slope_length = self.terrain_size
        slope_height = slope_length * math.tan(slope_angle)

        slope_path = f"{env_path}/Slope"

        # Create sloped platform
        self.world.scene.add(
            FixedCuboid(
                prim_path=slope_path,
                name=f"slope_{env_id}",
                position=[0, 0, slope_height/2],
                size=[slope_length, 2.0, slope_height],
                color=torch.tensor([0.6, 0.6, 0.6])
            )
        )

        # Rotate to create slope
        # Note: In actual implementation, you would use proper rotation

    def _create_rough_terrain(self, env_path, env_id, difficulty):
        """Create rough terrain with obstacles"""

        # Calculate roughness based on difficulty
        num_obstacles = int(5 + difficulty * 50)
        obstacle_size_range = [0.05, 0.1 + difficulty * 0.15]

        for i in range(num_obstacles):
            x_pos = random.uniform(-self.terrain_size/2, self.terrain_size/2)
            y_pos = random.uniform(-self.terrain_size/4, self.terrain_size/4)
            size = random.uniform(*obstacle_size_range)
            height = random.uniform(0.05, 0.1 + difficulty * 0.2)

            obstacle_path = f"{env_path}/RoughObstacle_{i}"

            self.world.scene.add(
                FixedSphere(
                    prim_path=obstacle_path,
                    name=f"rough_obstacle_{env_id}_{i}",
                    position=[x_pos, y_pos, height/2],
                    radius=size/2,
                    color=torch.tensor([0.4, 0.4, 0.4])
                )
            )

    def _compute_rewards(self):
        """Compute rewards for complex terrain navigation"""

        # Get base rewards from parent
        base_rewards = super()._compute_rewards()

        # Get current states
        base_pos = self.robot.get_world_positions()
        base_lin_vel = self.robot.get_linear_velocity()
        base_ang_vel = self.robot.get_angular_velocity()

        # Additional rewards for handling terrain variations
        terrain_adaptation_reward = torch.zeros(self.num_envs, device=self.device)

        # Reward for maintaining stable velocity on rough terrain
        vel_magnitude = torch.norm(base_lin_vel[:, :2], dim=1)
        stable_vel_reward = torch.exp(-torch.abs(vel_magnitude - self.target_velocity) / 0.5)

        terrain_adaptation_reward += stable_vel_reward * 0.5

        # Combine rewards
        total_reward = base_rewards + terrain_adaptation_reward

        return total_reward

def create_complex_terrain_environment():
    """Create and return a complex terrain training environment"""

    print("Creating complex terrain training environment...")

    env = ComplexTerrainEnv(num_envs=1, device="cuda" if torch.cuda.is_available() else "cpu")

    print("Complex terrain environment created successfully")
    return env

if __name__ == "__main__":
    # Test the complex terrain environment
    env = create_complex_terrain_environment()

    # Reset environment
    obs = env.reset()
    print(f"Initial observation shape: {obs.shape}")

    # Run a few steps
    for i in range(30):
        # Generate small random actions
        actions = torch.randn((1, 12)) * 0.05
        obs, rewards, dones, info = env.step(actions)

        print(f"Step {i}: reward = {rewards.item():.3f}")

    env.close()
    print("Complex terrain environment test completed")
```

## نصاب سیکھنے کا سیٹ اپ (Curriculum Learning Setup)

### ترقی پسند مشکل کی تربیت (Progressive Difficulty Training)

```python
#!/usr/bin/env python3
"""
Curriculum learning setup for progressive difficulty training
"""
class CurriculumLearningEnv:
    """Environment that progressively increases difficulty"""

    def __init__(self, base_env, curriculum_params=None):
        self.base_env = base_env
        self.curriculum_params = curriculum_params or self.get_default_curriculum_params()

        # Curriculum tracking
        self.current_stage = 0
        self.stage_thresholds = self.curriculum_params['stage_thresholds']
        self.performance_history = []
        self.episode_count = 0

        # Current difficulty parameters
        self.current_params = self._interpolate_params(0.0)

    def get_default_curriculum_params(self):
        """Get default curriculum parameters"""

        params = {
            'stage_thresholds': [0.3, 0.6, 0.8, 0.9],  # Performance thresholds for each stage
            'param_interpolation': {
                'obstacle_count': [0, 5, 10, 15, 20],  # Increasing obstacles
                'terrain_difficulty': [0.0, 0.3, 0.6, 0.8, 1.0],  # Increasing terrain difficulty
                'goal_distance': [2.0, 3.0, 4.0, 5.0, 6.0],  # Increasing goal distance
                'time_limit': [500, 800, 1200, 1500, 2000]  # Increasing time limits
            }
        }

        return params

    def _interpolate_params(self, progress):
        """Interpolate parameters based on curriculum progress"""

        # Calculate which stage we're in
        stage = min(int(progress * len(self.stage_thresholds)), len(self.stage_thresholds))

        # Get parameter values for current stage
        params = {}
        for param_name, values in self.curriculum_params['param_interpolation'].items():
            if stage < len(values):
                params[param_name] = values[stage]
            else:
                params[param_name] = values[-1]  # Use max value if beyond range

        return params

    def update_curriculum(self, episode_performance):
        """Update curriculum based on performance"""

        # Add performance to history
        self.performance_history.append(episode_performance)

        # Keep only recent performances
        if len(self.performance_history) > 100:
            self.performance_history = self.performance_history[-100:]

        # Calculate average performance
        if len(self.performance_history) >= 10:  # Need minimum episodes to evaluate
            avg_performance = sum(self.performance_history[-10:]) / 10
            curriculum_progress = avg_performance

            # Update current parameters based on progress
            self.current_params = self._interpolate_params(curriculum_progress)

            # Update environment with new parameters
            self._apply_current_params()

            print(f"Curriculum progress: {curriculum_progress:.3f}, stage: {self.current_params['obstacle_count']} obstacles")

    def _apply_current_params(self):
        """Apply current parameters to the base environment"""

        # This would involve updating the base environment's parameters
        # For example, updating obstacle counts, terrain difficulty, etc.
        pass

    def reset(self):
        """Reset the environment"""

        return self.base_env.reset()

    def step(self, action):
        """Take a step in the environment"""

        obs, reward, done, info = self.base_env.step(action)

        # Update curriculum based on episode completion
        if done:
            # Calculate episode performance (this would be specific to your task)
            episode_performance = float(torch.mean(reward))  # Simplified performance metric
            self.update_curriculum(episode_performance)
            self.episode_count += 1

        return obs, reward, done, info

    def close(self):
        """Close the environment"""

        self.base_env.close()

def create_curriculum_environment():
    """Create a curriculum learning environment"""

    print("Creating curriculum learning environment...")

    # Create base environment (navigation in this case)
    base_env = create_navigation_environment()

    # Create curriculum wrapper
    curriculum_env = CurriculumLearningEnv(base_env)

    print("Curriculum learning environment created")
    return curriculum_env

if __name__ == "__main__":
    # Test curriculum environment
    env = create_curriculum_environment()

    # Run a few episodes
    for episode in range(5):
        obs = env.reset()
        total_reward = 0

        for step in range(100):  # Limited steps for testing
            actions = torch.randn((1, 12)) * 0.05
            obs, rewards, dones, info = env.step(actions)
            total_reward += rewards.item()

            if dones:
                break

        print(f"Episode {episode}: Total reward = {total_reward:.3f}")

    env.close()
    print("Curriculum environment test completed")
```

## ماحولیاتی رینڈمائزیشن (Environment Randomization)

### تربیتی ماحول میں ڈومین رینڈمائزیشن

```python
#!/usr/bin/env python3
"""
Environment randomization techniques for improved generalization
"""
class EnvironmentRandomizer:
    """Randomizer for training environments"""

    def __init__(self, base_env):
        self.base_env = base_env
        self.randomization_params = self.get_default_randomization_params()

    def get_default_randomization_params(self):
        """Get default randomization parameters"""

        params = {
            'visual_randomization': {
                'lighting': {
                    'enabled': True,
                    'intensity_range': [0.5, 2.0],
                    'color_temperature_range': [4000, 8000]
                },
                'materials': {
                    'enabled': True,
                    'roughness_range': [0.1, 0.9],
                    'metallic_range': [0.0, 0.5]
                }
            },
            'physical_randomization': {
                'mass': {
                    'enabled': True,
                    'multiplier_range': [0.8, 1.2]
                },
                'friction': {
                    'enabled': True,
                    'multiplier_range': [0.5, 2.0]
                },
                'restitution': {
                    'enabled': True,
                    'range': [0.0, 0.5]
                }
            },
            'dynamic_randomization': {
                'external_forces': {
                    'enabled': True,
                    'magnitude_range': [0.0, 5.0]
                },
                'disturbance_frequency': {
                    'enabled': True,
                    'range': [0.1, 2.0]  # Hz
                }
            }
        }

        return params

    def randomize_visual_properties(self):
        """Randomize visual properties of the environment"""

        if not self.randomization_params['visual_randomization']['lighting']['enabled']:
            return

        # This would involve changing lighting conditions
        print("Randomizing visual properties...")

    def randomize_physical_properties(self):
        """Randomize physical properties of objects"""

        if not self.randomization_params['physical_randomization']['mass']['enabled']:
            return

        # Randomize robot mass properties
        mass_multiplier = random.uniform(
            *self.randomization_params['physical_randomization']['mass']['multiplier_range']
        )
        print(f"Randomizing mass with multiplier: {mass_multiplier}")

    def randomize_dynamics(self):
        """Randomize dynamic properties"""

        if not self.randomization_params['dynamic_randomization']['external_forces']['enabled']:
            return

        # Apply random external forces occasionally
        if random.random() < 0.1:  # 10% chance each step
            force_magnitude = random.uniform(
                *self.randomization_params['dynamic_randomization']['external_forces']['magnitude_range']
            )
            print(f"Applying random external force: {force_magnitude}")

    def apply_randomization(self):
        """Apply all randomization techniques"""

        self.randomize_visual_properties()
        self.randomize_physical_properties()
        self.randomize_dynamics()

    def reset(self, env_ids=None):
        """Reset environment with randomization"""

        # Apply randomization
        self.apply_randomization()

        # Reset base environment
        return self.base_env.reset(env_ids)

    def step(self, action):
        """Step environment with potential dynamic randomization"""

        # Apply dynamic randomization
        self.randomize_dynamics()

        # Step base environment
        return self.base_env.step(action)

    def close(self):
        """Close the environment"""

        self.base_env.close()

def create_randomized_environment():
    """Create an environment with randomization"""

    print("Creating randomized environment...")

    # Create base environment
    base_env = create_balance_environment()

    # Wrap with randomizer
    randomized_env = EnvironmentRandomizer(base_env)

    print("Randomized environment created")
    return randomized_env

if __name__ == "__main__":
    # Test randomized environment
    env = create_randomized_environment()

    # Run a few steps
    obs = env.reset()
    for step in range(20):
        actions = torch.randn((1, 12)) * 0.05
        obs, rewards, dones, info = env.step(actions)
        print(f"Step {step}: reward = {rewards.item():.3f}")

    env.close()
    print("Randomized environment test completed")
```

## کارکردگی کی اصلاح (Performance Optimization)

### موثر ماحولیاتی نفاذ

```python
#!/usr/bin/env python3
"""
Performance optimization for training environments
"""
class OptimizedTrainingEnv:
    """Optimized training environment implementation"""

    def __init__(self, num_envs=1024, device="cuda"):
        self.num_envs = num_envs
        self.device = device

        # Pre-allocate tensors to avoid memory allocation during training
        self._allocate_tensors()

        # Optimization settings
        self.vectorized_ops = True
        self.batch_processing = True
        self.memory_efficient = True

    def _allocate_tensors(self):
        """Pre-allocate tensors to avoid memory allocation during training"""

        # Allocate state tensors
        self.obs_buf = torch.zeros((self.num_envs, 48), device=self.device, dtype=torch.float32)
        self.rew_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.float32)
        self.reset_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.progress_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)

        # Allocate action tensors
        self.actions = torch.zeros((self.num_envs, 12), device=self.device, dtype=torch.float32)

        # Allocate state information tensors
        self.base_pos = torch.zeros((self.num_envs, 3), device=self.device)
        self.base_quat = torch.zeros((self.num_envs, 4), device=self.device)
        self.base_lin_vel = torch.zeros((self.num_envs, 3), device=self.device)
        self.base_ang_vel = torch.zeros((self.num_envs, 3), device=self.device)
        self.dof_pos = torch.zeros((self.num_envs, 12), device=self.device)
        self.dof_vel = torch.zeros((self.num_envs, 12), device=self.device)

    def reset(self, env_ids=None):
        """Optimized reset function"""

        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        # Vectorized reset for specified environments
        self._reset_actors(env_ids)
        self._reset_buffers(env_ids)

        # Return observations
        return self._compute_observations_batch(env_ids)

    def _reset_actors(self, env_ids):
        """Reset actors (robots) for specified environments"""

        # Vectorized position reset
        self.base_pos[env_ids, :] = 0.0
        self.base_pos[env_ids, 2] = 0.6  # Set height

        # Vectorized orientation reset
        self.base_quat[env_ids, :] = 0.0
        self.base_quat[env_ids, 3] = 1.0  # Set to identity

        # Reset DOF states
        self.dof_pos[env_ids, :] = 0.0
        self.dof_vel[env_ids, :] = 0.0

    def _reset_buffers(self, env_ids):
        """Reset environment buffers"""

        self.progress_buf[env_ids] = 0
        self.reset_buf[env_ids] = 0

    def step(self, actions):
        """Optimized step function"""

        # Store actions
        self.actions[:] = actions

        # Apply actions (vectorized)
        self._apply_actions_vectorized()

        # Compute next states (vectorized)
        self._compute_next_states()

        # Compute observations (vectorized)
        obs = self._compute_observations_batch()

        # Compute rewards (vectorized)
        rewards = self._compute_rewards_vectorized()

        # Compute dones (vectorized)
        dones = self._compute_dones_vectorized()

        # Update progress
        self.progress_buf += 1

        # Handle resets
        reset_env_ids = dones.nonzero(as_tuple=False).flatten()
        if len(reset_env_ids) > 0:
            self.reset(reset_env_ids)

        # Update reset buffer
        self.reset_buf[:] = dones

        return obs, rewards, dones, {}

    def _apply_actions_vectorized(self):
        """Apply actions in a vectorized manner"""

        # This would involve applying actions to all robots simultaneously
        # using vectorized operations
        pass

    def _compute_next_states(self):
        """Compute next states using vectorized operations"""

        # Update state tensors based on physics simulation
        # This is simplified - actual implementation would interface with physics engine
        pass

    def _compute_observations_batch(self, env_ids=None):
        """Compute observations for batch of environments"""

        if env_ids is None:
            env_ids = slice(None)

        # Vectorized observation computation
        obs = torch.cat([
            self.base_lin_vel[env_ids] * 0.1,
            self.base_ang_vel[env_ids] * 0.1,
            self.dof_pos[env_ids],
            self.dof_vel[env_ids],
        ], dim=-1)

        return obs

    def _compute_rewards_vectorized(self):
        """Compute rewards for all environments using vectorized operations"""

        # Calculate rewards using vectorized tensor operations
        # Example reward calculation:
        up_reward = self.base_quat[:, 3]  # W component of quaternion (uprightness)
        vel_reward = torch.exp(-torch.norm(self.base_lin_vel, dim=1) * 0.1)

        total_rewards = up_reward * 2.0 + vel_reward * 1.0

        return total_rewards

    def _compute_dones_vectorized(self):
        """Compute done flags using vectorized operations"""

        # Check termination conditions for all environments at once
        too_low = self.base_pos[:, 2] < 0.3
        max_length = self.progress_buf >= 1000

        return too_low | max_length

def benchmark_environment_performance(env_class, num_envs_list=[512, 1024, 2048]):
    """Benchmark environment performance with different numbers of environments"""

    import time

    print("Benchmarking environment performance...")

    for num_envs in num_envs_list:
        print(f"\nTesting with {num_envs} environments:")

        # Create environment
        env = env_class(num_envs=num_envs)

        # Warm up
        obs = env.reset()

        # Benchmark
        start_time = time.time()
        total_steps = 1000

        for step in range(total_steps):
            actions = torch.randn((num_envs, 12), device=env.device) * 0.1
            obs, rewards, dones, info = env.step(actions)

            if step % 200 == 0:
                elapsed = time.time() - start_time
                avg_fps = (step + 1) / elapsed
                print(f"  Step {step}: {avg_fps:.1f} FPS")

        end_time = time.time()
        total_time = end_time - start_time
        avg_fps = total_steps / total_time

        print(f"  Average performance: {avg_fps:.1f} FPS")

if __name__ == "__main__":
    # Benchmark optimized environment
    benchmark_environment_performance(OptimizedTrainingEnv, num_envs_list=[128, 256, 512])
```

## تربیتی ماحول کے لیے بہترین طریقے

### موثر تربیت کے لیے رہنما خطوط

1. **ماحولیاتی ڈیزائن**: آسان شروع کریں اور آہستہ آہستہ پیچیدگی میں اضافہ کریں۔
2. **انعام کی تشکیل (Reward Shaping)**: ایسے انعامات ڈیزائن کریں جو ایجنٹ کو مطلوبہ رویوں کی طرف رہنمائی کریں۔
3. **حفاظتی رکاوٹیں**: تربیت کے دوران محفوظ تلاش (exploration) کو یقینی بنائیں۔
4. **تنوع**: عمومیت (generalization) کو بہتر بنانے کے لیے مختلف منظرنامے شامل کریں۔
5. **توثیق**: باقاعدگی سے پالیسیوں کو متنوع ماحول میں ٹیسٹ کریں۔
6. **نگرانی**: تربیتی پیشرفت اور ماحولیاتی اعدادوشمار کو ٹریک کریں۔

### ہیومنوائڈ کے لیے مخصوص تحفظات

1. **پہلے توازن**: لوکوموشن سے پہلے بنیادی توازن میں مہارت حاصل کریں۔
2. **طبیعیات کی درستگی**: مستحکم کنٹرول کے لیے درست طبیعیات کی سمولیشن کو یقینی بنائیں۔
3. **رابطہ ہینڈلنگ**: چلنے کے لیے زمینی رابطوں کی مناسب نقل کریں۔
4. **حسی فیڈ بیک**: حقیقت پسندانہ سینسر سمولیشن شامل کریں۔
5. **ایکچویٹر ڈائنامکس**: ماڈل ایکچویٹر کی حدود اور تاخیر۔

## اگلے اقدامات

تربیتی ماحول بنانے کے بعد:

1. **ٹاسک کی وضاحت کریں**: اپنے ہیومنوائڈ روبوٹ کے لیے کنٹرول کے عین مطابق کاموں کی وضاحت کریں۔
2. **انعام کے فنکشنز نافذ کریں**: انعامی فنکشنز بنائیں جو مطلوبہ رویوں کو فروغ دیں۔
3. **تربیت شروع کریں**: آسان ماحول کے ساتھ شروع کریں اور آہستہ آہستہ پیچیدگی میں اضافہ کریں۔
4. **پیشرفت کی نگرانی کریں**: سیکھنے کی پیشرفت کو ٹریک کریں اور ہائپر پیرامیٹرز کو ایڈجسٹ کریں۔
5. **پالیسیوں کی توثیق کریں**: سمولیشن اور حقیقی دنیا کی ترتیبات میں تربیت یافتہ پالیسیوں کی جانچ کریں۔
6. **دہرائیں اور بہتر بنائیں**: نتائج کی بنیاد پر ماحول اور تربیت کو بہتر بنائیں۔

اگلا سیکشن کارکردگی کی اصلاح اور جدید تکنیکوں کا احاطہ کرتا ہے۔
