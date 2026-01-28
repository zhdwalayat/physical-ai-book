---
sidebar_position: 3
---

# Week 10: Sim-to-Real Transfer

## Learning Objectives

By the end of this week, you will be able to:

- **Understand** the sim-to-real gap and its fundamental causes
- **Apply** domain randomization techniques systematically
- **Train** reinforcement learning policies in Isaac Sim
- **Export** and deploy trained models to real hardware
- **Evaluate** transfer performance with quantitative metrics

---

## 1. The Sim-to-Real Gap

### 1.1 Understanding the Gap

The **sim-to-real gap** refers to the performance difference between simulation and reality:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          The Sim-to-Real Gap                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌─────────────────────────┐         ┌─────────────────────────┐         │
│    │      SIMULATION         │         │        REALITY          │         │
│    │                         │         │                         │         │
│    │  ✓ Perfect physics      │   GAP   │  ✗ Unmodeled dynamics   │         │
│    │  ✓ Ideal sensors        │  ───►   │  ✗ Sensor noise/drift   │         │
│    │  ✓ Instant reset        │         │  ✗ Slow, careful reset  │         │
│    │  ✓ Parallel envs        │         │  ✗ Single robot         │         │
│    │  ✓ Known parameters     │         │  ✗ Uncertain parameters │         │
│    │  ✓ Reproducible         │         │  ✗ Non-deterministic    │         │
│    │                         │         │                         │         │
│    │    Policy achieves      │         │    Same policy may      │         │
│    │    98% success rate     │         │    only achieve 40%     │         │
│    └─────────────────────────┘         └─────────────────────────┘         │
│                                                                              │
│    Goal: Minimize this gap through careful simulation design                │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Sources of the Gap

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     Sources of Sim-to-Real Gap                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  1. PHYSICS MISMATCH                                                  │  │
│  │     • Contact dynamics (friction, restitution)                        │  │
│  │     • Joint friction, damping, backlash                               │  │
│  │     • Mass distribution, inertia errors                               │  │
│  │     • Actuator dynamics (motor response, saturation)                  │  │
│  │     • Deformable objects (cables, soft materials)                     │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  2. SENSOR DISCREPANCIES                                              │  │
│  │     • Camera: Exposure, white balance, lens distortion               │  │
│  │     • Depth: Noise, missing data, reflections                         │  │
│  │     • IMU: Bias drift, temperature sensitivity                        │  │
│  │     • Encoders: Quantization, communication delays                    │  │
│  │     • Time synchronization between sensors                            │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  3. VISUAL DIFFERENCES                                                │  │
│  │     • Lighting conditions (intensity, color temperature)              │  │
│  │     • Textures and materials (reflectance, specularity)               │  │
│  │     • Backgrounds and clutter                                         │  │
│  │     • Object appearance variation                                     │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  4. ENVIRONMENTAL FACTORS                                             │  │
│  │     • Air resistance (significant for fast motions)                   │  │
│  │     • Temperature effects on electronics                              │  │
│  │     • Wear and tear over time                                         │  │
│  │     • Unexpected obstacles and interactions                           │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.3 Gap Comparison by Domain

| Aspect | Simulation | Reality | Gap Severity |
|--------|------------|---------|--------------|
| **Contact Forces** | Approximated | Complex, uncertain | High |
| **Sensor Noise** | Modeled or absent | Present, varying | Medium |
| **Visual Appearance** | Rendered | Natural lighting | High (vision-based) |
| **Latency** | Configurable | Fixed, variable | Medium |
| **Motor Response** | Idealized | Non-linear | Medium |
| **Object Dynamics** | Known | Unknown variations | High |

---

## 2. Domain Randomization

### 2.1 Concept and Motivation

**Domain Randomization (DR)**: Train on sufficiently diverse simulations that reality becomes "just another variant."

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Domain Randomization Concept                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│     Training Distribution                        Reality                     │
│                                                                              │
│     ┌─────────────────────────────┐         ┌─────────────────────────┐    │
│     │  ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○   │         │                         │    │
│     │ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○  │         │                         │    │
│     │○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ │         │           ★             │    │
│     │ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○  │   ═══►  │      (single point)     │    │
│     │  ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○ ○   │         │                         │    │
│     │   (many variants)          │         │                         │    │
│     └─────────────────────────────┘         └─────────────────────────┘    │
│                                                                              │
│     Wide randomization ensures reality falls within training distribution   │
│                                                                              │
│     ────────────────────────────────────────────────────────────────────    │
│                                                                              │
│     Key Insight: Policy must learn features that are                        │
│                  INVARIANT across all randomized variations                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 What to Randomize

```python
#!/usr/bin/env python3
"""
domain_randomization.py - Systematic domain randomization for sim-to-real
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class PhysicsRandomization:
    """Physics parameter randomization ranges."""

    # Mass (multiplicative factor)
    mass_range: Tuple[float, float] = (0.8, 1.2)

    # Friction coefficients
    friction_range: Tuple[float, float] = (0.5, 1.5)

    # Joint damping (multiplicative)
    damping_range: Tuple[float, float] = (0.5, 2.0)

    # Joint friction
    joint_friction_range: Tuple[float, float] = (0.01, 0.1)

    # Motor strength (multiplicative)
    motor_strength_range: Tuple[float, float] = (0.8, 1.2)

    # Ground restitution
    restitution_range: Tuple[float, float] = (0.0, 0.3)

    # Gravity variation
    gravity_range: Tuple[float, float] = (9.6, 10.0)


@dataclass
class VisualRandomization:
    """Visual appearance randomization ranges."""

    # Lighting intensity (lux)
    light_intensity_range: Tuple[float, float] = (300, 2000)

    # Light color temperature (Kelvin)
    light_temperature_range: Tuple[float, float] = (3500, 7000)

    # Light position jitter (meters)
    light_position_jitter: float = 1.0

    # Texture sets to sample from
    texture_sets: List[str] = field(default_factory=lambda: [
        "wood", "metal", "plastic", "concrete", "fabric"
    ])

    # Object color variation (HSV)
    hue_range: Tuple[float, float] = (-0.1, 0.1)
    saturation_range: Tuple[float, float] = (0.5, 1.0)
    value_range: Tuple[float, float] = (0.5, 1.0)

    # Background variation
    background_images: Optional[List[str]] = None


@dataclass
class SensorRandomization:
    """Sensor noise and characteristics randomization."""

    # Camera
    camera_noise_stddev: Tuple[float, float] = (0.0, 0.02)
    camera_exposure_range: Tuple[float, float] = (0.8, 1.2)
    camera_latency_range: Tuple[float, float] = (0.0, 0.05)  # seconds

    # Depth sensor
    depth_noise_stddev: Tuple[float, float] = (0.0, 0.02)  # meters
    depth_dropout_rate: Tuple[float, float] = (0.0, 0.1)

    # IMU
    gyro_noise: Tuple[float, float] = (0.0001, 0.001)  # rad/s
    gyro_bias: Tuple[float, float] = (-0.01, 0.01)  # rad/s
    accel_noise: Tuple[float, float] = (0.001, 0.02)  # m/s²
    accel_bias: Tuple[float, float] = (-0.1, 0.1)  # m/s²

    # Encoder
    encoder_noise: float = 0.001  # radians


@dataclass
class DynamicsRandomization:
    """Dynamics and actuation randomization."""

    # Action delay (timesteps)
    action_delay_range: Tuple[int, int] = (0, 3)

    # Action noise
    action_noise_stddev: Tuple[float, float] = (0.0, 0.05)

    # Communication dropout
    dropout_rate: Tuple[float, float] = (0.0, 0.02)

    # External disturbance forces (N)
    external_force_range: Tuple[float, float] = (0.0, 10.0)
    disturbance_probability: float = 0.01


class DomainRandomizer:
    """Apply domain randomization to simulation."""

    def __init__(
        self,
        physics: PhysicsRandomization = None,
        visual: VisualRandomization = None,
        sensor: SensorRandomization = None,
        dynamics: DynamicsRandomization = None,
    ):
        self.physics = physics or PhysicsRandomization()
        self.visual = visual or VisualRandomization()
        self.sensor = sensor or SensorRandomization()
        self.dynamics = dynamics or DynamicsRandomization()

    def sample_physics(self) -> dict:
        """Sample physics parameters."""
        return {
            'mass_scale': np.random.uniform(*self.physics.mass_range),
            'friction': np.random.uniform(*self.physics.friction_range),
            'damping_scale': np.random.uniform(*self.physics.damping_range),
            'joint_friction': np.random.uniform(*self.physics.joint_friction_range),
            'motor_strength': np.random.uniform(*self.physics.motor_strength_range),
            'restitution': np.random.uniform(*self.physics.restitution_range),
            'gravity': np.random.uniform(*self.physics.gravity_range),
        }

    def sample_visual(self) -> dict:
        """Sample visual parameters."""
        return {
            'light_intensity': np.random.uniform(*self.visual.light_intensity_range),
            'light_temperature': np.random.uniform(*self.visual.light_temperature_range),
            'light_offset': np.random.uniform(-1, 1, 3) * self.visual.light_position_jitter,
            'texture': np.random.choice(self.visual.texture_sets),
            'hue_shift': np.random.uniform(*self.visual.hue_range),
            'saturation': np.random.uniform(*self.visual.saturation_range),
            'value': np.random.uniform(*self.visual.value_range),
        }

    def sample_sensor(self) -> dict:
        """Sample sensor noise parameters."""
        return {
            'camera_noise': np.random.uniform(*self.sensor.camera_noise_stddev),
            'camera_exposure': np.random.uniform(*self.sensor.camera_exposure_range),
            'camera_latency': np.random.uniform(*self.sensor.camera_latency_range),
            'depth_noise': np.random.uniform(*self.sensor.depth_noise_stddev),
            'depth_dropout': np.random.uniform(*self.sensor.depth_dropout_rate),
            'gyro_noise': np.random.uniform(*self.sensor.gyro_noise),
            'gyro_bias': np.random.uniform(*self.sensor.gyro_bias, 3),
            'accel_noise': np.random.uniform(*self.sensor.accel_noise),
            'accel_bias': np.random.uniform(*self.sensor.accel_bias, 3),
        }

    def sample_dynamics(self) -> dict:
        """Sample dynamics parameters."""
        return {
            'action_delay': np.random.randint(*self.dynamics.action_delay_range),
            'action_noise': np.random.uniform(*self.dynamics.action_noise_stddev),
            'dropout_rate': np.random.uniform(*self.dynamics.dropout_rate),
            'external_force': np.random.uniform(*self.dynamics.external_force_range),
        }

    def sample_all(self) -> dict:
        """Sample all randomization parameters."""
        return {
            'physics': self.sample_physics(),
            'visual': self.sample_visual(),
            'sensor': self.sample_sensor(),
            'dynamics': self.sample_dynamics(),
        }
```

### 2.3 Isaac Sim Replicator Integration

```python
#!/usr/bin/env python3
"""
isaac_domain_randomization.py - Domain randomization in Isaac Sim
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation


def create_randomized_environment():
    """Create environment with domain randomization."""

    # Setup basic world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Register custom randomizers
    register_physics_randomizers()
    register_visual_randomizers()

    return world


def register_physics_randomizers():
    """Register physics randomization with Replicator."""

    @rep.randomizer.register("physics_randomizer")
    def physics_randomizer(robot_path: str, config: dict):
        """Randomize physics properties of robot and environment."""

        from pxr import UsdPhysics, PhysxSchema
        from omni.isaac.core.utils.stage import get_current_stage

        stage = get_current_stage()
        robot_prim = stage.GetPrimAtPath(robot_path)

        # Randomize link masses
        mass_scale = config.get('mass_scale', 1.0)
        for descendant in robot_prim.GetDescendants():
            if descendant.HasAPI(UsdPhysics.MassAPI):
                mass_api = UsdPhysics.MassAPI(descendant)
                original_mass = mass_api.GetMassAttr().Get()
                if original_mass:
                    mass_api.GetMassAttr().Set(original_mass * mass_scale)

        # Randomize joint properties
        for descendant in robot_prim.GetDescendants():
            if descendant.HasAPI(PhysxSchema.PhysxJointAPI):
                joint_api = PhysxSchema.PhysxJointAPI(descendant)

                # Joint friction
                joint_api.GetJointFrictionAttr().Set(
                    config.get('joint_friction', 0.05)
                )

        # Randomize ground friction
        ground_path = "/World/defaultGroundPlane/GroundPlane/CollisionPlane"
        ground_prim = stage.GetPrimAtPath(ground_path)
        if ground_prim:
            if ground_prim.HasAPI(UsdPhysics.MaterialAPI):
                mat_api = UsdPhysics.MaterialAPI(ground_prim)
                mat_api.GetStaticFrictionAttr().Set(config.get('friction', 1.0))
                mat_api.GetDynamicFrictionAttr().Set(config.get('friction', 1.0) * 0.8)

        return None


def register_visual_randomizers():
    """Register visual randomization with Replicator."""

    @rep.randomizer.register("lighting_randomizer")
    def lighting_randomizer():
        """Randomize scene lighting."""
        with rep.create.light(light_type="distant", name="RandomLight"):
            rep.modify.pose(
                rotation=rep.distribution.uniform((-180, -90, 0), (180, -30, 0))
            )
            rep.modify.attribute(
                "intensity",
                rep.distribution.uniform(500, 3000)
            )
            rep.modify.attribute(
                "colorTemperature",
                rep.distribution.uniform(3500, 7000)
            )
        return None

    @rep.randomizer.register("texture_randomizer")
    def texture_randomizer(prim_paths: list):
        """Randomize object textures."""
        textures = [
            "omniverse://localhost/NVIDIA/Materials/Base/Wood/Oak.mdl",
            "omniverse://localhost/NVIDIA/Materials/Base/Metal/Aluminum.mdl",
            "omniverse://localhost/NVIDIA/Materials/Base/Plastic/ABS.mdl",
            "omniverse://localhost/NVIDIA/Materials/Base/Stone/Concrete.mdl",
        ]

        with rep.get.prims(path_pattern=prim_paths):
            rep.randomizer.materials(
                materials=rep.distribution.choice(textures)
            )
        return None

    @rep.randomizer.register("color_randomizer")
    def color_randomizer(prim_paths: list):
        """Randomize object colors."""
        with rep.get.prims(path_pattern=prim_paths):
            rep.modify.attribute(
                "primvars:displayColor",
                rep.distribution.uniform((0.1, 0.1, 0.1), (1.0, 1.0, 1.0))
            )
        return None


def apply_randomization_per_episode(world: World, robot: Articulation, config: dict):
    """Apply randomization at the start of each episode."""

    # Physics randomization
    rep.randomizer.physics_randomizer(robot.prim_path, config['physics'])

    # Visual randomization
    with rep.trigger.on_custom_event(event_name="episode_reset"):
        rep.randomizer.lighting_randomizer()
        rep.randomizer.texture_randomizer(["/World/Objects/*"])
        rep.randomizer.color_randomizer(["/World/Objects/*"])


def create_curriculum(difficulty: float) -> dict:
    """Create randomization curriculum based on training progress.

    Args:
        difficulty: Training progress from 0 (easy) to 1 (hard)

    Returns:
        Randomization config dict with ranges scaled by difficulty
    """

    def scale_range(base_range, difficulty):
        """Scale randomization range by difficulty."""
        center = (base_range[0] + base_range[1]) / 2
        half_width = (base_range[1] - base_range[0]) / 2
        scaled_width = half_width * difficulty
        return (center - scaled_width, center + scaled_width)

    return {
        'physics': {
            'mass_scale': scale_range((0.8, 1.2), difficulty),
            'friction': scale_range((0.5, 1.5), difficulty),
            'joint_friction': scale_range((0.01, 0.1), difficulty),
        },
        'visual': {
            'light_intensity': scale_range((300, 2000), difficulty),
        },
        'sensor': {
            'noise_level': 0.01 * difficulty,
        }
    }
```

---

## 3. Reinforcement Learning in Isaac Sim

### 3.1 Isaac Gym / Orbit Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    RL Training Pipeline in Isaac Sim                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                      Isaac Sim / Isaac Lab                           │  │
│   │                                                                      │  │
│   │  ┌──────────────────────────────────────────────────────────────┐  │  │
│   │  │              Parallel Environments (4096+)                    │  │  │
│   │  │                                                               │  │  │
│   │  │  ┌────┐ ┌────┐ ┌────┐ ┌────┐     ┌────┐ ┌────┐ ┌────┐      │  │  │
│   │  │  │Env1│ │Env2│ │Env3│ │Env4│ ... │Env │ │Env │ │Env │      │  │  │
│   │  │  │    │ │    │ │    │ │    │     │4094│ │4095│ │4096│      │  │  │
│   │  │  └────┘ └────┘ └────┘ └────┘     └────┘ └────┘ └────┘      │  │  │
│   │  │                                                               │  │  │
│   │  │  All environments step in parallel on GPU                     │  │  │
│   │  └──────────────────────────────────────────────────────────────┘  │  │
│   │                              │                                      │  │
│   │                              ▼                                      │  │
│   │  ┌──────────────────────────────────────────────────────────────┐  │  │
│   │  │            Observations (GPU Tensor)                          │  │  │
│   │  │  [batch_size x obs_dim] = [4096 x 48]                        │  │  │
│   │  └──────────────────────────────────────────────────────────────┘  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                        │                                    │
│                                        ▼                                    │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                      RL Algorithm (PPO/SAC)                          │  │
│   │                                                                      │  │
│   │  ┌──────────────┐        ┌──────────────┐        ┌──────────────┐  │  │
│   │  │   Policy     │        │    Value     │        │  Experience  │  │  │
│   │  │   Network    │        │   Network    │        │   Buffer     │  │  │
│   │  │  (Actor)     │        │  (Critic)    │        │              │  │  │
│   │  └──────────────┘        └──────────────┘        └──────────────┘  │  │
│   │                                                                      │  │
│   │  All neural network ops run on same GPU as simulation               │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                        │                                    │
│                                        ▼                                    │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                      Actions (GPU Tensor)                            │  │
│   │  [batch_size x action_dim] = [4096 x 12]                            │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│   Performance: 100,000+ FPS simulation, 1B+ timesteps/hour                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Humanoid Walking Environment

```python
#!/usr/bin/env python3
"""
humanoid_walking_env.py - Humanoid walking RL environment
"""

import torch
import numpy as np
from typing import Dict, Tuple

# Isaac Lab imports (successor to Isaac Gym)
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab.managers import ObservationManager, RewardManager, TerminationManager
from omni.isaac.lab.utils import configclass


@configclass
class HumanoidWalkingEnvCfg:
    """Configuration for humanoid walking environment."""

    # Simulation
    sim_dt: float = 0.005  # 200 Hz physics
    decimation: int = 4     # 50 Hz control
    num_envs: int = 4096

    # Robot
    robot_urdf: str = "humanoid.urdf"
    fix_base: bool = False

    # Observation space
    observe_base_lin_vel: bool = True
    observe_base_ang_vel: bool = True
    observe_projected_gravity: bool = True
    observe_joint_pos: bool = True
    observe_joint_vel: bool = True
    observe_actions: bool = True

    # Action space
    action_scale: float = 0.5
    action_clip: float = 1.0

    # Rewards
    reward_tracking_lin_vel: float = 1.0
    reward_tracking_ang_vel: float = 0.5
    reward_lin_vel_z_penalty: float = -2.0
    reward_ang_vel_xy_penalty: float = -0.05
    reward_orientation_penalty: float = -0.5
    reward_torque_penalty: float = -0.00001
    reward_action_rate_penalty: float = -0.01
    reward_feet_air_time: float = 1.0
    reward_stumble_penalty: float = -1.0

    # Termination
    terminate_on_fall: bool = True
    fall_height_threshold: float = 0.3  # meters

    # Domain randomization
    randomize_friction: bool = True
    friction_range: Tuple[float, float] = (0.5, 1.5)
    randomize_mass: bool = True
    mass_range: Tuple[float, float] = (0.8, 1.2)
    push_robots: bool = True
    push_interval: int = 500  # steps
    max_push_vel: float = 0.5  # m/s


class HumanoidWalkingEnv(ManagerBasedRLEnv):
    """Humanoid walking environment with domain randomization."""

    def __init__(self, cfg: HumanoidWalkingEnvCfg):
        super().__init__(cfg)

        # Command tracking
        self.commands = torch.zeros(self.num_envs, 3, device=self.device)
        self.commands[:, 0] = 0.5  # Forward velocity target

        # Previous actions for smoothness penalty
        self.prev_actions = torch.zeros(
            self.num_envs, self.num_actions, device=self.device
        )

        # Feet contact tracking
        self.feet_air_time = torch.zeros(self.num_envs, 2, device=self.device)

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        """Compute observations for all environments."""

        # Base state (from simulation)
        base_quat = self.robot.get_base_orientation()
        base_lin_vel = self.robot.get_base_linear_velocity()
        base_ang_vel = self.robot.get_base_angular_velocity()

        # Project gravity to body frame
        gravity = torch.tensor([0, 0, -1], device=self.device)
        projected_gravity = self._quat_rotate_inverse(base_quat, gravity)

        # Joint state
        joint_pos = self.robot.get_joint_positions()
        joint_vel = self.robot.get_joint_velocities()

        # Normalize joint positions by default position
        joint_pos_normalized = joint_pos - self.default_joint_pos

        # Construct observation tensor
        obs = torch.cat([
            base_lin_vel * self.cfg.lin_vel_scale,           # 3
            base_ang_vel * self.cfg.ang_vel_scale,           # 3
            projected_gravity,                                # 3
            self.commands * self.cfg.commands_scale,         # 3
            joint_pos_normalized * self.cfg.dof_pos_scale,   # num_joints
            joint_vel * self.cfg.dof_vel_scale,              # num_joints
            self.prev_actions,                               # num_actions
        ], dim=-1)

        return {"policy": obs}

    def _compute_rewards(self) -> torch.Tensor:
        """Compute rewards for all environments."""

        # Get current state
        base_lin_vel = self.robot.get_base_linear_velocity()
        base_ang_vel = self.robot.get_base_angular_velocity()
        base_quat = self.robot.get_base_orientation()
        torques = self.robot.get_applied_joint_efforts()

        # Tracking rewards
        lin_vel_error = torch.sum(
            torch.square(self.commands[:, :2] - base_lin_vel[:, :2]), dim=1
        )
        lin_vel_reward = torch.exp(-lin_vel_error / 0.25) * self.cfg.reward_tracking_lin_vel

        ang_vel_error = torch.square(self.commands[:, 2] - base_ang_vel[:, 2])
        ang_vel_reward = torch.exp(-ang_vel_error / 0.25) * self.cfg.reward_tracking_ang_vel

        # Penalty terms
        lin_vel_z_penalty = torch.square(base_lin_vel[:, 2]) * self.cfg.reward_lin_vel_z_penalty
        ang_vel_xy_penalty = torch.sum(
            torch.square(base_ang_vel[:, :2]), dim=1
        ) * self.cfg.reward_ang_vel_xy_penalty

        # Orientation penalty (stay upright)
        gravity_proj = self._quat_rotate_inverse(
            base_quat, torch.tensor([0, 0, -1], device=self.device)
        )
        orientation_penalty = torch.sum(
            torch.square(gravity_proj[:, :2]), dim=1
        ) * self.cfg.reward_orientation_penalty

        # Energy penalty
        torque_penalty = torch.sum(torch.square(torques), dim=1) * self.cfg.reward_torque_penalty

        # Action smoothness penalty
        action_rate_penalty = torch.sum(
            torch.square(self.actions - self.prev_actions), dim=1
        ) * self.cfg.reward_action_rate_penalty

        # Feet air time reward (encourage gait)
        feet_contact = self._get_feet_contact()
        self._update_feet_air_time(feet_contact)
        feet_air_reward = self._compute_feet_air_time_reward()

        # Total reward
        reward = (
            lin_vel_reward +
            ang_vel_reward +
            lin_vel_z_penalty +
            ang_vel_xy_penalty +
            orientation_penalty +
            torque_penalty +
            action_rate_penalty +
            feet_air_reward
        )

        return reward

    def _check_termination(self) -> Tuple[torch.Tensor, torch.Tensor]:
        """Check termination conditions for all environments."""

        # Get base height
        base_pos = self.robot.get_base_position()
        base_height = base_pos[:, 2]

        # Fall detection
        fallen = base_height < self.cfg.fall_height_threshold

        # Episode timeout
        timeout = self.episode_length_buf >= self.cfg.max_episode_length

        return fallen, timeout

    def _apply_actions(self, actions: torch.Tensor):
        """Apply actions to robot."""

        # Scale and clip actions
        scaled_actions = actions * self.cfg.action_scale
        clipped_actions = torch.clamp(
            scaled_actions, -self.cfg.action_clip, self.cfg.action_clip
        )

        # Convert to joint position targets
        target_positions = self.default_joint_pos + clipped_actions

        # Apply to robot
        self.robot.set_joint_position_targets(target_positions)

        # Store for action rate penalty
        self.prev_actions = actions.clone()

    def _apply_domain_randomization(self):
        """Apply domain randomization at episode reset."""

        if self.cfg.randomize_friction:
            friction = torch.empty(self.num_envs, device=self.device).uniform_(
                *self.cfg.friction_range
            )
            self._set_ground_friction(friction)

        if self.cfg.randomize_mass:
            mass_scale = torch.empty(self.num_envs, device=self.device).uniform_(
                *self.cfg.mass_range
            )
            self._scale_robot_mass(mass_scale)

    def _push_robots(self):
        """Apply random pushes to robots during training."""

        if self.cfg.push_robots:
            push_mask = (self.common_step_counter % self.cfg.push_interval) == 0
            if push_mask.any():
                push_vel = torch.empty(
                    self.num_envs, 3, device=self.device
                ).uniform_(-self.cfg.max_push_vel, self.cfg.max_push_vel)
                push_vel[~push_mask] = 0
                self.robot.set_base_linear_velocity(
                    self.robot.get_base_linear_velocity() + push_vel
                )

    @staticmethod
    def _quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
        """Rotate vector by inverse of quaternion."""
        q_w = q[:, 3:4]
        q_vec = q[:, :3]
        a = v * (2.0 * q_w ** 2 - 1.0)
        b = torch.cross(q_vec, v, dim=-1) * q_w * 2.0
        c = q_vec * torch.sum(q_vec * v, dim=-1, keepdim=True) * 2.0
        return a - b + c
```

### 3.3 Training Script

```python
#!/usr/bin/env python3
"""
train_humanoid.py - Train humanoid walking policy with PPO
"""

import torch
import numpy as np
from datetime import datetime
import os

# RL algorithm (using rl_games or stable-baselines3)
from rl_games.algos_torch import players
from rl_games.common import env_configurations, vecenv
from rl_games.torch_runner import Runner

from humanoid_walking_env import HumanoidWalkingEnv, HumanoidWalkingEnvCfg


def create_training_config():
    """Create training configuration."""

    config = {
        "params": {
            "seed": 42,
            "algo": {
                "name": "a2c_continuous"
            },
            "model": {
                "name": "continuous_a2c_logstd"
            },
            "network": {
                "name": "actor_critic",
                "separate": False,
                "space": {
                    "continuous": {
                        "mu_activation": "None",
                        "sigma_activation": "None",
                        "mu_init": {"name": "default"},
                        "sigma_init": {"name": "const_initializer", "val": 0.0},
                        "fixed_sigma": True
                    }
                },
                "mlp": {
                    "units": [512, 256, 128],
                    "activation": "elu",
                    "d2rl": False,
                    "initializer": {"name": "default"},
                    "regularizer": {"name": "None"}
                }
            },
            "load_checkpoint": False,
            "load_path": None,
            "config": {
                "name": "HumanoidWalking",
                "full_experiment_name": "humanoid_walking",
                "env_name": "humanoid_walking",
                "multi_gpu": False,
                "ppo": True,
                "mixed_precision": True,
                "normalize_input": True,
                "normalize_value": True,
                "value_bootstrap": True,
                "num_actors": 4096,
                "reward_shaper": {
                    "scale_value": 1.0
                },
                "normalize_advantage": True,
                "gamma": 0.99,
                "tau": 0.95,
                "learning_rate": 3e-4,
                "lr_schedule": "adaptive",
                "kl_threshold": 0.008,
                "score_to_win": 20000,
                "max_epochs": 5000,
                "save_best_after": 100,
                "save_frequency": 50,
                "grad_norm": 1.0,
                "entropy_coef": 0.0,
                "truncate_grads": True,
                "e_clip": 0.2,
                "horizon_length": 24,
                "minibatch_size": 32768,
                "mini_epochs": 5,
                "critic_coef": 2,
                "clip_value": True,
                "seq_length": 4,
                "bounds_loss_coef": 0.001
            }
        }
    }

    return config


def train():
    """Main training loop."""

    # Create environment config
    env_cfg = HumanoidWalkingEnvCfg(
        num_envs=4096,
        sim_dt=0.005,
        decimation=4,
    )

    # Register environment
    env_configurations.register(
        "humanoid_walking",
        {
            "vecenv_type": "ISAAC",
            "env_creator": lambda **kwargs: HumanoidWalkingEnv(env_cfg),
        }
    )

    # Create training config
    config = create_training_config()

    # Setup logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = f"runs/humanoid_walking_{timestamp}"
    os.makedirs(log_dir, exist_ok=True)

    config["params"]["config"]["log_dir"] = log_dir
    config["params"]["config"]["train_dir"] = log_dir

    # Create runner and train
    runner = Runner()
    runner.load(config)
    runner.run({
        "train": True,
        "play": False,
        "checkpoint": None,
        "sigma": None,
    })


def evaluate(checkpoint_path: str, num_episodes: int = 100):
    """Evaluate trained policy."""

    # Load checkpoint
    player = players.PpoPlayerContinuous(...)
    player.restore(checkpoint_path)

    # Create environment
    env_cfg = HumanoidWalkingEnvCfg(num_envs=1)
    env = HumanoidWalkingEnv(env_cfg)

    # Evaluation loop
    successes = 0
    total_rewards = []

    for ep in range(num_episodes):
        obs = env.reset()
        done = False
        episode_reward = 0

        while not done:
            action = player.get_action(obs, is_determenistic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward

        total_rewards.append(episode_reward)
        if info.get("success", False):
            successes += 1

    print(f"Success rate: {successes / num_episodes * 100:.1f}%")
    print(f"Mean reward: {np.mean(total_rewards):.2f} ± {np.std(total_rewards):.2f}")


if __name__ == "__main__":
    train()
```

---

## 4. Model Export and Deployment

### 4.1 ONNX Export

```python
#!/usr/bin/env python3
"""
export_policy.py - Export trained policy to ONNX for deployment
"""

import torch
import torch.onnx
import numpy as np


class PolicyWrapper(torch.nn.Module):
    """Wrapper to export policy network to ONNX."""

    def __init__(self, policy_network, obs_dim: int, action_dim: int):
        super().__init__()
        self.policy = policy_network
        self.obs_dim = obs_dim
        self.action_dim = action_dim

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """Forward pass returning deterministic action."""
        # Get action mean (ignore log_std for deterministic policy)
        action_mean = self.policy.actor(obs)
        return action_mean


def export_to_onnx(
    checkpoint_path: str,
    output_path: str,
    obs_dim: int,
    action_dim: int,
    opset_version: int = 11
):
    """Export PyTorch policy to ONNX format."""

    # Load checkpoint
    checkpoint = torch.load(checkpoint_path, map_location='cpu')
    policy_state = checkpoint['model']

    # Reconstruct policy network (must match training architecture)
    from rl_games.algos_torch.network_builder import NetworkBuilder
    network_config = {
        "mlp": {
            "units": [512, 256, 128],
            "activation": "elu",
        }
    }
    builder = NetworkBuilder()
    policy_network = builder.build("actor_critic", **network_config)
    policy_network.load_state_dict(policy_state)
    policy_network.eval()

    # Create wrapper
    wrapper = PolicyWrapper(policy_network, obs_dim, action_dim)

    # Create dummy input
    dummy_obs = torch.randn(1, obs_dim)

    # Export to ONNX
    torch.onnx.export(
        wrapper,
        dummy_obs,
        output_path,
        export_params=True,
        opset_version=opset_version,
        do_constant_folding=True,
        input_names=['observation'],
        output_names=['action'],
        dynamic_axes={
            'observation': {0: 'batch_size'},
            'action': {0: 'batch_size'}
        }
    )

    print(f"Exported policy to {output_path}")

    # Verify export
    import onnx
    import onnxruntime as ort

    # Check ONNX model
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)

    # Test inference
    session = ort.InferenceSession(output_path)
    test_obs = np.random.randn(1, obs_dim).astype(np.float32)
    result = session.run(['action'], {'observation': test_obs})

    print(f"ONNX inference test - Input shape: {test_obs.shape}, Output shape: {result[0].shape}")

    return output_path


def quantize_onnx(onnx_path: str, output_path: str):
    """Quantize ONNX model for faster inference."""

    from onnxruntime.quantization import quantize_dynamic, QuantType

    quantize_dynamic(
        onnx_path,
        output_path,
        weight_type=QuantType.QUInt8
    )

    print(f"Quantized model saved to {output_path}")


# Example usage
if __name__ == "__main__":
    export_to_onnx(
        checkpoint_path="runs/humanoid_walking/nn/HumanoidWalking.pth",
        output_path="humanoid_policy.onnx",
        obs_dim=48,
        action_dim=12,
    )
```

### 4.2 Jetson Deployment

```python
#!/usr/bin/env python3
"""
jetson_inference.py - Run policy on NVIDIA Jetson with TensorRT
"""

import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time


class TensorRTPolicy:
    """TensorRT-optimized policy for Jetson deployment."""

    def __init__(self, onnx_path: str, fp16: bool = True):
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.engine = self._build_engine(onnx_path, fp16)
        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.inputs, self.outputs, self.bindings = self._allocate_buffers()

        # CUDA stream
        self.stream = cuda.Stream()

    def _build_engine(self, onnx_path: str, fp16: bool):
        """Build TensorRT engine from ONNX model."""

        builder = trt.Builder(self.logger)
        network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        network = builder.create_network(network_flags)
        parser = trt.OnnxParser(network, self.logger)

        # Parse ONNX
        with open(onnx_path, 'rb') as f:
            if not parser.parse(f.read()):
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                raise RuntimeError("ONNX parsing failed")

        # Build config
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB

        if fp16 and builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("Using FP16 precision")

        # Build engine
        engine = builder.build_serialized_network(network, config)
        runtime = trt.Runtime(self.logger)
        return runtime.deserialize_cuda_engine(engine)

    def _allocate_buffers(self):
        """Allocate GPU buffers for inference."""
        inputs = []
        outputs = []
        bindings = []

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            shape = self.engine.get_tensor_shape(name)

            # Allocate host and device buffers
            size = trt.volume(shape)
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            bindings.append(int(device_mem))

            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                inputs.append({'host': host_mem, 'device': device_mem, 'shape': shape})
            else:
                outputs.append({'host': host_mem, 'device': device_mem, 'shape': shape})

        return inputs, outputs, bindings

    def infer(self, observation: np.ndarray) -> np.ndarray:
        """Run inference on observation."""

        # Copy input to host buffer
        np.copyto(self.inputs[0]['host'], observation.ravel())

        # Transfer to GPU
        cuda.memcpy_htod_async(
            self.inputs[0]['device'],
            self.inputs[0]['host'],
            self.stream
        )

        # Run inference
        self.context.execute_async_v2(
            bindings=self.bindings,
            stream_handle=self.stream.handle
        )

        # Transfer output back
        cuda.memcpy_dtoh_async(
            self.outputs[0]['host'],
            self.outputs[0]['device'],
            self.stream
        )

        # Synchronize
        self.stream.synchronize()

        return self.outputs[0]['host'].reshape(self.outputs[0]['shape'])

    def benchmark(self, num_iterations: int = 1000) -> dict:
        """Benchmark inference latency."""

        dummy_obs = np.random.randn(1, 48).astype(np.float32)

        # Warmup
        for _ in range(100):
            self.infer(dummy_obs)

        # Benchmark
        latencies = []
        for _ in range(num_iterations):
            start = time.perf_counter()
            self.infer(dummy_obs)
            latencies.append((time.perf_counter() - start) * 1000)  # ms

        return {
            'mean_ms': np.mean(latencies),
            'std_ms': np.std(latencies),
            'min_ms': np.min(latencies),
            'max_ms': np.max(latencies),
            'fps': 1000 / np.mean(latencies)
        }


class RobotController:
    """Real-time robot control with TensorRT policy."""

    def __init__(self, policy_path: str, control_freq: float = 50.0):
        self.policy = TensorRTPolicy(policy_path, fp16=True)
        self.control_period = 1.0 / control_freq
        self.obs_dim = 48
        self.action_dim = 12

        # Observation normalization (from training)
        self.obs_mean = np.zeros(self.obs_dim, dtype=np.float32)
        self.obs_std = np.ones(self.obs_dim, dtype=np.float32)

    def set_normalization(self, mean: np.ndarray, std: np.ndarray):
        """Set observation normalization parameters."""
        self.obs_mean = mean.astype(np.float32)
        self.obs_std = std.astype(np.float32)

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        """Get action from policy."""

        # Normalize observation
        obs_normalized = (observation - self.obs_mean) / (self.obs_std + 1e-8)
        obs_normalized = obs_normalized.astype(np.float32).reshape(1, -1)

        # Inference
        action = self.policy.infer(obs_normalized)

        return action.flatten()

    def run_control_loop(self, robot_interface):
        """Main control loop."""

        print(f"Starting control loop at {1.0/self.control_period:.0f} Hz")
        benchmark = self.policy.benchmark()
        print(f"Policy latency: {benchmark['mean_ms']:.2f} ± {benchmark['std_ms']:.2f} ms")

        try:
            while True:
                loop_start = time.perf_counter()

                # Get robot state
                observation = robot_interface.get_observation()

                # Compute action
                action = self.get_action(observation)

                # Send to robot
                robot_interface.send_action(action)

                # Maintain control frequency
                elapsed = time.perf_counter() - loop_start
                sleep_time = self.control_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: Control loop overrun by {-sleep_time*1000:.1f} ms")

        except KeyboardInterrupt:
            print("Control loop stopped")


# Example usage
if __name__ == "__main__":
    # Benchmark TensorRT policy
    policy = TensorRTPolicy("humanoid_policy.onnx", fp16=True)
    results = policy.benchmark()

    print(f"\nTensorRT Benchmark Results:")
    print(f"  Mean latency: {results['mean_ms']:.2f} ms")
    print(f"  Std latency: {results['std_ms']:.2f} ms")
    print(f"  Min latency: {results['min_ms']:.2f} ms")
    print(f"  Max latency: {results['max_ms']:.2f} ms")
    print(f"  Throughput: {results['fps']:.0f} FPS")
```

---

## 5. Evaluation Metrics

### 5.1 Transfer Performance Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| **Success Rate** | Task completion percentage | > 80% |
| **Transfer Gap** | Sim vs Real performance drop | < 20% |
| **Sample Efficiency** | Real-world samples needed | Minimal (zero-shot ideal) |
| **Robustness** | Performance under perturbation | Graceful degradation |
| **Inference Latency** | Policy computation time | < 10ms |

### 5.2 Evaluation Script

```python
#!/usr/bin/env python3
"""
evaluate_transfer.py - Evaluate sim-to-real transfer performance
"""

import numpy as np
from typing import Dict, List
from dataclasses import dataclass


@dataclass
class TransferMetrics:
    """Metrics for sim-to-real transfer evaluation."""
    sim_success_rate: float
    real_success_rate: float
    transfer_gap: float
    sim_reward: float
    real_reward: float
    reward_gap: float
    robustness_score: float
    inference_latency_ms: float


def evaluate_transfer(
    sim_results: List[Dict],
    real_results: List[Dict],
    latency_ms: float
) -> TransferMetrics:
    """Compute transfer metrics from simulation and real-world results."""

    # Success rates
    sim_successes = sum(1 for r in sim_results if r['success'])
    real_successes = sum(1 for r in real_results if r['success'])

    sim_success_rate = sim_successes / len(sim_results)
    real_success_rate = real_successes / len(real_results)

    # Transfer gap (normalized difference)
    transfer_gap = (sim_success_rate - real_success_rate) / max(sim_success_rate, 1e-6)

    # Rewards
    sim_reward = np.mean([r['total_reward'] for r in sim_results])
    real_reward = np.mean([r['total_reward'] for r in real_results])
    reward_gap = (sim_reward - real_reward) / max(abs(sim_reward), 1e-6)

    # Robustness (variance in real-world performance)
    real_reward_std = np.std([r['total_reward'] for r in real_results])
    robustness_score = 1.0 / (1.0 + real_reward_std / max(abs(real_reward), 1e-6))

    return TransferMetrics(
        sim_success_rate=sim_success_rate,
        real_success_rate=real_success_rate,
        transfer_gap=transfer_gap,
        sim_reward=sim_reward,
        real_reward=real_reward,
        reward_gap=reward_gap,
        robustness_score=robustness_score,
        inference_latency_ms=latency_ms,
    )


def analyze_failure_modes(real_results: List[Dict]) -> Dict[str, int]:
    """Analyze failure modes in real-world experiments."""

    failure_modes = {
        'fall': 0,
        'timeout': 0,
        'collision': 0,
        'motor_fault': 0,
        'tracking_lost': 0,
        'unknown': 0,
    }

    for result in real_results:
        if not result['success']:
            mode = result.get('failure_mode', 'unknown')
            if mode in failure_modes:
                failure_modes[mode] += 1
            else:
                failure_modes['unknown'] += 1

    return failure_modes


def generate_report(metrics: TransferMetrics, failure_modes: Dict[str, int]) -> str:
    """Generate human-readable evaluation report."""

    report = """
================================================================================
                        Sim-to-Real Transfer Evaluation Report
================================================================================

PERFORMANCE METRICS
-------------------
Simulation Success Rate:  {:.1f}%
Real-World Success Rate:  {:.1f}%
Transfer Gap:             {:.1f}%

Simulation Reward:        {:.2f}
Real-World Reward:        {:.2f}
Reward Gap:               {:.1f}%

Robustness Score:         {:.2f} / 1.00
Inference Latency:        {:.2f} ms

FAILURE ANALYSIS
----------------
""".format(
        metrics.sim_success_rate * 100,
        metrics.real_success_rate * 100,
        metrics.transfer_gap * 100,
        metrics.sim_reward,
        metrics.real_reward,
        metrics.reward_gap * 100,
        metrics.robustness_score,
        metrics.inference_latency_ms,
    )

    total_failures = sum(failure_modes.values())
    if total_failures > 0:
        for mode, count in sorted(failure_modes.items(), key=lambda x: -x[1]):
            if count > 0:
                pct = count / total_failures * 100
                report += f"  {mode:20s}: {count:3d} ({pct:5.1f}%)\n"
    else:
        report += "  No failures recorded\n"

    report += """
RECOMMENDATIONS
---------------
"""

    if metrics.transfer_gap > 0.2:
        report += "- High transfer gap detected. Consider:\n"
        report += "  * Increasing domain randomization range\n"
        report += "  * Adding system identification for physics parameters\n"
        report += "  * Implementing adaptive domain randomization\n"

    if failure_modes.get('fall', 0) > total_failures * 0.3:
        report += "- Frequent falls detected. Consider:\n"
        report += "  * Training with more aggressive perturbations\n"
        report += "  * Adding recovery behaviors\n"

    if metrics.inference_latency_ms > 10:
        report += "- High inference latency. Consider:\n"
        report += "  * Using INT8 quantization\n"
        report += "  * Reducing network size\n"
        report += "  * Optimizing TensorRT engine\n"

    report += "\n" + "=" * 80 + "\n"

    return report


# Example usage
if __name__ == "__main__":
    # Simulated results (replace with actual data)
    sim_results = [
        {'success': True, 'total_reward': 150.0} for _ in range(80)
    ] + [
        {'success': False, 'total_reward': 50.0, 'failure_mode': 'timeout'} for _ in range(20)
    ]

    real_results = [
        {'success': True, 'total_reward': 120.0} for _ in range(60)
    ] + [
        {'success': False, 'total_reward': 30.0, 'failure_mode': 'fall'} for _ in range(25)
    ] + [
        {'success': False, 'total_reward': 40.0, 'failure_mode': 'collision'} for _ in range(15)
    ]

    metrics = evaluate_transfer(sim_results, real_results, latency_ms=5.2)
    failure_modes = analyze_failure_modes(real_results)
    report = generate_report(metrics, failure_modes)
    print(report)
```

---

## 6. Summary

### Sim-to-Real Techniques

| Technique | Approach | When to Use |
|-----------|----------|-------------|
| **Domain Randomization** | Train on varied simulations | Always (baseline) |
| **System Identification** | Match sim to real parameters | When possible to measure |
| **Adaptive DR** | Adjust randomization during training | Large distribution shift |
| **Fine-tuning** | Update policy with real data | When safe to collect |
| **Curriculum Learning** | Gradually increase difficulty | Complex tasks |

### Deployment Checklist

- [ ] Export policy to ONNX format
- [ ] Verify ONNX model correctness
- [ ] Build TensorRT engine with FP16
- [ ] Benchmark inference latency (< 10ms target)
- [ ] Implement observation normalization
- [ ] Test control loop timing
- [ ] Add safety monitoring
- [ ] Implement emergency stop

---

## Exercises

### Exercise 10.1: Domain Randomization Design (⭐⭐)

For a humanoid walking task, design randomization ranges for:
- Physics parameters (mass, friction, damping)
- Sensor noise (IMU, encoders)
- Visual appearance (if using vision)

Justify your choices with reasoning about real-world variation.

### Exercise 10.2: Training Pipeline (⭐⭐⭐)

Implement a complete training pipeline:
1. Create environment with domain randomization
2. Train PPO policy for 1000 epochs
3. Log training curves (reward, success rate)
4. Export best checkpoint to ONNX

### Exercise 10.3: Deployment Test (⭐⭐⭐)

Deploy your trained policy:
1. Build TensorRT engine on Jetson
2. Benchmark inference latency
3. Implement control loop with timing guarantees
4. Test in simulation with real-time constraints

### Exercise 10.4: Transfer Evaluation (⭐⭐)

Evaluate your policy:
1. Run 100 episodes in simulation
2. Run 100 episodes in a different simulation (altered parameters)
3. Compute transfer metrics
4. Analyze failure modes
5. Propose improvements

---

## Quiz

<details>
<summary>Q1: Why does domain randomization help with sim-to-real transfer?</summary>

Domain randomization helps because:
1. It exposes the policy to a wide distribution of scenarios during training
2. The policy learns features that are invariant across variations
3. Reality becomes "just another sample" from the training distribution
4. The policy cannot overfit to simulation-specific details
5. Robust features transfer better than brittle, simulation-specific ones

</details>

<details>
<summary>Q2: What are the key sources of the sim-to-real gap?</summary>

Key sources include:
1. **Physics mismatch**: Contact dynamics, actuator response, mass/inertia errors
2. **Sensor discrepancies**: Noise, bias drift, latency, calibration errors
3. **Visual differences**: Lighting, textures, backgrounds, reflections
4. **Unmodeled dynamics**: Air resistance, cable forces, wear effects
5. **Environmental factors**: Temperature, external disturbances

</details>

<details>
<summary>Q3: How do you optimize a policy for Jetson deployment?</summary>

Optimization steps:
1. Export to ONNX with proper opset version
2. Build TensorRT engine with FP16 (or INT8 for maximum speed)
3. Use CUDA streams for async memory transfers
4. Pre-allocate all GPU buffers
5. Avoid Python overhead in control loop (use C++ if needed)
6. Profile and benchmark to verify latency requirements

Target: < 10ms inference for 50Hz+ control loops.

</details>

<details>
<summary>Q4: What metrics should you use to evaluate sim-to-real transfer?</summary>

Key metrics:
1. **Success Rate**: Task completion in real world vs simulation
2. **Transfer Gap**: (Sim - Real) / Sim performance
3. **Robustness Score**: Consistency under perturbations
4. **Failure Mode Analysis**: Understanding why real-world fails
5. **Sample Efficiency**: Real-world data needed for adaptation
6. **Inference Latency**: Computational requirements on target hardware

</details>

---

## Module 3 Assessment

**Deliverable**: Complete Isaac Sim perception and training pipeline:

1. **Isaac Sim World**: Custom environment with robot and obstacles
2. **Synthetic Data Pipeline**: Generate 1000 annotated images
3. **VSLAM Integration**: cuVSLAM running with custom robot
4. **RL Training**: Train walking policy with domain randomization
5. **Deployment**: Export and benchmark on target platform

See [Week 14: Capstone](/assessments/week-14-capstone) for full requirements.

---

## Next Steps

In [Week 11: Humanoid Kinematics](/module-4-vla/week-11-humanoid-kinematics), we begin Module 4:
- Humanoid robot kinematic chains
- Forward and inverse kinematics
- Motion planning for manipulation
- Whole-body control concepts
