---
sidebar_position: 3
---

# Week 10: Sim-to-Real Transfer

## Learning Objectives

By the end of this week, you will be able to:

- Understand the Sim-to-Real gap and its causes
- Apply domain randomization techniques
- Train policies in simulation and deploy to hardware
- Evaluate transfer performance

## The Sim-to-Real Gap

Simulation differs from reality in:

| Aspect | Simulation | Reality |
|--------|------------|---------|
| **Physics** | Approximated | Complex, uncertain |
| **Sensors** | Idealized | Noisy, delayed |
| **Visuals** | Rendered | Natural lighting |
| **Dynamics** | Estimated | Varies with wear |

```
┌─────────────────────────────────────────────────────────────┐
│                      The Sim-to-Real Gap                     │
│                                                              │
│    Simulation          │            Reality                  │
│    ──────────          │            ───────                  │
│    • Perfect physics   │   • Unmodeled dynamics              │
│    • Clean sensors     │   • Sensor noise/drift              │
│    • Instant reset     │   • Slow, careful reset             │
│    • Parallel envs     │   • Single robot                    │
│                        │                                     │
│         Policy trained here ───► Must work here              │
└─────────────────────────────────────────────────────────────┘
```

## Domain Randomization

**Key Idea**: If trained on diverse simulations, the policy treats reality as "just another variation."

### What to Randomize

| Category | Parameters | Range |
|----------|------------|-------|
| **Visual** | Textures, lighting, colors | Wide |
| **Physical** | Mass, friction, damping | ±20% |
| **Sensor** | Noise, latency, bias | Realistic |
| **Environment** | Object positions, obstacles | Task-relevant |

### Implementation in Isaac Sim

```python
import omni.replicator.core as rep

def randomize_domain():
    # Visual randomization
    with rep.trigger.on_frame():
        # Randomize lighting
        rep.randomizer.light(
            intensity=rep.distribution.uniform(300, 1500),
            temperature=rep.distribution.uniform(4000, 7000)
        )

        # Randomize textures
        rep.randomizer.texture(
            textures=rep.distribution.choice([
                "omni://textures/wood/*",
                "omni://textures/metal/*",
                "omni://textures/concrete/*",
            ])
        )

def randomize_physics(robot):
    # Mass randomization
    for link in robot.links:
        original_mass = link.mass
        link.mass = original_mass * np.random.uniform(0.8, 1.2)

    # Friction randomization
    for joint in robot.joints:
        joint.friction = np.random.uniform(0.01, 0.1)
        joint.damping = np.random.uniform(0.001, 0.01)
```

## Reinforcement Learning Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    Sim-to-Real Training Pipeline                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌──────────────┐     ┌──────────────┐     ┌──────────────┐   │
│   │ Isaac Sim    │────►│ RL Training  │────►│ Policy       │   │
│   │ (Parallel)   │     │ (PPO/SAC)    │     │ Network      │   │
│   └──────────────┘     └──────────────┘     └──────┬───────┘   │
│         ▲                                          │            │
│         │                                          │            │
│   Domain Randomization                             ▼            │
│                                              ┌──────────────┐   │
│                                              │ Export ONNX  │   │
│                                              └──────┬───────┘   │
│                                                     │            │
│                                                     ▼            │
│                                              ┌──────────────┐   │
│                                              │ Jetson/Robot │   │
│                                              │ Deployment   │   │
│                                              └──────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Training with Isaac Gym

```python
from isaacgym import gymapi, gymtorch
import torch

class HumanoidEnv:
    def __init__(self, num_envs=4096):
        self.gym = gymapi.acquire_gym()
        self.num_envs = num_envs

        # Create parallel environments
        self.envs = []
        for i in range(num_envs):
            env = self.gym.create_env(...)
            self.envs.append(env)

    def step(self, actions):
        # Apply actions to all environments
        self.gym.set_dof_actuation_force_tensor(
            self.sim,
            gymtorch.unwrap_tensor(actions)
        )

        # Simulate
        self.gym.simulate(self.sim)

        # Get observations
        obs = self._get_observations()
        rewards = self._compute_rewards()
        dones = self._check_termination()

        return obs, rewards, dones, {}

    def _compute_rewards(self):
        # Reward for staying upright
        upright_reward = torch.exp(-torch.abs(self.torso_angle))

        # Reward for forward velocity
        velocity_reward = self.root_velocity[:, 0]

        # Penalty for energy consumption
        energy_penalty = -0.01 * torch.sum(self.actions ** 2, dim=-1)

        return upright_reward + velocity_reward + energy_penalty
```

## Deploying to Jetson

```python
# Export trained policy
torch.onnx.export(
    policy_network,
    dummy_input,
    "humanoid_policy.onnx",
    opset_version=11
)

# On Jetson: Load with TensorRT
import tensorrt as trt
import pycuda.driver as cuda

class PolicyInference:
    def __init__(self, onnx_path):
        # Build TensorRT engine
        logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(logger)
        network = builder.create_network(...)

        # Parse ONNX
        parser = trt.OnnxParser(network, logger)
        parser.parse_from_file(onnx_path)

        # Build engine
        self.engine = builder.build_cuda_engine(network)

    def infer(self, observation):
        # Run inference at 100+ Hz
        ...
```

## Evaluation Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| **Success Rate** | Task completion % | \>80% |
| **Transfer Gap** | Sim vs Real performance | \<20% drop |
| **Robustness** | Performance under perturbation | Graceful degradation |
| **Latency** | Inference time | \<10ms |

## Exercises

1. Implement domain randomization for a simple task
2. Train a policy in Isaac Sim (use provided baseline)
3. Export policy to ONNX format
4. Measure inference speed on Jetson

## Assessment: Isaac Perception Pipeline

Create a complete perception system. See [Week 14: Capstone](/assessments/week-14-capstone) for requirements.

## Next Steps

In [Week 11](/module-4-vla/week-11-humanoid-kinematics), we enter Module 4 and explore humanoid kinematics.
