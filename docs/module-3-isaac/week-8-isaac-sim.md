---
sidebar_position: 1
---

# Week 8: NVIDIA Isaac Sim

## Learning Objectives

By the end of this week, you will be able to:

- Install and configure NVIDIA Isaac Sim
- Understand the Omniverse platform architecture
- Create photorealistic simulation environments
- Generate synthetic training data

## What is Isaac Sim?

**NVIDIA Isaac Sim** is a scalable robotics simulation platform built on Omniverse:

- **Photorealistic rendering** via RTX ray tracing
- **Accurate physics** via PhysX 5
- **Domain randomization** for robust AI training
- **Synthetic data generation** for perception models

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **GPU** | RTX 3070 (8GB) | RTX 4090 (24GB) |
| **CPU** | Intel i7 / Ryzen 7 | Intel i9 / Ryzen 9 |
| **RAM** | 32 GB | 64 GB |
| **Storage** | 50 GB SSD | 100 GB NVMe |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 |

## Installation

### Via Omniverse Launcher

```bash
# 1. Download Omniverse Launcher
# https://www.nvidia.com/en-us/omniverse/

# 2. Install Isaac Sim from Exchange tab

# 3. Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

### Verify Installation

```bash
# Run sample scene
cd ~/.local/share/ov/pkg/isaac_sim-*
./python.sh standalone_examples/api/omni.isaac.core/hello_world.py
```

## Isaac Sim Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Isaac Sim Application                 │
├─────────────────────────────────────────────────────────┤
│    Extensions (Isaac ROS, Replicator, Cortex, etc.)     │
├─────────────────────────────────────────────────────────┤
│                     Omniverse Kit                        │
├──────────────────┬────────────────┬─────────────────────┤
│   RTX Renderer   │   PhysX 5.0    │   USD (OpenUSD)     │
└──────────────────┴────────────────┴─────────────────────┘
```

## USD: Universal Scene Description

Isaac Sim uses **USD** (developed by Pixar) for scene description:

```python
from pxr import Usd, UsdGeom, Gf

# Create a new stage
stage = Usd.Stage.CreateNew('humanoid_scene.usda')

# Add a robot
robot_prim = stage.DefinePrim('/World/Robot', 'Xform')
UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3f(0, 0, 1))

# Save
stage.GetRootLayer().Save()
```

## Creating a Simulation Environment

### Python API

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Create world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0, 0, 1.0]),
        size=np.array([0.5, 0.5, 0.5]),
        color=np.array([1.0, 0, 0]),
    )
)

# Run simulation
world.reset()
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

## Importing Robot Models

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

# Import URDF
from omni.importer.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()

# Import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False

# Import robot
result, prim_path = urdf_interface.parse_urdf(
    "/path/to/robot.urdf",
    import_config
)
```

## Synthetic Data Generation

### Replicator for Domain Randomization

```python
import omni.replicator.core as rep

# Randomize lighting
with rep.trigger.on_frame():
    rep.randomizer.light(
        intensity=rep.distribution.uniform(500, 2000),
        color=rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1))
    )

# Randomize textures
with rep.trigger.on_frame():
    rep.randomizer.texture(
        textures=["metal", "wood", "concrete"],
        input_prims="/World/Environment/*"
    )
```

## Exercises

1. Install Isaac Sim via Omniverse Launcher
2. Run the hello_world example
3. Create a scene with ground plane, lighting, and a cube
4. Import a URDF robot model into Isaac Sim

## Next Steps

In [Week 9](/module-3-isaac/week-9-perception-vslam), we explore Isaac ROS for perception and VSLAM.
