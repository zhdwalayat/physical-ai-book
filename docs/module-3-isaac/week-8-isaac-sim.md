---
sidebar_position: 1
---

# Week 8: NVIDIA Isaac Sim

## Learning Objectives

By the end of this week, you will be able to:

- **Install** and configure NVIDIA Isaac Sim via Omniverse
- **Understand** the Omniverse platform architecture and USD format
- **Create** photorealistic simulation environments with Python API
- **Import** URDF robot models and configure articulations
- **Generate** synthetic training data using Replicator

---

## 1. Introduction to Isaac Sim

### 1.1 What is Isaac Sim?

**NVIDIA Isaac Sim** is a scalable, photorealistic robotics simulation platform built on **Omniverse**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Isaac Sim Capabilities                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐  │
│    │  Photorealistic    │  │   Accurate         │  │   Synthetic Data   │  │
│    │  Rendering         │  │   Physics          │  │   Generation       │  │
│    │  ─────────────     │  │   ─────────        │  │   ────────────     │  │
│    │  • RTX Ray Tracing │  │  • PhysX 5.1       │  │  • Replicator      │  │
│    │  • Path Tracing    │  │  • GPU-accelerated │  │  • Domain Random.  │  │
│    │  • Real-time GI    │  │  • Soft body       │  │  • Annotation      │  │
│    │  • PBR Materials   │  │  • Fluids, cloth   │  │  • Multi-sensor    │  │
│    └────────────────────┘  └────────────────────┘  └────────────────────┘  │
│                                                                              │
│    ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐  │
│    │   ROS 2            │  │   RL Training      │  │   Cloud            │  │
│    │   Integration      │  │   Support          │  │   Deployment       │  │
│    │   ─────────────    │  │   ────────────     │  │   ────────────     │  │
│    │  • Isaac ROS       │  │  • Isaac Gym       │  │  • AWS/GCP/Azure   │  │
│    │  • Native bridges  │  │  • Parallel envs   │  │  • Kubernetes      │  │
│    │  • Action graphs   │  │  • PyTorch/JAX     │  │  • OV Farm         │  │
│    └────────────────────┘  └────────────────────┘  └────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Why Isaac Sim for Physical AI?

| Feature | Gazebo | Isaac Sim | Impact |
|---------|--------|-----------|--------|
| **Rendering** | OGRE (rasterization) | RTX (ray tracing) | Camera-based AI training |
| **Physics** | ODE/Bullet (CPU) | PhysX (GPU) | 10-100x faster RL training |
| **Parallel Envs** | Limited | 4096+ environments | Massive RL scaling |
| **Sensor Noise** | Basic models | Physically-based | Better sim-to-real |
| **Data Generation** | Manual | Replicator automation | ML dataset creation |

### 1.3 Isaac Sim Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Isaac Sim Architecture                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                     Isaac Sim Application                              │  │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────────┐ │  │
│  │  │ Isaac ROS   │ │ Replicator  │ │   Cortex    │ │ Isaac Gym/Orbit │ │  │
│  │  │ (ROS 2)     │ │ (Synth Data)│ │ (Behavior)  │ │ (RL Training)   │ │  │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────────┘ │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                      │                                       │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                        Omniverse Kit                                   │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │  │
│  │  │                    Extension System                              │ │  │
│  │  │    omni.kit    omni.usd    omni.physx    omni.graph              │ │  │
│  │  └─────────────────────────────────────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                      │                                       │
│  ┌──────────────────┬────────────────┬───────────────┬──────────────────┐  │
│  │   RTX Renderer   │   PhysX 5.1    │  USD Hydra    │   Carbonite      │  │
│  │   (Rendering)    │   (Physics)    │  (Scene)      │   (Platform)     │  │
│  └──────────────────┴────────────────┴───────────────┴──────────────────┘  │
│                                      │                                       │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                           NVIDIA GPU                                   │  │
│  │         CUDA Cores    │    RT Cores    │    Tensor Cores              │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. System Requirements and Installation

### 2.1 Hardware Requirements

| Component | Minimum | Recommended | Optimal |
|-----------|---------|-------------|---------|
| **GPU** | RTX 3070 (8GB) | RTX 4080 (16GB) | RTX 4090 (24GB) |
| **VRAM** | 8 GB | 16 GB | 24+ GB |
| **CPU** | Intel i7 / Ryzen 7 | Intel i9 / Ryzen 9 | 16+ cores |
| **RAM** | 32 GB | 64 GB | 128 GB |
| **Storage** | 50 GB SSD | 100 GB NVMe | 500 GB NVMe |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 | Ubuntu 22.04 |
| **Driver** | 525.60+ | 535.104+ | Latest |

### 2.2 Installation via Omniverse Launcher

```bash
# Step 1: Download Omniverse Launcher
# Visit: https://www.nvidia.com/en-us/omniverse/download/

# Step 2: Install the launcher
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# Step 3: Install Isaac Sim from Exchange tab
# Search for "Isaac Sim" and click Install

# Step 4: Launch Isaac Sim
~/.local/share/ov/pkg/isaac-sim-*/isaac-sim.sh
```

### 2.3 Container-Based Installation

```bash
# Using NGC Container (recommended for production)
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run with GPU support
docker run --gpus all -it --rm \
    -v ~/isaac-sim-data:/root/.local/share/ov/data \
    -v ~/Documents/isaac-sim:/root/Documents \
    -e "ACCEPT_EULA=Y" \
    nvcr.io/nvidia/isaac-sim:2023.1.1 \
    ./isaac-sim.sh

# Headless mode for training
docker run --gpus all -it --rm \
    -e "ACCEPT_EULA=Y" \
    nvcr.io/nvidia/isaac-sim:2023.1.1 \
    ./python.sh /isaac-sim/standalone_examples/api/omni.isaac.core/hello_world.py
```

### 2.4 Verify Installation

```python
#!/usr/bin/env python3
"""
verify_installation.py - Verify Isaac Sim installation
"""

# Run with: ./python.sh verify_installation.py

from omni.isaac.kit import SimulationApp

# Launch in headless mode for verification
simulation_app = SimulationApp({"headless": True})

# Import after SimulationApp is created
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
import omni.isaac.core.utils.numpy.rotations as rot_utils

# Create world
world = World(stage_units_in_meters=1.0)

# Verify core functionality
print("=" * 50)
print("Isaac Sim Installation Verification")
print("=" * 50)
print(f"Stage units: {world.stage_units_in_meters} meters")
print(f"Physics dt: {world.get_physics_dt()} seconds")
print(f"Rendering dt: {world.get_rendering_dt()} seconds")

# Check PhysX
from omni.physx import get_physx_interface
physx = get_physx_interface()
print(f"PhysX GPU enabled: {physx.is_gpu_dynamics_enabled()}")

# Check USD stage
stage = get_current_stage()
print(f"USD Stage path: {stage.GetRootLayer().identifier}")

print("=" * 50)
print("Installation verified successfully!")
print("=" * 50)

simulation_app.close()
```

---

## 3. USD: Universal Scene Description

### 3.1 USD Fundamentals

**USD** (Universal Scene Description), developed by Pixar, is the foundation of Isaac Sim:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          USD Hierarchy                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    /World                          (Root Prim - Xform)                      │
│    │                                                                        │
│    ├── /World/Environment          (Xform - Container)                      │
│    │   ├── /World/Environment/GroundPlane   (Mesh)                         │
│    │   ├── /World/Environment/Lighting      (DistantLight)                 │
│    │   └── /World/Environment/Sky           (DomeLight)                    │
│    │                                                                        │
│    ├── /World/Robot                (Articulation Root)                      │
│    │   ├── /World/Robot/base_link          (Rigid Body)                    │
│    │   ├── /World/Robot/hip_joint          (Revolute Joint)                │
│    │   ├── /World/Robot/upper_leg          (Rigid Body)                    │
│    │   └── ...                                                              │
│    │                                                                        │
│    └── /World/Objects              (Xform - Container)                      │
│        ├── /World/Objects/Box_01           (Rigid Body)                    │
│        └── /World/Objects/Box_02           (Rigid Body)                    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 USD Concepts

| Concept | Description | Example |
|---------|-------------|---------|
| **Prim** | Basic scene element | `/World/Robot` |
| **Attribute** | Property of a prim | `xformOp:translate` |
| **Schema** | Type definition | `UsdGeom.Mesh`, `UsdPhysics.RigidBodyAPI` |
| **Layer** | File containing prims | `scene.usda`, `robot.usd` |
| **Reference** | Include another USD file | Reference robot.usd at /World/Robot |
| **Variant** | Alternative configurations | `{color: red}`, `{color: blue}` |

### 3.3 Working with USD in Python

```python
#!/usr/bin/env python3
"""
usd_basics.py - USD fundamentals in Isaac Sim
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
import omni.kit.commands

# Get current stage
stage = get_current_stage()

# Create root prim
world_prim = stage.DefinePrim("/World", "Xform")
stage.SetDefaultPrim(world_prim)

# Create a transform hierarchy
environment = UsdGeom.Xform.Define(stage, "/World/Environment")

# Create a mesh (ground plane)
ground = UsdGeom.Mesh.Define(stage, "/World/Environment/Ground")
ground.CreatePointsAttr([
    (-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)
])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
ground.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])

# Add physics to ground (static collider)
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
ground_physics = UsdPhysics.MeshCollisionAPI.Apply(ground.GetPrim())
ground_physics.CreateApproximationAttr("none")  # Use exact mesh

# Create a dynamic cube
cube_path = "/World/Objects/Cube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube.CreateSizeAttr(0.5)

# Set cube position
xform = UsdGeom.Xformable(cube)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2))

# Add rigid body physics
cube_prim = stage.GetPrimAtPath(cube_path)
UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)

# Set mass
mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
mass_api.CreateMassAttr(1.0)  # 1 kg

# Create material for visual appearance
material_path = "/World/Materials/RedMaterial"
material = UsdShade.Material.Define(stage, material_path)
shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((1.0, 0.0, 0.0))
shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# Bind material to cube
UsdShade.MaterialBindingAPI(cube_prim).Bind(material)

# Add lighting
light = UsdLux.DistantLight.Define(stage, "/World/Environment/Sun")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.53)  # Sun angular diameter
xform = UsdGeom.Xformable(light)
xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 0))

# Save stage
stage.Export("/tmp/my_scene.usda")
print("Scene saved to /tmp/my_scene.usda")

# Run simulation
from omni.isaac.core import World
world = World()
world.reset()

for i in range(500):
    world.step(render=True)

simulation_app.close()
```

---

## 4. Creating Simulation Environments

### 4.1 The World Class

```python
#!/usr/bin/env python3
"""
world_setup.py - Creating simulation worlds in Isaac Sim
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "anti_aliasing": 2,  # FXAA
})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, VisualCuboid
from omni.isaac.core.objects import DynamicSphere, DynamicCylinder
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.materials import PreviewSurface


class HumanoidSimulationWorld:
    """Complete simulation world for humanoid robot training."""

    def __init__(self):
        # Create world with physics settings
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / 500.0,      # 500 Hz physics
            rendering_dt=1.0 / 60.0,      # 60 Hz rendering
        )

        # Setup scene
        self._setup_ground()
        self._setup_lighting()
        self._setup_obstacles()
        self._setup_cameras()

    def _setup_ground(self):
        """Create ground plane with physics."""
        self.world.scene.add_default_ground_plane(
            z_position=0,
            name="ground_plane",
            prim_path="/World/Ground",
            static_friction=0.8,
            dynamic_friction=0.6,
            restitution=0.1
        )

        # Alternative: Custom ground with material
        # ground = FixedCuboid(
        #     prim_path="/World/CustomGround",
        #     name="custom_ground",
        #     position=np.array([0, 0, -0.05]),
        #     scale=np.array([100, 100, 0.1]),
        #     color=np.array([0.5, 0.5, 0.5])
        # )
        # self.world.scene.add(ground)

    def _setup_lighting(self):
        """Create realistic lighting setup."""
        from pxr import UsdLux, Gf
        stage = self.world.stage

        # Dome light for ambient (sky)
        dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
        dome.CreateIntensityAttr(1000)
        dome.CreateTextureFileAttr("omniverse://localhost/NVIDIA/Assets/Skies/Clear/noon_grass_2k.hdr")

        # Directional light (sun)
        sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
        sun.CreateIntensityAttr(3000)
        sun.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
        sun.CreateAngleAttr(0.53)

        from omni.isaac.core.utils.rotations import euler_angles_to_quat
        sun_xform = XFormPrim(prim_path="/World/Lights/Sun")
        sun_xform.set_world_pose(
            orientation=euler_angles_to_quat(np.array([-50, 30, 0]), degrees=True)
        )

    def _setup_obstacles(self):
        """Create obstacle course for navigation."""
        # Static boxes
        for i in range(5):
            box = FixedCuboid(
                prim_path=f"/World/Obstacles/Box_{i}",
                name=f"box_{i}",
                position=np.array([3 + i * 2, np.random.uniform(-2, 2), 0.5]),
                scale=np.array([0.5, 0.5, 1.0]),
                color=np.array([0.3, 0.3, 0.8])
            )
            self.world.scene.add(box)

        # Dynamic objects for manipulation
        sphere = DynamicSphere(
            prim_path="/World/Objects/Ball",
            name="ball",
            position=np.array([1, 0, 1]),
            radius=0.1,
            color=np.array([1.0, 0.2, 0.2]),
            mass=0.5
        )
        self.world.scene.add(sphere)

        # Cylinder
        cylinder = DynamicCylinder(
            prim_path="/World/Objects/Can",
            name="can",
            position=np.array([1.5, 0.5, 0.15]),
            radius=0.05,
            height=0.15,
            color=np.array([0.8, 0.8, 0.2]),
            mass=0.3
        )
        self.world.scene.add(cylinder)

    def _setup_cameras(self):
        """Create camera sensors."""
        from omni.isaac.sensor import Camera

        # Third-person camera
        self.third_person_cam = Camera(
            prim_path="/World/Cameras/ThirdPerson",
            position=np.array([-3, 0, 2]),
            orientation=euler_angles_to_quat(np.array([0, 30, 0]), degrees=True),
            resolution=(1280, 720),
            frequency=30
        )

        # Top-down camera
        self.top_cam = Camera(
            prim_path="/World/Cameras/TopDown",
            position=np.array([5, 0, 10]),
            orientation=euler_angles_to_quat(np.array([0, 90, 0]), degrees=True),
            resolution=(640, 480),
            frequency=10
        )

    def run(self, num_steps=1000):
        """Run simulation."""
        self.world.reset()

        for i in range(num_steps):
            self.world.step(render=True)

            if i % 100 == 0:
                print(f"Step {i}/{num_steps}")


# Main execution
if __name__ == "__main__":
    sim_world = HumanoidSimulationWorld()
    sim_world.run(num_steps=2000)
    simulation_app.close()
```

### 4.2 Physics Configuration

```python
#!/usr/bin/env python3
"""
physics_config.py - Configure physics parameters in Isaac Sim
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.physx import get_physx_scene_query_interface
import omni.physx.scripts.utils as physx_utils
from pxr import UsdPhysics, PhysxSchema


def configure_physics_scene(world: World):
    """Configure physics scene parameters."""
    stage = world.stage

    # Get or create physics scene
    physics_scene_path = "/World/PhysicsScene"
    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)

    # Set gravity
    physics_scene.CreateGravityDirectionAttr().Set((0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    # PhysX-specific settings
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))

    # Solver settings
    physx_scene.CreateEnableCCDAttr().Set(True)  # Continuous collision detection
    physx_scene.CreateEnableStabilizationAttr().Set(True)
    physx_scene.CreateBounceThresholdAttr().Set(0.2)
    physx_scene.CreateFrictionOffsetThresholdAttr().Set(0.01)
    physx_scene.CreateFrictionCorrelationDistanceAttr().Set(0.025)

    # GPU dynamics
    physx_scene.CreateEnableGPUDynamicsAttr().Set(True)
    physx_scene.CreateBroadphaseTypeAttr().Set("GPU")
    physx_scene.CreateGpuMaxRigidContactCountAttr().Set(524288)
    physx_scene.CreateGpuMaxRigidPatchCountAttr().Set(81920)

    # Solver iterations (higher = more accurate, slower)
    physx_scene.CreateSolverTypeAttr().Set("TGS")  # Temporal Gauss-Seidel
    physx_scene.CreateMinPositionIterationCountAttr().Set(4)
    physx_scene.CreateMaxPositionIterationCountAttr().Set(32)
    physx_scene.CreateMinVelocityIterationCountAttr().Set(1)
    physx_scene.CreateMaxVelocityIterationCountAttr().Set(16)


def configure_contact_properties(prim, friction=0.8, restitution=0.1):
    """Configure contact properties for a prim."""
    from pxr import UsdPhysics, PhysxSchema

    # Add collision API if not present
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)

    # Create physics material
    stage = prim.GetStage()
    material_path = f"{prim.GetPath()}/PhysicsMaterial"
    material = UsdPhysics.MaterialAPI.Apply(
        stage.DefinePrim(material_path, "Material")
    )

    material.CreateStaticFrictionAttr().Set(friction)
    material.CreateDynamicFrictionAttr().Set(friction * 0.8)
    material.CreateRestitutionAttr().Set(restitution)

    # PhysX-specific material properties
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(
        stage.GetPrimAtPath(material_path)
    )
    physx_material.CreateFrictionCombineModeAttr().Set("average")
    physx_material.CreateRestitutionCombineModeAttr().Set("average")

    # Bind material to collision
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(material_path)),
        UsdShade.Tokens.weakerThanDescendants,
        "physics"
    )


# Physics quality presets
PHYSICS_PRESETS = {
    "fast": {
        "physics_dt": 1.0 / 120.0,
        "solver_iterations": 4,
        "substeps": 1,
    },
    "balanced": {
        "physics_dt": 1.0 / 240.0,
        "solver_iterations": 8,
        "substeps": 2,
    },
    "accurate": {
        "physics_dt": 1.0 / 500.0,
        "solver_iterations": 16,
        "substeps": 4,
    },
}
```

---

## 5. Importing Robot Models

### 5.1 URDF Import

```python
#!/usr/bin/env python3
"""
import_robot.py - Import URDF robot into Isaac Sim
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.articulations import Articulation

# Enable URDF importer extension
enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf


class RobotImporter:
    """Import and configure robot models."""

    def __init__(self, world: World):
        self.world = world
        self.urdf_interface = _urdf.acquire_urdf_interface()

    def import_urdf(
        self,
        urdf_path: str,
        prim_path: str = "/World/Robot",
        position: np.ndarray = np.array([0, 0, 0]),
        fix_base: bool = False,
    ):
        """Import a URDF robot model."""

        # Import configuration
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = fix_base
        import_config.import_inertia_tensor = True
        import_config.distance_scale = 1.0
        import_config.density = 0.0  # Use URDF values
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.default_drive_strength = 1000.0
        import_config.default_position_drive_damping = 100.0
        import_config.self_collision = False
        import_config.create_physics_scene = True

        # Parse and import URDF
        result, robot_prim_path = self.urdf_interface.parse_urdf(
            urdf_path,
            import_config
        )

        if not result:
            raise RuntimeError(f"Failed to import URDF: {urdf_path}")

        # Move to target prim path
        from omni.kit.commands import execute
        execute(
            "MovePrim",
            path_from=robot_prim_path,
            path_to=prim_path
        )

        # Set initial position
        from omni.isaac.core.utils.prims import set_prim_attribute_value
        from pxr import Gf
        set_prim_attribute_value(
            prim_path,
            "xformOp:translate",
            Gf.Vec3d(*position)
        )

        print(f"Robot imported at {prim_path}")
        return prim_path

    def configure_articulation(self, prim_path: str):
        """Configure articulation drives and limits."""
        stage = self.world.stage
        prim = stage.GetPrimAtPath(prim_path)

        # Find all joints
        from pxr import UsdPhysics, PhysxSchema

        for descendant in prim.GetDescendants():
            if descendant.HasAPI(UsdPhysics.DriveAPI):
                # Get joint type
                joint = UsdPhysics.Joint(descendant)
                joint_type = descendant.GetTypeName()

                if "Revolute" in joint_type:
                    # Configure revolute joint drive
                    drive = UsdPhysics.DriveAPI.Get(descendant, "angular")
                    drive.CreateTypeAttr().Set("force")
                    drive.CreateMaxForceAttr().Set(1000.0)
                    drive.CreateStiffnessAttr().Set(10000.0)
                    drive.CreateDampingAttr().Set(1000.0)

                    # PhysX drive settings
                    physx_joint = PhysxSchema.PhysxJointAPI.Apply(descendant)
                    physx_joint.CreateJointFrictionAttr().Set(0.1)

                elif "Prismatic" in joint_type:
                    # Configure prismatic joint drive
                    drive = UsdPhysics.DriveAPI.Get(descendant, "linear")
                    drive.CreateTypeAttr().Set("force")
                    drive.CreateMaxForceAttr().Set(500.0)
                    drive.CreateStiffnessAttr().Set(5000.0)
                    drive.CreateDampingAttr().Set(500.0)

        print(f"Articulation configured at {prim_path}")


def import_humanoid_robot():
    """Example: Import a humanoid robot."""
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    importer = RobotImporter(world)

    # Import URDF (example path - replace with actual path)
    robot_prim = importer.import_urdf(
        urdf_path="/path/to/humanoid.urdf",
        prim_path="/World/Humanoid",
        position=np.array([0, 0, 1.0]),
        fix_base=False
    )

    importer.configure_articulation(robot_prim)

    # Create Articulation wrapper for control
    robot = Articulation(prim_path=robot_prim, name="humanoid")
    world.scene.add(robot)

    # Reset and get joint info
    world.reset()
    robot.initialize()

    print(f"DOF names: {robot.dof_names}")
    print(f"DOF positions: {robot.get_joint_positions()}")

    # Run simulation
    for i in range(1000):
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    import_humanoid_robot()
```

### 5.2 Articulation Control

```python
#!/usr/bin/env python3
"""
articulation_control.py - Control robot articulations
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction


class ArticulationController:
    """Control robot articulations with position, velocity, or effort commands."""

    def __init__(self, articulation: Articulation):
        self.articulation = articulation
        self.num_dof = articulation.num_dof
        self.dof_names = articulation.dof_names

        # Control gains
        self.kp = np.ones(self.num_dof) * 1000.0  # Position gain
        self.kd = np.ones(self.num_dof) * 100.0   # Velocity gain

    def set_position_targets(self, positions: np.ndarray):
        """Set joint position targets."""
        assert len(positions) == self.num_dof

        action = ArticulationAction(
            joint_positions=positions,
            joint_velocities=None,
            joint_efforts=None
        )
        self.articulation.apply_action(action)

    def set_velocity_targets(self, velocities: np.ndarray):
        """Set joint velocity targets."""
        assert len(velocities) == self.num_dof

        action = ArticulationAction(
            joint_positions=None,
            joint_velocities=velocities,
            joint_efforts=None
        )
        self.articulation.apply_action(action)

    def set_effort_targets(self, efforts: np.ndarray):
        """Set joint effort (torque/force) targets."""
        assert len(efforts) == self.num_dof

        action = ArticulationAction(
            joint_positions=None,
            joint_velocities=None,
            joint_efforts=efforts
        )
        self.articulation.apply_action(action)

    def pd_control(self, target_pos: np.ndarray, target_vel: np.ndarray = None):
        """Apply PD control to reach target position."""
        if target_vel is None:
            target_vel = np.zeros(self.num_dof)

        current_pos = self.articulation.get_joint_positions()
        current_vel = self.articulation.get_joint_velocities()

        # PD control law
        pos_error = target_pos - current_pos
        vel_error = target_vel - current_vel
        efforts = self.kp * pos_error + self.kd * vel_error

        self.set_effort_targets(efforts)

    def get_state(self):
        """Get current articulation state."""
        return {
            "positions": self.articulation.get_joint_positions(),
            "velocities": self.articulation.get_joint_velocities(),
            "efforts": self.articulation.get_applied_joint_efforts(),
        }


# Example: Sinusoidal joint motion
def sinusoidal_motion_example():
    """Move joints in sinusoidal pattern."""
    world = World()
    world.scene.add_default_ground_plane()

    # Assuming robot already loaded (see import_robot.py)
    robot = Articulation(prim_path="/World/Humanoid", name="humanoid")
    world.scene.add(robot)
    world.reset()
    robot.initialize()

    controller = ArticulationController(robot)
    default_pos = robot.get_joint_positions()

    for step in range(2000):
        t = step * 0.01  # Time in seconds

        # Generate sinusoidal targets
        amplitude = 0.3
        frequency = 0.5
        targets = default_pos + amplitude * np.sin(2 * np.pi * frequency * t)

        # Apply control
        controller.set_position_targets(targets)

        world.step(render=True)

    simulation_app.close()
```

---

## 6. Synthetic Data Generation

### 6.1 Replicator Basics

```python
#!/usr/bin/env python3
"""
replicator_basics.py - Generate synthetic training data with Replicator
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.replicator.core as rep
from omni.isaac.core import World
import numpy as np


def setup_data_generation():
    """Configure synthetic data generation pipeline."""

    # Create scene
    world = World()
    world.scene.add_default_ground_plane()

    # Create objects for data generation
    with rep.new_layer():
        # Create camera
        camera = rep.create.camera(
            position=(3, 3, 2),
            look_at=(0, 0, 0.5),
            focal_length=24
        )

        # Create render product
        render_product = rep.create.render_product(camera, (1280, 720))

        # Create objects to randomize
        cubes = rep.create.cube(
            count=5,
            scale=rep.distribution.uniform((0.1, 0.1, 0.1), (0.3, 0.3, 0.3)),
            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0.5)),
            semantics=[("class", "cube")]
        )

        spheres = rep.create.sphere(
            count=3,
            scale=rep.distribution.uniform(0.05, 0.15),
            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0.3)),
            semantics=[("class", "sphere")]
        )

    return render_product


def domain_randomization():
    """Apply domain randomization."""

    # Randomize on each frame
    with rep.trigger.on_frame(num_frames=100):

        # Randomize lighting
        with rep.create.light(light_type="distant"):
            rep.modify.pose(
                rotation=rep.distribution.uniform((-180, -90, 0), (180, 90, 0))
            )
            rep.modify.attribute(
                "intensity",
                rep.distribution.uniform(500, 3000)
            )
            rep.modify.attribute(
                "color",
                rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))
            )

        # Randomize object positions
        with rep.get.prims(semantics=[("class", "cube")]):
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0.1), (1, 1, 0.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

        # Randomize textures
        with rep.get.prims(semantics=[("class", "cube")]):
            rep.randomizer.texture(
                textures=[
                    "omniverse://localhost/NVIDIA/Materials/Base/Wood/Oak.mdl",
                    "omniverse://localhost/NVIDIA/Materials/Base/Metal/Steel.mdl",
                    "omniverse://localhost/NVIDIA/Materials/Base/Stone/Concrete.mdl",
                ]
            )


def configure_annotators(render_product):
    """Configure data annotation outputs."""

    # RGB image
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([render_product])

    # Depth
    depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth.attach([render_product])

    # Semantic segmentation
    semantic = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
    semantic.attach([render_product])

    # Instance segmentation
    instance = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
    instance.attach([render_product])

    # 2D bounding boxes
    bbox_2d = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
    bbox_2d.attach([render_product])

    # 3D bounding boxes
    bbox_3d = rep.AnnotatorRegistry.get_annotator("bounding_box_3d")
    bbox_3d.attach([render_product])

    # Normals
    normals = rep.AnnotatorRegistry.get_annotator("normals")
    normals.attach([render_product])

    return {
        "rgb": rgb,
        "depth": depth,
        "semantic": semantic,
        "instance": instance,
        "bbox_2d": bbox_2d,
        "bbox_3d": bbox_3d,
        "normals": normals,
    }


def write_dataset(annotators, output_dir="/tmp/synthetic_data"):
    """Write dataset using BasicWriter."""

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=output_dir,
        rgb=True,
        distance_to_camera=True,
        semantic_segmentation=True,
        instance_segmentation=True,
        bounding_box_2d_tight=True,
        bounding_box_3d=True,
        normals=True,
    )
    writer.attach([annotators["rgb"]._render_product_path])

    # Generate data
    rep.orchestrator.run()

    print(f"Dataset written to {output_dir}")


# Main execution
if __name__ == "__main__":
    render_product = setup_data_generation()
    domain_randomization()
    annotators = configure_annotators(render_product)
    write_dataset(annotators)
    simulation_app.close()
```

### 6.2 Custom Randomizers

```python
#!/usr/bin/env python3
"""
custom_randomizers.py - Create custom domain randomization
"""

import omni.replicator.core as rep
import numpy as np


@rep.randomizer.register("robot_pose_randomizer")
def robot_pose_randomizer(prim_path: str):
    """Custom randomizer for robot initial poses."""

    # Get the robot prim
    with rep.get.prims(path_match=prim_path):
        # Randomize base position
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0.1), (2, 2, 0.1)),
            rotation=rep.distribution.uniform((0, 0, -180), (0, 0, 180))
        )

    return None


@rep.randomizer.register("joint_pose_randomizer")
def joint_pose_randomizer(
    joint_paths: list,
    joint_ranges: list,  # List of (min, max) tuples
):
    """Randomize joint positions within limits."""

    for joint_path, (min_val, max_val) in zip(joint_paths, joint_ranges):
        with rep.get.prims(path_match=joint_path):
            rep.modify.attribute(
                "drive:angular:physics:targetPosition",
                rep.distribution.uniform(min_val, max_val)
            )

    return None


@rep.randomizer.register("sensor_noise_randomizer")
def sensor_noise_randomizer():
    """Add realistic sensor noise to cameras."""

    # This would be applied post-capture
    # Example: Add Gaussian noise, blur, exposure variation

    return None


# Example usage
def apply_custom_randomization():
    """Apply custom randomizers to scene."""

    with rep.trigger.on_frame(num_frames=50):
        # Apply robot pose randomization
        rep.randomizer.robot_pose_randomizer("/World/Robot")

        # Apply joint randomization
        rep.randomizer.joint_pose_randomizer(
            joint_paths=[
                "/World/Robot/hip_joint",
                "/World/Robot/knee_joint",
                "/World/Robot/ankle_joint",
            ],
            joint_ranges=[
                (-0.5, 0.5),   # hip: ±0.5 rad
                (0, 1.5),      # knee: 0 to 1.5 rad
                (-0.3, 0.3),   # ankle: ±0.3 rad
            ]
        )
```

---

## 7. Summary

### Isaac Sim Component Overview

| Component | Purpose | Key APIs |
|-----------|---------|----------|
| **World** | Simulation management | `World`, `Scene` |
| **USD** | Scene description | `Usd`, `UsdGeom`, `UsdPhysics` |
| **PhysX** | Physics simulation | `PhysxSchema`, `UsdPhysics` |
| **Articulation** | Robot control | `Articulation`, `ArticulationAction` |
| **Replicator** | Synthetic data | `rep.create`, `rep.modify`, `rep.randomizer` |
| **Sensors** | Camera, LiDAR | `Camera`, `Lidar`, `ContactSensor` |

### File Types

| Extension | Purpose |
|-----------|---------|
| `.usd` | Binary USD file |
| `.usda` | ASCII USD file (human-readable) |
| `.usdc` | Crate (compressed) USD |
| `.usdz` | Zipped USD package |

---

## Exercises

### Exercise 8.1: Installation and Verification (⭐)

1. Install Isaac Sim via Omniverse Launcher
2. Run the verification script
3. Confirm GPU acceleration is working
4. Take a screenshot of the Isaac Sim UI

### Exercise 8.2: Custom World Creation (⭐⭐)

Create an Isaac Sim world with:
- A ground plane with custom friction
- At least 5 static obstacles
- 3 dynamic objects (cubes, spheres)
- Proper lighting (sun + ambient)
- Export scene as `.usda` file

### Exercise 8.3: Robot Import and Control (⭐⭐⭐)

1. Import a URDF robot (use provided or create your own)
2. Configure joint drives (stiffness, damping)
3. Implement sinusoidal joint motion
4. Record joint positions over time

### Exercise 8.4: Synthetic Data Pipeline (⭐⭐⭐)

Create a complete data generation pipeline:
- Scene with randomized objects
- Domain randomization (lighting, textures, positions)
- Multiple annotators (RGB, depth, segmentation, bboxes)
- Generate 100 images with annotations
- Validate output format

---

## Quiz

<details>
<summary>Q1: What are the key differences between Gazebo and Isaac Sim?</summary>

Isaac Sim offers:
- RTX ray tracing vs OGRE rasterization
- GPU-accelerated PhysX vs CPU-based physics
- 4096+ parallel environments for RL
- Physically-based sensor noise models
- Built-in synthetic data generation (Replicator)
- Native ROS 2 integration via Isaac ROS

</details>

<details>
<summary>Q2: What is USD and why is it important for Isaac Sim?</summary>

USD (Universal Scene Description) is:
- Developed by Pixar for film production
- A hierarchical scene format with prims and attributes
- Supports composition, variants, and layers
- Foundation for Omniverse collaboration
- Enables precise physics and rendering configurations

</details>

<details>
<summary>Q3: How does domain randomization help sim-to-real transfer?</summary>

Domain randomization:
- Exposes the model to wide variation during training
- Makes the policy treat reality as "just another variant"
- Reduces overfitting to simulation-specific features
- Key randomization targets: lighting, textures, physics, sensor noise
- Enables zero-shot transfer without real-world fine-tuning

</details>

<details>
<summary>Q4: What is the recommended physics timestep for humanoid robots?</summary>

For humanoid robots:
- 500 Hz (0.002s) is typical for accurate contact simulation
- Higher frequencies (1000 Hz) for fine manipulation
- GPU dynamics enables fast simulation at high frequencies
- Use TGS solver with 4-16 position iterations
- Enable CCD for fast-moving limbs

</details>

---

## Next Steps

In [Week 9: Perception and VSLAM](/module-3-isaac/week-9-perception-vslam), we explore:
- Isaac ROS perception stack
- Hardware-accelerated VSLAM (cuVSLAM)
- Object detection with DOPE
- Integration with Nav2
