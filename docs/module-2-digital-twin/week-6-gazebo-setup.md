---
sidebar_position: 1
---

# Week 6: Gazebo Simulation Setup

## Learning Objectives

By the end of this week, you will be able to:

- Install and configure Gazebo simulation environment
- Understand URDF and SDF robot description formats
- Create a basic simulation world
- Import and visualize a humanoid robot model

## What is Gazebo?

**Gazebo** is an open-source 3D robotics simulator that provides:

- Realistic physics simulation (ODE, Bullet, DART)
- Sensor simulation (cameras, LiDAR, IMU)
- ROS 2 integration
- Plugin architecture for extensibility

## Gazebo vs Isaac Sim

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| Cost | Free/Open Source | Free (NVIDIA GPU required) |
| Physics | ODE, Bullet, DART | PhysX |
| Graphics | Basic | Photorealistic (RTX) |
| ROS 2 | Native | Via Isaac ROS |
| Learning Curve | Moderate | Steep |
| Use Case | Prototyping, Education | Production, AI Training |

## Installation

### Gazebo Fortress on Ubuntu 22.04

```bash
# Install Gazebo
sudo apt update
sudo apt install ros-humble-ros-gz

# Install additional packages
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# Verify installation
gz sim --version
```

## URDF: Robot Description

**Unified Robot Description Format (URDF)** defines robot structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

## SDF: Simulation Description Format

**SDF** is more powerful for Gazebo simulation:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_world">

    <!-- Physics -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Creating Your First World

```bash
# Create a workspace
mkdir -p ~/humanoid_ws/src/humanoid_sim/worlds
mkdir -p ~/humanoid_ws/src/humanoid_sim/models
mkdir -p ~/humanoid_ws/src/humanoid_sim/launch

# Launch Gazebo with empty world
gz sim empty.sdf
```

## ROS 2 - Gazebo Bridge

```python
# launch/gazebo_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([gz_sim, bridge])
```

## Exercises

1. Install Gazebo Fortress and verify the installation
2. Create a simple URDF for a 2-link arm
3. Build a basic world with ground plane and lighting
4. Launch Gazebo with ROS 2 integration

## Next Steps

In [Week 7](/module-2-digital-twin/week-7-physics-sensors), we add physics properties and simulate sensors.
