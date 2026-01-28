---
sidebar_position: 1
---

# Week 6: Gazebo Simulation Setup

## Learning Objectives

By the end of this week, you will be able to:

- **Install** and configure Gazebo simulation environment
- **Create** URDF robot models from scratch
- **Build** simulation worlds with terrain and objects
- **Integrate** Gazebo with ROS 2 for robot control

---

## 1. Introduction to Gazebo

### 1.1 What is Gazebo?

**Gazebo** is an open-source 3D robotics simulator providing:

- Physics simulation (ODE, Bullet, DART, Simbody)
- Sensor simulation (cameras, LiDAR, IMU)
- Robot model support (URDF, SDF)
- ROS 2 integration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Gazebo Architecture                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌─────────────────────────────────────────────────────────────────┐     │
│    │                        Gazebo Server                             │     │
│    │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐       │     │
│    │  │    Physics    │  │   Rendering   │  │    Sensors    │       │     │
│    │  │    Engine     │  │    Engine     │  │   Simulation  │       │     │
│    │  │  (ODE/Bullet) │  │    (OGRE)     │  │               │       │     │
│    │  └───────────────┘  └───────────────┘  └───────────────┘       │     │
│    └─────────────────────────────────────────────────────────────────┘     │
│                                    │                                        │
│                              Transport                                      │
│                                    │                                        │
│    ┌─────────────────────────────────────────────────────────────────┐     │
│    │                        Gazebo Client                             │     │
│    │                    (GUI Visualization)                           │     │
│    └─────────────────────────────────────────────────────────────────┘     │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Installation

```bash
# Install Gazebo Harmonic (latest)
sudo apt update
sudo apt install ros-humble-ros-gz

# Or install specific packages
sudo apt install ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-ros-gz-image

# Verify installation
gz sim --version
```

---

## 2. URDF: Robot Description

### 2.1 URDF Structure

**URDF (Unified Robot Description Format)** defines robot structure:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          URDF Structure                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    <robot>                                                                  │
│        │                                                                    │
│        ├── <link name="base_link">                                         │
│        │       ├── <visual>        (how it looks)                          │
│        │       ├── <collision>     (for physics)                           │
│        │       └── <inertial>      (mass, inertia)                         │
│        │                                                                    │
│        ├── <link name="leg_link">                                          │
│        │       └── ...                                                      │
│        │                                                                    │
│        ├── <joint name="hip_joint">                                        │
│        │       ├── <parent link="base_link"/>                              │
│        │       ├── <child link="leg_link"/>                                │
│        │       ├── <origin xyz="0 0 0" rpy="0 0 0"/>                       │
│        │       ├── <axis xyz="0 1 0"/>                                     │
│        │       └── <limit lower="-1.57" upper="1.57"/>                     │
│        │                                                                    │
│        └── <transmission>  (actuator connection)                           │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Complete URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link (Torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0"
               izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Right Upper Leg -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.03" ixy="0" ixz="0"
               iyy="0.03" iyz="0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Hip Joint -->
  <joint name="right_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="0 -0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Right Lower Leg -->
  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.02" ixy="0" ixz="0"
               iyy="0.02" iyz="0"
               izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Knee Joint -->
  <joint name="right_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="80" velocity="2.0"/>
  </joint>

</robot>
```

### 2.3 Joint Types

| Type | Description | Example |
|------|-------------|---------|
| `revolute` | Rotation with limits | Hip, knee, elbow |
| `continuous` | Unlimited rotation | Wheel |
| `prismatic` | Linear sliding | Telescope arm |
| `fixed` | No movement | Sensor mount |
| `floating` | 6-DoF (base) | Free-floating base |

---

## 3. Creating Simulation Worlds

### 3.1 SDF World File

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_world">

    <!-- Physics Configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Box Obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## 4. ROS 2 - Gazebo Integration

### 4.1 Launch File

```python
# launch/gazebo_robot_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')

    # Robot description
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
```

### 4.2 Joint Control

```python
#!/usr/bin/env python3
"""
joint_controller.py - Control robot joints in Gazebo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        # Joint targets
        self.joint_targets = [0.0, 0.0, 0.0, 0.0]

    def control_loop(self):
        msg = Float64MultiArray()
        msg.data = self.joint_targets
        self.joint_pub.publish(msg)

    def set_joint_positions(self, positions: list):
        """Set target joint positions."""
        self.joint_targets = positions
```

---

## 5. Summary

| Component | File Type | Purpose |
|-----------|-----------|---------|
| URDF | `.urdf` | Robot model description |
| SDF | `.sdf` | World and model description |
| Launch | `.py` | Start simulation + nodes |
| Bridge | Node | Connect Gazebo ↔ ROS 2 |

---

## Exercises

### Exercise 6.1: URDF Modeling (⭐⭐)

Create a URDF for a robot arm with:
- 3 revolute joints
- Proper inertia values
- Visual and collision geometries

### Exercise 6.2: World Building (⭐⭐)

Create a Gazebo world with:
- Ground plane with friction
- 3 obstacles at different positions
- Proper lighting

### Exercise 6.3: ROS 2 Integration (⭐⭐⭐)

Create a launch file that:
- Starts Gazebo with your world
- Spawns your robot
- Bridges joint states to ROS 2

---

## Next Steps

In [Week 7: Physics and Sensor Simulation](/module-2-digital-twin/week-7-physics-sensors), we add:
- Detailed physics configuration
- Camera, LiDAR, and IMU simulation
- Sensor noise models
