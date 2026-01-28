---
sidebar_position: 2
---

# Week 7: Physics and Sensor Simulation

## Learning Objectives

By the end of this week, you will be able to:

- Configure physics properties for realistic simulation
- Simulate common robot sensors (camera, LiDAR, IMU)
- Understand sensor noise models
- Bridge sensor data to ROS 2 topics

## Physics Simulation

### Key Physics Properties

| Property | Description | Typical Values |
|----------|-------------|----------------|
| **Mass** | Weight of link | 0.1 - 50 kg |
| **Inertia** | Resistance to rotation | Calculated from geometry |
| **Friction** | Surface interaction | 0.5 - 1.0 (mu) |
| **Damping** | Energy dissipation | 0.01 - 0.1 |

### Physics Configuration

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

## Sensor Simulation

### Camera Sensor

```xml
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
</sensor>
```

### Depth Camera (RealSense Style)

```xml
<sensor name="depth_camera" type="depth_camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.21</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
  </camera>
</sensor>
```

### LiDAR Sensor

```xml
<sensor name="lidar" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
    </range>
  </lidar>
</sensor>
```

### IMU Sensor

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
      </x>
    </linear_acceleration>
  </imu>
</sensor>
```

## Sensor Noise Models

Real sensors are noisy. Simulation should reflect this:

```
Actual Value ───► Noise Model ───► Simulated Reading
                      │
              ┌───────┴───────┐
              │               │
          Gaussian       Bias Drift
          σ = 0.01       β = 0.001/s
```

## ROS 2 Sensor Bridge

```python
# Bridge sensor topics to ROS 2
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
        '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    ],
)
```

## Unity Integration (Overview)

For high-fidelity visualization and human-robot interaction:

- **Unity Robotics Hub**: ROS-TCP-Connector
- **URDF Importer**: Import robot models
- **Use Case**: HRI visualization, VR/AR interfaces

## Exercises

1. Add physics properties to your URDF robot
2. Simulate a camera and visualize in RViz2
3. Add noise to IMU sensor readings
4. Bridge all sensors to ROS 2 and record a bag file

## Assessment: Gazebo Simulation

Create a complete simulation environment. See [Week 14: Capstone](/assessments/week-14-capstone) for requirements.

## Next Steps

In [Week 8](/module-3-isaac/week-8-isaac-sim), we enter Module 3 and explore NVIDIA Isaac Sim.
