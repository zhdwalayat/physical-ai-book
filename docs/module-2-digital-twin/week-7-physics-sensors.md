---
sidebar_position: 2
---

# Week 7: Physics and Sensor Simulation

## Learning Objectives

By the end of this week, you will be able to:

- **Configure** physics properties for realistic simulation
- **Simulate** cameras, LiDAR, and IMU sensors
- **Model** sensor noise for realistic behavior
- **Bridge** sensor data to ROS 2 topics

---

## 1. Physics Configuration

### 1.1 Physics Engines

| Engine | Pros | Cons | Best For |
|--------|------|------|----------|
| **ODE** | Fast, stable | Less accurate contacts | General robotics |
| **Bullet** | Good for collisions | Memory intensive | Manipulation |
| **DART** | Accurate dynamics | Slower | Research |

### 1.2 Physics Parameters

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms steps -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>

  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>  <!-- Successive over-relaxation -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>  <!-- Constraint force mixing -->
      <erp>0.2</erp>  <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### 1.3 Surface Properties

```xml
<collision name="foot_collision">
  <geometry>
    <box><size>0.15 0.08 0.02</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- Friction coefficient -->
        <mu2>1.0</mu2>    <!-- Secondary friction -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient>
      <threshold>1.0</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0.0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1e8</kp>   <!-- Contact stiffness -->
        <kd>1.0</kd>   <!-- Contact damping -->
      </ode>
    </contact>
  </surface>
</collision>
```

---

## 2. Camera Simulation

### 2.1 RGB Camera

```xml
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>

  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
</sensor>
```

### 2.2 Depth Camera

```xml
<sensor name="depth_camera" type="depth_camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>

  <camera>
    <horizontal_fov>1.21</horizontal_fov>  <!-- 69 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm noise -->
    </noise>
  </camera>
</sensor>
```

---

## 3. LiDAR Simulation

### 3.1 2D LiDAR

```xml
<sensor name="lidar_2d" type="ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>

  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
</sensor>
```

### 3.2 3D LiDAR

```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>

  <lidar>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
        <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.3</min>
      <max>100.0</max>
    </range>
  </lidar>
</sensor>
```

---

## 4. IMU Simulation

### 4.1 IMU Configuration

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate>

  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></z>
    </angular_velocity>

    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

---

## 5. ROS 2 Bridge Configuration

### 5.1 Bridge Topics

```python
# launch/sensors_bridge.py

from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Depth
            '/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # LiDAR
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    return LaunchDescription([bridge])
```

### 5.2 Image Processing Node

```python
#!/usr/bin/env python3
"""
image_processor.py - Process camera images from Gazebo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Publish processed image
        self.processed_pub = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

    def image_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Process (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS message
        processed_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
        processed_msg.header = msg.header

        self.processed_pub.publish(processed_msg)
```

---

## 6. Summary

### Sensor Comparison

| Sensor | Update Rate | Data Type | Use Case |
|--------|-------------|-----------|----------|
| RGB Camera | 30 Hz | Image | Object detection |
| Depth Camera | 30 Hz | Image + PointCloud | 3D perception |
| 2D LiDAR | 10-40 Hz | LaserScan | Navigation |
| 3D LiDAR | 10-20 Hz | PointCloud2 | SLAM |
| IMU | 100-400 Hz | Imu | State estimation |

---

## Exercises

### Exercise 7.1: Camera Configuration (⭐⭐)

Configure a stereo camera pair with:
- 30 Hz update rate
- 1280x720 resolution
- Realistic noise model

### Exercise 7.2: Sensor Fusion (⭐⭐⭐)

Create a node that:
- Subscribes to IMU and camera
- Timestamps data for synchronization
- Publishes fused pose estimate

---

## Module 2 Assessment: Gazebo Simulation

**Deliverable**: Complete simulation with:

1. Robot URDF with proper physics
2. World with obstacles
3. Working sensors (camera + LiDAR)
4. ROS 2 bridge configuration

See [Week 14: Capstone](/assessments/week-14-capstone) for full requirements.

---

## Next Steps

In [Week 8: NVIDIA Isaac Sim](/module-3-isaac/week-8-isaac-sim), we begin Module 3:
- Photorealistic simulation
- Synthetic data generation
- Isaac Sim Python API
