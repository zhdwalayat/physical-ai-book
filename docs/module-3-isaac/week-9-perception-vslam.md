---
sidebar_position: 2
---

# Week 9: Perception and VSLAM

## Learning Objectives

By the end of this week, you will be able to:

- **Understand** Visual SLAM concepts and feature-based algorithms
- **Deploy** Isaac ROS for hardware-accelerated perception
- **Implement** cuVSLAM for real-time visual odometry
- **Integrate** perception with ROS 2 navigation stack
- **Evaluate** VSLAM performance with standard metrics

---

## 1. Introduction to Visual SLAM

### 1.1 What is VSLAM?

**Visual Simultaneous Localization and Mapping (VSLAM)** enables robots to:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          VSLAM: The Core Problem                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Given: Sequence of camera images                                         │
│    Find:  Camera trajectory + 3D map of environment                         │
│                                                                              │
│    ┌─────────────────────────────────────────────────────────────────────┐  │
│    │                        Visual SLAM Pipeline                          │  │
│    │                                                                      │  │
│    │   Camera    Feature      Feature      Motion        Map             │  │
│    │   Frames ──►Detection ──►Matching ──►Estimation ──►Building         │  │
│    │     │                                    │             │             │  │
│    │     │                                    ▼             ▼             │  │
│    │     │                              ┌──────────┐ ┌──────────────┐    │  │
│    │     │                              │ Camera   │ │ 3D Point     │    │  │
│    │     │                              │ Pose     │ │ Cloud Map    │    │  │
│    │     │                              └──────────┘ └──────────────┘    │  │
│    │     │                                    │             │             │  │
│    │     │                                    └──────┬──────┘             │  │
│    │     │                                           │                    │  │
│    │     │                                           ▼                    │  │
│    │     │                                    ┌──────────────┐           │  │
│    │     └───────────────────────────────────►│ Loop Closure │           │  │
│    │                                          │ Detection    │           │  │
│    │                                          └──────────────┘           │  │
│    └─────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 VSLAM Approaches

| Approach | Method | Pros | Cons |
|----------|--------|------|------|
| **Feature-based** | ORB, SIFT, SURF | Robust, well-understood | Misses texture-less regions |
| **Direct** | LSD-SLAM, DSO | Uses all pixels | Sensitive to lighting |
| **Hybrid** | ORB-SLAM3 | Best of both | Complex implementation |
| **Learning-based** | DROID-SLAM | End-to-end | Requires training data |

### 1.3 Key Components

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        VSLAM System Components                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────────┐                                                       │
│   │  Front-end      │  Real-time processing                                 │
│   │  ─────────────  │                                                       │
│   │  • Feature detection (ORB, FAST, Harris)                               │
│   │  • Feature tracking (optical flow, descriptor matching)                 │
│   │  • Visual odometry (frame-to-frame motion)                             │
│   │  • Local mapping (triangulation)                                        │
│   └────────┬────────┘                                                       │
│            │                                                                 │
│            ▼                                                                 │
│   ┌─────────────────┐                                                       │
│   │  Back-end       │  Optimization (runs in background)                    │
│   │  ─────────────  │                                                       │
│   │  • Bundle adjustment (minimize reprojection error)                      │
│   │  • Loop closure (detect revisited places)                               │
│   │  • Pose graph optimization (global consistency)                         │
│   │  • Map merging (multi-session SLAM)                                     │
│   └────────┬────────┘                                                       │
│            │                                                                 │
│            ▼                                                                 │
│   ┌─────────────────┐                                                       │
│   │  Output         │                                                       │
│   │  ─────────────  │                                                       │
│   │  • Camera pose (6-DoF: x, y, z, roll, pitch, yaw)                      │
│   │  • Sparse/Dense point cloud                                             │
│   │  • Keyframe database                                                    │
│   └─────────────────┘                                                       │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Isaac ROS Visual SLAM

### 2.1 cuVSLAM Overview

**NVIDIA cuVSLAM** is a hardware-accelerated VSLAM implementation:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          cuVSLAM Architecture                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Stereo Camera / Depth Camera                                              │
│           │                                                                  │
│           ▼                                                                  │
│   ┌───────────────────────────────────────────────────────────────────────┐ │
│   │                      GPU-Accelerated Pipeline                          │ │
│   │                                                                        │ │
│   │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐            │ │
│   │  │ Image Rectif.│───►│ Feature Ext. │───►│ Stereo Match │            │ │
│   │  │ (CUDA)       │    │ (CUDA/Tensor)│    │ (CUDA)       │            │ │
│   │  └──────────────┘    └──────────────┘    └──────────────┘            │ │
│   │                                                 │                      │ │
│   │                                                 ▼                      │ │
│   │  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐            │ │
│   │  │ Loop Closure │◄───│ Pose Optim.  │◄───│ Tracking     │            │ │
│   │  │ (GPU)        │    │ (GPU)        │    │ (GPU)        │            │ │
│   │  └──────────────┘    └──────────────┘    └──────────────┘            │ │
│   │                                                                        │ │
│   └───────────────────────────────────────────────────────────────────────┘ │
│           │                                                                  │
│           ▼                                                                  │
│   ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │
│   │ Visual Odometry │  │ Point Cloud     │  │ Pose Graph      │            │
│   │ /visual_slam/   │  │ /visual_slam/   │  │ /visual_slam/   │            │
│   │ tracking/odom   │  │ vis/landmarks   │  │ tracking/vo_pose│            │
│   └─────────────────┘  └─────────────────┘  └─────────────────┘            │
│                                                                              │
│   Performance: 60+ FPS on Jetson AGX Orin, 30 FPS on Jetson Orin Nano      │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Installation

```bash
# Create Isaac ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone required repositories
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git

# Setup Docker environment
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh

# Inside Docker container - Build packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-up-to isaac_ros_visual_slam
source install/setup.bash

# Verify installation
ros2 pkg list | grep isaac_ros_visual_slam
```

### 2.3 Launch Configuration

```python
#!/usr/bin/env python3
"""
launch/vslam_realsense_launch.py - Launch cuVSLAM with RealSense D435i
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    enable_imu = LaunchConfiguration('enable_imu', default='true')
    enable_debug = LaunchConfiguration('enable_debug', default='false')

    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': True,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,  # Linear interpolation
            'infra_width': 640,
            'infra_height': 480,
            'infra_fps': 60.0,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30.0,
            'gyro_fps': 200.0,
            'accel_fps': 200.0,
        }],
        output='screen'
    )

    # Image rectification nodes
    rectify_left = ComposableNode(
        name='rectify_left',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 640,
            'output_height': 480,
        }],
        remappings=[
            ('image_raw', '/camera/infra1/image_rect_raw'),
            ('camera_info', '/camera/infra1/camera_info'),
            ('image_rect', '/camera/left/image_rect'),
            ('camera_info_rect', '/camera/left/camera_info_rect'),
        ]
    )

    rectify_right = ComposableNode(
        name='rectify_right',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 640,
            'output_height': 480,
        }],
        remappings=[
            ('image_raw', '/camera/infra2/image_rect_raw'),
            ('camera_info', '/camera/infra2/camera_info'),
            ('image_rect', '/camera/right/image_rect'),
            ('camera_info_rect', '/camera/right/camera_info_rect'),
        ]
    )

    # cuVSLAM node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # Visual odometry parameters
            'enable_localization_n_mapping': True,
            'enable_imu_fusion': True,

            # Camera parameters
            'image_height': 480,
            'image_width': 640,
            'denoise_input_images': False,
            'rectified_images': True,

            # IMU noise parameters (calibrated for RealSense D435i)
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'imu_frame_id': 'camera_imu_optical_frame',

            # Tracking parameters
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'enable_slam_visualization': True,
            'enable_debug_mode': False,

            # Output frames
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',

            # Performance tuning
            'num_cameras': 2,
            'min_num_images': 150,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/left/image_rect'),
            ('visual_slam/camera_info_0', '/camera/left/camera_info_rect'),
            ('visual_slam/image_1', '/camera/right/image_rect'),
            ('visual_slam/camera_info_1', '/camera/right/camera_info_rect'),
            ('visual_slam/imu', '/camera/imu'),
        ]
    )

    # Container for composable nodes
    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_left,
            rectify_right,
            visual_slam_node,
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_debug', default_value='false'),
        realsense_node,
        container,
    ])
```

### 2.4 VSLAM Topics and Messages

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        cuVSLAM ROS 2 Topics                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Input Topics:                                                               │
│  ─────────────                                                              │
│  /visual_slam/image_0          sensor_msgs/Image     Left camera image      │
│  /visual_slam/image_1          sensor_msgs/Image     Right camera image     │
│  /visual_slam/camera_info_0    sensor_msgs/CameraInfo Camera calibration    │
│  /visual_slam/camera_info_1    sensor_msgs/CameraInfo Camera calibration    │
│  /visual_slam/imu              sensor_msgs/Imu       IMU measurements       │
│                                                                              │
│  Output Topics:                                                              │
│  ──────────────                                                             │
│  /visual_slam/tracking/odometry         nav_msgs/Odometry      6-DoF pose   │
│  /visual_slam/tracking/vo_pose          geometry_msgs/PoseStamped           │
│  /visual_slam/tracking/slam_path        nav_msgs/Path          Trajectory   │
│  /visual_slam/vis/observations_cloud    sensor_msgs/PointCloud2 Features    │
│  /visual_slam/vis/landmarks_cloud       sensor_msgs/PointCloud2 Map points  │
│  /visual_slam/status                    isaac_ros_visual_slam_interfaces/   │
│                                         VisualSlamStatus                    │
│                                                                              │
│  TF Transforms:                                                              │
│  ──────────────                                                             │
│  map → odom           Loop closure corrections                              │
│  odom → base_link     Visual odometry (continuous)                          │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Perception Pipeline

### 3.1 Complete Perception Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Humanoid Robot Perception Pipeline                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌────────────────────────────────────────────────────────────────────┐   │
│   │                        Sensor Suite                                 │   │
│   │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │   │
│   │  │ Stereo   │  │  Depth   │  │   IMU    │  │  LiDAR   │           │   │
│   │  │ Camera   │  │  Camera  │  │ (6-DoF)  │  │ (Option) │           │   │
│   │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │   │
│   └───────┼─────────────┼─────────────┼─────────────┼──────────────────┘   │
│           │             │             │             │                       │
│           ▼             ▼             ▼             ▼                       │
│   ┌────────────────────────────────────────────────────────────────────┐   │
│   │                     Isaac ROS Processing                            │   │
│   │                                                                     │   │
│   │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐    │   │
│   │  │ Image Proc      │  │ Depth Proc      │  │ Point Cloud     │    │   │
│   │  │ • Rectification │  │ • Filtering     │  │ • Voxel filter  │    │   │
│   │  │ • Color convert │  │ • Hole filling  │  │ • Registration  │    │   │
│   │  │ • Resize        │  │ • Decimation    │  │ • Ground removal│    │   │
│   │  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘    │   │
│   │           │                    │                    │              │   │
│   └───────────┼────────────────────┼────────────────────┼──────────────┘   │
│               │                    │                    │                   │
│               ▼                    ▼                    ▼                   │
│   ┌────────────────────────────────────────────────────────────────────┐   │
│   │                     Perception Modules                              │   │
│   │                                                                     │   │
│   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌───────────┐ │   │
│   │  │  cuVSLAM    │  │  Detection  │  │ Segmentation│  │   Depth   │ │   │
│   │  │ • Odometry  │  │ • YOLO      │  │ • Semantic  │  │ Estimation│ │   │
│   │  │ • Mapping   │  │ • DOPE      │  │ • Instance  │  │ • Stereo  │ │   │
│   │  │ • Loop close│  │ • CenterNet │  │ • Panoptic  │  │ • Mono    │ │   │
│   │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬─────┘ │   │
│   │         │                │                │               │        │   │
│   └─────────┼────────────────┼────────────────┼───────────────┼────────┘   │
│             │                │                │               │             │
│             ▼                ▼                ▼               ▼             │
│   ┌────────────────────────────────────────────────────────────────────┐   │
│   │                     Output to Navigation                            │   │
│   │                                                                     │   │
│   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌───────────┐ │   │
│   │  │ Odometry    │  │ Detections  │  │ Costmap     │  │ 3D Map    │ │   │
│   │  │ /odom       │  │ /detections │  │ /costmap    │  │ /map      │ │   │
│   │  └─────────────┘  └─────────────┘  └─────────────┘  └───────────┘ │   │
│   │                                                                     │   │
│   └────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 ROS 2 Perception Node

```python
#!/usr/bin/env python3
"""
perception_node.py - Unified perception node combining VSLAM and detection
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from vision_msgs.msg import Detection3DArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

import numpy as np
import cv2
from cv_bridge import CvBridge


class PerceptionNode(Node):
    """Unified perception node for humanoid robot."""

    def __init__(self):
        super().__init__('perception_node')

        # Parameters
        self.declare_parameter('use_depth', True)
        self.declare_parameter('detection_model', 'yolov8')
        self.declare_parameter('publish_rate', 30.0)

        self.use_depth = self.get_parameter('use_depth').value
        self.detection_model = self.get_parameter('detection_model').value

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Callback group for parallel execution
        self.cb_group = ReentrantCallbackGroup()

        # CV Bridge
        self.bridge = CvBridge()

        # TF2
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.current_pose = None
        self.current_rgb = None
        self.current_depth = None
        self.path = Path()
        self.path.header.frame_id = 'map'

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10,
            callback_group=self.cb_group
        )

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            sensor_qos,
            callback_group=self.cb_group
        )

        if self.use_depth:
            self.depth_sub = self.create_subscription(
                Image,
                '/camera/depth/image_rect_raw',
                self.depth_callback,
                sensor_qos,
                callback_group=self.cb_group
            )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/robot_path',
            10
        )

        self.detection_pub = self.create_publisher(
            Detection3DArray,
            '/detections',
            10
        )

        # Processing timer
        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.process_callback)

        self.get_logger().info('Perception node initialized')

    def odom_callback(self, msg: Odometry):
        """Process odometry from VSLAM."""
        self.current_pose = msg.pose.pose

        # Add to path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.path.poses.append(pose_stamped)

        # Limit path length
        if len(self.path.poses) > 1000:
            self.path.poses = self.path.poses[-1000:]

        # Publish robot pose
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)

    def rgb_callback(self, msg: Image):
        """Process RGB image."""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')

    def depth_callback(self, msg: Image):
        """Process depth image."""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def process_callback(self):
        """Periodic processing callback."""
        # Publish path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

        # Run detection if we have images
        if self.current_rgb is not None:
            detections = self.run_detection(self.current_rgb, self.current_depth)
            if detections:
                self.detection_pub.publish(detections)

    def run_detection(self, rgb: np.ndarray, depth: np.ndarray = None):
        """Run object detection on current frame."""
        # Placeholder for actual detection
        # In production, use Isaac ROS DNN Inference or similar
        detections = Detection3DArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = 'camera_color_optical_frame'

        # Example detection logic would go here
        # Using YOLOv8, DOPE, or other detection models

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 4. Object Detection with Isaac ROS

### 4.1 DOPE (Deep Object Pose Estimation)

```python
#!/usr/bin/env python3
"""
launch/dope_detection_launch.py - Launch DOPE for 6-DoF object pose estimation
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # DOPE encoder (ResNet-based)
    dope_encoder_node = ComposableNode(
        name='dope_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 480,
            'network_image_width': 640,
            'network_image_height': 480,
            'image_mean': [0.485, 0.456, 0.406],
            'image_stddev': [0.229, 0.224, 0.225],
        }],
        remappings=[
            ('image', '/camera/color/image_raw'),
            ('encoded_tensor', '/tensor_pub'),
        ]
    )

    # TensorRT inference node
    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': '/workspaces/isaac_ros-dev/models/dope/soup_60.onnx',
            'engine_file_path': '/tmp/dope_soup.plan',
            'output_binding_names': ['output'],
            'output_tensor_names': ['output'],
            'input_tensor_names': ['input'],
            'input_binding_names': ['input'],
            'verbose': False,
            'force_engine_update': False,
        }]
    )

    # DOPE decoder
    dope_decoder_node = ComposableNode(
        name='dope_decoder',
        package='isaac_ros_dope',
        plugin='nvidia::isaac_ros::dope::DopeDecoderNode',
        parameters=[{
            'object_name': 'soup',
            'tf_frame_name': 'soup_frame',

            # Camera intrinsics (adjust for your camera)
            'camera_matrix': [
                615.0, 0.0, 320.0,
                0.0, 615.0, 240.0,
                0.0, 0.0, 1.0
            ],

            # Peak detection parameters
            'min_points': 4,
            'min_score': 0.3,
        }]
    )

    # Container
    container = ComposableNodeContainer(
        name='dope_container',
        namespace='dope',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            dope_encoder_node,
            tensor_rt_node,
            dope_decoder_node,
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

### 4.2 Detection Output Processing

```python
#!/usr/bin/env python3
"""
detection_processor.py - Process 3D object detections for manipulation
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class DetectionProcessor(Node):
    """Process object detections for robot manipulation."""

    def __init__(self):
        super().__init__('detection_processor')

        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            '/dope/detections',
            self.detection_callback,
            10
        )

        # Publish grasp targets
        self.grasp_pub = self.create_publisher(
            PoseStamped,
            '/grasp_target',
            10
        )

        self.get_logger().info('Detection processor ready')

    def detection_callback(self, msg: Detection3DArray):
        """Process incoming detections."""
        for detection in msg.detections:
            # Get object pose in camera frame
            object_pose = PoseStamped()
            object_pose.header = detection.header
            object_pose.pose = detection.results[0].pose.pose

            # Transform to robot base frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    detection.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                grasp_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                    object_pose, transform
                )

                # Adjust grasp pose (approach from above)
                grasp_pose.pose.position.z += 0.1  # 10cm above object

                self.get_logger().info(
                    f'Object detected at: '
                    f'x={grasp_pose.pose.position.x:.3f}, '
                    f'y={grasp_pose.pose.position.y:.3f}, '
                    f'z={grasp_pose.pose.position.z:.3f}'
                )

                self.grasp_pub.publish(grasp_pose)

            except Exception as e:
                self.get_logger().warn(f'Transform failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 5. Integration with Nav2

### 5.1 VSLAM to Nav2 Bridge

```python
#!/usr/bin/env python3
"""
launch/vslam_nav2_launch.py - Integrate cuVSLAM with Nav2 navigation stack
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_robot_dir = get_package_share_directory('my_robot_pkg')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default='')

    # Nav2 parameters
    nav2_params = os.path.join(my_robot_dir, 'config', 'nav2_params.yaml')

    # VSLAM node (from previous launch file)
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            my_robot_dir, '/launch/vslam_realsense_launch.py'
        ])
    )

    # Static transform: base_link to camera
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.1', '0', '0.5',  # x, y, z
            '0', '0', '0',      # roll, pitch, yaw
            'base_link',
            'camera_link'
        ]
    )

    # Odometry to base_link transform (handled by cuVSLAM, but may need relay)
    odom_relay = Node(
        package='my_robot_pkg',
        executable='odom_relay',
        name='odom_relay',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_dir, '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': map_file,
            'use_composition': 'True',
        }.items()
    )

    # RViz for visualization
    rviz_config = os.path.join(my_robot_dir, 'rviz', 'nav2_vslam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        static_tf,
        vslam_launch,
        odom_relay,
        nav2_bringup,
        rviz,
    ])
```

### 5.2 Nav2 Configuration for VSLAM

```yaml
# config/nav2_params.yaml

amcl:
  ros__parameters:
    # Disable AMCL when using VSLAM for localization
    use_sim_time: false

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    # DWB controller for humanoid navigation
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5  # Humanoid walking speed
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          sensor_frame: camera_depth_optical_frame
          observation_persistence: 0.0
          expected_update_rate: 0.0
          data_type: "PointCloud2"
          min_obstacle_height: 0.05
          max_obstacle_height: 2.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          sensor_frame: camera_depth_optical_frame
          data_type: "PointCloud2"
          marking: True
          clearing: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

---

## 6. Performance Evaluation

### 6.1 VSLAM Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| **ATE** | Absolute Trajectory Error (RMSE) | < 1% of distance |
| **RPE** | Relative Pose Error | < 0.5° rotation, < 1cm translation |
| **Tracking Rate** | Frames successfully tracked | > 95% |
| **Loop Closure** | Correct place recognition | > 90% recall |
| **Latency** | Pose estimation delay | < 33ms (30 FPS) |

### 6.2 Evaluation Script

```python
#!/usr/bin/env python3
"""
vslam_evaluation.py - Evaluate VSLAM performance against ground truth
"""

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt


def compute_ate(estimated_poses, ground_truth_poses):
    """Compute Absolute Trajectory Error."""
    assert len(estimated_poses) == len(ground_truth_poses)

    errors = []
    for est, gt in zip(estimated_poses, ground_truth_poses):
        error = np.linalg.norm(est[:3] - gt[:3])
        errors.append(error)

    ate_rmse = np.sqrt(np.mean(np.array(errors) ** 2))
    ate_mean = np.mean(errors)
    ate_median = np.median(errors)
    ate_std = np.std(errors)

    return {
        'rmse': ate_rmse,
        'mean': ate_mean,
        'median': ate_median,
        'std': ate_std,
    }


def compute_rpe(estimated_poses, ground_truth_poses, delta=1):
    """Compute Relative Pose Error."""
    trans_errors = []
    rot_errors = []

    for i in range(len(estimated_poses) - delta):
        # Estimated relative pose
        est_delta = compute_relative_pose(
            estimated_poses[i], estimated_poses[i + delta]
        )

        # Ground truth relative pose
        gt_delta = compute_relative_pose(
            ground_truth_poses[i], ground_truth_poses[i + delta]
        )

        # Compute error
        trans_error = np.linalg.norm(est_delta[:3] - gt_delta[:3])
        rot_error = compute_rotation_error(est_delta[3:], gt_delta[3:])

        trans_errors.append(trans_error)
        rot_errors.append(rot_error)

    return {
        'trans_rmse': np.sqrt(np.mean(np.array(trans_errors) ** 2)),
        'rot_rmse': np.sqrt(np.mean(np.array(rot_errors) ** 2)),
        'trans_mean': np.mean(trans_errors),
        'rot_mean': np.mean(rot_errors),
    }


def compute_relative_pose(pose1, pose2):
    """Compute relative pose between two poses."""
    # Assumes pose format: [x, y, z, qx, qy, qz, qw]
    T1 = pose_to_matrix(pose1)
    T2 = pose_to_matrix(pose2)
    T_rel = np.linalg.inv(T1) @ T2
    return matrix_to_pose(T_rel)


def pose_to_matrix(pose):
    """Convert pose [x,y,z,qx,qy,qz,qw] to 4x4 matrix."""
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    r = Rotation.from_quat(pose[3:])
    T[:3, :3] = r.as_matrix()
    return T


def matrix_to_pose(T):
    """Convert 4x4 matrix to pose [x,y,z,qx,qy,qz,qw]."""
    pose = np.zeros(7)
    pose[:3] = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3])
    pose[3:] = r.as_quat()
    return pose


def compute_rotation_error(q1, q2):
    """Compute angular error between two quaternions (degrees)."""
    r1 = Rotation.from_quat(q1)
    r2 = Rotation.from_quat(q2)
    r_error = r1.inv() * r2
    angle = np.abs(r_error.magnitude())
    return np.degrees(angle)


def plot_trajectory_comparison(estimated, ground_truth, output_path='trajectory.png'):
    """Plot estimated vs ground truth trajectory."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # XY view
    axes[0].plot(ground_truth[:, 0], ground_truth[:, 1], 'g-', label='Ground Truth')
    axes[0].plot(estimated[:, 0], estimated[:, 1], 'b--', label='Estimated')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Top View (XY)')
    axes[0].legend()
    axes[0].axis('equal')
    axes[0].grid(True)

    # XZ view
    axes[1].plot(ground_truth[:, 0], ground_truth[:, 2], 'g-', label='Ground Truth')
    axes[1].plot(estimated[:, 0], estimated[:, 2], 'b--', label='Estimated')
    axes[1].set_xlabel('X (m)')
    axes[1].set_ylabel('Z (m)')
    axes[1].set_title('Side View (XZ)')
    axes[1].legend()
    axes[1].grid(True)

    # Error over time
    errors = np.linalg.norm(estimated[:, :3] - ground_truth[:, :3], axis=1)
    axes[2].plot(errors)
    axes[2].set_xlabel('Frame')
    axes[2].set_ylabel('Position Error (m)')
    axes[2].set_title('Tracking Error Over Time')
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig(output_path)
    plt.show()

    return output_path


# Example usage
if __name__ == '__main__':
    # Load trajectories (replace with actual data)
    estimated = np.random.randn(1000, 7)
    ground_truth = estimated + np.random.randn(1000, 7) * 0.01

    # Compute metrics
    ate = compute_ate(estimated, ground_truth)
    rpe = compute_rpe(estimated, ground_truth)

    print("ATE (Absolute Trajectory Error):")
    print(f"  RMSE: {ate['rmse']:.4f} m")
    print(f"  Mean: {ate['mean']:.4f} m")
    print(f"  Median: {ate['median']:.4f} m")

    print("\nRPE (Relative Pose Error):")
    print(f"  Translation RMSE: {rpe['trans_rmse']:.4f} m")
    print(f"  Rotation RMSE: {rpe['rot_rmse']:.4f} deg")

    # Plot
    plot_trajectory_comparison(estimated, ground_truth)
```

---

## 7. Summary

### Perception Pipeline Components

| Component | Tool | Purpose |
|-----------|------|---------|
| **VSLAM** | cuVSLAM | 6-DoF localization |
| **Object Detection** | DOPE, YOLO | Object pose estimation |
| **Depth Estimation** | Stereo matching | 3D reconstruction |
| **Navigation** | Nav2 | Path planning |

### Key Isaac ROS Packages

| Package | Function |
|---------|----------|
| `isaac_ros_visual_slam` | GPU-accelerated VSLAM |
| `isaac_ros_dope` | 6-DoF object pose |
| `isaac_ros_image_proc` | Image processing |
| `isaac_ros_tensor_rt` | DNN inference |

---

## Exercises

### Exercise 9.1: cuVSLAM Setup (⭐⭐)

1. Install Isaac ROS Docker environment
2. Connect a RealSense D435i camera
3. Launch cuVSLAM with stereo + IMU
4. Record a trajectory and visualize in RViz

### Exercise 9.2: Custom Detection Pipeline (⭐⭐⭐)

Create a detection pipeline that:
- Uses DOPE for object pose estimation
- Transforms detections to robot base frame
- Publishes grasp candidates
- Visualize in RViz with markers

### Exercise 9.3: Nav2 Integration (⭐⭐⭐)

Integrate cuVSLAM with Nav2:
- Configure costmaps with depth camera
- Set up DWB controller for humanoid
- Command robot to navigate to detected object
- Record and evaluate trajectory

### Exercise 9.4: VSLAM Evaluation (⭐⭐)

Evaluate your VSLAM system:
- Record data with ground truth (simulation or motion capture)
- Compute ATE and RPE metrics
- Generate trajectory comparison plots
- Identify failure cases

---

## Quiz

<details>
<summary>Q1: What are the main components of a VSLAM front-end?</summary>

The VSLAM front-end handles real-time processing:
- Feature detection (ORB, FAST, Harris corners)
- Feature tracking/matching between frames
- Visual odometry (frame-to-frame motion estimation)
- Local mapping (triangulation of 3D points)

The front-end runs at camera frame rate (30-60 FPS).

</details>

<details>
<summary>Q2: Why is IMU fusion beneficial for VSLAM?</summary>

IMU fusion provides:
- Motion prediction during visual tracking failures
- Scale estimation for monocular cameras
- Gravity direction for roll/pitch initialization
- Higher frequency state updates (200+ Hz)
- Robustness to fast motion and blur

cuVSLAM uses IMU for visual-inertial odometry (VIO).

</details>

<details>
<summary>Q3: What is loop closure and why is it important?</summary>

Loop closure detects when the robot returns to a previously visited location:
- Corrects accumulated drift in odometry
- Maintains global map consistency
- Uses place recognition (bag-of-words, learned descriptors)
- Triggers pose graph optimization

Without loop closure, SLAM drift grows unbounded over time.

</details>

<details>
<summary>Q4: How do you handle the coordinate transform from VSLAM to Nav2?</summary>

Coordinate transforms require:
1. VSLAM publishes map → odom → base_link transforms
2. Static transform from base_link to camera_link
3. Odometry topic remapping to Nav2 expected topic
4. Frame ID consistency in costmap configuration
5. Time synchronization between VSLAM and Nav2

Use tf2_ros for transform management.

</details>

---

## Next Steps

In [Week 10: Sim-to-Real Transfer](/module-3-isaac/week-10-sim-to-real), we explore:
- The sim-to-real gap and its causes
- Domain randomization techniques
- Training policies in Isaac Sim
- Deploying to real hardware
