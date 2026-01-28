---
sidebar_position: 2
---

# Week 9: Perception and VSLAM

## Learning Objectives

By the end of this week, you will be able to:

- Understand Visual SLAM concepts and algorithms
- Use Isaac ROS for hardware-accelerated perception
- Implement visual odometry for humanoid navigation
- Integrate perception with ROS 2 navigation stack

## What is VSLAM?

**Visual Simultaneous Localization and Mapping (VSLAM)** enables robots to:

- Build a map of the environment
- Localize within that map
- Using only camera input

```
Camera Frames ──► Feature Detection ──► Feature Matching
                                              │
                                              ▼
                                        Motion Estimation
                                              │
                        ┌─────────────────────┴─────────────────────┐
                        │                                           │
                        ▼                                           ▼
                  Localization                               Map Building
                  (Where am I?)                            (What's around?)
```

## Isaac ROS Visual SLAM

NVIDIA provides **hardware-accelerated VSLAM** via cuVSLAM:

- Runs on GPU (CUDA cores + Tensor cores)
- Real-time performance at 60+ FPS
- Supports stereo cameras and depth cameras

### Installation

```bash
# Clone Isaac ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Build with Docker
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container
cd /workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_visual_slam
source install/setup.bash
```

### Launch VSLAM

```python
# launch/vslam_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/left/image_raw'),
            ('visual_slam/image_1', '/camera/right/image_raw'),
            ('visual_slam/imu', '/imu'),
        ]
    )

    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
```

## Perception Pipeline

```
                    ┌─────────────────┐
                    │  RGB-D Camera   │
                    │  (RealSense)    │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
        ┌──────────┐  ┌──────────┐  ┌──────────┐
        │   RGB    │  │  Depth   │  │   IMU    │
        │  Stream  │  │  Stream  │  │  Stream  │
        └────┬─────┘  └────┬─────┘  └────┬─────┘
             │             │             │
             └─────────────┼─────────────┘
                           │
                           ▼
                    ┌──────────────┐
                    │  Isaac ROS   │
                    │   cuVSLAM    │
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
              ▼            ▼            ▼
        ┌──────────┐ ┌──────────┐ ┌──────────┐
        │   Pose   │ │ Point    │ │ Visual   │
        │ Estimate │ │ Cloud    │ │ Odometry │
        └──────────┘ └──────────┘ └──────────┘
```

## Object Detection with Isaac ROS

```python
# Using DOPE (Deep Object Pose Estimation)
dope_node = ComposableNode(
    name='dope_node',
    package='isaac_ros_dope',
    plugin='nvidia::isaac_ros::dope::DopeDecoderNode',
    parameters=[{
        'model_file_path': '/path/to/model.onnx',
        'object_name': 'soup_can',
    }],
)
```

## Integration with Navigation

```python
# Connect VSLAM to Nav2
from nav2_msgs.action import NavigateToPose

class VslamNavigator(Node):
    def __init__(self):
        super().__init__('vslam_navigator')

        # Subscribe to VSLAM pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/odometry',
            self.pose_callback,
            10
        )

        # Navigation action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
```

## Exercises

1. Set up Isaac ROS Docker environment
2. Run cuVSLAM with a RealSense camera
3. Visualize the point cloud in RViz2
4. Record and replay VSLAM data with ros2 bag

## Next Steps

In [Week 10](/module-3-isaac/week-10-sim-to-real), we explore Sim-to-Real transfer techniques.
