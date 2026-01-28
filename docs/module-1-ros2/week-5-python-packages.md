---
sidebar_position: 5
---

# Week 5: Python Packages and Launch Files

## Learning Objectives

By the end of this week, you will be able to:

- Create a ROS 2 Python package from scratch
- Write and configure `setup.py` and `package.xml`
- Create launch files to start multiple nodes
- Manage parameters and configurations

## Creating a ROS 2 Package

### Package Structure

```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── my_node.py
│   └── utils.py
├── launch/
│   └── robot_launch.py
├── config/
│   └── params.yaml
├── resource/
│   └── my_robot_pkg
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

### Create Package Command

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

## package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>Physical AI Robot Package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## setup.py

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Physical AI Robot Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
        ],
    },
)
```

## Launch Files

### Python Launch File

```python
# launch/robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_01',
            description='Name of the robot'
        ),

        Node(
            package='my_robot_pkg',
            executable='my_node',
            name='robot_controller',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'update_rate': 100.0,
            }],
            output='screen',
        ),

        Node(
            package='my_robot_pkg',
            executable='sensor_node',
            name='sensor_processor',
            remappings=[
                ('/input/camera', '/robot/camera/raw'),
            ],
        ),
    ])
```

### Running Launch Files

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash

# Run the launch file
ros2 launch my_robot_pkg robot_launch.py

# With arguments
ros2 launch my_robot_pkg robot_launch.py robot_name:=atlas_01
```

## Parameter Files

### config/params.yaml

```yaml
robot_controller:
  ros__parameters:
    robot_name: "humanoid_01"
    update_rate: 100.0
    joint_limits:
      hip: [-1.57, 1.57]
      knee: [0.0, 2.5]
      ankle: [-0.5, 0.5]
    sensors:
      camera_enabled: true
      lidar_enabled: true
      imu_frequency: 200
```

### Loading Parameters in Launch

```python
Node(
    package='my_robot_pkg',
    executable='my_node',
    parameters=[
        os.path.join(get_package_share_directory('my_robot_pkg'),
                     'config', 'params.yaml')
    ],
)
```

## Exercises

1. Create a complete ROS 2 package for a humanoid robot
2. Write a launch file that starts 3 nodes (controller, perception, planning)
3. Create a parameter file and load it in your launch file
4. Build and test your package

## Assessment: ROS 2 Package Development

This week's assessment requires you to create a functional ROS 2 package. See [Week 14: Capstone](/assessments/week-14-capstone) for full requirements.

## Next Steps

In [Week 6](/module-2-digital-twin/week-6-gazebo-setup), we enter Module 2 and begin building our digital twin in Gazebo.
