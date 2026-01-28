---
sidebar_position: 5
---

# Week 5: Python Packages and Launch Files

## Learning Objectives

By the end of this week, you will be able to:

- **Create** complete ROS 2 Python packages from scratch
- **Write** launch files to start complex multi-node systems
- **Manage** parameters through YAML configuration files
- **Test** ROS 2 nodes using pytest
- **Build** and install packages correctly

---

## 1. Creating a ROS 2 Python Package

### 1.1 Package Creation

```bash
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python \
    --node-name my_first_node \
    my_robot_pkg \
    --dependencies rclpy std_msgs geometry_msgs
```

### 1.2 Package Structure

```
my_robot_pkg/
├── my_robot_pkg/              # Python module
│   ├── __init__.py
│   ├── robot_controller.py    # Main node
│   ├── perception_node.py     # Perception node
│   └── utils/                 # Utility modules
│       ├── __init__.py
│       └── math_utils.py
├── config/                    # Configuration files
│   └── robot_params.yaml
├── launch/                    # Launch files
│   ├── robot_launch.py
│   └── simulation_launch.py
├── resource/                  # Package marker
│   └── my_robot_pkg
├── test/                      # Tests
│   ├── test_copyright.py
│   └── test_robot_controller.py
├── package.xml                # Package manifest
├── setup.py                   # Python setup
└── setup.cfg                  # Setup configuration
```

### 1.3 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.1.0</version>
  <description>Humanoid robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 1.4 setup.py

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Humanoid robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_pkg.robot_controller:main',
            'perception_node = my_robot_pkg.perception_node:main',
        ],
    },
)
```

---

## 2. Launch Files

### 2.1 Basic Launch File

```python
# launch/robot_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[{
                'robot_name': 'humanoid_01',
                'control_rate': 100.0,
            }],
        ),
        Node(
            package='my_robot_pkg',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),
    ])
```

### 2.2 Launch File with Arguments

```python
# launch/configurable_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_01',
        description='Name of the robot'
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Whether to run in simulation mode'
    )

    # Use arguments
    robot_controller = Node(
        package='my_robot_pkg',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'simulation': LaunchConfiguration('simulation'),
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        simulation_arg,
        robot_controller,
    ])
```

### 2.3 Launch with YAML Parameters

```yaml
# config/robot_params.yaml

robot_controller:
  ros__parameters:
    robot_name: "humanoid_01"
    control_rate: 100.0

    # Joint configuration
    joints:
      names: ["hip_pitch", "hip_roll", "knee", "ankle_pitch", "ankle_roll"]
      limits:
        position_min: [-1.57, -0.5, 0.0, -0.8, -0.3]
        position_max: [1.57, 0.5, 2.5, 0.8, 0.3]

    # Control gains
    pid:
      p: 100.0
      i: 0.1
      d: 10.0

perception_node:
  ros__parameters:
    camera_topic: "/camera/image_raw"
    detection_threshold: 0.7
```

```python
# launch/params_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_pkg')

    # Path to params file
    params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='my_robot_pkg',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
```

### 2.4 Including Other Launch Files

```python
# launch/full_system_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')

    # Include robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'robot_name': 'humanoid_01',
        }.items()
    )

    # Include gazebo launch (from another package)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_launch,
    ])
```

---

## 3. Parameter Management

### 3.1 Declaring and Using Parameters

```python
#!/usr/bin/env python3
"""
robot_controller.py - Robot controller with parameters
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters with descriptions
        self.declare_parameter(
            'robot_name',
            'default_robot',
            ParameterDescriptor(
                description='Name of the robot',
                type=ParameterType.PARAMETER_STRING
            )
        )

        self.declare_parameter(
            'control_rate',
            100.0,
            ParameterDescriptor(
                description='Control loop rate in Hz',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

        self.declare_parameter(
            'joints.names',
            ['joint1', 'joint2'],
            ParameterDescriptor(description='Joint names')
        )

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.joint_names = self.get_parameter('joints.names').value

        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Rate: {self.control_rate} Hz')
        self.get_logger().info(f'Joints: {self.joint_names}')

        # Create control timer
        period = 1.0 / self.control_rate
        self.timer = self.create_timer(period, self.control_loop)

        # Parameter callback for runtime updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Handle parameter updates at runtime."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'control_rate':
                self.control_rate = param.value
                # Recreate timer with new rate
                self.timer.cancel()
                period = 1.0 / self.control_rate
                self.timer = self.create_timer(period, self.control_loop)
                self.get_logger().info(f'Control rate updated to {self.control_rate} Hz')

        return SetParametersResult(successful=True)

    def control_loop(self):
        """Main control loop."""
        # Control logic here
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 4. Testing ROS 2 Nodes

### 4.1 Unit Test Example

```python
# test/test_robot_controller.py

import pytest
import rclpy
from my_robot_pkg.robot_controller import RobotController


@pytest.fixture(scope='module')
def ros_context():
    """Initialize ROS 2 context for tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestRobotController:
    def test_node_creation(self, ros_context):
        """Test that node can be created."""
        node = RobotController()
        assert node.get_name() == 'robot_controller'
        node.destroy_node()

    def test_default_parameters(self, ros_context):
        """Test default parameter values."""
        node = RobotController()
        assert node.robot_name == 'default_robot'
        assert node.control_rate == 100.0
        node.destroy_node()

    def test_parameter_update(self, ros_context):
        """Test parameter update callback."""
        node = RobotController()

        # Update parameter
        from rclpy.parameter import Parameter
        param = Parameter('control_rate', Parameter.Type.DOUBLE, 50.0)
        result = node.set_parameters([param])

        assert result[0].successful
        assert node.control_rate == 50.0
        node.destroy_node()
```

### 4.2 Running Tests

```bash
# Build with tests
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg

# Run tests
colcon test --packages-select my_robot_pkg

# View test results
colcon test-result --verbose
```

---

## 5. Build and Install

### 5.1 Building Packages

```bash
cd ~/ros2_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_pkg

# Build with symlinks (for development)
colcon build --symlink-install --packages-select my_robot_pkg

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 5.2 Sourcing Workspace

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Or add to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 5.3 Running Your Package

```bash
# Run single node
ros2 run my_robot_pkg robot_controller

# Run with parameters
ros2 run my_robot_pkg robot_controller --ros-args \
    -p robot_name:=my_humanoid \
    -p control_rate:=50.0

# Run launch file
ros2 launch my_robot_pkg robot_launch.py

# Run with arguments
ros2 launch my_robot_pkg configurable_launch.py robot_name:=atlas
```

---

## 6. Summary

### Package Checklist

- [ ] `package.xml` with correct dependencies
- [ ] `setup.py` with entry points
- [ ] `__init__.py` in all Python directories
- [ ] Launch files in `launch/` directory
- [ ] Config files in `config/` directory
- [ ] Tests in `test/` directory

### Key Commands

| Command | Purpose |
|---------|---------|
| `ros2 pkg create` | Create new package |
| `colcon build` | Build packages |
| `colcon test` | Run tests |
| `ros2 run` | Run single node |
| `ros2 launch` | Run launch file |

---

## Exercises

### Exercise 5.1: Create Complete Package (⭐⭐)

Create a package `humanoid_control` with:
- Two nodes (controller and monitor)
- YAML parameter file
- Launch file starting both nodes

### Exercise 5.2: Launch File Features (⭐⭐)

Create a launch file that:
- Accepts `simulation` argument (true/false)
- Loads different params based on argument
- Conditionally starts nodes

### Exercise 5.3: Testing (⭐⭐⭐)

Write tests for:
- Node initialization
- Parameter validation
- Message publishing (mock subscriber)

---

## Module 1 Assessment: ROS 2 Package Development

**Deliverable**: Complete ROS 2 package with:

1. **Nodes** (2+): Publisher/subscriber pair
2. **Custom Messages**: At least one custom message type
3. **Parameters**: YAML configuration file
4. **Launch File**: Starts all nodes with parameters
5. **Tests**: Basic unit tests

See [Week 14: Capstone](/assessments/week-14-capstone) for full requirements.

---

## Next Steps

In [Week 6: Gazebo Simulation Setup](/module-2-digital-twin/week-6-gazebo-setup), we begin Module 2:
- Gazebo simulation environment
- URDF robot models
- Physics simulation
- Sensor simulation
