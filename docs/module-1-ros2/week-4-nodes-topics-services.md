---
sidebar_position: 4
---

# Week 4: Nodes, Topics, and Services in Depth

## Learning Objectives

By the end of this week, you will be able to:

- **Design** multi-node architectures for robotics applications
- **Create** custom message and service definitions
- **Implement** advanced publisher/subscriber patterns
- **Debug** ROS 2 communication issues
- **Apply** best practices for node organization

---

## 1. Multi-Node Architecture Design

### 1.1 Node Design Principles

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Node Design Principles                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ✓ Single Responsibility                                                  │
│      Each node does ONE thing well                                          │
│                                                                              │
│    ✓ Loose Coupling                                                         │
│      Nodes communicate via topics/services, not direct calls                │
│                                                                              │
│    ✓ High Cohesion                                                          │
│      Related functionality grouped in same node                             │
│                                                                              │
│    ✓ Reusability                                                            │
│      Nodes should be usable in different robot configurations               │
│                                                                              │
│    ✗ God Nodes                                                              │
│      Avoid nodes that do everything                                         │
│                                                                              │
│    ✗ Tight Coupling                                                         │
│      Avoid hardcoded node names or direct dependencies                      │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Example: Humanoid Robot Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                  Humanoid Robot Node Graph                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                           ┌─────────────┐                                   │
│                           │  Supervisor │                                   │
│                           │    Node     │                                   │
│                           └──────┬──────┘                                   │
│                                  │                                          │
│            ┌─────────────────────┼─────────────────────┐                   │
│            │                     │                     │                   │
│            ▼                     ▼                     ▼                   │
│    ┌──────────────┐     ┌──────────────┐     ┌──────────────┐            │
│    │  Perception  │     │   Planning   │     │   Control    │            │
│    │    Node      │────►│    Node      │────►│    Node      │            │
│    └──────────────┘     └──────────────┘     └──────────────┘            │
│            │                     │                     │                   │
│            │                     │                     │                   │
│    ┌───────┴───────┐     ┌───────┴───────┐     ┌───────┴───────┐         │
│    │               │     │               │     │               │         │
│    ▼               ▼     ▼               ▼     ▼               ▼         │
│ ┌──────┐      ┌──────┐ ┌──────┐    ┌──────┐ ┌──────┐    ┌──────┐       │
│ │Camera│      │LiDAR │ │ Path │    │ Task │ │ Arm  │    │ Leg  │       │
│ │Driver│      │Driver│ │Planner│   │Planner│ │Cntrl │    │Cntrl │       │
│ └──────┘      └──────┘ └──────┘    └──────┘ └──────┘    └──────┘       │
│                                                                          │
│  Topics:                                                                 │
│  /camera/image         /perception/objects      /cmd_vel                 │
│  /lidar/scan           /plan/path               /joint_commands          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Custom Message Types

### 2.1 Message Definition

Create custom messages for your robot's data:

```
# File: msg/RobotState.msg

# Header with timestamp
std_msgs/Header header

# Robot identification
string robot_id

# Position (meters)
float64 x
float64 y
float64 z

# Orientation (quaternion)
float64 qx
float64 qy
float64 qz
float64 qw

# Velocity (m/s and rad/s)
float64 linear_velocity
float64 angular_velocity

# Joint states (variable length array)
float64[] joint_positions
float64[] joint_velocities

# Battery level (0-100)
uint8 battery_percentage

# Status flags
bool is_moving
bool is_emergency_stopped
```

### 2.2 Package Configuration

**package.xml:**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>humanoid_msgs</name>
  <version>0.1.0</version>
  <description>Custom messages for humanoid robot</description>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.8)
project(humanoid_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/JointCommand.msg"
  "srv/GetRobotState.srv"
  DEPENDENCIES std_msgs
)

ament_package()
```

### 2.3 Using Custom Messages

```python
#!/usr/bin/env python3
"""
robot_state_publisher.py - Publishes custom RobotState messages
"""

import rclpy
from rclpy.node import Node
from humanoid_msgs.msg import RobotState
from std_msgs.msg import Header


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        self.publisher = self.create_publisher(
            RobotState,
            'robot_state',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz

    def publish_state(self):
        msg = RobotState()

        # Fill header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Fill robot state
        msg.robot_id = 'humanoid_01'
        msg.x = 1.0
        msg.y = 2.0
        msg.z = 0.0
        msg.linear_velocity = 0.5
        msg.battery_percentage = 85
        msg.is_moving = True
        msg.is_emergency_stopped = False

        # Joint positions (example: 6 joints)
        msg.joint_positions = [0.0, 0.5, -0.3, 0.0, 0.2, 0.1]
        msg.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher.publish(msg)
```

---

## 3. Custom Service Types

### 3.1 Service Definition

```
# File: srv/GetRobotState.srv

# Request
string robot_id

---

# Response
bool success
string message
humanoid_msgs/RobotState state
```

### 3.2 Service Implementation

```python
#!/usr/bin/env python3
"""
robot_state_service.py - Service to get robot state
"""

import rclpy
from rclpy.node import Node
from humanoid_msgs.srv import GetRobotState
from humanoid_msgs.msg import RobotState


class RobotStateService(Node):
    def __init__(self):
        super().__init__('robot_state_service')

        self.srv = self.create_service(
            GetRobotState,
            'get_robot_state',
            self.get_state_callback
        )

        # Store current state
        self.current_state = RobotState()
        self.current_state.robot_id = 'humanoid_01'

        self.get_logger().info('Robot state service ready')

    def get_state_callback(self, request, response):
        if request.robot_id == self.current_state.robot_id:
            response.success = True
            response.message = 'State retrieved successfully'
            response.state = self.current_state
        else:
            response.success = False
            response.message = f'Unknown robot: {request.robot_id}'

        return response
```

---

## 4. Advanced Patterns

### 4.1 Synchronized Subscribers

When you need data from multiple topics at the same time:

```python
import message_filters
from sensor_msgs.msg import Image, CameraInfo


class SynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('sync_subscriber')

        # Create subscribers
        self.image_sub = message_filters.Subscriber(
            self, Image, '/camera/image'
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/info'
        )

        # Synchronize (approximate time sync)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.synced_callback)

    def synced_callback(self, image_msg, info_msg):
        """Called when both messages arrive at approximately same time."""
        self.get_logger().info(
            f'Received synced: image={image_msg.header.stamp}, '
            f'info={info_msg.header.stamp}'
        )
```

### 4.2 Timer-Based State Machine

```python
from enum import Enum, auto


class RobotState(Enum):
    IDLE = auto()
    MOVING = auto()
    GRASPING = auto()
    ERROR = auto()


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')

        self.state = RobotState.IDLE

        # State machine timer
        self.timer = self.create_timer(0.1, self.state_machine_tick)

        # Subscribers for events
        self.cmd_sub = self.create_subscription(
            String, 'command', self.command_callback, 10
        )

    def state_machine_tick(self):
        """Main state machine loop."""
        if self.state == RobotState.IDLE:
            self.handle_idle()
        elif self.state == RobotState.MOVING:
            self.handle_moving()
        elif self.state == RobotState.GRASPING:
            self.handle_grasping()
        elif self.state == RobotState.ERROR:
            self.handle_error()

    def transition_to(self, new_state: RobotState):
        """Transition to new state."""
        self.get_logger().info(f'Transition: {self.state} -> {new_state}')
        self.state = new_state
```

### 4.3 Component Composition

Run multiple nodes in a single process for efficiency:

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()

    # Create multiple nodes
    perception_node = PerceptionNode()
    planning_node = PlanningNode()
    control_node = ControlNode()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(perception_node)
    executor.add_node(planning_node)
    executor.add_node(control_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        perception_node.destroy_node()
        planning_node.destroy_node()
        control_node.destroy_node()
        rclpy.shutdown()
```

---

## 5. Debugging Tools

### 5.1 rqt_graph

Visualize node connections:

```bash
ros2 run rqt_graph rqt_graph
```

### 5.2 Topic Debugging

```bash
# Check topic frequency
ros2 topic hz /robot_state

# Check message bandwidth
ros2 topic bw /camera/image

# Delay analysis
ros2 topic delay /robot_state
```

### 5.3 Logging Best Practices

```python
class WellLoggedNode(Node):
    def __init__(self):
        super().__init__('well_logged_node')

        # Different log levels
        self.get_logger().debug('Debug info (verbose)')
        self.get_logger().info('Normal operation info')
        self.get_logger().warn('Warning - something unusual')
        self.get_logger().error('Error - something failed')
        self.get_logger().fatal('Fatal - cannot continue')

    def process_data(self, data):
        # Throttled logging (every 5 seconds)
        self.get_logger().info(
            f'Processing data: {data}',
            throttle_duration_sec=5.0
        )

        # Log only once
        self.get_logger().info(
            'First time setup complete',
            once=True
        )
```

---

## 6. Summary

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Custom Messages** | Domain-specific data | RobotState, JointCommand |
| **Custom Services** | Domain-specific queries | GetRobotState, Calibrate |
| **Synchronized Subs** | Multi-sensor fusion | Camera + Depth alignment |
| **State Machine** | Behavior sequencing | Task execution |
| **Composition** | Performance | Single-process multi-node |

---

## Exercises

### Exercise 4.1: Create Custom Message (⭐⭐)

Create a message type for humanoid joint commands with:
- Joint names (string array)
- Target positions (float64 array)
- Target velocities (float64 array)
- Max effort (float64 array)

### Exercise 4.2: Multi-Node System (⭐⭐⭐)

Design and implement a 3-node system:
1. **Sensor Node**: Publishes fake sensor data
2. **Processing Node**: Subscribes, processes, publishes result
3. **Logger Node**: Subscribes and logs all data

### Exercise 4.3: Service with Validation (⭐⭐)

Create a service that:
- Accepts a target position (x, y, z)
- Validates it's within workspace bounds
- Returns success/failure with appropriate message

---

## Quiz

<details>
<summary>Q1: Why use custom messages instead of primitive types?</summary>

Custom messages provide type safety, documentation, and structured data that primitive types cannot. They also enable automatic serialization and code generation.

</details>

<details>
<summary>Q2: When would you use message_filters.ApproximateTimeSynchronizer?</summary>

When you need data from multiple topics that were captured at approximately the same time, such as syncing camera images with IMU data for sensor fusion.

</details>

<details>
<summary>Q3: What is the benefit of node composition?</summary>

Running multiple nodes in a single process reduces inter-process communication overhead, lowers latency, and reduces memory usage.

</details>

---

## Next Steps

In [Week 5: Python Packages and Launch Files](/module-1-ros2/week-5-python-packages), we learn:
- Creating complete ROS 2 Python packages
- Launch files for complex systems
- Parameter management
- Testing ROS 2 nodes
