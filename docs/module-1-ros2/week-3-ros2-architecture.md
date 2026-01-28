---
sidebar_position: 3
---

# Week 3: ROS 2 Architecture

## Learning Objectives

By the end of this week, you will be able to:

- **Explain** the ROS 2 architecture and its advantages over ROS 1
- **Configure** a complete ROS 2 development environment
- **Create** functional ROS 2 nodes using Python (rclpy)
- **Understand** the DDS middleware and its role in communication
- **Navigate** the ROS 2 ecosystem and tooling

---

## 1. Why ROS 2?

### 1.1 ROS 1 Limitations

ROS 1 was groundbreaking but had architectural limitations:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS 1 Architecture                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                         ┌─────────────────┐                                 │
│                         │   ROS Master    │                                 │
│                         │  (Single Point  │                                 │
│                         │   of Failure!)  │                                 │
│                         └────────┬────────┘                                 │
│                                  │                                          │
│              ┌───────────────────┼───────────────────┐                     │
│              │                   │                   │                     │
│              ▼                   ▼                   ▼                     │
│         ┌─────────┐        ┌─────────┐        ┌─────────┐                 │
│         │  Node   │        │  Node   │        │  Node   │                 │
│         │    A    │◄──────►│    B    │◄──────►│    C    │                 │
│         └─────────┘        └─────────┘        └─────────┘                 │
│                     TCP/IP (TCPROS)                                        │
│                                                                             │
│    Problems:                                                                │
│    • Master required (what if it dies?)                                     │
│    • No real-time support                                                   │
│    • Security? What security?                                               │
│    • Python 2 only (initially)                                              │
│    • Linux only                                                             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 ROS 2 Improvements

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Master** | Required (single point of failure) | No master (peer-to-peer) |
| **Real-time** | No | Yes (with proper setup) |
| **Security** | None | DDS Security |
| **Multi-robot** | Difficult | Native support |
| **Platforms** | Linux only | Linux, Windows, macOS |
| **Python** | 2.7 | 3.8+ |
| **Middleware** | Custom TCPROS | Standard DDS |

### 1.3 ROS 2 Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Architecture                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌──────────────────────────────────────────────────────────────────┐    │
│    │                      Application Layer                            │    │
│    │                  (Your Nodes / Packages)                          │    │
│    └──────────────────────────────┬───────────────────────────────────┘    │
│                                   │                                         │
│    ┌──────────────────────────────┴───────────────────────────────────┐    │
│    │                    ROS Client Libraries                           │    │
│    │        ┌─────────┐   ┌─────────┐   ┌─────────┐                   │    │
│    │        │  rclpy  │   │  rclcpp │   │  rclrs  │                   │    │
│    │        │(Python) │   │  (C++)  │   │ (Rust)  │                   │    │
│    │        └────┬────┘   └────┬────┘   └────┬────┘                   │    │
│    └─────────────┼─────────────┼─────────────┼────────────────────────┘    │
│                  │             │             │                              │
│    ┌─────────────┴─────────────┴─────────────┴────────────────────────┐    │
│    │                   RCL (ROS Client Library)                        │    │
│    │                   Common C implementation                         │    │
│    └──────────────────────────────┬───────────────────────────────────┘    │
│                                   │                                         │
│    ┌──────────────────────────────┴───────────────────────────────────┐    │
│    │                   RMW (ROS Middleware Interface)                  │    │
│    │                   Abstraction over DDS vendors                    │    │
│    └──────────────────────────────┬───────────────────────────────────┘    │
│                                   │                                         │
│    ┌──────────────────────────────┴───────────────────────────────────┐    │
│    │                      DDS Implementation                           │    │
│    │   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │    │
│    │   │   Fast DDS   │  │ Cyclone DDS  │  │ Connext DDS  │          │    │
│    │   │   (Default)  │  │  (Eclipse)   │  │    (RTI)     │          │    │
│    │   └──────────────┘  └──────────────┘  └──────────────┘          │    │
│    └──────────────────────────────────────────────────────────────────┘    │
│                                                                             │
│    No Master Required! Nodes discover each other via DDS                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Environment Setup

### 2.1 System Requirements

| Requirement | Specification |
|-------------|---------------|
| **OS** | Ubuntu 22.04 LTS (recommended) |
| **RAM** | 8 GB minimum, 16 GB recommended |
| **Storage** | 20 GB free space |
| **CPU** | x86_64 or ARM64 |

### 2.2 Install ROS 2 Humble

```bash
#!/bin/bash
# install_ros2_humble.sh

# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y

# Source setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

### 2.3 Create Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build (even empty workspace)
colcon build

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.4 Workspace Structure

```
ros2_ws/
├── build/           # Build artifacts (generated)
├── install/         # Installed packages (generated)
├── log/             # Build logs (generated)
└── src/             # Source code (you create this)
    ├── my_package_1/
    │   ├── my_package_1/
    │   │   ├── __init__.py
    │   │   └── my_node.py
    │   ├── resource/
    │   ├── test/
    │   ├── package.xml
    │   └── setup.py
    └── my_package_2/
        └── ...
```

---

## 3. ROS 2 Core Concepts

### 3.1 Nodes

A **node** is a process that performs computation:

```python
#!/usr/bin/env python3
"""
minimal_node.py - The simplest ROS 2 node
"""

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    """A minimal ROS 2 node."""

    def __init__(self):
        # Initialize node with name
        super().__init__('minimal_node')

        # Log message
        self.get_logger().info('Hello from minimal_node!')


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = MinimalNode()

    # Spin (keep node alive)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.2 Node Lifecycle

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        ROS 2 Node Lifecycle                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌─────────────┐                                                          │
│    │   Created   │   Node object instantiated                               │
│    └──────┬──────┘                                                          │
│           │                                                                  │
│           ▼                                                                  │
│    ┌─────────────┐                                                          │
│    │ Unconfigured│   Parameters loaded, but not active                      │
│    └──────┬──────┘                                                          │
│           │ configure()                                                      │
│           ▼                                                                  │
│    ┌─────────────┐                                                          │
│    │  Inactive   │   Configured but not processing                          │
│    └──────┬──────┘                                                          │
│           │ activate()                                                       │
│           ▼                                                                  │
│    ┌─────────────┐                                                          │
│    │   Active    │◄──────────────────────────────────────┐                 │
│    │             │   Processing callbacks                 │                 │
│    └──────┬──────┘                                        │                 │
│           │ deactivate()                          activate()                │
│           ▼                                                │                 │
│    ┌─────────────┐                                        │                 │
│    │  Inactive   │────────────────────────────────────────┘                 │
│    └──────┬──────┘                                                          │
│           │ cleanup()                                                        │
│           ▼                                                                  │
│    ┌─────────────┐                                                          │
│    │ Unconfigured│                                                          │
│    └──────┬──────┘                                                          │
│           │ shutdown()                                                       │
│           ▼                                                                  │
│    ┌─────────────┐                                                          │
│    │  Finalized  │   Node destroyed                                         │
│    └─────────────┘                                                          │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 Topics (Publish/Subscribe)

Topics provide **asynchronous, many-to-many** communication:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Topic Communication                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Publishers                Topic                   Subscribers             │
│                                                                              │
│    ┌──────────┐                                     ┌──────────┐           │
│    │ Camera   │─────┐                         ┌────►│ Display  │           │
│    │  Node    │     │     ┌──────────────┐    │     │  Node    │           │
│    └──────────┘     │     │              │    │     └──────────┘           │
│                     ├────►│  /camera/    │────┤                            │
│    ┌──────────┐     │     │    image     │    │     ┌──────────┐           │
│    │ Replay   │─────┘     │              │    └────►│ Detector │           │
│    │  Node    │           └──────────────┘          │  Node    │           │
│    └──────────┘                                     └──────────┘           │
│                                                                              │
│    • Multiple publishers can write to same topic                            │
│    • Multiple subscribers can read from same topic                          │
│    • Decoupled: publishers don't know about subscribers                     │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Publisher Example:**

```python
#!/usr/bin/env python3
"""
publisher_node.py - Publishes messages to a topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """Node that publishes string messages."""

    def __init__(self):
        super().__init__('publisher_node')

        # Create publisher
        # Args: message type, topic name, queue size
        self.publisher = self.create_publisher(
            String,
            'chatter',
            10
        )

        # Create timer (calls callback every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        """Called by timer to publish message."""
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Subscriber Example:**

```python
#!/usr/bin/env python3
"""
subscriber_node.py - Subscribes to messages from a topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """Node that subscribes to string messages."""

    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg):
        """Called when message is received."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.4 Services (Request/Response)

Services provide **synchronous, one-to-one** communication:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Service Communication                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Client                   Service                    Server               │
│                                                                              │
│    ┌──────────┐         ┌─────────────┐          ┌──────────┐             │
│    │          │         │             │          │          │             │
│    │  Client  │────────►│  /add_ints  │─────────►│  Server  │             │
│    │   Node   │ Request │             │ Request  │   Node   │             │
│    │          │         │             │          │          │             │
│    │          │◄────────│             │◄─────────│          │             │
│    │          │ Response│             │ Response │          │             │
│    └──────────┘         └─────────────┘          └──────────┘             │
│                                                                              │
│    • One request, one response                                              │
│    • Client blocks until response (or timeout)                              │
│    • Only one server per service name                                       │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Service Server:**

```python
#!/usr/bin/env python3
"""
service_server.py - Provides a service
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionServer(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('addition_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.get_logger().info('Addition service ready')

    def add_callback(self, request, response):
        """Handle service request."""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AdditionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Service Client:**

```python
#!/usr/bin/env python3
"""
service_client.py - Calls a service
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionClient(Node):
    """Service client that requests addition."""

    def __init__(self):
        super().__init__('addition_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a: int, b: int):
        """Send addition request."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (async)
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = AdditionClient()

    # Send request
    future = node.send_request(3, 5)

    # Wait for result
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(f'Result: {result.sum}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.5 Actions (Goal/Feedback/Result)

Actions handle **long-running tasks** with feedback:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Action Communication                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Client                   Action                     Server               │
│                                                                              │
│    ┌──────────┐         ┌─────────────┐          ┌──────────┐             │
│    │          │──Goal──►│             │──Goal───►│          │             │
│    │  Action  │         │  /navigate  │          │  Action  │             │
│    │  Client  │◄─Accept─│             │◄─Accept──│  Server  │             │
│    │          │         │             │          │          │             │
│    │          │◄Feedback│             │◄Feedback─│          │             │
│    │          │◄Feedback│             │◄Feedback─│          │             │
│    │          │◄Feedback│             │◄Feedback─│          │             │
│    │          │         │             │          │          │             │
│    │          │◄─Result─│             │◄─Result──│          │             │
│    └──────────┘         └─────────────┘          └──────────┘             │
│                                                                              │
│    • Goal: What to achieve                                                  │
│    • Feedback: Progress updates (periodic)                                  │
│    • Result: Final outcome                                                  │
│    • Can be cancelled mid-execution                                         │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.6 Parameters

Parameters are **node-specific configuration values**:

```python
#!/usr/bin/env python3
"""
parameter_node.py - Using parameters
"""

import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    """Node demonstrating parameter usage."""

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('enabled', True)

        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value
        enabled = self.get_parameter('enabled').value

        self.get_logger().info(f'Robot: {robot_name}')
        self.get_logger().info(f'Max speed: {max_speed}')
        self.get_logger().info(f'Enabled: {enabled}')

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Called when parameters change."""
        for param in params:
            self.get_logger().info(
                f'Parameter {param.name} changed to {param.value}'
            )
        return rclpy.parameter.SetParametersResult(successful=True)
```

---

## 4. Quality of Service (QoS)

### 4.1 QoS Profiles

QoS settings control **reliability and performance**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        QoS Policy Options                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Reliability:                                                              │
│    ┌───────────────┐     ┌───────────────┐                                 │
│    │   RELIABLE    │     │  BEST_EFFORT  │                                 │
│    │               │     │               │                                 │
│    │ Guarantees    │     │ May lose      │                                 │
│    │ delivery      │     │ messages      │                                 │
│    │ (retransmit)  │     │ (faster)      │                                 │
│    └───────────────┘     └───────────────┘                                 │
│                                                                              │
│    Durability:                                                               │
│    ┌───────────────┐     ┌───────────────┐                                 │
│    │  TRANSIENT    │     │   VOLATILE    │                                 │
│    │    LOCAL      │     │               │                                 │
│    │               │     │               │                                 │
│    │ Late joiners  │     │ Only live     │                                 │
│    │ get history   │     │ messages      │                                 │
│    └───────────────┘     └───────────────┘                                 │
│                                                                              │
│    History:                                                                  │
│    ┌───────────────┐     ┌───────────────┐                                 │
│    │  KEEP_LAST    │     │   KEEP_ALL    │                                 │
│    │    (N)        │     │               │                                 │
│    │               │     │               │                                 │
│    │ Keep last N   │     │ Keep all      │                                 │
│    │ messages      │     │ messages      │                                 │
│    └───────────────┘     └───────────────┘                                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Common QoS Profiles

```python
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy
)

# Sensor data profile (fast, may lose some)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE
)

# Reliable command profile (guaranteed delivery)
command_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

# Using QoS
self.sensor_pub = self.create_publisher(
    SensorMsg, 'sensor_data', sensor_qos
)

self.cmd_sub = self.create_subscription(
    Command, 'commands', self.cmd_callback, command_qos
)
```

---

## 5. ROS 2 Command Line Tools

### 5.1 Essential Commands

```bash
# Node commands
ros2 node list                    # List running nodes
ros2 node info /node_name         # Info about a node

# Topic commands
ros2 topic list                   # List all topics
ros2 topic info /topic_name       # Info about a topic
ros2 topic echo /topic_name       # Print messages
ros2 topic pub /topic_name msg_type "data"  # Publish message
ros2 topic hz /topic_name         # Message frequency

# Service commands
ros2 service list                 # List all services
ros2 service type /service_name   # Get service type
ros2 service call /service_name type "args"  # Call service

# Parameter commands
ros2 param list /node_name        # List parameters
ros2 param get /node_name param   # Get parameter value
ros2 param set /node_name param value  # Set parameter

# Package commands
ros2 pkg list                     # List packages
ros2 pkg executables package_name # List executables

# Run commands
ros2 run package_name executable  # Run a node
ros2 launch package_name file.py  # Run launch file
```

### 5.2 Introspection Example

```bash
# Terminal 1: Run publisher
ros2 run demo_nodes_cpp talker

# Terminal 2: Inspect
ros2 node list
# Output: /talker

ros2 topic list
# Output: /chatter, /parameter_events, /rosout

ros2 topic info /chatter
# Output: Type: std_msgs/msg/String
#         Publisher count: 1
#         Subscription count: 0

ros2 topic echo /chatter
# Output: data: 'Hello World: 1'
#         data: 'Hello World: 2'
#         ...
```

---

## 6. Summary

### ROS 2 Communication Comparison

| Pattern | Use Case | Timing | Cardinality |
|---------|----------|--------|-------------|
| **Topics** | Streaming data | Async | N:M |
| **Services** | Quick queries | Sync | 1:1 |
| **Actions** | Long tasks | Async | 1:1 |
| **Parameters** | Configuration | On-demand | N/A |

### Key Takeaways

1. **No master** — ROS 2 uses peer-to-peer discovery via DDS
2. **Quality of Service** — Fine-grained control over communication
3. **Multi-platform** — Works on Linux, Windows, macOS
4. **Real-time capable** — With proper configuration
5. **Security built-in** — DDS Security for authentication/encryption

---

## Exercises

### Exercise 3.1: Install and Verify (⭐ Beginner)

**Objective**: Set up ROS 2 environment.

1. Install ROS 2 Humble following Section 2.2
2. Run the demo: `ros2 run demo_nodes_cpp talker`
3. In another terminal: `ros2 run demo_nodes_cpp listener`
4. Verify messages are received

### Exercise 3.2: Create Publisher/Subscriber (⭐⭐ Intermediate)

**Objective**: Create your own pub/sub pair.

Create a package with:
- Publisher that sends `geometry_msgs/msg/Twist` (velocity commands)
- Subscriber that logs received commands

<details>
<summary>Solution Structure</summary>

```
my_robot_control/
├── my_robot_control/
│   ├── __init__.py
│   ├── velocity_publisher.py
│   └── velocity_subscriber.py
├── package.xml
└── setup.py
```

</details>

### Exercise 3.3: Service Implementation (⭐⭐ Intermediate)

**Objective**: Create a service that calculates robot battery percentage.

Service should:
- Accept current voltage (float)
- Return percentage (0-100)
- Assume voltage range: 10.0V (empty) to 12.6V (full)

### Exercise 3.4: QoS Experimentation (⭐⭐⭐ Advanced)

**Objective**: Understand QoS impact.

1. Create publisher with BEST_EFFORT reliability
2. Create subscriber with RELIABLE reliability
3. What happens? Why?
4. Fix the mismatch

---

## Quiz: Week 3 Check

<details>
<summary>Q1: What replaced the ROS Master in ROS 2? (2 pts)</summary>

**Answer**: DDS (Data Distribution Service) provides peer-to-peer discovery without a central master.

</details>

<details>
<summary>Q2: When would you use a Service vs a Topic? (2 pts)</summary>

**Answer**:
- **Service**: For synchronous request-response, like getting a value or triggering an action
- **Topic**: For streaming data, like sensor readings or continuous state updates

</details>

<details>
<summary>Q3: What does BEST_EFFORT reliability mean? (2 pts)</summary>

**Answer**: Messages may be lost if the network is congested or the subscriber can't keep up. No retransmission occurs. This is faster but less reliable than RELIABLE.

</details>

<details>
<summary>Q4: What are the three parts of an Action? (2 pts)</summary>

**Answer**: Goal (what to achieve), Feedback (progress updates), Result (final outcome).

</details>

<details>
<summary>Q5: How do you list all running nodes? (2 pts)</summary>

**Answer**: `ros2 node list`

</details>

---

## Next Steps

In [Week 4: Nodes, Topics, and Services](/module-1-ros2/week-4-nodes-topics-services), we dive deeper into:
- Advanced publisher/subscriber patterns
- Custom message types
- Service definitions
- Multi-node architectures
