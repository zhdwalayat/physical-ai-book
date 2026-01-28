---
sidebar_position: 3
---

# Week 3: ROS 2 Architecture

## Learning Objectives

By the end of this week, you will be able to:

- Explain the ROS 2 architecture and design philosophy
- Understand the DDS (Data Distribution Service) middleware
- Set up a ROS 2 development environment
- Create and run your first ROS 2 nodes

## What is ROS 2?

**Robot Operating System 2 (ROS 2)** is not an operating system—it's a middleware framework that provides:

- Communication infrastructure
- Hardware abstraction
- Package management
- Tools and libraries

## Why ROS 2?

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time support | No | Yes |
| Multi-robot | Limited | Native |
| Security | None | DDS Security |
| Platform | Linux only | Linux, Windows, macOS |
| Communication | Custom | DDS Standard |

## ROS 2 Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Application Layer                  │
│              (Your Robot Code / Nodes)               │
├─────────────────────────────────────────────────────┤
│                    ROS 2 Client Libraries            │
│                  (rclpy, rclcpp, rclrs)              │
├─────────────────────────────────────────────────────┤
│                       RCL (ROS Client Library)       │
├─────────────────────────────────────────────────────┤
│                         RMW                          │
│              (ROS Middleware Interface)              │
├─────────────────────────────────────────────────────┤
│                  DDS Implementation                  │
│            (Fast DDS, Cyclone DDS, etc.)             │
└─────────────────────────────────────────────────────┘
```

## Environment Setup

### Install ROS 2 Humble on Ubuntu 22.04

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
```

### Source the Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Your First Node

```python
# my_first_node.py
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello, ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. Install ROS 2 Humble on your workstation
2. Run `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_cpp listener`
3. Write a node that prints "Physical AI" every second

## Next Steps

In [Week 4](/module-1-ros2/week-4-nodes-topics-services), we explore ROS 2 communication: nodes, topics, and services.
