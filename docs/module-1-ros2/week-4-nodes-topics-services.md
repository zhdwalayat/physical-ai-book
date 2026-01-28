---
sidebar_position: 4
---

# Week 4: Nodes, Topics, and Services

## Learning Objectives

By the end of this week, you will be able to:

- Create ROS 2 publishers and subscribers
- Understand the topic-based communication model
- Implement services for request-response patterns
- Use actions for long-running tasks

## ROS 2 Communication Patterns

```
┌─────────────────────────────────────────────────────────────┐
│                    Communication Patterns                    │
├─────────────────┬─────────────────┬─────────────────────────┤
│     Topics      │    Services     │        Actions          │
│  (Pub/Sub)      │  (Req/Res)      │   (Goal/Feedback)       │
│                 │                 │                         │
│  Publisher ──►  │  Client ──►     │  Client ──►             │
│      │          │      │          │      │                  │
│      ▼          │      ▼          │      ▼                  │
│  Subscriber     │  Server         │  Server                 │
│                 │      │          │      │                  │
│  One-way        │      ▼          │      ▼                  │
│  Streaming      │  Response       │  Feedback + Result      │
└─────────────────┴─────────────────┴─────────────────────────┘
```

## Topics: Publisher/Subscriber

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is operational'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Services: Request/Response

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

## When to Use What?

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topics** | Continuous data streams | Sensor data, robot state |
| **Services** | One-time requests | Get parameter, trigger action |
| **Actions** | Long-running tasks | Navigate to goal, pick object |

## Exercises

1. Create a publisher that sends robot joint angles
2. Create a subscriber that logs sensor readings
3. Implement a service that returns robot battery level
4. Use `ros2 topic list` and `ros2 topic echo` to inspect messages

## Next Steps

In [Week 5](/module-1-ros2/week-5-python-packages), we learn to create proper ROS 2 packages and launch files.
