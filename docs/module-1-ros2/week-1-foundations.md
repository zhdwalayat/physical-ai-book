---
sidebar_position: 1
---

# Week 1: Foundations of Physical AI

## Learning Objectives

By the end of this week, you will be able to:

- Define Physical AI and explain how it differs from traditional AI
- Describe the concept of embodied intelligence
- Identify the key components of a humanoid robot system
- Understand the sensor systems used in humanoid robotics

## What is Physical AI?

Physical AI refers to artificial intelligence systems that **function in reality** and **comprehend physical laws**. Unlike traditional AI that processes data in isolation, Physical AI must:

- Interact with the physical world
- Understand gravity, friction, and dynamics
- React to unpredictable environments
- Coordinate perception with action

## From Digital AI to Physical AI

```
Traditional AI                    Physical AI
─────────────                    ───────────
Input → Process → Output         Sense → Think → Act
                                      ↑         ↓
                                      └─────────┘
                                    (Feedback Loop)
```

## The Humanoid Robotics Landscape

| Company | Robot | Key Features |
|---------|-------|--------------|
| Tesla | Optimus | General purpose, manufacturing |
| Figure | 01/02 | AI-native, LLM integration |
| Boston Dynamics | Atlas | Dynamic movement, parkour |
| Unitree | H1/G1 | Affordable, open SDK |
| Agility | Digit | Commercial deployment |

## Sensor Systems Overview

### Vision (The Eyes)
- RGB Cameras
- Depth Cameras (Intel RealSense)
- Stereo Vision

### Proprioception (Body Awareness)
- Joint Encoders
- Force/Torque Sensors
- IMUs (Inertial Measurement Units)

### Environment Sensing
- LiDAR
- Ultrasonic Sensors
- Contact Sensors

## Exercises

1. Research one humanoid robot and list its sensor suite
2. Explain why feedback loops are essential for Physical AI
3. Compare the challenges of indoor vs outdoor robot navigation

## Next Steps

In [Week 2](/module-1-ros2/week-2-embodied-intelligence), we'll dive deeper into embodied intelligence and how robots learn from physical interaction.
