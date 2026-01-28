---
sidebar_position: 1
---

# Week 1: Foundations of Physical AI

## Learning Objectives

By the end of this week, you will be able to:

- **Define** Physical AI and distinguish it from traditional AI systems
- **Explain** the concept of embodied intelligence and its significance
- **Identify** key components and sensor systems of humanoid robots
- **Compare** leading humanoid robot platforms and their architectures
- **Describe** the sense-think-act cycle in robotics

---

## 1. What is Physical AI?

### 1.1 Definition

**Physical AI** refers to artificial intelligence systems that:

1. **Function in reality** — operate in the physical world, not just digital environments
2. **Comprehend physical laws** — understand gravity, friction, momentum, and dynamics
3. **Interact with objects** — manipulate, grasp, and transform physical matter
4. **Navigate environments** — move through 3D spaces with obstacles

> "Physical AI bridges the gap between the digital brain and the physical body."

### 1.2 Physical AI vs Traditional AI

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Traditional AI vs Physical AI                             │
├─────────────────────────────────┬───────────────────────────────────────────┤
│         Traditional AI          │              Physical AI                   │
├─────────────────────────────────┼───────────────────────────────────────────┤
│  Input → Process → Output       │  Sense → Think → Act → Feedback Loop      │
│                                 │                                           │
│  • Digital data only            │  • Physical world interaction             │
│  • No real-time constraints     │  • Real-time requirements                 │
│  • Perfect information          │  • Noisy, uncertain sensors               │
│  • Deterministic environment    │  • Stochastic environment                 │
│  • No physical consequences     │  • Safety-critical actions                │
└─────────────────────────────────┴───────────────────────────────────────────┘
```

### 1.3 Why Physical AI Matters Now

Several technological advances have converged to make Physical AI practical:

| Technology | Advancement | Impact |
|------------|-------------|--------|
| **GPUs** | RTX series, tensor cores | Real-time simulation and inference |
| **LLMs** | GPT-4, Claude | Natural language task understanding |
| **Simulation** | Isaac Sim, Gazebo | Safe training environments |
| **Sensors** | LiDAR, depth cameras | Accurate 3D perception |
| **Actuators** | Brushless motors, harmonic drives | Precise, powerful movement |

---

## 2. Embodied Intelligence

### 2.1 The Embodiment Hypothesis

Traditional AI assumes intelligence is purely computational — a disembodied algorithm. **Embodied cognition** challenges this:

> "Intelligence emerges from the interaction between an agent, its body, and its environment."

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Embodied Intelligence                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    ┌──────────────┐      ┌──────────────┐      ┌──────────────┐           │
│    │              │      │              │      │              │           │
│    │    BRAIN     │◄────►│     BODY     │◄────►│ ENVIRONMENT  │           │
│    │   (AI/ML)    │      │  (Sensors +  │      │  (Physical   │           │
│    │              │      │  Actuators)  │      │    World)    │           │
│    └──────────────┘      └──────────────┘      └──────────────┘           │
│                                                                              │
│    The body is not just a vessel — it's part of the cognitive system        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Key Principles

1. **Morphological Computation**: The body itself performs computation
   - Example: The shape of a hand naturally guides grasping

2. **Sensorimotor Coupling**: Perception and action are inseparable
   - Example: We look where we reach, we reach where we look

3. **Environmental Scaffolding**: The world serves as external memory
   - Example: A cup's handle affords grasping; we don't compute how to grasp it

### 2.3 Implications for Robotics

| Principle | Implication | Design Choice |
|-----------|-------------|---------------|
| Morphological Computation | Design bodies that simplify control | Compliant joints, shaped fingertips |
| Sensorimotor Coupling | Tight perception-action loops | Low-latency control (\<10ms) |
| Environmental Scaffolding | Use environment features | Leverage walls for navigation |

---

## 3. The Sense-Think-Act Cycle

### 3.1 Overview

Every robot operates on a continuous loop:

```
                         ┌─────────────────────┐
                         │       SENSE         │
                         │    (Perception)     │
                         │  Cameras, LiDAR,    │
                         │   IMU, Touch        │
                         └──────────┬──────────┘
                                    │
                                    ▼
┌─────────────────────┐   ┌─────────────────────┐   ┌─────────────────────┐
│     ENVIRONMENT     │   │       THINK         │   │        ACT          │
│                     │◄──│    (Cognition)      │──►│    (Execution)      │
│  Physical World     │   │  Planning, Learning │   │  Motors, Grippers   │
│  Objects, Obstacles │   │  Decision Making    │   │  Locomotion         │
└─────────────────────┘   └─────────────────────┘   └─────────────────────┘
         ▲                                                    │
         │                                                    │
         └────────────────────────────────────────────────────┘
                              Feedback Loop
```

### 3.2 Timing Requirements

Physical AI has strict timing constraints:

| Component | Typical Frequency | Latency Budget |
|-----------|-------------------|----------------|
| **Motor Control** | 1000 Hz | \<1 ms |
| **Balance Control** | 200-500 Hz | \<5 ms |
| **Perception** | 30-60 Hz | \<30 ms |
| **Planning** | 10-50 Hz | \<100 ms |
| **High-Level AI** | 1-10 Hz | \<1000 ms |

### 3.3 The Challenge of Real-Time

```python
# Pseudo-code: Real-time control loop
import time

CONTROL_FREQUENCY = 1000  # Hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQUENCY  # 1 ms

while robot.is_running():
    start_time = time.perf_counter()

    # 1. SENSE: Read sensors
    imu_data = robot.read_imu()
    joint_positions = robot.read_encoders()

    # 2. THINK: Compute control
    torques = controller.compute(imu_data, joint_positions)

    # 3. ACT: Send commands
    robot.send_torques(torques)

    # Ensure timing
    elapsed = time.perf_counter() - start_time
    if elapsed < CONTROL_PERIOD:
        time.sleep(CONTROL_PERIOD - elapsed)
    else:
        print(f"WARNING: Control loop overrun: {elapsed*1000:.2f} ms")
```

---

## 4. Sensor Systems for Humanoid Robots

### 4.1 Sensor Categories

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Humanoid Robot Sensor Systems                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │
│  │  EXTEROCEPTION  │  │  PROPRIOCEPTION │  │   INTERACTION   │            │
│  │  (External)     │  │  (Internal)     │  │   (Contact)     │            │
│  ├─────────────────┤  ├─────────────────┤  ├─────────────────┤            │
│  │ • Cameras (RGB) │  │ • Joint Encoders│  │ • Force/Torque  │            │
│  │ • Depth Sensors │  │ • IMU           │  │ • Tactile Arrays│            │
│  │ • LiDAR         │  │ • Motor Current │  │ • Pressure Mats │            │
│  │ • Microphones   │  │ • Temperature   │  │ • Whiskers      │            │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘            │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Vision Sensors

#### RGB Cameras

```yaml
# Typical RGB Camera Specifications
Type: Global Shutter CMOS
Resolution: 1920 x 1080 (1080p)
Frame Rate: 30-60 FPS
Field of View: 70-90° horizontal
Interface: USB 3.0 / MIPI CSI
Use Cases:
  - Object recognition
  - Face detection
  - Scene understanding
```

#### Depth Cameras (Intel RealSense D435i)

```yaml
# Intel RealSense D435i Specifications
Type: Active Stereo + IMU
Depth Resolution: 1280 x 720
Depth Range: 0.3m - 10m
Frame Rate: Up to 90 FPS
IMU: 6-axis (Accel + Gyro)
Interface: USB 3.0
Use Cases:
  - SLAM
  - 3D reconstruction
  - Obstacle avoidance
```

#### LiDAR

```yaml
# Typical LiDAR Specifications
Type: Rotating / Solid-State
Range: 0.1m - 100m
Accuracy: ±2 cm
Points per Second: 300,000 - 1,000,000
Channels: 16-128
Use Cases:
  - Long-range navigation
  - High-precision mapping
  - Outdoor environments
```

### 4.3 Proprioceptive Sensors

#### IMU (Inertial Measurement Unit)

An IMU measures:
- **Accelerometer**: Linear acceleration (3 axes)
- **Gyroscope**: Angular velocity (3 axes)
- **Magnetometer** (optional): Orientation relative to magnetic north

```
                    IMU Axes

                       Z (Yaw)
                        │
                        │    Y (Pitch)
                        │   /
                        │  /
                        │ /
                        │/
         ──────────────────────────── X (Roll)
```

#### Joint Encoders

Measure joint angles with high precision:

| Type | Resolution | Accuracy | Cost |
|------|------------|----------|------|
| Incremental | 1000-10000 PPR | ±0.1° | Low |
| Absolute | 12-19 bits | ±0.01° | Medium |
| Magnetic | 12-14 bits | ±0.1° | Low |

### 4.4 Force/Torque Sensors

Critical for manipulation and balance:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      6-Axis Force/Torque Sensor                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                  Forces                        Torques                       │
│               ┌─────────┐                   ┌─────────┐                     │
│               │         │                   │         │                     │
│          Fx ──┤   ●─────┼── Fy         Tx ──┤   ↻─────┼── Ty               │
│               │    │    │                   │    │    │                     │
│               └────┼────┘                   └────┼────┘                     │
│                    │                             │                           │
│                   Fz                            Tz                           │
│                                                                              │
│   Typical Specifications:                                                    │
│   • Force Range: ±500N                                                       │
│   • Torque Range: ±50Nm                                                      │
│   • Resolution: 0.01N / 0.001Nm                                              │
│   • Sample Rate: 1000 Hz                                                     │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. The Humanoid Robotics Landscape

### 5.1 Leading Platforms

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Humanoid Robot Comparison (2024-2025)                     │
├──────────────┬─────────────┬─────────────┬─────────────┬───────────────────┤
│    Robot     │   Company   │    Price    │    Focus    │    Key Feature    │
├──────────────┼─────────────┼─────────────┼─────────────┼───────────────────┤
│ Optimus Gen2 │ Tesla       │ ~$20k*      │ Manufacturing│ AI Day demos      │
│ Figure 01/02 │ Figure AI   │ N/A         │ General     │ OpenAI integration│
│ Atlas        │ Boston Dyn. │ N/A         │ Research    │ Dynamic movement  │
│ H1           │ Unitree     │ ~$90k       │ Research    │ Open SDK          │
│ G1           │ Unitree     │ ~$16k       │ Education   │ Affordable        │
│ Digit        │ Agility     │ N/A         │ Logistics   │ Commercial deploy │
│ NEO          │ 1X Tech     │ N/A         │ Consumer    │ Home assistant    │
└──────────────┴─────────────┴─────────────┴─────────────┴───────────────────┘
* Projected consumer price
```

### 5.2 Case Study: Tesla Optimus

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Tesla Optimus Gen 2                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Height: 1.73m (5'8")                    ┌─────────┐                       │
│   Weight: 56.6 kg (125 lbs)               │  Head   │ Cameras, sensors      │
│   Payload: 9 kg (20 lbs)                  └────┬────┘                       │
│   Speed: 8 km/h (5 mph)                        │                            │
│                                           ┌────┴────┐                       │
│   Actuators:                        ┌─────┤  Torso  ├─────┐ Battery, compute│
│   • 28 structural                   │     └────┬────┘     │                 │
│   • 11 per hand (22 total)     ┌────┴────┐     │    ┌────┴────┐            │
│                                │   Arm   │     │    │   Arm   │ 11 DoF each │
│   Sensors:                     └────┬────┘     │    └────┬────┘            │
│   • Cameras (Tesla FSD)             │          │         │                  │
│   • Force/Torque at hands      ┌────┴────┐     │    ┌────┴────┐            │
│   • IMU                        │  Hand   │     │    │  Hand   │ 11 DoF each │
│   • Joint encoders             └─────────┘     │    └─────────┘            │
│                                           ┌────┴────┐                       │
│   AI Stack:                         ┌─────┤  Pelvis ├─────┐                 │
│   • End-to-end neural nets          │     └─────────┘     │                 │
│   • Tesla FSD chips            ┌────┴────┐          ┌────┴────┐            │
│   • Dojo training              │   Leg   │          │   Leg   │ 6 DoF each  │
│                                └────┬────┘          └────┬────┘            │
│                                     │                    │                  │
│                                ┌────┴────┐          ┌────┴────┐            │
│                                │  Foot   │          │  Foot   │            │
│                                └─────────┘          └─────────┘            │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.3 Case Study: Unitree G1

The **Unitree G1** is particularly relevant for education:

| Feature | Specification |
|---------|---------------|
| **Price** | ~$16,000 USD |
| **Height** | 1.27m |
| **Weight** | 35 kg |
| **DoF** | 23-43 (configurable) |
| **Runtime** | ~2 hours |
| **SDK** | ROS 2 compatible |
| **Simulation** | Isaac Sim, MuJoCo |

**Why G1 for Learning?**
1. Affordable for educational institutions
2. Open SDK enables custom control
3. ROS 2 native support
4. Active community and documentation

---

## 6. Summary

### Key Takeaways

1. **Physical AI** = AI that operates in and understands the physical world
2. **Embodied Intelligence** = Intelligence emerges from brain-body-environment interaction
3. **Sense-Think-Act** = The fundamental loop of all robotic systems
4. **Sensors** = Exteroception + Proprioception + Interaction
5. **The field is accelerating** = Multiple viable humanoid platforms now exist

### Concept Map

```
                                Physical AI
                                    │
                    ┌───────────────┼───────────────┐
                    │               │               │
              Embodied         Sense-Think-Act    Hardware
             Intelligence          Cycle          Platforms
                    │               │               │
            ┌───────┴───────┐       │       ┌──────┴──────┐
            │               │       │       │             │
        Morphology    Environment   │    Sensors     Actuators
                                    │
                            ┌───────┼───────┐
                            │       │       │
                         Sense   Think    Act
```

---

## Exercises

### Exercise 1.1: Embodied vs Disembodied (⭐ Beginner)

**Objective**: Distinguish between embodied and disembodied AI systems.

**Instructions**: Classify each system as primarily embodied or disembodied:

| System | Embodied or Disembodied? | Why? |
|--------|--------------------------|------|
| ChatGPT | | |
| Roomba vacuum | | |
| Google Search | | |
| Self-driving car | | |
| Chess AI (Stockfish) | | |
| Boston Dynamics Spot | | |

<details>
<summary>Solution</summary>

| System | Classification | Reasoning |
|--------|----------------|-----------|
| ChatGPT | Disembodied | Processes text, no physical interaction |
| Roomba | Embodied | Navigates physical space, senses obstacles |
| Google Search | Disembodied | Information retrieval, no physical presence |
| Self-driving car | Embodied | Senses environment, controls physical vehicle |
| Chess AI | Disembodied | Symbolic reasoning, no physical world model |
| Spot | Embodied | Locomotion, balance, physical manipulation |

</details>

### Exercise 1.2: Sensor Selection (⭐⭐ Intermediate)

**Objective**: Choose appropriate sensors for specific tasks.

**Scenario**: You're designing a humanoid robot for a warehouse. Select sensors for each task:

1. **Navigating between shelves**: Which sensors? Why?
2. **Picking items from shelves**: Which sensors? Why?
3. **Detecting human workers**: Which sensors? Why?
4. **Maintaining balance while carrying heavy loads**: Which sensors? Why?

<details>
<summary>Hints</summary>

- Consider range, accuracy, and cost trade-offs
- Some tasks may need multiple sensor types
- Think about failure modes and redundancy

</details>

### Exercise 1.3: Control Loop Analysis (⭐⭐ Intermediate)

**Objective**: Calculate timing budgets for a control system.

**Given**:
- Motor control requires 1000 Hz updates
- Your perception system takes 25 ms per frame
- Planning takes 80 ms per update
- You have a single-threaded system

**Questions**:
1. Can this system work? Why or why not?
2. How would you restructure it to work?
3. Draw a timing diagram showing the solution.

<details>
<summary>Solution</summary>

1. **No** - Motor control needs 1 ms cycle time, but perception alone takes 25 ms.

2. **Solution**: Use parallel threads with different frequencies:
   - Motor control thread: 1000 Hz (1 ms)
   - Perception thread: 40 Hz (25 ms)
   - Planning thread: 10 Hz (80 ms)

3. **Timing Diagram**:
```
Motor:   |1ms|1ms|1ms|1ms|1ms|... (continuous at 1000 Hz)
Percept: |----25ms----|----25ms----|...
Planning:|--------80ms--------|--------80ms--------|...
```

</details>

### Exercise 1.4: Research Project (⭐⭐⭐ Advanced)

**Objective**: Deep dive into a humanoid robot platform.

**Instructions**:
1. Choose one humanoid robot from Section 5.1
2. Research its:
   - Sensor suite (list all sensors)
   - Actuator specifications
   - Software stack (what OS, middleware, AI frameworks)
   - Publicly demonstrated capabilities
3. Write a 1-page technical summary
4. Present findings to the class (or create a diagram)

**Deliverable**: 1-page PDF with diagrams

---

## Quiz: Week 1 Check

**Test your understanding** (5 questions, 10 points total)

<details>
<summary>Question 1 (2 pts): What distinguishes Physical AI from traditional AI?</summary>

**Answer**: Physical AI operates in and understands the physical world, requiring real-time interaction with sensors and actuators, while traditional AI processes digital data without physical embodiment.

</details>

<details>
<summary>Question 2 (2 pts): Name the three components of the sense-think-act cycle.</summary>

**Answer**:
1. **Sense** (Perception) - Gathering data from sensors
2. **Think** (Cognition) - Processing, planning, decision making
3. **Act** (Execution) - Sending commands to actuators

</details>

<details>
<summary>Question 3 (2 pts): What is an IMU and what does it measure?</summary>

**Answer**: An IMU (Inertial Measurement Unit) measures:
- Linear acceleration (accelerometer, 3 axes)
- Angular velocity (gyroscope, 3 axes)
- Optionally: magnetic heading (magnetometer, 3 axes)

</details>

<details>
<summary>Question 4 (2 pts): Why is timing critical in robotics control loops?</summary>

**Answer**: Physical systems operate in real-time and require consistent update rates for stability. Missing deadlines can cause:
- Loss of balance (for bipedal robots)
- Jerky motion
- Safety hazards
- Control instability

</details>

<details>
<summary>Question 5 (2 pts): Which humanoid robot mentioned is most suitable for educational use and why?</summary>

**Answer**: **Unitree G1** because:
- Affordable (~$16k)
- ROS 2 compatible SDK
- Open for custom development
- Active community support
- Simulation support (Isaac Sim, MuJoCo)

</details>

---

## Further Reading

### Essential
- Pfeifer, R., & Bongard, J. (2006). *How the Body Shapes the Way We Think*
- Brooks, R. A. (1991). "Intelligence Without Representation"

### Recommended
- NVIDIA Isaac Sim Documentation: [developer.nvidia.com/isaac-sim](https://developer.nvidia.com/isaac-sim)
- ROS 2 Documentation: [docs.ros.org](https://docs.ros.org)

### Humanoid Robot Company Sites
- Tesla Optimus: [tesla.com/optimus](https://tesla.com/optimus)
- Figure AI: [figure.ai](https://figure.ai)
- Unitree: [unitree.com](https://unitree.com)
- Agility Robotics: [agilityrobotics.com](https://agilityrobotics.com)

---

## Next Steps

In [Week 2: Embodied Intelligence](/module-1-ros2/week-2-embodied-intelligence), we dive deeper into:
- The history of embodied cognition
- Morphological computation in detail
- Designing robot bodies for intelligence
- The sense-think-act cycle in practice
