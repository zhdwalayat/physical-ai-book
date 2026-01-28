---
sidebar_position: 2
---

# Week 2: Embodied Intelligence

## Learning Objectives

By the end of this week, you will be able to:

- **Trace** the historical development of embodied cognition theory
- **Explain** morphological computation and its role in simplifying control
- **Analyze** how body design affects intelligence and behavior
- **Apply** embodied principles to robot design decisions
- **Evaluate** the trade-offs between computation and mechanical design

---

## 1. The History of Embodied Cognition

### 1.1 The Disembodied AI Era (1950s-1980s)

Early AI assumed intelligence was **purely symbolic computation**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     Classical AI (GOFAI) Paradigm                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Symbols ──► Rules ──► Logic ──► Reasoning ──► Intelligence               │
│                                                                              │
│    Key Assumptions:                                                          │
│    • Intelligence = Symbol manipulation                                      │
│    • Body is just input/output device                                        │
│    • Environment can be fully modeled                                        │
│    • Cognition is computation                                                │
│                                                                              │
│    Famous Systems:                                                           │
│    • SHRDLU (1970) - Block world understanding                               │
│    • MYCIN (1976) - Medical diagnosis                                        │
│    • Deep Blue (1997) - Chess                                                │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 The Embodied Turn (1980s-1990s)

Researchers began questioning the disembodied approach:

| Year | Researcher | Contribution |
|------|------------|--------------|
| 1984 | Francisco Varela | Autopoiesis and embodied mind |
| 1986 | Rodney Brooks | Subsumption architecture |
| 1991 | Brooks | "Intelligence Without Representation" |
| 1995 | Rolf Pfeifer | Embodied AI principles |
| 1999 | Andy Clark | "Being There" - extended mind |

### 1.3 Brooks' Revolution

**Rodney Brooks** challenged classical AI with radical claims:

> "The world is its own best model."

> "Intelligence is determined by the dynamics of interaction with the world."

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Classical vs Brooks' Approach                             │
├────────────────────────────────────┬────────────────────────────────────────┤
│          Classical AI              │         Brooks' Approach               │
├────────────────────────────────────┼────────────────────────────────────────┤
│                                    │                                        │
│    ┌──────────┐                    │         ┌──────────┐                  │
│    │  World   │                    │         │  World   │◄──┐              │
│    │  Model   │                    │         └──────────┘   │              │
│    └────┬─────┘                    │               ▲        │              │
│         │                          │               │        │              │
│         ▼                          │         ┌─────┴──────┐ │              │
│    ┌──────────┐                    │         │  Behavior  │ │              │
│    │ Planner  │                    │         │   Layer 3  │ │              │
│    └────┬─────┘                    │         └─────┬──────┘ │              │
│         │                          │               │        │              │
│         ▼                          │         ┌─────┴──────┐ │              │
│    ┌──────────┐                    │         │  Behavior  │ │              │
│    │ Executor │                    │         │   Layer 2  │ │              │
│    └────┬─────┘                    │         └─────┬──────┘ │              │
│         │                          │               │        │              │
│         ▼                          │         ┌─────┴──────┐ │              │
│    ┌──────────┐                    │         │  Behavior  │─┘              │
│    │  Robot   │                    │         │   Layer 1  │                │
│    └──────────┘                    │         └────────────┘                │
│                                    │                                        │
│    Sense → Model → Plan → Act     │    Direct Sense → Act coupling         │
│                                    │    (Subsumption Architecture)          │
└────────────────────────────────────┴────────────────────────────────────────┘
```

---

## 2. Morphological Computation

### 2.1 What is Morphological Computation?

**Morphological computation** is when the body's physical structure performs computation that would otherwise require the brain.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Morphological Computation                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Traditional:          Body performs computation:                          │
│                                                                              │
│    Brain ────────────►   Brain ──────┬──────► Simpler                       │
│      │                     │         │        brain needed                   │
│      │ Complex             │         │                                       │
│      │ computation         │         ▼                                       │
│      │                     │       Body                                      │
│      ▼                     │    (does some                                   │
│    Simple body             │    computation)                                 │
│                            │                                                 │
│                            ▼                                                 │
│                         Result                                               │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Examples in Nature

| Animal | Morphological Feature | Computation Offloaded |
|--------|----------------------|----------------------|
| **Tuna** | Crescent tail shape | Efficient propulsion without active control |
| **Fly** | Wing hinge elasticity | Stable flight from passive dynamics |
| **Human hand** | Fingertip shape | Automatic grasp stability |
| **Kangaroo** | Tendon elasticity | Energy-efficient hopping |
| **Gecko** | Toe pad structure | Adhesion without active control |

### 2.3 Examples in Robotics

#### Passive Dynamic Walkers

A **passive dynamic walker** walks down a slope with no motors or control:

```
                    Passive Dynamic Walker

                         ○ ← Head (mass)
                        /│\
                       / │ \
                      /  │  \
                     /   │   \
                    /    │    \
                   ○     │     ○ ← Hip joints
                  /      │      \
                 /       │       \
                /        │        \
               /         │         \
              /          │          \
             ○           │           ○ ← Knee joints
            /            │            \
           /             │             \
          /              │              \
         ●───────────────┴───────────────● ← Feet

    No motors! Walks purely from gravity + body dynamics
```

**Key insight**: The "computation" of walking emerges from body geometry.

#### Compliant Grippers

```python
# Traditional gripper: Complex control needed
def grasp_object_traditional(gripper, object_pose):
    """Requires precise force control."""
    # 1. Plan approach trajectory
    trajectory = plan_approach(gripper, object_pose)

    # 2. Execute approach
    for waypoint in trajectory:
        gripper.move_to(waypoint)

    # 3. Close with force control
    while not gripper.contact_detected():
        gripper.close(speed=0.01)

    # 4. Apply precise grasp force
    gripper.set_force(GRASP_FORCE)

    # 5. Continuously monitor and adjust
    while grasping:
        if gripper.force > MAX_FORCE:
            gripper.reduce_force()
        if gripper.slipping():
            gripper.increase_force()


# Compliant gripper: Body does the work
def grasp_object_compliant(gripper, object_pose):
    """Compliance handles force regulation."""
    # 1. Move to object
    gripper.move_to(object_pose)

    # 2. Close gripper
    gripper.close()

    # Done! Compliance handles the rest:
    # - Soft fingers conform to object shape
    # - Passive compliance regulates force
    # - No force sensor needed
```

### 2.4 Design Principles

| Principle | Description | Example |
|-----------|-------------|---------|
| **Exploit passive dynamics** | Use physics, don't fight it | Pendulum-like legs |
| **Embed springs** | Store and release energy | Series elastic actuators |
| **Use compliance** | Let body deform to environment | Soft grippers |
| **Shape matters** | Geometry determines behavior | Curved fingertips |
| **Material properties** | Stiffness, damping, friction | Rubber soles |

---

## 3. The Sensorimotor Loop

### 3.1 Tight Coupling of Perception and Action

In embodied systems, perception and action are **inseparable**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Sensorimotor Loop                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                          ┌─────────────┐                                    │
│                          │   Action    │                                    │
│                          │  (Movement) │                                    │
│                          └──────┬──────┘                                    │
│                                 │                                           │
│                    ┌────────────┼────────────┐                              │
│                    │            │            │                              │
│                    ▼            ▼            ▼                              │
│              ┌──────────┐ ┌──────────┐ ┌──────────┐                        │
│              │  Body    │ │  World   │ │  Other   │                        │
│              │  Change  │ │  Change  │ │  Agents  │                        │
│              └────┬─────┘ └────┬─────┘ └────┬─────┘                        │
│                   │            │            │                              │
│                   └────────────┼────────────┘                              │
│                                │                                           │
│                                ▼                                           │
│                          ┌──────────┐                                      │
│                          │  Sensor  │                                      │
│                          │  Input   │                                      │
│                          └────┬─────┘                                      │
│                               │                                            │
│                               ▼                                            │
│                          ┌──────────┐                                      │
│                          │Perception│────────► Back to Action              │
│                          └──────────┘                                      │
│                                                                             │
│    Key: Actions change what sensors perceive.                               │
│         Perception guides what actions to take.                             │
│         They form one integrated system.                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Active Perception

Robots should **move to see better**, not just passively observe:

```python
class ActivePerceptionNode(Node):
    """
    Active perception: Move sensors to improve perception.
    """

    def __init__(self):
        super().__init__('active_perception')

        # Camera on pan-tilt unit
        self.pan_tilt_pub = self.create_publisher(
            JointState, '/head/joint_commands', 10
        )

        # Object detection subscription
        self.detection_sub = self.create_subscription(
            Detection, '/detections', self.detection_callback, 10
        )

    def detection_callback(self, msg):
        """Move head to track and improve detection."""
        for detection in msg.detections:
            # If detection confidence is low, look more directly
            if detection.confidence < 0.7:
                # Calculate direction to object
                angle_to_object = self.compute_angle(detection.bbox)

                # Move head toward object
                self.look_at(angle_to_object)

            # If object is at edge of view, center it
            if self.is_at_edge(detection.bbox):
                self.center_on(detection.bbox)

    def look_at(self, pan_angle, tilt_angle=0.0):
        """Command head to look at angle."""
        cmd = JointState()
        cmd.name = ['head_pan', 'head_tilt']
        cmd.position = [pan_angle, tilt_angle]
        self.pan_tilt_pub.publish(cmd)
```

### 3.3 Sensorimotor Contingencies

**Sensorimotor contingencies** are the rules relating actions to sensory changes:

| Action | Sensory Consequence | What Robot Learns |
|--------|--------------------|--------------------|
| Move forward | Objects grow larger | Depth from motion |
| Turn head | Scene shifts in opposite direction | Self-motion detection |
| Grasp object | Tactile sensors activate | Object presence |
| Push object | Object moves, force felt | Object mass estimation |

---

## 4. Environmental Scaffolding

### 4.1 Using the Environment as Memory

Instead of building complex internal models, use the environment:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Environmental Scaffolding                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Internal Model Approach:          Scaffolding Approach:                   │
│                                                                              │
│    ┌────────────────────┐            ┌────────────────────┐                │
│    │    Complete 3D     │            │   Simple reactive  │                │
│    │    world model     │            │      behaviors     │                │
│    │    in memory       │            │                    │                │
│    │                    │            │  + Use environment │                │
│    │    • All objects   │            │    as reference    │                │
│    │    • All positions │            │                    │                │
│    │    • All relations │            │  • Follow walls    │                │
│    │                    │            │  • Track landmarks │                │
│    └────────────────────┘            │  • React to stimuli│                │
│                                      └────────────────────┘                │
│    High memory + computation         Low memory + computation               │
│    Brittle to changes                Robust to changes                      │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Affordances

**Affordances** (James Gibson, 1979) are action possibilities offered by the environment:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Affordances                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Object          Affordance              Robot Action                      │
│    ──────          ──────────              ────────────                      │
│                                                                              │
│    ┌─────┐                                                                  │
│    │     │         "Graspable"             gripper.close()                  │
│    │ Cup │──────►  "Containable"     ───►  pour_into()                      │
│    │  ⌒  │         "Liftable"              arm.lift()                       │
│    └─────┘                                                                  │
│                                                                              │
│    ═══════         "Supportable"           place_on()                       │
│    Table    ────►  "Slidable"        ───►  push_along()                     │
│                    "Under-passable"        navigate_under()                 │
│                                                                              │
│    ┌──┐                                                                     │
│    │  │            "Openable"              hand.rotate()                    │
│    │▓▓│──────────► "Passable"        ───►  navigate_through()              │
│    │  │  Door      "Closable"              hand.push()                      │
│    └──┘                                                                     │
│                                                                              │
│    Key: Affordances depend on BOTH the environment AND the robot's body     │
│         A door affords "openable" only if the robot has hands               │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.3 Implementing Affordance Detection

```python
from dataclasses import dataclass
from enum import Enum
from typing import List


class AffordanceType(Enum):
    GRASPABLE = "graspable"
    SUPPORTABLE = "supportable"
    CONTAINABLE = "containable"
    OPENABLE = "openable"
    PUSHABLE = "pushable"


@dataclass
class Affordance:
    """Detected affordance for an object."""
    object_id: str
    affordance_type: AffordanceType
    confidence: float
    action_params: dict


class AffordanceDetector:
    """
    Detect affordances based on object properties and robot capabilities.
    """

    def __init__(self, robot_capabilities: dict):
        self.capabilities = robot_capabilities

    def detect_affordances(self, objects: List[dict]) -> List[Affordance]:
        """
        Detect affordances for perceived objects.

        Args:
            objects: List of detected objects with properties

        Returns:
            List of detected affordances
        """
        affordances = []

        for obj in objects:
            # Check graspable (requires gripper + appropriate size)
            if self._is_graspable(obj):
                affordances.append(Affordance(
                    object_id=obj['id'],
                    affordance_type=AffordanceType.GRASPABLE,
                    confidence=self._grasp_confidence(obj),
                    action_params={'grasp_pose': self._compute_grasp(obj)}
                ))

            # Check supportable (flat top surface)
            if self._is_supportable(obj):
                affordances.append(Affordance(
                    object_id=obj['id'],
                    affordance_type=AffordanceType.SUPPORTABLE,
                    confidence=0.9,
                    action_params={'surface_height': obj['bbox']['max_z']}
                ))

            # Check pushable (moveable, not too heavy)
            if self._is_pushable(obj):
                affordances.append(Affordance(
                    object_id=obj['id'],
                    affordance_type=AffordanceType.PUSHABLE,
                    confidence=0.8,
                    action_params={'push_direction': self._best_push_dir(obj)}
                ))

        return affordances

    def _is_graspable(self, obj: dict) -> bool:
        """Check if object affords grasping."""
        # Must have gripper
        if not self.capabilities.get('has_gripper'):
            return False

        # Object must fit in gripper
        max_gripper_width = self.capabilities.get('gripper_max_width', 0.1)
        obj_min_dim = min(obj['dimensions'].values())

        return obj_min_dim < max_gripper_width

    def _grasp_confidence(self, obj: dict) -> float:
        """Compute confidence for grasp affordance."""
        # Higher confidence for:
        # - Objects with clear grasp points (handles)
        # - Objects of familiar categories
        # - Objects at reachable positions

        confidence = 0.5

        if obj.get('has_handle'):
            confidence += 0.3

        if obj.get('category') in ['cup', 'bottle', 'tool']:
            confidence += 0.2

        return min(confidence, 1.0)
```

---

## 5. Designing Embodied Robots

### 5.1 Body-Brain Co-Design

Optimal robot design considers body and brain together:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      Body-Brain Co-Design                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Traditional (Sequential):                                                 │
│                                                                              │
│    Design Body ──► Design Controller ──► Hope it works                      │
│         │                  │                   │                             │
│         │                  │                   ▼                             │
│         │                  │              Often fails                        │
│         │                  │              (mismatch)                         │
│                                                                              │
│    ─────────────────────────────────────────────────────────────────────    │
│                                                                              │
│    Co-Design (Simultaneous):                                                 │
│                                                                              │
│              ┌─────────────────────────┐                                    │
│              │    Task Requirements    │                                    │
│              └───────────┬─────────────┘                                    │
│                          │                                                   │
│              ┌───────────┼───────────┐                                      │
│              │           │           │                                      │
│              ▼           ▼           ▼                                      │
│         ┌────────┐  ┌────────┐  ┌────────┐                                 │
│         │  Body  │◄─┤  Co-   │─►│ Brain  │                                 │
│         │ Design │  │Optimize│  │ Design │                                 │
│         └───┬────┘  └────────┘  └───┬────┘                                 │
│             │                       │                                       │
│             └───────────┬───────────┘                                       │
│                         │                                                    │
│                         ▼                                                    │
│              ┌─────────────────────────┐                                    │
│              │   Optimal Robot System  │                                    │
│              └─────────────────────────┘                                    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 Design Trade-offs

| Design Choice | More Body | More Brain |
|--------------|-----------|------------|
| **Walking** | Passive dynamics, tuned leg mass | Active balance control |
| **Grasping** | Compliant fingers, shaped tips | Force control, planning |
| **Navigation** | Bumpers, whiskers | SLAM, path planning |
| **Reaching** | Many DoF, redundancy | Inverse kinematics solver |

### 5.3 Case Study: Series Elastic Actuators

**Series Elastic Actuators (SEAs)** embed computation in mechanical compliance:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Series Elastic Actuator (SEA)                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Traditional Rigid Actuator:                                               │
│                                                                              │
│    Motor ─────────────────────────────────────► Load                        │
│           (Direct coupling, high stiffness)                                 │
│                                                                              │
│    Problems: Impacts shock motor, requires precise force control             │
│                                                                              │
│    ─────────────────────────────────────────────────────────────────────    │
│                                                                              │
│    Series Elastic Actuator:                                                  │
│                                                                              │
│    Motor ──────┬─── Spring ───┬──────────────► Load                        │
│                │    ╱╲╱╲╱╲    │                                             │
│                │              │                                             │
│                │   (Measures  │                                             │
│                │   deflection)│                                             │
│                │              │                                             │
│                └──Encoder ────┘                                             │
│                                                                              │
│    Benefits:                                                                 │
│    • Spring absorbs impacts (safety)                                         │
│    • Spring deflection = force (sensing)                                     │
│    • Energy storage for explosive movements                                  │
│    • Passive compliance (no active control needed)                           │
│                                                                              │
│    Force = k × (θ_motor - θ_load)                                           │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Summary

### Key Takeaways

1. **Embodied cognition** challenges the view that intelligence is pure computation
2. **Morphological computation** offloads control to body structure
3. **Sensorimotor coupling** unifies perception and action
4. **Environmental scaffolding** uses the world instead of internal models
5. **Body-brain co-design** optimizes both together

### Comparison Table

| Approach | Computation | Body | Environment |
|----------|-------------|------|-------------|
| Classical AI | Maximum | Minimal | Modeled |
| Embodied AI | Shared | Essential | Leveraged |
| Extreme Embodied | Minimal | Maximum | External memory |

---

## Exercises

### Exercise 2.1: Passive Dynamics Analysis (⭐ Beginner)

**Objective**: Understand passive dynamic walking.

Watch this video: [Passive Dynamic Walker](https://www.youtube.com/watch?v=WOPED7I5Lac)

**Questions**:
1. Why doesn't the walker need motors?
2. What would happen if the legs were heavier at the feet?
3. Why does it only work on slopes?

<details>
<summary>Solution</summary>

1. Gravity provides energy; the leg geometry converts it to forward motion. The mass distribution creates a natural pendulum swing.

2. Heavier feet would slow the swing (longer pendulum period), potentially causing the walker to fall forward before the leg swings through.

3. The slope provides continuous energy input. On flat ground, friction would quickly stop it.

</details>

### Exercise 2.2: Morphological Design (⭐⭐ Intermediate)

**Objective**: Apply morphological computation principles.

**Task**: Design a gripper for picking up eggs without breaking them.

**Constraints**:
- No force sensors allowed
- Only on/off motor control (no variable force)
- Must handle eggs of varying sizes

**Deliverable**: Sketch your design and explain how the body handles force regulation.

<details>
<summary>Hints</summary>

- Think about compliant materials
- Consider how the gripper shape could conform to curved surfaces
- Look up "soft robotics grippers" for inspiration

</details>

### Exercise 2.3: Affordance Detection (⭐⭐ Intermediate)

**Objective**: Implement basic affordance detection.

**Task**: Complete this function to detect if an object is "sittable":

```python
def is_sittable(object_properties: dict, robot_properties: dict) -> bool:
    """
    Determine if an object affords sitting for this robot.

    Args:
        object_properties: {
            'height': float,  # Height of top surface (meters)
            'width': float,   # Width of surface (meters)
            'depth': float,   # Depth of surface (meters)
            'is_stable': bool,  # Won't tip over
            'max_load': float,  # Maximum weight it can support (kg)
        }
        robot_properties: {
            'hip_height': float,  # Height of robot's hip joint (meters)
            'body_width': float,  # Width of robot body (meters)
            'weight': float,  # Robot weight (kg)
        }

    Returns:
        True if the object affords sitting
    """
    # YOUR CODE HERE
    pass
```

<details>
<summary>Solution</summary>

```python
def is_sittable(object_properties: dict, robot_properties: dict) -> bool:
    obj = object_properties
    robot = robot_properties

    # Check height is appropriate (within 20% of hip height)
    height_ok = 0.8 * robot['hip_height'] <= obj['height'] <= 1.2 * robot['hip_height']

    # Check surface is wide enough
    width_ok = obj['width'] >= robot['body_width'] * 0.8

    # Check surface is deep enough (at least 30cm)
    depth_ok = obj['depth'] >= 0.3

    # Check stability
    stable = obj['is_stable']

    # Check weight capacity
    supports_weight = obj['max_load'] >= robot['weight']

    return height_ok and width_ok and depth_ok and stable and supports_weight
```

</details>

### Exercise 2.4: Research Project (⭐⭐⭐ Advanced)

**Objective**: Analyze embodied design in a real robot.

**Task**: Choose one robot and analyze its embodied features:
- Boston Dynamics Spot
- Festo BionicKangaroo
- MIT Cheetah
- SoftBank Pepper

**Analyze**:
1. What morphological computation does it use?
2. How does body design simplify control?
3. What trade-offs were made?

**Deliverable**: 2-page analysis with diagrams

---

## Quiz: Week 2 Check

<details>
<summary>Q1: What is morphological computation? (2 pts)</summary>

**Answer**: Morphological computation is when the physical structure of the body performs computation that would otherwise require the brain/controller. Example: A compliant gripper that automatically regulates grasp force through material deformation.

</details>

<details>
<summary>Q2: Name three principles of embodied intelligence. (3 pts)</summary>

**Answer**: Any three of:
1. Morphological computation
2. Sensorimotor coupling
3. Environmental scaffolding
4. Active perception
5. Body-brain co-design

</details>

<details>
<summary>Q3: What is an affordance? Give an example. (2 pts)</summary>

**Answer**: An affordance is an action possibility offered by the environment to an agent. Example: A chair affords "sitting" to a human (but not to a fish).

</details>

<details>
<summary>Q4: Why did Rodney Brooks say "the world is its own best model"? (2 pts)</summary>

**Answer**: Instead of building complex internal world models that are expensive to compute and may become outdated, robots can use the actual world as a reference by sensing it directly. The world is always accurate and doesn't require memory.

</details>

<details>
<summary>Q5: What advantage does a Series Elastic Actuator provide? (1 pt)</summary>

**Answer**: SEAs provide built-in compliance (safety), force sensing (through spring deflection), energy storage, and shock absorption—all through mechanical design rather than active control.

</details>

---

## Further Reading

- Pfeifer, R., & Bongard, J. (2006). *How the Body Shapes the Way We Think*
- Brooks, R. A. (1991). "Intelligence Without Representation"
- Gibson, J. J. (1979). *The Ecological Approach to Visual Perception*
- Clark, A. (1998). *Being There: Putting Brain, Body, and World Together Again*

---

## Next Steps

In [Week 3: ROS 2 Architecture](/module-1-ros2/week-3-ros2-architecture), we begin hands-on work with ROS 2:
- ROS 2 design philosophy and architecture
- Setting up your development environment
- Creating your first ROS 2 nodes
- Understanding the DDS middleware
