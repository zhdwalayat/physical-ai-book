---
sidebar_position: 2
---

# Week 12: Locomotion and Manipulation

## Learning Objectives

By the end of this week, you will be able to:

- Understand bipedal locomotion principles
- Implement walking gait patterns
- Design manipulation strategies for humanoid hands
- Coordinate whole-body motion

## Bipedal Locomotion

### The Walking Cycle

```
   ┌──────────────────────────────────────────────────────────────┐
   │                     Walking Gait Cycle                        │
   │                                                               │
   │   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐     │
   │   │ Double  │   │ Single  │   │ Double  │   │ Single  │     │
   │   │ Support │──►│ Support │──►│ Support │──►│ Support │──►  │
   │   │  (L+R)  │   │   (L)   │   │  (L+R)  │   │   (R)   │     │
   │   └─────────┘   └─────────┘   └─────────┘   └─────────┘     │
   │       10%          40%           10%           40%           │
   │                                                               │
   └──────────────────────────────────────────────────────────────┘
```

### Zero Moment Point (ZMP)

For stable walking, the ZMP must stay within the support polygon:

```python
class ZMPWalkingController:
    def __init__(self, robot):
        self.robot = robot
        self.com_height = 0.8  # meters
        self.step_length = 0.3
        self.step_duration = 0.5

    def plan_footsteps(self, target_position):
        """Plan sequence of footsteps to reach target."""
        footsteps = []
        current_pos = self.robot.get_position()

        while np.linalg.norm(current_pos - target_position) > self.step_length:
            direction = (target_position - current_pos)
            direction = direction / np.linalg.norm(direction)

            next_step = current_pos + direction * self.step_length
            footsteps.append(next_step)
            current_pos = next_step

        return footsteps

    def generate_com_trajectory(self, footsteps):
        """Generate CoM trajectory using Linear Inverted Pendulum Model."""
        g = 9.81
        omega = np.sqrt(g / self.com_height)

        com_trajectory = []

        for i, footstep in enumerate(footsteps):
            # LIPM dynamics: x(t) = x0*cosh(ωt) + v0/ω*sinh(ωt)
            t = np.linspace(0, self.step_duration, 50)

            for ti in t:
                x = footstep[0] * np.cosh(omega * ti)
                com_trajectory.append(x)

        return com_trajectory
```

### Walking State Machine

```python
from enum import Enum

class WalkingState(Enum):
    STANDING = 0
    LEFT_SWING = 1
    RIGHT_SWING = 2
    DOUBLE_SUPPORT = 3

class WalkingStateMachine:
    def __init__(self):
        self.state = WalkingState.STANDING
        self.phase_time = 0

    def update(self, dt, contact_left, contact_right):
        self.phase_time += dt

        if self.state == WalkingState.STANDING:
            if self.should_start_walking():
                self.state = WalkingState.LEFT_SWING
                self.phase_time = 0

        elif self.state == WalkingState.LEFT_SWING:
            if contact_left and self.phase_time > 0.3:
                self.state = WalkingState.DOUBLE_SUPPORT
                self.phase_time = 0

        elif self.state == WalkingState.DOUBLE_SUPPORT:
            if self.phase_time > 0.1:
                # Alternate legs
                self.state = WalkingState.RIGHT_SWING
                self.phase_time = 0

        elif self.state == WalkingState.RIGHT_SWING:
            if contact_right and self.phase_time > 0.3:
                self.state = WalkingState.DOUBLE_SUPPORT
                self.phase_time = 0

        return self.state
```

## Manipulation with Humanoid Hands

### Grasp Types

| Grasp | Description | Use Case |
|-------|-------------|----------|
| **Power** | Full hand wrap | Heavy objects |
| **Precision** | Fingertip | Small objects |
| **Lateral** | Thumb-side | Cards, keys |
| **Hook** | Fingers curled | Handles |

### Grasp Planning

```python
class GraspPlanner:
    def __init__(self, hand_model):
        self.hand = hand_model

    def plan_grasp(self, object_mesh, object_pose):
        """
        Plan grasp points for an object.
        """
        # Sample grasp candidates
        candidates = self.sample_grasp_candidates(object_mesh)

        # Score each candidate
        scored_grasps = []
        for grasp in candidates:
            score = self.evaluate_grasp(grasp, object_mesh)
            scored_grasps.append((score, grasp))

        # Return best grasp
        scored_grasps.sort(reverse=True)
        return scored_grasps[0][1]

    def evaluate_grasp(self, grasp, object_mesh):
        """
        Evaluate grasp quality using force closure metric.
        """
        contact_points = grasp.contact_points
        contact_normals = grasp.contact_normals

        # Check force closure
        G = self.compute_grasp_matrix(contact_points, contact_normals)
        rank = np.linalg.matrix_rank(G)

        # Higher rank = better force closure
        return rank

    def execute_grasp(self, grasp):
        """
        Execute planned grasp.
        """
        # 1. Move hand to pre-grasp pose
        pre_grasp = grasp.get_pregrasp_pose()
        self.hand.move_to(pre_grasp)

        # 2. Approach object
        approach = grasp.get_approach_pose()
        self.hand.move_to(approach)

        # 3. Close fingers
        self.hand.close_fingers(grasp.finger_positions)

        # 4. Verify grasp
        return self.hand.is_grasping()
```

## Whole-Body Coordination

```python
class WholeBodyController:
    def __init__(self, robot):
        self.robot = robot

    def coordinated_reach(self, target_hand_pos, maintain_balance=True):
        """
        Reach with hand while maintaining balance.

        Uses hierarchical task control:
        1. Balance (highest priority)
        2. Hand position (secondary)
        3. Posture (lowest priority)
        """
        # Task 1: Balance - keep CoM over support
        J_balance = self.compute_balance_jacobian()
        task_balance = self.compute_balance_task()

        # Task 2: Hand position
        J_hand = self.compute_hand_jacobian()
        task_hand = target_hand_pos - self.robot.get_hand_position()

        # Null space projection
        N1 = np.eye(self.robot.num_joints) - np.linalg.pinv(J_balance) @ J_balance

        # Hierarchical solution
        q_dot = np.linalg.pinv(J_balance) @ task_balance
        q_dot += N1 @ np.linalg.pinv(J_hand @ N1) @ (task_hand - J_hand @ q_dot)

        return q_dot
```

## Nav2 Integration for Humanoid Navigation

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class HumanoidNavigator:
    def __init__(self):
        self.navigator = BasicNavigator()

    def navigate_to(self, x, y, theta):
        """
        Navigate humanoid to target pose using Nav2.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = np.sin(theta / 2)
        goal_pose.pose.orientation.w = np.cos(theta / 2)

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # Monitor progress
            pass

        result = self.navigator.getResult()
        return result
```

## Exercises

1. Implement a simple walking gait generator
2. Plan footsteps to reach a target 2 meters away
3. Design a grasp strategy for picking up a cylinder
4. Integrate walking with Nav2 for autonomous navigation

## Next Steps

In [Week 13](/module-4-vla/week-13-conversational-robotics), we add conversational AI to our humanoid.
