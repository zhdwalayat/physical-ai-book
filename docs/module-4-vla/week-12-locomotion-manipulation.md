---
sidebar_position: 2
---

# Week 12: Locomotion and Manipulation

## Learning Objectives

By the end of this week, you will be able to:

- **Implement** bipedal walking using Zero Moment Point control
- **Design** walking gaits using the Linear Inverted Pendulum Model
- **Plan** grasps for humanoid manipulation tasks
- **Coordinate** whole-body motion during loco-manipulation
- **Integrate** walking with ROS 2 Nav2 navigation stack

---

## 1. Bipedal Locomotion Fundamentals

### 1.1 The Walking Cycle

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Human Walking Gait Cycle                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Phase Timeline (one complete cycle = two steps)                           │
│   ───────────────────────────────────────────────────────────────────────   │
│   │ Double │ Right │ Double │ Left  │                                       │
│   │Support │Swing  │Support │Swing  │                                       │
│   │  10%   │  40%  │  10%   │  40%  │                                       │
│   ───────────────────────────────────────────────────────────────────────   │
│                                                                              │
│   Foot Contact Pattern:                                                      │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  L: ████████████████────────────────████████████████                │  │
│   │  R: ────────────────████████████████────────────────████████████    │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│          ▲         ▲         ▲         ▲         ▲                          │
│          │         │         │         │         │                          │
│       Heel      Toe-off   Heel      Toe-off   Heel                         │
│       Strike    (L)       Strike    (R)       Strike                        │
│       (L)                 (R)                 (L)                           │
│                                                                              │
│   Key Events:                                                               │
│   • Heel Strike: Foot contacts ground                                       │
│   • Loading Response: Weight transfers to new stance leg                   │
│   • Mid-Stance: Body passes over stance foot                               │
│   • Toe-Off: Stance leg pushes off                                          │
│   • Swing: Leg swings forward                                               │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Linear Inverted Pendulum Model (LIPM)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Linear Inverted Pendulum Model                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│    Physical Model:                                                          │
│                   ○ ← CoM (mass m, height h)                                │
│                  /│                                                          │
│                 / │                                                          │
│                /  │ h = constant                                            │
│               /   │                                                          │
│              /    │                                                          │
│             ●─────┴───── Ground plane                                       │
│           Foot                                                               │
│                                                                              │
│    Assumptions:                                                             │
│    1. CoM moves at constant height h                                        │
│    2. Point contact with ground                                              │
│    3. Massless legs                                                          │
│    4. No angular momentum about CoM                                         │
│                                                                              │
│    Dynamics:                                                                │
│                                                                              │
│        ẍ = (g/h)(x - p)          where p = foot position (support point)   │
│                                                                              │
│        ω = √(g/h)                natural frequency                          │
│                                                                              │
│    Solution:                                                                │
│                                                                              │
│        x(t) = (x₀ - p)cosh(ωt) + (ẋ₀/ω)sinh(ωt) + p                       │
│        ẋ(t) = (x₀ - p)ω·sinh(ωt) + ẋ₀·cosh(ωt)                            │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.3 Walking Pattern Generator

```python
#!/usr/bin/env python3
"""
walking_pattern.py - Bipedal walking pattern generation using LIPM
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum


class SupportPhase(Enum):
    """Support phase enumeration."""
    DOUBLE_SUPPORT = 0
    LEFT_SUPPORT = 1
    RIGHT_SUPPORT = 2


@dataclass
class FootstepPlan:
    """Planned footstep sequence."""
    positions: List[np.ndarray]  # [(x, y, θ), ...]
    foot: List[str]              # ['left', 'right', ...]
    timings: List[float]         # Step durations


@dataclass
class WalkingParams:
    """Walking parameters."""
    step_length: float = 0.25      # meters
    step_width: float = 0.10       # meters
    step_height: float = 0.05      # meters
    step_duration: float = 0.5     # seconds
    double_support_ratio: float = 0.1
    com_height: float = 0.85       # meters


class LIPMWalkingGenerator:
    """Walking pattern generator using Linear Inverted Pendulum Model."""

    def __init__(self, params: WalkingParams):
        self.params = params
        self.gravity = 9.81

        # LIPM natural frequency
        self.omega = np.sqrt(self.gravity / self.params.com_height)

    def plan_footsteps(
        self,
        start_pos: np.ndarray,
        goal_pos: np.ndarray,
        start_foot: str = 'right'
    ) -> FootstepPlan:
        """Plan footstep sequence to reach goal.

        Args:
            start_pos: Starting position [x, y, θ]
            goal_pos: Goal position [x, y, θ]
            start_foot: Which foot to step first

        Returns:
            FootstepPlan with positions and timings
        """
        positions = []
        feet = []
        timings = []

        current_pos = start_pos.copy()
        current_foot = start_foot

        # Direction to goal
        direction = goal_pos[:2] - current_pos[:2]
        distance = np.linalg.norm(direction)

        if distance < 0.01:
            return FootstepPlan([], [], [])

        direction = direction / distance
        heading = np.arctan2(direction[1], direction[0])

        # Plan steps
        while distance > self.params.step_length / 2:
            # Step position
            step_pos = current_pos[:2] + direction * self.params.step_length

            # Lateral offset based on which foot
            lateral = np.array([-direction[1], direction[0]])
            if current_foot == 'left':
                step_pos += lateral * self.params.step_width / 2
            else:
                step_pos -= lateral * self.params.step_width / 2

            positions.append(np.array([step_pos[0], step_pos[1], heading]))
            feet.append(current_foot)
            timings.append(self.params.step_duration)

            # Update state
            current_pos[:2] = step_pos
            current_foot = 'left' if current_foot == 'right' else 'right'
            distance = np.linalg.norm(goal_pos[:2] - current_pos[:2])

        return FootstepPlan(positions, feet, timings)

    def generate_com_trajectory(
        self,
        footsteps: FootstepPlan,
        initial_com: np.ndarray,
        initial_com_vel: np.ndarray,
        dt: float = 0.01
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Generate Center of Mass trajectory from footstep plan.

        Uses LIPM dynamics to compute CoM motion.

        Args:
            footsteps: Planned footsteps
            initial_com: Initial CoM position [x, y]
            initial_com_vel: Initial CoM velocity [vx, vy]
            dt: Time step

        Returns:
            com_positions: Nx2 array of CoM positions
            com_velocities: Nx2 array of CoM velocities
        """
        if len(footsteps.positions) == 0:
            return np.array([initial_com]), np.array([initial_com_vel])

        com_positions = []
        com_velocities = []

        x = initial_com.copy()
        v = initial_com_vel.copy()

        for i, (foot_pos, duration) in enumerate(
            zip(footsteps.positions, footsteps.timings)
        ):
            # Support point
            p = foot_pos[:2]

            # Generate trajectory for this step
            t = 0
            while t < duration:
                # Store current state
                com_positions.append(x.copy())
                com_velocities.append(v.copy())

                # LIPM dynamics: ẍ = ω²(x - p)
                for dim in range(2):
                    x_rel = x[dim] - p[dim]
                    v_next = v[dim] + self.omega**2 * x_rel * dt
                    x[dim] = x[dim] + v[dim] * dt
                    v[dim] = v_next

                t += dt

        return np.array(com_positions), np.array(com_velocities)

    def generate_foot_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        duration: float,
        dt: float = 0.01
    ) -> np.ndarray:
        """Generate swing foot trajectory with smooth motion.

        Uses cycloid-like profile for natural swing.

        Args:
            start_pos: Starting foot position [x, y, z]
            end_pos: Ending foot position [x, y, z]
            duration: Swing duration
            dt: Time step

        Returns:
            Nx3 array of foot positions
        """
        n_points = int(duration / dt)
        trajectory = np.zeros((n_points, 3))

        for i in range(n_points):
            # Phase (0 to 1)
            phase = i / (n_points - 1)

            # Horizontal: smooth interpolation
            s = 3 * phase**2 - 2 * phase**3  # Cubic ease in/out
            trajectory[i, :2] = start_pos[:2] + s * (end_pos[:2] - start_pos[:2])

            # Vertical: bell curve
            h = self.params.step_height
            trajectory[i, 2] = h * np.sin(np.pi * phase)

        return trajectory


class WalkingController:
    """Real-time walking controller."""

    def __init__(self, params: WalkingParams, robot_model: 'HumanoidModel'):
        self.params = params
        self.robot = robot_model
        self.generator = LIPMWalkingGenerator(params)

        # State
        self.phase = SupportPhase.DOUBLE_SUPPORT
        self.phase_time = 0.0
        self.step_count = 0

        # Current targets
        self.left_foot_target = np.zeros(6)   # [x, y, z, roll, pitch, yaw]
        self.right_foot_target = np.zeros(6)
        self.com_target = np.zeros(3)

    def update(
        self,
        dt: float,
        current_com: np.ndarray,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
        velocity_command: np.ndarray
    ) -> dict:
        """Update walking controller state.

        Args:
            dt: Time step
            current_com: Current CoM position
            left_foot_pos: Current left foot pose
            right_foot_pos: Current right foot pose
            velocity_command: Desired [vx, vy, ω] velocity

        Returns:
            Dictionary with target poses for feet and CoM
        """
        self.phase_time += dt

        # Phase transitions
        swing_duration = self.params.step_duration * (1 - self.params.double_support_ratio)
        double_duration = self.params.step_duration * self.params.double_support_ratio

        if self.phase == SupportPhase.DOUBLE_SUPPORT:
            if self.phase_time >= double_duration:
                # Transition to single support
                if self.step_count % 2 == 0:
                    self.phase = SupportPhase.LEFT_SUPPORT  # Right swing
                else:
                    self.phase = SupportPhase.RIGHT_SUPPORT  # Left swing
                self.phase_time = 0.0
                self._plan_next_step(velocity_command)

        elif self.phase in [SupportPhase.LEFT_SUPPORT, SupportPhase.RIGHT_SUPPORT]:
            if self.phase_time >= swing_duration:
                # Transition to double support
                self.phase = SupportPhase.DOUBLE_SUPPORT
                self.phase_time = 0.0
                self.step_count += 1

        # Compute targets based on phase
        targets = self._compute_targets(
            self.phase_time,
            current_com,
            left_foot_pos,
            right_foot_pos
        )

        return targets

    def _plan_next_step(self, velocity_command: np.ndarray):
        """Plan the next step based on velocity command."""
        vx, vy, omega = velocity_command

        # Compute step target
        step_x = vx * self.params.step_duration
        step_y = vy * self.params.step_duration
        step_theta = omega * self.params.step_duration

        # Clamp to limits
        step_x = np.clip(step_x, -self.params.step_length, self.params.step_length)
        step_y = np.clip(step_y, -self.params.step_width, self.params.step_width)

        # Store for trajectory generation
        self._next_step_target = np.array([step_x, step_y, step_theta])

    def _compute_targets(
        self,
        phase_time: float,
        current_com: np.ndarray,
        left_foot: np.ndarray,
        right_foot: np.ndarray
    ) -> dict:
        """Compute target poses based on current phase."""
        targets = {
            'left_foot': left_foot.copy(),
            'right_foot': right_foot.copy(),
            'com': current_com.copy(),
            'phase': self.phase,
        }

        if self.phase == SupportPhase.DOUBLE_SUPPORT:
            # CoM moves toward swing foot
            pass

        elif self.phase == SupportPhase.LEFT_SUPPORT:
            # Right foot swings
            progress = phase_time / (self.params.step_duration * 0.9)
            progress = np.clip(progress, 0, 1)

            # Interpolate foot position
            swing_phase = np.sin(np.pi * progress)
            targets['right_foot'][2] = self.params.step_height * swing_phase

        elif self.phase == SupportPhase.RIGHT_SUPPORT:
            # Left foot swings
            progress = phase_time / (self.params.step_duration * 0.9)
            progress = np.clip(progress, 0, 1)

            swing_phase = np.sin(np.pi * progress)
            targets['left_foot'][2] = self.params.step_height * swing_phase

        return targets
```

---

## 2. Zero Moment Point Control

### 2.1 ZMP Walking Controller

```python
#!/usr/bin/env python3
"""
zmp_controller.py - ZMP-based walking controller
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class ZMPControllerParams:
    """ZMP controller parameters."""
    com_height: float = 0.85
    preview_time: float = 1.6      # Preview horizon
    preview_samples: int = 160     # Number of preview samples
    control_dt: float = 0.01       # Control timestep

    # Gains
    Qe: float = 1.0     # ZMP tracking weight
    Qx: float = 0.0     # State weight
    R: float = 1e-6     # Input weight


class ZMPPreviewController:
    """Preview control for ZMP walking.

    Uses preview control to track a reference ZMP trajectory
    while generating smooth CoM motion.
    """

    def __init__(self, params: ZMPControllerParams):
        self.params = params
        self.gravity = 9.81

        # LIPM state-space matrices
        self._setup_state_space()

        # Compute preview gains
        self._compute_preview_gains()

    def _setup_state_space(self):
        """Setup discrete state-space model."""
        dt = self.params.control_dt
        h = self.params.com_height
        g = self.gravity

        # Continuous LIPM state-space: [x, ẋ, ẍ]
        # ẋ = Ax + Bu
        # y = Cx (y = ZMP)
        omega2 = g / h

        # Discrete-time with zero-order hold
        self.A = np.array([
            [1, dt, dt**2/2],
            [0, 1, dt],
            [0, 0, 1]
        ])

        self.B = np.array([
            [dt**3/6],
            [dt**2/2],
            [dt]
        ])

        self.C = np.array([[1, 0, -h/g]])

    def _compute_preview_gains(self):
        """Compute preview control gains using LQR."""
        from scipy import linalg

        A, B, C = self.A, self.B, self.C
        Qe = self.params.Qe
        Qx = self.params.Qx
        R = self.params.R
        N = self.params.preview_samples

        # Augmented system for integral action
        # [e, x]' = [A_aug] [e, x] + [B_aug] u + [F] p_ref
        A_aug = np.block([
            [1, C @ A],
            [np.zeros((3, 1)), A]
        ])
        B_aug = np.vstack([C @ B, B])
        F = np.array([[1], [0], [0], [0]])

        # Weights
        Q = np.diag([Qe, Qx, Qx, Qx])

        # Solve discrete Riccati equation
        P = linalg.solve_discrete_are(A_aug, B_aug, Q, R)

        # State feedback gain
        self.Gi = 1.0 / (R + B_aug.T @ P @ B_aug) * B_aug.T @ P @ A_aug
        self.Gx = self.Gi[0, 1:4]
        self.Gi = self.Gi[0, 0]

        # Preview gains
        self.Gp = np.zeros(N)
        Ac = A_aug - B_aug @ self.Gi.reshape(1, -1) @ np.eye(4)

        X = -Ac.T @ P @ F
        for i in range(N):
            self.Gp[i] = (1.0 / (R + B_aug.T @ P @ B_aug)) * B_aug.T @ X
            X = Ac.T @ X

    def compute_control(
        self,
        state: np.ndarray,
        zmp_ref: np.ndarray,
        error_integral: float
    ) -> Tuple[float, float]:
        """Compute jerk input using preview control.

        Args:
            state: Current state [x, ẋ, ẍ]
            zmp_ref: Reference ZMP trajectory (preview horizon)
            error_integral: Integrated ZMP error

        Returns:
            u: Jerk input
            new_error_integral: Updated error integral
        """
        # Current ZMP
        zmp = self.C @ state

        # ZMP error
        zmp_error = zmp[0] - zmp_ref[0]

        # Update error integral
        new_error_integral = error_integral + zmp_error

        # Preview term
        preview_term = 0.0
        for i in range(min(len(zmp_ref), len(self.Gp))):
            preview_term += self.Gp[i] * zmp_ref[i]

        # Control input
        u = -self.Gi * new_error_integral - self.Gx @ state + preview_term

        return u, new_error_integral


class ZMPWalkingExecutor:
    """Execute walking using ZMP control with IK."""

    def __init__(
        self,
        robot_model: 'HumanoidModel',
        fk_solver: 'ForwardKinematics',
        ik_solver: 'InverseKinematics'
    ):
        self.robot = robot_model
        self.fk = fk_solver
        self.ik = ik_solver

        # Controllers for each axis
        params = ZMPControllerParams()
        self.x_controller = ZMPPreviewController(params)
        self.y_controller = ZMPPreviewController(params)

        # State
        self.x_state = np.zeros(3)  # [x, ẋ, ẍ]
        self.y_state = np.zeros(3)  # [y, ẏ, ÿ]
        self.x_error_int = 0.0
        self.y_error_int = 0.0

    def step(
        self,
        zmp_ref_x: np.ndarray,
        zmp_ref_y: np.ndarray,
        left_foot_target: np.ndarray,
        right_foot_target: np.ndarray,
        current_angles: dict
    ) -> dict:
        """Execute one control step.

        Args:
            zmp_ref_x: Reference ZMP x trajectory
            zmp_ref_y: Reference ZMP y trajectory
            left_foot_target: Target left foot pose
            right_foot_target: Target right foot pose
            current_angles: Current joint angles

        Returns:
            joint_angles: Target joint angles
        """
        dt = 0.01

        # Compute CoM control
        ux, self.x_error_int = self.x_controller.compute_control(
            self.x_state, zmp_ref_x, self.x_error_int
        )
        uy, self.y_error_int = self.y_controller.compute_control(
            self.y_state, zmp_ref_y, self.y_error_int
        )

        # Integrate state (triple integrator)
        A, B = self.x_controller.A, self.x_controller.B
        self.x_state = A @ self.x_state + B.flatten() * ux
        self.y_state = A @ self.y_state + B.flatten() * uy

        # CoM position
        com_target = np.array([
            self.x_state[0],
            self.y_state[0],
            0.85  # Fixed height
        ])

        # Solve IK for both legs
        # (Simplified - would need proper whole-body IK)
        left_angles, _ = self.ik.solve(
            'left_leg', left_foot_target[:3], left_foot_target[3:]
        )
        right_angles, _ = self.ik.solve(
            'right_leg', right_foot_target[:3], right_foot_target[3:]
        )

        # Combine
        joint_angles = {**left_angles, **right_angles}

        return joint_angles
```

---

## 3. Humanoid Manipulation

### 3.1 Grasp Planning

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Grasp Types for Humanoid Hands                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Power Grasp             Precision Grasp          Lateral Pinch            │
│   ───────────             ───────────────          ──────────────           │
│                                                                              │
│   ┌─────────┐              ○ ○                      │  │                    │
│   │ ████████│             ○   ○                     │  │                    │
│   │ ████████│              │   │                    │  ○                    │
│   │ ████████│              │   │                    │  │                    │
│   │ ████████│              │   │                    │  │                    │
│   └─────────┘              └───┘                    └──┘                    │
│                                                                              │
│   • Full hand wrap        • Fingertip contact      • Thumb-side grasp      │
│   • High force            • Fine control           • For flat objects      │
│   • Large objects         • Small objects          • Keys, cards           │
│                                                                              │
│   ───────────────────────────────────────────────────────────────────────   │
│                                                                              │
│   Grasp Quality Metrics:                                                    │
│   • Force Closure: Can resist forces in all directions                      │
│   • Grasp Wrench Space: Set of wrenches that can be applied                │
│   • Epsilon Metric: Largest perturbation that can be resisted              │
│   • Volume Metric: Size of grasp wrench space                              │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Grasp Planner Implementation

```python
#!/usr/bin/env python3
"""
grasp_planner.py - Grasp planning for humanoid manipulation
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from scipy.spatial.transform import Rotation as R


@dataclass
class GraspCandidate:
    """A candidate grasp pose."""
    position: np.ndarray      # Grasp center position
    orientation: np.ndarray   # Grasp orientation (quaternion)
    approach: np.ndarray      # Approach direction
    grasp_width: float        # Opening width
    quality: float            # Quality score
    grasp_type: str           # 'power', 'precision', 'lateral'


@dataclass
class ContactPoint:
    """Contact point for grasp analysis."""
    position: np.ndarray
    normal: np.ndarray
    friction: float = 0.5


class GraspPlanner:
    """Plan grasps for humanoid manipulation."""

    def __init__(
        self,
        hand_width: float = 0.10,
        max_grasp_width: float = 0.12,
        min_grasp_width: float = 0.02
    ):
        self.hand_width = hand_width
        self.max_grasp_width = max_grasp_width
        self.min_grasp_width = min_grasp_width

    def sample_grasp_candidates(
        self,
        object_mesh: 'Mesh',
        object_pose: np.ndarray,
        num_samples: int = 100
    ) -> List[GraspCandidate]:
        """Sample grasp candidates on object surface.

        Args:
            object_mesh: Object mesh
            object_pose: Object pose [x, y, z, qx, qy, qz, qw]
            num_samples: Number of candidates to sample

        Returns:
            List of grasp candidates
        """
        candidates = []

        # Get object bounding box
        bbox = self._compute_bbox(object_mesh, object_pose)
        center = (bbox[0] + bbox[1]) / 2
        dimensions = bbox[1] - bbox[0]

        for _ in range(num_samples):
            # Sample approach direction
            phi = np.random.uniform(0, 2 * np.pi)
            theta = np.random.uniform(0, np.pi)
            approach = np.array([
                np.sin(theta) * np.cos(phi),
                np.sin(theta) * np.sin(phi),
                np.cos(theta)
            ])

            # Grasp center (random offset from object center)
            offset = np.random.uniform(-0.5, 0.5, 3) * dimensions * 0.3
            grasp_center = center + offset

            # Grasp orientation (approach as -z, compute x and y)
            z_axis = -approach
            x_axis = np.cross([0, 0, 1], z_axis)
            if np.linalg.norm(x_axis) < 0.01:
                x_axis = np.cross([0, 1, 0], z_axis)
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)

            rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
            orientation = R.from_matrix(rot_matrix).as_quat()

            # Estimate grasp width based on object dimension
            grasp_width = min(
                self.max_grasp_width,
                max(self.min_grasp_width, np.min(dimensions) * 0.8)
            )

            candidate = GraspCandidate(
                position=grasp_center,
                orientation=orientation,
                approach=approach,
                grasp_width=grasp_width,
                quality=0.0,  # To be computed
                grasp_type='power'
            )
            candidates.append(candidate)

        return candidates

    def evaluate_grasp(
        self,
        candidate: GraspCandidate,
        object_mesh: 'Mesh'
    ) -> float:
        """Evaluate grasp quality.

        Args:
            candidate: Grasp candidate
            object_mesh: Object mesh

        Returns:
            Quality score (0 to 1)
        """
        # Simplified quality metrics
        quality = 0.0

        # 1. Approach direction (prefer top-down)
        vertical_alignment = np.abs(candidate.approach[2])
        quality += 0.3 * vertical_alignment

        # 2. Grasp width appropriateness
        width_score = 1.0 - abs(candidate.grasp_width - 0.08) / 0.10
        quality += 0.3 * max(0, width_score)

        # 3. Distance from center (prefer centered grasps)
        # Would need actual object center
        quality += 0.2

        # 4. Force closure (simplified check)
        # Full analysis would involve contact wrench space
        quality += 0.2

        return np.clip(quality, 0, 1)

    def rank_grasps(
        self,
        candidates: List[GraspCandidate],
        object_mesh: 'Mesh'
    ) -> List[GraspCandidate]:
        """Rank grasp candidates by quality.

        Args:
            candidates: List of grasp candidates
            object_mesh: Object mesh

        Returns:
            Sorted list of candidates (best first)
        """
        for candidate in candidates:
            candidate.quality = self.evaluate_grasp(candidate, object_mesh)

        return sorted(candidates, key=lambda g: g.quality, reverse=True)

    def _compute_bbox(
        self,
        mesh: 'Mesh',
        pose: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute axis-aligned bounding box."""
        # Placeholder - would transform mesh vertices
        return np.array([-0.05, -0.05, 0]), np.array([0.05, 0.05, 0.1])


class GraspExecutor:
    """Execute planned grasps."""

    def __init__(
        self,
        robot_model: 'HumanoidModel',
        ik_solver: 'InverseKinematics'
    ):
        self.robot = robot_model
        self.ik = ik_solver

        # Approach distance
        self.approach_distance = 0.10
        self.grasp_speed = 0.05  # m/s

    def compute_approach_pose(
        self,
        grasp: GraspCandidate
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute pre-grasp approach pose.

        Returns:
            position: Approach position
            orientation: Approach orientation
        """
        approach_pos = grasp.position + grasp.approach * self.approach_distance
        return approach_pos, grasp.orientation

    def plan_grasp_trajectory(
        self,
        grasp: GraspCandidate,
        current_hand_pose: np.ndarray
    ) -> List[np.ndarray]:
        """Plan trajectory for grasp execution.

        Phases:
        1. Move to approach pose
        2. Approach object
        3. Close gripper
        4. Lift

        Returns:
            List of waypoint poses
        """
        waypoints = []

        # 1. Approach pose
        approach_pos, approach_ori = self.compute_approach_pose(grasp)
        waypoints.append(np.concatenate([approach_pos, approach_ori]))

        # 2. Grasp pose
        waypoints.append(np.concatenate([grasp.position, grasp.orientation]))

        # 3. Lift pose (same orientation, higher position)
        lift_pos = grasp.position.copy()
        lift_pos[2] += 0.10  # Lift 10cm
        waypoints.append(np.concatenate([lift_pos, grasp.orientation]))

        return waypoints

    async def execute_grasp(
        self,
        grasp: GraspCandidate,
        hand_controller: 'HandController'
    ) -> bool:
        """Execute a planned grasp.

        Args:
            grasp: Grasp to execute
            hand_controller: Controller for gripper

        Returns:
            Success status
        """
        # Open gripper
        await hand_controller.open(grasp.grasp_width * 1.2)

        # Move to approach
        approach_pos, approach_ori = self.compute_approach_pose(grasp)
        success = await self._move_to_pose(approach_pos, approach_ori)
        if not success:
            return False

        # Approach object
        success = await self._move_to_pose(grasp.position, grasp.orientation)
        if not success:
            return False

        # Close gripper
        await hand_controller.close(grasp.grasp_width * 0.9)

        # Check if grasp succeeded (object detected)
        if not hand_controller.has_object():
            return False

        # Lift
        lift_pos = grasp.position.copy()
        lift_pos[2] += 0.10
        await self._move_to_pose(lift_pos, grasp.orientation)

        return True

    async def _move_to_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray
    ) -> bool:
        """Move hand to target pose using IK."""
        # Solve IK
        angles, success = self.ik.solve(
            'right_arm', position, orientation
        )

        if not success:
            return False

        # Execute motion (would interface with actual controller)
        # ...

        return True
```

---

## 4. Whole-Body Coordination

### 4.1 Hierarchical Task Control

```python
#!/usr/bin/env python3
"""
whole_body_control.py - Whole-body motion coordination
"""

import numpy as np
from typing import List, Dict, Optional, Tuple


class Task:
    """Base class for control tasks."""

    def __init__(self, name: str, priority: int):
        self.name = name
        self.priority = priority  # Lower = higher priority

    def compute_error(self, state: dict) -> np.ndarray:
        """Compute task error."""
        raise NotImplementedError

    def compute_jacobian(self, state: dict) -> np.ndarray:
        """Compute task Jacobian."""
        raise NotImplementedError


class PositionTask(Task):
    """Track end-effector position."""

    def __init__(
        self,
        name: str,
        priority: int,
        chain_name: str,
        target_position: np.ndarray,
        fk_solver: 'ForwardKinematics',
        model: 'HumanoidModel'
    ):
        super().__init__(name, priority)
        self.chain_name = chain_name
        self.target = target_position
        self.fk = fk_solver
        self.model = model

    def compute_error(self, state: dict) -> np.ndarray:
        pos, _ = self.fk.get_end_effector_pose(self.chain_name, state['joint_angles'])
        return self.target - pos

    def compute_jacobian(self, state: dict) -> np.ndarray:
        # Compute position Jacobian (3 x n_joints)
        from inverse_kinematics import DampedLeastSquaresIK
        ik = DampedLeastSquaresIK(self.model, self.fk)
        J_full = ik.compute_jacobian(self.chain_name, state['joint_angles'])
        return J_full[:3, :]  # Position only


class CoMTask(Task):
    """Track center of mass position."""

    def __init__(
        self,
        name: str,
        priority: int,
        target_com: np.ndarray,
        com_solver: 'CenterOfMass'
    ):
        super().__init__(name, priority)
        self.target = target_com
        self.com_solver = com_solver

    def compute_error(self, state: dict) -> np.ndarray:
        com, _ = self.com_solver.compute_total_com(state['joint_angles'])
        return self.target - com

    def compute_jacobian(self, state: dict) -> np.ndarray:
        return self.com_solver.compute_com_jacobian(state['joint_angles'])


class PostureTask(Task):
    """Maintain reference posture."""

    def __init__(
        self,
        name: str,
        priority: int,
        reference_posture: Dict[str, float]
    ):
        super().__init__(name, priority)
        self.reference = reference_posture

    def compute_error(self, state: dict) -> np.ndarray:
        errors = []
        for joint_name, ref_angle in self.reference.items():
            current = state['joint_angles'].get(joint_name, 0.0)
            errors.append(ref_angle - current)
        return np.array(errors)

    def compute_jacobian(self, state: dict) -> np.ndarray:
        # Identity for joint-space task
        n = len(self.reference)
        return np.eye(n)


class WholeBodyController:
    """Whole-body controller using prioritized task control."""

    def __init__(self, model: 'HumanoidModel'):
        self.model = model
        self.tasks: List[Task] = []

        # Control parameters
        self.damping = 0.01
        self.max_velocity = 2.0

    def add_task(self, task: Task):
        """Add a task to the controller."""
        self.tasks.append(task)
        # Sort by priority
        self.tasks.sort(key=lambda t: t.priority)

    def remove_task(self, task_name: str):
        """Remove a task by name."""
        self.tasks = [t for t in self.tasks if t.name != task_name]

    def compute_joint_velocities(
        self,
        state: dict,
        dt: float = 0.01
    ) -> np.ndarray:
        """Compute joint velocities using prioritized task control.

        Uses null-space projection to handle task hierarchy.
        """
        n_joints = self.model.num_joints
        q_dot = np.zeros(n_joints)
        null_space = np.eye(n_joints)

        for task in self.tasks:
            # Get task error and Jacobian
            error = task.compute_error(state)
            J = task.compute_jacobian(state)

            # Project Jacobian into remaining null space
            J_projected = J @ null_space

            # Damped pseudo-inverse
            JJT = J_projected @ J_projected.T
            damped = JJT + self.damping ** 2 * np.eye(J_projected.shape[0])
            J_pinv = J_projected.T @ np.linalg.inv(damped)

            # Compute contribution to joint velocity
            q_dot_task = J_pinv @ error

            # Add contribution
            q_dot += null_space @ q_dot_task

            # Update null space
            null_space = null_space @ (np.eye(n_joints) - J_pinv @ J_projected)

        # Limit velocities
        q_dot = np.clip(q_dot, -self.max_velocity, self.max_velocity)

        return q_dot

    def step(self, state: dict, dt: float = 0.01) -> Dict[str, float]:
        """Execute one control step.

        Returns:
            New joint angles
        """
        q_dot = self.compute_joint_velocities(state, dt)

        # Integrate
        joint_names = list(state['joint_angles'].keys())
        new_angles = {}
        for i, name in enumerate(joint_names):
            new_angles[name] = state['joint_angles'][name] + q_dot[i] * dt

            # Enforce limits
            limits = self.model.joints[name].limits
            new_angles[name] = np.clip(new_angles[name], limits[0], limits[1])

        return new_angles


# Example: Reach while balancing
def reach_while_balancing_example():
    """Demonstrate reaching while maintaining balance."""

    # Create robot and solvers
    model = HumanoidModel()
    fk = ForwardKinematics(model)
    com_solver = CenterOfMass(model, fk)

    # Create controller
    controller = WholeBodyController(model)

    # Add tasks (priority order)
    # 1. Balance (highest priority)
    com_target = np.array([0.0, 0.0, 0.85])
    balance_task = CoMTask("balance", 0, com_target, com_solver)
    controller.add_task(balance_task)

    # 2. Hand position (secondary)
    hand_target = np.array([0.5, 0.2, 1.0])
    reach_task = PositionTask("reach", 1, "right_arm", hand_target, fk, model)
    controller.add_task(reach_task)

    # 3. Posture (lowest priority)
    reference_posture = {j: 0.0 for j in model.joint_order}
    posture_task = PostureTask("posture", 2, reference_posture)
    controller.add_task(posture_task)

    # Simulate
    state = {'joint_angles': {j: 0.0 for j in model.joint_order}}

    for i in range(100):
        new_angles = controller.step(state, dt=0.01)
        state['joint_angles'] = new_angles

        # Check task errors
        com_error = np.linalg.norm(balance_task.compute_error(state))
        hand_error = np.linalg.norm(reach_task.compute_error(state))

        if i % 10 == 0:
            print(f"Step {i}: CoM error = {com_error:.4f}, Hand error = {hand_error:.4f}")
```

---

## 5. Nav2 Integration

### 5.1 Humanoid Navigation with Nav2

```python
#!/usr/bin/env python3
"""
humanoid_navigator.py - Nav2 integration for humanoid robot navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import numpy as np
from typing import Optional, List


class HumanoidNavigator(Node):
    """High-level navigation for humanoid robot using Nav2."""

    def __init__(self):
        super().__init__('humanoid_navigator')

        # Navigation action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Velocity command publisher (for direct control)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # State subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # State
        self.current_pose = None
        self.is_navigating = False

        self.get_logger().info('Humanoid navigator initialized')

    def odom_callback(self, msg: Odometry):
        """Store current pose from odometry."""
        self.current_pose = msg.pose.pose

    async def navigate_to_pose(
        self,
        x: float,
        y: float,
        theta: float,
        frame_id: str = 'map'
    ) -> bool:
        """Navigate to a target pose.

        Args:
            x, y: Target position
            theta: Target orientation (yaw)
            frame_id: Reference frame

        Returns:
            True if navigation succeeded
        """
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = np.sin(theta / 2)
        goal.pose.pose.orientation.w = np.cos(theta / 2)

        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f}, {theta:.2f})')
        self.is_navigating = True

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback
        )

        # Wait for goal acceptance
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.is_navigating = False
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await result_future

        self.is_navigating = False

        if result.result.result == 0:  # Success
            self.get_logger().info('Navigation succeeded')
            return True
        else:
            self.get_logger().warn(f'Navigation failed with code {result.result.result}')
            return False

    def _nav_feedback_callback(self, feedback_msg):
        """Process navigation feedback."""
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f}m', throttle_duration_sec=2.0)

    def stop(self):
        """Stop the robot."""
        self.cancel_navigation()

        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def cancel_navigation(self):
        """Cancel current navigation goal."""
        if self.is_navigating:
            self.nav_client.cancel_goal_async()
            self.is_navigating = False


class HumanoidMobileManipulation(Node):
    """Combined navigation and manipulation for humanoid."""

    def __init__(self):
        super().__init__('humanoid_mobile_manipulation')

        self.navigator = HumanoidNavigator()
        # self.grasp_planner = GraspPlanner()
        # self.grasp_executor = GraspExecutor(...)

    async def pick_object(
        self,
        object_pose: np.ndarray,
        object_name: str
    ) -> bool:
        """Navigate to and pick up an object.

        Args:
            object_pose: Object pose [x, y, z, qx, qy, qz, qw]
            object_name: Name of object for logging

        Returns:
            True if pick succeeded
        """
        self.get_logger().info(f'Picking up {object_name}')

        # 1. Compute approach position (0.5m from object)
        obj_x, obj_y = object_pose[0], object_pose[1]
        approach_distance = 0.5

        # Approach from behind
        approach_x = obj_x - approach_distance
        approach_y = obj_y
        approach_theta = 0  # Face forward

        # 2. Navigate to approach position
        self.get_logger().info('Navigating to approach position')
        success = await self.navigator.navigate_to_pose(
            approach_x, approach_y, approach_theta
        )
        if not success:
            self.get_logger().error('Failed to reach approach position')
            return False

        # 3. Plan grasp
        self.get_logger().info('Planning grasp')
        # grasp = self.grasp_planner.plan(object_mesh, object_pose)

        # 4. Execute grasp
        self.get_logger().info('Executing grasp')
        # success = await self.grasp_executor.execute_grasp(grasp)

        return True

    async def place_object(
        self,
        place_pose: np.ndarray,
        location_name: str
    ) -> bool:
        """Navigate to and place held object.

        Args:
            place_pose: Target place pose
            location_name: Name of location for logging

        Returns:
            True if place succeeded
        """
        self.get_logger().info(f'Placing object at {location_name}')

        # 1. Navigate near place location
        place_x, place_y = place_pose[0], place_pose[1]
        approach_x = place_x - 0.4
        approach_y = place_y

        success = await self.navigator.navigate_to_pose(
            approach_x, approach_y, 0
        )
        if not success:
            return False

        # 2. Execute place motion
        self.get_logger().info('Executing place motion')
        # success = await self.place_executor.execute_place(place_pose)

        return True


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNavigator()

    # Example: Navigate to a point
    import asyncio

    async def run():
        success = await node.navigate_to_pose(2.0, 1.0, 0.0)
        print(f"Navigation result: {success}")

    asyncio.run(run())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 6. Summary

### Locomotion Methods Comparison

| Method | Pros | Cons | Use Case |
|--------|------|------|----------|
| **ZMP Control** | Stable, proven | Conservative | Flat terrain |
| **LIPM** | Simple model | Limited dynamics | Pattern generation |
| **Capture Point** | Handles pushes | More complex | Dynamic walking |
| **RL-based** | Handles terrain | Training required | Complex environments |

### Key Concepts

| Concept | Description |
|---------|-------------|
| **ZMP** | Point where moments sum to zero |
| **LIPM** | Simplified CoM dynamics model |
| **Support Polygon** | Convex hull of foot contacts |
| **Force Closure** | Grasp can resist any wrench |
| **Null-space** | Joint motion that doesn't affect task |

---

## Exercises

### Exercise 12.1: Walking Pattern Generation (⭐⭐)

1. Implement LIPM-based CoM trajectory generation
2. Generate swing foot trajectories with smooth profiles
3. Visualize the walking pattern
4. Test with different step lengths and durations

### Exercise 12.2: ZMP Walking Controller (⭐⭐⭐)

1. Implement ZMP preview controller
2. Generate reference ZMP trajectory from footsteps
3. Track ZMP while generating CoM motion
4. Verify ZMP stays within support polygon

### Exercise 12.3: Grasp Planning (⭐⭐)

1. Sample grasp candidates on an object
2. Implement grasp quality evaluation
3. Rank grasps and visualize best options
4. Plan approach trajectory for top grasp

### Exercise 12.4: Mobile Manipulation (⭐⭐⭐)

1. Integrate navigation with manipulation
2. Navigate to object, pick it up
3. Navigate to goal, place object
4. Handle failures gracefully

---

## Quiz

<details>
<summary>Q1: What is the Zero Moment Point (ZMP)?</summary>

The ZMP is the point on the ground where the sum of all moments (due to gravity and inertia) equals zero. For stable walking:
- ZMP must stay within the support polygon
- Computed as: p_zmp = p_com - (h/g) * ẍ_com
- When ZMP exits support polygon, the robot will tip

</details>

<details>
<summary>Q2: What is the advantage of preview control for walking?</summary>

Preview control advantages:
1. Uses future reference trajectory to anticipate CoM motion
2. Reduces tracking error compared to reactive control
3. Enables smooth transitions between steps
4. Optimal in LQR sense for tracking performance
5. Handles the non-minimum phase nature of LIPM

</details>

<details>
<summary>Q3: What makes a grasp "force closure"?</summary>

A grasp has force closure when:
1. Contact forces can resist any external wrench
2. The grasp wrench space spans 6D (3 force + 3 torque)
3. Mathematically: G @ f = w is solvable for any w with f ≥ 0
4. Requires at least 3 contacts (with friction)
5. Enables stable holding without active control

</details>

<details>
<summary>Q4: How does whole-body control handle multiple tasks?</summary>

Whole-body control uses prioritized task control:
1. Tasks are ordered by priority
2. Higher priority tasks are satisfied first
3. Lower priority tasks use null-space of higher ones
4. Null-space projection: N = I - J†J
5. Combined velocity: q̇ = J₁†e₁ + N₁(J₂†e₂ + N₂(...))

This ensures critical tasks (like balance) are never violated.

</details>

---

## Next Steps

In [Week 13: Conversational Robotics](/module-4-vla/week-13-conversational-robotics), we add:
- Speech recognition with Whisper
- LLM-based task planning
- Voice-to-action pipelines
- Multi-modal human-robot interaction
