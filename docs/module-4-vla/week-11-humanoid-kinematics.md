---
sidebar_position: 1
---

# Week 11: Humanoid Kinematics

## Learning Objectives

By the end of this week, you will be able to:

- **Model** humanoid robot kinematic chains and joint structures
- **Implement** forward kinematics using DH parameters
- **Solve** inverse kinematics for humanoid limbs
- **Compute** Jacobians for velocity-level control
- **Design** balance controllers using Center of Mass analysis

---

## 1. Humanoid Robot Structure

### 1.1 Kinematic Tree

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     Humanoid Kinematic Tree                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│                              ┌───────────┐                                  │
│                              │   Head    │                                  │
│                              │  (2 DoF)  │                                  │
│                              └─────┬─────┘                                  │
│                                    │ Neck                                   │
│                                    │                                        │
│           ┌────────────────────────┼────────────────────────┐              │
│           │                        │                        │              │
│     ┌─────┴─────┐            ┌─────┴─────┐           ┌─────┴─────┐        │
│     │ L.Shoulder│            │   Torso   │           │ R.Shoulder│        │
│     │  (3 DoF)  │            │  (Base)   │           │  (3 DoF)  │        │
│     └─────┬─────┘            └─────┬─────┘           └─────┬─────┘        │
│           │                        │                        │              │
│     ┌─────┴─────┐                  │                 ┌─────┴─────┐        │
│     │  L.Elbow  │                  │                 │  R.Elbow  │        │
│     │  (2 DoF)  │                  │                 │  (2 DoF)  │        │
│     └─────┬─────┘                  │                 └─────┬─────┘        │
│           │                        │                        │              │
│     ┌─────┴─────┐            ┌─────┴─────┐           ┌─────┴─────┐        │
│     │  L.Wrist  │            │  Pelvis   │           │  R.Wrist  │        │
│     │  (2 DoF)  │            │   Base    │           │  (2 DoF)  │        │
│     └─────┬─────┘            └─────┬─────┘           └─────┬─────┘        │
│           │                        │                        │              │
│     ┌─────┴─────┐        ┌─────────┼─────────┐       ┌─────┴─────┐        │
│     │  L.Hand   │        │                   │       │  R.Hand   │        │
│     │ (5+ DoF)  │   ┌────┴────┐       ┌────┴────┐   │ (5+ DoF)  │        │
│     └───────────┘   │  L.Hip  │       │  R.Hip  │   └───────────┘        │
│                     │ (3 DoF) │       │ (3 DoF) │                        │
│                     └────┬────┘       └────┬────┘                        │
│                          │                 │                              │
│                     ┌────┴────┐       ┌────┴────┐                        │
│                     │ L.Knee  │       │ R.Knee  │                        │
│                     │ (1 DoF) │       │ (1 DoF) │                        │
│                     └────┬────┘       └────┬────┘                        │
│                          │                 │                              │
│                     ┌────┴────┐       ┌────┴────┐                        │
│                     │ L.Ankle │       │ R.Ankle │                        │
│                     │ (2 DoF) │       │ (2 DoF) │                        │
│                     └────┬────┘       └────┬────┘                        │
│                          │                 │                              │
│                     ┌────┴────┐       ┌────┴────┐                        │
│                     │ L.Foot  │       │ R.Foot  │                        │
│                     │ (End)   │       │ (End)   │                        │
│                     └─────────┘       └─────────┘                        │
│                                                                          │
│     Total DoF: 20-40+ depending on hand complexity                       │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Degrees of Freedom by Joint

| Joint Group | Joint | DoF | Typical Axes | Range |
|-------------|-------|-----|--------------|-------|
| **Head** | Neck Pan | 1 | Yaw (Z) | ±90° |
| | Neck Tilt | 1 | Pitch (Y) | -45° to +30° |
| **Arm** | Shoulder Pitch | 1 | Y | -180° to +60° |
| | Shoulder Roll | 1 | X | -90° to +30° |
| | Shoulder Yaw | 1 | Z | ±90° |
| | Elbow Pitch | 1 | Y | 0° to +150° |
| | Elbow Yaw | 1 | Z | ±90° |
| | Wrist Pitch | 1 | Y | ±45° |
| | Wrist Roll | 1 | X | ±90° |
| **Leg** | Hip Yaw | 1 | Z | ±45° |
| | Hip Roll | 1 | X | ±45° |
| | Hip Pitch | 1 | Y | -90° to +30° |
| | Knee Pitch | 1 | Y | 0° to +150° |
| | Ankle Pitch | 1 | Y | -45° to +45° |
| | Ankle Roll | 1 | X | ±30° |

### 1.3 Humanoid Robot Model Class

```python
#!/usr/bin/env python3
"""
humanoid_model.py - Complete humanoid robot kinematic model
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from scipy.spatial.transform import Rotation as R


@dataclass
class JointInfo:
    """Joint information structure."""
    name: str
    parent: str
    child: str
    joint_type: str  # 'revolute', 'prismatic', 'fixed'
    axis: np.ndarray
    position: np.ndarray  # relative to parent
    orientation: np.ndarray  # quaternion [x, y, z, w]
    limits: Tuple[float, float]  # (lower, upper) in radians or meters
    velocity_limit: float
    effort_limit: float


@dataclass
class LinkInfo:
    """Link information structure."""
    name: str
    mass: float
    inertia: np.ndarray  # 3x3 matrix
    com_position: np.ndarray  # center of mass relative to link frame


class HumanoidModel:
    """Complete humanoid robot kinematic model."""

    def __init__(self, urdf_path: Optional[str] = None):
        self.joints: Dict[str, JointInfo] = {}
        self.links: Dict[str, LinkInfo] = {}
        self.joint_order: List[str] = []

        # Kinematic chains
        self.chains = {
            'left_arm': [],
            'right_arm': [],
            'left_leg': [],
            'right_leg': [],
            'head': [],
        }

        if urdf_path:
            self._load_urdf(urdf_path)
        else:
            self._create_default_model()

    def _create_default_model(self):
        """Create a default humanoid model (simplified)."""

        # Define link dimensions
        link_lengths = {
            'torso': 0.4,
            'upper_arm': 0.3,
            'lower_arm': 0.25,
            'upper_leg': 0.4,
            'lower_leg': 0.4,
            'foot': 0.15,
        }

        # Define link masses
        link_masses = {
            'torso': 15.0,
            'upper_arm': 2.0,
            'lower_arm': 1.5,
            'upper_leg': 5.0,
            'lower_leg': 3.0,
            'foot': 1.0,
        }

        # Create joints (simplified - left leg example)
        self.joints['left_hip_yaw'] = JointInfo(
            name='left_hip_yaw',
            parent='torso',
            child='left_hip_roll_link',
            joint_type='revolute',
            axis=np.array([0, 0, 1]),
            position=np.array([0, 0.1, -0.2]),  # relative to torso
            orientation=np.array([0, 0, 0, 1]),
            limits=(-0.785, 0.785),  # ±45°
            velocity_limit=6.0,
            effort_limit=150.0
        )

        self.joints['left_hip_roll'] = JointInfo(
            name='left_hip_roll',
            parent='left_hip_roll_link',
            child='left_hip_pitch_link',
            joint_type='revolute',
            axis=np.array([1, 0, 0]),
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1]),
            limits=(-0.785, 0.785),
            velocity_limit=6.0,
            effort_limit=150.0
        )

        self.joints['left_hip_pitch'] = JointInfo(
            name='left_hip_pitch',
            parent='left_hip_pitch_link',
            child='left_upper_leg',
            joint_type='revolute',
            axis=np.array([0, 1, 0]),
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1]),
            limits=(-1.57, 0.52),  # -90° to +30°
            velocity_limit=6.0,
            effort_limit=200.0
        )

        self.joints['left_knee_pitch'] = JointInfo(
            name='left_knee_pitch',
            parent='left_upper_leg',
            child='left_lower_leg',
            joint_type='revolute',
            axis=np.array([0, 1, 0]),
            position=np.array([0, 0, -link_lengths['upper_leg']]),
            orientation=np.array([0, 0, 0, 1]),
            limits=(0, 2.6),  # 0° to 150°
            velocity_limit=6.0,
            effort_limit=150.0
        )

        self.joints['left_ankle_pitch'] = JointInfo(
            name='left_ankle_pitch',
            parent='left_lower_leg',
            child='left_ankle_roll_link',
            joint_type='revolute',
            axis=np.array([0, 1, 0]),
            position=np.array([0, 0, -link_lengths['lower_leg']]),
            orientation=np.array([0, 0, 0, 1]),
            limits=(-0.785, 0.785),
            velocity_limit=6.0,
            effort_limit=100.0
        )

        self.joints['left_ankle_roll'] = JointInfo(
            name='left_ankle_roll',
            parent='left_ankle_roll_link',
            child='left_foot',
            joint_type='revolute',
            axis=np.array([1, 0, 0]),
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1]),
            limits=(-0.52, 0.52),  # ±30°
            velocity_limit=6.0,
            effort_limit=80.0
        )

        # Define kinematic chain
        self.chains['left_leg'] = [
            'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
            'left_knee_pitch', 'left_ankle_pitch', 'left_ankle_roll'
        ]

        # Set joint order
        self.joint_order = list(self.joints.keys())

    @property
    def num_joints(self) -> int:
        """Number of actuated joints."""
        return len([j for j in self.joints.values() if j.joint_type != 'fixed'])

    def get_joint_limits(self) -> np.ndarray:
        """Get joint limits as array of (lower, upper) pairs."""
        limits = []
        for name in self.joint_order:
            joint = self.joints[name]
            if joint.joint_type != 'fixed':
                limits.append(joint.limits)
        return np.array(limits)

    def get_chain_joints(self, chain_name: str) -> List[str]:
        """Get joint names for a kinematic chain."""
        return self.chains.get(chain_name, [])
```

---

## 2. Forward Kinematics

### 2.1 Homogeneous Transformations

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Homogeneous Transformation Matrix                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│     T = ┌                    ┐                                              │
│         │  R₃ₓ₃    p₃ₓ₁     │     R = Rotation matrix (3x3)                │
│         │                    │     p = Position vector (3x1)                │
│         │  0₁ₓ₃      1      │                                               │
│         └                    ┘                                              │
│                                                                              │
│     Example: Transform from frame A to frame B                              │
│                                                                              │
│         ᴬTᴮ = ┌                           ┐                                 │
│               │ cos(θ) -sin(θ)  0   x     │                                 │
│               │ sin(θ)  cos(θ)  0   y     │                                 │
│               │   0       0     1   z     │                                 │
│               │   0       0     0   1     │                                 │
│               └                           ┘                                 │
│                                                                              │
│     Chain rule: ⁰Tₙ = ⁰T₁ · ¹T₂ · ²T₃ · ... · ⁿ⁻¹Tₙ                        │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Forward Kinematics Implementation

```python
#!/usr/bin/env python3
"""
forward_kinematics.py - Forward kinematics computation
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Dict, List, Tuple


class ForwardKinematics:
    """Forward kinematics solver for humanoid robot."""

    def __init__(self, model: 'HumanoidModel'):
        self.model = model

    def rotation_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """Create rotation matrix from axis-angle representation."""
        return R.from_rotvec(axis * angle).as_matrix()

    def homogeneous_transform(
        self,
        rotation: np.ndarray,
        translation: np.ndarray
    ) -> np.ndarray:
        """Create 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T

    def joint_transform(self, joint: 'JointInfo', angle: float) -> np.ndarray:
        """Compute transformation for a single joint."""
        # Base transform (fixed offset)
        base_rot = R.from_quat(joint.orientation).as_matrix()
        T_base = self.homogeneous_transform(base_rot, joint.position)

        # Joint rotation
        if joint.joint_type == 'revolute':
            R_joint = self.rotation_matrix(joint.axis, angle)
            T_joint = self.homogeneous_transform(R_joint, np.zeros(3))
        elif joint.joint_type == 'prismatic':
            T_joint = self.homogeneous_transform(np.eye(3), joint.axis * angle)
        else:  # fixed
            T_joint = np.eye(4)

        return T_base @ T_joint

    def compute_chain(
        self,
        chain_name: str,
        joint_angles: Dict[str, float]
    ) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
        """Compute forward kinematics for a kinematic chain.

        Args:
            chain_name: Name of the chain ('left_leg', 'right_arm', etc.)
            joint_angles: Dictionary of joint angles

        Returns:
            end_effector_transform: 4x4 transform of end-effector
            intermediate_transforms: Dictionary of transforms for each link
        """
        chain_joints = self.model.get_chain_joints(chain_name)

        T_current = np.eye(4)  # Start at base frame
        transforms = {'base': T_current.copy()}

        for joint_name in chain_joints:
            joint = self.model.joints[joint_name]
            angle = joint_angles.get(joint_name, 0.0)

            # Compute joint transform
            T_joint = self.joint_transform(joint, angle)

            # Chain transforms
            T_current = T_current @ T_joint

            # Store intermediate transform
            transforms[joint.child] = T_current.copy()

        return T_current, transforms

    def compute_all_chains(
        self,
        joint_angles: Dict[str, float]
    ) -> Dict[str, np.ndarray]:
        """Compute forward kinematics for all chains."""
        results = {}
        for chain_name in self.model.chains.keys():
            T_end, _ = self.compute_chain(chain_name, joint_angles)
            results[chain_name] = T_end
        return results

    def get_end_effector_pose(
        self,
        chain_name: str,
        joint_angles: Dict[str, float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get end-effector position and orientation.

        Returns:
            position: (3,) array [x, y, z]
            quaternion: (4,) array [x, y, z, w]
        """
        T, _ = self.compute_chain(chain_name, joint_angles)
        position = T[:3, 3]
        quaternion = R.from_matrix(T[:3, :3]).as_quat()
        return position, quaternion


# Example usage
def forward_kinematics_example():
    """Demonstrate forward kinematics computation."""
    from humanoid_model import HumanoidModel

    # Create model
    model = HumanoidModel()
    fk = ForwardKinematics(model)

    # Set joint angles (in radians)
    joint_angles = {
        'left_hip_yaw': 0.0,
        'left_hip_roll': 0.0,
        'left_hip_pitch': -0.5,  # Hip flexed
        'left_knee_pitch': 1.0,  # Knee bent
        'left_ankle_pitch': 0.5,
        'left_ankle_roll': 0.0,
    }

    # Compute FK
    T_foot, transforms = fk.compute_chain('left_leg', joint_angles)

    # Extract position
    foot_position = T_foot[:3, 3]
    print(f"Left foot position: {foot_position}")

    # Get as pose
    pos, quat = fk.get_end_effector_pose('left_leg', joint_angles)
    print(f"Position: {pos}")
    print(f"Orientation (quat): {quat}")

    return T_foot


if __name__ == "__main__":
    forward_kinematics_example()
```

---

## 3. Inverse Kinematics

### 3.1 IK Problem Formulation

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Inverse Kinematics Problem                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Given:  Target pose (position + orientation)                              │
│   Find:   Joint angles that achieve target                                  │
│                                                                              │
│   ┌─────────────────────┐         ┌─────────────────────┐                  │
│   │  Target Position    │         │   Joint Angles      │                  │
│   │  xₜ, yₜ, zₜ         │  ────►  │   q₁, q₂, ..., qₙ   │                  │
│   │  + Orientation      │   IK    │                     │                  │
│   └─────────────────────┘         └─────────────────────┘                  │
│                                                                              │
│   Challenges:                                                               │
│   • Multiple solutions (redundancy)                                         │
│   • No solution (target unreachable)                                        │
│   • Singularities (infinite solutions)                                      │
│   • Joint limits                                                            │
│                                                                              │
│   Methods:                                                                   │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │  1. Analytical     - Closed-form for specific kinematic structures   │  │
│   │  2. Numerical      - Iterative optimization (Newton-Raphson)         │  │
│   │  3. Jacobian-based - Damped least squares, pseudo-inverse            │  │
│   │  4. Learning-based - Neural network approximation                     │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Numerical IK Solver

```python
#!/usr/bin/env python3
"""
inverse_kinematics.py - Inverse kinematics solvers for humanoid robots
"""

import numpy as np
from scipy.optimize import minimize, least_squares
from scipy.spatial.transform import Rotation as R
from typing import Optional, Tuple, Dict
import warnings


class InverseKinematics:
    """Inverse kinematics solver using numerical optimization."""

    def __init__(
        self,
        model: 'HumanoidModel',
        fk_solver: 'ForwardKinematics'
    ):
        self.model = model
        self.fk = fk_solver

        # Solver parameters
        self.position_weight = 1.0
        self.orientation_weight = 0.5
        self.regularization = 0.01

    def solve(
        self,
        chain_name: str,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        initial_guess: Optional[Dict[str, float]] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-6
    ) -> Tuple[Dict[str, float], bool]:
        """Solve IK using optimization.

        Args:
            chain_name: Name of kinematic chain
            target_position: Target [x, y, z] position
            target_orientation: Optional target quaternion [x, y, z, w]
            initial_guess: Initial joint angles (default: zeros)
            max_iterations: Maximum optimization iterations
            tolerance: Position error tolerance

        Returns:
            joint_angles: Dictionary of solved joint angles
            success: Whether IK succeeded
        """
        chain_joints = self.model.get_chain_joints(chain_name)
        n_joints = len(chain_joints)

        # Get joint limits
        limits = []
        for joint_name in chain_joints:
            joint = self.model.joints[joint_name]
            limits.append(joint.limits)
        bounds = list(zip(*limits))  # Separate lower and upper

        # Initial guess
        if initial_guess is None:
            q0 = np.zeros(n_joints)
        else:
            q0 = np.array([initial_guess.get(j, 0.0) for j in chain_joints])

        # Objective function
        def objective(q):
            # Convert to dictionary
            angles = {chain_joints[i]: q[i] for i in range(n_joints)}

            # Forward kinematics
            pos, quat = self.fk.get_end_effector_pose(chain_name, angles)

            # Position error
            pos_error = np.sum((pos - target_position) ** 2)

            # Orientation error
            if target_orientation is not None:
                # Quaternion error (1 - |q1 · q2|)
                ori_error = 1 - np.abs(np.dot(quat, target_orientation)) ** 2
                ori_error *= self.orientation_weight
            else:
                ori_error = 0.0

            # Regularization (prefer small joint angles)
            reg = self.regularization * np.sum(q ** 2)

            return self.position_weight * pos_error + ori_error + reg

        # Solve
        result = minimize(
            objective,
            q0,
            method='L-BFGS-B',
            bounds=list(zip(bounds[0], bounds[1])),
            options={'maxiter': max_iterations, 'ftol': tolerance}
        )

        # Convert result to dictionary
        solution = {chain_joints[i]: result.x[i] for i in range(n_joints)}

        # Check success
        final_pos, _ = self.fk.get_end_effector_pose(chain_name, solution)
        position_error = np.linalg.norm(final_pos - target_position)
        success = position_error < tolerance * 100  # Relaxed tolerance for success

        return solution, success


class DampedLeastSquaresIK:
    """Jacobian-based IK using damped least squares (Levenberg-Marquardt)."""

    def __init__(
        self,
        model: 'HumanoidModel',
        fk_solver: 'ForwardKinematics'
    ):
        self.model = model
        self.fk = fk_solver
        self.damping = 0.1

    def compute_jacobian(
        self,
        chain_name: str,
        joint_angles: Dict[str, float],
        delta: float = 1e-6
    ) -> np.ndarray:
        """Compute Jacobian numerically using finite differences."""
        chain_joints = self.model.get_chain_joints(chain_name)
        n_joints = len(chain_joints)

        # Current end-effector pose
        pos0, quat0 = self.fk.get_end_effector_pose(chain_name, joint_angles)

        # Initialize Jacobian (6 x n_joints: 3 position + 3 orientation)
        J = np.zeros((6, n_joints))

        for i, joint_name in enumerate(chain_joints):
            # Perturb joint angle
            angles_plus = joint_angles.copy()
            angles_plus[joint_name] += delta

            # Forward kinematics
            pos_plus, quat_plus = self.fk.get_end_effector_pose(
                chain_name, angles_plus
            )

            # Position Jacobian
            J[:3, i] = (pos_plus - pos0) / delta

            # Orientation Jacobian (using angular velocity representation)
            r0 = R.from_quat(quat0)
            r_plus = R.from_quat(quat_plus)
            r_diff = r_plus * r0.inv()
            J[3:6, i] = r_diff.as_rotvec() / delta

        return J

    def solve_step(
        self,
        chain_name: str,
        current_angles: Dict[str, float],
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None
    ) -> Dict[str, float]:
        """Compute one IK step using damped least squares."""
        chain_joints = self.model.get_chain_joints(chain_name)

        # Current pose
        pos, quat = self.fk.get_end_effector_pose(chain_name, current_angles)

        # Position error
        pos_error = target_position - pos

        # Orientation error
        if target_orientation is not None:
            r_current = R.from_quat(quat)
            r_target = R.from_quat(target_orientation)
            r_error = r_target * r_current.inv()
            ori_error = r_error.as_rotvec()
        else:
            ori_error = np.zeros(3)

        # Combined error
        error = np.concatenate([pos_error, ori_error])

        # Jacobian
        J = self.compute_jacobian(chain_name, current_angles)

        # Damped least squares solution
        # Δq = J^T (J J^T + λ²I)^-1 e
        JJT = J @ J.T
        damped = JJT + self.damping ** 2 * np.eye(6)
        delta_q = J.T @ np.linalg.solve(damped, error)

        # Apply update
        new_angles = {}
        for i, joint_name in enumerate(chain_joints):
            new_angles[joint_name] = current_angles[joint_name] + delta_q[i]

            # Enforce joint limits
            limits = self.model.joints[joint_name].limits
            new_angles[joint_name] = np.clip(
                new_angles[joint_name], limits[0], limits[1]
            )

        return new_angles

    def solve(
        self,
        chain_name: str,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        initial_guess: Optional[Dict[str, float]] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-4
    ) -> Tuple[Dict[str, float], bool]:
        """Iteratively solve IK using damped least squares."""
        chain_joints = self.model.get_chain_joints(chain_name)

        # Initialize
        if initial_guess is None:
            current = {j: 0.0 for j in chain_joints}
        else:
            current = initial_guess.copy()

        for iteration in range(max_iterations):
            # Compute IK step
            current = self.solve_step(
                chain_name, current, target_position, target_orientation
            )

            # Check convergence
            pos, _ = self.fk.get_end_effector_pose(chain_name, current)
            error = np.linalg.norm(pos - target_position)

            if error < tolerance:
                return current, True

        # Didn't converge
        return current, False


# Analytical IK for leg (6-DoF)
class AnalyticalLegIK:
    """Analytical IK for humanoid leg (assuming specific kinematic structure)."""

    def __init__(self, upper_leg_length: float, lower_leg_length: float):
        self.L1 = upper_leg_length
        self.L2 = lower_leg_length

    def solve(
        self,
        foot_position: np.ndarray,
        foot_orientation: Optional[np.ndarray] = None
    ) -> Tuple[Dict[str, float], bool]:
        """Solve leg IK analytically.

        Assumes: hip at origin, leg points down in neutral pose.
        """
        x, y, z = foot_position

        # Distance to target
        D = np.sqrt(x**2 + y**2 + z**2)

        # Check reachability
        if D > self.L1 + self.L2 or D < abs(self.L1 - self.L2):
            return {}, False

        # Knee angle (law of cosines)
        cos_knee = (self.L1**2 + self.L2**2 - D**2) / (2 * self.L1 * self.L2)
        cos_knee = np.clip(cos_knee, -1, 1)
        knee_pitch = np.pi - np.arccos(cos_knee)

        # Hip angles
        # Angle at hip in the leg plane
        cos_alpha = (self.L1**2 + D**2 - self.L2**2) / (2 * self.L1 * D)
        cos_alpha = np.clip(cos_alpha, -1, 1)
        alpha = np.arccos(cos_alpha)

        # Angle from vertical to target
        beta = np.arctan2(np.sqrt(x**2 + y**2), -z)

        hip_pitch = beta - alpha
        hip_roll = np.arctan2(y, -z)
        hip_yaw = np.arctan2(x, np.sqrt(y**2 + z**2))

        # Ankle (keep foot level by default)
        ankle_pitch = -(hip_pitch + knee_pitch)
        ankle_roll = -hip_roll

        solution = {
            'hip_yaw': hip_yaw,
            'hip_roll': hip_roll,
            'hip_pitch': hip_pitch,
            'knee_pitch': knee_pitch,
            'ankle_pitch': ankle_pitch,
            'ankle_roll': ankle_roll,
        }

        return solution, True
```

---

## 4. Jacobian and Velocity Control

### 4.1 The Manipulator Jacobian

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Manipulator Jacobian                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   The Jacobian J relates joint velocities to end-effector velocities:       │
│                                                                              │
│       ẋ = J(q) · q̇                                                          │
│                                                                              │
│   where:                                                                     │
│       ẋ = [vx, vy, vz, ωx, ωy, ωz]ᵀ   (6x1 twist vector)                   │
│       q̇ = [q̇₁, q̇₂, ..., q̇ₙ]ᵀ         (nx1 joint velocities)                │
│       J = 6×n matrix                                                        │
│                                                                              │
│   Jacobian structure:                                                       │
│                                                                              │
│       J = ┌                                                     ┐           │
│           │  ∂x/∂q₁   ∂x/∂q₂   ...   ∂x/∂qₙ   │  ← Linear     │           │
│           │  ∂y/∂q₁   ∂y/∂q₂   ...   ∂y/∂qₙ   │    velocity   │           │
│           │  ∂z/∂q₁   ∂z/∂q₂   ...   ∂z/∂qₙ   │               │           │
│           │  ────────────────────────────────  │               │           │
│           │  Jω₁      Jω₂      ...   Jωₙ      │  ← Angular    │           │
│           └                                     ┘    velocity   │           │
│                                                                              │
│   Uses:                                                                     │
│   • Resolved-rate motion control                                            │
│   • Singularity detection (det(J) → 0)                                      │
│   • Force/torque transformation: τ = Jᵀ F                                   │
│   • Manipulability analysis                                                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Resolved-Rate Motion Control

```python
#!/usr/bin/env python3
"""
resolved_rate_control.py - Velocity-level control using Jacobian
"""

import numpy as np
from typing import Dict, Optional


class ResolvedRateController:
    """Resolved-rate motion control using Jacobian pseudo-inverse."""

    def __init__(
        self,
        model: 'HumanoidModel',
        fk_solver: 'ForwardKinematics',
        ik_solver: 'DampedLeastSquaresIK'
    ):
        self.model = model
        self.fk = fk_solver
        self.ik = ik_solver

        # Control parameters
        self.damping = 0.05
        self.max_joint_velocity = 2.0  # rad/s

    def compute_joint_velocities(
        self,
        chain_name: str,
        current_angles: Dict[str, float],
        target_twist: np.ndarray
    ) -> np.ndarray:
        """Compute joint velocities for desired end-effector twist.

        Args:
            chain_name: Kinematic chain name
            current_angles: Current joint angles
            target_twist: Desired [vx, vy, vz, ωx, ωy, ωz] twist

        Returns:
            Joint velocities for each joint in chain
        """
        # Compute Jacobian
        J = self.ik.compute_jacobian(chain_name, current_angles)

        # Damped pseudo-inverse
        JT = J.T
        JJT = J @ JT
        damped = JJT + self.damping ** 2 * np.eye(6)
        J_pinv = JT @ np.linalg.inv(damped)

        # Compute joint velocities
        q_dot = J_pinv @ target_twist

        # Limit velocities
        q_dot = np.clip(q_dot, -self.max_joint_velocity, self.max_joint_velocity)

        return q_dot

    def track_trajectory(
        self,
        chain_name: str,
        current_angles: Dict[str, float],
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        dt: float = 0.01,
        gain: float = 5.0
    ) -> Dict[str, float]:
        """Track target pose using resolved-rate control.

        Args:
            chain_name: Kinematic chain
            current_angles: Current joint configuration
            target_position: Target position [x, y, z]
            target_orientation: Optional target orientation (quaternion)
            dt: Control timestep
            gain: Proportional gain

        Returns:
            New joint angles after one control step
        """
        chain_joints = self.model.get_chain_joints(chain_name)

        # Current pose
        current_pos, current_quat = self.fk.get_end_effector_pose(
            chain_name, current_angles
        )

        # Position error → desired velocity
        pos_error = target_position - current_pos
        desired_linear_vel = gain * pos_error

        # Orientation error → desired angular velocity
        if target_orientation is not None:
            from scipy.spatial.transform import Rotation as R
            r_current = R.from_quat(current_quat)
            r_target = R.from_quat(target_orientation)
            r_error = r_target * r_current.inv()
            desired_angular_vel = gain * r_error.as_rotvec()
        else:
            desired_angular_vel = np.zeros(3)

        # Combine into twist
        desired_twist = np.concatenate([desired_linear_vel, desired_angular_vel])

        # Compute joint velocities
        q_dot = self.compute_joint_velocities(
            chain_name, current_angles, desired_twist
        )

        # Integrate
        new_angles = {}
        for i, joint_name in enumerate(chain_joints):
            new_angles[joint_name] = current_angles[joint_name] + q_dot[i] * dt

            # Enforce limits
            limits = self.model.joints[joint_name].limits
            new_angles[joint_name] = np.clip(
                new_angles[joint_name], limits[0], limits[1]
            )

        return new_angles


class NullSpaceController:
    """Control with null-space optimization for redundant robots."""

    def __init__(
        self,
        model: 'HumanoidModel',
        fk_solver: 'ForwardKinematics'
    ):
        self.model = model
        self.fk = fk_solver
        self.damping = 0.05

    def compute_with_null_space(
        self,
        chain_name: str,
        current_angles: Dict[str, float],
        primary_task: np.ndarray,
        secondary_task: np.ndarray,
        J: np.ndarray
    ) -> np.ndarray:
        """Compute joint velocities with null-space optimization.

        Args:
            chain_name: Kinematic chain
            current_angles: Current configuration
            primary_task: Primary task velocity (e.g., hand position)
            secondary_task: Secondary task (e.g., elbow position, posture)
            J: Jacobian for primary task

        Returns:
            Joint velocities satisfying both tasks when possible
        """
        n_joints = J.shape[1]

        # Primary task solution
        J_pinv = np.linalg.pinv(J)
        q_dot_primary = J_pinv @ primary_task

        # Null space projector
        N = np.eye(n_joints) - J_pinv @ J

        # Secondary task in null space
        q_dot_secondary = N @ secondary_task

        # Combined solution
        q_dot = q_dot_primary + q_dot_secondary

        return q_dot
```

---

## 5. Center of Mass and Balance

### 5.1 Center of Mass Computation

```python
#!/usr/bin/env python3
"""
center_of_mass.py - CoM computation and balance control
"""

import numpy as np
from typing import Dict, List, Tuple


class CenterOfMass:
    """Center of mass computation for humanoid robot."""

    def __init__(self, model: 'HumanoidModel', fk_solver: 'ForwardKinematics'):
        self.model = model
        self.fk = fk_solver

    def compute_link_com(
        self,
        link_name: str,
        joint_angles: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """Compute world position of link's center of mass.

        Returns:
            position: CoM position in world frame
            mass: Link mass
        """
        # Get link transform (need to implement full FK to each link)
        # This is a simplified version
        link = self.model.links.get(link_name)
        if link is None:
            return np.zeros(3), 0.0

        # Get link frame transform
        T_link = self._get_link_transform(link_name, joint_angles)

        # Transform local CoM to world frame
        com_local = np.append(link.com_position, 1)
        com_world = T_link @ com_local

        return com_world[:3], link.mass

    def compute_total_com(
        self,
        joint_angles: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """Compute total center of mass of the robot.

        Returns:
            com: Total CoM position [x, y, z]
            total_mass: Sum of all link masses
        """
        total_com = np.zeros(3)
        total_mass = 0.0

        for link_name, link in self.model.links.items():
            com, mass = self.compute_link_com(link_name, joint_angles)
            total_com += mass * com
            total_mass += mass

        if total_mass > 0:
            total_com /= total_mass

        return total_com, total_mass

    def compute_com_jacobian(
        self,
        joint_angles: Dict[str, float],
        delta: float = 1e-6
    ) -> np.ndarray:
        """Compute Jacobian of CoM with respect to joint angles.

        The CoM Jacobian relates joint velocities to CoM velocity:
            ẋ_com = J_com · q̇
        """
        joint_names = list(joint_angles.keys())
        n_joints = len(joint_names)

        # Current CoM
        com0, _ = self.compute_total_com(joint_angles)

        # Compute Jacobian
        J_com = np.zeros((3, n_joints))

        for i, joint_name in enumerate(joint_names):
            angles_plus = joint_angles.copy()
            angles_plus[joint_name] += delta

            com_plus, _ = self.compute_total_com(angles_plus)
            J_com[:, i] = (com_plus - com0) / delta

        return J_com

    def _get_link_transform(
        self,
        link_name: str,
        joint_angles: Dict[str, float]
    ) -> np.ndarray:
        """Get transform to a link frame (simplified implementation)."""
        # This would trace back through the kinematic tree
        # Placeholder returning identity
        return np.eye(4)


class BalanceController:
    """Balance controller using CoM and Zero Moment Point (ZMP)."""

    def __init__(
        self,
        model: 'HumanoidModel',
        fk_solver: 'ForwardKinematics',
        com_solver: 'CenterOfMass'
    ):
        self.model = model
        self.fk = fk_solver
        self.com = com_solver

        # Control gains
        self.Kp = 100.0  # Position gain
        self.Kd = 20.0   # Damping gain

        # Robot parameters
        self.com_height = 0.9  # Approximate CoM height
        self.gravity = 9.81

    def compute_zmp(
        self,
        com_position: np.ndarray,
        com_acceleration: np.ndarray
    ) -> np.ndarray:
        """Compute Zero Moment Point from CoM dynamics.

        The ZMP is the point where the total moment equals zero:
            p_zmp = p_com - (h / g) * ẍ_com

        Args:
            com_position: Current CoM position [x, y, z]
            com_acceleration: CoM acceleration [ax, ay, az]

        Returns:
            zmp: Zero Moment Point [x, y] on ground
        """
        h = com_position[2]  # Height of CoM
        g = self.gravity

        zmp_x = com_position[0] - (h / g) * com_acceleration[0]
        zmp_y = com_position[1] - (h / g) * com_acceleration[1]

        return np.array([zmp_x, zmp_y])

    def is_balanced(
        self,
        zmp: np.ndarray,
        support_polygon: np.ndarray
    ) -> bool:
        """Check if ZMP is within support polygon.

        Args:
            zmp: ZMP position [x, y]
            support_polygon: Nx2 array of polygon vertices

        Returns:
            True if ZMP is inside support polygon
        """
        from matplotlib.path import Path
        polygon = Path(support_polygon)
        return polygon.contains_point(zmp)

    def compute_support_polygon(
        self,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
        foot_width: float = 0.08,
        foot_length: float = 0.15,
        stance: str = 'double'
    ) -> np.ndarray:
        """Compute support polygon from foot positions.

        Args:
            left_foot_pos: Left foot position [x, y]
            right_foot_pos: Right foot position [x, y]
            foot_width: Foot width
            foot_length: Foot length
            stance: 'double', 'left', or 'right'

        Returns:
            Vertices of support polygon
        """
        def foot_polygon(pos):
            """Create rectangular foot polygon."""
            x, y = pos[:2]
            return np.array([
                [x - foot_length/2, y - foot_width/2],
                [x + foot_length/2, y - foot_width/2],
                [x + foot_length/2, y + foot_width/2],
                [x - foot_length/2, y + foot_width/2],
            ])

        if stance == 'left':
            return foot_polygon(left_foot_pos)
        elif stance == 'right':
            return foot_polygon(right_foot_pos)
        else:  # double support
            # Convex hull of both feet
            left_poly = foot_polygon(left_foot_pos)
            right_poly = foot_polygon(right_foot_pos)
            all_points = np.vstack([left_poly, right_poly])

            from scipy.spatial import ConvexHull
            hull = ConvexHull(all_points)
            return all_points[hull.vertices]

    def balance_control(
        self,
        joint_angles: Dict[str, float],
        target_com: np.ndarray,
        com_velocity: np.ndarray
    ) -> np.ndarray:
        """Compute joint torques to maintain balance.

        Uses PD control on CoM position.

        Args:
            joint_angles: Current joint configuration
            target_com: Desired CoM position
            com_velocity: Current CoM velocity

        Returns:
            Joint torques
        """
        # Current CoM
        current_com, _ = self.com.compute_total_com(joint_angles)

        # Error
        error = target_com - current_com

        # PD control
        com_force = self.Kp * error - self.Kd * com_velocity

        # CoM Jacobian
        J_com = self.com.compute_com_jacobian(joint_angles)

        # Convert to joint torques: τ = Jᵀ F
        torques = J_com.T @ com_force

        return torques
```

---

## 6. Summary

### Kinematics Methods Comparison

| Method | Pros | Cons | Use Case |
|--------|------|------|----------|
| **Analytical IK** | Fast, exact | Structure-specific | Known kinematic chains |
| **Numerical IK** | General | Slower, may not converge | Arbitrary robots |
| **Jacobian-based** | Continuous, smooth | Singularities | Real-time control |
| **Learning-based** | Fast inference | Training required | High-DoF systems |

### Key Equations

| Concept | Equation | Description |
|---------|----------|-------------|
| **Forward Kinematics** | x = f(q) | Joint angles → end-effector |
| **Jacobian** | ẋ = J(q)q̇ | Velocity mapping |
| **Inverse Kinematics** | q = f⁻¹(x) | End-effector → joint angles |
| **ZMP** | p_zmp = p_com - (h/g)ẍ_com | Balance criterion |

---

## Exercises

### Exercise 11.1: Forward Kinematics (⭐⭐)

1. Implement forward kinematics for a 6-DoF leg
2. Verify by computing foot position at different joint angles
3. Visualize the workspace (reachable positions)

### Exercise 11.2: Inverse Kinematics (⭐⭐⭐)

1. Implement numerical IK using damped least squares
2. Test with multiple target positions
3. Handle unreachable targets gracefully
4. Compare with analytical solution (if available)

### Exercise 11.3: Jacobian Control (⭐⭐⭐)

1. Compute the Jacobian for your robot's arm
2. Implement resolved-rate motion control
3. Track a circular trajectory with the hand
4. Detect and handle singularities

### Exercise 11.4: Balance Analysis (⭐⭐)

1. Compute CoM for different postures
2. Implement support polygon computation
3. Verify ZMP stays within support polygon during motion
4. Visualize stability margin

---

## Quiz

<details>
<summary>Q1: What is the difference between forward and inverse kinematics?</summary>

**Forward Kinematics (FK)**: Given joint angles, compute end-effector pose
- Unique solution
- Computationally simple
- Direct chain of transformations

**Inverse Kinematics (IK)**: Given end-effector pose, compute joint angles
- May have multiple solutions (redundancy)
- May have no solution (unreachable)
- May be singular (infinite solutions)
- Computationally more complex

</details>

<details>
<summary>Q2: Why is the Jacobian important for robot control?</summary>

The Jacobian is critical because it:
1. Maps joint velocities to end-effector velocities
2. Enables velocity-level control (resolved-rate)
3. Maps end-effector forces to joint torques (transpose)
4. Reveals singularities (det(J) → 0)
5. Measures manipulability (condition number)
6. Enables null-space optimization for redundant robots

</details>

<details>
<summary>Q3: What is the Zero Moment Point (ZMP) and why is it important?</summary>

The ZMP is the point on the ground where the total moment of inertial and gravity forces equals zero:

p_zmp = p_com - (h/g) * ẍ_com

It's important because:
1. Stability criterion: ZMP must stay within support polygon
2. Enables walking pattern generation
3. Simpler than full dynamics (3D → 2D)
4. Used by most biped walking controllers

</details>

<details>
<summary>Q4: How do you handle singularities in Jacobian-based control?</summary>

Singularity handling approaches:
1. **Damped Least Squares**: Add λ²I to JJᵀ before inversion
2. **Singularity-Robust Inverse**: Reduce motion near singularity
3. **Task-Space Limiting**: Avoid commanding motion toward singularity
4. **Null-Space Optimization**: Use redundancy to move away
5. **Trajectory Planning**: Plan paths that avoid singular configurations

</details>

---

## Next Steps

In [Week 12: Locomotion and Manipulation](/module-4-vla/week-12-locomotion-manipulation), we explore:
- Bipedal walking algorithms
- Grasp planning and execution
- Whole-body coordination
- Nav2 integration for humanoid navigation
