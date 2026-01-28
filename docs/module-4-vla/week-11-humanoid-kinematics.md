---
sidebar_position: 1
---

# Week 11: Humanoid Kinematics

## Learning Objectives

By the end of this week, you will be able to:

- Understand humanoid robot kinematics
- Implement forward and inverse kinematics
- Model humanoid joint structure
- Control humanoid posture and balance

## Humanoid Skeleton Structure

```
                    ┌───────┐
                    │ Head  │
                    └───┬───┘
                        │
           ┌────────────┼────────────┐
           │            │            │
      ┌────┴────┐  ┌────┴────┐  ┌────┴────┐
      │L.Shoulder│  │  Torso  │  │R.Shoulder│
      └────┬────┘  └────┬────┘  └────┬────┘
           │            │            │
      ┌────┴────┐       │       ┌────┴────┐
      │ L.Elbow │       │       │ R.Elbow │
      └────┬────┘       │       └────┬────┘
           │            │            │
      ┌────┴────┐       │       ┌────┴────┐
      │ L.Wrist │       │       │ R.Wrist │
      └─────────┘       │       └─────────┘
                        │
                   ┌────┴────┐
                   │  Pelvis  │
                   └────┬────┘
              ┌─────────┼─────────┐
              │                   │
         ┌────┴────┐         ┌────┴────┐
         │ L.Hip   │         │ R.Hip   │
         └────┬────┘         └────┬────┘
              │                   │
         ┌────┴────┐         ┌────┴────┐
         │ L.Knee  │         │ R.Knee  │
         └────┬────┘         └────┬────┘
              │                   │
         ┌────┴────┐         ┌────┴────┐
         │ L.Ankle │         │ R.Ankle │
         └─────────┘         └─────────┘
```

## Degrees of Freedom

| Joint | Typical DoF | Axes |
|-------|-------------|------|
| **Neck** | 2-3 | Pitch, Yaw, (Roll) |
| **Shoulder** | 3 | Pitch, Roll, Yaw |
| **Elbow** | 1-2 | Pitch, (Yaw) |
| **Wrist** | 2-3 | Pitch, Yaw, (Roll) |
| **Hip** | 3 | Pitch, Roll, Yaw |
| **Knee** | 1 | Pitch |
| **Ankle** | 2 | Pitch, Roll |

**Total**: 20-30+ DoF for a typical humanoid

## Forward Kinematics

Given joint angles, compute end-effector position:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def forward_kinematics(joint_angles, link_lengths):
    """
    Compute end-effector position from joint angles.

    Args:
        joint_angles: List of joint angles [q1, q2, ..., qn]
        link_lengths: List of link lengths [L1, L2, ..., Ln]

    Returns:
        position: (x, y, z) end-effector position
        orientation: Rotation matrix
    """
    T = np.eye(4)  # Start with identity transform

    for i, (angle, length) in enumerate(zip(joint_angles, link_lengths)):
        # Rotation about z-axis
        Rz = R.from_euler('z', angle).as_matrix()

        # Translation along x-axis
        trans = np.array([length, 0, 0])

        # Build transform
        Ti = np.eye(4)
        Ti[:3, :3] = Rz
        Ti[:3, 3] = trans

        T = T @ Ti

    position = T[:3, 3]
    orientation = T[:3, :3]

    return position, orientation
```

## Inverse Kinematics

Given desired end-effector position, compute joint angles:

```python
import numpy as np
from scipy.optimize import minimize

class InverseKinematics:
    def __init__(self, robot_model):
        self.robot = robot_model

    def solve(self, target_position, target_orientation=None):
        """
        Solve IK using numerical optimization.
        """
        def objective(q):
            # Forward kinematics
            pos, ori = self.robot.forward_kinematics(q)

            # Position error
            pos_error = np.linalg.norm(pos - target_position)

            # Orientation error (if specified)
            ori_error = 0
            if target_orientation is not None:
                ori_error = np.linalg.norm(ori - target_orientation)

            return pos_error + 0.1 * ori_error

        # Initial guess
        q0 = np.zeros(self.robot.num_joints)

        # Solve with bounds
        result = minimize(
            objective,
            q0,
            method='SLSQP',
            bounds=self.robot.joint_limits
        )

        return result.x
```

## Jacobian-Based Control

The Jacobian relates joint velocities to end-effector velocities:

```python
def compute_jacobian(robot, q, delta=1e-6):
    """
    Compute Jacobian numerically.
    """
    n_joints = len(q)
    J = np.zeros((6, n_joints))  # 6 for position + orientation

    pos0, _ = robot.forward_kinematics(q)

    for i in range(n_joints):
        q_plus = q.copy()
        q_plus[i] += delta

        pos_plus, _ = robot.forward_kinematics(q_plus)

        J[:3, i] = (pos_plus - pos0) / delta

    return J

def resolved_rate_control(robot, q, target_velocity, dt):
    """
    Move end-effector at desired velocity using Jacobian.
    """
    J = compute_jacobian(robot, q)

    # Pseudo-inverse for redundant robots
    J_pinv = np.linalg.pinv(J)

    # Joint velocities
    q_dot = J_pinv @ target_velocity

    # Integrate
    q_new = q + q_dot * dt

    return q_new
```

## Balance Control

Humanoid balance requires controlling the **Center of Mass (CoM)**:

```python
class BalanceController:
    def __init__(self, robot):
        self.robot = robot
        self.Kp = 100.0  # Proportional gain
        self.Kd = 20.0   # Derivative gain

    def compute_com(self, q):
        """Compute Center of Mass position."""
        total_mass = 0
        com = np.zeros(3)

        for link in self.robot.links:
            mass = link.mass
            pos = link.get_world_position(q)
            com += mass * pos
            total_mass += mass

        return com / total_mass

    def compute_zmp(self, com, com_ddot):
        """Compute Zero Moment Point."""
        g = 9.81
        z_com = com[2]

        zmp_x = com[0] - (z_com / g) * com_ddot[0]
        zmp_y = com[1] - (z_com / g) * com_ddot[1]

        return np.array([zmp_x, zmp_y])

    def balance(self, q, target_com):
        """PD control to maintain balance."""
        com = self.compute_com(q)
        error = target_com - com

        # Compute torques
        tau = self.Kp * error - self.Kd * self.robot.get_com_velocity()

        return tau
```

## Exercises

1. Implement forward kinematics for a 3-link arm
2. Solve inverse kinematics for reaching a target
3. Compute the Jacobian for your robot model
4. Implement a simple balance controller

## Next Steps

In [Week 12](/module-4-vla/week-12-locomotion-manipulation), we explore locomotion and manipulation.
