"""
Task Hierarchy for Whole-Body Control

Implements prioritized task structure:
Priority 0 (Highest): Contact constraints
Priority 1: Body orientation/position
Priority 2: Joint regularization
"""

import numpy as np
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class Task:
    """
    Single task in WBC hierarchy

    Each task has:
    - Desired acceleration
    - Task Jacobian
    - Priority level
    - Weight
    """
    name: str
    priority: int
    weight: float
    jacobian: np.ndarray  # Task Jacobian
    desired_accel: np.ndarray  # Desired task-space acceleration


class TaskHierarchy:
    """
    Manages prioritized tasks for WBC

    Tasks are solved in priority order using nullspace projection
    """

    def __init__(self):
        self.tasks: List[Task] = []

    def add_task(self, task: Task):
        """Add task to hierarchy"""
        self.tasks.append(task)
        # Sort by priority (lower number = higher priority)
        self.tasks.sort(key=lambda t: t.priority)

    def clear_tasks(self):
        """Clear all tasks"""
        self.tasks = []

    def get_desired_acceleration(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute desired acceleration considering all tasks

        Returns:
            base_accel: Desired base acceleration [6]
            joint_accel: Desired joint acceleration [n_joints]
        """
        if not self.tasks:
            return np.zeros(6), np.zeros(0)

        # For simplicity, use weighted sum of tasks
        # Full implementation would use nullspace projection

        total_weight = sum(t.weight for t in self.tasks)
        base_accel = np.zeros(6)

        for task in self.tasks:
            if task.jacobian.shape[1] == 6:  # Base task
                base_accel += task.weight / total_weight * task.desired_accel

        # Joint acceleration (placeholder)
        joint_accel = np.zeros(0)

        return base_accel, joint_accel


def create_body_orientation_task(
    current_orientation: np.ndarray,
    desired_orientation: np.ndarray,
    current_angular_vel: np.ndarray,
    kp: float = 50.0,
    kd: float = 5.0
) -> Task:
    """
    Create task for body orientation control

    Args:
        current_orientation: Current orientation (quaternion)
        desired_orientation: Desired orientation (quaternion)
        current_angular_vel: Current angular velocity
        kp: Proportional gain
        kd: Derivative gain

    Returns:
        Task for orientation control
    """
    # Compute orientation error (simplified - should use proper quaternion math)
    import pybullet as p

    current_euler = p.getEulerFromQuaternion(current_orientation)
    desired_euler = p.getEulerFromQuaternion(desired_orientation)

    orientation_error = np.array(desired_euler) - np.array(current_euler)

    # PD control for desired angular acceleration
    desired_angular_accel = kp * orientation_error - kd * current_angular_vel

    # Task Jacobian (full 6D base, but only affects angular part)
    jacobian = np.zeros((6, 6))
    jacobian[3:6, 3:6] = np.eye(3)

    # Desired acceleration in 6D base coordinates (linear=0, angular=desired)
    desired_accel_6d = np.zeros(6)
    desired_accel_6d[3:6] = desired_angular_accel

    return Task(
        name="body_orientation",
        priority=1,
        weight=10.0,
        jacobian=jacobian,
        desired_accel=desired_accel_6d
    )


def create_body_position_task(
    current_position: np.ndarray,
    desired_position: np.ndarray,
    current_velocity: np.ndarray,
    kp: float = 100.0,
    kd: float = 10.0
) -> Task:
    """
    Create task for body position control (CoM height, etc.)

    Args:
        current_position: Current CoM position
        desired_position: Desired CoM position
        current_velocity: Current CoM velocity
        kp: Proportional gain
        kd: Derivative gain

    Returns:
        Task for position control
    """
    position_error = desired_position - current_position

    # PD control
    desired_accel = kp * position_error - kd * current_velocity

    # Task Jacobian (linear part of base)
    jacobian = np.zeros((3, 6))
    jacobian[0:3, 0:3] = np.eye(3)

    return Task(
        name="body_position",
        priority=1,
        weight=10.0,
        jacobian=jacobian,
        desired_accel=desired_accel
    )


def create_com_tracking_task(
    com_offset: np.ndarray,
    com_velocity: np.ndarray,
    kp: float = 50.0,
    kd: float = 5.0
) -> Task:
    """
    Create task for CoM tracking (keep CoM above support polygon)

    Args:
        com_offset: CoM offset from desired position
        com_velocity: CoM velocity
        kp: Proportional gain
        kd: Derivative gain

    Returns:
        Task for CoM tracking
    """
    # Desired acceleration to bring CoM back to center
    desired_accel = -kp * com_offset - kd * com_velocity

    # Only care about horizontal position
    jacobian = np.zeros((2, 6))
    jacobian[0:2, 0:2] = np.eye(2)

    # Append zero for vertical component to make it 6D
    desired_accel_full = np.zeros(6)
    desired_accel_full[0:2] = desired_accel

    return Task(
        name="com_tracking",
        priority=1,
        weight=5.0,
        jacobian=np.eye(6),  # Use full base Jacobian
        desired_accel=desired_accel_full
    )


if __name__ == "__main__":
    print("WBC Task Hierarchy Module")
    print("\nAvailable task types:")
    print("1. Body orientation control")
    print("2. Body position control")
    print("3. CoM tracking")
    print("\nUsage: Create tasks and add to TaskHierarchy")
