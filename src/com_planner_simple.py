"""
Simplified CoM Trajectory Planner for Bipedal Walking

Uses a PD-based approach with preview for ZMP tracking.
Simpler and more stable than full preview control LQR.

Author: Phase 4.1 Implementation (Simplified)
Date: 2025-11-25
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class SimpleCoMPlannerParams:
    """Parameters for Simple CoM Planner"""

    # Physical parameters
    com_height: float = 0.689  # meters
    gravity: float = 9.81  # m/s^2

    # Control parameters
    zmp_kp: float = 10.0  # Proportional gain for ZMP tracking (increased)
    zmp_kd: float = 3.0  # Derivative gain for ZMP tracking (increased)
    preview_time: float = 0.5  # seconds (how far ahead to look)
    dt: float = 0.01  # seconds (planning timestep)

    # Damping
    velocity_damping: float = 0.95  # Damping factor (0-1, higher = less damping)

    def __post_init__(self):
        """Compute derived parameters"""
        self.omega = np.sqrt(self.gravity / self.com_height)
        self.preview_steps = int(self.preview_time / self.dt)


class SimpleCoMPlanner:
    """
    Simple CoM Planner using PD control with preview

    Key idea:
    - Desired CoM acceleration comes from ZMP error
    - Preview future ZMP to anticipate changes
    - Use LIPM to convert CoM to ZMP: p = x - ẍ/ω²
    """

    def __init__(self, params: Optional[SimpleCoMPlannerParams] = None):
        self.params = params or SimpleCoMPlannerParams()

        # State: [position, velocity]
        self.com_pos = 0.0
        self.com_vel = 0.0

        print(f"[Simple CoM Planner] Initialized")
        print(f"  CoM height: {self.params.com_height}m")
        print(f"  Omega: {self.params.omega:.3f} rad/s")
        print(f"  ZMP gains: Kp={self.params.zmp_kp}, Kd={self.params.zmp_kd}")
        print(f"  Preview time: {self.params.preview_time}s")

    def reset(self, initial_pos: float = 0.0, initial_vel: float = 0.0):
        """Reset planner state"""
        self.com_pos = initial_pos
        self.com_vel = initial_vel

    def compute_com_command(self,
                           current_zmp_ref: float,
                           future_zmp_ref: Optional[np.ndarray] = None) -> Tuple[float, float, float]:
        """
        Compute CoM position, velocity, acceleration

        Args:
            current_zmp_ref: Current desired ZMP (m)
            future_zmp_ref: Optional array of future ZMP positions for preview

        Returns:
            (position, velocity, acceleration)
        """
        omega2 = self.params.omega ** 2

        # Current ZMP from LIPM: p = x - ẍ/ω²
        # We need to solve for ẍ given desired p
        # Rearrange: ẍ = ω²(x - p_desired)

        # Basic PD control on ZMP error
        zmp_error = self.com_pos - current_zmp_ref

        # Preview: if we have future ZMP, adjust for upcoming changes
        preview_correction = 0.0
        if future_zmp_ref is not None and len(future_zmp_ref) > 0:
            # Weight future ZMP values (closer = more important)
            weights = np.exp(-np.arange(len(future_zmp_ref)) * 0.5)
            weights /= weights.sum()
            preview_target = np.dot(weights, future_zmp_ref)
            preview_correction = self.params.zmp_kp * 0.3 * (preview_target - current_zmp_ref)

        # Desired acceleration to track ZMP
        desired_accel = -(self.params.zmp_kp * zmp_error +
                         self.params.zmp_kd * self.com_vel)
        desired_accel += preview_correction

        # Integrate to get next state
        dt = self.params.dt
        self.com_vel = self.com_vel * self.params.velocity_damping + desired_accel * dt
        self.com_pos = self.com_pos + self.com_vel * dt

        return self.com_pos, self.com_vel, desired_accel

    def plan_trajectory(self,
                       zmp_reference: np.ndarray,
                       initial_state: Optional[Tuple[float, float]] = None) -> np.ndarray:
        """
        Plan complete CoM trajectory

        Args:
            zmp_reference: Array of desired ZMP positions
            initial_state: Optional (position, velocity)

        Returns:
            Array of shape (N, 3) with [position, velocity, acceleration]
        """
        N = len(zmp_reference)
        trajectory = np.zeros((N, 3))

        if initial_state is not None:
            self.reset(initial_state[0], initial_state[1])

        for i in range(N):
            # Get preview window
            preview_start = min(i + 1, N)
            preview_end = min(i + 1 + self.params.preview_steps, N)
            future_zmp = zmp_reference[preview_start:preview_end] if preview_end > preview_start else None

            # Compute CoM
            pos, vel, acc = self.compute_com_command(zmp_reference[i], future_zmp)
            trajectory[i] = [pos, vel, acc]

        return trajectory


class SimpleCoMPlanner2D:
    """2D CoM Planner (X and Y directions)"""

    def __init__(self, params: Optional[SimpleCoMPlannerParams] = None):
        self.params = params or SimpleCoMPlannerParams()
        self.planner_x = SimpleCoMPlanner(self.params)
        self.planner_y = SimpleCoMPlanner(self.params)

        print(f"[Simple CoM Planner 2D] Initialized for X and Y")

    def reset(self, initial_com: np.ndarray, initial_vel: np.ndarray):
        """Reset both planners"""
        self.planner_x.reset(initial_com[0], initial_vel[0])
        self.planner_y.reset(initial_com[1], initial_vel[1])

    def compute_com_command(self,
                           zmp_ref: np.ndarray,
                           future_zmp_x: Optional[np.ndarray] = None,
                           future_zmp_y: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute 2D CoM command

        Args:
            zmp_ref: [x, y] current ZMP reference
            future_zmp_x: Optional future X ZMP values
            future_zmp_y: Optional future Y ZMP values

        Returns:
            (position, velocity, acceleration) as [x, y] arrays
        """
        pos_x, vel_x, acc_x = self.planner_x.compute_com_command(zmp_ref[0], future_zmp_x)
        pos_y, vel_y, acc_y = self.planner_y.compute_com_command(zmp_ref[1], future_zmp_y)

        return (
            np.array([pos_x, pos_y]),
            np.array([vel_x, vel_y]),
            np.array([acc_x, acc_y])
        )

    def plan_trajectory(self,
                       zmp_reference: np.ndarray,
                       initial_state: Optional[Tuple[np.ndarray, np.ndarray]] = None) -> np.ndarray:
        """
        Plan 2D CoM trajectory

        Args:
            zmp_reference: (N, 2) array of [x, y] ZMP positions
            initial_state: Optional ([x,y] position, [vx,vy] velocity)

        Returns:
            (N, 2, 3) array with [position, velocity, acceleration] for [x, y]
        """
        N = len(zmp_reference)
        trajectory = np.zeros((N, 2, 3))

        if initial_state is not None:
            self.reset(initial_state[0], initial_state[1])

        traj_x = self.planner_x.plan_trajectory(zmp_reference[:, 0])
        traj_y = self.planner_y.plan_trajectory(zmp_reference[:, 1])

        trajectory[:, 0, :] = traj_x
        trajectory[:, 1, :] = traj_y

        return trajectory
