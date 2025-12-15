"""
CoM Trajectory Planner for Bipedal Walking

Implements Preview Control for ZMP-based CoM trajectory generation.
Based on Kajita et al. (2003) - "Biped Walking Pattern Generation by Preview Control
of Zero-Moment Point"

Key Concept:
- Linear Inverted Pendulum Model (LIPM): ẍ = (g/h) * (x - p)
  where x is CoM position, p is ZMP position, h is CoM height, g is gravity
- Preview control: Plan CoM trajectory that produces desired ZMP trajectory
- Ensures ZMP stays inside support polygon for dynamic stability

Author: Phase 4.1 Implementation
Date: 2025-11-25
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
import scipy.linalg


@dataclass
class CoMPlannerParams:
    """Parameters for Preview Control CoM Planner"""

    # Physical parameters
    com_height: float = 0.689  # meters (from Phase 2 standing height)
    gravity: float = 9.81  # m/s^2

    # Preview control parameters
    preview_horizon: float = 1.0  # seconds (how far ahead to look)
    dt: float = 0.01  # seconds (planning timestep, 100 Hz)

    # Control weights
    Q_e: float = 1.0  # ZMP error weight
    Q_x: float = 0.0  # CoM position weight (usually 0)
    R: float = 1e-6  # Control input weight (small regularization)

    # Safety margins
    zmp_margin: float = 0.02  # meters (stay 2cm inside support polygon)

    def __post_init__(self):
        """Compute derived parameters"""
        self.preview_steps = int(self.preview_horizon / self.dt)
        self.omega = np.sqrt(self.gravity / self.com_height)  # Natural frequency


class PreviewCoMPlanner:
    """
    Preview Control for CoM Trajectory Planning

    Solves the optimal control problem:
        minimize: sum(Q_e * e^2 + R * u^2)
        subject to: ẍ = omega^2 * (x - p)  (LIPM dynamics)
                    e = p_ref - p          (ZMP tracking error)

    where:
        x = CoM position
        p = ZMP position
        p_ref = desired ZMP trajectory
        u = jerk (third derivative of position)
        e = ZMP tracking error
    """

    def __init__(self, params: Optional[CoMPlannerParams] = None):
        """
        Initialize Preview Control CoM planner

        Args:
            params: Planner parameters (uses defaults if None)
        """
        self.params = params or CoMPlannerParams()

        # State space model matrices
        self._build_state_space_model()

        # Compute preview control gains
        self._compute_preview_gains()

        # Internal state
        self.x = np.zeros(3)  # [position, velocity, acceleration]
        self.future_zmp_ref = []  # Preview buffer for reference ZMP

        print(f"[CoM Planner] Initialized Preview Control")
        print(f"  CoM height: {self.params.com_height}m")
        print(f"  Natural frequency: {self.params.omega:.3f} rad/s")
        print(f"  Preview horizon: {self.params.preview_horizon}s ({self.params.preview_steps} steps)")
        print(f"  Planning dt: {self.params.dt}s ({1/self.params.dt:.0f} Hz)")

    def _build_state_space_model(self):
        """
        Build discrete-time state space model for LIPM

        Continuous-time dynamics:
            ẋ = A_c * x + B_c * u
            p = C * x

        where:
            x = [x, ẋ, ẍ]  (CoM position, velocity, acceleration)
            u = ẍdot       (jerk - third derivative)
            p = x - ẍ/omega^2  (ZMP from LIPM)
        """
        omega2 = self.params.omega ** 2
        dt = self.params.dt

        # Continuous-time matrices
        A_c = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 0]
        ])

        B_c = np.array([
            [0],
            [0],
            [1]
        ])

        C = np.array([[1, 0, -1/omega2]])  # ZMP = x - ẍ/omega^2

        # Discretize using zero-order hold
        # x[k+1] = A * x[k] + B * u[k]
        # p[k] = C * x[k]
        self.A = np.array([
            [1, dt, dt**2/2],
            [0, 1,  dt],
            [0, 0,  1]
        ])

        self.B = np.array([
            [dt**3/6],
            [dt**2/2],
            [dt]
        ])

        self.C = C

        # Augmented system for integral action (tracks ZMP error)
        # x_aug = [e, x]  where e = sum(p_ref - p)
        # e[k+1] = e[k] + p_ref[k] - C*x[k]
        self.A_aug = np.block([
            [1, -self.C],
            [np.zeros((3, 1)), self.A]
        ])

        self.B_aug = np.vstack([
            np.zeros((1, 1)),
            self.B
        ])

        self.C_aug = np.array([1, 0, 0, -1/omega2])  # [e, x] -> e + ZMP

    def _compute_preview_gains(self):
        """
        Compute preview control gains using Riccati equation

        Solves discrete-time LQR problem with preview:
            K_p: Proportional gain on state
            K_i: Integral gain on accumulated error
            K_f: Preview gains for future reference (one gain per preview step)
        """
        Q_e = self.params.Q_e
        Q_x = self.params.Q_x
        R = self.params.R
        N = self.params.preview_steps

        # State cost matrix
        Q = np.diag([Q_e, Q_x, 0, 0])  # Penalize error and CoM position

        # Control cost matrix
        R_mat = np.array([[R]])

        # Solve algebraic Riccati equation for infinite horizon
        try:
            P = scipy.linalg.solve_discrete_are(self.A_aug, self.B_aug, Q, R_mat)
        except np.linalg.LinAlgError:
            print("[CoM Planner] Warning: Riccati equation ill-conditioned, using simplified gains")
            P = np.eye(4) * Q_e

        # Compute control gains
        # u = -K_p * x - K_i * e + sum(K_f[i] * p_ref[k+i])
        K_temp = np.linalg.inv(R_mat + self.B_aug.T @ P @ self.B_aug) @ self.B_aug.T @ P

        # Extract gains
        self.K_i = K_temp[0, 0]  # Integral gain
        self.K_p = K_temp[0, 1:]  # State feedback gain

        # Compute preview gains (one per future timestep)
        self.K_f = np.zeros(N)
        A_c = self.A_aug - self.B_aug @ K_temp @ self.A_aug  # Closed-loop system

        X = -A_c.T @ P
        for i in range(N):
            self.K_f[i] = (np.linalg.inv(R_mat + self.B_aug.T @ P @ self.B_aug) @
                           self.B_aug.T @ X)[0, 0]
            X = A_c.T @ X

        print(f"[CoM Planner] Preview gains computed")
        print(f"  K_i (integral): {self.K_i:.6f}")
        print(f"  K_p (state): {self.K_p}")
        print(f"  K_f range: [{self.K_f.min():.6f}, {self.K_f.max():.6f}]")

    def reset(self, initial_com_pos: float = 0.0, initial_com_vel: float = 0.0):
        """
        Reset planner state

        Args:
            initial_com_pos: Initial CoM position (m)
            initial_com_vel: Initial CoM velocity (m/s)
        """
        self.x = np.array([initial_com_pos, initial_com_vel, 0.0])
        self.future_zmp_ref = []
        self.accumulated_error = 0.0
        self.accumulated_error = 0.0

    def update_zmp_reference(self, zmp_ref_trajectory: np.ndarray):
        """
        Update future ZMP reference trajectory

        Args:
            zmp_ref_trajectory: Array of future ZMP positions (length >= preview_steps)
        """
        if len(zmp_ref_trajectory) < self.params.preview_steps:
            # Pad with last value if trajectory too short
            padding = np.full(
                self.params.preview_steps - len(zmp_ref_trajectory),
                zmp_ref_trajectory[-1]
            )
            zmp_ref_trajectory = np.concatenate([zmp_ref_trajectory, padding])

        self.future_zmp_ref = zmp_ref_trajectory[:self.params.preview_steps]

    def compute_com_command(self, current_zmp_ref: float) -> Tuple[float, float, float]:
        """
        Compute optimal CoM position, velocity, acceleration using preview control

        Args:
            current_zmp_ref: Current desired ZMP position (m)

        Returns:
            Tuple of (com_position, com_velocity, com_acceleration) in meters
        """
        # Current ZMP from LIPM
        current_zmp = self.x[0] - self.x[2] / (self.params.omega ** 2)

        # ZMP tracking error
        zmp_error = current_zmp_ref - current_zmp
        self.accumulated_error += zmp_error

        # Augmented state: [accumulated_error, position, velocity, acceleration]
        x_aug = np.array([self.accumulated_error, self.x[0], self.x[1], self.x[2]])

        # State feedback control
        u_fb = -self.K_i * self.accumulated_error - self.K_p @ self.x

        # Preview feedforward control
        u_ff = 0.0
        if len(self.future_zmp_ref) >= self.params.preview_steps:
            for i in range(self.params.preview_steps):
                u_ff += self.K_f[i] * self.future_zmp_ref[i]

        # Total control input (jerk)
        u = u_fb + u_ff

        # Update state: x[k+1] = A * x[k] + B * u[k]
        self.x = self.A @ self.x + self.B.flatten() * u

        return self.x[0], self.x[1], self.x[2]

    def plan_trajectory(self,
                       zmp_reference: np.ndarray,
                       initial_state: Optional[Tuple[float, float, float]] = None) -> np.ndarray:
        """
        Plan complete CoM trajectory for given ZMP reference

        Args:
            zmp_reference: Array of desired ZMP positions over time
            initial_state: Optional (position, velocity, acceleration) tuple

        Returns:
            Array of shape (N, 3) with [position, velocity, acceleration] at each timestep
        """
        N = len(zmp_reference)
        trajectory = np.zeros((N, 3))

        # Initialize state
        if initial_state is not None:
            self.x = np.array(initial_state)
            self.accumulated_error = 0.0

        # Plan trajectory
        for i in range(N):
            # Update preview window
            preview_window = zmp_reference[i:i+self.params.preview_steps]
            self.update_zmp_reference(preview_window)

            # Compute CoM command
            pos, vel, acc = self.compute_com_command(zmp_reference[i])
            trajectory[i] = [pos, vel, acc]

        return trajectory

    def get_zmp_from_com(self, com_pos: float, com_acc: float) -> float:
        """
        Compute ZMP from CoM state using LIPM

        Args:
            com_pos: CoM position (m)
            com_acc: CoM acceleration (m/s^2)

        Returns:
            ZMP position (m)
        """
        return com_pos - com_acc / (self.params.omega ** 2)


class CoMPlanner2D:
    """
    2D CoM Planner (X and Y directions)

    Wraps two independent PreviewCoMPlanner instances for lateral (Y) and
    sagittal (X) planes.
    """

    def __init__(self, params: Optional[CoMPlannerParams] = None):
        """
        Initialize 2D CoM planner

        Args:
            params: Planner parameters (shared between X and Y)
        """
        self.params = params or CoMPlannerParams()

        # Independent planners for each direction
        self.planner_x = PreviewCoMPlanner(self.params)
        self.planner_y = PreviewCoMPlanner(self.params)

        print(f"[CoM Planner 2D] Initialized for X and Y directions")

    def reset(self, initial_com: np.ndarray, initial_vel: np.ndarray):
        """
        Reset both planners

        Args:
            initial_com: [x, y] initial CoM position (m)
            initial_vel: [vx, vy] initial CoM velocity (m/s)
        """
        self.planner_x.reset(initial_com[0], initial_vel[0])
        self.planner_y.reset(initial_com[1], initial_vel[1])

    def compute_com_command(self, zmp_ref: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute 2D CoM command

        Args:
            zmp_ref: [x, y] desired ZMP position (m)

        Returns:
            Tuple of (position, velocity, acceleration) arrays [x, y]
        """
        # X direction
        pos_x, vel_x, acc_x = self.planner_x.compute_com_command(zmp_ref[0])

        # Y direction
        pos_y, vel_y, acc_y = self.planner_y.compute_com_command(zmp_ref[1])

        return (
            np.array([pos_x, pos_y]),
            np.array([vel_x, vel_y]),
            np.array([acc_x, acc_y])
        )

    def update_zmp_reference(self, zmp_ref_x: np.ndarray, zmp_ref_y: np.ndarray):
        """
        Update future ZMP reference for both directions

        Args:
            zmp_ref_x: Future X ZMP positions
            zmp_ref_y: Future Y ZMP positions
        """
        self.planner_x.update_zmp_reference(zmp_ref_x)
        self.planner_y.update_zmp_reference(zmp_ref_y)

    def plan_trajectory(self,
                       zmp_reference: np.ndarray,
                       initial_state: Optional[Tuple[np.ndarray, np.ndarray]] = None) -> np.ndarray:
        """
        Plan complete 2D CoM trajectory

        Args:
            zmp_reference: Array of shape (N, 2) with [x, y] ZMP positions
            initial_state: Optional tuple of ([x, y] position, [vx, vy] velocity)

        Returns:
            Array of shape (N, 2, 3) with [position, velocity, acceleration] for [x, y]
        """
        N = len(zmp_reference)
        trajectory = np.zeros((N, 2, 3))

        # Initialize if provided
        if initial_state is not None:
            pos, vel = initial_state
            self.reset(pos, vel)

        # Plan X and Y trajectories independently
        traj_x = self.planner_x.plan_trajectory(zmp_reference[:, 0])
        traj_y = self.planner_y.plan_trajectory(zmp_reference[:, 1])

        trajectory[:, 0, :] = traj_x  # X: [pos, vel, acc]
        trajectory[:, 1, :] = traj_y  # Y: [pos, vel, acc]

        return trajectory
