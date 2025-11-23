"""
Model Predictive Control (MPC) for bipedal robot balance

Uses Linear Inverted Pendulum Model (LIPM) for CoM trajectory tracking
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class MPCParams:
    """MPC controller parameters"""
    prediction_horizon: int = 16  # Number of steps to predict ahead
    control_horizon: int = 8      # Number of control inputs to optimize
    dt: float = 0.1               # Time step for MPC (100ms)
    com_height: float = 0.35      # Center of mass height (m)
    gravity: float = 9.81         # Gravitational acceleration (m/s^2)

    # Cost function weights
    Q_position: float = 1.0       # CoM position tracking weight
    Q_velocity: float = 0.1       # CoM velocity tracking weight
    R_zmp: float = 1e-6           # ZMP control effort weight

    # Constraints
    max_zmp_offset: float = 0.08  # Maximum ZMP offset from support center (m)


class LinearInvertedPendulumMPC:
    """
    MPC controller using Linear Inverted Pendulum Model (LIPM)

    The LIPM approximates the robot as a point mass at constant height
    with ZMP control for balance.
    """

    def __init__(self, params: MPCParams = None):
        """
        Initialize MPC controller

        Args:
            params: MPC parameters
        """
        self.params = params if params is not None else MPCParams()

        # Calculate natural frequency of inverted pendulum
        # omega = sqrt(g/h)
        self.omega = np.sqrt(self.params.gravity / self.params.com_height)

        # Build state space model matrices
        self._build_state_space_model()

        # Precompute MPC matrices for efficiency
        self._build_mpc_matrices()

    def _build_state_space_model(self):
        """
        Build discrete-time state space model for LIPM

        State: x = [pos_x, vel_x, pos_y, vel_y]
        Control: u = [zmp_x, zmp_y]

        Dynamics (continuous):
        ddot{x} = omega^2 * (x - zmp_x)
        ddot{y} = omega^2 * (y - zmp_y)
        """
        dt = self.params.dt
        omega = self.omega

        # Continuous-time A matrix (decoupled x and y)
        # For x: [x, dx/dt]' = [0, 1; omega^2, 0] * [x, dx/dt]' + [-omega^2] * zmp_x
        A_cont_1d = np.array([
            [0, 1],
            [omega**2, 0]
        ])

        B_cont_1d = np.array([
            [0],
            [-omega**2]
        ])

        # Discretize using matrix exponential approximation
        # For small dt: A_d ≈ I + A_c * dt
        # B_d ≈ B_c * dt
        A_discrete_1d = np.eye(2) + A_cont_1d * dt
        B_discrete_1d = B_cont_1d * dt

        # Full 4D state space (x and y are decoupled)
        self.A = np.zeros((4, 4))
        self.A[0:2, 0:2] = A_discrete_1d
        self.A[2:4, 2:4] = A_discrete_1d

        self.B = np.zeros((4, 2))
        self.B[0:2, 0:1] = B_discrete_1d
        self.B[2:4, 1:2] = B_discrete_1d

    def _build_mpc_matrices(self):
        """
        Precompute MPC prediction and control matrices

        The optimal control problem is:
        min_{U} ||P*x0 + Q*U - X_ref||^2 + ||R*U||^2

        where U = [u_0, u_1, ..., u_{N-1}] is the control sequence
        """
        N = self.params.prediction_horizon
        M = self.params.control_horizon

        n_state = 4  # [x, dx, y, dy]
        n_control = 2  # [zmp_x, zmp_y]

        # Build prediction matrix P (state prediction from initial state)
        # X = P*x0 + Q*U
        P = np.zeros((N * n_state, n_state))
        for i in range(N):
            P[i*n_state:(i+1)*n_state, :] = np.linalg.matrix_power(self.A, i+1)

        # Build control matrix Q (state prediction from control inputs)
        Q = np.zeros((N * n_state, M * n_control))
        for i in range(N):
            for j in range(min(i+1, M)):
                # Effect of control u_j on state at step i+1
                A_power = np.linalg.matrix_power(self.A, i-j)
                Q[i*n_state:(i+1)*n_state, j*n_control:(j+1)*n_control] = A_power @ self.B

        self.P_mpc = P
        self.Q_mpc = Q

        # Build cost matrices
        # State cost (only care about position and velocity)
        Q_state = np.diag([
            self.params.Q_position,  # x position
            self.params.Q_velocity,  # x velocity
            self.params.Q_position,  # y position
            self.params.Q_velocity   # y velocity
        ])
        Q_state_extended = np.kron(np.eye(N), Q_state)

        # Control cost
        R_control = np.diag([self.params.R_zmp, self.params.R_zmp])
        R_control_extended = np.kron(np.eye(M), R_control)

        # Compute Hessian for QP: H = Q^T * Q_state * Q + R
        self.H = Q.T @ Q_state_extended @ Q + R_control_extended

        # Make sure H is symmetric
        self.H = (self.H + self.H.T) / 2

        # Add small regularization for numerical stability
        self.H += np.eye(M * n_control) * 1e-6

        # Store cost matrices for gradient computation
        self.Q_state_extended = Q_state_extended
        self.R_control_extended = R_control_extended

    def compute_optimal_zmp(self,
                           current_state: np.ndarray,
                           reference_trajectory: np.ndarray,
                           support_center: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute optimal ZMP trajectory using MPC

        Args:
            current_state: Current CoM state [x, dx, y, dy]
            reference_trajectory: Reference CoM trajectory (N x 4)
            support_center: Current support polygon center [x, y]

        Returns:
            optimal_zmp: Optimal ZMP for current step [zmp_x, zmp_y]
            predicted_trajectory: Predicted CoM trajectory (N x 4)
        """
        N = self.params.prediction_horizon
        M = self.params.control_horizon

        # Flatten reference trajectory
        x_ref = reference_trajectory.flatten()

        # Compute gradient: g = Q^T * Q_state * (P*x0 - x_ref)
        prediction = self.P_mpc @ current_state
        g = self.Q_mpc.T @ self.Q_state_extended @ (prediction - x_ref)

        # Solve QP: min 0.5 * u^T * H * u + g^T * u
        # Using simple analytical solution (no constraints for now)
        # u_opt = -H^{-1} * g
        try:
            u_optimal = np.linalg.solve(self.H, -g)
        except np.linalg.LinAlgError:
            # If singular, use pseudo-inverse
            u_optimal = -np.linalg.pinv(self.H) @ g

        # Extract first control input
        optimal_zmp = u_optimal[0:2]

        # Apply constraints: ZMP should be near support center
        zmp_offset = optimal_zmp - support_center
        zmp_offset_magnitude = np.linalg.norm(zmp_offset)

        if zmp_offset_magnitude > self.params.max_zmp_offset:
            # Clip to maximum offset
            zmp_offset = zmp_offset / zmp_offset_magnitude * self.params.max_zmp_offset
            optimal_zmp = support_center + zmp_offset

        # Compute predicted trajectory with optimal control
        u_sequence = u_optimal.reshape((M, 2))
        predicted_trajectory = self._predict_trajectory(current_state, u_sequence)

        return optimal_zmp, predicted_trajectory

    def _predict_trajectory(self, x0: np.ndarray, u_sequence: np.ndarray) -> np.ndarray:
        """
        Predict state trajectory given initial state and control sequence

        Args:
            x0: Initial state [x, dx, y, dy]
            u_sequence: Control sequence (M x 2)

        Returns:
            trajectory: Predicted trajectory (N x 4)
        """
        N = self.params.prediction_horizon
        M = self.params.control_horizon

        trajectory = np.zeros((N, 4))
        x = x0.copy()

        for i in range(N):
            # Apply control (use last control if beyond control horizon)
            u = u_sequence[min(i, M-1)] if i < M else u_sequence[-1]

            # Predict next state
            x = self.A @ x + self.B @ u
            trajectory[i] = x

        return trajectory

    def generate_reference_trajectory(self,
                                     current_com: np.ndarray,
                                     target_com: np.ndarray,
                                     current_velocity: np.ndarray = None) -> np.ndarray:
        """
        Generate smooth reference trajectory from current to target CoM

        Args:
            current_com: Current CoM position [x, y]
            target_com: Target CoM position [x, y]
            current_velocity: Current CoM velocity [dx, dy] (optional)

        Returns:
            reference: Reference trajectory (N x 4)
        """
        N = self.params.prediction_horizon

        if current_velocity is None:
            current_velocity = np.zeros(2)

        # Use 5th order polynomial for smooth trajectory
        reference = np.zeros((N, 4))

        for i in range(N):
            # Normalized time [0, 1]
            s = (i + 1) / N

            # 5th order polynomial: ensures smooth position, velocity, and acceleration
            # s(t) = 10t^3 - 15t^4 + 6t^5
            poly = 10*s**3 - 15*s**4 + 6*s**5
            poly_dot = (30*s**2 - 60*s**3 + 30*s**4) / (N * self.params.dt)

            # Interpolate position
            pos = current_com + (target_com - current_com) * poly

            # Interpolate velocity
            vel = current_velocity * (1 - poly) + (target_com - current_com) / (N * self.params.dt) * poly_dot

            reference[i, 0] = pos[0]  # x
            reference[i, 1] = vel[0]  # dx
            reference[i, 2] = pos[1]  # y
            reference[i, 3] = vel[1]  # dy

        return reference


if __name__ == "__main__":
    # Test MPC controller
    print("=== Testing MPC Controller ===\n")

    params = MPCParams(
        prediction_horizon=16,
        control_horizon=8,
        dt=0.1,
        com_height=0.35
    )

    mpc = LinearInvertedPendulumMPC(params)

    # Test case: CoM at origin, want to move to (0.1, 0.05)
    current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, dx, y, dy]
    target_com = np.array([0.1, 0.05])
    current_com = np.array([0.0, 0.0])
    support_center = np.array([0.0, 0.0])

    # Generate reference
    reference = mpc.generate_reference_trajectory(current_com, target_com)

    print("Reference trajectory (first 5 steps):")
    for i in range(5):
        print(f"  Step {i}: pos=({reference[i,0]:.4f}, {reference[i,2]:.4f}), "
              f"vel=({reference[i,1]:.4f}, {reference[i,3]:.4f})")

    # Compute optimal ZMP
    optimal_zmp, predicted = mpc.compute_optimal_zmp(current_state, reference, support_center)

    print(f"\nOptimal ZMP: ({optimal_zmp[0]:.4f}, {optimal_zmp[1]:.4f})")
    print("\nPredicted trajectory (first 5 steps):")
    for i in range(5):
        print(f"  Step {i}: pos=({predicted[i,0]:.4f}, {predicted[i,2]:.4f}), "
              f"vel=({predicted[i,1]:.4f}, {predicted[i,3]:.4f})")

    print("\n✓ MPC controller test completed!")
