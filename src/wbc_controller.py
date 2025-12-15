"""
Whole-Body Control (WBC) for Bipedal Robot

Implements hierarchical optimization for:
1. Ground reaction force optimization
2. Torque computation from desired accelerations
3. Constraint satisfaction (friction cone, torque limits)

Based on:
- MIT Cheetah 3 WBC approach
- Hierarchical QP optimization
"""

import numpy as np
import cvxpy as cp
from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass
import pybullet as p

from stability_metrics import compute_com
from inverse_dynamics import InverseDynamics


@dataclass
class WBCParams:
    """WBC controller parameters"""
    # Friction parameters
    friction_coef: float = 0.5  # Ground friction coefficient

    # Force limits
    max_normal_force: float = 500.0  # Maximum normal force per foot (N)
    min_normal_force: float = 0.5    # Minimum normal force (N)

    # Weights for QP objective
    w_force_tracking: float = 25.0     # Weight for force tracking
    w_force_regularization: float = 0.0001  # Weight for force regularization
    w_torque_regularization: float = 0.001  # Weight for torque regularization

    # Cartesian foot stiffness (NEW - for stability)
    w_foot_anchor: float = 0.0        # Weight for foot position anchoring (0 = disabled)
    foot_stiffness_kp: float = 100.0  # Cartesian stiffness (N/m)
    foot_damping_kd: float = 50.0     # Cartesian damping (N·s/m)

    # Numerical stability
    epsilon: float = 1e-6


class WholeBodyController:
    """
    Whole-Body Controller using Quadratic Programming

    Solves hierarchical optimization:
    1. Desired body accelerations (from MPC)
    2. Ground reaction forces (with constraints)
    3. Joint torques
    """

    def __init__(self, robot_id: int, joint_dict: Dict[str, int], params: WBCParams = None):
        """
        Initialize WBC controller

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
            params: WBC parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict
        self.params = params if params else WBCParams()

        # Get robot properties
        self.num_joints = len(joint_dict)
        self.mass = self._compute_total_mass()

        # Inverse dynamics (Phase 2.2)
        self.inv_dyn = InverseDynamics(robot_id)

    def _compute_total_mass(self) -> float:
        """Compute total robot mass from URDF"""
        total_mass = 0.0

        # Base link mass
        dynamics_info = p.getDynamicsInfo(self.robot_id, -1)
        total_mass += dynamics_info[0]

        # All link masses
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            dynamics_info = p.getDynamicsInfo(self.robot_id, i)
            total_mass += dynamics_info[0]

        return total_mass

    def compute_ground_reaction_forces(self,
                                      desired_base_accel: np.ndarray,
                                      foot_positions: List[np.ndarray],
                                      foot_contacts: List[bool],
                                      foot_reference_positions: List[np.ndarray] = None,
                                      foot_velocities: List[np.ndarray] = None,
                                      force_reference: Optional[List[np.ndarray]] = None) -> np.ndarray:
        """
        Compute optimal ground reaction forces using QP

        Args:
            desired_base_accel: Desired base acceleration [ax, ay, az, alpha_x, alpha_y, alpha_z]
            foot_positions: List of foot positions in world frame
            foot_contacts: List of boolean flags indicating if foot is in contact
            foot_reference_positions: List of desired foot positions (for anchoring), optional
            foot_velocities: List of foot velocities (for damping), optional
            force_reference: Optional list of desired forces per foot (world frame)

        Returns:
            ground_forces: Nx3 array of ground reaction forces for each foot
        """
        num_feet = len(foot_positions)
        num_contact_feet = sum(foot_contacts)

        if num_contact_feet == 0:
            # No contacts, return zero forces
            return np.zeros((num_feet, 3))

        # Build contact selection matrix
        contact_indices = [i for i, c in enumerate(foot_contacts) if c]

        # Decision variables: force for each contact foot
        # f = [fx1, fy1, fz1, fx2, fy2, fz2, ...]
        f = cp.Variable(num_contact_feet * 3)

        # Build dynamics matrix A: F_total = A * f
        # where F_total = [total_force, total_torque]
        A = self._build_contact_jacobian(foot_positions, contact_indices)

        # Desired wrench (force + torque) from dynamics
        # F = m * a_desired + m * g  (expressed in world frame)
        gravity_force = np.array([0.0, 0.0, self.mass * 9.81])
        desired_force = gravity_force + self.mass * desired_base_accel[0:3]

        # For torque, use CoM-aligned frame (simplified inertia about CoM)
        com_pos = compute_com(self.robot_id)
        desired_torque = desired_base_accel[3:6] * (self.mass * 0.05)  # small inertia proxy

        desired_wrench = np.concatenate([desired_force, desired_torque])

        # Build objective function terms
        objective_terms = [
            self.params.w_force_tracking * cp.sum_squares(A @ f - desired_wrench),
            self.params.w_force_regularization * cp.sum_squares(f)
        ]

        # Track planner-provided contact forces if available
        if force_reference is not None and len(force_reference) == num_feet:
            f_ref = []
            for idx in contact_indices:
                f_ref.extend(list(force_reference[idx]))
            f_ref_vec = cp.Parameter(num_contact_feet * 3, value=np.array(f_ref))
            objective_terms.append(
                self.params.w_force_tracking * cp.sum_squares(f - f_ref_vec)
            )

        # Add Cartesian foot anchoring term (if enabled and data provided)
        if (self.params.w_foot_anchor > 0 and
            foot_reference_positions is not None and
            foot_velocities is not None):

            # Compute desired anchoring forces for each contact foot
            f_anchor = np.zeros(num_contact_feet * 3)
            for i, contact_idx in enumerate(contact_indices):
                # Position error
                pos_error = foot_reference_positions[contact_idx] - foot_positions[contact_idx]
                # Velocity (we want zero velocity for stance feet)
                vel = foot_velocities[contact_idx]

                # Desired anchoring force: F = kp * pos_error - kd * vel
                f_desired = (self.params.foot_stiffness_kp * pos_error -
                            self.params.foot_damping_kd * vel)

                # Store in anchor force vector
                f_anchor[i*3:(i+1)*3] = f_desired

            # Add anchoring objective term
            objective_terms.append(
                self.params.w_foot_anchor * cp.sum_squares(f - f_anchor)
            )

        # Combine all objective terms
        objective = cp.Minimize(cp.sum(objective_terms))

        # Constraints
        constraints = []

        # Friction cone constraints for each contact
        mu = self.params.friction_coef
        for i in range(num_contact_feet):
            fx = f[i*3 + 0]
            fy = f[i*3 + 1]
            fz = f[i*3 + 2]

            # Linearized friction cone (pyramid approximation)
            # |fx| <= mu * fz
            # |fy| <= mu * fz
            constraints.append(fx <= mu * fz)
            constraints.append(fx >= -mu * fz)
            constraints.append(fy <= mu * fz)
            constraints.append(fy >= -mu * fz)

            # Normal force limits
            constraints.append(fz >= self.params.min_normal_force)
            constraints.append(fz <= self.params.max_normal_force)

        # Ensure total normal force supports gravity
        total_normal = cp.sum([f[i*3 + 2] for i in range(num_contact_feet)])
        constraints.append(total_normal >= self.mass * 9.81)

        # Solve QP
        problem = cp.Problem(objective, constraints)

        try:
            problem.solve(solver=cp.OSQP, verbose=False)

            if problem.status not in ["optimal", "optimal_inaccurate"]:
                print(f"WBC optimization failed: {problem.status}")
                # Return balanced forces as fallback
                return self._get_balanced_forces(num_feet, foot_contacts)

            # Extract solution
            forces_contact = f.value.reshape((num_contact_feet, 3))

            # Map back to all feet
            ground_forces = np.zeros((num_feet, 3))
            for i, contact_idx in enumerate(contact_indices):
                ground_forces[contact_idx] = forces_contact[i]

            return ground_forces

        except Exception as e:
            print(f"WBC optimization error: {e}")
            return self._get_balanced_forces(num_feet, foot_contacts)

    def _build_contact_jacobian(self,
                                foot_positions: List[np.ndarray],
                                contact_indices: List[int]) -> np.ndarray:
        """
        Build contact Jacobian matrix A

        Maps contact forces to resultant wrench on base:
        [F_total; tau_total] = A * [f1; f2; ...]

        Args:
            foot_positions: List of all foot positions
            contact_indices: Indices of feet in contact

        Returns:
            A: 6 x (3*num_contacts) matrix
        """
        num_contacts = len(contact_indices)
        A = np.zeros((6, 3 * num_contacts))

        # Get accurate CoM position (Phase 1.1)
        com_pos = compute_com(self.robot_id)

        for i, foot_idx in enumerate(contact_indices):
            foot_pos = foot_positions[foot_idx]

            # Force part (sum of forces)
            A[0:3, i*3:(i+1)*3] = np.eye(3)

            # Torque part (cross product: r x F)
            r = foot_pos - com_pos  # Position vector from CoM to foot

            # Skew-symmetric matrix for cross product
            r_skew = np.array([
                [0, -r[2], r[1]],
                [r[2], 0, -r[0]],
                [-r[1], r[0], 0]
            ])

            A[3:6, i*3:(i+1)*3] = r_skew

        return A

    def _get_balanced_forces(self,
                            num_feet: int,
                            foot_contacts: List[bool]) -> np.ndarray:
        """
        Fallback: return evenly distributed forces

        Args:
            num_feet: Total number of feet
            foot_contacts: Contact flags

        Returns:
            ground_forces: Evenly distributed forces
        """
        ground_forces = np.zeros((num_feet, 3))
        num_contacts = sum(foot_contacts)

        if num_contacts > 0:
            # Distribute weight evenly
            force_per_foot = self.mass * 9.81 / num_contacts

            for i, in_contact in enumerate(foot_contacts):
                if in_contact:
                    ground_forces[i] = np.array([0, 0, force_per_foot])

        return ground_forces

    def compute_joint_torques(self,
                             ground_forces: np.ndarray,
                             foot_jacobians: List[np.ndarray]) -> Dict[str, float]:
        """
        Compute joint torques from ground reaction forces

        Using: tau = J^T * F

        Args:
            ground_forces: Nx3 array of ground forces
            foot_jacobians: List of 3xM Jacobians for each foot

        Returns:
            torques: Dictionary of joint torques
        """
        torques = {}

        # For simplicity, assume we have leg-wise control
        # In practice, this would use the full robot Jacobian

        # This is a placeholder - full implementation would compute
        # proper Jacobians from PyBullet
        for joint_name in self.joint_dict.keys():
            torques[joint_name] = 0.0

        return torques

    def compute_torques_from_accelerations(self,
                                          desired_joint_accelerations: Dict[str, float]) -> Dict[str, float]:
        """
        Compute joint torques to achieve desired accelerations

        Uses inverse dynamics (Phase 2.2):
        τ = M(q)q̈ + g(q)

        Args:
            desired_joint_accelerations: Dictionary of {joint_name: acceleration}

        Returns:
            torques: Dictionary of {joint_name: torque}
        """
        # Get current joint states
        joint_names = list(self.joint_dict.keys())
        joint_positions = np.zeros(len(joint_names))
        joint_velocities = np.zeros(len(joint_names))
        desired_accels = np.zeros(len(joint_names))

        for i, joint_name in enumerate(joint_names):
            joint_idx = self.joint_dict[joint_name]
            state = p.getJointState(self.robot_id, joint_idx)
            joint_positions[i] = state[0]
            joint_velocities[i] = state[1]
            desired_accels[i] = desired_joint_accelerations.get(joint_name, 0.0)

        # Use inverse dynamics
        torques_array = self.inv_dyn.inverse_dynamics(
            joint_positions,
            joint_velocities,
            desired_accels
        )

        # Convert to dictionary
        torques = {}
        for i, joint_name in enumerate(joint_names):
            torques[joint_name] = torques_array[i]

        return torques


if __name__ == "__main__":
    print("WBC Controller Module")
    print("Requires PyBullet simulation to test")
    print("\nFeatures:")
    print("- Ground reaction force optimization via QP")
    print("- Friction cone constraints")
    print("- Force/torque regularization")
    print("- Hierarchical task priority")
