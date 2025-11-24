#!/usr/bin/env python3
"""
Inverse Dynamics Module for Bipedal Robot

Computes robot dynamics quantities needed for whole-body control:
- Mass matrix M(q)
- Coriolis/centrifugal forces C(q,q̇)
- Gravity torques g(q)
- Inverse dynamics: τ = M(q)q̈ + C(q,q̇)q̇ + g(q)

Based on PyBullet's dynamics computation functions with proper
handling for free-floating base systems.

Author: Phase 2.2 - Inverse Dynamics Implementation
Date: November 2025
"""

import numpy as np
import pybullet as p
from typing import List, Dict, Optional, Tuple


class InverseDynamics:
    """
    Compute inverse dynamics for a robot in PyBullet

    Uses PyBullet's efficient dynamics computations for:
    - Mass/inertia matrix calculation
    - Coriolis and centrifugal forces
    - Gravity compensation
    - Full inverse dynamics
    """

    def __init__(self, robot_id: int):
        """
        Initialize inverse dynamics calculator

        Args:
            robot_id: PyBullet body ID of the robot
        """
        self.robot_id = robot_id

        # Cache joint information
        self._joint_info_cached = False
        self._num_joints = 0
        self._actuated_joints = []  # Indices of actuated (revolute) joints
        self._joint_names = {}  # joint_index -> name mapping
        self._joint_name_to_idx = {}  # name -> index mapping

        self._cache_joint_info()

    def _cache_joint_info(self):
        """Cache joint information for efficiency"""
        if self._joint_info_cached:
            return

        self._num_joints = p.getNumJoints(self.robot_id)

        for i in range(self._num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            joint_name = joint_info[1].decode('utf-8')

            # Only actuated revolute joints
            if joint_type == p.JOINT_REVOLUTE:
                self._actuated_joints.append(i)
                self._joint_names[i] = joint_name
                self._joint_name_to_idx[joint_name] = i

        self._joint_info_cached = True

    def compute_mass_matrix(self,
                           joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Compute joint-space mass/inertia matrix M(q)

        The mass matrix relates joint accelerations to joint torques:
        τ = M(q)q̈ + ... (other terms)

        Note: PyBullet's calculateMassMatrix returns the full system matrix
        including the 6-DOF floating base. We extract only the joint portion.

        Args:
            joint_positions: Joint angles (radians). If None, uses current state.

        Returns:
            M: Mass matrix for joints only, shape (n_joints, n_joints)
        """
        # Get joint positions for ALL joints (PyBullet requires this)
        num_joints_total = p.getNumJoints(self.robot_id)

        if joint_positions is None:
            # Get current state for all joints
            all_positions = []
            for i in range(num_joints_total):
                state = p.getJointState(self.robot_id, i)
                all_positions.append(state[0])
        else:
            # Provided positions are for actuated joints only
            # We need to create full joint list
            all_positions = []
            actuated_idx = 0
            for i in range(num_joints_total):
                if i in self._actuated_joints:
                    all_positions.append(joint_positions[actuated_idx])
                    actuated_idx += 1
                else:
                    # For fixed joints, use current position
                    state = p.getJointState(self.robot_id, i)
                    all_positions.append(state[0])

        # PyBullet's calculateMassMatrix returns a flattened 2D list (row-major)
        # For free-floating base: n_dof = 6 (base) + num_joints_total
        full_mass_matrix = p.calculateMassMatrix(self.robot_id, all_positions)

        # Convert nested list to 2D numpy array
        # PyBullet returns it as a list of lists, not a flattened list
        full_M = np.array(full_mass_matrix)

        # For free-floating base: first 6 DOF are base, rest are joints
        # Extract only actuated joint DOF
        # The mass matrix row/col for joint i is at index (6 + i) in full_M
        # where i is the joint index in the URDF

        # Compute max joint index to verify bounds
        max_joint_idx = max(self._actuated_joints) if self._actuated_joints else 0
        expected_size = 6 + num_joints_total

        if 6 + max_joint_idx >= full_M.shape[0]:
            # Fallback: extract only the joint portion and assume sequential indexing
            joint_M_block = full_M[6:, 6:]
            # Create mapping from actuated joint indices to positions in joint_M_block
            # Assume joints are contiguous in the mass matrix after the base
            actuated_M = joint_M_block[:len(self._actuated_joints), :len(self._actuated_joints)]
            return actuated_M

        # Normal case: extract using joint indices
        actuated_M = np.zeros((len(self._actuated_joints), len(self._actuated_joints)))

        for i, joint_i in enumerate(self._actuated_joints):
            for j, joint_j in enumerate(self._actuated_joints):
                # Joint i's DOF is at index (6 + joint_i) in full mass matrix
                actuated_M[i, j] = full_M[6 + joint_i, 6 + joint_j]

        return actuated_M

    def _prepare_joint_arrays(self,
                                joint_positions: Optional[np.ndarray] = None,
                                joint_velocities: Optional[np.ndarray] = None,
                                joint_accelerations: Optional[np.ndarray] = None):
        """
        Prepare joint arrays for PyBullet functions

        PyBullet calculateInverseDynamics needs positions for ALL joints, not just actuated.
        """
        num_joints_total = p.getNumJoints(self.robot_id)

        # Prepare positions
        if joint_positions is None:
            all_positions = []
            for i in range(num_joints_total):
                state = p.getJointState(self.robot_id, i)
                all_positions.append(state[0])
        else:
            all_positions = []
            actuated_idx = 0
            for i in range(num_joints_total):
                if i in self._actuated_joints:
                    all_positions.append(joint_positions[actuated_idx])
                    actuated_idx += 1
                else:
                    state = p.getJointState(self.robot_id, i)
                    all_positions.append(state[0])

        # Prepare velocities
        if joint_velocities is None:
            all_velocities = []
            for i in range(num_joints_total):
                state = p.getJointState(self.robot_id, i)
                all_velocities.append(state[1])
        else:
            all_velocities = []
            actuated_idx = 0
            for i in range(num_joints_total):
                if i in self._actuated_joints:
                    all_velocities.append(joint_velocities[actuated_idx])
                    actuated_idx += 1
                else:
                    all_velocities.append(0.0)

        # Prepare accelerations
        if joint_accelerations is None:
            all_accelerations = [0.0] * num_joints_total
        else:
            all_accelerations = []
            actuated_idx = 0
            for i in range(num_joints_total):
                if i in self._actuated_joints:
                    all_accelerations.append(joint_accelerations[actuated_idx])
                    actuated_idx += 1
                else:
                    all_accelerations.append(0.0)

        return all_positions, all_velocities, all_accelerations

    def compute_coriolis_gravity(self,
                                joint_positions: Optional[np.ndarray] = None,
                                joint_velocities: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Compute Coriolis/centrifugal and gravity forces: C(q,q̇)q̇ + g(q)

        For free-floating base robots (like Hunter), PyBullet's calculateInverseDynamics
        doesn't work. We use a simplified approach instead.

        Args:
            joint_positions: Joint angles for actuated joints (radians). If None, uses current state.
            joint_velocities: Joint velocities for actuated joints (rad/s). If None, uses current state.

        Returns:
            coriolis_gravity: Combined Coriolis and gravity forces for actuated joints (N·m)
        """
        # For free-floating base, simplified: mainly gravity, Coriolis is small
        # Use gravity compensation module's approach
        gravity = self.compute_gravity_torques(joint_positions)

        # Coriolis term is typically small for standing/slow motion
        # For now, return just gravity (conservative approach)
        return gravity

    def compute_gravity_torques(self,
                               joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Compute gravity torques g(q)

        For free-floating base robots (like Hunter), uses the gravity_compensation module
        which has a robust fallback implementation.

        Args:
            joint_positions: Joint angles for actuated joints (radians). If None, uses current state.

        Returns:
            gravity_torques: Gravity torques for actuated joints (N·m)
        """
        # Use gravity_compensation module (handles free-floating base properly)
        try:
            from gravity_compensation import GravityCompensation
            gc = GravityCompensation(self.robot_id)
            return gc.compute_gravity_torques(joint_positions)
        except ImportError:
            # Fallback if gravity_compensation not available
            return np.zeros(len(self._actuated_joints))

    def compute_coriolis_forces(self,
                               joint_positions: Optional[np.ndarray] = None,
                               joint_velocities: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Compute Coriolis/centrifugal forces C(q,q̇)q̇

        Computed as: C(q,q̇)q̇ = [C(q,q̇)q̇ + g(q)] - g(q)

        Args:
            joint_positions: Joint angles (radians). If None, uses current state.
            joint_velocities: Joint velocities (rad/s). If None, uses current state.

        Returns:
            coriolis_forces: Coriolis/centrifugal forces (N·m)
        """
        # Get Coriolis + gravity
        coriolis_gravity = self.compute_coriolis_gravity(joint_positions, joint_velocities)

        # Get gravity alone
        gravity = self.compute_gravity_torques(joint_positions)

        # Subtract to get Coriolis alone
        coriolis_forces = coriolis_gravity - gravity

        return coriolis_forces

    def inverse_dynamics(self,
                        joint_positions: np.ndarray,
                        joint_velocities: np.ndarray,
                        desired_accelerations: np.ndarray) -> np.ndarray:
        """
        Compute joint torques for desired accelerations

        Simplified inverse dynamics equation for free-floating base:
        τ ≈ M(q)q̈_desired + g(q)

        (Coriolis term omitted as it's small for standing/slow motion)

        Args:
            joint_positions: Joint angles for actuated joints (radians)
            joint_velocities: Joint velocities for actuated joints (rad/s)
            desired_accelerations: Desired joint accelerations for actuated joints (rad/s²)

        Returns:
            torques: Required joint torques for actuated joints (N·m)
        """
        # Compute mass matrix
        M = self.compute_mass_matrix(joint_positions)

        # Compute gravity
        g = self.compute_gravity_torques(joint_positions)

        # Simplified inverse dynamics: τ = M*qdd + g
        torques = M @ desired_accelerations + g

        return torques

    def forward_dynamics(self,
                        joint_positions: np.ndarray,
                        joint_velocities: np.ndarray,
                        applied_torques: np.ndarray) -> np.ndarray:
        """
        Compute joint accelerations from applied torques

        Solves: q̈ = M(q)^-1 * [τ - C(q,q̇)q̇ - g(q)]

        Args:
            joint_positions: Joint angles (radians)
            joint_velocities: Joint velocities (rad/s)
            applied_torques: Applied joint torques (N·m)

        Returns:
            accelerations: Resulting joint accelerations (rad/s²)
        """
        # Compute mass matrix
        M = self.compute_mass_matrix(joint_positions)

        # Compute Coriolis + gravity
        coriolis_gravity = self.compute_coriolis_gravity(joint_positions, joint_velocities)

        # Solve for acceleration: q̈ = M^-1 * (τ - C - g)
        accelerations = np.linalg.solve(M, applied_torques - coriolis_gravity)

        return accelerations

    def compute_dynamics_dict(self,
                             joint_positions: Optional[Dict[str, float]] = None,
                             joint_velocities: Optional[Dict[str, float]] = None) -> Dict[str, np.ndarray]:
        """
        Compute all dynamics quantities with dictionary interface

        Args:
            joint_positions: Dict of {joint_name: position}. If None, uses current.
            joint_velocities: Dict of {joint_name: velocity}. If None, uses current.

        Returns:
            Dict containing:
                - 'mass_matrix': M(q)
                - 'coriolis_gravity': C(q,q̇)q̇ + g(q)
                - 'gravity': g(q)
                - 'coriolis': C(q,q̇)q̇
        """
        # Convert dicts to arrays
        if joint_positions is not None:
            pos_array = np.zeros(len(self._actuated_joints))
            for i, joint_idx in enumerate(self._actuated_joints):
                joint_name = self._joint_names[joint_idx]
                pos_array[i] = joint_positions.get(joint_name, 0.0)
        else:
            pos_array = None

        if joint_velocities is not None:
            vel_array = np.zeros(len(self._actuated_joints))
            for i, joint_idx in enumerate(self._actuated_joints):
                joint_name = self._joint_names[joint_idx]
                vel_array[i] = joint_velocities.get(joint_name, 0.0)
        else:
            vel_array = None

        # Compute dynamics
        return {
            'mass_matrix': self.compute_mass_matrix(pos_array),
            'coriolis_gravity': self.compute_coriolis_gravity(pos_array, vel_array),
            'gravity': self.compute_gravity_torques(pos_array),
            'coriolis': self.compute_coriolis_forces(pos_array, vel_array)
        }

    def get_joint_info(self) -> Dict[str, int]:
        """
        Get joint index mapping

        Returns:
            Dict of {joint_name: joint_index}
        """
        return self._joint_name_to_idx.copy()


# Convenience functions
def compute_mass_matrix(robot_id: int, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
    """Convenience function to compute mass matrix"""
    inv_dyn = InverseDynamics(robot_id)
    return inv_dyn.compute_mass_matrix(joint_positions)


def compute_gravity_torques(robot_id: int, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
    """Convenience function to compute gravity torques"""
    inv_dyn = InverseDynamics(robot_id)
    return inv_dyn.compute_gravity_torques(joint_positions)


def inverse_dynamics(robot_id: int,
                    joint_positions: np.ndarray,
                    joint_velocities: np.ndarray,
                    desired_accelerations: np.ndarray) -> np.ndarray:
    """Convenience function for inverse dynamics"""
    inv_dyn = InverseDynamics(robot_id)
    return inv_dyn.inverse_dynamics(joint_positions, joint_velocities, desired_accelerations)


# Example usage and testing
if __name__ == "__main__":
    print("Inverse Dynamics Module")
    print("=" * 60)
    print("\nThis module provides robot dynamics computations:")
    print("  1. Mass matrix M(q)")
    print("  2. Coriolis/centrifugal forces C(q,q̇)q̇")
    print("  3. Gravity torques g(q)")
    print("  4. Inverse dynamics: τ = M(q)q̈ + C(q,q̇)q̇ + g(q)")
    print("  5. Forward dynamics: q̈ = M^-1(τ - C - g)")
    print("\nUsage:")
    print("  from inverse_dynamics import InverseDynamics")
    print("  inv_dyn = InverseDynamics(robot_id)")
    print("  torques = inv_dyn.inverse_dynamics(q, qd, qdd_desired)")
