"""
Full-Body Inverse Kinematics Solver

Solves for base position + joint angles simultaneously to satisfy:
- Foot position constraints (if foot in contact)
- CoM position target
- Upright base orientation
- Joint limits

Uses scipy.optimize for nonlinear optimization with PyBullet forward kinematics.

Author: Phase 4.2 Implementation
Date: 2025-11-25
"""

import numpy as np
import pybullet as p
from scipy.optimize import minimize, Bounds
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List
import time


@dataclass
class FullBodyIKParams:
    """Parameters for full-body IK solver"""

    # Optimization weights
    foot_weight: float = 100.0  # Weight for foot position tracking
    com_weight: float = 50.0  # Weight for CoM position tracking (XYZ)
    orientation_weight: float = 20.0  # Weight for upright orientation
    regularization_weight: float = 1.0  # Weight for staying close to current config

    # Solver settings
    max_iterations: int = 100
    tolerance: float = 1e-4
    method: str = 'SLSQP'  # Sequential Least Squares Programming

    # Physical constraints
    com_height: float = 0.689  # Desired CoM height (m)
    max_roll_pitch: float = 0.2  # Maximum roll/pitch deviation (rad, ~11 degrees)
    base_height_min: float = 0.60  # Bound base height to avoid crouch/loft
    base_height_max: float = 0.72


class FullBodyIKSolver:
    """
    Full-Body IK Solver for bipedal robot

    Solves optimization problem:
        minimize: w_foot * ||foot_pos - target||^2 +
                 w_com * ||com_pos - target||^2 +
                 w_orient * (roll^2 + pitch^2) +
                 w_reg * ||config - current||^2

        subject to: joint_limits
    """

    def __init__(self, robot_id: int, joint_dict: Dict[str, int], params: Optional[FullBodyIKParams] = None):
        """
        Initialize full-body IK solver

        Args:
            robot_id: PyBullet robot body ID
            joint_dict: Dictionary mapping joint names to indices
            params: Solver parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict
        self.params = params or FullBodyIKParams()

        # Get joint info
        self.num_joints = len(joint_dict)
        self.joint_names = list(joint_dict.keys())
        self.joint_indices = [joint_dict[name] for name in self.joint_names]

        # Get joint limits
        self._get_joint_limits()

        # Foot link indices
        self.left_foot_link = joint_dict['leg_l5_joint']  # Ankle link
        self.right_foot_link = joint_dict['leg_r5_joint']

        print(f"[Full-Body IK] Initialized")
        print(f"  Joints: {self.num_joints}")
        print(f"  Method: {self.params.method}")
        print(f"  Max iterations: {self.params.max_iterations}")
        print(f"  Weights: foot={self.params.foot_weight}, com={self.params.com_weight}")

    def _get_joint_limits(self):
        """Extract joint limits from PyBullet"""
        self.joint_lower_limits = []
        self.joint_upper_limits = []

        for joint_idx in self.joint_indices:
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            self.joint_lower_limits.append(joint_info[8])  # lower limit
            self.joint_upper_limits.append(joint_info[9])  # upper limit

    def _config_to_state(self, config: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Convert optimization config to robot state

        Args:
            config: [base_x, base_y, base_z, base_roll, base_pitch, base_yaw, joint1, ..., joint10]

        Returns:
            base_pos, base_orn (quaternion), joint_angles
        """
        base_pos = config[0:3]
        base_euler = config[3:6]  # roll, pitch, yaw
        joint_angles = config[6:]

        # Convert Euler to quaternion
        base_orn = p.getQuaternionFromEuler(base_euler)

        return base_pos, base_orn, joint_angles

    def _compute_forward_kinematics(self, config: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for given configuration

        Args:
            config: Full configuration vector

        Returns:
            left_foot_pos, right_foot_pos, com_pos
        """
        base_pos, base_orn, joint_angles = self._config_to_state(config)

        # Set robot state (temporarily for FK computation)
        p.resetBasePositionAndOrientation(self.robot_id, base_pos, base_orn)
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, joint_angles[i])

        # Get foot positions
        left_foot_state = p.getLinkState(self.robot_id, self.left_foot_link)
        right_foot_state = p.getLinkState(self.robot_id, self.right_foot_link)

        left_foot_pos = np.array(left_foot_state[0])  # World position
        right_foot_pos = np.array(right_foot_state[0])

        # Compute CoM
        com_pos = self._compute_com()

        return left_foot_pos, right_foot_pos, com_pos

    def _compute_com(self) -> np.ndarray:
        """Compute center of mass position"""
        total_mass = 0.0
        com_pos = np.zeros(3)

        # Base link
        base_mass = p.getDynamicsInfo(self.robot_id, -1)[0]
        base_pos = np.array(p.getBasePositionAndOrientation(self.robot_id)[0])
        total_mass += base_mass
        com_pos += base_mass * base_pos

        # All other links
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            link_mass = p.getDynamicsInfo(self.robot_id, i)[0]
            if link_mass > 0:
                link_state = p.getLinkState(self.robot_id, i)
                link_pos = np.array(link_state[0])
                total_mass += link_mass
                com_pos += link_mass * link_pos

        com_pos /= total_mass
        return com_pos

    def _objective_function(self, config: np.ndarray, targets: Dict, contacts: Tuple[bool, bool]) -> float:
        """
        Objective function for optimization

        Args:
            config: Configuration vector
            targets: Dictionary with 'left_foot', 'right_foot', 'com'
            contacts: (left_contact, right_contact) booleans

        Returns:
            Scalar cost
        """
        # Compute forward kinematics
        left_foot_pos, right_foot_pos, com_pos = self._compute_forward_kinematics(config)

        cost = 0.0

        # Foot position costs (track both stance and swing; swing slightly softer)
        if 'left_foot' in targets:
            foot_error = np.linalg.norm(left_foot_pos - targets['left_foot'])
            weight = self.params.foot_weight * (1.0 if contacts[0] else 0.7)
            cost += weight * foot_error ** 2

        if 'right_foot' in targets:
            foot_error = np.linalg.norm(right_foot_pos - targets['right_foot'])
            weight = self.params.foot_weight * (1.0 if contacts[1] else 0.7)
            cost += weight * foot_error ** 2

        # CoM position cost (full XYZ to enforce height)
        if 'com' in targets:
            com_error = np.linalg.norm(com_pos - targets['com'])
            cost += self.params.com_weight * com_error ** 2

        # Orientation cost (penalize roll and pitch)
        _, _, base_euler = self._config_to_state(config)
        roll, pitch = base_euler[0], base_euler[1]
        cost += self.params.orientation_weight * (roll ** 2 + pitch ** 2)

        # Regularization (stay close to initial guess)
        if hasattr(self, '_initial_config'):
            config_error = np.linalg.norm(config - self._initial_config)
            cost += self.params.regularization_weight * config_error ** 2

        return cost

    def solve(self,
              left_foot_target: Optional[np.ndarray] = None,
              right_foot_target: Optional[np.ndarray] = None,
              com_target: Optional[np.ndarray] = None,
              left_contact: bool = True,
              right_contact: bool = True,
              initial_config: Optional[np.ndarray] = None) -> Optional[Dict]:
        """
        Solve full-body IK

        Args:
            left_foot_target: Target position for left foot [x, y, z]
            right_foot_target: Target position for right foot [x, y, z]
            com_target: Target CoM position [x, y, z]
            left_contact: Whether left foot is in contact
            right_contact: Whether right foot is in contact
            initial_config: Initial guess for optimization

        Returns:
            Dictionary with 'base_pos', 'base_orn', 'joint_angles', or None if failed
        """
        # Build targets dictionary
        targets = {}
        if left_foot_target is not None:
            targets['left_foot'] = np.array(left_foot_target)
        if right_foot_target is not None:
            targets['right_foot'] = np.array(right_foot_target)
        if com_target is not None:
            targets['com'] = np.array(com_target)

        contacts = (left_contact, right_contact)

        # Initial guess
        if initial_config is None:
            # Use current robot state
            base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
            base_euler = p.getEulerFromQuaternion(base_orn)
            joint_states = p.getJointStates(self.robot_id, self.joint_indices)
            joint_angles = [state[0] for state in joint_states]

            initial_config = np.concatenate([base_pos, base_euler, joint_angles])

        self._initial_config = initial_config

        # Bounds
        bounds = Bounds(
            lb=np.concatenate([
                [-10, -10, self.params.base_height_min],  # Base position (x, y, z)
                [-self.params.max_roll_pitch, -self.params.max_roll_pitch, -np.pi],  # Base orientation
                self.joint_lower_limits  # Joint angles
            ]),
            ub=np.concatenate([
                [10, 10, self.params.base_height_max],  # Base position
                [self.params.max_roll_pitch, self.params.max_roll_pitch, np.pi],  # Base orientation
                self.joint_upper_limits  # Joint angles
            ])
        )

        # Solve optimization
        start_time = time.time()
        result = minimize(
            fun=lambda x: self._objective_function(x, targets, contacts),
            x0=initial_config,
            method=self.params.method,
            bounds=bounds,
            options={
                'maxiter': self.params.max_iterations,
                'ftol': self.params.tolerance,
                'disp': False
            }
        )
        solve_time = time.time() - start_time

        if not result.success:
            print(f"[Full-Body IK] Optimization failed: {result.message}")
            return None

        # Extract solution
        base_pos, base_orn, joint_angles = self._config_to_state(result.x)

        # Build result dictionary
        solution = {
            'base_pos': base_pos,
            'base_orn': base_orn,
            'joint_angles': dict(zip(self.joint_names, joint_angles)),
            'cost': result.fun,
            'iterations': result.nit,
            'solve_time': solve_time,
            'success': True
        }

        return solution
