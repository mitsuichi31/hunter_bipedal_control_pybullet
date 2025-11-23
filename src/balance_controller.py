"""
Balance controller integrating MPC and ZMP control for bipedal robot
"""

import numpy as np
import pybullet as p
from typing import Dict, Tuple, Optional
from dataclasses import dataclass

from mpc_controller import LinearInvertedPendulumMPC, MPCParams
from inverse_kinematics import BipedalIKSolver
from stability_metrics import compute_com, compute_com_velocity, compute_zmp


@dataclass
class BalanceParams:
    """Balance controller parameters"""
    # MPC parameters
    mpc_dt: float = 0.1           # MPC control period (100ms)
    com_height: float = 0.35       # Desired CoM height

    # Stance parameters
    stance_width: float = 0.18     # Distance between feet (lateral)
    foot_length: float = 0.10      # Foot length (forward/back)

    # Control gains
    ankle_damping: float = 5.0     # Ankle damping for stability
    hip_stiffness: float = 100.0   # Hip stiffness for posture


class ZMPBalanceController:
    """
    Balance controller that combines:
    1. MPC for CoM trajectory planning
    2. ZMP calculation for stability
    3. IK for joint angle computation
    """

    def __init__(self,
                 robot_id: int,
                 joint_dict: Dict[str, int],
                 balance_params: BalanceParams = None,
                 mpc_params: MPCParams = None):
        """
        Initialize balance controller

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
            balance_params: Balance controller parameters
            mpc_params: MPC parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict

        # Parameters
        self.balance_params = balance_params if balance_params else BalanceParams()

        # Update MPC params with balance params
        if mpc_params is None:
            mpc_params = MPCParams()
        mpc_params.dt = self.balance_params.mpc_dt
        mpc_params.com_height = self.balance_params.com_height

        # Create MPC controller
        self.mpc = LinearInvertedPendulumMPC(mpc_params)

        # Create IK solver
        self.ik_solver = BipedalIKSolver(robot_id, joint_dict)

        # State
        self.time = 0.0
        self.last_mpc_time = 0.0
        self.current_zmp_target = np.array([0.0, 0.0])
        self.com_reference = None

    def get_current_com_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current center of mass position and velocity

        Uses accurate computation from stability_metrics module that
        considers all links weighted by their masses.

        Returns:
            com_pos: CoM position [x, y, z]
            com_vel: CoM velocity [dx, dy, dz]
        """
        # Use accurate CoM computation from stability_metrics (Phase 1.1)
        com_pos = compute_com(self.robot_id)
        com_vel = compute_com_velocity(self.robot_id)

        return com_pos, com_vel

    def get_support_polygon_center(self) -> np.ndarray:
        """
        Calculate center of support polygon (average of foot positions)

        Returns:
            support_center: [x, y]
        """
        left_foot_pos, right_foot_pos = self.ik_solver.get_foot_positions()

        # Average foot position (x, y only)
        support_center = np.array([
            (left_foot_pos[0] + right_foot_pos[0]) / 2.0,
            (left_foot_pos[1] + right_foot_pos[1]) / 2.0
        ])

        return support_center

    def compute_zmp(self) -> np.ndarray:
        """
        Compute actual ZMP position based on dynamics

        Uses accurate computation from stability_metrics module that
        includes acceleration terms: zmp_x = x - (h/g) * ddot_x

        Returns:
            zmp: ZMP position [x, y]
        """
        # Use accurate ZMP computation from stability_metrics (Phase 1.2)
        zmp = compute_zmp(self.robot_id)

        return zmp

    def update_mpc(self, dt: float, target_com_offset: np.ndarray = None) -> np.ndarray:
        """
        Update MPC controller to compute optimal ZMP

        Args:
            dt: Simulation time step
            target_com_offset: Desired CoM offset from support center [dx, dy]

        Returns:
            optimal_zmp: Optimal ZMP target [x, y]
        """
        self.time += dt

        # Only update MPC at specified control rate
        if self.time - self.last_mpc_time < self.balance_params.mpc_dt:
            return self.current_zmp_target

        self.last_mpc_time = self.time

        # Get current state
        com_pos, com_vel = self.get_current_com_state()
        support_center = self.get_support_polygon_center()

        # Current state for MPC: [x, dx, y, dy]
        current_state = np.array([
            com_pos[0], com_vel[0],
            com_pos[1], com_vel[1]
        ])

        # Target CoM (keep above support center by default)
        if target_com_offset is None:
            target_com_offset = np.array([0.0, 0.0])

        target_com = support_center + target_com_offset

        # Generate reference trajectory
        self.com_reference = self.mpc.generate_reference_trajectory(
            current_com=com_pos[0:2],
            target_com=target_com,
            current_velocity=com_vel[0:2]
        )

        # Compute optimal ZMP
        optimal_zmp, predicted_trajectory = self.mpc.compute_optimal_zmp(
            current_state=current_state,
            reference_trajectory=self.com_reference,
            support_center=support_center
        )

        self.current_zmp_target = optimal_zmp

        return optimal_zmp

    def compute_balance_joint_angles(self,
                                    zmp_target: np.ndarray,
                                    desired_com_height: float = None) -> Dict[str, float]:
        """
        Compute joint angles to achieve desired ZMP and CoM height

        Args:
            zmp_target: Target ZMP position [x, y]
            desired_com_height: Desired CoM height (uses default if None)

        Returns:
            joint_angles: Dictionary of joint angles
        """
        if desired_com_height is None:
            desired_com_height = self.balance_params.com_height

        # Get current support center
        support_center = self.get_support_polygon_center()

        # Calculate desired foot positions for double support
        # Feet should be placed symmetrically around support center
        stance_width = self.balance_params.stance_width

        # Target foot positions (place feet to maintain balance)
        # ZMP should be between the feet, so adjust foot placement
        left_foot_target = np.array([
            support_center[0],
            support_center[1] + stance_width / 2,
            0.0  # On ground
        ])

        right_foot_target = np.array([
            support_center[0],
            support_center[1] - stance_width / 2,
            0.0  # On ground
        ])

        # Solve IK for both legs
        joint_angles = self.ik_solver.solve_both_legs(
            left_target=left_foot_target,
            right_target=right_foot_target
        )

        return joint_angles

    def compute_standing_balance(self) -> Dict[str, float]:
        """
        Compute joint angles for balanced standing

        Returns:
            joint_angles: Dictionary of joint angles for standing
        """
        # Update MPC to get optimal ZMP
        zmp_target = self.update_mpc(0.001)  # Small dt for now

        # Get current state
        com_pos, com_vel = self.get_current_com_state()
        support_center = self.get_support_polygon_center()

        # Get base orientation for balance feedback
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_euler = p.getEulerFromQuaternion(base_orn)
        roll = base_euler[0]
        pitch = base_euler[1]

        # Calculate CoM offset from support
        com_offset = com_pos[0:2] - support_center

        # Enhanced balance control with multiple feedback terms
        # 1. Position feedback (CoM offset from support)
        # 2. Velocity feedback (CoM velocity damping)
        # 3. Orientation feedback (tilt correction)

        # Lateral balance (Y-axis, controlled by hip roll)
        lateral_position_error = com_offset[1]
        lateral_velocity_error = com_vel[1]
        lateral_orientation_error = roll

        # Minimal corrections for stability with straight-leg configuration (2025-11-23)
        # Straight legs are inherently stable - only minor adjustments needed
        hip_roll_correction = -(
            lateral_orientation_error * 0.02    # Only correct orientation (was 0.08)
        )

        # Forward/back balance - disabled for straight-leg stability
        # Active pitch control can destabilize straight legs
        hip_pitch_correction = 0.0  # Disabled (was active)
        ankle_pitch_correction = 0.0  # Disabled (was active)

        # Clamp corrections to prevent extreme values
        hip_roll_correction = np.clip(hip_roll_correction, -0.15, 0.15)
        hip_pitch_correction = np.clip(hip_pitch_correction, -0.2, 0.2)
        ankle_pitch_correction = np.clip(ankle_pitch_correction, -0.15, 0.15)

        # Base standing configuration with feedback corrections
        # Updated to stable straight-leg pose (2025-11-23)
        base_config = {
            'leg_l1_joint': -0.1 + hip_roll_correction,
            'leg_l2_joint': 0.0,
            'leg_l3_joint': 0.0 + hip_pitch_correction,     # Straight (was -0.4)
            'leg_l4_joint': 0.0,                             # Straight (was 0.8)
            'leg_l5_joint': 0.0 - hip_pitch_correction + ankle_pitch_correction,  # Straight (was -0.4)

            'leg_r1_joint': 0.1 - hip_roll_correction,
            'leg_r2_joint': 0.0,
            'leg_r3_joint': 0.0 + hip_pitch_correction,     # Straight (was -0.4)
            'leg_r4_joint': 0.0,                             # Straight (was 0.8)
            'leg_r5_joint': 0.0 - hip_pitch_correction + ankle_pitch_correction,  # Straight (was -0.4)
        }

        return base_config

    def reset(self):
        """Reset controller state"""
        self.time = 0.0
        self.last_mpc_time = 0.0
        self.current_zmp_target = np.array([0.0, 0.0])
        self.com_reference = None


if __name__ == "__main__":
    print("Balance controller module")
    print("This module integrates MPC and ZMP control")
    print("Use with simulation environment for testing")
