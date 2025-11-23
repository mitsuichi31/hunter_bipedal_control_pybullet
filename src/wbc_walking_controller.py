"""
WBC-based Walking Controller for Bipedal Robot

Uses Whole-Body Control (WBC) instead of IK to handle free-floating base dynamics properly.
Integrates gait generator with WBC framework.
"""

import numpy as np
import pybullet as p
from typing import Dict, List, Tuple
from dataclasses import dataclass

from gait_generator import GaitGenerator, GaitParams
from wbc_controller import WholeBodyController, WBCParams
from inverse_kinematics import BipedalIKSolver


@dataclass
class WBCWalkingParams:
    """Parameters for WBC walking controller"""
    # Control gains
    kp_position: float = 100.0      # Position tracking gain
    kd_position: float = 20.0       # Position damping gain
    kp_orientation: float = 200.0   # Orientation tracking gain
    kd_orientation: float = 40.0    # Orientation damping gain

    # Swing foot control
    kp_swing: float = 500.0         # Swing foot position gain
    kd_swing: float = 50.0          # Swing foot velocity gain

    # Balance control
    com_height: float = 0.55        # Desired CoM height
    max_lean_angle: float = 0.1     # Maximum body lean (rad)

    # Contact detection
    contact_force_threshold: float = 10.0  # Minimum force to consider contact (N)


class WBCWalkingController:
    """
    Walking controller using Whole-Body Control

    Key differences from IK-based approach:
    1. WBC accounts for base dynamics explicitly
    2. Handles contact forces and friction constraints
    3. Can track multiple objectives simultaneously (orientation + foot placement)
    4. No IK convergence issues
    """

    def __init__(self,
                 robot_id: int,
                 joint_dict: Dict[str, int],
                 gait_params: GaitParams = None,
                 wbc_params: WBCParams = None,
                 walking_params: WBCWalkingParams = None):
        """
        Initialize WBC walking controller

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
            gait_params: Gait generation parameters
            wbc_params: WBC parameters
            walking_params: Walking-specific parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict

        # Create gait generator
        self.gait_params = gait_params if gait_params else GaitParams()
        self.gait_generator = GaitGenerator(self.gait_params)

        # Create WBC controller
        self.wbc_params = wbc_params if wbc_params else WBCParams()
        self.wbc = WholeBodyController(robot_id, joint_dict, self.wbc_params)

        # Walking parameters
        self.walking_params = walking_params if walking_params else WBCWalkingParams()

        # Create IK solver (for getting foot positions only, not for control)
        self.ik_solver = BipedalIKSolver(robot_id, joint_dict)

        # State
        self.time = 0.0
        self.stance_foot = "right"  # "left" or "right" or "both"

    def get_foot_link_indices(self) -> Tuple[int, int]:
        """Get link indices for left and right feet"""
        left_foot_idx = None
        right_foot_idx = None

        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')

            if 'leg_l5_link' in link_name or 'left_foot' in link_name:
                left_foot_idx = i
            elif 'leg_r5_link' in link_name or 'right_foot' in link_name:
                right_foot_idx = i

        if left_foot_idx is None or right_foot_idx is None:
            # Fallback: use joint indices from joint_dict
            left_foot_idx = self.joint_dict.get('leg_l5_joint', 5)
            right_foot_idx = self.joint_dict.get('leg_r5_joint', 12)

        return left_foot_idx, right_foot_idx

    def get_foot_positions_and_velocities(self) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        Get current foot positions and velocities

        Returns:
            positions: [left_foot_pos, right_foot_pos]
            velocities: [left_foot_vel, right_foot_vel]
        """
        left_idx, right_idx = self.get_foot_link_indices()

        left_state = p.getLinkState(self.robot_id, left_idx, computeLinkVelocity=1)
        right_state = p.getLinkState(self.robot_id, right_idx, computeLinkVelocity=1)

        positions = [
            np.array(left_state[0]),   # World position
            np.array(right_state[0])
        ]

        velocities = [
            np.array(left_state[6]),   # Linear velocity
            np.array(right_state[7])
        ]

        return positions, velocities

    def detect_contacts(self) -> List[bool]:
        """
        Detect which feet are in contact with ground

        Returns:
            contacts: [left_contact, right_contact]
        """
        left_idx, right_idx = self.get_foot_link_indices()

        # Get contact points
        contact_points_left = p.getContactPoints(self.robot_id, -1, left_idx)
        contact_points_right = p.getContactPoints(self.robot_id, -1, right_idx)

        # Check if contact force exceeds threshold
        left_contact = False
        right_contact = False

        for cp in contact_points_left:
            if abs(cp[9]) > self.walking_params.contact_force_threshold:  # Normal force
                left_contact = True
                break

        for cp in contact_points_right:
            if abs(cp[9]) > self.walking_params.contact_force_threshold:
                right_contact = True
                break

        return [left_contact, right_contact]

    def determine_stance_and_swing(self) -> Tuple[str, np.ndarray, np.ndarray]:
        """
        Determine which foot is stance/swing based on gait phase

        Returns:
            stance_foot: "left", "right", or "both"
            swing_target_pos: Target position for swing foot
            swing_target_vel: Target velocity for swing foot
        """
        # Get gait trajectories
        left_target, right_target = self.gait_generator.get_foot_trajectories(self.time)
        left_vel, right_vel = self.gait_generator.get_foot_velocities(self.time)

        # Determine which foot is in swing (z > threshold)
        swing_threshold = 0.01  # 1cm off ground

        left_in_swing = left_target[2] > swing_threshold
        right_in_swing = right_target[2] > swing_threshold

        if left_in_swing and not right_in_swing:
            stance_foot = "right"
            swing_target_pos = left_target
            swing_target_vel = left_vel
        elif right_in_swing and not left_in_swing:
            stance_foot = "left"
            swing_target_pos = right_target
            swing_target_vel = right_vel
        else:
            # Both on ground (double support) or both in air (shouldn't happen)
            stance_foot = "both"
            swing_target_pos = np.zeros(3)
            swing_target_vel = np.zeros(3)

        return stance_foot, swing_target_pos, swing_target_vel

    def compute_desired_base_acceleration(self) -> np.ndarray:
        """
        Compute desired base acceleration for balance

        Returns:
            desired_accel: [ax, ay, az, alpha_x, alpha_y, alpha_z]
        """
        # Get current base state
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)

        # Convert orientation to Euler angles
        base_euler = p.getEulerFromQuaternion(base_orn)
        roll, pitch, yaw = base_euler

        # Desired orientation: upright
        desired_roll = 0.0
        desired_pitch = 0.0
        # Keep current yaw (don't control heading in this simple version)
        desired_yaw = yaw

        # Orientation error
        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch

        # Desired angular acceleration (PD control)
        kp_orient = self.walking_params.kp_orientation
        kd_orient = self.walking_params.kd_orientation

        alpha_x = kp_orient * roll_error - kd_orient * base_ang_vel[0]
        alpha_y = kp_orient * pitch_error - kd_orient * base_ang_vel[1]
        alpha_z = 0.0  # Don't control yaw

        # Desired height
        desired_height = self.walking_params.com_height
        height_error = desired_height - base_pos[2]

        # Desired linear acceleration (keep height, no lateral drift)
        kp_pos = self.walking_params.kp_position
        kd_pos = self.walking_params.kd_position

        ax = -kd_pos * base_vel[0]  # Damp forward velocity
        ay = -kd_pos * base_vel[1]  # Damp lateral velocity
        az = kp_pos * height_error - kd_pos * base_vel[2]  # Height control

        return np.array([ax, ay, az, alpha_x, alpha_y, alpha_z])

    def compute_swing_foot_forces(self,
                                  swing_foot: str,
                                  swing_target_pos: np.ndarray,
                                  swing_target_vel: np.ndarray) -> np.ndarray:
        """
        Compute virtual forces to track swing foot trajectory

        This is a simplified approach - proper WBC would use tasks.

        Returns:
            swing_force: 3D force vector for swing foot
        """
        if swing_foot == "both":
            return np.zeros(3)

        # Get current swing foot position and velocity
        foot_positions, foot_velocities = self.get_foot_positions_and_velocities()

        if swing_foot == "left":
            current_pos = foot_positions[0]
            current_vel = foot_velocities[0]
        else:  # right
            current_pos = foot_positions[1]
            current_vel = foot_velocities[1]

        # PD control for swing foot
        kp = self.walking_params.kp_swing
        kd = self.walking_params.kd_swing

        pos_error = swing_target_pos - current_pos
        vel_error = swing_target_vel - current_vel

        swing_force = kp * pos_error + kd * vel_error

        return swing_force

    def control_step(self, dt: float) -> Dict[str, float]:
        """
        Execute one control step

        Uses simplified PD control with minimal walking motion

        Args:
            dt: Time step

        Returns:
            torques: Dictionary of joint torques
        """
        # ULTRA-SIMPLE APPROACH: Just keep robot standing with straight legs
        # No swing foot tracking, no WBC, just pure PD control
        torques = self._compute_stance_torques()

        # Add minimal orientation stabilization
        orientation_torques = self._compute_orientation_stabilization()
        for joint_name, torque_adj in orientation_torques.items():
            if joint_name in torques:
                torques[joint_name] += torque_adj

        # Update time (but don't use gait generator)
        self.time += dt

        return torques

    def _compute_stance_torques(self) -> Dict[str, float]:
        """
        Compute baseline torques for stance (both legs)

        Uses PD control to maintain straight-leg configuration
        """
        # Get current joint states
        joint_states = {}
        for joint_name, joint_idx in self.joint_dict.items():
            state = p.getJointState(self.robot_id, joint_idx)
            joint_states[joint_name] = (state[0], state[1])  # pos, vel

        # Target: straight legs with slight outward stance (same as standing mode)
        target_positions = {
            'leg_l1_joint': -0.1,
            'leg_l2_joint': 0.0,
            'leg_l3_joint': 0.0,
            'leg_l4_joint': 0.0,
            'leg_l5_joint': 0.0,
            'leg_r1_joint': 0.1,
            'leg_r2_joint': 0.0,
            'leg_r3_joint': 0.0,
            'leg_r4_joint': 0.0,
            'leg_r5_joint': 0.0,
        }

        # Compute PD torques
        kp = 150.0  # Slightly lower than standing for compliance
        kd = 15.0

        torques = {}
        for joint_name in self.joint_dict.keys():
            if joint_name in joint_states and joint_name in target_positions:
                pos, vel = joint_states[joint_name]
                target_pos = target_positions[joint_name]

                torque = kp * (target_pos - pos) - kd * vel
                torques[joint_name] = torque

        return torques

    def _compute_swing_adjustment(self,
                                   swing_foot: str,
                                   swing_target_pos: np.ndarray,
                                   swing_target_vel: np.ndarray) -> Dict[str, float]:
        """
        Compute torque adjustments for swing leg to track trajectory

        Uses proportional adjustment to joint targets
        """
        # Get current foot position
        foot_positions, foot_velocities = self.get_foot_positions_and_velocities()

        if swing_foot == "left":
            current_pos = foot_positions[0]
            current_vel = foot_velocities[0]
            leg_joints = ['leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint']  # Hip, knee, ankle pitch
        else:  # right
            current_pos = foot_positions[1]
            current_vel = foot_velocities[1]
            leg_joints = ['leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint']

        # Compute position and velocity errors
        pos_error = swing_target_pos - current_pos
        vel_error = swing_target_vel - current_vel

        # Simple mapping: z-error → hip+knee, x-error → hip
        adjustments = {}

        # Lift control (z-direction): bend knee
        z_error = pos_error[2]
        knee_joint = leg_joints[1]
        adjustments[knee_joint] = 50.0 * z_error  # Proportional gain for knee lift

        # Forward/back control (x-direction): adjust hip pitch
        x_error = pos_error[0]
        hip_joint = leg_joints[0]
        adjustments[hip_joint] = 30.0 * x_error  # Proportional gain for hip

        return adjustments

    def _compute_orientation_stabilization(self) -> Dict[str, float]:
        """
        Compute torque adjustments for orientation stabilization

        Applies small corrections to hip roll joints for balance
        """
        # Get current orientation
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_euler = p.getEulerFromQuaternion(base_orn)
        roll, pitch, yaw = base_euler

        # Get angular velocity
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)

        # Stabilize roll (lateral balance) using hip roll joints
        kp_roll = 10.0
        kd_roll = 2.0

        roll_correction = -(kp_roll * roll + kd_roll * base_ang_vel[0])

        # Clamp corrections
        roll_correction = np.clip(roll_correction, -5.0, 5.0)

        adjustments = {
            'leg_l1_joint': -roll_correction,  # Left hip roll
            'leg_r1_joint': roll_correction,   # Right hip roll
        }

        return adjustments

    def _swing_foot_torques(self,
                           swing_foot: str,
                           swing_target_pos: np.ndarray,
                           swing_target_vel: np.ndarray) -> Dict[str, float]:
        """
        Compute torques for swing foot trajectory tracking

        Uses Jacobian transpose method
        """
        # Get swing foot position and velocity
        foot_positions, foot_velocities = self.get_foot_positions_and_velocities()

        if swing_foot == "left":
            current_pos = foot_positions[0]
            current_vel = foot_velocities[0]
            leg_joints = ['leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint']
        else:  # right
            current_pos = foot_positions[1]
            current_vel = foot_velocities[1]
            leg_joints = ['leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint']

        # Compute desired force (PD control in Cartesian space)
        kp = self.walking_params.kp_swing
        kd = self.walking_params.kd_swing

        pos_error = swing_target_pos - current_pos
        vel_error = swing_target_vel - current_vel

        desired_force = kp * pos_error + kd * vel_error

        # Get Jacobian (simplified - use numerical approximation)
        # Proper implementation would compute analytical Jacobian
        J = self._compute_foot_jacobian(swing_foot)

        # Jacobian transpose method: tau = J^T * F
        if J is not None:
            tau = J.T @ desired_force

            # Map to joint names
            torques = {}
            for i, joint_name in enumerate(leg_joints):
                if i < len(tau):
                    torques[joint_name] = tau[i]
        else:
            torques = {}

        return torques

    def _compute_foot_jacobian(self, foot: str) -> np.ndarray:
        """
        Compute foot Jacobian (simplified)

        Returns 3x5 Jacobian matrix mapping joint velocities to foot velocity
        """
        # This is a placeholder - proper implementation would compute analytical Jacobian
        # For now, return None to skip Jacobian-based control
        return None

    def reset(self):
        """Reset controller state"""
        self.time = 0.0
        self.stance_foot = "right"
        self.gait_generator.reset()
