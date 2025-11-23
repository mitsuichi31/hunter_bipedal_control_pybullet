"""
Inverse Kinematics solver for Hunter bipedal robot
"""

import pybullet as p
import numpy as np
from typing import List, Dict, Tuple, Optional


class IKSolver:
    """
    Inverse Kinematics solver using PyBullet's IK engine

    This solver can compute joint angles needed to reach a target
    end-effector position and orientation.
    """

    def __init__(self,
                 robot_id: int,
                 end_effector_link_index: int,
                 joint_indices: List[int],
                 max_iterations: int = 100,
                 residual_threshold: float = 1e-4):
        """
        Initialize IK solver

        Args:
            robot_id: PyBullet robot body ID
            end_effector_link_index: Index of the end effector link
            joint_indices: List of joint indices to control
            max_iterations: Maximum number of IK iterations
            residual_threshold: Convergence threshold
        """
        self.robot_id = robot_id
        self.end_effector_link_index = end_effector_link_index
        self.joint_indices = joint_indices
        self.max_iterations = max_iterations
        self.residual_threshold = residual_threshold

    def solve_position_ik(self,
                         target_position: List[float],
                         target_orientation: Optional[List[float]] = None,
                         lower_limits: Optional[List[float]] = None,
                         upper_limits: Optional[List[float]] = None,
                         joint_ranges: Optional[List[float]] = None,
                         rest_poses: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        Solve inverse kinematics for a target position

        Args:
            target_position: Target 3D position [x, y, z]
            target_orientation: Target orientation quaternion [x, y, z, w] (optional)
            lower_limits: Joint lower limits
            upper_limits: Joint upper limits
            joint_ranges: Joint ranges (for null space projection)
            rest_poses: Rest poses (for null space projection)

        Returns:
            List of joint angles, or None if IK fails
        """
        if target_orientation is None:
            # Position-only IK
            kwargs = {
                'bodyUniqueId': self.robot_id,
                'endEffectorLinkIndex': self.end_effector_link_index,
                'targetPosition': target_position,
                'maxNumIterations': self.max_iterations,
                'residualThreshold': self.residual_threshold
            }

            # Add optional parameters
            if lower_limits is not None:
                kwargs['lowerLimits'] = lower_limits
            if upper_limits is not None:
                kwargs['upperLimits'] = upper_limits
            if joint_ranges is not None:
                kwargs['jointRanges'] = joint_ranges
            if rest_poses is not None:
                kwargs['restPoses'] = rest_poses

            joint_poses = p.calculateInverseKinematics(**kwargs)
        else:
            # Position and orientation IK
            kwargs = {
                'bodyUniqueId': self.robot_id,
                'endEffectorLinkIndex': self.end_effector_link_index,
                'targetPosition': target_position,
                'targetOrientation': target_orientation,
                'maxNumIterations': self.max_iterations,
                'residualThreshold': self.residual_threshold
            }

            # Add optional parameters
            if lower_limits is not None:
                kwargs['lowerLimits'] = lower_limits
            if upper_limits is not None:
                kwargs['upperLimits'] = upper_limits
            if joint_ranges is not None:
                kwargs['jointRanges'] = joint_ranges
            if rest_poses is not None:
                kwargs['restPoses'] = rest_poses

            joint_poses = p.calculateInverseKinematics(**kwargs)

        # PyBullet returns values for all joints in order
        # We need to extract only the joints we care about
        result = []
        for joint_idx in self.joint_indices:
            if joint_idx < len(joint_poses):
                result.append(joint_poses[joint_idx])
            else:
                # If index is out of range, use 0 as default
                result.append(0.0)

        return result


class LegIKSolver:
    """
    Specialized IK solver for a single leg of the Hunter robot

    This solver handles the 5-DOF leg kinematics
    """

    def __init__(self,
                 robot_id: int,
                 leg_name: str,  # "left" or "right"
                 joint_dict: Dict[str, int]):
        """
        Initialize leg IK solver

        Args:
            robot_id: PyBullet robot body ID
            leg_name: "left" or "right"
            joint_dict: Dictionary mapping joint names to indices
        """
        self.robot_id = robot_id
        self.leg_name = leg_name

        # Get joint names for this leg
        prefix = "leg_l" if leg_name == "left" else "leg_r"

        self.joint_names = [
            f"{prefix}1_joint",
            f"{prefix}2_joint",
            f"{prefix}3_joint",
            f"{prefix}4_joint",
            f"{prefix}5_joint"
        ]

        self.joint_indices = [joint_dict[name] for name in self.joint_names]

        # End effector is the foot link (leg_l5_link or leg_r5_link)
        # We need to find the link index, not joint index
        foot_link_name = f"{prefix}5_link"

        # Find link index by searching through all joints
        self.end_effector_index = self.joint_indices[-1]  # Default to last joint
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            link_name = joint_info[12].decode('utf-8')  # Child link name
            if link_name == foot_link_name:
                self.end_effector_index = i
                break

        # Get joint limits for ALL joints (PyBullet IK requires this)
        num_joints = p.getNumJoints(robot_id)
        all_lower_limits = []
        all_upper_limits = []
        all_joint_ranges = []
        all_rest_poses = []

        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            joint_type = joint_info[2]

            if joint_type == p.JOINT_REVOLUTE:
                lower = joint_info[8]
                upper = joint_info[9]
                all_lower_limits.append(lower)
                all_upper_limits.append(upper)
                all_joint_ranges.append(upper - lower)
                # Use straight-leg rest pose (updated 2025-11-23 for stability)
                # Straight legs are more stable and match our standing configuration
                all_rest_poses.append(0.0)
            else:
                # Fixed joints - use 0
                all_lower_limits.append(0.0)
                all_upper_limits.append(0.0)
                all_joint_ranges.append(0.0)
                all_rest_poses.append(0.0)

        self.lower_limits = all_lower_limits
        self.upper_limits = all_upper_limits
        self.joint_ranges = all_joint_ranges
        self.rest_poses = all_rest_poses

        # Create IK solver
        self.ik_solver = IKSolver(
            robot_id=robot_id,
            end_effector_link_index=self.end_effector_index,
            joint_indices=self.joint_indices
        )

    def solve(self,
             target_position: List[float],
             target_orientation: Optional[List[float]] = None) -> Dict[str, float]:
        """
        Solve IK for target foot position

        Args:
            target_position: Target foot position [x, y, z]
            target_orientation: Target orientation (optional)

        Returns:
            Dictionary mapping joint names to angles
        """
        joint_angles = self.ik_solver.solve_position_ik(
            target_position=target_position,
            target_orientation=target_orientation,
            lower_limits=self.lower_limits,
            upper_limits=self.upper_limits,
            joint_ranges=self.joint_ranges,
            rest_poses=self.rest_poses
        )

        if joint_angles is None:
            return {}

        # Map to joint names
        result = {}
        for i, joint_name in enumerate(self.joint_names):
            if i < len(joint_angles):
                result[joint_name] = joint_angles[i]

        return result

    def get_current_foot_position(self) -> np.ndarray:
        """Get current foot position in world coordinates"""
        link_state = p.getLinkState(self.robot_id, self.end_effector_index)
        return np.array(link_state[0])


class BipedalIKSolver:
    """
    IK solver for both legs of the bipedal robot

    Manages IK for left and right legs simultaneously
    """

    def __init__(self, robot_id: int, joint_dict: Dict[str, int]):
        """
        Initialize bipedal IK solver

        Args:
            robot_id: PyBullet robot body ID
            joint_dict: Dictionary mapping joint names to indices
        """
        self.robot_id = robot_id

        # Create IK solvers for each leg
        self.left_leg_ik = LegIKSolver(robot_id, "left", joint_dict)
        self.right_leg_ik = LegIKSolver(robot_id, "right", joint_dict)

    def solve_both_legs(self,
                       left_target: List[float],
                       right_target: List[float],
                       left_orientation: Optional[List[float]] = None,
                       right_orientation: Optional[List[float]] = None) -> Dict[str, float]:
        """
        Solve IK for both legs

        Args:
            left_target: Target position for left foot
            right_target: Target position for right foot
            left_orientation: Target orientation for left foot (optional)
            right_orientation: Target orientation for right foot (optional)

        Returns:
            Dictionary mapping all joint names to angles
        """
        left_angles = self.left_leg_ik.solve(left_target, left_orientation)
        right_angles = self.right_leg_ik.solve(right_target, right_orientation)

        # Combine results
        result = {}
        result.update(left_angles)
        result.update(right_angles)

        return result

    def solve_left_leg(self,
                      target_position: List[float],
                      target_orientation: Optional[List[float]] = None) -> Dict[str, float]:
        """Solve IK for left leg only"""
        return self.left_leg_ik.solve(target_position, target_orientation)

    def solve_right_leg(self,
                       target_position: List[float],
                       target_orientation: Optional[List[float]] = None) -> Dict[str, float]:
        """Solve IK for right leg only"""
        return self.right_leg_ik.solve(target_position, target_orientation)

    def get_foot_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current positions of both feet

        Returns:
            (left_foot_position, right_foot_position)
        """
        left_pos = self.left_leg_ik.get_current_foot_position()
        right_pos = self.right_leg_ik.get_current_foot_position()
        return left_pos, right_pos


if __name__ == "__main__":
    # This is a test that would require a running simulation
    print("IK Solver module loaded successfully")
    print("To test, run with a simulation environment")
