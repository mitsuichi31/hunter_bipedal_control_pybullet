"""
Position Control Walking Controller

Integrates:
- GaitGenerator: Foot trajectory planning
- CoMPlanner: ZMP-based CoM trajectory planning
- FullBodyIK: Whole-body IK solving

Uses pure position control (no torque control) for robust walking.

Author: Phase 4.3 Implementation
Date: 2025-11-25
"""

import numpy as np
import pybullet as p
from typing import Dict, Optional, Tuple
from dataclasses import dataclass

from gait_generator import GaitGenerator, GaitParams
from com_planner_simple import SimpleCoMPlanner2D, SimpleCoMPlannerParams
from full_body_ik import FullBodyIKSolver, FullBodyIKParams


@dataclass
class WalkingControllerParams:
    """Parameters for position control walking controller"""

    # Gait parameters
    gait: GaitParams = None

    # CoM planning parameters
    com_planning: SimpleCoMPlannerParams = None

    # IK parameters
    ik: FullBodyIKParams = None

    # Control modes
    standing_mode: bool = False  # If True, freeze at current position
    enable_walking: bool = True  # Enable walking gait

    # Safety
    max_com_velocity: float = 0.5  # m/s
    emergency_stop_tilt: float = 0.35  # rad (~20 degrees)

    def __post_init__(self):
        """Initialize sub-parameters if not provided"""
        if self.gait is None:
            self.gait = GaitParams(
                step_length=0.02,  # Very conservative: 2cm
                step_height=0.01,  # Very conservative: 1cm
                step_period=2.0,   # Very slow: 2 seconds per step
                stance_width=0.18,
                double_support_ratio=0.5  # 50% double support
            )

        if self.com_planning is None:
            self.com_planning = SimpleCoMPlannerParams()

        if self.ik is None:
            self.ik = FullBodyIKParams()


class PositionControlWalkingController:
    """
    Position Control Walking Controller

    Pipeline:
    1. GaitGenerator → foot trajectories
    2. Compute desired ZMP from gait state
    3. CoMPlanner → CoM trajectory to track ZMP
    4. FullBodyIK → solve for base + joint angles
    5. Return position commands
    """

    def __init__(self,
                 robot_id: int,
                 joint_dict: Dict[str, int],
                 params: Optional[WalkingControllerParams] = None):
        """
        Initialize position control walking controller

        Args:
            robot_id: PyBullet robot body ID
            joint_dict: Dictionary mapping joint names to indices
            params: Controller parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict
        self.params = params or WalkingControllerParams()

        # Initialize components
        self.gait_generator = GaitGenerator(self.params.gait)
        self.com_planner = SimpleCoMPlanner2D(self.params.com_planning)
        self.ik_solver = FullBodyIKSolver(robot_id, joint_dict, self.params.ik)

        # State
        self.time = 0.0
        self.last_ik_solution = None
        self.emergency_stop = False

        # Foot link indices
        self.left_foot_link = joint_dict['leg_l5_joint']
        self.right_foot_link = joint_dict['leg_r5_joint']

        print(f"[Position Control Walking] Initialized")
        print(f"  Standing mode: {self.params.standing_mode}")
        print(f"  Gait: {self.params.gait.step_length}m steps, {self.params.gait.step_period}s period")
        print(f"  Components: GaitGen + CoMPlanner + FullBodyIK")

    def reset(self):
        """Reset controller state"""
        self.time = 0.0
        self.gait_generator.reset()

        # Reset CoM planner to current state
        com_pos = self._compute_com()
        self.com_planner.reset(com_pos[:2], np.zeros(2))

        self.last_ik_solution = None
        self.emergency_stop = False

    def _compute_com(self) -> np.ndarray:
        """Compute current center of mass"""
        total_mass = 0.0
        com_pos = np.zeros(3)

        # Base
        base_mass = p.getDynamicsInfo(self.robot_id, -1)[0]
        base_pos = np.array(p.getBasePositionAndOrientation(self.robot_id)[0])
        total_mass += base_mass
        com_pos += base_mass * base_pos

        # Links
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            link_mass = p.getDynamicsInfo(self.robot_id, i)[0]
            if link_mass > 0:
                link_state = p.getLinkState(self.robot_id, i)
                link_pos = np.array(link_state[0])
                total_mass += link_mass
                com_pos += link_mass * link_pos

        return com_pos / total_mass

    def _get_contact_state(self) -> Tuple[bool, bool]:
        """
        Determine which feet are in contact

        Returns:
            (left_contact, right_contact) booleans
        """
        # Get foot heights
        left_foot_state = p.getLinkState(self.robot_id, self.left_foot_link)
        right_foot_state = p.getLinkState(self.robot_id, self.right_foot_link)

        left_height = left_foot_state[0][2]  # Z position
        right_height = right_foot_state[0][2]

        # Consider in contact if within 2cm of ground
        contact_threshold = 0.02
        left_contact = left_height < contact_threshold
        right_contact = right_height < contact_threshold

        # During standing, both feet always in contact
        if self.params.standing_mode:
            return (True, True)

        return (left_contact, right_contact)

    def _compute_desired_zmp(self,
                            left_foot_pos: np.ndarray,
                            right_foot_pos: np.ndarray,
                            contacts: Tuple[bool, bool]) -> np.ndarray:
        """
        Compute desired ZMP position based on gait state

        During double support: ZMP between feet
        During single support: ZMP at stance foot

        Args:
            left_foot_pos: Left foot target position
            right_foot_pos: Right foot target position
            contacts: (left_contact, right_contact)

        Returns:
            Desired ZMP [x, y] in world frame
        """
        left_contact, right_contact = contacts

        if left_contact and right_contact:
            # Double support: ZMP between feet
            # Weight based on gait phase for smooth transition
            phase = self.gait_generator.phase

            # Compute weight (smoother transition during double support)
            if phase < np.pi:
                # First half: transitioning from right to left
                weight = 0.5 + 0.5 * np.sin(phase - np.pi/2)
            else:
                # Second half: transitioning from left to right
                weight = 0.5 + 0.5 * np.sin(phase - 3*np.pi/2)

            zmp = weight * left_foot_pos[:2] + (1 - weight) * right_foot_pos[:2]

        elif left_contact:
            # Left stance only
            zmp = left_foot_pos[:2]

        elif right_contact:
            # Right stance only
            zmp = right_foot_pos[:2]

        else:
            # No contact (shouldn't happen, but handle gracefully)
            # Use midpoint
            zmp = (left_foot_pos[:2] + right_foot_pos[:2]) / 2

        return zmp

    def _check_safety(self) -> bool:
        """
        Check safety conditions

        Returns:
            True if safe to continue, False if emergency stop needed
        """
        # Check base orientation
        _, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(base_orn)
        roll, pitch = abs(euler[0]), abs(euler[1])

        if roll > self.params.emergency_stop_tilt or pitch > self.params.emergency_stop_tilt:
            print(f"[Walking] EMERGENCY STOP: Excessive tilt (Roll={np.degrees(roll):.1f}°, Pitch={np.degrees(pitch):.1f}°)")
            return False

        return True

    def update(self, dt: float) -> Optional[Dict]:
        """
        Update walking controller

        Args:
            dt: Time step (seconds)

        Returns:
            Dictionary of joint position commands, or None if emergency stop
        """
        # Safety check
        if not self._check_safety():
            self.emergency_stop = True
            return None

        # Update time
        self.time += dt

        # Standing mode: return fixed position
        if self.params.standing_mode:
            return self._get_standing_commands()

        # 1. Get foot trajectories from gait generator
        self.gait_generator.update(dt)
        left_foot_target, right_foot_target = self.gait_generator.get_foot_trajectories(self.time)

        # Convert to world frame (add current base position offset)
        base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        left_foot_target_world = left_foot_target + np.array([base_pos[0], base_pos[1], 0])
        right_foot_target_world = right_foot_target + np.array([base_pos[0], base_pos[1], 0])

        # 2. Determine contact state
        contacts = self._get_contact_state()

        # 3. Compute desired ZMP
        zmp_desired = self._compute_desired_zmp(left_foot_target_world, right_foot_target_world, contacts)

        # 4. Plan CoM trajectory to track ZMP
        com_pos, com_vel, com_acc = self.com_planner.compute_com_command(zmp_desired)
        com_target = np.array([com_pos[0], com_pos[1], self.params.ik.com_height])

        # 5. Solve full-body IK
        ik_solution = self.ik_solver.solve(
            left_foot_target=left_foot_target_world,
            right_foot_target=right_foot_target_world,
            com_target=com_target,
            left_contact=contacts[0],
            right_contact=contacts[1],
            initial_config=None  # Will use current state
        )

        if ik_solution is None:
            print(f"[Walking] Warning: IK failed at t={self.time:.2f}s")
            # Use last successful solution if available
            if self.last_ik_solution is not None:
                ik_solution = self.last_ik_solution
            else:
                return self._get_standing_commands()

        self.last_ik_solution = ik_solution

        # 6. Return position commands
        return ik_solution['joint_angles']

    def _get_standing_commands(self) -> Dict:
        """Get standing position commands (straight legs)"""
        standing_config = {
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
        return standing_config
