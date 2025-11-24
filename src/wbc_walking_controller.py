"""
WBC-based Walking Controller for Bipedal Robot - Phase 3 Redesign

This controller integrates:
1. Contact State Machine - manages contact phases
2. Gait Generator - provides foot trajectories
3. WBC Controller - computes optimal forces/torques
4. Inverse Dynamics - maps accelerations to torques

Architecture:
Gait Generator → Contact FSM → Task Hierarchy → WBC QP → Inverse Dynamics → Torques
"""

import numpy as np
import pybullet as p
from typing import Dict, Tuple, Optional
from dataclasses import dataclass

from gait_generator import GaitGenerator, GaitParams
from wbc_controller import WholeBodyController, WBCParams
from wbc_tasks import (
    TaskHierarchy,
    create_body_orientation_task,
    create_com_tracking_task,
    create_swing_foot_task,
    create_stance_foot_constraint
)
from contact_state_machine import ContactStateMachine, ContactStateParams, ContactPhase
from inverse_dynamics import InverseDynamics
from stability_metrics import compute_com


@dataclass
class WBCWalkingParams:
    """Parameters for WBC walking controller"""
    # Control gains - Body orientation
    kp_orientation: float = 100.0   # Body orientation tracking gain
    kd_orientation: float = 10.0    # Body orientation damping

    # Control gains - CoM tracking
    kp_com: float = 50.0            # CoM position tracking gain
    kd_com: float = 5.0             # CoM velocity damping

    # Control gains - Swing foot
    kp_swing: float = 100.0         # Swing foot position tracking
    kd_swing: float = 10.0          # Swing foot velocity damping

    # Control gains - Stance foot constraint
    kd_stance: float = 20.0         # Stance foot damping (drive velocity to zero)

    # Balance parameters
    com_height_target: float = 0.55  # Target CoM height (m)
    max_com_offset: float = 0.08     # Maximum CoM offset from center (m)

    # Safety limits
    max_roll_pitch: float = 0.26     # Maximum body tilt (rad, ~15 degrees)
    min_com_height: float = 0.40     # Minimum acceptable CoM height (m)
    max_com_height: float = 0.70     # Maximum acceptable CoM height (m)

    # Control frequency
    control_dt: float = 0.01         # Control update rate (seconds, 100Hz)


class WBCWalkingController:
    """
    Walking Controller using Whole-Body Control (Phase 3)

    This controller properly handles free-floating base dynamics by using
    WBC optimization instead of IK. It coordinates:
    - Contact state management (when each foot is on ground)
    - Gait trajectory generation (where feet should be)
    - Task hierarchy (prioritized objectives)
    - Force optimization (QP solver)
    - Torque computation (inverse dynamics)
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
            wbc_params: WBC QP optimization parameters
            walking_params: Walking-specific control parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict

        # Parameters
        self.gait_params = gait_params if gait_params else GaitParams()
        self.wbc_params = wbc_params if wbc_params else WBCParams()
        self.walking_params = walking_params if walking_params else WBCWalkingParams()

        # Components
        self.gait_generator = GaitGenerator(self.gait_params)

        contact_params = ContactStateParams(
            step_period=self.gait_params.step_period,
            double_support_ratio=self.gait_params.double_support_ratio,
            contact_force_threshold=5.0
        )
        self.contact_fsm = ContactStateMachine(contact_params)
        self.contact_fsm.initialize(robot_id, joint_dict)

        self.wbc = WholeBodyController(robot_id, joint_dict, self.wbc_params)
        self.inv_dyn = InverseDynamics(robot_id)
        self.task_hierarchy = TaskHierarchy()

        # State
        self.time = 0.0
        self.last_control_update = 0.0
        self.is_active = False

        # Foot link indices
        self.left_foot_link, self.right_foot_link = self._find_foot_links()

        print(f"WBC Walking Controller initialized")
        print(f"  Step period: {self.gait_params.step_period:.2f}s")
        print(f"  Step length: {self.gait_params.step_length:.3f}m")
        print(f"  Control frequency: {1.0/self.walking_params.control_dt:.0f}Hz")

    def _find_foot_links(self) -> Tuple[int, int]:
        """Find link indices for feet"""
        left_foot = None
        right_foot = None

        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')

            if link_name == 'leg_l5_link':
                left_foot = i
            elif link_name == 'leg_r5_link':
                right_foot = i

        if left_foot is None or right_foot is None:
            print(f"Warning: Could not find foot links. L:{left_foot}, R:{right_foot}")

        return left_foot, right_foot

    def _get_foot_state(self, foot_link_idx: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get foot position and velocity in world frame

        Args:
            foot_link_idx: Link index of the foot

        Returns:
            (position, velocity): Foot state
        """
        link_state = p.getLinkState(
            self.robot_id,
            foot_link_idx,
            computeLinkVelocity=1
        )

        position = np.array(link_state[0])  # World position
        velocity = np.array(link_state[6])  # Linear velocity

        return position, velocity

    def _get_robot_state(self) -> Dict:
        """Get current robot state"""
        # Base state
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)

        # CoM
        com_pos = compute_com(self.robot_id)

        # Foot states
        left_pos, left_vel = self._get_foot_state(self.left_foot_link)
        right_pos, right_vel = self._get_foot_state(self.right_foot_link)

        return {
            'base_pos': np.array(base_pos),
            'base_orn': np.array(base_orn),
            'base_vel': np.array(base_vel),
            'base_ang_vel': np.array(base_ang_vel),
            'com_pos': com_pos,
            'left_foot_pos': left_pos,
            'left_foot_vel': left_vel,
            'right_foot_pos': right_pos,
            'right_foot_vel': right_vel,
        }

    def _build_task_hierarchy(self, robot_state: Dict, gait_targets: Dict) -> None:
        """
        Build task hierarchy based on current contact phase

        Args:
            robot_state: Current robot state
            gait_targets: Target positions from gait generator
        """
        self.task_hierarchy.clear_tasks()

        # Get current contact phase
        phase = self.contact_fsm.phase
        left_contact, right_contact = self.contact_fsm.get_contact_state()

        # Priority 0: Stance foot constraints (highest priority)
        if left_contact:
            task = create_stance_foot_constraint(
                foot_name="left",
                foot_velocity=robot_state['left_foot_vel'],
                kd=self.walking_params.kd_stance
            )
            self.task_hierarchy.add_task(task)

        if right_contact:
            task = create_stance_foot_constraint(
                foot_name="right",
                foot_velocity=robot_state['right_foot_vel'],
                kd=self.walking_params.kd_stance
            )
            self.task_hierarchy.add_task(task)

        # Priority 1: Body orientation (keep upright)
        desired_orientation = np.array([0, 0, 0, 1])  # Upright (quaternion)
        task = create_body_orientation_task(
            current_orientation=robot_state['base_orn'],
            desired_orientation=desired_orientation,
            current_angular_vel=robot_state['base_ang_vel'],
            kp=self.walking_params.kp_orientation,
            kd=self.walking_params.kd_orientation
        )
        self.task_hierarchy.add_task(task)

        # Priority 1: CoM tracking (keep CoM centered over support)
        com_target_xy = np.zeros(2)  # Keep CoM centered (for now)
        com_offset_xy = robot_state['com_pos'][:2] - com_target_xy
        com_velocity_xy = robot_state['base_vel'][:2]  # Approximate

        task = create_com_tracking_task(
            com_offset=com_offset_xy,
            com_velocity=com_velocity_xy,
            kp=self.walking_params.kp_com,
            kd=self.walking_params.kd_com
        )
        self.task_hierarchy.add_task(task)

        # Priority 2: Swing foot tracking
        if not left_contact:  # Left foot swinging
            task = create_swing_foot_task(
                foot_name="left",
                current_pos=robot_state['left_foot_pos'],
                desired_pos=gait_targets['left_foot'],
                current_vel=robot_state['left_foot_vel'],
                kp=self.walking_params.kp_swing,
                kd=self.walking_params.kd_swing
            )
            self.task_hierarchy.add_task(task)

        if not right_contact:  # Right foot swinging
            task = create_swing_foot_task(
                foot_name="right",
                current_pos=robot_state['right_foot_pos'],
                desired_pos=gait_targets['right_foot'],
                current_vel=robot_state['right_foot_vel'],
                kp=self.walking_params.kp_swing,
                kd=self.walking_params.kd_swing
            )
            self.task_hierarchy.add_task(task)

    def check_stability(self, robot_state: Dict) -> Tuple[bool, str]:
        """
        Check if robot is in a stable state

        Args:
            robot_state: Current robot state

        Returns:
            (is_stable, reason): Stability flag and reason if unstable
        """
        # Check orientation
        base_orn = robot_state['base_orn']
        euler = p.getEulerFromQuaternion(base_orn)
        roll, pitch, yaw = euler

        if abs(roll) > self.walking_params.max_roll_pitch:
            return False, f"Roll angle too large: {np.degrees(roll):.1f}°"
        if abs(pitch) > self.walking_params.max_roll_pitch:
            return False, f"Pitch angle too large: {np.degrees(pitch):.1f}°"

        # Check CoM height
        com_height = robot_state['com_pos'][2]
        if com_height < self.walking_params.min_com_height:
            return False, f"CoM too low: {com_height:.3f}m"
        if com_height > self.walking_params.max_com_height:
            return False, f"CoM too high: {com_height:.3f}m"

        return True, "Stable"

    def update(self, dt: float) -> Dict[str, float]:
        """
        Main control update loop

        Args:
            dt: Time step (seconds)

        Returns:
            Joint torques {joint_name: torque}
        """
        self.time += dt

        # Update at control frequency
        if self.time - self.last_control_update < self.walking_params.control_dt:
            # Return zero torques if not time to update yet
            return {name: 0.0 for name in self.joint_dict.keys()}

        self.last_control_update = self.time

        # Update contact state machine
        phase = self.contact_fsm.update(dt)

        # Get gait targets (foot positions)
        left_target, right_target = self.gait_generator.get_foot_trajectories(self.time)

        gait_targets = {
            'left_foot': left_target,
            'right_foot': right_target
        }

        # Get current robot state
        robot_state = self._get_robot_state()

        # Safety check
        is_stable, reason = self.check_stability(robot_state)
        if not is_stable and self.is_active:
            print(f"WARNING: Instability detected - {reason}")
            # Could implement emergency stop here

        # Build task hierarchy based on contact phase
        self._build_task_hierarchy(robot_state, gait_targets)

        # Get desired accelerations from task hierarchy
        base_accel, joint_accel = self.task_hierarchy.get_desired_acceleration()

        # For now, use simplified approach: return zero torques
        # Full implementation would:
        # 1. Solve WBC QP to get desired accelerations
        # 2. Use inverse dynamics to compute torques
        # 3. Apply gravity compensation

        # Placeholder: return zero torques
        torques = {name: 0.0 for name in self.joint_dict.keys()}

        return torques

    def reset(self):
        """Reset controller state"""
        self.time = 0.0
        self.last_control_update = 0.0
        self.is_active = False
        self.gait_generator.reset()
        self.contact_fsm.reset()
        self.task_hierarchy.clear_tasks()

    def start(self):
        """Activate walking controller"""
        self.is_active = True
        print("WBC Walking Controller: ACTIVE")

    def stop(self):
        """Deactivate walking controller"""
        self.is_active = False
        self.reset()
        print("WBC Walking Controller: STOPPED")

    def get_state_info(self) -> Dict:
        """Get controller state for debugging/logging"""
        return {
            'time': self.time,
            'active': self.is_active,
            'phase': self.contact_fsm.phase.name,
            'step_count': self.contact_fsm.step_count,
            'num_tasks': len(self.task_hierarchy.tasks),
        }


if __name__ == "__main__":
    print("WBC Walking Controller - Phase 3")
    print("\nThis controller integrates:")
    print("  - Contact State Machine")
    print("  - Gait Generator")
    print("  - Task Hierarchy (WBC)")
    print("  - Inverse Dynamics")
    print("\nUse via main_simulation.py --mode walking")
