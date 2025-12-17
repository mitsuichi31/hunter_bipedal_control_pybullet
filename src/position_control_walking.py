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
from robot_constants import STANDING_CONFIG, standing_config_copy
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
    estimator_cutoff_hz: float = 8.0  # low-pass cutoff for CoM estimation
    zmp_feedback_gain: float = 0.1  # feedback gain on ZMP error (0 = open-loop)
    zmp_correction_limit: float = 0.05  # max ZMP correction (meters)

    def __post_init__(self):
        """Initialize sub-parameters if not provided"""
        if self.gait is None:
            self.gait = GaitParams(
                step_length=0.04,  # More assertive: 4cm
                step_height=0.01,  # Moderate clearance: 1.0cm
                step_period=2.0,   # Slower cadence: 2.0 seconds per step
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
        self.reference_xy = np.zeros(2)  # Nominal world-frame walking reference
        self._desired_forward_velocity = 0.0
        self._next_diag_print = 0.0
        self._lateral_bias_limit = 0.08

        # State estimation (CoM)
        self.filtered_com_pos = np.zeros(3)
        self.filtered_com_vel = np.zeros(3)
        self.filtered_com_acc = np.zeros(3)
        self._prev_com_measurement = None
        self._prev_filtered_vel = None
        self._prev_raw_vel = None

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

        base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        self.reference_xy = np.array(base_pos[:2])
        if self.params.gait.step_period > 1e-6:
            self._desired_forward_velocity = self.params.gait.step_length / self.params.gait.step_period
        else:
            self._desired_forward_velocity = 0.0
        self._next_diag_print = 0.0
        self._lateral_bias_limit = 0.08

        # Reset CoM planner to current state
        com_pos = self._compute_com()
        self.com_planner.reset(com_pos[:2], np.zeros(2))

        # Reset estimator
        self.filtered_com_pos = com_pos
        self.filtered_com_vel = np.zeros(3)
        self.filtered_com_acc = np.zeros(3)
        self._prev_com_measurement = com_pos
        self._prev_filtered_vel = np.zeros(3)
        self._prev_raw_vel = np.zeros(3)

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
        # During standing, both feet always in contact
        if self.params.standing_mode:
            return (True, True)

        # Phase-based contact schedule (aligns with gait generator)
        phase = self.gait_generator.phase % (2.0 * np.pi)
        swing_dur = np.pi * max(0.05, 1.0 - self.params.gait.double_support_ratio)
        swing_start = (np.pi - swing_dur) / 2.0
        swing_end = swing_start + swing_dur

        def in_stance(leg_phase: float) -> bool:
            leg_phase = leg_phase % (2.0 * np.pi)
            return not (swing_start <= leg_phase < swing_end)

        left_contact = in_stance(phase)
        right_contact = in_stance(phase + np.pi)  # out of phase

        # Height-based override: if a foot is clearly on ground, mark contact
        contact_threshold = 0.02
        left_height = p.getLinkState(self.robot_id, self.left_foot_link)[0][2]
        right_height = p.getLinkState(self.robot_id, self.right_foot_link)[0][2]
        if left_height < contact_threshold:
            left_contact = True
        if right_height < contact_threshold:
            right_contact = True

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

    def _update_state_estimate(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Low-pass filter CoM position and velocity for feedback-ready estimates.

        Args:
            dt: Timestep (seconds)

        Returns:
            (filtered_position, filtered_velocity) as 3D vectors
        """
        measured_com = self._compute_com()

        if self._prev_com_measurement is None or dt <= 0:
            self.filtered_com_pos = measured_com
            self.filtered_com_vel = np.zeros(3)
            self.filtered_com_acc = np.zeros(3)
            self._prev_com_measurement = measured_com
            self._prev_filtered_vel = np.zeros(3)
            self._prev_raw_vel = np.zeros(3)
            return self.filtered_com_pos, self.filtered_com_vel

        raw_vel = (measured_com - self._prev_com_measurement) / dt
        raw_acc = (raw_vel - self._prev_raw_vel) / dt

        # Exponential smoothing coefficient based on cutoff frequency
        alpha = 1.0 - np.exp(-dt * 2 * np.pi * self.params.estimator_cutoff_hz)
        alpha = np.clip(alpha, 0.0, 1.0)

        self.filtered_com_pos = (1 - alpha) * self.filtered_com_pos + alpha * measured_com
        self.filtered_com_vel = (1 - alpha) * self.filtered_com_vel + alpha * raw_vel
        self.filtered_com_acc = (1 - alpha) * self.filtered_com_acc + alpha * raw_acc

        self._prev_com_measurement = measured_com
        self._prev_filtered_vel = self.filtered_com_vel.copy()
        self._prev_raw_vel = raw_vel
        return self.filtered_com_pos, self.filtered_com_vel

    def _estimate_zmp_actual(self) -> np.ndarray:
        """
        Estimate actual ZMP from filtered CoM state using LIPM relation.

        Returns:
            Estimated ZMP [x, y]
        """
        omega2 = self.params.com_planning.omega ** 2
        # p = x - ẍ / ω²
        return self.filtered_com_pos[:2] - self.filtered_com_acc[:2] / omega2

    def get_state_estimate(self) -> Dict[str, np.ndarray]:
        """Expose latest filtered CoM estimate for diagnostics/tests."""
        return {
            "com_pos": self.filtered_com_pos.copy(),
            "com_vel": self.filtered_com_vel.copy(),
            "com_acc": self.filtered_com_acc.copy(),
            "zmp_actual": self._estimate_zmp_actual(),
        }

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

        # Current base state for reference anchoring and diagnostics
        base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)

        # Advance nominal walking frame using desired forward velocity to avoid
        # targets chasing instantaneous base drift.
        if self.params.enable_walking:
            self.reference_xy[0] += self._desired_forward_velocity * dt
            # Integrate lateral bias to recenter base y around 0 in the reference frame
            lat_gain = 1.5
            self.reference_xy[1] += (-lat_gain * base_pos[1]) * dt
            self.reference_xy[1] = np.clip(self.reference_xy[1],
                                           -self._lateral_bias_limit,
                                           self._lateral_bias_limit)

        # Update state estimate (CoM) for feedback/planner sync
        com_pos_est, com_vel_est = self._update_state_estimate(dt)

        # Standing mode: return fixed position
        if self.params.standing_mode:
            return self._get_standing_commands()

        # 1. Get foot trajectories from gait generator
        self.gait_generator.update(dt)
        left_foot_target, right_foot_target = self.gait_generator.get_foot_trajectories(self.time)

        # Convert to world frame using nominal walking reference (decoupled from instantaneous base drift)
        walk_frame = np.array([self.reference_xy[0], self.reference_xy[1], 0.0])
        left_foot_target_world = left_foot_target + walk_frame
        right_foot_target_world = right_foot_target + walk_frame

        # 2. Determine contact state
        contacts = self._get_contact_state()
        # Safety fallback: if no contacts detected, assume double support so feet are driven to the ground
        if not any(contacts):
            contacts = (True, True)

        # 3. Compute desired ZMP
        zmp_desired = self._compute_desired_zmp(left_foot_target_world, right_foot_target_world, contacts)

        # Feedback: adjust ZMP target based on estimated ZMP error
        zmp_actual = self._estimate_zmp_actual()
        zmp_error = zmp_desired - zmp_actual
        zmp_correction = self.params.zmp_feedback_gain * zmp_error
        zmp_correction = np.clip(zmp_correction,
                                 -self.params.zmp_correction_limit,
                                 self.params.zmp_correction_limit)
        zmp_command = zmp_desired + zmp_correction

        # Lateral recentering (simple): bias ZMP toward midline based on base_y
        lateral_bias_gain = 0.20
        lateral_bias_limit = 0.04
        base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        lat_bias = -lateral_bias_gain * base_pos[1]
        lat_bias = np.clip(lat_bias, -lateral_bias_limit, lateral_bias_limit)
        zmp_command[1] += lat_bias

        # Yaw/lateral straightening: nudge swing foot targets toward midline/opposite drift
        yaw_correction_gain = 0.05   # Heading correction (reduced)
        lateral_correction_gain = 0.10  # Soft lateral drift correction
        forward_yaw_gain = 0.0      # Disable x-shift from yaw for simplicity
        forward_lat_gain = 0.05     # Mild forward foot bias from lateral error
        yaw_to_y_gain = 0.02        # Small yaw-driven lateral foot shift to straighten heading
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_yaw = p.getEulerFromQuaternion(base_orn)[2]
        yaw_shift = -yaw_correction_gain * base_yaw
        lat_shift = -lateral_correction_gain * base_pos[1]
        # Apply corrections only to swing feet
        left_contact, right_contact = contacts
        # Enforce symmetric lateral anchors during swing around stance width midline
        swing_y = self.params.gait.stance_width / 2.0
        if not left_contact:
            left_foot_target_world = np.array([left_foot_target_world[0] + forward_yaw_gain * (-base_yaw),
                                               swing_y + yaw_shift + lat_shift,
                                               left_foot_target_world[2]])
            left_foot_target_world[0] += -forward_lat_gain * base_pos[1]
        if not right_contact:
            right_foot_target_world = np.array([right_foot_target_world[0] + forward_yaw_gain * (-base_yaw),
                                                -swing_y + yaw_shift + lat_shift,
                                                right_foot_target_world[2]])
            right_foot_target_world[0] += -forward_lat_gain * base_pos[1]

        # Lightweight diagnostics to debug drift/target alignment
        if self.time >= self._next_diag_print and self.time <= 15.0:
            print(
                "[Walking diag]"
                f" t={self.time:5.2f}s"
                f" base=({base_pos[0]:+.3f},{base_pos[1]:+.3f})"
                f" ref=({self.reference_xy[0]:+.3f},{self.reference_xy[1]:+.3f})"
                f" foot_x=({left_foot_target_world[0]:+.3f},{right_foot_target_world[0]:+.3f})"
                f" zmp=({zmp_command[0]:+.3f},{zmp_command[1]:+.3f})"
            )
            self._next_diag_print += 1.0

        # 4. Plan CoM trajectory to track ZMP (anchor planner state to estimate)
        self.com_planner.planner_x.com_pos = com_pos_est[0]
        self.com_planner.planner_y.com_pos = com_pos_est[1]
        self.com_planner.planner_x.com_vel = com_vel_est[0]
        self.com_planner.planner_y.com_vel = com_vel_est[1]

        com_pos, com_vel, com_acc = self.com_planner.compute_com_command(zmp_command)
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
        return standing_config_copy()
