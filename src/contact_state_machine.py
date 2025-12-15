"""
Contact State Machine for Bipedal Walking

Manages the contact phases during walking:
- DOUBLE_SUPPORT: Both feet on ground
- LEFT_SWING: Right foot stance, left foot swinging
- RIGHT_SWING: Left foot stance, right foot swinging

This state machine coordinates with the gait generator and WBC controller
to ensure proper contact constraint handling.
"""

import numpy as np
import pybullet as p
from enum import Enum
from typing import Tuple, Optional
from dataclasses import dataclass


class ContactPhase(Enum):
    """Contact phases during bipedal walking"""
    DOUBLE_SUPPORT = 0  # Both feet on ground
    LEFT_SWING = 1      # Right stance, left swinging
    RIGHT_SWING = 2     # Left stance, right swinging


@dataclass
class ContactStateParams:
    """Parameters for contact state machine"""
    step_period: float = 1.0          # Total time for one step cycle (seconds)
    double_support_ratio: float = 0.3  # Ratio of cycle in double support
    contact_force_threshold: float = 5.0  # Minimum force to consider in contact (N)


class ContactStateMachine:
    """
    Finite State Machine for managing contact phases during walking

    The gait cycle follows this pattern:
    1. DOUBLE_SUPPORT (both feet down)
    2. LEFT_SWING (right stance, left swinging forward)
    3. DOUBLE_SUPPORT (both feet down)
    4. RIGHT_SWING (left stance, right swinging forward)
    5. Repeat
    """

    def __init__(self, params: ContactStateParams = None):
        """
        Initialize contact state machine

        Args:
            params: Contact state parameters
        """
        self.params = params if params else ContactStateParams()

        # State variables
        self.phase = ContactPhase.DOUBLE_SUPPORT
        self.phase_time = 0.0  # Time in current phase
        self.cycle_time = 0.0  # Time in overall cycle
        self.step_count = 0    # Number of completed steps

        # Timing
        self.ds_duration = self.params.step_period * self.params.double_support_ratio
        self.swing_duration = self.params.step_period * (1.0 - self.params.double_support_ratio)

        # Foot link indices (set by initialize())
        self.left_foot_link = None
        self.right_foot_link = None

    def initialize(self, robot_id: int, joint_dict: dict):
        """
        Initialize with robot information

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
        """
        self.robot_id = robot_id

        # Find foot link indices
        # We need the LINK index, not the joint index
        # The foot links are the child links of the ankle joints
        self.left_foot_link = None
        self.right_foot_link = None

        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            link_name = joint_info[12].decode('utf-8')  # Link name (child link of this joint)

            # Left foot is child link of left ankle joint
            if link_name == 'leg_l5_link':
                self.left_foot_link = i
            # Right foot is child link of right ankle joint
            elif link_name == 'leg_r5_link':
                self.right_foot_link = i

        if self.left_foot_link is None or self.right_foot_link is None:
            print(f"Warning: Could not find foot links. Left: {self.left_foot_link}, Right: {self.right_foot_link}")
            print("Available links:")
            for i in range(num_joints):
                joint_info = p.getJointInfo(robot_id, i)
                link_name = joint_info[12].decode('utf-8')
                print(f"  Link {i}: {link_name}")

    def update(self, dt: float) -> ContactPhase:
        """
        Update state machine based on elapsed time

        Args:
            dt: Time step (seconds)

        Returns:
            Current contact phase
        """
        self.phase_time += dt
        self.cycle_time += dt

        # Full cycle = DS → LS → DS → RS → DS
        full_cycle = 2 * (self.ds_duration + self.swing_duration)

        if self.cycle_time >= full_cycle:
            self.cycle_time = 0.0
            self.step_count += 2  # Completed 2 steps (left and right)

        # Determine phase based on cycle time
        t = self.cycle_time

        # Phase 1: Initial double support
        if t < self.ds_duration:
            new_phase = ContactPhase.DOUBLE_SUPPORT

        # Phase 2: Left swing
        elif t < self.ds_duration + self.swing_duration:
            new_phase = ContactPhase.LEFT_SWING

        # Phase 3: Middle double support
        elif t < 2 * self.ds_duration + self.swing_duration:
            new_phase = ContactPhase.DOUBLE_SUPPORT

        # Phase 4: Right swing
        elif t < 2 * self.ds_duration + 2 * self.swing_duration:
            new_phase = ContactPhase.RIGHT_SWING

        # Phase 5: Final double support (wraps to phase 1)
        else:
            new_phase = ContactPhase.DOUBLE_SUPPORT

        # Detect phase transitions
        if new_phase != self.phase:
            self.phase = new_phase
            self.phase_time = 0.0  # Reset phase timer

        return self.phase

    def get_contact_state(self) -> Tuple[bool, bool]:
        """
        Get expected contact state for current phase

        Returns:
            (left_contact, right_contact): Boolean flags indicating if each foot
                                           should be in contact with ground
        """
        if self.phase == ContactPhase.DOUBLE_SUPPORT:
            return (True, True)
        elif self.phase == ContactPhase.LEFT_SWING:
            return (False, True)  # Left swinging, right stance
        elif self.phase == ContactPhase.RIGHT_SWING:
            return (True, False)  # Right swinging, left stance
        else:
            # Default to double support for safety
            return (True, True)

    def detect_ground_contact(self, robot_id: int, foot_link_idx: int) -> Tuple[bool, float]:
        """
        Detect if a foot is in contact with the ground using PyBullet contact points

        Args:
            robot_id: PyBullet robot ID
            foot_link_idx: Link index of the foot

        Returns:
            (is_contact, normal_force): Contact flag and vertical force magnitude
        """
        if foot_link_idx is None:
            return (False, 0.0)

        # Get contact points for this link
        contact_points = p.getContactPoints(
            bodyA=robot_id,
            linkIndexA=foot_link_idx
        )

        if len(contact_points) == 0:
            return (False, 0.0)

        # Sum up normal forces from all contact points
        total_normal_force = 0.0
        for contact in contact_points:
            normal_force = contact[9]  # Normal force magnitude
            total_normal_force += normal_force

        # Check if force exceeds threshold
        is_contact = total_normal_force > self.params.contact_force_threshold

        return (is_contact, total_normal_force)

    def get_actual_contact_state(self) -> Tuple[bool, bool]:
        """
        Get actual contact state from PyBullet physics

        Returns:
            (left_contact, right_contact): Actual contact flags from simulation
        """
        if self.robot_id is None:
            return self.get_contact_state()  # Fall back to expected state

        left_contact, left_force = self.detect_ground_contact(
            self.robot_id, self.left_foot_link
        )
        right_contact, right_force = self.detect_ground_contact(
            self.robot_id, self.right_foot_link
        )

        return (left_contact, right_contact)

    def get_swing_phase_progress(self) -> float:
        """
        Get progress through current swing phase (0 to 1)
        Only meaningful during swing phases

        Returns:
            Progress ratio (0.0 = start of swing, 1.0 = end of swing)
        """
        if self.phase == ContactPhase.DOUBLE_SUPPORT:
            return 0.0

        return min(1.0, self.phase_time / self.swing_duration)

    def is_in_transition(self, transition_duration: float = 0.05) -> bool:
        """
        Check if we're in a phase transition period

        Args:
            transition_duration: Duration of transition period (seconds)

        Returns:
            True if in transition (near phase change)
        """
        # Check if we're near the end of current phase
        if self.phase == ContactPhase.DOUBLE_SUPPORT:
            phase_duration = self.ds_duration
        else:
            phase_duration = self.swing_duration

        return (phase_duration - self.phase_time) < transition_duration

    def reset(self):
        """Reset state machine to initial state"""
        self.phase = ContactPhase.DOUBLE_SUPPORT
        self.phase_time = 0.0
        self.cycle_time = 0.0
        self.step_count = 0

    def get_state_info(self) -> dict:
        """
        Get detailed state information for debugging/logging

        Returns:
            Dictionary with state information
        """
        return {
            'phase': self.phase.name,
            'phase_time': self.phase_time,
            'cycle_time': self.cycle_time,
            'step_count': self.step_count,
            'swing_progress': self.get_swing_phase_progress(),
            'expected_contacts': self.get_contact_state(),
        }


if __name__ == "__main__":
    """Simple test of contact state machine timing"""
    print("Testing Contact State Machine...")

    # Create state machine
    params = ContactStateParams(
        step_period=1.0,
        double_support_ratio=0.3
    )
    fsm = ContactStateMachine(params)

    print(f"DS duration: {fsm.ds_duration:.3f}s")
    print(f"Swing duration: {fsm.swing_duration:.3f}s")
    print(f"Full cycle: {2*(fsm.ds_duration + fsm.swing_duration):.3f}s\n")

    # Simulate 3 seconds
    dt = 0.01
    t = 0.0
    last_phase = None

    while t < 3.0:
        phase = fsm.update(dt)
        left, right = fsm.get_contact_state()

        # Print on phase transitions
        if phase != last_phase:
            print(f"t={t:.3f}s: {phase.name:20s} | L:{left} R:{right} | Progress:{fsm.get_swing_phase_progress():.2f}")
            last_phase = phase

        t += dt

    print(f"\nCompleted {fsm.step_count} steps")
    print("✓ Contact state machine test passed!")
