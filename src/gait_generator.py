"""
Gait trajectory generator for bipedal walking

Generates simple sinusoidal-based foot trajectories with ZMP consideration
"""

import numpy as np
from typing import Tuple, Dict
from dataclasses import dataclass


@dataclass
class GaitParams:
    """Parameters for gait generation"""
    step_length: float = 0.1      # Step length (m)
    step_height: float = 0.05      # Foot lift height (m)
    step_period: float = 1.0       # Time for one step (s)
    stance_width: float = 0.18     # Distance between feet (m)
    body_height: float = 0.45      # Nominal body height (m)
    double_support_ratio: float = 0.2  # Ratio of double support phase


class GaitGenerator:
    """
    Simple gait generator using sinusoidal trajectories

    Generates foot positions for walking based on time and gait parameters
    """

    def __init__(self, params: GaitParams = None):
        """
        Initialize gait generator

        Args:
            params: Gait parameters (uses defaults if None)
        """
        self.params = params if params is not None else GaitParams()

        # Internal state
        self.phase = 0.0  # Current gait phase [0, 2π]
        self.step_count = 0

    def reset(self):
        """Reset gait generator state"""
        self.phase = 0.0
        self.step_count = 0

    def update(self, dt: float):
        """
        Update gait phase

        Args:
            dt: Time step (s)
        """
        # Update phase
        phase_increment = 2.0 * np.pi * dt / self.params.step_period
        self.phase += phase_increment

        # Wrap phase to [0, 2π]
        if self.phase >= 2.0 * np.pi:
            self.phase -= 2.0 * np.pi
            self.step_count += 1

    def get_foot_trajectories(self, time: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get target positions for both feet at a given time

        Args:
            time: Current time (s)

        Returns:
            (left_foot_position, right_foot_position) as 3D numpy arrays
        """
        # Compute phase based on time
        phase = 2.0 * np.pi * time / self.params.step_period

        # Generate foot positions
        left_pos = self._compute_foot_position(phase, leg="left")
        right_pos = self._compute_foot_position(phase, leg="right")

        return left_pos, right_pos

    def _compute_foot_position(self, phase: float, leg: str) -> np.ndarray:
        """
        Compute foot position for a single leg

        Args:
            phase: Current gait phase [0, 2π]
            leg: "left" or "right"

        Returns:
            3D foot position [x, y, z]
        """
        # Left and right legs are out of phase by π
        if leg == "left":
            leg_phase = phase
        else:
            leg_phase = phase + np.pi

        # Normalize phase to [0, 2π]
        leg_phase = leg_phase % (2.0 * np.pi)

        # Determine if leg is in swing or stance phase
        # Swing phase: 0 to π, Stance phase: π to 2π
        is_swing = leg_phase < np.pi

        # Lateral position (y-direction)
        y_offset = self.params.stance_width / 2.0 if leg == "left" else -self.params.stance_width / 2.0

        # Compute forward position (x-direction)
        if is_swing:
            # Swing phase: foot moves forward
            normalized_phase = leg_phase / np.pi  # [0, 1]
            x = self.params.step_length * (normalized_phase - 0.5)
        else:
            # Stance phase: foot stays on ground, body moves forward
            normalized_phase = (leg_phase - np.pi) / np.pi  # [0, 1]
            x = self.params.step_length * (0.5 - normalized_phase)

        # Compute vertical position (z-direction)
        if is_swing:
            # Swing phase: lift foot in a sinusoidal arc
            normalized_phase = leg_phase / np.pi  # [0, 1]
            z = self.params.step_height * np.sin(np.pi * normalized_phase)
        else:
            # Stance phase: foot on ground
            z = 0.0

        return np.array([x, y_offset, z])

    def get_foot_velocities(self, time: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get target velocities for both feet at a given time

        Args:
            time: Current time (s)

        Returns:
            (left_foot_velocity, right_foot_velocity) as 3D numpy arrays
        """
        phase = 2.0 * np.pi * time / self.params.step_period

        left_vel = self._compute_foot_velocity(phase, leg="left")
        right_vel = self._compute_foot_velocity(phase, leg="right")

        return left_vel, right_vel

    def _compute_foot_velocity(self, phase: float, leg: str) -> np.ndarray:
        """
        Compute foot velocity for a single leg

        Args:
            phase: Current gait phase [0, 2π]
            leg: "left" or "right"

        Returns:
            3D foot velocity [vx, vy, vz]
        """
        # Compute velocity by numerical differentiation
        dt = 0.001
        pos1 = self._compute_foot_position(phase, leg)
        pos2 = self._compute_foot_position(phase + 2.0 * np.pi * dt / self.params.step_period, leg)

        velocity = (pos2 - pos1) / dt

        return velocity

    def get_com_trajectory(self, time: float) -> np.ndarray:
        """
        Get center of mass (CoM) trajectory

        Simple lateral shift for stability

        Args:
            time: Current time (s)

        Returns:
            3D CoM position [x, y, z]
        """
        phase = 2.0 * np.pi * time / self.params.step_period

        # CoM shifts laterally to maintain stability
        # Shift toward stance leg
        lateral_shift = 0.03  # 3cm shift

        # CoM oscillates between left and right
        y_com = lateral_shift * np.sin(phase)

        # Forward movement
        x_com = self.params.step_length * time / self.params.step_period

        # Height
        z_com = self.params.body_height

        return np.array([x_com, y_com, z_com])


class StaticGaitGenerator:
    """
    Static gait generator for initial testing

    Generates standing poses and simple weight shifts
    """

    def __init__(self, stance_width: float = 0.18, body_height: float = 0.45):
        """
        Initialize static gait generator

        Args:
            stance_width: Distance between feet
            body_height: Body height above ground
        """
        self.stance_width = stance_width
        self.body_height = body_height

    def get_standing_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get foot positions for standing pose

        Returns:
            (left_foot_position, right_foot_position)
        """
        left_pos = np.array([0.0, self.stance_width / 2.0, 0.0])
        right_pos = np.array([0.0, -self.stance_width / 2.0, 0.0])

        return left_pos, right_pos

    def get_weight_shift(self, time: float, period: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get foot positions with lateral weight shift

        Args:
            time: Current time
            period: Period of weight shift oscillation

        Returns:
            (left_foot_position, right_foot_position)
        """
        # Lateral shift
        shift_amplitude = 0.02  # 2cm
        y_shift = shift_amplitude * np.sin(2.0 * np.pi * time / period)

        left_pos = np.array([0.0, self.stance_width / 2.0 + y_shift, 0.0])
        right_pos = np.array([0.0, -self.stance_width / 2.0 + y_shift, 0.0])

        return left_pos, right_pos


if __name__ == "__main__":
    # Test gait generator
    print("Testing GaitGenerator:")

    params = GaitParams(
        step_length=0.1,
        step_height=0.05,
        step_period=1.0,
        stance_width=0.18,
        body_height=0.45
    )

    gait = GaitGenerator(params)

    print(f"\nGait parameters:")
    print(f"  Step length: {params.step_length} m")
    print(f"  Step height: {params.step_height} m")
    print(f"  Step period: {params.step_period} s")

    print(f"\nFoot trajectories at different times:")
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        left_pos, right_pos = gait.get_foot_trajectories(t)
        print(f"  t={t:.2f}s: Left={left_pos}, Right={right_pos}")

    print("\nTesting StaticGaitGenerator:")
    static_gait = StaticGaitGenerator()

    left_pos, right_pos = static_gait.get_standing_pose()
    print(f"  Standing pose: Left={left_pos}, Right={right_pos}")

    left_pos, right_pos = static_gait.get_weight_shift(0.5, period=2.0)
    print(f"  Weight shift at t=0.5s: Left={left_pos}, Right={right_pos}")
