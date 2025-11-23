#!/usr/bin/env python3
"""
Stability Metrics Module for Hunter Bipedal Robot

Provides accurate computation of:
- Center of Mass (CoM) position, velocity, acceleration
- Zero Moment Point (ZMP) with dynamics
- Stability margin
- Support polygon analysis

This module replaces simplified approximations in balance_controller.py
with accurate multi-body dynamics computations.

Author: Stability Improvement Phase 1
Date: November 2025
"""

import numpy as np
import pybullet as p
from typing import Tuple, List, Optional


class StabilityMetrics:
    """
    Compute stability metrics for bipedal robot

    Key improvements over previous implementation:
    1. Accurate CoM using all link masses (not just base)
    2. True ZMP computation with acceleration terms
    3. Stability margin relative to support polygon
    4. CoM velocity and acceleration computation
    """

    def __init__(self, robot_id: int):
        """
        Initialize stability metrics calculator

        Args:
            robot_id: PyBullet body ID of the robot
        """
        self.robot_id = robot_id
        self.gravity = 9.81  # m/s^2

        # Cache for link information (computed once)
        self._link_info_cached = False
        self._num_joints = 0
        self._link_indices = []  # Indices of all links

        # Previous state for numerical differentiation
        self._prev_com_pos = None
        self._prev_com_vel = None
        self._prev_time = None

    def _cache_link_info(self):
        """Cache link information for efficiency"""
        if self._link_info_cached:
            return

        self._num_joints = p.getNumJoints(self.robot_id)
        # Include base link (-1) and all joint links
        self._link_indices = [-1] + list(range(self._num_joints))
        self._link_info_cached = True

    def _get_link_mass_and_position(self, link_index: int) -> Tuple[float, np.ndarray]:
        """
        Get mass and center of mass position for a single link

        Args:
            link_index: -1 for base link, >= 0 for joint links

        Returns:
            (mass, position) tuple
        """
        if link_index == -1:
            # Base link
            dynamics_info = p.getDynamicsInfo(self.robot_id, -1)
            mass = dynamics_info[0]

            # Get base position and orientation
            base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)

            # Local CoM offset
            local_inertial_pos = dynamics_info[3]

            # Transform local offset to world frame
            # world_com = base_pos + R(base_orn) * local_inertial_pos
            rotation_matrix = np.array(p.getMatrixFromQuaternion(base_orn)).reshape(3, 3)
            world_offset = rotation_matrix @ np.array(local_inertial_pos)
            com_pos = np.array(base_pos) + world_offset

        else:
            # Joint link
            dynamics_info = p.getDynamicsInfo(self.robot_id, link_index)
            mass = dynamics_info[0]

            # Link state gives position/orientation
            link_state = p.getLinkState(self.robot_id, link_index)
            link_world_pos = np.array(link_state[0])
            link_world_orn = link_state[1]

            # Local inertial frame offset
            local_inertial_pos = dynamics_info[3]

            # Transform to world frame
            rotation_matrix = np.array(p.getMatrixFromQuaternion(link_world_orn)).reshape(3, 3)
            world_offset = rotation_matrix @ np.array(local_inertial_pos)
            com_pos = link_world_pos + world_offset

        return mass, com_pos

    def compute_com_position(self) -> np.ndarray:
        """
        Compute accurate center of mass position

        Uses weighted average of all link CoM positions:
        CoM = Σ(m_i * p_i) / Σ(m_i)

        Returns:
            CoM position [x, y, z] in world frame (meters)
        """
        self._cache_link_info()

        total_mass = 0.0
        weighted_com = np.zeros(3)

        for link_idx in self._link_indices:
            mass, pos = self._get_link_mass_and_position(link_idx)

            if mass > 1e-6:  # Ignore massless links
                total_mass += mass
                weighted_com += mass * pos

        if total_mass < 1e-6:
            raise ValueError("Robot has zero total mass!")

        com_position = weighted_com / total_mass
        return com_position

    def compute_com_velocity(self, dt: Optional[float] = None) -> np.ndarray:
        """
        Compute center of mass velocity

        Can use either:
        1. Numerical differentiation of CoM position (if dt provided)
        2. Weighted average of link velocities (more accurate)

        Args:
            dt: Time step for numerical differentiation (optional)

        Returns:
            CoM velocity [vx, vy, vz] in world frame (m/s)
        """
        self._cache_link_info()

        total_mass = 0.0
        weighted_com_vel = np.zeros(3)

        for link_idx in self._link_indices:
            mass, _ = self._get_link_mass_and_position(link_idx)

            if mass < 1e-6:
                continue

            # Get link velocity
            if link_idx == -1:
                # Base link velocity
                base_vel, _ = p.getBaseVelocity(self.robot_id)
                link_vel = np.array(base_vel)
            else:
                # Joint link velocity
                link_state = p.getLinkState(self.robot_id, link_idx, computeLinkVelocity=1)
                link_vel = np.array(link_state[6])  # World linear velocity

            total_mass += mass
            weighted_com_vel += mass * link_vel

        com_velocity = weighted_com_vel / total_mass
        return com_velocity

    def compute_com_acceleration(self, dt: float) -> np.ndarray:
        """
        Compute center of mass acceleration via numerical differentiation

        Args:
            dt: Time step (seconds)

        Returns:
            CoM acceleration [ax, ay, az] in world frame (m/s^2)
        """
        current_vel = self.compute_com_velocity()

        # First call - cannot compute acceleration yet
        if self._prev_com_vel is None:
            self._prev_com_vel = current_vel
            return np.zeros(3)

        # Numerical differentiation: a = (v_current - v_prev) / dt
        com_acceleration = (current_vel - self._prev_com_vel) / dt

        # Update previous state
        self._prev_com_vel = current_vel

        return com_acceleration

    def compute_total_mass(self) -> float:
        """
        Compute total robot mass

        Returns:
            Total mass in kg
        """
        self._cache_link_info()

        total_mass = 0.0
        for link_idx in self._link_indices:
            dynamics_info = p.getDynamicsInfo(self.robot_id, link_idx)
            mass = dynamics_info[0]
            total_mass += mass

        return total_mass

    def compute_zmp(self, com_pos: Optional[np.ndarray] = None,
                    com_acc: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Compute Zero Moment Point (ZMP) using dynamics

        ZMP is where the resultant ground reaction force acts.
        Formula: zmp_x = x - (h/g) * ddot_x
                zmp_y = y - (h/g) * ddot_y

        where:
        - (x, y, h) is CoM position
        - (ddot_x, ddot_y) is CoM acceleration
        - g is gravity

        Args:
            com_pos: CoM position (computed if not provided)
            com_acc: CoM acceleration (computed if not provided)

        Returns:
            ZMP position [x, y] in ground plane (meters)
        """
        if com_pos is None:
            com_pos = self.compute_com_position()

        if com_acc is None:
            # Cannot compute ZMP without acceleration
            # Return CoM projection as approximation
            return com_pos[0:2]

        # Extract height and horizontal accelerations
        h = com_pos[2]  # Height above ground
        ddot_x = com_acc[0]
        ddot_y = com_acc[1]

        # ZMP formula
        zmp_x = com_pos[0] - (h / self.gravity) * ddot_x
        zmp_y = com_pos[1] - (h / self.gravity) * ddot_y

        return np.array([zmp_x, zmp_y])

    def compute_support_polygon(self, foot_positions: List[np.ndarray]) -> np.ndarray:
        """
        Compute support polygon from foot contact points

        Args:
            foot_positions: List of [x, y] positions of contact points

        Returns:
            Support polygon vertices as Nx2 array
        """
        if len(foot_positions) < 3:
            # Not enough points for polygon - return bounding box
            positions = np.array(foot_positions)
            min_x, min_y = positions.min(axis=0)
            max_x, max_y = positions.max(axis=0)

            # Bounding box vertices
            polygon = np.array([
                [min_x, min_y],
                [max_x, min_y],
                [max_x, max_y],
                [min_x, max_y]
            ])
            return polygon

        # Compute convex hull
        from scipy.spatial import ConvexHull
        positions = np.array(foot_positions)
        hull = ConvexHull(positions)

        # Return vertices in order
        polygon = positions[hull.vertices]
        return polygon

    def compute_stability_margin(self, zmp: np.ndarray,
                                 support_polygon: np.ndarray) -> float:
        """
        Compute stability margin as distance from ZMP to polygon edge

        Positive margin = stable (ZMP inside polygon)
        Negative margin = unstable (ZMP outside polygon)

        Args:
            zmp: ZMP position [x, y]
            support_polygon: Polygon vertices Nx2

        Returns:
            Stability margin in meters (positive = stable)
        """
        # Simple implementation: distance to nearest edge
        # For more sophisticated: use signed distance to polygon

        min_distance = float('inf')
        n_vertices = len(support_polygon)

        for i in range(n_vertices):
            # Edge from vertex i to vertex (i+1) % n
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % n_vertices]

            # Distance from point to line segment
            distance = self._point_to_segment_distance(zmp, p1, p2)
            min_distance = min(min_distance, distance)

        # Check if ZMP is inside polygon (positive) or outside (negative)
        inside = self._is_point_inside_polygon(zmp, support_polygon)

        return min_distance if inside else -min_distance

    def _point_to_segment_distance(self, point: np.ndarray,
                                   seg_start: np.ndarray,
                                   seg_end: np.ndarray) -> float:
        """Compute distance from point to line segment"""
        # Vector from seg_start to seg_end
        seg_vec = seg_end - seg_start
        seg_length_sq = np.dot(seg_vec, seg_vec)

        if seg_length_sq < 1e-8:
            # Degenerate segment (point)
            return np.linalg.norm(point - seg_start)

        # Project point onto line
        t = np.dot(point - seg_start, seg_vec) / seg_length_sq
        t = np.clip(t, 0, 1)  # Clamp to segment

        # Closest point on segment
        closest = seg_start + t * seg_vec

        return np.linalg.norm(point - closest)

    def _is_point_inside_polygon(self, point: np.ndarray,
                                 polygon: np.ndarray) -> bool:
        """Check if point is inside polygon using ray casting"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]

            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside

            p1x, p1y = p2x, p2y

        return inside

    def get_metrics(self, dt: float,
                   foot_positions: Optional[List[np.ndarray]] = None) -> dict:
        """
        Compute all stability metrics at once

        Args:
            dt: Time step for acceleration computation
            foot_positions: Contact point positions (optional)

        Returns:
            Dictionary of metrics:
            {
                'com_position': [x, y, z],
                'com_velocity': [vx, vy, vz],
                'com_acceleration': [ax, ay, az],
                'zmp': [x, y],
                'total_mass': m,
                'stability_margin': margin (if foot_positions provided)
            }
        """
        com_pos = self.compute_com_position()
        com_vel = self.compute_com_velocity()
        com_acc = self.compute_com_acceleration(dt)
        zmp = self.compute_zmp(com_pos, com_acc)
        total_mass = self.compute_total_mass()

        metrics = {
            'com_position': com_pos,
            'com_velocity': com_vel,
            'com_acceleration': com_acc,
            'zmp': zmp,
            'total_mass': total_mass
        }

        # Compute stability margin if foot positions provided
        if foot_positions is not None and len(foot_positions) > 0:
            try:
                polygon = self.compute_support_polygon(foot_positions)
                margin = self.compute_stability_margin(zmp, polygon)
                metrics['stability_margin'] = margin
                metrics['support_polygon'] = polygon
            except:
                # If scipy not available, skip polygon metrics
                pass

        return metrics


# Convenience functions for standalone use
def compute_com(robot_id: int) -> np.ndarray:
    """Compute CoM position for a robot"""
    metrics = StabilityMetrics(robot_id)
    return metrics.compute_com_position()


def compute_com_velocity(robot_id: int, dt: float = 0.001) -> np.ndarray:
    """Compute CoM velocity for a robot"""
    metrics = StabilityMetrics(robot_id)
    return metrics.compute_com_velocity(dt)


def compute_zmp(robot_id: int, dt: float = 0.001) -> np.ndarray:
    """Compute ZMP for a robot"""
    metrics = StabilityMetrics(robot_id)
    com_pos = metrics.compute_com_position()
    com_acc = metrics.compute_com_acceleration(dt)
    return metrics.compute_zmp(com_pos, com_acc)


# Example usage
if __name__ == "__main__":
    print("Stability Metrics Module")
    print("=" * 60)
    print("\nThis module provides accurate stability computations:")
    print("  1. Center of Mass (all links, not just base)")
    print("  2. ZMP with dynamics (not just CoM projection)")
    print("  3. Stability margin computation")
    print("\nImport in your controller:")
    print("  from stability_metrics import StabilityMetrics")
    print("  metrics = StabilityMetrics(robot_id)")
    print("  com = metrics.compute_com_position()")
