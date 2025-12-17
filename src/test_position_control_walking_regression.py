"""
Regression test for the position-control walking controller.

This headless test checks that the current walking implementation makes forward
progress while maintaining heading and lateral alignment. The thresholds mirror
the straightness observed in recent tuning (few mm of lateral drift over ~12s).
"""

import math
import os
from typing import Dict, Tuple

import numpy as np
import pybullet as p
import pybullet_data

from robot_constants import BASE_HEIGHT, FOOT_JOINTS, standing_config_copy
from position_control_walking import PositionControlWalkingController, WalkingControllerParams


def _setup_robot() -> Tuple[int, Dict[str, int]]:
    """Boot PyBullet in DIRECT mode and load Hunter at the canonical height."""
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)
    p.setRealTimeSimulation(0)

    plane_id = p.loadURDF("plane.urdf")

    urdf_path = os.path.join(os.path.dirname(__file__), "../models/urdf/hunter.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT], useFixedBase=False)

    joint_dict: Dict[str, int] = {}
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if "leg" in name and "joint" in name:
            joint_dict[name] = i

    # Set initial straight-leg stance.
    standing_config = standing_config_copy()
    for joint_name, angle in standing_config.items():
        p.resetJointState(robot_id, joint_dict[joint_name], angle)

    # Reduce foot slip during the regression to stabilize forward steps.
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    for foot_joint in FOOT_JOINTS:
        p.changeDynamics(robot_id, joint_dict[foot_joint], lateralFriction=1.0)

    return robot_id, joint_dict


def _apply_position_control(robot_id: int, joint_dict: Dict[str, int], commands: Dict[str, float]) -> None:
    """Apply joint targets using PyBullet's POSITION_CONTROL mode."""
    for joint_name, target_angle in commands.items():
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_dict[joint_name],
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=300.0,
        )


def test_position_control_walking_regression() -> None:
    """Walk forward for ~12s and enforce drift/yaw/height bounds."""
    robot_id, joint_dict = _setup_robot()

    params = WalkingControllerParams(standing_mode=False, enable_walking=True)
    controller = PositionControlWalkingController(robot_id, joint_dict, params)
    controller.reset()

    sim_dt = 0.001
    control_dt = 0.02
    duration = 12.0
    control_decimation = int(control_dt / sim_dt)

    positions_x = []
    positions_y = []
    heights = []
    rolls = []
    pitches = []
    yaws = []

    position_commands = None
    for step in range(int(duration / sim_dt)):
        t = step * sim_dt

        if step % control_decimation == 0:
            position_commands = controller.update(control_dt)

        if position_commands is None:
            p.disconnect()
            raise AssertionError(f"Controller stopped (emergency) at t={t:.2f}s")

        _apply_position_control(robot_id, joint_dict, position_commands)
        p.stepSimulation()

        if step % 10 == 0:
            base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
            roll, pitch, yaw = p.getEulerFromQuaternion(base_orn)
            positions_x.append(base_pos[0])
            positions_y.append(base_pos[1])
            heights.append(base_pos[2])
            rolls.append(math.degrees(roll))
            pitches.append(math.degrees(pitch))
            yaws.append(yaw)  # keep radians for unwrapping

    p.disconnect()

    yaw_unwrapped = np.unwrap(np.array(yaws))

    forward_progress = positions_x[-1] - positions_x[0]
    lateral_drift = positions_y[-1] - positions_y[0]
    yaw_drift_deg = math.degrees(yaw_unwrapped[-1] - yaw_unwrapped[0])

    max_tilt = max(max(abs(r) for r in rolls), max(abs(p) for p in pitches))
    min_height = min(heights)

    assert forward_progress > 0.08, f"Expected forward progress > 0.08m, got {forward_progress:.3f}m"
    assert abs(lateral_drift) < 0.02, f"Lateral drift too large: {lateral_drift:.3f}m"
    assert abs(yaw_drift_deg) < 3.0, f"Yaw drift too large: {yaw_drift_deg:.2f}°"
    assert max_tilt < 10.0, f"Body tilt exceeded limit: {max_tilt:.2f}°"
    assert min_height > 0.55, f"Robot height dropped: min z={min_height:.3f}m"
