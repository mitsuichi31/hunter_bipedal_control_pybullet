"""
Normalize external joint command dicts into HunterSimulation.apply_hybrid_command format.
"""

from typing import Any, Dict, Union


HybridValue = Union[float, int, Dict[str, Any]]
JointCommands = Dict[str, HybridValue]


def normalize_joint_commands(cmds: JointCommands) -> JointCommands:
    """
    Normalize several acceptable command representations into the supported hybrid format.

    Matches HunterSimulation.apply_hybrid_command() expectations:
      - float/int: position target
      - {'mode': 'position', 'value': target, ...}
      - {'mode': 'torque', 'value': torque}
      - {'mode': 'hybrid', 'position': ..., 'velocity': ..., 'kp': ..., 'kd': ..., 'torque': ...}
    """
    out: JointCommands = {}

    for joint_name, cmd in cmds.items():
        if isinstance(cmd, (float, int)):
            out[joint_name] = float(cmd)
            continue

        if not isinstance(cmd, dict):
            raise TypeError(f"Unsupported command type for {joint_name}: {type(cmd)}")

        mode = cmd.get("mode", "position")

        if mode == "torque":
            if "value" in cmd:
                out[joint_name] = {"mode": "torque", "value": float(cmd["value"])}
            elif "torque" in cmd:
                out[joint_name] = {"mode": "torque", "value": float(cmd["torque"])}
            else:
                raise KeyError(f"torque mode needs 'value' or 'torque' for {joint_name}")
            continue

        if mode == "hybrid":
            if "position" not in cmd and "value" not in cmd:
                raise KeyError(f"hybrid mode needs 'position' or 'value' for {joint_name}")
            out[joint_name] = {
                "mode": "hybrid",
                "position": float(cmd.get("position", cmd.get("value"))),
                "velocity": float(cmd.get("velocity", 0.0)),
                "kp": float(cmd.get("kp", 0.0)),
                "kd": float(cmd.get("kd", 0.0)),
                "torque": float(cmd.get("torque", 0.0)),
            }
            continue

        # position (default)
        if "value" in cmd:
            target = cmd["value"]
        elif "position" in cmd:
            target = cmd["position"]
        else:
            raise KeyError(f"position mode needs 'value' or 'position' for {joint_name}")

        out[joint_name] = {"mode": "position", "value": float(target)}
        if "kp" in cmd:
            out[joint_name]["kp"] = float(cmd["kp"])
        if "kd" in cmd:
            out[joint_name]["kd"] = float(cmd["kd"])
        if "velocity" in cmd:
            out[joint_name]["velocity"] = float(cmd["velocity"])

    return out

