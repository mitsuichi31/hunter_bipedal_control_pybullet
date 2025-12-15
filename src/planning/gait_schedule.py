"""
Gait schedule utilities aligned with ROS-style mode sequences.
"""

from dataclasses import dataclass
from typing import List, Dict, Any
import itertools


@dataclass
class GaitDefinition:
    sequence: List[str]
    switching_times: List[float]


class GaitSchedule:
    """
    Cycles through gait phases defined by mode sequences and switching times.
    """

    def __init__(self, gaits: Dict[str, GaitDefinition], default: str = "stance"):
        self.gaits = gaits
        self.current_name = default
        self.phase = 0.0
        self.index = 0

    def set_gait(self, name: str):
        if name not in self.gaits:
            raise ValueError(f"Gait {name} not found")
        self.current_name = name
        self.phase = 0.0
        self.index = 0

    def step(self, dt: float) -> str:
        gait = self.gaits[self.current_name]
        # Iterate through switching intervals
        if self.index + 1 < len(gait.switching_times):
            end_t = gait.switching_times[self.index + 1]
            if self.phase + dt >= end_t:
                self.index = (self.index + 1) % (len(gait.sequence))
                if self.index == 0:
                    self.phase = 0.0
                else:
                    self.phase += dt
            else:
                self.phase += dt
        else:
            self.phase = 0.0
            self.index = 0
        return gait.sequence[self.index]

    @staticmethod
    def from_config(config: Dict[str, Any]) -> "GaitSchedule":
        gaits = {
            name: GaitDefinition(sequence=val["sequence"],
                                 switching_times=val["switching_times"])
            for name, val in config.get("gaits", {}).items()
        }
        return GaitSchedule(gaits=gaits, default=config.get("default", "stance"))
