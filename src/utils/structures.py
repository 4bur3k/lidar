from dataclasses import dataclass

@dataclass
class Waypoint:
    x: float
    y: float
    was_reached: bool
    