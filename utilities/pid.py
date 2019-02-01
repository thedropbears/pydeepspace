from dataclasses import dataclass


@dataclass
class PID:
    P: float
    I: float
    D: float
    F: float = 0.0
