from dataclasses import dataclass, asdict
from typing import List, Dict, Any
import numpy as np


@dataclass
class TelemetryFrame:
    timestamp: float
    target_pos: List[float]
    actual_pos: List[float]
    pos_error: List[float]
    actual_vel: List[float]
    vel_error: List[float]
    desired_force: List[float]
    desired_moment: List[float]
    attitude_error: List[float]
    angular_vel_error: List[float]
    desired_thrust: float
    flight_state: str


class TelemetryLogger:

    def __init__(self, max_records: int = 10000):
        self.max_records = max_records
        self.history: List[TelemetryFrame] = []

    def log(
        self,
        timestamp: float,
        target_pos: np.ndarray,
        actual_pos: np.ndarray,
        pos_error: np.ndarray,
        actual_vel: np.ndarray,
        vel_error: np.ndarray,
        desired_force: np.ndarray,
        desired_moment: np.ndarray,
        attitude_error: np.ndarray,
        angular_vel_error: np.ndarray,
        desired_thrust: float,
        flight_state: str
    ):
        frame = TelemetryFrame(
            timestamp=float(timestamp),
            target_pos=target_pos.tolist() if isinstance(target_pos, np.ndarray) else list(target_pos),
            actual_pos=actual_pos.tolist() if isinstance(actual_pos, np.ndarray) else list(actual_pos),
            pos_error=pos_error.tolist() if isinstance(pos_error, np.ndarray) else list(pos_error),
            actual_vel=actual_vel.tolist() if isinstance(actual_vel, np.ndarray) else list(actual_vel),
            vel_error=vel_error.tolist() if isinstance(vel_error, np.ndarray) else list(vel_error),
            desired_force=desired_force.tolist() if isinstance(desired_force, np.ndarray) else list(desired_force),
            desired_moment=desired_moment.tolist() if isinstance(desired_moment, np.ndarray) else list(desired_moment),
            attitude_error=attitude_error.tolist() if isinstance(attitude_error, np.ndarray) else list(attitude_error),
            angular_vel_error=angular_vel_error.tolist() if isinstance(angular_vel_error, np.ndarray) else list(angular_vel_error),
            desired_thrust=float(desired_thrust),
            flight_state=str(flight_state)
        )

        self.history.append(frame)
        if len(self.history) > self.max_records:
            self.history.pop(0)

    def get_latest(self) -> TelemetryFrame:
        return self.history[-1] if self.history else None
