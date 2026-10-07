from dataclasses import dataclass, field
import numpy as np


@dataclass
class TargetState:
    """Desired target kinematic state in ENU map frame."""
    position: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 1.0], dtype=float)) # p_d
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))               # v_d
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))           # a_d
    yaw: float = 0.0                                                                              # psi_d
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))       # omega_d
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))   # omega_dot_d


class TargetManager:

    def __init__(self, initial_position=None, initial_yaw=0.0):
        pos = initial_position if initial_position is not None else [1.0, 0.0, 1.0]
        self._target = TargetState(
            position=np.array(pos, dtype=float),
            velocity=np.zeros(3, dtype=float),
            acceleration=np.zeros(3, dtype=float),
            yaw=float(initial_yaw),
            angular_velocity=np.zeros(3, dtype=float),
            angular_acceleration=np.zeros(3, dtype=float)
        )

    def set_target_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        self._target.position = np.array([x, y, z], dtype=float)
        self._target.yaw = float(yaw)

    def get_target(self) -> TargetState:
        return self._target

    def update_target(self, target_state: TargetState):
        self._target = target_state
