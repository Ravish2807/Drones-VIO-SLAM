import numpy as np
from .target_manager import TargetState


class WaypointGenerator:

    def __init__(self, position, yaw=0.0):
        self.position = np.array(position, dtype=float)
        self.yaw = float(yaw)

    def set_waypoint(self, position, yaw=0.0):
        self.position = np.array(position, dtype=float)
        self.yaw = float(yaw)

    def get_desired_state(self) -> TargetState:
        return TargetState(
            position=self.position.copy(),
            velocity=np.zeros(3, dtype=float),
            acceleration=np.zeros(3, dtype=float),
            yaw=self.yaw,
            angular_velocity=np.zeros(3, dtype=float),
            angular_acceleration=np.zeros(3, dtype=float)
        )
