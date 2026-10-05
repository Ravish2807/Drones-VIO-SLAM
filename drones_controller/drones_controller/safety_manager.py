import numpy as np
from .command_interface import ControlCommand
from .state_adapter import DroneState, validate_rotation_matrix


class SafetyManager:

    def __init__(
        self,
        max_velocity: float = 0.5,
        state_timeout: float = 0.2
    ):
        self.max_velocity = float(max_velocity)
        self.state_timeout = float(state_timeout)

    def validate_state(self, state: DroneState) -> bool:
        if state is None:
            return False

        if not np.all(np.isfinite(state.position)):
            return False
        if not np.all(np.isfinite(state.velocity)):
            return False
        if not np.all(np.isfinite(state.angular_velocity)):
            return False
        if not validate_rotation_matrix(state.rotation):
            return False

        return True

    def validate_command(self, command: ControlCommand) -> ControlCommand:
        if command is None or command.v_cmd is None:
            return self._create_zero_command()

        # Check for NaN / Inf in velocity command
        if not np.all(np.isfinite(command.v_cmd)):
            return self._create_zero_command()

        # Enforce max velocity limit
        speed = np.linalg.norm(command.v_cmd)
        if speed > self.max_velocity and speed > 1e-6:
            command.v_cmd = command.v_cmd * (self.max_velocity / speed)

        return command

    def _create_zero_command(self) -> ControlCommand:
        return ControlCommand(
            v_cmd=np.zeros(3, dtype=float),
            yaw_rate=0.0,
            e_p=np.zeros(3, dtype=float),
            e_v=np.zeros(3, dtype=float),
            F_d=np.zeros(3, dtype=float),
            R_d=np.eye(3, dtype=float),
            M_d=np.zeros(3, dtype=float),
            thrust_d=0.0
        )
