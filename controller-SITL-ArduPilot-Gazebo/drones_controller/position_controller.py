from dataclasses import dataclass
import numpy as np


@dataclass
class PositionControllerOutput:
    e_p: np.ndarray             # Position error vector [ep_x, ep_y, ep_z]
    e_v: np.ndarray             # Velocity error vector [ev_x, ev_y, ev_z]
    v_cmd: np.ndarray           # Velocity command vector [vx, vy, vz]
    a_cmd: np.ndarray           # Desired acceleration vector for SO(3) debug
    F_d: np.ndarray             # Desired force vector for SO(3) debug
    target_reached: bool        # Flag indicating target position reached


class PositionController:

    def __init__(
        self,
        mass: float = 1.5,
        gravity: float = 9.81,
        kp=None,
        kv=None,
        max_velocity: float = 0.5,
        position_tolerance: float = 0.10,
        velocity_tolerance: float = 0.05
    ):
        self.mass = float(mass)
        self.gravity = float(gravity)

        kp_arr = np.array(kp if kp is not None else [0.5, 0.5, 0.5], dtype=float)
        kv_arr = np.array(kv if kv is not None else [0.2, 0.2, 0.2], dtype=float)

        self.Kp = np.diag(kp_arr)
        self.Kv = np.diag(kv_arr)

        self.max_velocity = float(max_velocity)
        self.position_tolerance = float(position_tolerance)
        self.velocity_tolerance = float(velocity_tolerance)
        self.e3 = np.array([0.0, 0.0, 1.0], dtype=float)

    def compute(
        self,
        p: np.ndarray,
        v: np.ndarray,
        p_d: np.ndarray,
        v_d: np.ndarray = None,
        a_d: np.ndarray = None
    ) -> PositionControllerOutput:

        if v_d is None:
            v_d = np.zeros(3, dtype=float)
        if a_d is None:
            a_d = np.zeros(3, dtype=float)

        e_p = p_d - p
        e_v = v_d - v

        pos_error_norm = np.linalg.norm(e_p)
        vel_norm = np.linalg.norm(v)

        target_reached = (
            pos_error_norm < self.position_tolerance
            and vel_norm < self.velocity_tolerance
        )

        if target_reached:
            v_cmd = np.zeros(3, dtype=float)
        else:
            v_cmd_raw = self.Kp @ e_p + self.Kv @ e_v
            speed = np.linalg.norm(v_cmd_raw)

            if speed > self.max_velocity and speed > 1e-6:
                v_cmd = v_cmd_raw * (self.max_velocity / speed)
            else:
                v_cmd = v_cmd_raw

        # Calculate desired acceleration & force for SO(3) debug pipeline
        a_cmd = self.Kp @ e_p + self.Kv @ e_v + a_d
        F_d = self.mass * (a_cmd + self.gravity * self.e3)

        return PositionControllerOutput(
            e_p=e_p,
            e_v=e_v,
            v_cmd=v_cmd,
            a_cmd=a_cmd,
            F_d=F_d,
            target_reached=target_reached
        )
