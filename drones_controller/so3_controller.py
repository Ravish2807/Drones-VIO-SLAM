from dataclasses import dataclass
import numpy as np
from .state_adapter import vee, hat


@dataclass
class SO3ControllerOutput:
    e_R: np.ndarray        # Attitude error vector [eR_x, eR_y, eR_z]
    e_omega: np.ndarray    # Angular velocity error vector [eW_x, eW_y, eW_z]
    M_d: np.ndarray        # Control moment vector [Mx, My, Mz] in body frame


class SO3Controller:

    def __init__(
        self,
        inertia,
        kR,
        kOmega
    ):
        self.J = np.diag(np.array(inertia, dtype=float))
        self.KR = np.diag(np.array(kR, dtype=float))
        self.KOmega = np.diag(np.array(kOmega, dtype=float))

    def compute_attitude_error(self, R: np.ndarray, R_d: np.ndarray) -> np.ndarray:
        E = R_d.T @ R - R.T @ R_d
        return 0.5 * vee(E)

    def compute_angular_velocity_error(
        self,
        R: np.ndarray,
        R_d: np.ndarray,
        omega: np.ndarray,
        omega_d: np.ndarray
    ) -> np.ndarray:
        return omega - R.T @ R_d @ omega_d

    def compute_moment(
        self,
        R: np.ndarray,
        R_d: np.ndarray,
        omega: np.ndarray,
        omega_d: np.ndarray,
        omega_dot_d: np.ndarray,
        e_R: np.ndarray,
        e_omega: np.ndarray
    ) -> np.ndarray:

        relative_R = R.T @ R_d
        coriolis = np.cross(omega, self.J @ omega)
        omega_hat = hat(omega)

        feedforward = self.J @ (
            omega_hat @ relative_R @ omega_d - relative_R @ omega_dot_d
        )

        M_d = (
            -self.KR @ e_R
            - self.KOmega @ e_omega
            + coriolis
            - feedforward
        )

        return M_d

    def compute(
        self,
        R: np.ndarray,
        omega: np.ndarray,
        R_d: np.ndarray,
        omega_d: np.ndarray = None,
        omega_dot_d: np.ndarray = None
    ) -> SO3ControllerOutput:

        if omega_d is None:
            omega_d = np.zeros(3, dtype=float)
        if omega_dot_d is None:
            omega_dot_d = np.zeros(3, dtype=float)

        e_R = self.compute_attitude_error(R, R_d)
        e_omega = self.compute_angular_velocity_error(R, R_d, omega, omega_d)
        M_d = self.compute_moment(R, R_d, omega, omega_d, omega_dot_d, e_R, e_omega)

        return SO3ControllerOutput(
            e_R=e_R,
            e_omega=e_omega,
            M_d=M_d
        )
