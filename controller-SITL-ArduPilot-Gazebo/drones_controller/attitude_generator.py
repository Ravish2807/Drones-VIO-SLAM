from dataclasses import dataclass
import numpy as np


@dataclass
class AttitudeTarget:
    R_d: np.ndarray        # Desired 3x3 rotation matrix in SO(3)
    thrust_d: float        # Desired scalar thrust magnitude T_d = ||F_d||
    b3_d: np.ndarray       # Desired body Z axis vector


class AttitudeGenerator:

    def compute(self, F_d: np.ndarray, yaw_d: float) -> AttitudeTarget:
        norm_F = np.linalg.norm(F_d)

        if norm_F < 1e-6:
            R_d = np.eye(3, dtype=float)
            return AttitudeTarget(R_d=R_d, thrust_d=0.0, b3_d=np.array([0.0, 0.0, 1.0]))

        b3 = F_d / norm_F

        yaw_heading = np.array([
            np.cos(yaw_d),
            np.sin(yaw_d),
            0.0
        ], dtype=float)

        b2 = np.cross(b3, yaw_heading)
        norm_b2 = np.linalg.norm(b2)

        if norm_b2 < 1e-6:
            yaw_d += 1e-3
            yaw_heading = np.array([
                np.cos(yaw_d),
                np.sin(yaw_d),
                0.0
            ], dtype=float)
            b2 = np.cross(b3, yaw_heading)
            norm_b2 = np.linalg.norm(b2)

        b2 /= norm_b2
        b1 = np.cross(b2, b3)
        b1 /= np.linalg.norm(b1)

        R_d = np.column_stack([b1, b2, b3])

        return AttitudeTarget(
            R_d=R_d,
            thrust_d=float(norm_F),
            b3_d=b3
        )
