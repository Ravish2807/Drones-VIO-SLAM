import numpy as np


class AttitudeGenerator:
    """Construct body-to-world R_des with body +z aligned to desired force."""
    def compute(self, force_world, yaw=0.0):
        force = np.asarray(force_world, float).reshape(3)
        b3 = force / max(np.linalg.norm(force), 1e-9)
        b1_heading = np.array([np.cos(yaw), np.sin(yaw), 0.])
        b2 = np.cross(b3, b1_heading)
        if np.linalg.norm(b2) < 1e-8:
            b2 = np.array([-np.sin(yaw), np.cos(yaw), 0.])
        b2 /= np.linalg.norm(b2)
        b1 = np.cross(b2, b3)
        return np.column_stack((b1, b2, b3))
