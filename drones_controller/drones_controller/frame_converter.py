import numpy as np

# Transformation matrix from ENU (East-North-Up) to NED (North-East-Down)
# ENU: X=East, Y=North, Z=Up
# NED: X=North, Y=East, Z=Down
R_ENU_TO_NED = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0]
], dtype=float)

R_NED_TO_ENU = R_ENU_TO_NED.T


def enu_to_ned_vector(vec_enu: np.ndarray) -> np.ndarray:
    """Convert a 3D vector (position, velocity, acceleration, force) from ENU to NED."""
    vec = np.asarray(vec_enu, dtype=float)
    return R_ENU_TO_NED @ vec


def ned_to_enu_vector(vec_ned: np.ndarray) -> np.ndarray:
    """Convert a 3D vector (position, velocity, acceleration, force) from NED to ENU."""
    vec = np.asarray(vec_ned, dtype=float)
    return R_NED_TO_ENU @ vec


def enu_to_ned_rotation(R_enu: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix from ENU to NED frame."""
    return R_ENU_TO_NED @ R_enu @ R_ENU_TO_NED.T


def ned_to_enu_rotation(R_ned: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix from NED to ENU frame."""
    return R_NED_TO_ENU @ R_ned @ R_NED_TO_ENU.T


def enu_to_ned_quaternion(q_enu: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [x, y, z, w] from ENU to NED.
    q_ned = q_rot * q_enu
    """
    qx, qy, qz, qw = q_enu
    # R_ENU_TO_NED corresponds to 180 pitch-roll swap equivalent
    # Direct component mapping for ENU to NED quaternion:
    # x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu, w_ned = w_enu (normalized)
    q_ned = np.array([qy, qx, -qz, qw], dtype=float)
    norm = np.linalg.norm(q_ned)
    if norm > 1e-9:
        q_ned /= norm
    return q_ned


def ned_to_enu_quaternion(q_ned: np.ndarray) -> np.ndarray:
    """Convert quaternion [x, y, z, w] from NED to ENU."""
    qx, qy, qz, qw = q_ned
    q_enu = np.array([qy, qx, -qz, qw], dtype=float)
    norm = np.linalg.norm(q_enu)
    if norm > 1e-9:
        q_enu /= norm
    return q_enu
