import unittest
import numpy as np
from drones_controller.frame_converter import (
    enu_to_ned_vector,
    ned_to_enu_vector,
    enu_to_ned_rotation,
    ned_to_enu_rotation,
    enu_to_ned_quaternion,
    ned_to_enu_quaternion
)


class TestFrameConverter(unittest.TestCase):

    def test_vector_conversions(self):
        # ENU: X=1 (East), Y=2 (North), Z=3 (Up)
        # NED: X=2 (North), Y=1 (East), Z=-3 (Down)
        v_enu = np.array([1.0, 2.0, 3.0])
        v_ned = enu_to_ned_vector(v_enu)
        np.testing.assert_allclose(v_ned, [2.0, 1.0, -3.0])

        v_back = ned_to_enu_vector(v_ned)
        np.testing.assert_allclose(v_back, v_enu)

    def test_rotation_conversions(self):
        R_identity = np.eye(3)
        R_ned = enu_to_ned_rotation(R_identity)
        R_enu_back = ned_to_enu_rotation(R_ned)
        np.testing.assert_allclose(R_enu_back, R_identity)

    def test_quaternion_conversions(self):
        q_enu = np.array([0.0, 0.0, 0.0, 1.0])
        q_ned = enu_to_ned_quaternion(q_enu)
        q_back = ned_to_enu_quaternion(q_ned)
        np.testing.assert_allclose(q_back, q_enu)


if __name__ == '__main__':
    unittest.main()
