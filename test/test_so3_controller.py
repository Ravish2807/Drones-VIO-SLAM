import unittest
import numpy as np
from drones_controller.so3_controller import SO3Controller


class TestSO3Controller(unittest.TestCase):

    def test_zero_attitude_error(self):
        controller = SO3Controller(
            inertia=[0.02, 0.02, 0.04],
            kR=[4.0, 4.0, 2.0],
            kOmega=[0.8, 0.8, 0.5]
        )

        R = np.eye(3)
        R_d = np.eye(3)
        omega = np.zeros(3)

        out = controller.compute(R=R, omega=omega, R_d=R_d)

        np.testing.assert_allclose(out.e_R, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(out.e_omega, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(out.M_d, [0.0, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
