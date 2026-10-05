import unittest
import numpy as np
from drones_controller.attitude_generator import AttitudeGenerator
from drones_controller.state_adapter import validate_rotation_matrix


class TestAttitudeGenerator(unittest.TestCase):

    def test_pure_upward_force(self):
        generator = AttitudeGenerator()
        F_d = np.array([0.0, 0.0, 14.715])  # pure vertical hover force
        yaw_d = 0.0

        target = generator.compute(F_d, yaw_d)

        self.assertAlmostEqual(target.thrust_d, 14.715)
        np.testing.assert_allclose(target.b3_d, [0.0, 0.0, 1.0])
        self.assertTrue(validate_rotation_matrix(target.R_d))
        np.testing.assert_allclose(target.R_d, np.eye(3), atol=1e-5)

    def test_tilted_force(self):
        generator = AttitudeGenerator()
        F_d = np.array([5.0, 0.0, 14.715])
        yaw_d = 0.0

        target = generator.compute(F_d, yaw_d)

        self.assertTrue(validate_rotation_matrix(target.R_d))
        self.assertGreater(target.thrust_d, 14.715)


if __name__ == '__main__':
    unittest.main()
