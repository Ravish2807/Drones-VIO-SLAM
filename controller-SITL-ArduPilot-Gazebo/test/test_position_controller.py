import unittest
import numpy as np
from drones_controller.position_controller import PositionController


class TestPositionController(unittest.TestCase):

    def test_hover_target_reached(self):
        controller = PositionController(
            mass=1.5,
            gravity=9.81,
            kp=[0.5, 0.5, 0.5],
            kv=[0.2, 0.2, 0.2],
            max_velocity=0.5,
            position_tolerance=0.10,
            velocity_tolerance=0.05
        )
        p = np.array([1.0, 0.0, 1.0])
        v = np.array([0.0, 0.0, 0.0])
        p_d = np.array([1.0, 0.0, 1.0])

        out = controller.compute(p, v, p_d)

        # In steady state target reached:
        self.assertTrue(out.target_reached)
        np.testing.assert_allclose(out.v_cmd, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(out.e_p, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(out.e_v, [0.0, 0.0, 0.0])

    def test_velocity_command_saturation(self):
        controller = PositionController(
            mass=1.5,
            gravity=9.81,
            kp=[1.0, 1.0, 1.0],
            kv=[0.2, 0.2, 0.2],
            max_velocity=0.5,
            position_tolerance=0.10,
            velocity_tolerance=0.05
        )
        p = np.array([0.0, 0.0, 0.0])
        v = np.array([0.0, 0.0, 0.0])
        p_d = np.array([5.0, 0.0, 0.0])  # Large error -> raw v_cmd = [5.0, 0, 0]

        out = controller.compute(p, v, p_d)

        self.assertFalse(out.target_reached)
        # Saturated velocity command magnitude should be max_velocity = 0.5
        speed = np.linalg.norm(out.v_cmd)
        self.assertAlmostEqual(speed, 0.5)
        np.testing.assert_allclose(out.v_cmd, [0.5, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
