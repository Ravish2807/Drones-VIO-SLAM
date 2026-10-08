"""Thin adapter around gym-pybullet-drones CtrlAviary."""
import numpy as np


class PyBulletSimulator:
    def __init__(self, physics_hz=240, control_hz=60, gui=False,
                 initial_position=None, initial_rpy=None):
        try:
            from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
            from gym_pybullet_drones.utils.enums import DroneModel, Physics
        except ImportError as exc:
            raise RuntimeError("gym-pybullet-drones is required; install requirements.txt") from exc
        import pybullet as bullet

        class DisturbanceAviary(CtrlAviary):
            """Inject configured world-frame wrench on every physics substep."""
            external_force = np.zeros(3)
            external_torque = np.zeros(3)

            def _physics(aviary, rpm, drone_id):
                if np.any(aviary.external_force) or np.any(aviary.external_torque):
                    body = aviary.DRONE_IDS[drone_id]
                    pos, _ = bullet.getBasePositionAndOrientation(body, physicsClientId=aviary.CLIENT)
                    bullet.applyExternalForce(body, -1, aviary.external_force.tolist(), pos,
                                              bullet.WORLD_FRAME, physicsClientId=aviary.CLIENT)
                    bullet.applyExternalTorque(body, -1, aviary.external_torque.tolist(),
                                               bullet.WORLD_FRAME, physicsClientId=aviary.CLIENT)
                return super(DisturbanceAviary, aviary)._physics(rpm, drone_id)

        initial_xyzs = None if initial_position is None else np.asarray(initial_position, float).reshape(1,3)
        initial_rpys = None if initial_rpy is None else np.asarray(initial_rpy, float).reshape(1,3)
        self.env = DisturbanceAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB,
                              pyb_freq=int(physics_hz), ctrl_freq=int(control_hz), gui=bool(gui),
                              initial_xyzs=initial_xyzs, initial_rpys=initial_rpys,
                              record=False, obstacles=False, user_debug_gui=False)
        self.obs, self.info = self.env.reset()

    def state(self):
        s = np.asarray(self.obs[0], dtype=float)
        # BaseAviary's linear and angular velocities come from PyBullet's world-frame
        # base velocity. The controller convention stores angular velocity in body axes.
        from scipy.spatial.transform import Rotation
        q = s[3:7]
        R = Rotation.from_quat(q).as_matrix()
        return {"position": s[0:3].copy(), "quaternion": q.copy(),
                "R": R, "euler": s[7:10].copy(),
                "velocity": s[10:13].copy(), "omega": R.T @ s[13:16]}

    def step(self, rpm):
        rpm = np.asarray(rpm, dtype=float).reshape((1, 4))
        result = self.env.step(rpm)
        if len(result) == 5:
            self.obs, _, terminated, truncated, self.info = result
        else:
            self.obs, _, done, self.info = result
            terminated, truncated = done, False
        if bool(terminated) or bool(truncated):
            raise RuntimeError("PyBullet episode terminated unexpectedly")

    def set_disturbance(self, force_world, torque_world):
        self.env.external_force = np.asarray(force_world, float).reshape(3)
        self.env.external_torque = np.asarray(torque_world, float).reshape(3)

    def close(self): self.env.close()
