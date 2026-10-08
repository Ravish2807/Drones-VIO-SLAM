"""Phase 2 run conditions kept separate from the baseline control laws."""
from dataclasses import dataclass, field
import numpy as np
from math_utils.so3 import matrix_to_euler


@dataclass
class RunConditions:
    initial_position: list[float] | None = None
    initial_rpy_deg: list[float] = field(default_factory=lambda: [0., 0., 0.])
    gain_scale: float = 1.0
    gain_scales: dict[str, float] = field(default_factory=dict)
    motor_max_scale: float = 1.0
    noise_std: dict[str, float] = field(default_factory=dict)
    disturbance_force: list[float] = field(default_factory=lambda: [0., 0., 0.])
    disturbance_torque: list[float] = field(default_factory=lambda: [0., 0., 0.])
    disturbance_start: float = 0.0
    disturbance_duration: float = 0.0
    random_seed: int = 1

    def active_disturbance(self, t):
        active = self.disturbance_start <= t < self.disturbance_start + self.disturbance_duration
        return (np.asarray(self.disturbance_force, float) if active else np.zeros(3),
                np.asarray(self.disturbance_torque, float) if active else np.zeros(3))

    def noisy_state(self, state, rng):
        measured = {k: np.array(v, copy=True) if isinstance(v, np.ndarray) else v for k,v in state.items()}
        for key in ("position", "velocity", "omega"):
            sigma = float(self.noise_std.get(key, 0.))
            if sigma:
                measured[key] = state[key] + rng.normal(0., sigma, 3)
        sigma = float(self.noise_std.get("attitude", 0.))
        if sigma:
            from scipy.spatial.transform import Rotation
            delta = Rotation.from_rotvec(rng.normal(0., sigma, 3)).as_matrix()
            measured["R"] = state["R"] @ delta
            measured["euler"] = matrix_to_euler(measured["R"])
        return measured

    def to_dict(self):
        return {"initial_position":self.initial_position,"initial_rpy_deg":self.initial_rpy_deg,
                "gain_scale":self.gain_scale,"gain_scales":self.gain_scales,"motor_max_scale":self.motor_max_scale,"noise_std":self.noise_std,
                "disturbance_force_N":self.disturbance_force,"disturbance_torque_Nm":self.disturbance_torque,
                "disturbance_start_s":self.disturbance_start,"disturbance_duration_s":self.disturbance_duration,
                "random_seed":self.random_seed}
