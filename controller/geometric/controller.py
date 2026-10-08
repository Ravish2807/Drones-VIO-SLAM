"""Geometric SO(3) controller with rigid-body gyroscopic compensation."""
import numpy as np
from controller.baseline.controller import BaselineController


class GeometricSO3Controller(BaselineController):
    name = "geometric_so3"

    def __init__(self,config):
        super().__init__(config)
        inertia=config["inertia"]
        self.inertia=np.diag(np.asarray(inertia,float)) if np.asarray(inertia).ndim==1 else np.asarray(inertia,float)

    def attitude_torque(self,state,R_des,omega_des):
        torque,eR,ew,terms=super().attitude_torque(state,R_des,omega_des)
        # Desired body rate is zero in the Phase 3 attitude tests and current
        # position references; this is the corresponding Euler rigid-body term.
        compensation=np.cross(state.angular_velocity,self.inertia@state.angular_velocity)
        return torque+compensation,eR,ew,{**terms,"gyroscopic_compensation":compensation}
