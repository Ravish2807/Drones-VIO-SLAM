import numpy as np
from math_utils.so3 import rotation_error


class AttitudeController:
    def __init__(self,kp,kw): self.kp,self.kw=np.asarray(kp,float).reshape(3),np.asarray(kw,float).reshape(3)
    def compute(self,R,omega,R_des,omega_des=None):
        omega=np.asarray(omega,float).reshape(3); omega_des=np.zeros(3) if omega_des is None else np.asarray(omega_des,float).reshape(3)
        eR=rotation_error(R_des,R); ew=omega-omega_des
        return {"attitude_error":eR,"angular_velocity_error":ew,"torque":-self.kp*eR-self.kw*ew}
