"""Simplified point-mass/attitude placeholder; not a flight validation model."""
import numpy as np
from math_utils.so3 import project_to_so3, hat, matrix_to_euler


class DryRunPlant:
    def __init__(self, inertia, gravity, dt, mass=.027, initial_position=None, initial_rpy=None):
        self.J=np.asarray(inertia,float); self.g=float(gravity); self.dt=float(dt)
        from scipy.spatial.transform import Rotation
        self.m=float(mass)
        self.p=np.array([0.,0.,.08]) if initial_position is None else np.asarray(initial_position,float).reshape(3)
        self.v=np.zeros(3); self.R=np.eye(3) if initial_rpy is None else Rotation.from_euler("xyz",initial_rpy).as_matrix()
        self.w=np.zeros(3); self.force_dist=np.zeros(3); self.torque_dist=np.zeros(3)
    def state(self):
        return {"position":self.p.copy(),"velocity":self.v.copy(),"R":self.R.copy(),"omega":self.w.copy(),
                "euler":matrix_to_euler(self.R),"quaternion":np.array([0.,0.,0.,1.])}
    def set_disturbance(self, force_world, torque_world):
        self.force_dist=np.asarray(force_world,float).reshape(3); self.torque_dist=np.asarray(torque_world,float).reshape(3)
    def step(self,rpm,mixer):
        # CF2X thrust/moment coefficients and mixer use RPM squared.
        w2=np.asarray(rpm)**2; wrench=mixer.B@w2
        acc=(wrench[0]/mixer.mass*(self.R@np.array([0.,0.,1.]))-np.array([0.,0.,self.g])
             +self.force_dist/mixer.mass)
        self.v+=acc*self.dt; self.p+=self.v*self.dt
        torque_body=wrench[1:]+self.R.T@self.torque_dist
        self.w+=torque_body/self.J*self.dt; self.R=project_to_so3(self.R@(np.eye(3)+hat(self.w*self.dt)))
