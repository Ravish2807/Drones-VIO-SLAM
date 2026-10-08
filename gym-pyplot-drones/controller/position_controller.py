import numpy as np
from controller.attitude_generation import AttitudeGenerator


class PositionController:
    def __init__(self,kp,kv,mass,gravity=9.81,max_tilt_deg=30):
        self.kp,self.kv=np.asarray(kp,float).reshape(3),np.asarray(kv,float).reshape(3)
        self.mass,self.gravity=float(mass),float(gravity); self.max_tilt=np.deg2rad(max_tilt_deg)
        self.attitude_generator=AttitudeGenerator()

    def compute(self,position,velocity,desired_position,desired_velocity=None,desired_acceleration=None,yaw=0):
        p,v,pd=np.asarray(position).reshape(3),np.asarray(velocity).reshape(3),np.asarray(desired_position).reshape(3)
        vd=np.zeros(3) if desired_velocity is None else np.asarray(desired_velocity).reshape(3)
        ad=np.zeros(3) if desired_acceleration is None else np.asarray(desired_acceleration).reshape(3)
        ep,ev=pd-p,vd-v; accel=ad+self.kp*ep+self.kv*ev
        force=self.mass*(accel+np.array([0.,0.,self.gravity]))
        limit=max(force[2],1e-6)*np.tan(self.max_tilt); lateral=np.linalg.norm(force[:2])
        if lateral>limit: force[:2]*=limit/lateral
        thrust=float(np.linalg.norm(force)); R_des=self.attitude_generator.compute(force,yaw)
        return {"position_error":ep,"velocity_error":ev,"acceleration":accel,"force":force,"thrust":thrust,"R_des":R_des}
