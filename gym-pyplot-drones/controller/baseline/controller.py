"""Phase 2 baseline laws, retained as the fixed comparison reference."""
import numpy as np
from controller.interface import Controller, ControlOutput
from math_utils.so3 import rotation_error


class BaselineController(Controller):
    name = "baseline"

    def __init__(self, config):
        pg=config["position_gains"]; ag=config["attitude_gains"]
        self.position_kp=np.asarray(pg["kp"],float); self.position_kv=np.asarray(pg["kv"],float)
        self.mass=float(config["mass"]); self.gravity=float(config["gravity"])
        self.max_tilt=np.deg2rad(float(config["max_tilt_deg"]))
        self.attitude_kp=np.asarray(ag["kp"],float); self.attitude_kw=np.asarray(ag["kw"],float)

    @staticmethod
    def desired_rotation(force,yaw):
        b3=force/max(np.linalg.norm(force),1e-9); b1c=np.array([np.cos(yaw),np.sin(yaw),0.])
        b2=np.cross(b3,b1c)
        if np.linalg.norm(b2)<1e-8: b2=np.array([-np.sin(yaw),np.cos(yaw),0.])
        b2/=np.linalg.norm(b2); return np.column_stack((np.cross(b2,b3),b2,b3))

    def attitude_torque(self, state, R_des, omega_des):
        eR=rotation_error(R_des,state.rotation); ew=state.angular_velocity-omega_des
        torque=-self.attitude_kp*eR-self.attitude_kw*ew
        return torque,eR,ew,{"proportional":-self.attitude_kp*eR,"damping":-self.attitude_kw*ew}

    def compute(self,state,desired):
        if desired.attitude_only:
            R_des=desired.rotation if desired.rotation is not None else np.eye(3)
            thrust=float(desired.thrust if desired.thrust is not None else 0.)
            force=thrust*R_des[:,2]; ep=desired.position-state.position; ev=desired.velocity-state.velocity
            accel=np.zeros(3)
        else:
            ep=desired.position-state.position; ev=desired.velocity-state.velocity
            accel=desired.acceleration+self.position_kp*ep+self.position_kv*ev
            force=self.mass*(accel+np.array([0.,0.,self.gravity]))
            lateral_limit=max(force[2],1e-6)*np.tan(self.max_tilt); lateral=np.linalg.norm(force[:2])
            if lateral>lateral_limit: force[:2]*=lateral_limit/lateral
            computed_thrust=float(np.linalg.norm(force)); computed_R=self.desired_rotation(force,desired.yaw)
            R_des=desired.rotation if desired.rotation is not None else computed_R
            thrust=float(desired.thrust if desired.thrust is not None else computed_thrust)
            if desired.rotation is not None: force=thrust*R_des[:,2]
        torque,eR,ew,terms=self.attitude_torque(state,R_des,desired.angular_velocity)
        return ControlOutput(thrust,torque,np.asarray(R_des),np.asarray(ep),np.asarray(ev),np.asarray(accel),
                             np.asarray(force),eR,ew,{"attitude_terms":terms})
