import numpy as np


class MotorMixer:
    """Map [T,tau_x,tau_y,tau_z] to squared rotor angular speeds."""
    def __init__(self,mass,gravity,kf,km,rotor_positions,spin_directions,min_rpm,max_rpm):
        self.mass,self.gravity,self.kf,self.km=float(mass),float(gravity),float(kf),float(km)
        self.positions=np.asarray(rotor_positions,float).reshape(4,3); self.spin=np.asarray(spin_directions,float).reshape(4)
        self.min_rpm,self.max_rpm=float(min_rpm),float(max_rpm)
        # r x [0, 0, kf*w^2] = [y*kf*w^2, -x*kf*w^2, 0].
        self.B=np.vstack((np.full(4,self.kf),self.kf*self.positions[:,1],-self.kf*self.positions[:,0],self.km*self.spin))
        self._pinv=np.linalg.pinv(self.B)
    def mix(self,thrust,torque):
        raw=self._pinv@np.r_[float(thrust),np.asarray(torque,float).reshape(3)]; clipped=np.clip(raw,self.min_rpm**2,self.max_rpm**2)
        return {"rpm":np.sqrt(clipped),"omega_squared":clipped,"saturated":bool(np.any(np.abs(raw-clipped)>1e-6)),"achieved_wrench":self.B@clipped}
    def hover_rpm(self): return np.full(4,np.sqrt(self.mass*self.gravity/(4*self.kf)))
