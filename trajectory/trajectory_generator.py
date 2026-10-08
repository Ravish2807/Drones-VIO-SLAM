import numpy as np


def hold(position,yaw=0.):
    p=np.asarray(position,float).reshape(3)
    return lambda t:{"position":p.copy(),"velocity":np.zeros(3),"acceleration":np.zeros(3),"yaw":float(yaw)}


def smooth_waypoints(waypoints,segment_duration=2.,yaw=0.):
    pts=np.asarray(waypoints,float).reshape(-1,3)
    if len(pts)<2 or segment_duration<=0: raise ValueError("Need two waypoints and positive segment duration")
    def sample(t):
        s=np.clip(float(t)/segment_duration,0,len(pts)-1); i=min(int(s),len(pts)-2); u=s-i
        if t>=segment_duration*(len(pts)-1): return {"position":pts[-1].copy(),"velocity":np.zeros(3),"acceleration":np.zeros(3),"yaw":float(yaw)}
        h=10*u**3-15*u**4+6*u**5; dh=(30*u**2-60*u**3+30*u**4)/segment_duration
        ddh=(60*u-180*u**2+120*u**3)/segment_duration**2; d=pts[i+1]-pts[i]
        return {"position":pts[i]+h*d,"velocity":dh*d,"acceleration":ddh*d,"yaw":float(yaw)}
    return sample


def phase1_demo(): return smooth_waypoints([[0,0,.08],[0,0,1],[0,0,1],[1,0,1],[0,0,1],[0,0,.08]],3.)
