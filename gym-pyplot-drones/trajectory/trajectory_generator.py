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


def step(start=(0., 0., 1.), goal=(1., 0., 1.), step_time=2.):
    p0, p1 = np.asarray(start, float).reshape(3), np.asarray(goal, float).reshape(3)
    def sample(t):
        p = p0 if t < step_time else p1
        return {"position": p.copy(), "velocity": np.zeros(3), "acceleration": np.zeros(3), "yaw": 0.}
    return sample


def square(center=(0., 0., 1.), side=1., segment_duration=3.):
    cx, cy, cz = np.asarray(center, float).reshape(3)
    points = [[cx,cy,cz],[cx+side,cy,cz],[cx+side,cy+side,cz],
              [cx,cy+side,cz],[cx,cy,cz]]
    return smooth_waypoints(points, segment_duration)


def circle(center=(0., 0., 1.), radius=.5, angular_rate=.5):
    cx, cy, cz = np.asarray(center, float).reshape(3)
    def sample(t):
        a = angular_rate * float(t)
        return {"position":np.array([cx+radius*np.cos(a),cy+radius*np.sin(a),cz]),
                "velocity":np.array([-radius*angular_rate*np.sin(a),radius*angular_rate*np.cos(a),0.]),
                "acceleration":np.array([-radius*angular_rate**2*np.cos(a),-radius*angular_rate**2*np.sin(a),0.]),"yaw":0.}
    return sample


def trajectory_3d(radii=(.5,.5,.25), rates=(.4,.5,.6), center=(0.,0.,1.)):
    radii, rates, center = np.asarray(radii,float), np.asarray(rates,float), np.asarray(center,float)
    if radii.shape != (3,) or rates.shape != (3,) or center.shape != (3,):
        raise ValueError("radii, rates, and center must each have 3 elements")
    def sample(t):
        a = rates*float(t)
        return {"position":center+radii*np.sin(a), "velocity":radii*rates*np.cos(a),
                "acceleration":-radii*rates**2*np.sin(a), "yaw":0.}
    return sample
