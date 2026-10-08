import csv
import json
from pathlib import Path
import numpy as np
from math_utils.so3 import matrix_to_euler
from math_utils.so3 import rotation_error


class DataLogger:
    """Record reference, true/measured state, errors, inputs, and environment."""
    def __init__(self): self.rows=[]

    def record(self,t,state,ref,pc,ac,mix,environment=None,measured_state=None):
        measured_state = state if measured_state is None else measured_state
        env = environment or {}
        row={"time":float(t)}
        fields={
            "position":state["position"],"measured_position":measured_state["position"],
            "desired_position":ref["position"],"velocity":state["velocity"],
            "measured_velocity":measured_state["velocity"],"desired_velocity":ref["velocity"],
            "position_error":np.asarray(ref["position"])-np.asarray(state["position"]),
            "measured_position_error":pc["position_error"],
            "velocity_error":np.asarray(ref["velocity"])-np.asarray(state["velocity"]),
            "measured_velocity_error":pc["velocity_error"],
            "acceleration_des":pc["acceleration"],"euler":state["euler"],
            "desired_euler":matrix_to_euler(pc["R_des"]),"attitude_error":rotation_error(pc["R_des"],state["R"]),
            "measured_attitude_error":ac["attitude_error"],
            "omega":state["omega"],"measured_omega":measured_state["omega"],
            "desired_omega":np.zeros(3),"angular_velocity_error":ac["angular_velocity_error"],
            "torque":ac["torque"],"motor_rpm":mix["rpm"],
            "external_force":env.get("force",np.zeros(3)),"external_torque":env.get("torque",np.zeros(3)),
        }
        for key,val in fields.items():
            for i,x in enumerate(np.asarray(val).reshape(-1)): row[f"{key}_{i}"]=float(x)
        for prefix,rotation in (("R",state["R"]),("R_des",pc["R_des"])):
            for i,x in enumerate(np.asarray(rotation).reshape(-1)): row[f"{prefix}_{i}"]=float(x)
        for i,x in enumerate(np.asarray(measured_state["R"]).reshape(-1)): row[f"R_measured_{i}"]=float(x)
        rel=np.asarray(pc["R_des"]).T@np.asarray(state["R"])
        row["attitude_error_angle_rad"]=float(np.arccos(np.clip((np.trace(rel)-1.)/2.,-1.,1.)))
        row["thrust"]=float(pc["thrust"])
        row["motor_saturated"]=int(bool(mix["saturated"]))
        for category,terms in env.get("controller_terms",{}).items():
            if isinstance(terms,dict):
                for term,value in terms.items():
                    for i,x in enumerate(np.asarray(value).reshape(-1)): row[f"term_{term}_{i}"]=float(x)
        self.rows.append(row)

    def save_csv(self,path):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        if self.rows:
            with path.open("w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=list(self.rows[0])); w.writeheader(); w.writerows(self.rows)

    def metrics(self,settling_tolerance=.05,step_time=None,step_start=None,step_goal=None,step_axis=0,
                disturbance_end=None,recovery_tolerance=.05,attitude_step_time=None,attitude_tolerance_deg=2.):
        if not self.rows: return {}
        t=np.array([r["time"] for r in self.rows]); n=len(t)
        err=np.array([[r[f"position_error_{i}"] for i in range(3)] for r in self.rows])
        pos=np.array([[r[f"position_{i}"] for i in range(3)] for r in self.rows])
        pd=np.array([[r[f"desired_position_{i}"] for i in range(3)] for r in self.rows])
        er=np.array([[r[f"attitude_error_{i}"] for i in range(3)] for r in self.rows])
        angle=np.array([r["attitude_error_angle_rad"] for r in self.rows])
        tau=np.array([[r[f"torque_{i}"] for i in range(3)] for r in self.rows])
        thrust=np.array([r["thrust"] for r in self.rows]); sat=np.array([r["motor_saturated"] for r in self.rows])
        dt=float(np.median(np.diff(t))) if n>1 else 0.
        metrics={
            "position_rmse_3d_m":float(np.sqrt(np.mean(np.sum(err**2,axis=1)))),
            "position_max_error_3d_m":float(np.max(np.linalg.norm(err,axis=1))),
            "attitude_error_vector_rmse_rad":float(np.sqrt(np.mean(np.sum(er**2,axis=1)))),
            "attitude_error_angle_rmse_deg":float(np.rad2deg(np.sqrt(np.mean(angle**2)))),
            "attitude_error_angle_max_deg":float(np.rad2deg(np.max(angle))),
            "thrust_squared_integral_N2s":float(np.sum(thrust**2)*dt),
            "torque_squared_integral_N2m2s":float(np.sum(np.sum(tau**2,axis=1))*dt),
            "motor_saturation_samples":int(np.sum(sat)),
            "motor_saturation_percent":float(100.*np.mean(sat)),
            "motor_saturation_duration_s":float(np.sum(sat)*dt),
            "samples":n,
        }
        measured_error=np.array([[r[f"measured_position_error_{i}"] for i in range(3)] for r in self.rows])
        metrics["measured_position_rmse_3d_m"]=float(np.sqrt(np.mean(np.sum(measured_error**2,axis=1))))
        metrics["measurement_position_noise_rmse_m"]=float(np.sqrt(np.mean(np.sum((pos-np.array([[r[f"measured_position_{i}"] for i in range(3)] for r in self.rows]))**2,axis=1))))
        for i,axis in enumerate("xyz"):
            metrics[f"position_rmse_{axis}_m"]=float(np.sqrt(np.mean(err[:,i]**2)))
            metrics[f"position_max_abs_error_{axis}_m"]=float(np.max(np.abs(err[:,i])))
            metrics[f"position_final_error_{axis}_m"]=float(err[-1,i])
        window=t>=max(t[0],t[-1]-min(2.,max(0.,t[-1]-t[0])))
        metrics["final_window_altitude_tracking_rmse_m"]=float(np.sqrt(np.mean(err[window,2]**2)))
        metrics["steady_state_position_error_3d_m"]=float(np.linalg.norm(np.mean(err[window],axis=0)))
        if step_time is not None and step_start is not None and step_goal is not None:
            axis=int(step_axis); y=pos[:,axis]; delta=float(step_goal-step_start); direction=1. if delta>=0 else -1.
            after=np.flatnonzero(t>=step_time); metrics.update({"step_rise_time_10_90_s":None,"step_settling_time_s":None,"step_overshoot_m":None})
            if len(after) and abs(delta)>1e-9:
                fraction=direction*(y[after]-step_start)/abs(delta)
                i10=np.flatnonzero(fraction>=.1); i90=np.flatnonzero(fraction>=.9)
                if len(i10) and len(i90) and i90[0]>=i10[0]: metrics["step_rise_time_10_90_s"]=float(t[after[i90[0]]]-t[after[i10[0]]])
                metrics["step_overshoot_m"]=float(max(0.,np.max(direction*(y[after]-step_goal))))
                within=np.abs(y-step_goal)<=settling_tolerance
                settled=np.flatnonzero((t>=step_time)&within&np.logical_and.accumulate(within[::-1])[::-1])
                if len(settled): metrics["step_settling_time_s"]=float(t[settled[0]]-step_time)
        if disturbance_end is not None and t[-1]>=disturbance_end:
            after=np.flatnonzero(t>=disturbance_end); in_band=np.linalg.norm(err,axis=1)<=recovery_tolerance
            recovered=np.flatnonzero((t>=disturbance_end)&in_band&np.logical_and.accumulate(in_band[::-1])[::-1])
            metrics["disturbance_recovery_time_s"]=float(t[recovered[0]]-disturbance_end) if len(recovered) else None
        if attitude_step_time is not None:
            angle_deg=np.rad2deg(angle); in_band=angle_deg<=attitude_tolerance_deg
            settled=np.flatnonzero((t>=attitude_step_time)&in_band&np.logical_and.accumulate(in_band[::-1])[::-1])
            metrics["attitude_settling_time_2deg_s"]=float(t[settled[0]]-attitude_step_time) if len(settled) else None
        metrics["numerically_finite"] = bool(np.all(np.isfinite(err)) and np.all(np.isfinite(angle)))
        return {key:(None if isinstance(value,(float,np.floating)) and not np.isfinite(value) else value)
                for key,value in metrics.items()}

    def save_metrics(self,path,metrics): Path(path).write_text(json.dumps(metrics,indent=2,allow_nan=False)+"\n")
