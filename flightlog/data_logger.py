import csv
from pathlib import Path
import numpy as np
from math_utils.so3 import matrix_to_euler


class DataLogger:
    def __init__(self): self.rows=[]
    def record(self,t,state,ref,pc,ac,mix):
        row={"time":float(t)}
        fields={"position":state["position"],"desired_position":ref["position"],"velocity":state["velocity"],
                "desired_velocity":ref["velocity"],"position_error":pc["position_error"],"acceleration_des":pc["acceleration"],
                "euler":state["euler"],"desired_euler":matrix_to_euler(pc["R_des"]),"attitude_error":ac["attitude_error"],
                "omega":state["omega"],"torque":ac["torque"],"motor_rpm":mix["rpm"]}
        for key,val in fields.items():
            for i,x in enumerate(np.asarray(val).reshape(-1)): row[f"{key}_{i}"]=float(x)
        row["thrust"]=float(pc["thrust"]); row["motor_saturated"]=int(mix["saturated"]); self.rows.append(row)
    def save_csv(self,path):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        if self.rows:
            with path.open("w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=list(self.rows[0])); w.writeheader(); w.writerows(self.rows)
    def metrics(self,hover_target=1.,hover_window=2.):
        if not self.rows: return {}
        t=np.array([r["time"] for r in self.rows]); n=len(t)
        p=np.array([[r[f"position_{i}"] for i in range(3)] for r in self.rows]); pd=np.array([[r[f"desired_position_{i}"] for i in range(3)] for r in self.rows])
        er=np.array([[r[f"attitude_error_{i}"] for i in range(3)] for r in self.rows]); tau=np.array([[r[f"torque_{i}"] for i in range(3)] for r in self.rows])
        thrust=np.array([r["thrust"] for r in self.rows]); dt=float(np.median(np.diff(t))) if n>1 else 0.
        mask=t>=max(t[0],t[-1]-hover_window)
        return {"position_rmse_m":float(np.sqrt(np.mean(np.sum((pd-p)**2,axis=1)))),
                "attitude_error_rmse_rad":float(np.sqrt(np.mean(np.sum(er**2,axis=1)))),
                "final_window_altitude_tracking_rmse_m":float(np.sqrt(np.mean((p[mask,2]-pd[mask,2])**2))),
                "final_window_hover_altitude_error_m":float(np.sqrt(np.mean((p[mask,2]-hover_target)**2))),
                "thrust_squared_integral":float(np.sum(thrust**2)*dt),
                "torque_squared_integral":float(np.sum(np.sum(tau**2,axis=1))*dt),
                "motor_saturation_samples":int(sum(r["motor_saturated"] for r in self.rows)),"samples":n}
    def save_metrics(self,path,metrics):
        Path(path).write_text("Phase 1 flight metrics\n"+"".join(f"{k}: {v}\n" for k,v in metrics.items()))
