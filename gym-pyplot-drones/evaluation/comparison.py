from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


COMPARE_METRICS=("position_rmse_3d_m","position_max_error_3d_m","attitude_error_angle_rmse_deg",
                 "step_settling_time_s","step_overshoot_m","thrust_squared_integral_N2s",
                 "torque_squared_integral_N2m2s","motor_saturation_percent","disturbance_recovery_time_s",
                 "attitude_settling_time_2deg_s")


def save_comparison(rows,output_dir):
    out=Path(output_dir); out.mkdir(parents=True,exist_ok=True)
    fields=list(dict.fromkeys(key for row in rows for key in row))
    with (out/"comparison.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    cases=list(dict.fromkeys(r["case"] for r in rows)); controllers=list(dict.fromkeys(r["controller"] for r in rows))
    plotted=[m for m in COMPARE_METRICS if any(m in r and isinstance(r[m],(int,float)) and r[m] is not None for r in rows)]
    if plotted:
        fig,axes=plt.subplots(len(plotted),1,figsize=(11,max(3,2.5*len(plotted))),squeeze=False)
        for ax,metric in zip(axes[:,0],plotted):
            x=np.arange(len(cases)); width=.8/max(1,len(controllers))
            for ci,name in enumerate(controllers):
                vals=[]
                for case in cases:
                    found=next((r.get(metric) for r in rows if r["case"]==case and r["controller"]==name),None)
                    vals.append(np.nan if found is None else found)
                ax.bar(x+(ci-(len(controllers)-1)/2)*width,vals,width,label=name)
            ax.set_ylabel(metric); ax.set_xticks(x,cases,rotation=35,ha="right"); ax.grid(axis="y",alpha=.3); ax.legend()
        fig.tight_layout(); fig.savefig(out/"comparison.png",dpi=150); plt.close(fig)
