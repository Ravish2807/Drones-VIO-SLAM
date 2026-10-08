from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def make_plots(rows,output_dir):
    out=Path(output_dir); out.mkdir(parents=True,exist_ok=True); t=np.array([r["time"] for r in rows])
    def vectors(file,groups,labels,title,ylabel):
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
        for i,ax in enumerate(axes):
            for key,label in zip(groups,labels): ax.plot(t,[r[f"{key}_{i}"] for r in rows],label=label)
            ax.set_ylabel(f"{ylabel}[{i}]"); ax.grid(True); ax.legend()
        axes[-1].set_xlabel("time [s]"); fig.suptitle(title); fig.tight_layout(); fig.savefig(out/file,dpi=150); plt.close(fig)
    vectors("position.png",["desired_position","position"],["desired","actual"],"Position tracking","m ")
    vectors("position_error.png",["position_error"],["error"],"Position error","m ")
    vectors("velocity.png",["desired_velocity","velocity"],["desired","actual"],"Velocity","m/s ")
    vectors("attitude.png",["desired_euler","euler"],["desired","actual"],"Attitude (roll, pitch, yaw)","rad ")
    vectors("attitude_error.png",["attitude_error"],["error"],"SO(3) attitude error","rad ")
    vectors("angular_velocity.png",["omega"],["body rate"],"Angular velocity","rad/s ")
    fig,ax=plt.subplots(figsize=(10,4)); ax.plot(t,np.rad2deg([r["attitude_error_angle_rad"] for r in rows]))
    ax.set(xlabel="time [s]",ylabel="rotation error [deg]",title="Geodesic attitude error"); ax.grid(True); fig.tight_layout(); fig.savefig(out/"attitude_error_angle.png",dpi=150); plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(10,7),sharex=True); ax[0].plot(t,[r["thrust"] for r in rows]); ax[0].set_ylabel("T [N]"); ax[0].grid(True)
    for i,label in enumerate(("τx","τy","τz")): ax[1].plot(t,[r[f"torque_{i}"] for r in rows],label=label)
    ax[1].set_ylabel("torque [N m]"); ax[1].set_xlabel("time [s]"); ax[1].legend(); ax[1].grid(True)
    fig.suptitle("Control inputs"); fig.tight_layout(); fig.savefig(out/"control_inputs.png",dpi=150); plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5))
    for i in range(4): ax.plot(t,[r[f"motor_rpm_{i}"] for r in rows],label=f"M{i+1}")
    ax.set(xlabel="time [s]",ylabel="RPM",title="Motor commands"); ax.grid(True); ax.legend(); fig.tight_layout(); fig.savefig(out/"motors.png",dpi=150); plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for i,label in enumerate(("Fx","Fy","Fz")): ax[0].plot(t,[r[f"external_force_{i}"] for r in rows],label=label)
    for i,label in enumerate(("τx","τy","τz")): ax[1].plot(t,[r[f"external_torque_{i}"] for r in rows],label=label)
    ax[0].set_ylabel("force [N]"); ax[1].set_ylabel("torque [N m]"); ax[1].set_xlabel("time [s]")
    for a in ax: a.grid(True); a.legend()
    fig.suptitle("External disturbances"); fig.tight_layout(); fig.savefig(out/"disturbances.png",dpi=150); plt.close(fig)
    fig=plt.figure(figsize=(8,6)); ax=fig.add_subplot(111,projection="3d")
    for key,label in (("position","actual"),("desired_position","desired")):
        xyz=np.array([[r[f"{key}_{i}"] for i in range(3)] for r in rows]); ax.plot(*xyz.T,label=label)
    ax.set(xlabel="x [m]",ylabel="y [m]",zlabel="z [m]",title="3D trajectory"); ax.legend(); fig.tight_layout(); fig.savefig(out/"trajectory.png",dpi=150); plt.close(fig)
