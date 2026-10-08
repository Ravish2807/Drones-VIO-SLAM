from pathlib import Path
import time
import json
import platform
import yaml
import numpy as np

from controller.factory import create_controller
from controller.interface import ControlState, DesiredState
from controller.motor_mixer import MotorMixer
from experiments.conditions import RunConditions
from flightlog.data_logger import DataLogger
from simulation.simulator import PyBulletSimulator
from trajectory.trajectory_generator import hold, smooth_waypoints, phase1_demo, step, square, circle, trajectory_3d
from visualization.plots import make_plots


DEFAULT_DURATION={"hover":10.,"position_step":12.,"square":16.,"circle":20.,"trajectory_3d":20.,"trajectory":12.,"demo":15.,
                 "attitude_roll":6.,"attitude_pitch":6.,"attitude_yaw":6.,"attitude_combined":6.}


def build_reference(name):
    if name=="hover": return hold([0,0,1])
    if name=="position_step": return step([0,0,1],[1,0,1],2.)
    if name=="square": return square([0,0,1],1.,3.)
    if name=="circle": return circle([0,0,1],.5,.5)
    if name=="trajectory_3d": return trajectory_3d((.5,.5,.25),(.4,.5,.6),(0,0,1))
    if name=="trajectory": return smooth_waypoints([[0,0,.08],[0,0,1],[.5,.5,1],[0,0,1]],3.)
    if name=="demo": return phase1_demo()
    if name.startswith("attitude_"):
        from scipy.spatial.transform import Rotation
        axis=name.removeprefix("attitude_")
        angles={"roll":[10.,0.,0.],"pitch":[0.,10.,0.],"yaw":[0.,0.,10.],"combined":[10.,-10.,15.]}
        if axis not in angles: raise ValueError(f"Unknown attitude test: {name}")
        Rd=Rotation.from_euler("xyz",angles[axis],degrees=True).as_matrix(); identity=np.eye(3)
        return lambda t:{"position":np.array([0.,0.,1.]),"velocity":np.zeros(3),"acceleration":np.zeros(3),
                         "yaw":0.,"R":identity.copy() if t<1. else Rd.copy(),"omega":np.zeros(3),
                         "thrust":.027*9.81,"attitude_only":True}
    raise ValueError(f"Unknown experiment: {name}")


def run(experiment="hover",duration=None,gui=False,output=None,dry_run=False,
        config_path="config/baseline_phase2.yaml",conditions=None,controller_name="baseline"):
    with Path(config_path).open() as f: c=yaml.safe_load(f)
    condition=conditions or RunConditions()
    if isinstance(condition,dict): condition=RunConditions(**condition)
    duration=float(duration if duration is not None else DEFAULT_DURATION.get(experiment,10.))
    if duration<=0: raise ValueError("duration must be positive and finite")
    ref=build_reference(experiment); dt=1./c["control_hz"]
    scales={key:condition.gain_scale*float(condition.gain_scales.get(key,1.)) for key in ("position_kp","position_kv","attitude_kp","attitude_kw")}
    control_config={**c,"position_gains":{"kp":np.asarray(c["position_gains"]["kp"],float)*scales["position_kp"],
                                          "kv":np.asarray(c["position_gains"]["kv"],float)*scales["position_kv"]},
                    "attitude_gains":{"kp":np.asarray(c["attitude_gains"]["kp"],float)*scales["attitude_kp"],
                                      "kw":np.asarray(c["attitude_gains"]["kw"],float)*scales["attitude_kw"]}}
    controller=create_controller(controller_name,control_config)
    mixer=MotorMixer(c["mass"],c["gravity"],c["thrust_coefficient"],c["moment_coefficient"],
                     c["rotor_positions"],c["rotor_spin_directions"],c["min_rpm"],c["max_rpm"]*condition.motor_max_scale)
    initial=condition.initial_position
    if initial is None: initial=[0.,0.,1.] if experiment in ("hover","position_step","square","circle","trajectory_3d") or experiment.startswith("attitude_") else [0.,0.,.08]
    condition.initial_position=list(map(float,initial))
    initial_rpy=np.deg2rad(np.asarray(condition.initial_rpy_deg,float))
    if dry_run:
        from simulation.dry_run import DryRunPlant
        sim=DryRunPlant(c["inertia"],c["gravity"],dt,c["mass"],initial,initial_rpy)
    else:
        sim=PyBulletSimulator(c["physics_hz"],c["control_hz"],gui,initial,initial_rpy)
    rng=np.random.default_rng(condition.random_seed); logger=DataLogger()
    count=int(np.ceil(duration*c["control_hz"])); started=time.time()
    step_args={"step_time":2.,"step_start":0.,"step_goal":1.,"step_axis":0} if experiment=="position_step" else {}
    attitude_step_time=1. if experiment.startswith("attitude_") else None
    try:
        for k in range(count):
            t=k*dt
            true_state=sim.state(); measured=condition.noisy_state(true_state,rng); desired=ref(t)
            state_in=ControlState.from_mapping(measured); desired_in=DesiredState.from_mapping(desired)
            out=controller.compute(state_in,desired_in)
            pc={"position_error":out.position_error,"velocity_error":out.velocity_error,"acceleration":out.desired_acceleration,
                "force":out.desired_force,"thrust":out.thrust,"R_des":out.desired_rotation}
            ac={"attitude_error":out.attitude_error,"angular_velocity_error":out.angular_velocity_error,"torque":out.torque}
            mix=mixer.mix(out.thrust,out.torque)
            force,torque=condition.active_disturbance(t)
            sim.set_disturbance(force,torque)
            logger.record(t,true_state,desired,pc,ac,mix,{"force":force,"torque":torque,"controller_terms":out.terms},measured)
            if dry_run: sim.step(mix["rpm"],mixer)
            else: sim.step(mix["rpm"])
        dest=Path(output or Path("results")/"phase3"/controller.name/experiment); dest.mkdir(parents=True,exist_ok=True)
        logger.save_csv(dest/"data.csv")
        disturb_active=np.linalg.norm(condition.disturbance_force)>0 or np.linalg.norm(condition.disturbance_torque)>0
        disturbance_end=(condition.disturbance_start+condition.disturbance_duration) if disturb_active and condition.disturbance_duration>0 else None
        metrics=logger.metrics(step_time=step_args.get("step_time"),step_start=step_args.get("step_start"),step_goal=step_args.get("step_goal"),step_axis=step_args.get("step_axis"),disturbance_end=disturbance_end,attitude_step_time=attitude_step_time)
        logger.save_metrics(dest/"metrics.json",metrics)
        effective={"position_kp":control_config["position_gains"]["kp"].tolist(),
                   "position_kv":control_config["position_gains"]["kv"].tolist(),
                   "attitude_kp":control_config["attitude_gains"]["kp"].tolist(),
                   "attitude_kw":control_config["attitude_gains"]["kw"].tolist()}
        with (dest/"config.yaml").open("w") as f:
            yaml.safe_dump({"controller":controller.name,"experiment":experiment,"duration_s":duration,
                            "simulation_dt_s":1./c["physics_hz"],"controller_dt_s":dt,
                            "condition":condition.to_dict(),"effective_gains":effective,
                            "effective_motor_max_rpm":c["max_rpm"]*condition.motor_max_scale,"model":c},f,sort_keys=False)
        (dest/"metadata.json").write_text(json.dumps({"controller":controller.name,"experiment":experiment,
            "samples":len(logger.rows),"python":platform.python_version(),"platform":platform.platform(),
            "elapsed_wall_s":time.time()-started,"random_seed":condition.random_seed},indent=2)+"\n")
        make_plots(logger.rows,dest)
        print(f"[INFO] Saved {len(logger.rows)} samples to {dest} ({time.time()-started:.1f}s)")
        for key,value in metrics.items(): print(f"{key}: {value}")
        return metrics
    finally:
        if hasattr(sim,"close"): sim.close()
