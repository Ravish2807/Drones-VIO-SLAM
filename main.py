import argparse
import numpy as np
from experiments.conditions import RunConditions
from experiments.run_experiment import run


def main():
    p=argparse.ArgumentParser(description="Phase 1/2/3 cascaded quadrotor experiment runner")
    p.add_argument("--experiment",choices=["hover","position_step","square","circle","trajectory_3d","trajectory","demo","attitude_roll","attitude_pitch","attitude_yaw","attitude_combined"],default="hover")
    p.add_argument("--controller",choices=["baseline","geometric"],default="baseline")
    p.add_argument("--duration",type=float); p.add_argument("--gui",action="store_true")
    p.add_argument("--dry-run",action="store_true",help="Simplified plant; not PyBullet validation")
    p.add_argument("--output"); p.add_argument("--config",default="config/baseline_phase2.yaml")
    p.add_argument("--initial-position",nargs=3,type=float)
    p.add_argument("--initial-rpy-deg",nargs=3,type=float,default=[0.,0.,0.])
    p.add_argument("--gain-scale",type=float,default=1.)
    p.add_argument("--position-kp-scale",type=float,default=1.)
    p.add_argument("--position-kv-scale",type=float,default=1.)
    p.add_argument("--attitude-kp-scale",type=float,default=1.)
    p.add_argument("--attitude-kw-scale",type=float,default=1.)
    p.add_argument("--motor-max-scale",type=float,default=1.)
    p.add_argument("--position-noise",type=float,default=0.)
    p.add_argument("--velocity-noise",type=float,default=0.)
    p.add_argument("--attitude-noise-deg",type=float,default=0.)
    p.add_argument("--angular-rate-noise",type=float,default=0.)
    p.add_argument("--disturbance-force",nargs=3,type=float,default=[0.,0.,0.])
    p.add_argument("--disturbance-torque",nargs=3,type=float,default=[0.,0.,0.])
    p.add_argument("--disturbance-start",type=float,default=0.)
    p.add_argument("--disturbance-duration",type=float,default=0.)
    p.add_argument("--seed",type=int,default=1)
    a=p.parse_args()
    c=RunConditions(initial_position=a.initial_position,initial_rpy_deg=a.initial_rpy_deg,gain_scale=a.gain_scale,
        gain_scales={"position_kp":a.position_kp_scale,"position_kv":a.position_kv_scale,"attitude_kp":a.attitude_kp_scale,"attitude_kw":a.attitude_kw_scale},
        motor_max_scale=a.motor_max_scale,
        noise_std={"position":a.position_noise,"velocity":a.velocity_noise,"attitude":np.deg2rad(a.attitude_noise_deg),"omega":a.angular_rate_noise},
        disturbance_force=a.disturbance_force,disturbance_torque=a.disturbance_torque,
        disturbance_start=a.disturbance_start,disturbance_duration=a.disturbance_duration,random_seed=a.seed)
    run(a.experiment,a.duration,a.gui,a.output,a.dry_run,a.config,c,a.controller)


if __name__=="__main__": main()
