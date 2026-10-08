"""Run a repeatable Phase 2 validation matrix and aggregate its metrics."""
import argparse
import csv
from pathlib import Path

from experiments.conditions import RunConditions
from experiments.run_experiment import run


def cases(quick=False):
    rows=[("hover","hover",RunConditions(),10.),
          ("position_step","position_step",RunConditions(),12.)]
    if quick: return rows
    rows += [(name,name,RunConditions(),duration) for name,duration in
             (("square",16.),("circle",20.),("trajectory_3d",20.))]
    rows += [
        ("initial_position_error","hover",RunConditions(initial_position=[1.,1.,.15]),10.),
        ("initial_attitude_error","hover",RunConditions(initial_position=[0.,0.,1.],initial_rpy_deg=[10.,0.,0.]),10.),
        ("disturbance_small","hover",RunConditions(disturbance_force=[.03,0.,0.],disturbance_start=3.,disturbance_duration=1.),10.),
        ("disturbance_large","hover",RunConditions(disturbance_force=[.08,0.,0.],disturbance_start=3.,disturbance_duration=1.),10.),
        ("disturbance_vertical","hover",RunConditions(disturbance_force=[0.,0.,.03],disturbance_start=3.,disturbance_duration=1.),10.),
        ("disturbance_torque","hover",RunConditions(disturbance_torque=[0.,.00005,0.],disturbance_start=3.,disturbance_duration=1.),10.),
        ("disturbance_impulse","hover",RunConditions(disturbance_force=[.03,0.,0.],disturbance_start=3.,disturbance_duration=.1),10.),
        ("disturbance_continuous","hover",RunConditions(disturbance_force=[.01,0.,0.],disturbance_start=2.,disturbance_duration=8.),10.),
        ("measurement_noise","circle",RunConditions(noise_std={"position":.02,"velocity":.05,"attitude":.0174533,"omega":.02}),20.),
        ("motor_saturation","position_step",RunConditions(motor_max_scale=.75),12.)]
    for key,label in (("position_kp","Kp"),("position_kv","Kv"),("attitude_kp","KR"),("attitude_kw","Kω")):
        for scale in (.5,1.,1.5):
            rows.append((f"{label}_{scale:g}x","position_step",RunConditions(gain_scales={key:scale}),12.))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",default="results/phase2")
    parser.add_argument("--config",default="config/baseline_phase2.yaml")
    parser.add_argument("--controller",choices=["baseline","geometric"],default="baseline")
    parser.add_argument("--quick",action="store_true",help="Run only hover and position-step baselines")
    parser.add_argument("--dry-run",action="store_true",help="Use the simplified plant; not PyBullet validation")
    args=parser.parse_args(); root=Path(args.output); root.mkdir(parents=True,exist_ok=True)
    summary=[]
    for name,experiment,condition,duration in cases(args.quick):
        print(f"\n=== Phase 2 case: {name} ===")
        metrics=run(experiment,duration,False,root/name,args.dry_run,args.config,condition,args.controller)
        summary.append({"case":name,"experiment":experiment,"duration_s":duration,**metrics})
    fields=list(dict.fromkeys(k for row in summary for k in row))
    with (root/"summary.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(summary)
    print(f"\nPhase 2 summary: {root/'summary.csv'}")


if __name__=="__main__": main()
