"""Run the same Phase 2/attitude matrix with baseline and geometric controllers."""
import argparse
from pathlib import Path
from experiments.conditions import RunConditions
from experiments.run_suite import cases
from experiments.run_experiment import run
from evaluation.comparison import save_comparison


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",default="results/phase3")
    p.add_argument("--config",default="config/baseline_phase2.yaml")
    p.add_argument("--quick",action="store_true",help="Compare hover, step, and isolated attitude steps")
    p.add_argument("--dry-run",action="store_true",help="Pipeline comparison only; not PyBullet evidence")
    a=p.parse_args(); scenarios=cases(quick=a.quick)
    scenarios += [(n,n,RunConditions(initial_position=[0.,0.,1.]),6.) for n in
                  ("attitude_roll","attitude_pitch","attitude_yaw","attitude_combined")]
    rows=[]
    for name,experiment,condition,duration in scenarios:
        for controller in ("baseline","geometric"):
            print(f"\n=== {controller}: {name} ===")
            metrics=run(experiment,duration,False,Path(a.output)/controller/name,a.dry_run,a.config,condition,controller)
            rows.append({"case":name,"controller":controller,"experiment":experiment,"duration_s":duration,**metrics})
    save_comparison(rows,a.output)
    print(f"\nController comparison saved to {Path(a.output)/'comparison.csv'}")


if __name__=="__main__": main()
