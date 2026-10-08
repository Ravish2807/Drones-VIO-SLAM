import argparse
from experiments.run_experiment import run


def main():
    p=argparse.ArgumentParser(description="Phase 1 cascaded quadrotor controller")
    p.add_argument("--experiment",choices=["hover","position_step","trajectory","demo"],default="hover")
    p.add_argument("--duration",type=float); p.add_argument("--gui",action="store_true")
    p.add_argument("--dry-run",action="store_true",help="Simplified plant; not PyBullet validation")
    p.add_argument("--output"); p.add_argument("--config",default="config/drone.yaml")
    a=p.parse_args(); run(a.experiment,a.duration,a.gui,a.output,a.dry_run,a.config)


if __name__=="__main__": main()
