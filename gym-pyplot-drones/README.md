# Quadrotor controller: baseline, validation, and comparison

This project implements trajectory tracking, position control, desired attitude/thrust generation, SO(3) attitude control, motor allocation, PyBullet simulation, logging, plots, and tracking metrics. VIO is out of scope.

Workspace check: Python 3.13.12, Git 2.34.1, and `gym-pybullet-drones` is not installed. The official repository recommends Python 3.12, cloning its repository, and installing it editable; PyBullet needs a local build toolchain above Python 3.10. On Ubuntu, install `build-essential`, create a Python 3.12 environment, install `requirements.txt`, then clone the official simulator and run `pip install -e .` from its root. First run an untouched simulator example to verify GUI/physics, then use `python main.py --experiment hover --gui`. `--dry-run` uses a simplified plant and does not validate flight.

Conventions: right-handed ENU world (+z up); body +z is the thrust axis; thrust is `+T b3`; `R` maps body vectors to world; angular velocity and torque are body-frame x/y/z. Error is `0.5 vee(Rd.T @ R - R.T @ Rd)` and torque is `-KR eR - Kω eω`. Euler angles are for plots only.

`config/drone.yaml` contains CF2X starting parameters and an assumed X-frame rotor order/sign. Confirm these against the installed package's model assets and rotor setup before relying on the mixer. Each run writes CSV, metrics, position/error/velocity/attitude/attitude-error/angular-rate/control/motor plots, and an actual-vs-desired 3D trajectory under `results/<experiment>/`.

Run `python main.py --experiment {hover|position_step|square|circle|trajectory_3d|trajectory|demo}`. Options include `--gui`, `--duration`, `--output`, `--config`, and Phase 2 conditions such as `--initial-position`, `--initial-rpy-deg`, `--gain-scale`, state noise, and timed external force/torque. Every experiment writes `data.csv`, `config.yaml`, `metrics.json`, and plots. Metrics include per-axis and 3D RMSE, maximum tracking error, attitude error angle, step rise/settling/overshoot, control effort, saturation count/percentage, and disturbance recovery time where applicable.

Run the complete repeatable Phase 2 matrix headlessly with `python -m experiments.run_suite`; use `--quick` for only hover and step, and `--dry-run` for pipeline checks only. The suite includes square/circle/3D trajectories, initial position/attitude errors, force/torque disturbances, state noise, saturation stress, and a 0.5/1/1.5 gain scale sweep. The baseline controller equations are unchanged; all conditions are applied around the controller. `flightlog/` avoids shadowing Python's standard-library `logging` module.

## Phase 3: controller comparison

Phase 2's controller and parameters are frozen in `controller/baseline/controller.py` and `config/baseline_phase2.yaml`. The shared API accepts `ControlState` and `DesiredState` and returns a collective-thrust/body-torque `ControlOutput`. `geometric_so3` uses the same position loop and SO(3) feedback plus \(\omega\times J\omega\) compensation. Isolated roll, pitch, yaw, and combined attitude steps are available. The equations and conventions are recorded in `docs/phase3_baseline.md`.

Run one controller or compare baseline with geometric using exactly the same experiment cases and seeds:

```bash
python main.py --controller geometric --experiment hover
python main.py --controller geometric --experiment attitude_roll
python -m experiments.compare_controllers --quick
python -m experiments.compare_controllers
```

The comparison writes per-run reproducibility artifacts below `results/phase3/<controller>/<case>/`, plus `comparison.csv` and `comparison.png`. LQR and MPC are intentionally not implemented yet; validate this pair with the same PyBullet matrix before adding another controller.
