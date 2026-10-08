# Phase 1: controlled quadrotor and flight data

This project implements trajectory tracking, position control, desired attitude/thrust generation, SO(3) attitude control, motor allocation, PyBullet simulation, logging, plots, and tracking metrics. VIO is out of scope.

Workspace check: Python 3.13.12, Git 2.34.1, and `gym-pybullet-drones` is not installed. The official repository recommends Python 3.12, cloning its repository, and installing it editable; PyBullet needs a local build toolchain above Python 3.10. On Ubuntu, install `build-essential`, create a Python 3.12 environment, install `requirements.txt`, then clone the official simulator and run `pip install -e .` from its root. First run an untouched simulator example to verify GUI/physics, then use `python main.py --experiment hover --gui`. `--dry-run` uses a simplified plant and does not validate flight.

Conventions: right-handed ENU world (+z up); body +z is the thrust axis; thrust is `+T b3`; `R` maps body vectors to world; angular velocity and torque are body-frame x/y/z. Error is `0.5 vee(Rd.T @ R - R.T @ Rd)` and torque is `-KR eR - Kω eω`. Euler angles are for plots only.

`config/drone.yaml` contains CF2X starting parameters and an assumed X-frame rotor order/sign. Confirm these against the installed package's model assets and rotor setup before relying on the mixer. Each run writes CSV, metrics, position/error/velocity/attitude/attitude-error/angular-rate/control/motor plots, and an actual-vs-desired 3D trajectory under `results/<experiment>/`.

Run `python main.py --experiment {hover|position_step|trajectory|demo}`. Options include `--gui`, `--duration`, `--output`, `--config`, and `--dry-run`. Metrics are descriptive: position/attitude RMSE, final-window altitude error, control effort integrals, and motor saturation samples. `flightlog/` is used instead of `logging/` to avoid shadowing Python's standard library.
