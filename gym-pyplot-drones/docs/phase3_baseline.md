# Phase 3A — Frozen Phase 2 baseline

## Control equations

The translational loop is the Phase 2 diagonal PD law with gravity compensation:

\[
e_p=p_d-p,\quad e_v=v_d-v,\quad a_d=a_{ref}+K_p e_p+K_v e_v,
\]
\[
F_d=m(a_d+[0,0,g]^T),\quad T=\|F_d\|,\quad b_{3d}=F_d/\|F_d\|.
\]

The desired body axes are generated from \(b_{3d}\) and desired yaw. The baseline attitude law is:

\[
e_R=\tfrac12(R_d^TR-R^TR_d)^\vee,\quad e_\omega=\omega-\omega_d,
\quad \tau=-K_R e_R-K_\omega e_\omega.
\]

Thrust and body torque go through the shared motor mixer; motors are clipped to configured RPM limits. The baseline intentionally has no \(\omega\times J\omega\) compensation.

## Frozen parameters

The frozen simulator, model, gains, mixer layout assumption, and rates are in [`config/baseline_phase2.yaml`](../config/baseline_phase2.yaml). Controller implementation is in [`controller/baseline/controller.py`](../controller/baseline/controller.py). Phase 3 runs default to this snapshot; do not tune it during controller comparisons. Its motor order/signs remain an assumption until checked against the installed model asset.

## Common comparison contract

Controllers receive `ControlState(position, velocity, rotation, angular_velocity)` and `DesiredState(...)`, and return `ControlOutput(thrust, torque, ...)`. The output wrench is collective thrust in newtons and body torque in N·m; the simulator, mixer, logger, and experiment conditions are shared.

## First Phase 3 controller

`geometric_so3` uses the same position loop and SO(3) error as the baseline, and adds the Euler rigid-body gyroscopic compensation \(\omega\times J\omega\) to torque. This comparison isolates that term. Desired body rates are zero in the current references. Attitude-only roll, pitch, yaw, and combined-step tests can be run with the same interface.
