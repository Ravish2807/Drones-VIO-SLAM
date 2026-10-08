from pathlib import Path
import time
import yaml

from controller.position_controller import PositionController
from controller.attitude_controller import AttitudeController
from controller.motor_mixer import MotorMixer
from flightlog.data_logger import DataLogger
from simulation.simulator import PyBulletSimulator
from trajectory.trajectory_generator import hold, smooth_waypoints, phase1_demo
from visualization.plots import make_plots


def run(
    experiment="hover",
    duration=None,
    gui=False,
    output=None,
    dry_run=False,
    config_path="config/drone.yaml"
):
    # ---------------------------------------------------------
    # Load configuration
    # ---------------------------------------------------------
    with Path(config_path).open() as f:
        c = yaml.safe_load(f)

    dt = 1.0 / c["control_hz"]

    # ---------------------------------------------------------
    # Controllers
    # ---------------------------------------------------------
    pos = PositionController(
        c["position_gains"]["kp"],
        c["position_gains"]["kv"],
        c["mass"],
        c["gravity"],
        c["max_tilt_deg"]
    )

    att = AttitudeController(
        c["attitude_gains"]["kp"],
        c["attitude_gains"]["kw"]
    )

    mixer = MotorMixer(
        c["mass"],
        c["gravity"],
        c["thrust_coefficient"],
        c["moment_coefficient"],
        c["rotor_positions"],
        c["rotor_spin_directions"],
        c["min_rpm"],
        c["max_rpm"]
    )

    # ---------------------------------------------------------
    # Trajectory / reference
    # ---------------------------------------------------------
    if experiment == "hover":
        # Hover at the requested final position
        ref = hold([2, 1, 2])

    elif experiment == "position_step":
        ref = smooth_waypoints(
            [
                [0, 0, 0.08],
                [0, 0, 1],
                [1, 0, 1]
            ],
            3.0
        )

        if duration is None:
            duration = 12.0

    elif experiment == "trajectory":
        ref = smooth_waypoints(
            [
                [0, 0, 0.08],
                [0, 0, 1],
                [0.5, 0.5, 1],
                [0, 0, 1]
            ],
            3.0
        )

        if duration is None:
            duration = 12.0

    elif experiment == "demo":
        ref = phase1_demo()

        if duration is None:
            duration = 15.0

    else:
        raise ValueError(f"Unknown experiment: {experiment}")

    # ---------------------------------------------------------
    # Simulation
    # ---------------------------------------------------------
    if dry_run:
        from simulation.dry_run import DryRunPlant

        sim = DryRunPlant(
            c["inertia"],
            c["gravity"],
            dt
        )
    else:
        sim = PyBulletSimulator(
            c["physics_hz"],
            c["control_hz"],
            gui
        )

    # ---------------------------------------------------------
    # Logging
    # ---------------------------------------------------------
    logger = DataLogger()
    started = time.time()

    # duration == None means:
    # RUN FOREVER until Ctrl+C
    if duration is None:
        count = None
        print("[INFO] Running simulation indefinitely.")
        print("[INFO] Press Ctrl+C to stop the simulation.")

    else:
        count = int(duration * c["control_hz"])
        print(f"[INFO] Running simulation for {duration:.2f} seconds.")

    try:

        k = 0

        while count is None or k < count:

            # -------------------------------------------------
            # Current simulation time
            # -------------------------------------------------
            t = k * dt

            # -------------------------------------------------
            # Read current drone state
            # -------------------------------------------------
            state = sim.state()

            # -------------------------------------------------
            # Get desired trajectory state
            # -------------------------------------------------
            desired = ref(t)

            # -------------------------------------------------
            # Position controller
            # -------------------------------------------------
            pc = pos.compute(
                state["position"],
                state["velocity"],
                desired["position"],
                desired["velocity"],
                desired["acceleration"],
                desired["yaw"]
            )

            # -------------------------------------------------
            # Attitude controller
            # -------------------------------------------------
            ac = att.compute(
                state["R"],
                state["omega"],
                pc["R_des"]
            )

            # -------------------------------------------------
            # Motor mixer
            # -------------------------------------------------
            mix = mixer.mix(
                pc["thrust"],
                ac["torque"]
            )

            # -------------------------------------------------
            # Log data
            # -------------------------------------------------
            logger.record(
                t,
                state,
                desired,
                pc,
                ac,
                mix
            )

            # -------------------------------------------------
            # Step simulation
            # -------------------------------------------------
            if dry_run:
                sim.step(
                    mix["rpm"],
                    mixer
                )
            else:
                sim.step(
                    mix["rpm"]
                )

            k += 1

    except KeyboardInterrupt:
        print("\n")
        print("[INFO] Ctrl+C detected.")
        print("[INFO] Stopping simulation...")

    finally:

        # -----------------------------------------------------
        # Save results
        # -----------------------------------------------------
        if len(logger.rows) > 0:

            dest = Path(
                output or Path("results") / experiment
            )

            dest.mkdir(
                parents=True,
                exist_ok=True
            )

            logger.save_csv(
                dest / "flight_data.csv"
            )

            metrics = logger.metrics()

            logger.save_metrics(
                dest / "phase1_metrics.txt",
                metrics
            )

            make_plots(
                logger.rows,
                dest
            )

            elapsed = time.time() - started

            print(
                f"[INFO] Saved {len(logger.rows)} samples "
                f"to {dest} ({elapsed:.1f}s wall time)"
            )

            for key, value in metrics.items():
                print(f"{key}: {value}")

        # -----------------------------------------------------
        # Close simulator
        # -----------------------------------------------------
        if hasattr(sim, "close"):
            sim.close()

        print("[INFO] Simulation closed.")