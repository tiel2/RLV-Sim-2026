"""Verify spherical rotating-Earth translation, attitude and phase histories."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from sim.run import run_configured
from sim.telemetry import physics_arrays
from validation.rotating_earth import orbit_case, torque_free_case

ROOT = Path(__file__).resolve().parents[1]
VACUUM = [
    "environment/earth=rotating",
    "environment.earth.latitude_deg=28.6",
    "environment/atmosphere=vacuum",
    "plant/thrust=vacuum",
    "plant/actuator=ideal",
    "plant/slosh=none",
    "environment/wind=none",
    "plant.rcs.mode=averaged",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/verification/rotating_earth")
    parser.add_argument("--skip-phases", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reports, histories = {}, {}
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        for name, e, heading, latitude in (
            ("circular_equatorial", 0.0, 0.0, 0.0),
            ("circular_inclined", 0.0, 63.0, 28.6),
            ("elliptic_southern", 0.2, -41.0, -37.0),
            ("elliptic_near_pole", 0.2, 24.0, 89.0),
        ):
            cfg = compose(config_name="config", overrides=VACUUM)
            cfg.environment.earth.latitude_deg = latitude
            cfg.plant_dt = 0.5
            (args.output / f"{name}_native_config.yaml").write_text(
                OmegaConf.to_yaml(cfg, resolve=True)
            )
            coarse, _ = orbit_case(cfg, eccentricity=e, heading_deg=heading, dt=1.0)
            report, history = orbit_case(cfg, eccentricity=e, heading_deg=heading, dt=0.5)
            report["coarse_dt_1s"] = coarse
            reports[name], histories[name] = report, history
            np.savez_compressed(args.output / f"{name}.npz", **history)
            print(f"{name}: {report['status']} ({history['t'][-1]:.1f} s)", flush=True)
        cfg = compose(config_name="config", overrides=VACUUM)
        cfg.plant.vehicle.mass.I_dry[2] *= 1.15
        cfg.plant.vehicle.mass.inertia_products = [100.0, -200.0, 300.0]
        cfg.plant_dt = 0.01
        (args.output / "torque_free_native_config.yaml").write_text(
            OmegaConf.to_yaml(cfg, resolve=True)
        )
        reports["torque_free_asymmetric"] = torque_free_case(cfg)
        print(f"torque-free: {reports['torque_free_asymmetric']['status']}", flush=True)
        if not args.skip_phases:
            phases = {}
            for name in ("ascent", "entry_burn", "aero_descent", "powered_descent", "landing"):
                cfg = compose(config_name="config", overrides=[f"scenario=project2_{name}"])
                (args.output / f"project2_{name}.yaml").write_text(
                    OmegaConf.to_yaml(cfg, resolve=True)
                )
                _, rotating = run_configured(cfg, record_telemetry=False)
                # Same curved geometry and initial conditions: isolate rotation from curvature.
                cfg.environment.earth.rotation_rate_rad_s = 0.0
                _, nonrotating = run_configured(cfg, record_telemetry=False)
                rp, np_ = physics_arrays(rotating), physics_arrays(nonrotating)
                same_times = np.array_equal(rotating.t, nonrotating.t)
                common, ri, ni = np.intersect1d(
                    np.round(rotating.t, 9), np.round(nonrotating.t, 9), return_indices=True
                )
                phases[name] = dict(
                    comparison="rotating sphere vs zero-spin sphere; identical IC/"
                    + ("controller configuration" if name == "landing" else "inputs"),
                    same_sample_times=same_times,
                    compared_until_s=float(common[-1]),
                    final_time_s=float(rotating.t[-1]),
                    terminal_time_change_s=float(rotating.t[-1] - nonrotating.t[-1]),
                    landing_class=rotating.landing_class,
                    landing_success=bool(rotating.success) if name == "landing" else None,
                    final_altitude_m=float(rp["altitude_m"][-1]),
                    final_speed_mps=float(rp["speed_mps"][-1]),
                    final_main_propellant_kg=float(rp["main_propellant"][-1]),
                    final_altitude_change_m=float(rp["altitude_m"][-1] - np_["altitude_m"][-1]),
                    final_speed_change_mps=float(rp["speed_mps"][-1] - np_["speed_mps"][-1]),
                    maximum_position_difference_m=float(
                        np.max(np.linalg.norm(rotating.r_ned[ri] - nonrotating.r_ned[ni], axis=1))
                    ),
                    maximum_rate_difference_rad_s=float(
                        np.max(np.linalg.norm(rotating.w_body[ri] - nonrotating.w_body[ni], axis=1))
                    ),
                )
                np.savez_compressed(
                    args.output / f"phase_{name}.npz",
                    t=rotating.t,
                    zero_spin_t=nonrotating.t,
                    rotating_position=rotating.r_ned,
                    zero_spin_position=nonrotating.r_ned,
                )
                print(f"phase {name}: rotation sensitivity recorded", flush=True)
            reports["phase_rotation_sensitivity"] = phases
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for name, history in histories.items():
        axes[0].semilogy(history["t"], np.maximum(history["position_error_m"], 1e-12), label=name)
        axes[1].semilogy(history["t"], np.maximum(history["velocity_error_mps"], 1e-14), label=name)
    for ax, unit in zip(axes, ("Position error [m]", "Velocity error [m/s]"), strict=True):
        ax.set(xlabel="Time [s]", ylabel=unit)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(args.output / "kepler_errors.png", dpi=160)
    plt.close(fig)
    passed = all(
        row["status"] == "passed" and row.get("coarse_dt_1s", row)["status"] == "passed"
        for key, row in reports.items()
        if key != "phase_rotation_sensitivity"
    )
    sources = [
        "src/native/plant.cpp",
        "src/plant/dynamics.py",
        "src/plant/earth.py",
        "src/validation/rotating_earth.py",
        "src/validation/ascent.py",
        "scripts/check_rotating_earth.py",
    ]
    sources.extend(str(p.relative_to(ROOT)) for p in sorted((ROOT / "configs").rglob("*.yaml")))
    sources.extend(str(p.relative_to(ROOT)) for p in sorted((ROOT / "src/gnc").rglob("*.py")))
    summary = dict(
        status="passed" if passed else "failed",
        checks=reports,
        scope="spherical central gravity and uniformly rotating Earth; not flight validation",
        source_sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in sources},
    )
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    raise SystemExit(0 if passed else 2)


if __name__ == "__main__":
    main()
