"""Run the report's navigation experiments and export figures, tables, and evidence.

From the repository root:
    .venv/bin/python report_assets/generate_navigation.py

Physical native IMU channels drive the rotating-frame ESKF. Closed-loop
pairs differ only in navigation. Seed and long-ascent campaigns assess initialization
and outages. Replay comparisons use one identity-feedback truth
trajectory and identical sensor samples, independently copied into each estimator.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.metadata
import json
import subprocess
import sys
from dataclasses import fields, replace
from pathlib import Path
from types import SimpleNamespace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from hydra import compose, initialize_config_dir
from navigation_robustness import run as run_robustness
from omegaconf import OmegaConf

from core.interfaces import NavigationHistory, StateHistory
from core.seeding import rng_from_seed, write_manifest
from gnc.build import build_nav
from plant.dynamics import ic_from_scenario, initial_state
from sim.run import run_configured
from sim.telemetry import navigation_errors, write_physics, write_telemetry

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("project2_landing", "flight_calibrated_landing")
LABELS = ("Project 2 / ideal actuators", "Calibrated / realistic actuators")
KEYS = ("r_ned", "v_ned", "attitude_rad", "w_body")
UNITS = (
    "Position error [m]",
    "Velocity error [m/s]",
    "Attitude error [deg]",
    "Angular-rate error [deg/s]",
)
SCALES = (1.0, 1.0, 180 / np.pi, 180 / np.pi)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config(scenario, navigation):
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        return compose(
            config_name="config",
            overrides=[
                f"scenario={scenario}",
                f"gnc/navigation={navigation}",
                "seed=0",
                "run.run_id=report_navigation_seed0",
            ],
        )


def magnitude(value):
    return np.linalg.norm(value, axis=1) if value.ndim == 2 else np.abs(value)


def statistics(errors):
    return {
        key: {
            "rmse": float(np.sqrt(np.mean(magnitude(value) ** 2))),
            "max": float(np.max(magnitude(value))),
        }
        for key, value in errors.items()
    }


def save_figure(fig, assets, name):
    fig.tight_layout()
    fig.savefig(assets / f"{name}.pdf", metadata={"CreationDate": None})
    fig.savefig(assets / f"{name}.png", dpi=160)
    plt.close(fig)


def finish_axes(axes):
    for ax in np.asarray(axes).flat:
        ax.grid(alpha=0.2)
        ax.set_xlabel("Elapsed time [s]")


def closed_loop(output):
    runs, configurations, metrics = {}, {}, {}
    for scenario in SCENARIOS:
        for navigation in ("identity", "synthetic_eskf"):
            cfg = config(scenario, navigation)
            folder = output / scenario / navigation
            folder.mkdir(parents=True, exist_ok=True)
            resolved = OmegaConf.to_yaml(cfg, resolve=True)
            (folder / "resolved_config.yaml").write_text(resolved)
            write_manifest(
                folder,
                int(cfg.seed),
                resolved,
                extra={"experiment": "navigation_report_closed_loop"},
            )
            _, result = run_configured(cfg)
            write_telemetry(result, folder)
            physics = write_physics(result, folder)
            if result.landing_class is None:
                raise ValueError(f"No ground contact in {scenario}/{navigation}")
            contact_time = float(result.t[-1])
            altitude = physics["altitude_m"]
            if altitude[-1] < 0 <= altitude[-2]:
                fraction = altitude[-2] / (altitude[-2] - altitude[-1])
                contact_time = float(result.t[-2] + fraction * (result.t[-1] - result.t[-2]))
            stats = statistics(navigation_errors(result.navigation))
            row = {
                "scenario": scenario,
                "navigation": navigation,
                "seed": int(cfg.seed),
                "actuator_model": str(cfg.plant.actuator.model),
                "contact_time_s": contact_time,
                "native_terminal_time_s": float(result.t[-1]),
                "contact_speed_mps": float(result.landing_speed),
                "contact_position_error_m": float(result.landing_position_error),
                "contact_tilt_deg": float(np.rad2deg(result.landing_tilt)),
                "landing_class": result.landing_class,
                "success": bool(result.success),
                "navigation_samples": len(result.navigation.truth.t),
                "navigation_last_time_s": float(result.navigation.truth.t[-1]),
                "navigation_errors": stats,
                "initial_height_m": float(-cfg.scenario.ic.r_ned[2]),
                "initial_down_speed_mps": float(cfg.scenario.ic.v_ned[2]),
                "initial_mass_kg": float(cfg.scenario.ic.m),
                "periods_s": {
                    name: float(cfg.scenario.horizon[name])
                    for name in ("nav_dt", "ctrl_dt", "guidance_dt", "log_dt")
                },
                "config_sha256": sha(folder / "resolved_config.yaml"),
                "navigation_sha256": sha(folder / "navigation.npz"),
            }
            (folder / "evaluation.json").write_text(json.dumps(row, indent=2) + "\n")
            key = f"{scenario}/{navigation}"
            runs[key], configurations[key], metrics[key] = result, cfg, row
            print(
                f"{key}: {result.landing_class}, position RMSE={stats['r_ned']['rmse']:.3f} m",
                flush=True,
            )
        # Establish that plant, IC, guidance, control, and timing are identical in each pair.
        pair = []
        for navigation in ("identity", "synthetic_eskf"):
            value = OmegaConf.to_container(configurations[f"{scenario}/{navigation}"], resolve=True)
            del value["gnc"]["navigation"]
            pair.append(value)
        if pair[0] != pair[1]:
            raise ValueError(f"Non-navigation configuration differs for {scenario}")
    return runs, configurations, metrics


def replay(runs, configurations, output):
    scenario = "flight_calibrated_landing"
    base = runs[f"{scenario}/identity"]
    truth = base.navigation.truth
    cfg = configurations[f"{scenario}/synthetic_eskf"]
    # Recover exactly the navigation seed stream used by the production loop.
    from plant.dynamics import build_plant

    bundle = build_plant(cfg)
    state0 = initial_state(ic_from_scenario(cfg.scenario, cfg.seed), bundle.params)
    rng = rng_from_seed(state0.key)
    sensor = build_nav(cfg)
    estimators = {name: build_nav(cfg) for name in ("imu_only", "eskf", "eskf_outage")}
    memories = {name: {} for name in estimators}
    logs = {name: [] for name in estimators}
    diagnostics = {name: {"bg": [], "ba": [], "P_diag": []} for name in estimators}
    checks = {
        name: {"min_covariance_eigenvalue": float("inf"), "max_quaternion_norm_error": 0.0}
        for name in estimators
    }
    observations = []
    outage = (4.0, 8.0)
    for i, t in enumerate(truth.t):
        state = SimpleNamespace(
            **{field.name: getattr(truth, field.name)[i] for field in fields(truth)}
        )
        sample = sensor.measure(
            state,
            int(rng.bit_generator.random_raw()),
            specific_force_body=base.navigation.inertial["specific_force_body"][i],
            inertial_rate_body=base.navigation.inertial["omega_inertial_body"][i],
        )
        if i == 0 and not sample.gnss_valid:
            raise ValueError("This comparison requires a common GNSS initialization sample")
        observations.append(copy.deepcopy(sample))
        for name, nav in estimators.items():
            # The current filter may modify its initialization arrays in place.
            # Independent copies preserve the common measurements and prevent cross-talk.
            obs = copy.deepcopy(sample)
            if name == "imu_only" and i > 0:
                obs = replace(obs, gnss_valid=False, radar_valid=False)
            elif name == "eskf_outage" and outage[0] <= t <= outage[1]:
                obs = replace(obs, gnss_valid=False)
            estimate, memories[name] = nav.estimate(obs, None, memories[name])
            logs[name].append(estimate)
            for key in diagnostics[name]:
                value = np.diag(memories[name]["P"]) if key == "P_diag" else memories[name][key]
                diagnostics[name][key].append(np.asarray(value).copy())
            eigenvalue = float(np.linalg.eigvalsh(memories[name]["P"])[0])
            checks[name]["min_covariance_eigenvalue"] = min(
                checks[name]["min_covariance_eigenvalue"], eigenvalue
            )
            checks[name]["max_quaternion_norm_error"] = max(
                checks[name]["max_quaternion_norm_error"],
                float(abs(np.linalg.norm(estimate.q_bn) - 1)),
            )
    histories, metrics = {}, {}
    arrays = {f"truth_{field.name}": getattr(truth, field.name) for field in fields(truth)}
    for key in (
        "t",
        "r_ned",
        "v_ned",
        "accel_body",
        "gyro_body",
        "gnss_valid",
        "radar_alt_m",
        "radar_valid",
    ):
        arrays[f"sensor_{key}"] = np.asarray([getattr(obs, key) for obs in observations])
    for name in estimators:
        estimate = StateHistory(
            **{
                field.name: np.asarray([getattr(row, field.name) for row in logs[name]])
                for field in fields(StateHistory)
            }
        )
        history = NavigationHistory(truth=truth, measurement=truth, estimate=estimate)
        errors = navigation_errors(history)
        histories[name] = (history, errors)
        for key, value in errors.items():
            arrays[f"{name}_error_{key}"] = value
        for field in fields(estimate):
            arrays[f"{name}_estimate_{field.name}"] = getattr(estimate, field.name)
        for key, value in diagnostics[name].items():
            arrays[f"{name}_{key}"] = np.asarray(value)
        if not all(np.all(np.isfinite(value)) for value in errors.values()):
            raise ValueError(f"Nonfinite replay estimate: {name}")
        if checks[name]["min_covariance_eigenvalue"] < -1e-10:
            raise ValueError(f"Indefinite covariance: {name}")
        metrics[name] = {
            "errors": statistics(errors),
            **checks[name],
            "final_gyro_bias_error_deg_s": np.rad2deg(
                arrays[f"{name}_bg"][-1] - sensor.bg
            ).tolist(),
            "final_accel_bias_error_mps2": (arrays[f"{name}_ba"][-1] - sensor.ba).tolist(),
        }
    arrays["true_gyro_bias"] = sensor.bg
    arrays["true_accel_bias"] = sensor.ba
    folder = output / "fixed_truth_replay"
    folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder / "navigation_replay.npz", **arrays)
    sample_counts = {
        "imu": len(observations),
        "gnss_valid": int(np.count_nonzero(arrays["sensor_gnss_valid"])),
        "radar_valid": int(np.count_nonzero(arrays["sensor_radar_valid"])),
        "gnss_valid_with_outage": int(
            np.count_nonzero(
                arrays["sensor_gnss_valid"] & ~((truth.t >= outage[0]) & (truth.t <= outage[1]))
            )
        ),
    }
    return (
        histories,
        arrays,
        {
            "cases": metrics,
            "sample_counts": sample_counts,
            "outage_s": outage,
            "last_time_s": float(truth.t[-1]),
            "samples_sha256": sha(folder / "navigation_replay.npz"),
        },
    )


def figures(runs, histories, arrays, assets):
    fig, axes = plt.subplots(4, 2, figsize=(9.3, 8.6), sharex="col")
    for column, (scenario, label) in enumerate(zip(SCENARIOS, LABELS, strict=True)):
        result = runs[f"{scenario}/synthetic_eskf"]
        errors = navigation_errors(result.navigation)
        for row, (key, unit, scale) in enumerate(zip(KEYS, UNITS, SCALES, strict=True)):
            ax = axes[row, column]
            ax.plot(result.navigation.truth.t, magnitude(errors[key]) * scale, label="ESKF")
            ax.axhline(0, color="gray", linestyle=":", label="IdentityNav error")
            ax.set_ylabel(unit)
            if row == 0:
                ax.set_title(label)
                ax.legend(fontsize=8)
    finish_axes(axes)
    save_figure(fig, assets, "navigation_closed_loop")

    fig, axes = plt.subplots(2, 2, figsize=(9.3, 6.0), sharex="col")
    for column, (scenario, label) in enumerate(zip(SCENARIOS, LABELS, strict=True)):
        for navigation, style in (("identity", "-"), ("synthetic_eskf", "--")):
            result = runs[f"{scenario}/{navigation}"]
            # Near-pad tangent height; contact KPIs use the plant's radial surface.
            axes[0, column].plot(result.t, -result.r_ned[:, 2], style, label=navigation)
            axes[1, column].plot(result.t, np.linalg.norm(result.v_ned, axis=1), style)
        axes[0, column].set_title(label)
        axes[0, column].set_ylabel("Height -D [m]")
        axes[1, column].set_ylabel("Speed [m/s]")
        axes[0, column].legend(fontsize=8)
    finish_axes(axes)
    save_figure(fig, assets, "navigation_landing")

    fig, axes = plt.subplots(2, 2, figsize=(9.3, 6.2))
    for ax, key, unit, scale in zip(axes.flat, KEYS, UNITS, SCALES, strict=True):
        for name, label in (
            ("imu_only", "IMU only after initialization"),
            ("eskf", "ESKF"),
            ("eskf_outage", "ESKF, GNSS outage"),
        ):
            history, errors = histories[name]
            ax.plot(history.truth.t, magnitude(errors[key]) * scale, label=label)
        ax.axvspan(4, 8, color="gray", alpha=0.12)
        ax.set_ylabel(unit)
    axes[0, 0].legend(fontsize=8)
    finish_axes(axes)
    save_figure(fig, assets, "navigation_replay")

    fig, axes = plt.subplots(3, 2, figsize=(9.3, 6.7), sharex=True)
    t = arrays["truth_t"]
    for j, axis in enumerate("xyz"):
        for column, key, scale, unit in ((0, "bg", 180 / np.pi, "deg/s"), (1, "ba", 1.0, "m/s²")):
            truth_key = "true_gyro_bias" if key == "bg" else "true_accel_bias"
            for name, label in (("eskf", "ESKF"), ("eskf_outage", "GNSS outage")):
                axes[j, column].plot(t, arrays[f"{name}_{key}"][:, j] * scale, label=label)
            axes[j, column].axhline(
                arrays[truth_key][j] * scale, color="black", linestyle=":", label="Injected bias"
            )
            axes[j, column].axvspan(4, 8, color="gray", alpha=0.12)
            axes[j, column].set_ylabel(f"{axis} bias [{unit}]")
    axes[0, 0].set_title("Gyroscope bias estimates")
    axes[0, 1].set_title("Accelerometer bias estimates")
    axes[0, 0].legend(fontsize=8)
    finish_axes(axes)
    save_figure(fig, assets, "navigation_biases")


def tables(closed, replay_metrics, assets):
    lines = [
        "% Generated by report_assets/generate_navigation.py; do not edit numbers.",
        r"\begin{tabularx}{\textwidth}{ll*{4}{>{\raggedleft\arraybackslash}X}}\toprule",
        r"Scenario & Navigation & Contact [s] & Speed [m/s] & Miss [m] & Tilt [deg]\\\midrule",
    ]
    for scenario, label in zip(SCENARIOS, ("Project 2", "Calibrated"), strict=True):
        for nav, nav_label in (("identity", "Identity"), ("synthetic_eskf", "ESKF")):
            row = closed[f"{scenario}/{nav}"]
            lines.append(
                f"{label} & {nav_label} & {row['contact_time_s']:.3f} & "
                f"{row['contact_speed_mps']:.3f} & "
                f"{row['contact_position_error_m']:.3f} & "
                f"{row['contact_tilt_deg']:.3f}" + r"\\"
            )
    lines += [r"\bottomrule\end{tabularx}"]
    (assets / "navigation_contact_table.tex").write_text("\n".join(lines) + "\n")
    lines = [
        r"\begin{tabularx}{\textwidth}{ll*{4}{>{\raggedleft\arraybackslash}X}}\toprule",
        r"Scenario & Statistic & Position [m] & Velocity [m/s] & Attitude [deg] & Rate [deg/s]"
        r"\\\midrule",
    ]
    for scenario, label in zip(SCENARIOS, ("Project 2", "Calibrated"), strict=True):
        stats = closed[f"{scenario}/synthetic_eskf"]["navigation_errors"]
        for stat in ("rmse", "max"):
            values = " & ".join(
                f"{stats[key][stat] * scale:.3f}" for key, scale in zip(KEYS, SCALES, strict=True)
            )
            lines.append(f"{label} & {stat.upper()} & {values}" + r"\\")
    lines += [r"\bottomrule\end{tabularx}"]
    (assets / "navigation_error_table.tex").write_text("\n".join(lines) + "\n")
    lines = [
        r"\begin{tabularx}{\textwidth}{l*{4}{>{\raggedleft\arraybackslash}X}}\toprule",
        r"Replay case & Position [m] & Velocity [m/s] & Attitude [deg] & Rate [deg/s]"
        r"\\\midrule",
    ]
    for name, label in (
        ("imu_only", "IMU only"),
        ("eskf", "ESKF"),
        ("eskf_outage", "ESKF + GNSS outage"),
    ):
        stats = replay_metrics["cases"][name]["errors"]
        values = " & ".join(
            f"{stats[key]['rmse'] * scale:.3f}" for key, scale in zip(KEYS, SCALES, strict=True)
        )
        lines.append(f"{label} & {values}" + r"\\")
    lines += [r"\bottomrule\end{tabularx}"]
    (assets / "navigation_replay_table.tex").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/report_navigation")
    parser.add_argument("--assets", type=Path, default=ROOT / "report_assets")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    args.assets.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.prop_cycle": plt.cycler(color=["#245780", "#c36526", "#45836a"]),
        }
    )
    runs, configurations, closed = closed_loop(args.output)
    histories, arrays, replay_metrics = replay(runs, configurations, args.output)
    figures(runs, histories, arrays, args.assets)
    tables(closed, replay_metrics, args.assets)
    robustness = run_robustness(config, statistics, args.output, args.assets)
    paths = [
        Path(__file__),
        ROOT / "report_assets/navigation_robustness.py",
        ROOT / "pyproject.toml",
    ]
    for pattern in ("src/**/*.py", "src/native/*.cpp", "src/native/*.hpp", "configs/**/*.yaml"):
        paths.extend(ROOT.glob(pattern))
    metrics = {
        "basis": "Native-force IMU / rotating ESKF; paired seed-0 runs, "
        "30 seed trials, and 15 ascent replays",
        "source_revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "python": sys.version,
        "versions": {
            name: importlib.metadata.version(name)
            for name in ("numpy", "scipy", "hydra-core", "matplotlib", "cvxpy", "clarabel")
        },
        "source_sha256": {str(path.relative_to(ROOT)): sha(path) for path in sorted(set(paths))},
        "closed_loop": closed,
        "fixed_truth_replay": replay_metrics,
        "robustness": robustness,
        "conventions": {
            "vector_errors": "Euclidean vector norm; sample RMSE and maximum on each nav grid",
            "attitude": "Shortest quaternion rotation angle, radians in JSON; degrees in tables",
            "contact": "Interpolated radial ground contact; nav grid stops before terminal contact",
            "replay": "One fixed calibrated identity trajectory; identical samples and initial "
            "posterior; no estimator feeds back into the replay plant",
            "imu_only": "GNSS/radar initialization retained, all subsequent aiding disabled",
            "outage": "Same samples, GNSS updates disabled at t=4 through 8 s; radar retained",
            "scope": "Ten seeds per landing condition; five seeds per 150-second ascent condition; "
            "pointwise NEES and NIS diagnostics, not operational qualification",
        },
    }
    text = json.dumps(metrics, indent=2, allow_nan=False) + "\n"
    (args.assets / "navigation_metrics.json").write_text(text)
    (args.output / "summary.json").write_text(text)
    print(json.dumps({"closed_loop": closed, "replay": replay_metrics}, indent=2))


if __name__ == "__main__":
    main()
