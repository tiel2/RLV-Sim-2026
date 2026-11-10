"""Independent navigation stress campaign: closed-loop seeds and long native ascent replay."""

from __future__ import annotations

import copy
import json
from dataclasses import replace
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

from gnc.build import build_nav
from gnc.navigation.synthetic_eskf import rot
from sim.run import run_configured
from sim.telemetry import navigation_errors, physics_arrays


def error_state(state, estimate, memory, sensor):
    attitude = Rotation.from_matrix(rot(state.q_bn) @ rot(estimate.q_bn).T).as_rotvec()
    return np.r_[
        state.r_ned - estimate.r_ned,
        state.v_ned - estimate.v_ned,
        attitude,
        sensor.bg - memory["bg"],
        sensor.ba - memory["ba"],
    ]


def run(config, statistics, output, assets):
    folder = output / "robustness"
    folder.mkdir(parents=True, exist_ok=True)
    closed = []
    for scenario, case in (
        ("project2_landing", "nominal"),
        ("flight_calibrated_landing", "nominal"),
        ("project2_landing", "startup_outage"),
    ):
        for seed in range(10):
            cfg = config(scenario, "synthetic_eskf")
            cfg.seed = seed
            if case == "startup_outage":
                cfg.gnc.navigation.gnss_outage_start = 0.0
                cfg.gnc.navigation.gnss_outage_end = 3.0
                cfg.gnc.navigation.prior_position_offset = [10.0, -8.0, 5.0]
                cfg.gnc.navigation.prior_velocity_offset = [0.5, -0.4, 0.3]
            (folder / f"{scenario}_{case}_{seed}.yaml").write_text(
                OmegaConf.to_yaml(cfg, resolve=True)
            )
            _, result = run_configured(cfg)
            row = dict(
                scenario=scenario,
                case=case,
                seed=seed,
                success=bool(result.success),
                landing_class=result.landing_class,
                speed_mps=result.landing_speed,
                miss_m=result.landing_position_error,
                tilt_deg=float(np.rad2deg(result.landing_tilt)),
                errors=statistics(navigation_errors(result.navigation)),
            )
            closed.append(row)
        print(f"Navigation seed campaign: {scenario}/{case} completed", flush=True)
    groups = []
    for scenario, case in dict.fromkeys((r["scenario"], r["case"]) for r in closed):
        rows = [r for r in closed if (r["scenario"], r["case"]) == (scenario, case)]
        groups.append(
            dict(
                scenario=scenario,
                case=case,
                n=len(rows),
                successes=sum(r["success"] for r in rows),
                speed_max_mps=max(r["speed_mps"] for r in rows),
                miss_max_m=max(r["miss_m"] for r in rows),
                position_rmse_mean_m=float(np.mean([r["errors"]["r_ned"]["rmse"] for r in rows])),
            )
        )

    # Same native 6-DOF ascent as the physics campaign, sampled at the IMU period.
    cfg = config("project2_ascent", "synthetic_eskf")
    cfg.scenario.horizon.log_dt = 0.01
    (folder / "ascent_truth_config.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    _, trajectory = run_configured(cfg)
    physics = physics_arrays(trajectory)
    force = (physics["F_body_total"] + physics["F_pad_body"]) / trajectory.m[:, None]
    gyro = physics["omega_inertial_body"]
    np.savez_compressed(
        folder / "ascent_truth.npz",
        t=trajectory.t,
        r_ned=trajectory.r_ned,
        v_ned=trajectory.v_ned,
        q_bn=trajectory.q_bn,
        w_body=trajectory.w_body,
        mass=trajectory.m,
        specific_force_body=force,
        omega_inertial_body=gyro,
    )
    cases = ("nominal", "startup_5s", "outage_20s")
    records, ensembles = [], {name: [] for name in cases}
    traces = {}
    for seed in range(5):
        sensor = build_nav(cfg)
        estimators = {name: build_nav(cfg) for name in cases}
        estimators["startup_5s"].initial_position += [10.0, -8.0, 5.0]
        estimators["startup_5s"].initial_velocity += [0.5, -0.4, 0.3]
        memories = {name: {} for name in cases}
        logs = {
            name: {
                k: []
                for k in (
                    "position",
                    "velocity",
                    "attitude",
                    "rate",
                    "nees",
                    "bg",
                    "ba",
                    "P_diag",
                    "nis_gnss",
                    "nis_radar",
                )
            }
            for name in cases
        }
        sensor_log = []
        for i, t in enumerate(trajectory.t):
            state = SimpleNamespace(
                t=float(t),
                r_ned=trajectory.r_ned[i],
                v_ned=trajectory.v_ned[i],
                q_bn=trajectory.q_bn[i],
                w_body=trajectory.w_body[i],
                m=trajectory.m[i],
            )
            obs = sensor.measure(
                state, [seed, 20261110], specific_force_body=force[i], inertial_rate_body=gyro[i]
            )
            sensor_log.append(
                np.r_[
                    obs.accel_body,
                    obs.gyro_body,
                    obs.r_ned,
                    obs.v_ned,
                    obs.radar_alt_m,
                    obs.gnss_valid,
                    obs.radar_valid,
                    obs.q_bn,
                    obs.alignment_valid,
                ]
            )
            for name, nav in estimators.items():
                sample = copy.deepcopy(obs)
                if (name == "startup_5s" and t <= 5.0 + 1e-8) or (
                    name == "outage_20s" and 60.0 - 1e-8 <= t <= 80.0 + 1e-8
                ):
                    sample = replace(sample, gnss_valid=False)
                estimate, mem = nav.estimate(sample, None, memories[name])
                memories[name] = mem
                e = error_state(state, estimate, mem, sensor)
                values = dict(
                    position=np.linalg.norm(e[:3]),
                    velocity=np.linalg.norm(e[3:6]),
                    attitude=np.linalg.norm(e[6:9]),
                    rate=np.linalg.norm(estimate.w_body - state.w_body),
                    nees=float(e @ np.linalg.solve(mem["P"], e)),
                    bg=mem["bg"],
                    ba=mem["ba"],
                    P_diag=np.diag(mem["P"]),
                )
                if not np.all(np.isfinite(e)):
                    raise ValueError(f"Nonfinite ascent navigation: {name}/{seed}/{t}")
                for key, value in values.items():
                    logs[name][key].append(np.array(value, copy=True))
                for innovation in mem["innovations"]:
                    logs[name][f"nis_{innovation['channel']}"].append([t, innovation["nis"]])
        samples = np.asarray(sensor_log)
        arrays = {
            "sensor_accel_body": samples[:, :3],
            "sensor_gyro_body": samples[:, 3:6],
            "sensor_r_ned": samples[:, 6:9],
            "sensor_v_ned": samples[:, 9:12],
            "sensor_radar_alt_m": samples[:, 12],
            "sensor_gnss_valid": samples[:, 13],
            "sensor_radar_valid": samples[:, 14],
            "sensor_alignment_q": samples[:, 15:19],
            "sensor_alignment_valid": samples[:, 19],
        }
        for name in cases:
            values = {key: np.asarray(value) for key, value in logs[name].items()}
            arrays.update({f"{name}_{key}": value for key, value in values.items()})
            ensembles[name].append(values["nees"])
            if seed == 0:
                traces[name] = values
            record = dict(
                case=name,
                seed=seed,
                first_gnss_time=memories[name]["first_gnss_time"],
                initialization_source=memories[name]["initialization_source"],
                final_position_m=float(values["position"][-1]),
                final_attitude_deg=float(np.rad2deg(values["attitude"][-1])),
                initial_attitude_deg=float(np.rad2deg(values["attitude"][0])),
                minimum_covariance_diagonal=float(values["P_diag"].min()),
            )
            for key in ("position", "velocity", "attitude", "rate"):
                record[f"{key}_rmse"] = float(np.sqrt(np.mean(values[key] ** 2)))
                record[f"{key}_max"] = float(values[key].max())
            for channel in ("gnss", "radar"):
                nis = values[f"nis_{channel}"]
                record[f"nis_{channel}_mean"] = float(np.mean(nis[:, 1])) if len(nis) else None
            records.append(record)
        np.savez_compressed(folder / f"ascent_seed{seed}.npz", **arrays)
        print(f"150 s ascent navigation replay: seed {seed} completed", flush=True)
    n = 5
    limits = (chi2.ppf(0.025, 15 * n) / n, chi2.ppf(0.975, 15 * n) / n)
    summary = []
    for name in cases:
        nees = np.mean(ensembles[name], axis=0)
        rows = [r for r in records if r["case"] == name]
        summary.append(
            dict(
                case=name,
                n=n,
                duration_s=float(trajectory.t[-1]),
                position_rmse_mean_m=float(np.mean([r["position_rmse"] for r in rows])),
                velocity_rmse_mean_mps=float(np.mean([r["velocity_rmse"] for r in rows])),
                attitude_rmse_mean_deg=float(
                    np.rad2deg(np.mean([r["attitude_rmse"] for r in rows]))
                ),
                position_max_m=max(r["position_max"] for r in rows),
                mean_nees=float(np.mean(nees)),
                nees_pointwise_95_coverage=float(
                    np.mean((nees >= limits[0]) & (nees <= limits[1]))
                ),
                nis_gnss_mean=float(np.mean([r["nis_gnss_mean"] for r in rows])),
                nis_radar_mean=float(np.mean([r["nis_radar_mean"] for r in rows])),
            )
        )
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    labels = {
        "nominal": "Nominal",
        "startup_5s": "GNSS absent 0–5 s",
        "outage_20s": "GNSS absent 60–80 s",
    }
    for name in cases:
        axes[0].plot(trajectory.t, traces[name]["position"], label=labels[name])
        axes[1].plot(trajectory.t, np.rad2deg(traces[name]["attitude"]), label=labels[name])
        axes[2].plot(trajectory.t, np.mean(ensembles[name], axis=0), label=labels[name])
    axes[0].set_ylabel("Position error [m], seed 0")
    axes[1].set_ylabel("Attitude error [deg], seed 0")
    axes[2].set_ylabel("Mean NEES, 5 seeds")
    axes[2].set_yscale("log")
    axes[2].axhspan(*limits, color="grey", alpha=0.2, label="Pointwise 95% band (15 states)")
    axes[2].set_xlabel("Ascent elapsed time [s]")
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(assets / "navigation_robustness.pdf", metadata={"CreationDate": None})
    fig.savefig(assets / "navigation_robustness.png", dpi=160)
    plt.close(fig)
    lines = [
        r"\begin{tabularx}{\textwidth}{Yrrrr}\toprule",
        r"Case & Soft success & Max speed [m/s] & Max miss [m] & Mean pos. RMSE [m]\\\midrule",
    ]
    for row in groups:
        label = "Project 2" if row["scenario"] == "project2_landing" else "Calibrated"
        if row["case"] == "startup_outage":
            label += ", startup outage"
        lines.append(
            f"{label} & {row['successes']}/{row['n']} & {row['speed_max_mps']:.3f} & "
            f"{row['miss_max_m']:.3f} & {row['position_rmse_mean_m']:.3f}" + r"\\"
        )
    lines.append(r"\bottomrule\end{tabularx}")
    (assets / "navigation_robustness_table.tex").write_text("\n".join(lines) + "\n")
    lines = [
        r"\begin{tabularx}{\textwidth}{Yrrrr}\toprule",
        r"Ascent case & Pos. RMSE [m] & Vel. RMSE [m/s] & Att. RMSE [deg] & Mean NEES\\\midrule",
    ]
    for row in summary:
        label = {
            "nominal": "Nominal",
            "startup_5s": "Startup outage (5 s)",
            "outage_20s": "In-flight outage (20 s)",
        }[row["case"]]
        lines.append(
            f"{label} & {row['position_rmse_mean_m']:.3f} & "
            f"{row['velocity_rmse_mean_mps']:.3f} & {row['attitude_rmse_mean_deg']:.3f} & "
            f"{row['mean_nees']:.2f}" + r"\\"
        )
    lines.append(r"\bottomrule\end{tabularx}")
    (assets / "navigation_ascent_table.tex").write_text("\n".join(lines) + "\n")
    result = dict(
        closed_loop_runs=closed,
        closed_loop_summary=groups,
        ascent_runs=records,
        ascent_summary=summary,
        nees_pointwise_95_limits=limits,
        conventions="Means over seeds; NEES bands are pointwise ensemble bands, "
        "not independent tests over correlated time samples. Constant injected biases; "
        "all channels retain modeled noise and 1 degree alignment uncertainty.",
    )
    (folder / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result
