"""Reproducible sensor-based navigation comparisons on a fixed truth history."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.stats import chi2

from core.frames import euler_from_quat
from core.seeding import fold_in, master_key
from gnc.navigation.synthetic_eskf import SyntheticESKF, UnfilteredINS, qmul, qnorm

_COMPARED_STATES = ("r_ned", "v_ned", "q_bn", "w_body")


def _state_at(history, index: int):
    return SimpleNamespace(
        **{
            name: np.array(getattr(history, name)[index], copy=True)
            for name in (*_COMPARED_STATES, "m", "t")
        }
    )


def _attitude_error(truth: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    truth = np.asarray(truth, dtype=float)
    estimate = np.asarray(estimate, dtype=float)
    truth = truth / np.linalg.norm(truth, axis=1, keepdims=True)
    estimate = estimate / np.linalg.norm(estimate, axis=1, keepdims=True)
    conjugate = truth.copy()
    conjugate[:, 1:] *= -1.0
    relative = np.stack(
        [
            qmul(conjugate_row, estimate_row)
            for conjugate_row, estimate_row in zip(conjugate, estimate, strict=True)
        ]
    )
    return 2.0 * np.arctan2(np.linalg.norm(relative[:, 1:], axis=1), np.abs(relative[:, 0]))


def _attitude_error_vector(truth: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    truth = np.asarray(truth, dtype=float)
    estimate = np.asarray(estimate, dtype=float)
    truth = truth / np.linalg.norm(truth, axis=1, keepdims=True)
    estimate = estimate / np.linalg.norm(estimate, axis=1, keepdims=True)
    conjugate = truth.copy()
    conjugate[:, 1:] *= -1.0
    relative = np.stack(
        [
            qmul(estimate_row, conjugate_row)
            for estimate_row, conjugate_row in zip(estimate, conjugate, strict=True)
        ]
    )
    relative[relative[:, 0] < 0.0] *= -1.0
    vector_norm = np.linalg.norm(relative[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(vector_norm, relative[:, 0])
    scale = np.divide(angle, vector_norm, out=np.full_like(angle, 2.0), where=vector_norm > 1e-12)
    return relative[:, 1:] * scale[:, None]


def _attitude_error_euler_deg(truth: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    truth = np.asarray(truth, dtype=float)
    estimate = np.asarray(estimate, dtype=float)
    truth = truth / np.linalg.norm(truth, axis=1, keepdims=True)
    estimate = estimate / np.linalg.norm(estimate, axis=1, keepdims=True)
    conjugate = truth.copy()
    conjugate[:, 1:] *= -1.0
    relative = np.stack(
        [
            qmul(estimate_row, conjugate_row)
            for estimate_row, conjugate_row in zip(estimate, conjugate, strict=True)
        ]
    )
    relative[relative[:, 0] < 0.0] *= -1.0
    return np.rad2deg(np.stack([euler_from_quat(q) for q in relative]))


def _error_metrics(error: np.ndarray) -> dict:
    error = np.asarray(error, dtype=float)
    magnitude = np.abs(error) if error.ndim == 1 else np.linalg.norm(error, axis=1)
    metrics = {
        "rmse": float(np.sqrt(np.mean(magnitude**2))),
        "max": float(np.max(magnitude)),
    }
    if error.ndim == 2:
        metrics["rmse_by_axis"] = np.sqrt(np.mean(error**2, axis=0)).tolist()
    return metrics


def compare_navigation(
    truth,
    sensor: SyntheticESKF,
    seed: int,
) -> tuple[dict[str, np.ndarray], dict]:
    """Run INS-only and aided ESKF against identical sensor packets and truth epochs."""
    times = np.asarray(truth.t, dtype=float)
    if times.ndim != 1 or len(times) < 2 or not np.all(np.isfinite(times)):
        raise ValueError("truth history must contain at least two finite epochs")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("truth epochs must be strictly increasing")

    initial_state = _state_at(truth, 0)
    if sensor.initial_position is None:
        sensor.initial_position = np.asarray(initial_state.r_ned, dtype=float).copy()
    if sensor.initial_velocity is None:
        sensor.initial_velocity = np.asarray(initial_state.v_ned, dtype=float).copy()
    sensor.initial_attitude = qnorm(initial_state.q_bn)
    if sensor.initial_mass is None:
        sensor.initial_mass = float(initial_state.m)
    baseline = UnfilteredINS(initial_state=initial_state)
    filter_memory = sensor.initial_memory()
    baseline_memory = baseline.initial_memory()
    observations = []
    estimates = {"unfiltered": [], "filtered": []}
    filtered_memory = []
    sensor_bias_truth = []
    for index in range(len(times)):
        state = _state_at(truth, index)
        observation = sensor.measure(state, fold_in(master_key(seed), index))
        observations.append(observation)
        sensor_bias_truth.append(
            (sensor.ba + sensor._walk_ba, sensor.bg + sensor._walk_bg)
        )
        filtered, filter_memory = sensor.estimate(observation, None, filter_memory)
        unfiltered, baseline_memory = baseline.estimate(observation, None, baseline_memory)
        estimates["filtered"].append(filtered)
        estimates["unfiltered"].append(unfiltered)
        filtered_memory.append(
            {
                key: np.array(filter_memory[key], copy=True)
                for key in (
                    "P",
                    "bg",
                    "ba",
                    "gnss_nis",
                    "gnss_normalized_innovation",
                    "gnss_update_accepted",
                    "radar_nis",
                    "radar_normalized_innovation",
                    "radar_update_accepted",
                )
            }
        )

    arrays = {
        "t": times.copy(),
        **{
            f"truth_{name}": np.asarray(getattr(truth, name), dtype=float).copy()
            for name in _COMPARED_STATES
        },
        "truth_m": np.asarray(truth.m, dtype=float).copy(),
        "truth_accel_bias_mps2": np.stack([row[0] for row in sensor_bias_truth]),
        "truth_gyro_bias_radps": np.stack([row[1] for row in sensor_bias_truth]),
        "measurement_accel_body": np.stack([row.accel_body for row in observations]),
        "measurement_gyro_body": np.stack([row.gyro_body for row in observations]),
        "measurement_gnss_position_ned": np.stack([row.r_ned for row in observations]),
        "measurement_gnss_velocity_ned": np.stack([row.v_ned for row in observations]),
        "measurement_gnss_scheduled": np.asarray(
            [row.gnss_scheduled for row in observations]
        ),
        "measurement_gnss_valid": np.asarray([row.gnss_valid for row in observations]),
        "measurement_gnss_outlier": np.asarray([row.gnss_outlier for row in observations]),
        "measurement_radar_altitude_m": np.asarray(
            [row.radar_alt_m for row in observations], dtype=float
        ),
        "measurement_radar_scheduled": np.asarray(
            [row.radar_scheduled for row in observations]
        ),
        "measurement_radar_valid": np.asarray([row.radar_valid for row in observations]),
    }
    summary = {"n_samples": len(times), "unfiltered": {}, "filtered": {}}
    arrays["filtered_covariance_diag"] = np.stack(
        [np.diag(row["P"]) for row in filtered_memory]
    )
    arrays["filtered_covariance_9"] = np.stack(
        [row["P"][:9, :9] for row in filtered_memory]
    )
    arrays["filtered_gyro_bias_estimate_radps"] = np.stack(
        [row["bg"] for row in filtered_memory]
    )
    arrays["filtered_accel_bias_estimate_mps2"] = np.stack(
        [row["ba"] for row in filtered_memory]
    )
    arrays["gnss_nis"] = np.asarray([row["gnss_nis"] for row in filtered_memory])
    arrays["gnss_update_accepted"] = np.asarray(
        [row["gnss_update_accepted"] for row in filtered_memory]
    )
    arrays["gnss_normalized_innovation"] = np.stack(
        [row["gnss_normalized_innovation"] for row in filtered_memory]
    )
    arrays["radar_nis"] = np.asarray([row["radar_nis"] for row in filtered_memory])
    arrays["radar_update_accepted"] = np.asarray(
        [row["radar_update_accepted"] for row in filtered_memory]
    )
    arrays["radar_normalized_innovation"] = np.stack(
        [row["radar_normalized_innovation"] for row in filtered_memory]
    )
    for kind in ("unfiltered", "filtered"):
        state_records = estimates[kind]
        for name in _COMPARED_STATES:
            arrays[f"{kind}_{name}"] = np.stack(
                [np.asarray(getattr(row, name), dtype=float) for row in state_records]
            )
        errors = {
            "position_m": arrays[f"{kind}_r_ned"] - arrays["truth_r_ned"],
            "velocity_mps": arrays[f"{kind}_v_ned"] - arrays["truth_v_ned"],
            "attitude_rad": _attitude_error(
                arrays["truth_q_bn"], arrays[f"{kind}_q_bn"]
            ),
            "attitude_euler_deg": _attitude_error_euler_deg(
                arrays["truth_q_bn"], arrays[f"{kind}_q_bn"]
            ),
            "attitude_vector_rad": (
                _attitude_error_vector(arrays["truth_q_bn"], arrays[f"{kind}_q_bn"])
                if kind == "filtered"
                else np.zeros((len(times), 3))
            ),
            "angular_rate_radps": arrays[f"{kind}_w_body"] - arrays["truth_w_body"],
        }
        if kind == "unfiltered":
            errors.pop("attitude_vector_rad")
        summary[kind] = {
            name: _error_metrics(error) for name, error in errors.items()
        }
        arrays.update({f"{kind}_error_{name}": error for name, error in errors.items()})

    summary["filtered"]["gyro_bias_radps"] = _error_metrics(
        arrays["filtered_gyro_bias_estimate_radps"] - arrays["truth_gyro_bias_radps"]
    )
    summary["filtered"]["accel_bias_mps2"] = _error_metrics(
        arrays["filtered_accel_bias_estimate_mps2"] - arrays["truth_accel_bias_mps2"]
    )

    consistency_error = np.column_stack(
        (
            arrays["filtered_error_position_m"],
            arrays["filtered_error_velocity_mps"],
            arrays["filtered_error_attitude_vector_rad"],
        )
    )
    nees = np.asarray(
        [
            error @ np.linalg.solve(covariance, error)
            for error, covariance in zip(
                consistency_error, arrays["filtered_covariance_9"], strict=True
            )
        ]
    )
    nees_lower = float(chi2.ppf(0.025, 9))
    nees_upper = float(chi2.ppf(0.975, 9))
    summary["consistency"] = {
        "nees_mean": float(np.mean(nees)),
        "nees_dof": 9,
        "nees_95pct_bounds": [nees_lower, nees_upper],
        "nees_coverage_fraction": float(np.mean((nees >= nees_lower) & (nees <= nees_upper))),
        "position_3sigma_component_coverage": np.mean(
            np.abs(arrays["filtered_error_position_m"])
            <= 3.0 * np.sqrt(arrays["filtered_covariance_diag"][:, :3]),
            axis=0,
        ).tolist(),
        "velocity_3sigma_component_coverage": np.mean(
            np.abs(arrays["filtered_error_velocity_mps"])
            <= 3.0 * np.sqrt(arrays["filtered_covariance_diag"][:, 3:6]),
            axis=0,
        ).tolist(),
        "attitude_3sigma_component_coverage": np.mean(
            np.abs(arrays["filtered_error_attitude_vector_rad"])
            <= 3.0 * np.sqrt(arrays["filtered_covariance_diag"][:, 6:9]),
            axis=0,
        ).tolist(),
    }
    summary["innovation"] = {
        "gnss_updates": int(np.count_nonzero(np.isfinite(arrays["gnss_nis"]))),
        "gnss_accepted": int(np.count_nonzero(arrays["gnss_update_accepted"])),
        "gnss_rejected": int(
            np.count_nonzero(np.isfinite(arrays["gnss_nis"]) & ~arrays["gnss_update_accepted"])
        ),
        "gnss_mean_accepted_nis": (
            float(np.mean(arrays["gnss_nis"][arrays["gnss_update_accepted"]]))
            if np.any(arrays["gnss_update_accepted"])
            else None
        ),
        "gnss_mean_nis_all": (
            float(np.nanmean(arrays["gnss_nis"]))
            if np.any(np.isfinite(arrays["gnss_nis"]))
            else None
        ),
        "radar_updates": int(np.count_nonzero(np.isfinite(arrays["radar_nis"]))),
        "radar_accepted": int(np.count_nonzero(arrays["radar_update_accepted"])),
        "radar_rejected": int(
            np.count_nonzero(np.isfinite(arrays["radar_nis"]) & ~arrays["radar_update_accepted"])
        ),
        "radar_mean_accepted_nis": (
            float(np.mean(arrays["radar_nis"][arrays["radar_update_accepted"]]))
            if np.any(arrays["radar_update_accepted"])
            else None
        ),
        "radar_mean_nis_all": (
            float(np.nanmean(arrays["radar_nis"]))
            if np.any(np.isfinite(arrays["radar_nis"]))
            else None
        ),
    }
    gnss_scheduled = arrays["measurement_gnss_scheduled"]
    outage_window = (times >= sensor.outage0) & (times <= sensor.outage1)
    outage_samples = gnss_scheduled & outage_window
    outage_enabled = sensor.outage1 > sensor.outage0
    gnss_missing = gnss_scheduled & ~arrays["measurement_gnss_valid"]
    summary["sensors"] = {
        "gnss_samples": int(np.count_nonzero(arrays["measurement_gnss_valid"])),
        "gnss_scheduled_samples": int(
            np.count_nonzero(arrays["measurement_gnss_scheduled"])
        ),
        "gnss_dropouts": int(np.count_nonzero(gnss_missing & ~outage_samples)),
        "gnss_outage_scheduled_samples": int(np.count_nonzero(outage_samples))
        if outage_enabled
        else 0,
        "gnss_outage_samples": int(np.count_nonzero(gnss_missing & outage_samples))
        if outage_enabled
        else 0,
        "gnss_outliers": int(np.count_nonzero(arrays["measurement_gnss_outlier"])),
        "radar_samples": int(np.count_nonzero(arrays["measurement_radar_valid"])),
        "radar_scheduled_samples": int(
            np.count_nonzero(arrays["measurement_radar_scheduled"])
        ),
        "imu_samples": len(times),
        "latency_s": 0.0,
        "imu_rate_hz": sensor.imu_rate_hz,
        "gnss_rate_hz": 1.0 / sensor.gperiod,
        "radar_rate_hz": 1.0 / sensor.rperiod,
        "gnss_outage_s": [sensor.outage0, sensor.outage1],
    }
    summary["initialization"] = {
        "position_ned_m": np.asarray(initial_state.r_ned, dtype=float).tolist(),
        "velocity_ned_mps": np.asarray(initial_state.v_ned, dtype=float).tolist(),
        "attitude_quaternion_bn": qnorm(initial_state.q_bn).tolist(),
        "mass_kg": float(initial_state.m),
        "shared_by_both_estimators": True,
    }
    return arrays, summary


def plot_navigation_comparison(arrays: dict, summary: dict, out_dir) -> dict[str, str]:
    """Write flight-path overlays and navigation error/consistency figures."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    t = arrays["t"]
    styles = {
        "truth": ("Truth / ideal 6-DOF reference", "black", "-", 2.0),
        "unfiltered": ("IMU-only INS", "tab:orange", "--", 1.0),
        "filtered": ("GNSS/radar ESKF", "tab:blue", "-", 1.1),
    }

    paths = {}
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    trajectories = {}
    for kind in styles:
        position = arrays[f"{kind}_r_ned"]
        trajectories[kind] = {
            "north": position[:, 0],
            "east": position[:, 1],
            "altitude": -position[:, 2],
            "downrange": np.linalg.norm(position[:, :2] - position[0, :2], axis=1),
        }
    for kind, (label, color, line_style, width) in styles.items():
        path = trajectories[kind]
        axes[0, 0].plot(
            path["east"], path["north"], line_style, color=color, lw=width, label=label
        )
        axes[0, 1].plot(
            path["downrange"],
            path["altitude"],
            line_style,
            color=color,
            lw=width,
            label=label,
        )
        axes[1, 0].plot(t, path["altitude"], line_style, color=color, lw=width, label=label)
        speed = np.linalg.norm(arrays[f"{kind}_v_ned"], axis=1)
        axes[1, 1].plot(t, speed, line_style, color=color, lw=width, label=label)
    axes[0, 0].set(xlabel="East position (m)", ylabel="North position (m)", title="Horizontal path")
    axes[0, 1].set(
        xlabel="Downrange from initial position (m)",
        ylabel="Altitude above local origin (m)",
        title="Ideal flight path and estimated paths",
    )
    axes[1, 0].set(xlabel="Time (s)", ylabel="Altitude (m)", title="Altitude history")
    axes[1, 1].set(xlabel="Time (s)", ylabel="Speed (m/s)", title="Speed history")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize="small")
    path = output / "flight_path_comparison.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["flight_path"] = str(path)

    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True, constrained_layout=True)
    for kind, (label, color, line_style, width) in styles.items():
        position = arrays[f"{kind}_r_ned"]
        plotted = (position[:, 0], position[:, 1], -position[:, 2])
        for axis, values, component in zip(axes[0:1], (position,), (0,), strict=True):
            axis.plot(t, values[:, component], line_style, color=color, lw=width, label=label)
        for axis, values, component in zip(
            axes[1:2], (arrays[f"{kind}_v_ned"],), (0,), strict=True
        ):
            axis.plot(t, values[:, component], line_style, color=color, lw=width, label=label)
        if kind == "truth":
            axes[0].plot(t, plotted[1], line_style, color=color, lw=width, alpha=0.55)
            axes[0].plot(t, plotted[2], line_style, color=color, lw=width, alpha=0.8)
            axes[1].plot(
                t,
                arrays[f"{kind}_v_ned"][:, 1],
                line_style,
                color=color,
                lw=width,
                alpha=0.55,
            )
            axes[1].plot(
                t,
                arrays[f"{kind}_v_ned"][:, 2],
                line_style,
                color=color,
                lw=width,
                alpha=0.8,
            )
        if kind != "truth":
            for component in (1, 2):
                axes[0].plot(t, plotted[component], line_style, color=color, lw=width, alpha=0.75)
                axes[1].plot(
                    t,
                    arrays[f"{kind}_v_ned"][:, component],
                    line_style,
                    color=color,
                    lw=width,
                    alpha=0.75,
                )
        attitude = (
            np.zeros((len(t), 3))
            if kind == "truth"
            else arrays[f"{kind}_error_attitude_euler_deg"]
        )
        rate = np.rad2deg(arrays[f"{kind}_w_body"])
        if kind == "truth":
            axes[2].plot(t, attitude[:, 0], line_style, color=color, lw=width, label=label)
            axes[2].plot(t, attitude[:, 1], line_style, color=color, lw=width, alpha=0.55)
            axes[2].plot(t, attitude[:, 2], line_style, color=color, lw=width, alpha=0.8)
            axes[3].plot(t, rate[:, 0], line_style, color=color, lw=width, label=label)
            axes[3].plot(t, rate[:, 1], line_style, color=color, lw=width, alpha=0.55)
            axes[3].plot(t, rate[:, 2], line_style, color=color, lw=width, alpha=0.8)
        else:
            for component in range(3):
                axes[2].plot(t, attitude[:, component], line_style, color=color, lw=width)
                axes[3].plot(t, rate[:, component], line_style, color=color, lw=width)
    axes[0].set_ylabel("Position N/E/alt (m)")
    axes[1].set_ylabel("Velocity N/E/D (m/s)")
    axes[2].set_ylabel("Euler attitude error (deg)")
    axes[3].set_ylabel("Body rate x/y/z (deg/s)")
    axes[3].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize="small")
    axes[0].set_title("Truth and state estimates (position, velocity, attitude error, rates)")
    path = output / "navigation_state_comparison.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["states"] = str(path)

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    error_groups = (
        (
            "filtered_error_position_m",
            slice(0, 3),
            ("North", "East", "Down"),
            "Position error (m)",
        ),
        (
            "filtered_error_velocity_mps",
            slice(3, 6),
            ("North", "East", "Down"),
            "Velocity error (m/s)",
        ),
        (
            "filtered_error_attitude_vector_rad",
            slice(6, 9),
            ("x", "y", "z"),
            "Attitude error vector (deg)",
        ),
    )
    axis_colors = ("tab:blue", "tab:orange", "tab:green")
    for axis, (error_key, covariance_slice, labels, ylabel) in zip(
        axes, error_groups, strict=True
    ):
        errors = arrays[error_key]
        sigma = np.sqrt(arrays["filtered_covariance_diag"][:, covariance_slice])
        if "attitude" in error_key:
            errors = np.rad2deg(errors)
            sigma = np.rad2deg(sigma)
        for component, (label, color) in enumerate(zip(labels, axis_colors, strict=True)):
            axis.plot(t, errors[:, component], color=color, lw=0.8, label=f"{label} error")
            axis.plot(t, 3.0 * sigma[:, component], color=color, ls=":", lw=0.8)
            axis.plot(t, -3.0 * sigma[:, component], color=color, ls=":", lw=0.8)
        axis.axhline(0.0, color="black", lw=0.6)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize="small", ncol=3)
    outage = summary["sensors"]["gnss_outage_s"]
    if outage[1] > outage[0]:
        for axis in axes:
            axis.axvspan(outage[0], outage[1], color="grey", alpha=0.15, label="GNSS outage")
    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title("Aided ESKF errors with component-wise posterior ±3σ bounds")
    path = output / "navigation_consistency.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["consistency"] = str(path)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharex=True, constrained_layout=True)
    gnss_valid = arrays["measurement_gnss_valid"]
    radar_valid = arrays["measurement_radar_valid"]
    for component, label in enumerate(("N position", "E position", "D position")):
        mask = gnss_valid
        axes[0, 0].plot(
            t[mask],
            arrays["gnss_normalized_innovation"][mask, component],
            ".",
            ms=2,
            label=label,
        )
    for component, label in enumerate(("N velocity", "E velocity", "D velocity"), start=3):
        axes[0, 0].plot(
            t[gnss_valid],
            arrays["gnss_normalized_innovation"][gnss_valid, component],
            ".",
            ms=2,
            label=label,
        )
    axes[0, 0].axhline(3.0, color="red", ls="--")
    axes[0, 0].axhline(-3.0, color="red", ls="--")
    outliers = arrays["measurement_gnss_outlier"] & gnss_valid
    for component in range(6):
        axes[0, 0].plot(
            t[outliers],
            arrays["gnss_normalized_innovation"][outliers, component],
            "x",
            color="red",
            ms=5,
        )
    axes[0, 0].set(title="GNSS normalized innovations", ylabel="Innovation / σ")
    axes[0, 1].plot(
        t[radar_valid], arrays["radar_normalized_innovation"][radar_valid, 0], ".", ms=2
    )
    axes[0, 1].axhline(3.0, color="red", ls="--")
    axes[0, 1].axhline(-3.0, color="red", ls="--")
    axes[0, 1].set(title="Radar normalized innovations", ylabel="Innovation / σ")
    axes[1, 0].plot(
        t, np.rad2deg(arrays["filtered_gyro_bias_estimate_radps"]), lw=0.9
    )
    axes[1, 0].plot(
        t, np.rad2deg(arrays["truth_gyro_bias_radps"]), "--", lw=0.8, alpha=0.8
    )
    axes[1, 0].set(title="Estimated gyro bias", ylabel="deg/s", xlabel="Time (s)")
    axes[1, 1].plot(t, arrays["filtered_accel_bias_estimate_mps2"], lw=0.9)
    axes[1, 1].plot(
        t, arrays["truth_accel_bias_mps2"], "--", lw=0.8, alpha=0.8
    )
    axes[1, 1].set(title="Estimated accelerometer bias", ylabel="m/s²", xlabel="Time (s)")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    outage = summary["sensors"]["gnss_outage_s"]
    if outage[1] > outage[0]:
        for axis in axes[0]:
            axis.axvspan(outage[0], outage[1], color="grey", alpha=0.15)
    axes[0, 0].legend(fontsize="x-small", ncol=2)
    path = output / "navigation_innovations_bias.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["innovations_bias"] = str(path)
    return paths


def write_navigation_comparison(
    truth,
    sensor: SyntheticESKF,
    seed: int,
    out_dir: str | Path,
    *,
    scenario: str,
    sensor_configuration: dict,
) -> dict:
    """Save same-truth INS/ESKF metrics and plots into a run's result directory."""
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    arrays, metrics = compare_navigation(truth, sensor, seed)
    np.savez_compressed(output / "navigation_comparison.npz", **arrays)
    plots = plot_navigation_comparison(arrays, metrics, output)
    metadata = {
        "scenario": scenario,
        "seed": int(seed),
        "truth_navigation": "recorded simulator trajectory",
        "comparison_run": (
            "reproducible seeded sensor/estimator replay on this truth; independent of the "
            "online navigation samples used by the closed-loop controller"
        ),
        "sensor_configuration": sensor_configuration,
        "frame_and_units": {
            "position_velocity": "fixed-origin NED, m and m/s",
            "attitude": "scalar-first body-to-NED quaternion",
            "angular_rate": "body/reference rad/s",
            "accelerometer": "body-frame specific force, m/s^2, gravity excluded",
            "radar_altitude": "positive altitude above the flat NED origin, m",
            "latency": "zero seconds for this selected synchronous sensor model",
            "sensor_rates": {
                "imu_hz": sensor.imu_rate_hz,
                "gnss_hz": 1.0 / sensor.gperiod,
                "radar_altimeter_hz": 1.0 / sensor.rperiod,
            },
        },
        "limitations": [
            "Synthetic sensor parameters are assumptions, not calibrated hardware specifications.",
            "The first IMU sample assumes zero translational acceleration; later samples "
            "differentiate velocity at navigation sample times.",
            "The local gravity mechanization is a fixed-origin NED approximation.",
            "Both estimators use the same exact initial position, velocity, attitude, and mass.",
            "Mass is held at its known initial value; mass is not a scored navigation state.",
            "Covariance consistency is approximate because samples are temporally correlated "
            "and the local NED/filter model is not a rotating-Earth mechanization.",
        ],
        "metrics": metrics,
        "plots": plots,
    }
    (output / "navigation_comparison.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    return metadata
