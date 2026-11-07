"""Compare observable trajectories with pinned webcast data, without a validation verdict."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def error_statistics(time, error, *, max_gap_s=1.1):
    """Time-weighted trapezoidal errors; exclude intervals spanning missing coverage."""
    time, error = np.asarray(time), np.asarray(error)
    if len(time) < 2:
        return {
            "samples": len(time),
            "covered_duration_s": 0.0,
            "bias": None,
            "rmse": None,
            "max_abs": None,
        }
    dt = np.diff(time)
    valid = (dt > 0) & (dt <= max_gap_s)
    duration = float(np.sum(dt[valid]))
    if duration == 0:
        return {
            "samples": len(time),
            "covered_duration_s": 0.0,
            "bias": None,
            "rmse": None,
            "max_abs": None,
        }
    bias = np.sum((error[:-1] + error[1:])[valid] * 0.5 * dt[valid]) / duration
    mse = np.sum((error[:-1] ** 2 + error[1:] ** 2)[valid] * 0.5 * dt[valid]) / duration
    return {
        "samples": len(time),
        "covered_duration_s": duration,
        "bias": float(bias),
        "rmse": float(np.sqrt(mse)),
        "max_abs": float(np.max(np.abs(error))),
    }


def compare_flight_reference(result, config, *, data_root=None):
    """Explicit offsets/conventions are recorded assumptions, never fitted here."""
    if config.get("policy") is not None:
        from validation.flight_comparison_policy import compare_policy_reference

        return compare_policy_reference(result, config, data_root=data_root)
    if config.dataset != "crs11":
        raise ValueError(f"unsupported flight reference {config.dataset!r}")
    root = (
        Path(data_root)
        if data_root
        else Path(__file__).resolve().parents[2] / "data/reference/crs11"
    )
    manifest = json.loads((root / "manifest.json").read_text())
    for item in manifest["files"]:
        if hashlib.sha256((root / item["file"]).read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"reference integrity failure: {item['file']}")
    raw = json.loads((root / "stage1_raw.json").read_text())
    rt = np.asarray(raw["time"])
    offset = float(config.time_offset_s)
    height_offset = float(config.altitude_offset_m)
    start, end = float(config.start_s), float(config.end_s)
    if not np.all(np.isfinite([offset, height_offset, start, end])) or end <= start:
        raise ValueError("reference offsets/range must be finite with end > start")
    mask = (rt >= start) & (rt <= end)
    mask &= (rt + offset >= result.t[0]) & (rt + offset <= result.t[-1])
    if config.get("exclude_initial_sample", False):
        mask &= rt + offset > result.t[0] + 1e-9
    time = rt[mask]
    if len(time) < 2:
        raise ValueError("insufficient overlapping reference coverage")
    arrays = {"reference_time_s": time, "simulation_time_s": time + offset}
    for key, observed, simulated in (
        (
            "altitude_m",
            np.asarray(raw["altitude"]) * 1000,
            np.array([row["altitude"] for row in result.aux["per_tick"]]) + height_offset,
        ),
        ("speed_m_s", np.asarray(raw["velocity"]), np.linalg.norm(result.v_ned, axis=1)),
    ):
        arrays[f"reference_{key}"] = observed[mask]
        arrays[f"simulation_{key}"] = np.interp(time + offset, result.t, simulated)
        arrays[f"error_{key}"] = arrays[f"simulation_{key}"] - observed[mask]
    events = json.loads((root / "events.json").read_text())
    segment = str(config.get("segment", "ascent"))
    windows = {"requested_overlap": (start, end)}
    if segment == "ascent":
        windows.update(
            early_ascent=(0, events["throttle_down_start"]),
            throttle_down=(events["throttle_down_start"], events["throttle_down_end"]),
            late_ascent=(events["throttle_down_end"], events["meco"]),
        )
    else:
        windows[segment] = (start, end)
    metrics = {}
    for phase, (lo, hi) in windows.items():
        selected = (time >= lo) & (time <= hi)
        metrics[phase] = {
            key: error_statistics(time[selected], arrays[f"error_{key}"][selected])
            for key in ("altitude_m", "speed_m_s")
        }
    checkpoints = {}
    event_names = {
        "ascent": [("meco_command", "meco")],
        "entry_burn": [("entry_start", "entry_start"), ("entry_end", "entry_end")],
        "aero_descent": [("entry_end", "entry_end"), ("landing_start", "landing_start")],
        "powered_descent": [("landing_start", "landing_start"), ("landing_end", "landing_end")],
    }
    epochs = [
        (label, events[key] + offset)
        for label, key in event_names.get(segment, [])
        if start <= events[key] <= end
    ]
    epochs += [("initial", float(result.t[0])), ("final", float(result.t[-1]))]
    for label, epoch in epochs:
        if epoch < result.t[0] - 1e-9 or epoch > result.t[-1] + 1e-9:
            continue
        index = int(np.argmin(np.abs(result.t - epoch)))
        if abs(float(result.t[index]) - epoch) > 1e-7:
            continue  # never present a nearby log sample as an exact event state
        aux = result.aux["per_tick"][index]
        checkpoints[label] = {
            "time_s": float(result.t[index]),
            "reference_time_s": float(result.t[index] - offset),
            **{
                key: np.asarray(getattr(result, key)[index]).tolist()
                for key in ("r_ned", "v_ned", "q_bn", "w_body", "m", "throttle_act", "tvc")
            },
            **{
                key: np.asarray(aux[key]).tolist()
                for key in ("main_propellant", "rcs_fuel", "x_cg", "inertia_diag", "thrust_mag")
            },
        }
    report = {
        "status": "comparison_only",
        "role": str(config.role),
        "segment": segment,
        "reference_commit": manifest["commit"],
        "source": "stage1_raw.json",
        "conventions": str(config.conventions),
        "time_mapping": "simulation_time = reference_time + time_offset_s",
        "time_offset_s": offset,
        "altitude_offset_m": height_offset,
        "requested_range_s": [start, end],
        "overlap_range_s": [float(time[0]), float(time[-1])],
        "simulation_reference_range_s": [float(result.t[0] - offset), float(result.t[-1] - offset)],
        "initial_sample_excluded": bool(config.get("exclude_initial_sample", False)),
        "available_overlap_duration_s": float(
            max(0, min(end, result.t[-1] - offset) - max(start, result.t[0] - offset))
        ),
        "comparison_ends_early": bool(result.t[-1] - offset < end - 1e-8),
        "reference_zero_altitude_samples": int(
            np.count_nonzero(arrays["reference_altitude_m"] == 0)
        ),
        "weighting": "trapezoidal time weighting; intervals > 1.1 s excluded; no extrapolation",
        "error_sign": "simulation minus reference",
        "metrics": metrics,
        "checkpoints": checkpoints,
        "event_time_scores": None,
        "limitations": [
            "Timing anchors are prescribed inputs, not independently predicted events.",
            "Altitude datum/speed frame remain assumptions; no acceptance limits assigned.",
            "Only the requested reference window and actual simulated overlap are scored.",
            "Initial conditions and prescribed event times are not independent predictions.",
            "Zero displayed altitude is not an exact touchdown measurement.",
        ],
    }
    return report, arrays


def write_flight_comparison(result, config, out_dir, *, plot=True):
    report, arrays = compare_flight_reference(result, config)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "flight_comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    np.savez_compressed(out / "flight_comparison.npz", **arrays)
    if plot:
        import matplotlib.pyplot as plt

        time = arrays["reference_time_s"]
        fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        for row, (key, label, scale) in enumerate(
            (("altitude_m", "Altitude [km]", 1000), ("speed_m_s", "Speed [m/s]", 1))
        ):
            # Point observations avoid visually connecting the raw-data gap.
            eligible = arrays.get(f"{key}_score_eligible", np.ones(len(time), dtype=bool))
            axes[row, 0].plot(
                time[eligible],
                arrays[f"reference_{key}"][eligible] / scale,
                ".",
                ms=2,
                label="Scored webcast",
            )
            if np.any(~eligible):
                axes[row, 0].plot(
                    time[~eligible],
                    arrays[f"reference_{key}"][~eligible] / scale,
                    "x",
                    ms=2,
                    color="0.65",
                    label="Excluded / no overlap",
                )
            axes[row, 0].plot(
                time, arrays[f"simulation_{key}"] / scale, ".", ms=1, label="Simulation"
            )
            axes[row, 0].set_ylabel(label)
            axes[row, 0].legend()
            axes[row, 1].plot(time, arrays[f"error_{key}"] / scale, ".", ms=1)
            axes[row, 1].set_ylabel(f"Error: {label}")
            axes[row, 1].axhline(0, color="black", lw=0.5)
        for ax in axes.flat:
            ax.grid(alpha=0.25)
            ax.set_xlabel("Reference T+ [s]")
        fig.suptitle(f"CRS-11 {report['segment']}: {config.role} (assumed datum/frame)")
        fig.tight_layout()
        (out / "plots").mkdir(exist_ok=True)
        fig.savefig(out / "plots/flight_comparison.png", dpi=160)
        plt.close(fig)
    return report
