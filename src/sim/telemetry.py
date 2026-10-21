"""Export independent state, navigation, and actuator timelines with explicit semantics."""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path

import numpy as np


def navigation_errors(history) -> dict:
    """Estimate minus truth at matching epochs; attitude uses the shortest rotation angle."""
    truth, estimate = history.truth, history.estimate
    if not np.allclose(truth.t, estimate.t, rtol=0.0, atol=1e-8):
        raise ValueError("navigation errors require matching truth/estimate epochs")
    error = {
        key: np.asarray(getattr(estimate, key)) - np.asarray(getattr(truth, key))
        for key in ("r_ned", "v_ned", "w_body", "m")
    }
    qt, qe = np.asarray(truth.q_bn), np.asarray(estimate.q_bn)
    qt = qt / np.linalg.norm(qt, axis=1, keepdims=True)
    qe = qe / np.linalg.norm(qe, axis=1, keepdims=True)
    scalar = np.sum(qt * qe, axis=1)
    vector = qt[:, :1] * qe[:, 1:] - qe[:, :1] * qt[:, 1:] - np.cross(qt[:, 1:], qe[:, 1:])
    error["attitude_rad"] = 2.0 * np.arctan2(np.linalg.norm(vector, axis=1), np.abs(scalar))
    return error


def write_telemetry(result, out_dir) -> dict[str, Path]:
    """NPZ arrays carry timestamps; schema JSON documents frames, units, and sampling."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = {}
    timing = out / "timing.json"
    timing.write_text(json.dumps(result.aux["schedule"], indent=2))
    paths["timing"] = timing
    schema = {
        "version": 1,
        "time": "seconds since simulation epoch; native state time, not wall-clock time",
        "state_frame": "r_ned/v_ned: North-East-Down; altitude=-r_ned[2]",
        "attitude": "q_bn=[w,x,y,z], body-to-NED; w_body in body rad/s",
        "state_units": "r:m, v:m/s, m:kg",
        "trajectory": "initial truth, log_dt endpoints, prescribed events, and terminal endpoint",
        "physics": {
            "downrange_m": "horizontal distance from initial NED position (not track arclength)",
            "flight_path_angle_rad": "ground-relative, positive climb; NaN at zero speed",
            "forces_moments": "body frame; N and N m; gravity force is reported separately",
            "inertia_diag": "diagonal inertia [kg m^2]; products are configured constants",
            "fin_deployment": "0/1 aerodynamic exposure; no hinge/impact/inertia dynamics",
            "events": "inputs exact on grid; depletion detection uncertainty is recorded",
            "event_sampling": "configuration is post-event; actuators precede next input",
        },
        "navigation": "synchronous truth/measurement/estimate at every nav update; not log_dt",
        "navigation_errors": "estimate minus truth; attitude is shortest rotation angle in rad",
        "actuators": {
            "sampling": "one row per native block [t_start,t_end], before next endpoint command",
            "t_command": "creation time of latest controller output",
            "t_held": "creation time of command delivered after control-period transport delay",
            "commanded": "latest GNC or prescribed input before transport delay",
            "held": "input delivered to native plant after transport delay",
            "effective": "endpoint plant inputs after actuator limits/lag/RCS duty",
            "units": "throttle:fraction, gimbal/fin:rad, rcs:normalized signed duty",
            "faults": "engine health multipliers and nozzle cant/bias are downstream effects",
            "initial": "delay startup holds supplied actuator state; initial RCS command is zero",
        },
    }
    if result.navigation is not None:
        arrays = {}
        for kind in ("truth", "measurement", "estimate"):
            history = getattr(result.navigation, kind)
            arrays.update(
                {f"{kind}_{field.name}": getattr(history, field.name) for field in fields(history)}
            )
        errors = navigation_errors(result.navigation)
        arrays.update({f"error_{key}": value for key, value in errors.items()})
        path = out / "navigation.npz"
        np.savez_compressed(path, **arrays)
        paths["navigation"] = path
        stats = {}
        for key, value in errors.items():
            magnitude = np.linalg.norm(value, axis=1) if value.ndim == 2 else np.abs(value)
            stats[key] = {
                "rmse": float(np.sqrt(np.mean(magnitude**2))),
                "max": float(np.max(magnitude)),
            }
        stats["n_samples"] = len(result.navigation.truth.t)
        path = out / "navigation_stats.json"
        path.write_text(json.dumps(stats, indent=2))
        paths["navigation_stats"] = path
    if result.actuators is not None:
        history = result.actuators
        arrays = {key: getattr(history, key) for key in ("t_start", "t_end", "t_command", "t_held")}
        for kind in ("commanded", "held", "effective"):
            command = getattr(history, kind)
            arrays.update(
                {f"{kind}_{field.name}": getattr(command, field.name) for field in fields(command)}
            )
        path = out / "actuators.npz"
        np.savez_compressed(path, **arrays)
        paths["actuators"] = path
    path = out / "telemetry_schema.json"
    path.write_text(json.dumps(schema, indent=2))
    paths["schema"] = path
    return paths


def physics_arrays(result):
    """Truth and diagnostic arrays on the trajectory grid (including event epochs)."""
    t = np.asarray(result.t)
    speed = np.linalg.norm(result.v_ned, axis=1)
    horizontal_speed = np.linalg.norm(result.v_ned[:, :2], axis=1)
    gamma = np.arctan2(-result.v_ned[:, 2], horizontal_speed)
    gamma = np.where(speed > 1e-9, gamma, np.nan)  # undefined at rest
    arrays = {
        "t": t,
        "r_ned": result.r_ned,
        "v_ned": result.v_ned,
        "q_bn": result.q_bn,
        "w_body": result.w_body,
        "mass": result.m,
        "altitude_m": -result.r_ned[:, 2],
        "speed_mps": speed,
        "downrange_m": np.linalg.norm(result.r_ned[:, :2] - result.r_ned[0, :2], axis=1),
        "flight_path_angle_rad": gamma,
    }
    ticks = result.aux.get("per_tick", [])
    if ticks:
        for key in set.intersection(*(set(row) for row in ticks)) - {"t"}:
            arrays[key] = np.stack([row[key] for row in ticks])
    return arrays


def write_physics(result, out_dir):
    """Execution, mission performance, and analytical validation are distinct outcomes."""
    out = Path(out_dir)
    arrays = physics_arrays(result)
    np.savez_compressed(out / "physics.npz", **arrays)
    events = result.aux.get("events", [])
    (out / "events.json").write_text(json.dumps(events, indent=2))
    metrics = {
        "execution_status": "ground_contact"
        if result.landing_class is not None
        else "horizon_reached",
        "input_mode": result.aux.get("input_mode", "closed_loop"),
        "duration_s": float(result.t[-1] - result.t[0]),
        "landing_assessed": result.landing_class is not None,
        "landing_success": bool(result.success) if result.landing_class is not None else None,
        "max_dynamic_pressure_pa": float(np.max(arrays["aero_q_dyn"])),
        "max_mach": float(np.max(arrays["mach"])),
        "max_alpha_deg": float(np.rad2deg(np.max(arrays["aero_alpha"]))),
        "propellant_consumed_kg": float(result.m[0] - result.m[-1]),
        "final_main_propellant_kg": float(arrays["main_propellant"][-1]),
        "final_rcs_propellant_kg": float(arrays["rcs_fuel"][-1]),
        "final_altitude_m": float(arrays["altitude_m"][-1]),
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False))
    return arrays
