"""Compare deterministic simulations at common logging epochs as plant_dt decreases."""

import numpy as np
from omegaconf import OmegaConf

from sim.run import run_configured
from sim.telemetry import physics_arrays

# Numerical agreement targets, not vehicle accuracy claims.
DEFAULT_LIMITS = {
    "position_m": 0.05,
    "velocity_mps": 0.05,
    "attitude_rad": 1e-3,
    "mass_kg": 0.01,
    "dynamic_pressure_pa": 10.0,
    "terminal_time_s": 0.005,
}


def compare_results(coarse, fine, log_dt):
    """Use shared log epochs; do not interpolate across command/configuration events."""

    def epochs(result):
        ticks = np.rint((result.t - result.t[0]) / log_dt).astype(int)
        good = np.isclose(result.t - result.t[0], ticks * log_dt, atol=1e-8, rtol=0)
        return {int(k): i for i, k in enumerate(ticks) if good[i]}

    ca, fa = epochs(coarse), epochs(fine)
    common = sorted(ca.keys() & fa.keys())
    if len(common) < 2:
        raise ValueError("convergence comparison needs at least two shared log epochs")
    ci, fi = [ca[k] for k in common], [fa[k] for k in common]
    qc, qf = coarse.q_bn[ci], fine.q_bn[fi]
    signs = np.where(np.sum(qc * qf, axis=1) < 0, -1.0, 1.0)
    attitude = 4 * np.arcsin(np.clip(np.linalg.norm(qc - signs[:, None] * qf, axis=1) / 2, 0, 1))
    pc, pf = physics_arrays(coarse), physics_arrays(fine)
    errors = {
        "position_m": np.max(np.linalg.norm(coarse.r_ned[ci] - fine.r_ned[fi], axis=1)),
        "velocity_mps": np.max(np.linalg.norm(coarse.v_ned[ci] - fine.v_ned[fi], axis=1)),
        "attitude_rad": np.max(attitude),
        "mass_kg": np.max(np.abs(coarse.m[ci] - fine.m[fi])),
        "dynamic_pressure_pa": np.max(np.abs(pc["aero_q_dyn"][ci] - pf["aero_q_dyn"][fi])),
        "terminal_time_s": abs(float(coarse.t[-1] - fine.t[-1])),
    }
    return {
        "errors": {k: float(v) for k, v in errors.items()},
        "shared_samples": len(common),
        "compared_until_s": float(fine.t[fi[-1]]),
        "same_contact_status": (coarse.landing_class is None) == (fine.landing_class is None),
    }


def check_convergence(cfg, steps=(0.002, 0.001, 0.0005), limits=None):
    steps = [float(x) for x in steps]
    if len(steps) < 3 or any(not np.isfinite(x) or x <= 0 for x in steps):
        raise ValueError("use at least three positive finite plant steps")
    if any(a <= b for a, b in zip(steps[:-1], steps[1:], strict=True)):
        raise ValueError("plant steps must be strictly decreasing")
    if str(cfg.scenario.get("input_mode", "closed_loop")) != "prescribed":
        raise ValueError("this physics convergence check requires prescribed inputs")
    if cfg.environment.wind.model not in ("none", "steady") or cfg.scenario.dispersion.enable:
        raise ValueError("disable stochastic wind and initial dispersion for convergence")
    tolerances = {**DEFAULT_LIMITS, **(limits or {})}
    if set(tolerances) != set(DEFAULT_LIMITS) or any(
        not np.isfinite(v) or v < 0 for v in tolerances.values()
    ):
        raise ValueError("unknown or invalid convergence limits")
    runs, configs = [], []
    for dt in steps:
        local = OmegaConf.create(OmegaConf.to_container(cfg, resolve=True))
        local.plant_dt = dt
        _, result = run_configured(local, record_telemetry=False)
        runs.append(result)
        configs.append(OmegaConf.to_container(local, resolve=True))
    pairs = []
    for i in range(len(runs) - 1):
        row = compare_results(runs[i], runs[i + 1], float(cfg.scenario.horizon.log_dt))
        row.update(coarse_dt=steps[i], fine_dt=steps[i + 1])
        row["passed"] = row["same_contact_status"] and all(
            np.isfinite(value) and value <= tolerances[key] for key, value in row["errors"].items()
        )
        pairs.append(row)
    return {
        "status": "passed" if all(x["passed"] for x in pairs) else "failed",
        "steps_s": steps,
        "limits": tolerances,
        "comparisons": pairs,
        "criterion": "all adjacent step pairs meet numerical agreement limits",
        "scope": "deterministic fixed-input comparison; no assumed RK4 order across events",
        "resolved_configs": configs,
    }
