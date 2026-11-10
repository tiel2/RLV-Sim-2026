"""Offline input/parameter identification using the unmodified native 6-DOF plant.

Every candidate is propagated from initial conditions; observations enter only the
optimizer residual. Scales/bounds/regularization are engineering assumptions.
"""

from __future__ import annotations

import numpy as np
from omegaconf import OmegaConf

from sim.run import run_configured
from validation.flight_reference import compare_flight_reference

CHANNELS = (("altitude_m", 100.0), ("speed_m_s", 10.0))


def clone(cfg):
    return OmegaConf.create(OmegaConf.to_container(cfg, resolve=True))


def time_weights(time, eligible, max_gap=1.1):
    """Original-row trapezoidal weights; never bridge excluded observations."""
    time, eligible = np.asarray(time), np.asarray(eligible, dtype=bool)
    dt = np.diff(time)
    connected = eligible[:-1] & eligible[1:] & (dt > 0) & (dt <= max_gap)
    weights = np.zeros(len(time))
    weights[:-1] += dt * connected / 2
    weights[1:] += dt * connected / 2
    if weights.sum() <= 0:
        raise ValueError("no eligible adjacent observation intervals")
    return weights / weights.sum()


class ObservationResidual:
    """Freeze eligible rows before search, so early termination cannot improve a score."""

    def __init__(self, result, reference, scales=None, *, include_initialization_support=False):
        self.report, self.arrays = compare_flight_reference(result, reference)
        self.channels = tuple((scales or dict(CHANNELS)).items())
        self.weights = {
            key: time_weights(
                self.arrays["reference_time_s"],
                (
                    self.arrays[f"{key}_quality_eligible"]
                    & self.arrays["simulation_overlap"]
                    & (self.arrays["simulation_time_s"] > 1e-9)
                    if include_initialization_support
                    else self.arrays[f"{key}_score_eligible"]
                ),
            )
            for key, _ in self.channels
        }
        self.size = sum(np.count_nonzero(w) for w in self.weights.values())

    def __call__(self, result):
        arrays = self.arrays
        values = {
            "altitude_m": np.array([p["altitude"] for p in result.aux["per_tick"]]),
            "speed_m_s": np.linalg.norm(result.v_ned, axis=1),
        }
        residual = []
        for key, scale in self.channels:
            weights = self.weights[key]
            mask = weights > 0
            epochs = arrays["simulation_time_s"][mask]
            if epochs[0] < result.t[0] or epochs[-1] > result.t[-1] + 1e-9:
                raise ValueError("candidate does not cover the frozen observation support")
            predicted = np.interp(epochs, result.t, values[key])
            if not np.all(np.isfinite(predicted)):
                raise ValueError("nonfinite candidate observations")
            residual.extend(
                (predicted - arrays[f"reference_{key}"][mask]) * np.sqrt(weights[mask]) / scale
            )
        return np.asarray(residual)


def physical_screen(cfg, result):
    p = result.aux["per_tick"]
    return bool(
        (
            abs(result.t[-1] - cfg.scenario.horizon.t_max) <= cfg.plant_dt + 1e-9
            or (
                cfg.scenario.get("input_mode") == "closed_loop"
                and result.success
                and result.landing_class == "soft"
            )
        )
        and np.all(np.isfinite(result.v_ned))
        and min(row["main_propellant"] for row in p) > 0
        and max(row["mach"] for row in p) <= cfg.plant.aero.grid.mach_max
        and max(row["aero_alpha"] for row in p)
        <= np.deg2rad(cfg.plant.aero.grid.alpha_max_deg) + 1e-8
    )


def ascent_spec(cfg, recipe):
    p = recipe.ascent
    initial = np.r_[p.throttle_initial, p.gimbal_initial_mrad]
    bounds = np.array(
        [list(p.throttle_bounds)] * len(p.throttle_times_s) + list(p.gimbal_bounds_mrad)
    )

    def candidates(x):
        c = clone(cfg)
        nt = len(p.throttle_times_s)
        rows = {
            float(t): dict(t=float(t), throttle=float(u))
            for t, u in zip(p.throttle_times_s, x[:nt], strict=True)
        }
        for (start, end), value in zip(p.gimbal_windows_s, x[nt:], strict=True):
            rows.setdefault(float(start), dict(t=float(start)))["gimbal"] = [
                [float(value * 0.001), 0.0]
            ] * 9
            if end < c.scenario.horizon.t_max:
                rows.setdefault(float(end), dict(t=float(end)))["gimbal"] = 0.0
        rows[0.0]["fin_deployment"] = 0.0
        c.scenario.inputs = [rows[t] for t in sorted(rows)]
        c.scenario.ic.throttle0 = float(x[0])
        return [c]

    def prior(x):
        nt = len(p.throttle_times_s)
        return np.r_[
            p.throttle_difference_regularization * np.diff(x[:nt]),
            p.late_gimbal_regularization * x[nt + 1 :],
        ]

    return initial, bounds.T, candidates, prior


def recovery_spec(cfgs, recipe):
    """Shared coefficients and actual fuel handoffs, independent position/velocity states."""
    p = recipe.recovery
    gamma0 = np.array(
        [np.rad2deg(np.arctan2(c.scenario.ic.v_ned[2], c.scenario.ic.v_ned[0])) for c in cfgs]
    )
    gamma_bounds = [
        [
            g + p.initial_flight_path_offset_bounds_deg[0],
            min(p.maximum_initial_flight_path_deg, g + p.initial_flight_path_offset_bounds_deg[1]),
        ]
        for g in gamma0
    ]
    ne, npow = len(p.entry_throttle_times_s), len(p.powered_throttle_times_s)
    # Entry throttles; three flight-path angles; two attitude offsets;
    # shared Cd-base/Cd-transonic/Cd-hyper/Cn;
    # powered throttles; ignition delay; entry initial mass (tonnes).
    initial = np.r_[
        [0.485] * ne, gamma0, [0, 0, 1, 1, 1, 1], [0.437] * npow, p.powered_ignition_initial_s, 40
    ]
    if p.get("initial_parameters") is not None:
        initial = np.asarray(p.initial_parameters, dtype=float)
    bounds = np.array(
        [list(p.throttle_bounds)] * ne
        + gamma_bounds
        + [list(p.initial_attitude_offset_bounds_deg)] * 2
        + [list(p.common_drag_scale_bounds)] * 3
        + [list(p.common_normal_scale_bounds)]
        + [list(p.throttle_bounds)] * npow
        + [list(p.powered_ignition_bounds_s)]
        + [list(p.initial_mass_bounds_tonnes)]
    )

    def candidates(x):
        cs = [clone(c) for c in cfgs]
        for i, c in enumerate(cs):
            for j, key in enumerate(("cd_base", "cd_transonic", "cd_hyper")):
                c.plant.aero.coeffs[key] *= float(x[ne + 5 + j])
            c.plant.aero.coeffs.cd_alpha *= float(x[ne + 5])
            c.plant.aero.coeffs.cn_alpha *= float(x[ne + 8])
            gamma = float(x[ne + i])
            speed = float(np.linalg.norm(c.scenario.ic.v_ned))
            c.scenario.ic.v_ned = [
                float(speed * np.cos(np.deg2rad(gamma))),
                0.0,
                float(speed * np.sin(np.deg2rad(gamma))),
            ]
            c.scenario.ic.pitch_deg = gamma + (0.0 if i == 0 else float(x[ne + 2 + i]))
            ignition = round(float(x[-2]) / float(c.plant_dt)) * float(c.plant_dt)
            times = [
                p.entry_throttle_times_s,
                [0.0],
                [0.0, ignition, *p.powered_throttle_times_s[1:]],
            ][i]
            throttles = [x[:ne], [0.0], [0.0, *x[ne + 9 : ne + 9 + npow]]][i]
            rows = []
            for t, u in zip(times, throttles, strict=True):
                active = [0, 1, 5] if i == 0 else [0]
                row = dict(
                    t=float(t),
                    label=str(c.scenario.reference.segment),
                    throttle=[float(u) if j in active else 0.0 for j in range(9)],
                )
                if t == 0:
                    row.update(gimbal=0.0, fin=0.0, rcs=0.0, fin_deployment=1.0)
                rows.append(row)
            c.scenario.inputs = rows
        cs[0].scenario.ic.m = float(x[-1] * 1000)
        return cs

    def prior(x):
        return np.r_[
            p.coefficient_prior_regularization * (x[ne + 5 : ne + 9] - 1),
            p.angle_prior_regularization * (x[ne : ne + 3] - gamma0),
            p.angle_prior_regularization * x[ne + 3 : ne + 5],
        ]

    return initial, bounds.T, candidates, prior


def propagate_candidates(configs, *, couple_mass=False):
    runs = []
    for cfg in configs:
        if couple_mass and runs:
            previous = runs[-1][1]
            cfg.scenario.ic.m = float(previous.m[-1])
            cfg.scenario.ic.rcs_fuel = float(previous.aux["per_tick"][-1]["rcs_fuel"])
        _, result = run_configured(cfg, record_telemetry=False)
        runs.append((cfg, result))
    return runs


def closed_loop_landing_cfg(base_cfg, powered_cfg, powered_result, recipe):
    """LCvx plan + live-state position/velocity feedback + PD attitude control."""
    p = recipe.landing
    cfg = clone(base_cfg)
    cfg.scenario.name = "flight_calibrated_landing"
    cfg.scenario.description = "Calibrated initial state; LCvx/PD closed-loop soft landing"
    cfg.scenario.input_mode = "closed_loop"
    cfg.scenario.pop("inputs", None)
    cfg.scenario.ic.r_ned = [0.0, 0.0, -float(p.initial_altitude_m)]
    cfg.scenario.ic.v_ned = [0.0, 0.0, float(np.linalg.norm(base_cfg.scenario.ic.v_ned))]
    cfg.scenario.ic.m = float(powered_result.m[-1])
    cfg.scenario.ic.rcs_fuel = float(powered_result.aux["per_tick"][-1]["rcs_fuel"])
    cfg.scenario.ic.throttle0 = float(powered_result.throttle_act[-1, 0])
    cfg.scenario.ic.pitch_deg = 90.0
    cfg.scenario.ic.yaw_deg = 0.0
    cfg.scenario.ic.w_body = [0.0, 0.0, 0.0]
    cfg.scenario.horizon.t_max = float(p.horizon_s)
    cfg.scenario.horizon.log_dt = 0.02
    cfg.plant.actuator.model = p.actuator
    cfg.plant.aero.coeffs = clone(powered_cfg.plant.aero.coeffs)
    for key, value in p.guidance.items():
        cfg.gnc.guidance[key] = (
            OmegaConf.to_container(value) if OmegaConf.is_config(value) else value
        )
    return cfg
