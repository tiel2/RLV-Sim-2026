"""Bounded reconstruction with an assumption-conditioned landing and coupled fuel budgets.

Writes new flight_reconstructed_* overlays; preserves the unfitted configurations and runs.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf
from scipy.optimize import minimize

from core.interfaces import Phase
from gnc.build import build_gnc
from plant.dynamics import build_plant
from sim.run import run_configured
from sim.telemetry import physics_arrays
from validation.flight_reconstruction import direction_envelope, landing_state
from validation.flight_reference import compare_flight_reference
from validation.segment_spec import load_snapshot

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/verification/public_flight_comparison/reconstruction_20261107"
NAMES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]


def base(name):
    return compose(config_name="config", overrides=[f"scenario=flight_comparison_{name}"])


def screening(cfg, result):
    p = physics_arrays(result)
    valid = (
        abs(result.t[-1] - cfg.scenario.horizon.t_max) <= cfg.plant_dt
        and np.all(np.isfinite(p["v_ned"]))
        and np.all(p["main_propellant"] > 0)
        and np.all(np.rad2deg(p["aero_alpha"]) <= cfg.plant.aero.grid.alpha_max_deg + 1e-8)
        and np.all(p["mach"] <= cfg.plant.aero.grid.mach_max)
    )
    return bool(valid)


def score(cfg, result):
    if not screening(cfg, result):
        raise ValueError("candidate failed horizon/fuel/body-table screen")
    report, _ = compare_flight_reference(result, cfg.scenario.reference)
    value = report["normalized_reconstruction_mse"]
    if value is None or not np.isfinite(value):
        raise ValueError("no full eligible comparison")
    return value


def ascent_run(x):
    cfg = base("ascent")
    for index, throttle in zip([0, 3, 4], x[:3], strict=True):
        cfg.scenario.inputs[index].throttle = float(throttle)
    cfg.scenario.ic.throttle0 = float(x[0])
    cfg.scenario.inputs[1].gimbal = [[float(x[3]), 0.0] for _ in range(9)]
    _, result = run_configured(cfg, record_telemetry=False)
    return score(cfg, result), [(cfg, result)]


def recovery_run(x, recipe):
    mass, gas = float(x[0]), float(recipe.mass.rcs_fuel_kg)
    runs, scores = [], []
    for name in NAMES[1:4]:
        cfg = base(name)
        cfg.scenario.ic.m, cfg.scenario.ic.rcs_fuel = mass, gas
        if name != "entry_burn":
            key = "aero" if name == "aero_descent" else "powered"
            lo, hi = recipe.search.recovery[f"{key}_mass_bounds_kg"]
            if not lo <= mass <= hi:
                raise ValueError(f"{key} coupled mass outside declared bounds")
        if name != "aero_descent":
            value = float(x[1] if name == "entry_burn" else x[2])
            cfg.scenario.inputs[0].throttle = [
                value if t else 0.0 for t in cfg.scenario.inputs[0].throttle
            ]
        _, result = run_configured(cfg, record_telemetry=False)
        scores.append(score(cfg, result))
        runs.append((cfg, result))
        mass, gas = float(result.m[-1]), float(result.aux["per_tick"][-1]["rcs_fuel"])
    lo, hi = recipe.search.recovery.landing_mass_bounds_kg
    if not lo <= mass <= hi:
        raise ValueError("landing coupled mass outside declared bounds")
    return float(np.mean(scores)), runs


def bounded_search(label, run, spec, budget, options):
    trace, best = [], {}

    def objective(x):
        try:
            value, runs = run(x)
            status = "screened"
            if value < best.get("objective", np.inf):
                best.update(objective=float(value), parameters=list(map(float, x)), runs=runs)
        except ValueError as error:
            value, status = 1e12, str(error)
        trace.append(dict(parameters=list(map(float, x)), objective=float(value), status=status))
        if len(trace) % 5 == 0:
            print(label, len(trace), "best", best.get("objective"), flush=True)
        return value

    objective(np.asarray(spec.initial))
    result = minimize(
        objective,
        np.asarray(spec.initial),
        method="Powell",
        bounds=spec.bounds,
        options=dict(maxfev=budget - 1, xtol=options.xtol, ftol=options.ftol),
    )
    if not best:
        raise RuntimeError(f"{label}: no screened candidate")
    report = dict(
        best={k: v for k, v in best.items() if k != "runs"},
        trace=trace,
        optimizer_success=bool(result.success),
        stop_reason=str(result.message),
        selection="best screened evaluation, not a convergence claim",
    )
    (OUT / f"{label}_search.json").write_text(json.dumps(report, indent=2) + "\n")
    return best["runs"], report


def landing_cfg(mass, gas, fraction, speed, recipe):
    cfg = base("landing")
    cfg.plant.vehicle.mass.m_dry = recipe.mass.common_booster_dry_kg
    cfg.scenario.ic.m, cfg.scenario.ic.rcs_fuel = mass, gas
    state = landing_state(
        fraction, speed, recipe.landing.initial_altitude_m, float(cfg.environment.gravity.radius)
    )
    cfg.scenario.ic.r_ned, cfg.scenario.ic.v_ned = state["r_ned"], state["v_ned"]
    return cfg, state


def export(name, cfg, recipe):
    scenario = dict(
        name=f"flight_reconstructed_{name}",
        description="Bounded observation reconstruction; assumptions and held-in fit documented",
        ic=OmegaConf.to_container(cfg.scenario.ic),
        reference=dict(role="bounded_observation_reconstruction_v2"),
        reconstruction=dict(
            future_parameters=[], parameter_source="configs/analysis/flight_reconstruction.yaml"
        ),
    )
    if name != "landing":
        scenario["inputs"] = OmegaConf.to_container(cfg.scenario.inputs)
        scenario["reconstruction"]["provenance"] = dict(
            mass_propellant_actuators=(
                "fixed ascent mass"
                if name == "ascent"
                else "entry mass bounded; recovery masses use actual fuel flow"
            ),
            inputs="bounded input fit; held-in reconstruction, not measured commands",
        )
    else:
        scenario["reconstruction"]["provenance"] = dict(
            velocity_components="6 s speed integral; fixed fraction; 100 m envelope; midpoint",
            position_heading_attitude_rates="target-centered sphere; northward/upright assumed",
            landing_target="forward stopping-distance engineering target, not measured LZ-1",
            mass_propellant_actuators="terminal mass carried; 25000 kg structure; ideal actuation",
        )
    fragment = dict(defaults=[f"flight_comparison_{name}", "_self_"], scenario=scenario)
    if name == "landing":
        fragment["plant"] = dict(vehicle=dict(mass=dict(m_dry=recipe.mass.common_booster_dry_kg)))
    path = ROOT / f"configs/scenario/flight_reconstructed_{name}.yaml"
    path.write_text(
        "# @package _global_\n# Generated by scripts/reconstruct_flight.py.\n"
        + OmegaConf.to_yaml(fragment)
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    recipe_path = ROOT / "configs/analysis/flight_reconstruction.yaml"
    recipe = OmegaConf.load(recipe_path)
    _, time, height, speed, _ = load_snapshot(ROOT / "data/reference/crs11")
    envelopes = [
        direction_envelope(
            time,
            height,
            speed,
            epoch=recipe.landing.epoch_s,
            span=recipe.landing.lookback_s,
            height0=recipe.landing.initial_altitude_m,
            tolerance=float(tol),
        )
        for tol in recipe.landing.envelope_sensitivity_m
    ]
    nominal = next(r for r in envelopes if r["tolerance_m"] == recipe.landing.altitude_envelope_m)
    if nominal["nominal_fraction"] is None:
        raise ValueError("declared landing envelope infeasible; no automatic tolerance widening")
    (OUT / "direction_envelope.json").write_text(json.dumps(envelopes, indent=2) + "\n")
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        ascent, asc_report = bounded_search(
            "ascent",
            ascent_run,
            recipe.search.ascent,
            recipe.search.ascent_max_evaluations,
            recipe.search,
        )
        recovery, rec_report = bounded_search(
            "recovery",
            lambda x: recovery_run(x, recipe),
            recipe.search.recovery,
            recipe.search.recovery_max_evaluations,
            recipe.search,
        )
        terminal = recovery[-1][1]
        mass, gas = float(terminal.m[-1]), float(terminal.aux["per_tick"][-1]["rcs_fuel"])
        sensitivity = []
        selected_cfg = None
        for fraction in [*nominal["fraction_interval"], nominal["nominal_fraction"]]:
            cfg, state = landing_cfg(mass, gas, fraction, nominal["speed0_mps"], recipe)
            bundle = build_plant(cfg)
            gnc = build_gnc(cfg, bundle.params)
            law = gnc.guidance.phase_guidance[int(Phase.POWERED_DESCENT)]._law
            plan = law._plan(np.array(cfg.scenario.ic.r_ned), np.array(cfg.scenario.ic.v_ned), mass)
            row = dict(**state, mass_kg=mass, initial_plan_feasible=plan is not None)
            if plan is not None:
                _, result = run_configured(cfg, record_telemetry=False)
                row.update(
                    plan_duration_s=float(plan["tf"]),
                    duration_s=float(result.t[-1]),
                    landing_class=result.landing_class,
                    success=bool(result.success),
                    speed_mps=float(result.landing_speed),
                    position_error_m=float(result.landing_position_error),
                    fuel_kg=float(result.fuel_margin),
                )
                if fraction == nominal["nominal_fraction"] and result.success:
                    selected_cfg = cfg
            sensitivity.append(row)
            print("landing", row, flush=True)
        (OUT / "landing_sensitivity.json").write_text(json.dumps(sensitivity, indent=2) + "\n")
        if selected_cfg is None:
            raise RuntimeError(
                "nominal landing did not soft-land; no alternative silently selected"
            )
        for name, (cfg, _) in zip(NAMES[:4], ascent + recovery, strict=True):
            export(name, cfg, recipe)
        export("landing", selected_cfg, recipe)
    summary = dict(
        recipe=OmegaConf.to_container(recipe),
        ascent=asc_report["best"],
        recovery=rec_report["best"],
        landing=sensitivity,
        stage="bounded candidates ready for archived full-run verification",
    )
    sources = [recipe_path, Path(__file__), ROOT / "src/validation/flight_reconstruction.py"]
    summary["source_sha256"] = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources
    }
    (OUT / "search_summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
