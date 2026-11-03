"""Verify independent prescribed-input physics experiments.

Run run_single.py scenarios separately for full telemetry/replay exports. This checker
uses independent equations and shared-grid step refinement. Webcast comparison is
optional via --compare-flight and does not determine the physics verdict.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from _plotting import plot_episode, plot_physics
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from sim.run import run_configured
from sim.telemetry import physics_arrays
from validation.ascent import compare, reference
from validation.convergence import check_convergence
from validation.flight_reference import write_flight_comparison
from validation.references import mass_properties

ROOT = Path(__file__).resolve().parents[1]
CASES = ("ascent", "entry_burn", "aero_descent", "powered_descent")
REFERENCE_LIMITS = {"r_ned": 0.01, "v_ned": 0.001, "w_body": 1e-6, "q_bn": 1e-6, "m": 0.001}
STEP_LIMITS = {
    "position_m": 0.1,
    "velocity_mps": 0.01,
    "attitude_rad": 1e-4,
    "mass_kg": 0.01,
    "dynamic_pressure_pa": 10.0,
    "terminal_time_s": 0.005,
}


def verify(cfg, out, *, compare_flight=False):
    out.mkdir(parents=True, exist_ok=True)
    (out / "resolved_config.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    bundle, actual = run_configured(cfg, record_telemetry=False)
    constants = OmegaConf.to_container(cfg, resolve=True)
    print(f"{cfg.scenario.name}: independent DOP853 and tolerance refinement", flush=True)
    expected = reference(constants, actual.t, rtol=1e-11, allow_neutral_fins=True)
    refined = reference(constants, actual.t, rtol=1e-12, allow_neutral_fins=True)
    errors = compare(actual, refined)
    stability = compare(SimpleNamespace(**expected), refined)
    independent_passed = all(
        errors[k] <= tol and stability[k] <= tol / 10 for k, tol in REFERENCE_LIMITS.items()
    )
    p = physics_arrays(actual)
    v = cfg.plant.vehicle
    dry = float(bundle.params.mass.m_dry)
    properties = [
        mass_properties(v, m, gas) for m, gas in zip(refined["m"], p["rcs_fuel"], strict=True)
    ]
    active = np.zeros(v.engines.n_engines, dtype=bool)
    for row in cfg.scenario.inputs:
        active |= np.broadcast_to(np.asarray(row.get("throttle", 0)), active.shape) > 0
    force = p["engine_force_body"]
    diagnostics = dict(
        quaternion_norm_error=float(np.max(abs(np.linalg.norm(actual.q_bn, axis=1) - 1))),
        mass_bookkeeping_error_kg=float(
            np.max(abs(actual.m - dry - p["main_propellant"] - p["rcs_fuel"]))
        ),
        minimum_main_propellant_kg=float(np.min(p["main_propellant"])),
        maximum_mass_increase_kg=float(max(0, np.max(np.diff(actual.m)))),
        rcs_budget_error_kg=float(np.max(abs(p["rcs_fuel"] - cfg.scenario.ic.rcs_fuel))),
        cg_error_m=float(np.max(abs(p["x_cg"] - np.array([row[0] for row in properties])))),
        inertia_relative_error=float(
            np.max(
                np.linalg.norm(
                    p["inertia_diag"] - np.array([np.diag(row[1]) for row in properties]), axis=1
                )
                / np.linalg.norm(p["inertia_diag"], axis=1)
            )
        ),
        inactive_engine_force_n=float(np.max(abs(force[:, ~active]))) if np.any(~active) else 0,
        final_thrust_n=float(p["thrust_mag"][-1]),
        final_altitude_m=float(p["altitude_m"][-1]),
        final_speed_mps=float(p["speed_mps"][-1]),
        final_main_propellant_kg=float(p["main_propellant"][-1]),
        maximum_alpha_deg=float(np.rad2deg(np.max(p["aero_alpha"]))),
        maximum_mach=float(np.max(p["mach"])),
    )
    checks = dict(
        finite_state=bool(
            all(
                np.all(np.isfinite(getattr(actual, k)))
                for k in ("t", "r_ned", "v_ned", "q_bn", "w_body", "m")
            )
        ),
        quaternion=diagnostics["quaternion_norm_error"] <= 1e-12,
        mass_bookkeeping=diagnostics["mass_bookkeeping_error_kg"] <= 0.001,
        main_propellant=diagnostics["minimum_main_propellant_kg"] >= 0,
        no_mass_creation=diagnostics["maximum_mass_increase_kg"] <= 1e-8,
        rcs_budget=diagnostics["rcs_budget_error_kg"] <= 1e-8,
        mass_properties=(
            diagnostics["cg_error_m"] <= 1e-6 and diagnostics["inertia_relative_error"] <= 1e-7
        ),
        inactive_engines=diagnostics["inactive_engine_force_n"] <= 1e-8,
        actual_shutdown=diagnostics["final_thrust_n"] <= 1e-6,
        independent_equations=independent_passed,
    )
    print(f"{cfg.scenario.name}: 2/1/0.5 ms native step refinement", flush=True)
    convergence = check_convergence(cfg, limits=STEP_LIMITS)
    checks["step_refinement"] = convergence["status"] == "passed"
    (out / "convergence.json").write_text(json.dumps(convergence, indent=2, allow_nan=False) + "\n")
    comparison = (
        write_flight_comparison(actual, cfg.scenario.reference, out) if compare_flight else None
    )
    alpha_limit = float(cfg.plant.aero.grid.alpha_max_deg)
    applicability = dict(
        body_lookup_domain_met=bool(
            np.all(np.rad2deg(p["aero_alpha"]) <= alpha_limit + 1e-8)
            and np.all(p["mach"] <= cfg.plant.aero.grid.mach_max)
        ),
        body_alpha_limit_deg=alpha_limit,
        alpha_outside_table_samples=int(np.sum(np.rad2deg(p["aero_alpha"]) > alpha_limit + 1e-8)),
        mach_outside_table_samples=int(np.sum(p["mach"] > cfg.plant.aero.grid.mach_max)),
        empirical_vehicle_validated=False,
        note="Native/reference use the same declared clipped domain; numerical agreement does not "
        "validate out-of-domain or real-vehicle aerodynamics.",
    )
    checks["body_lookup_domain"] = applicability["body_lookup_domain_met"]
    report = dict(
        status="passed" if all(checks.values()) else "failed",
        scope="declared-equation implementation and numerical verification; not flight validation",
        checks=checks,
        diagnostics=diagnostics,
        independent_reference=dict(
            method="full segment, configuration-only matrix DOP853; neutral fins",
            rtol=[1e-11, 1e-12],
            errors=errors,
            refinement_errors=stability,
            limits=REFERENCE_LIMITS,
        ),
        model_applicability=applicability,
        flight_comparison_status=comparison["status"] if comparison else "not_requested",
        experiment_completed=bool(abs(actual.t[-1] - cfg.scenario.horizon.t_max) <= cfg.plant_dt),
        landing_assessed=actual.landing_class is not None,
        landing_success=bool(actual.success) if actual.landing_class is not None else None,
        termination_reason=int(actual.termination_reason),
    )
    (out / "verification.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(out / "independent_reference.npz", t=actual.t, **refined)
    np.savez_compressed(out / "physics.npz", **p)
    plot_physics(p, out / "plots")
    plot_episode(actual, out / "plots", title=str(cfg.scenario.name) + " | physics experiment")
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for ax, key, values in zip(
        axes.flat,
        ["r_ned", "v_ned", "q_bn", "m"],
        [actual.r_ned, actual.v_ned, actual.q_bn, actual.m],
        strict=True,
    ):
        if key == "q_bn":
            from validation.references import attitude_error

            error = attitude_error(values, refined[key])
        else:
            delta = values - refined[key]
            error = np.linalg.norm(delta, axis=1) if delta.ndim > 1 else abs(delta)
        ax.semilogy(actual.t, np.maximum(error, 1e-16))
        ax.axhline(REFERENCE_LIMITS[key], color="red", ls="--")
        ax.set(xlabel="Local time [s]", ylabel=f"{key} error (SI)")
        ax.grid(alpha=0.25)
    fig.suptitle(f"{cfg.scenario.name}: independent equations (not flight-data accuracy)")
    fig.tight_layout()
    fig.savefig(out / "independent_errors.png", dpi=150)
    plt.close(fig)
    return report, comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scenario", choices=CASES, action="append")
    parser.add_argument(
        "--compare-flight",
        action="store_true",
        help="add optional CRS-11 observable comparison; not an acceptance criterion",
    )
    args = parser.parse_args()
    if args.output is None:
        folder = "optional_comparison" if args.compare_flight else "physics"
        args.output = ROOT / "results/verification/project2_segments" / folder
    args.output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    with initialize_config_dir(config_dir=str(ROOT / "configs"), version_base=None):
        for name in args.scenario or CASES:
            cfg = compose(config_name="config", overrides=[f"scenario=project2_{name}"])
            cfg.scenario.reference.enabled = bool(args.compare_flight)
            report, comparison = verify(cfg, args.output / name, compare_flight=args.compare_flight)
            summaries[name] = dict(
                verification_status=report["status"],
                model_applicability=report["model_applicability"],
                final_altitude_m=report["diagnostics"]["final_altitude_m"],
                landing_assessed=report["landing_assessed"],
                experiment_completed=report["experiment_completed"],
                diagnostics=report["diagnostics"],
                flight_comparison_status=report["flight_comparison_status"],
                flight_errors=comparison["metrics"]["requested_overlap"] if comparison else None,
                comparison_ends_early=comparison["comparison_ends_early"] if comparison else None,
            )
    sources = [ROOT / "src/native/plant.cpp", Path(__file__)]
    sources += sorted((ROOT / "src/validation").glob("*.py"))
    sources += sorted((ROOT / "src/sim").glob("*.py"))
    sources += sorted((ROOT / "configs").rglob("*.yaml"))
    summary = dict(
        status="passed"
        if all(s["verification_status"] == "passed" for s in summaries.values())
        else "failed",
        scope="physics implementation, numerical agreement and declared body-table applicability",
        optional_flight_comparison=bool(args.compare_flight),
        selected_cases=list(summaries),
        complete_demonstration_set=set(summaries) == set(CASES),
        scenarios=summaries,
        source_sha256={
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources
        },
    )
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "source_sha256"}, indent=2))
    if summary["status"] != "passed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
