"""P4 fixed-candidate physical audits, input replay and deterministic sensitivity.

Run from the repository root with .venv/bin/python. Existing P3 artifacts are read-only.
Each invocation requires a new output directory; failed probe cases remain in the report.
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
from omegaconf import OmegaConf

from sim.run import run_configured
from sim.telemetry import physics_arrays
from validation.ascent import compare
from validation.flight_reference import compare_flight_reference
from validation.flight_sensitivity import (
    actuator_arrays,
    common_metrics,
    observation_probe,
    perturb,
    physical_audit,
)
from validation.landing_input_reference import reference

ROOT = Path(__file__).resolve().parents[1]
NAMES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def archive_probe(out, name, cfg, result, nominal):
    out.mkdir(parents=True, exist_ok=True)
    OmegaConf.save(cfg, out / "resolved_config.yaml", resolve=True)
    p, a = physics_arrays(result), actuator_arrays(result.actuators)
    report, arrays = compare_flight_reference(result, cfg.scenario.reference)
    np.savez_compressed(out / "physics.npz", **p)
    np.savez_compressed(out / "actuators.npz", **a)
    np.savez_compressed(out / "flight_comparison.npz", **arrays)
    audit = physical_audit(p, a, cfg)
    row = dict(
        phase=name,
        duration_s=float(result.t[-1]),
        termination=int(result.termination_reason),
        horizon_reached=bool(abs(result.t[-1] - cfg.scenario.horizon.t_max) <= cfg.plant_dt),
        landing_success=bool(result.success) if name == "landing" else None,
        landing_class=result.landing_class,
        landing_speed_mps=float(result.landing_speed) if name == "landing" else None,
        landing_position_error_m=float(result.landing_position_error)
        if name == "landing"
        else None,
        final_mass_kg=float(result.m[-1]),
        fuel_margin_kg=float(result.fuel_margin),
        final_altitude_m=float(p["altitude_m"][-1]),
        final_speed_mps=float(p["speed_mps"][-1]),
        physical_audit=audit,
        full_overlap_metrics=report["metrics"]["requested_overlap"],
        common_coverage=common_metrics(nominal, arrays),
    )
    if name == "landing":
        row["guidance_diagnostics"] = result.aux.get("guidance_degradation", {})
        row["solver_counts"] = {
            key: value
            for key, value in result.aux.get("solver", {}).items()
            if key.startswith("n_")
        }
        row["ignition_command_time_s"] = result.aux.get("t_ignition")
    write(out / "assessment.json", row)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/verification/public_flight_comparison/p4_20261107",
    )
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    recipe_path = ROOT / "configs/analysis/flight_sensitivity.yaml"
    recipe = OmegaConf.load(recipe_path)
    base = ROOT / recipe.nominal
    frozen = {str(p.relative_to(ROOT)): sha(p) for p in base.rglob("*") if p.is_file()}
    old = ROOT / "results/verification/public_flight_comparison/step3_20261107/before"
    frozen.update({str(p.relative_to(ROOT)): sha(p) for p in old.rglob("*") if p.is_file()})
    sources = [recipe_path, Path(__file__)]
    sources += list((ROOT / "src").rglob("*.py")) + [ROOT / "src/native/plant.cpp"]
    sources += list((ROOT / "configs").rglob("*.yaml"))
    sources += [p for p in (ROOT / "data/reference/crs11").rglob("*") if p.is_file()]
    manifest = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    write(out / "manifest.json", dict(source_sha256=manifest, frozen_artifact_sha256=frozen))
    OmegaConf.save(recipe, out / "recipe.yaml")
    configs, arrays, physics = {}, {}, {}
    nominal = {}
    prior = json.loads((base / "summary.json").read_text())
    # Reject a stale/mismatched archive rather than trusting an old PASS string.
    for name in NAMES:
        expected = prior["phases"][name]["artifact_sha256"]
        for filename, digest in expected.items():
            if sha(base / name / filename) != digest:
                raise ValueError(f"archived nominal artifact changed: {name}/{filename}")
        configs[name] = OmegaConf.load(base / name / "resolved_config.yaml")
        arrays[name] = dict(np.load(base / name / "flight_comparison.npz"))
        physics[name] = dict(np.load(base / name / "physics.npz"))
        a = dict(np.load(base / name / "actuators.npz"))
        nominal[name] = physical_audit(physics[name], a, configs[name])
    write(out / "nominal_physical_audits.json", nominal)
    print(
        "Nominal physical audits", {k: v["failed_checks"] for k, v in nominal.items()}, flush=True
    )

    print("Independent full landing plant integration with held commands", flush=True)
    p = physics["landing"]
    act = dict(np.load(base / "landing/actuators.npz"))
    constants = OmegaConf.to_container(configs["landing"], resolve=True)
    coarse = reference(constants, act, p["t"], rtol=1e-10)
    fine = reference(constants, act, p["t"], rtol=1e-12)
    actual = SimpleNamespace(**{k: p[k] for k in ("r_ned", "v_ned", "w_body", "q_bn")}, m=p["mass"])
    errors, refinement = compare(actual, fine), compare(SimpleNamespace(**coarse), fine)
    limits = dict(recipe.numerical_limits.landing_input_replay)
    replay = dict(
        errors=errors,
        refinement=refinement,
        limits=limits,
        passed=all(
            errors[k] <= v
            and refinement[k] <= v * recipe.numerical_limits.reference_refinement_fraction
            for k, v in limits.items()
        ),
        scope="continuous independent plant integration; recorded held commands; configured IC; "
        "no state resets, feedback/controller/contact detection not independently validated",
    )
    write(out / "landing_input_replay.json", replay)
    np.savez_compressed(out / "landing_input_reference.npz", t=p["t"], **fine)
    print("Landing plant reference", replay, flush=True)

    cases = [
        (f"{key}_{label}", {key: sign})
        for key in ("speed", "direction", "pitch", "mass", "thrust", "drag", "wind")
        for label, sign in (("minus", -1), ("plus", 1))
    ]
    cases += [
        (row.name, {k: v for k, v in row.items() if k != "name"}) for row in recipe.combinations
    ]
    cases += [("landing_realistic", {"realistic": True})]
    campaigns = {}
    for label, changes in cases:
        mass, gas, rows, handoffs = None, None, {}, []
        names = ["landing"] if changes.get("realistic") else NAMES
        for name in names:
            # Pad speed/direction are fixed engineering conditions, not uncertain flight ICs.
            if name == "ascent" and ("speed" in changes or "direction" in changes):
                rows[name] = dict(status="not_applied", reason="fixed stationary launch pad")
                continue
            carried = mass if name in ("aero_descent", "powered_descent", "landing") else None
            cfg = perturb(
                configs[name], changes, recipe.probes, carried_mass=carried, carried_gas=gas
            )
            if carried is not None:
                handoffs.append(
                    dict(
                        next_phase=name,
                        total_mass_difference_kg=cfg.scenario.ic.m - mass,
                        gas_difference_kg=cfg.scenario.ic.rcs_fuel - gas,
                    )
                )
            print("Probe", label, name, flush=True)
            case_out = out / label / name
            case_out.mkdir(parents=True)
            OmegaConf.save(cfg, case_out / "resolved_config.yaml", resolve=True)
            try:
                _, result = run_configured(cfg, record_telemetry=True)
                rows[name] = archive_probe(out / label / name, name, cfg, result, arrays[name])
                if name != "ascent":
                    mass, gas = float(result.m[-1]), float(result.aux["per_tick"][-1]["rcs_fuel"])
                if (
                    name in ("entry_burn", "aero_descent", "powered_descent")
                    and not rows[name]["horizon_reached"]
                ):
                    rows[name]["chain_stopped"] = "preceding phase did not reach declared boundary"
                    break
            except (ValueError, RuntimeError) as error:
                rows[name] = dict(
                    status="rejected_capacity"
                    if "wet capacity" in str(error)
                    else "execution_error",
                    error=str(error),
                )
                write(case_out / "assessment.json", rows[name])
                if name != "ascent":
                    break
        campaigns[label] = dict(changes=changes, phases=rows, mass_handoffs=handoffs)
        write(out / "campaigns.json", campaigns)

    observations = {}
    for name in NAMES:
        epoch = configs[name].scenario.reference.start_s
        observations[name] = {
            f"datum_{offset:+g}m": observation_probe(
                physics[name], arrays[name], epoch, datum_m=offset
            )
            for offset in recipe.observation_probes.altitude_datum_m
        }
        observations[name].update(
            {
                f"time_{offset:+g}s": observation_probe(
                    physics[name], arrays[name], epoch, time_shift_s=offset
                )
                for offset in recipe.observation_probes.time_offset_s
            }
        )
    write(out / "observation_sensitivity.json", observations)
    prior_display = base.parent / "display_tolerance_sensitivity.json"
    display = json.loads(prior_display.read_text())
    write(out / "display_tolerance_sensitivity.json", display)
    preserved = all(sha(ROOT / path) == digest for path, digest in frozen.items())
    numerical_pass = replay["passed"] and all(
        r["numerical_verification"]["convergence"]["status"] == "passed"
        and (name == "landing" or r["numerical_verification"]["independent"]["passed"])
        for name, r in prior["phases"].items()
    )
    report = dict(
        status="evaluation_complete",
        nominal_scoped_physical_pass=all(not r["failed_checks"] for r in nominal.values()),
        numerical_pass=bool(numerical_pass),
        landing_input_replay=replay,
        nominal_physical_audits=nominal,
        campaigns=campaigns,
        observation_sensitivity=observations,
        display_sensitivity=display,
        display_source_sha256=sha(prior_display),
        nominal_mass_handoffs=prior["mass_handoffs"],
        preserved_prior_artifacts=preserved,
        scope="engineering sensitivity; no measured uncertainty distribution, no probability of "
        "success, no refit, no independent empirical validation",
        source_sha256=manifest,
    )
    report["phase_verdicts"] = {
        name: dict(
            nominal_model_checks="passed" if not nominal[name]["failed_checks"] else "failed",
            empirical_accuracy="not_certified_fitted_observations_only",
            sensitivity_cases=sum(
                "physical_audit" in c["phases"].get(name, {}) for c in campaigns.values()
            ),
            rejected_cases={
                label: c["phases"][name]
                for label, c in campaigns.items()
                if c["phases"].get(name, {}).get("status")
                in ("execution_error", "rejected_capacity")
            },
            robust_soft_landing=False if name == "landing" else None,
            scope="ideal actuators and specified direction/target; realistic probe crashes"
            if name == "landing"
            else "prescribed inputs; no actual control-law validation",
        )
        for name in NAMES
    }
    write(out / "summary.json", report)
    lines = [
        "# P4 frozen candidate sensitivity",
        "",
        "Engineering probes, not measured uncertainty bounds or Monte Carlo probabilities.",
        "",
        "| Case | Landing class | speed [m/s] | position [m] | failed physical checks |",
        "|---|---|---:|---:|---|",
    ]
    for label, campaign in campaigns.items():
        r = campaign["phases"].get("landing", {})
        lines.append(
            f"| {label} | {r.get('landing_class', 'not run')} | "
            f"{r.get('landing_speed_mps')} | {r.get('landing_position_error_m')} | "
            f"{r.get('physical_audit', {}).get('failed_checks', [])} |"
        )
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    fig, axes = plt.subplots(5, 2, figsize=(13, 17))
    for i, name in enumerate(NAMES):
        for j, key in enumerate(("altitude_m", "speed_m_s")):
            labels, values = [], []
            for label, campaign in campaigns.items():
                r = campaign["phases"].get(name, {})
                if "common_coverage" in r:
                    m = r["common_coverage"][key]
                    if m["probe"]["rmse"] is not None and m["nominal"]["rmse"] is not None:
                        labels.append(label)
                        values.append(m["probe"]["rmse"] - m["nominal"]["rmse"])
            ax = axes[i, j]
            ax.barh(labels, values, color=["#b34a3c" if v > 0 else "#247c73" for v in values])
            ax.axvline(0, color="black", lw=0.6)
            ax.set(title=f"{name}: {key}", xlabel="RMSE change on common coverage [m or m/s]")
            ax.tick_params(axis="y", labelsize=7)
            ax.grid(axis="x", alpha=0.2)
    fig.suptitle(
        "Fixed-candidate engineering probes (no refit)\n"
        "Coverage and physical/landing failures must be read with summary.json"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out / "sensitivity.png", dpi=150)
    plt.close(fig)
    print(
        "P4 archived",
        out,
        "nominal physical",
        report["nominal_scoped_physical_pass"],
        "numerical",
        numerical_pass,
        "prior artifacts preserved",
        preserved,
        flush=True,
    )
    if not (preserved and numerical_pass and report["nominal_scoped_physical_pass"]):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
