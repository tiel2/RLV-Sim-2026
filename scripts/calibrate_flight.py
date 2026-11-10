"""Fit bounded inputs/parameters and run an LCvx/PD closed-loop calibrated landing.

Usage: python scripts/calibrate_flight.py --write-configs
Original flight_reconstructed_* and physics/closed-loop landing regressions are preserved.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf
from scipy.optimize import least_squares

from validation.flight_calibration import (
    ObservationResidual,
    ascent_spec,
    closed_loop_landing_cfg,
    physical_screen,
    propagate_candidates,
    recovery_spec,
)
from validation.flight_comparison_policy import masked_statistics
from validation.flight_reference import compare_flight_reference

ROOT = Path(__file__).resolve().parents[1]
NAMES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]


class EvaluationBudget(Exception):
    pass


def fit(label, spec, observations, recipe, output, *, couple_mass=False, resume=False, source=None):
    initial, bounds, candidates, prior, *extra = spec
    search_path = (source or output) / f"{label}_search.json"
    if resume and search_path.exists():
        report = json.loads(search_path.read_text())
        x = np.asarray(report["best"]["parameters"])
        if np.any(x < bounds[0]) or np.any(x > bounds[1]):
            raise ValueError(f"{label}: archived parameters outside current bounds")
        runs = propagate_candidates(candidates(x), couple_mass=couple_mass)
        if not all(physical_screen(c, r) for c, r in runs):
            raise ValueError(f"{label}: archived candidate fails physical screen")
        data = np.concatenate([o(r) for o, (_, r) in zip(observations, runs, strict=True)])
        if not np.isclose(
            data @ data / (2 * len(runs)), report["best"]["data_mse"], rtol=1e-8, atol=1e-10
        ):
            raise ValueError(f"{label}: archived objective differs from current rollout/mask")
        report["resumed_from_verified_search"] = True
        if source is not None:
            report["source_search_file"] = str(search_path)
            (output / f"{label}_search.json").write_text(json.dumps(report, indent=2) + "\n")
            for cfg, _ in runs:
                OmegaConf.save(cfg, output / f"best_{cfg.scenario.reference.segment}.yaml")
        print("resume", label, flush=True)
        return runs, report
    terminal = extra[0] if extra else lambda runs: np.empty(0)
    best, trace = {}, []
    size = sum(o.size for o in observations) + len(prior(initial))
    if extra:
        size += 2
    opt = recipe.optimizer
    budget = min(opt.max_evaluations, opt.get(f"{label}_max_evaluations", opt.max_evaluations))

    def objective(x):
        if len(trace) >= budget:
            raise EvaluationBudget
        try:
            runs = propagate_candidates(candidates(x), couple_mass=couple_mass)
            data = np.concatenate([o(r) for o, (_, r) in zip(observations, runs, strict=True)])
            residual = np.r_[data, prior(x), terminal(runs)]
            if not np.all(np.isfinite(residual)):
                raise ValueError("nonfinite residual")
            screened = all(physical_screen(c, r) for c, r in runs)
            if not screened:
                raise ValueError("candidate failed horizon/fuel/body-table screen")
            objective_value = float(residual @ residual)
            status = "screened" if screened else "outside_physics_screen"
            # Rank with the exact regularized objective minimized by least_squares.
            if screened and objective_value < best.get("objective", np.inf):
                best.update(
                    objective=objective_value,
                    data_mse=float(data @ data / (2 * len(runs))),
                    parameters=x.tolist(),
                    runs=runs,
                )
                for cfg, _ in runs:
                    name = str(cfg.scenario.reference.segment)
                    OmegaConf.save(cfg, output / f"best_{name}.yaml")
        except ValueError as error:
            # A rejected rollout never earns an overlap-only score.
            residual = np.full(size, 1e4)
            objective_value, status = float(residual @ residual), str(error)
        trace.append(dict(parameters=x.tolist(), objective=objective_value, status=status))
        if len(trace) % 25 == 0:
            print(label, len(trace), "best data MSE", best.get("data_mse"), flush=True)
        return residual

    success, reason = False, ""
    try:
        result = least_squares(
            objective,
            initial,
            bounds=bounds,
            x_scale="jac",
            diff_step=opt.diff_step,
            ftol=opt.ftol,
            xtol=opt.xtol,
            gtol=opt.gtol,
            max_nfev=budget,
        )
        success, reason = bool(result.success), str(result.message)
    except EvaluationBudget:
        reason = "total rollout evaluation budget reached (including numerical Jacobians)"
    if not best:
        raise RuntimeError(f"{label}: no physically screened candidate")
    report = dict(
        best={k: v for k, v in best.items() if k != "runs"},
        trace=trace,
        optimizer_success=success,
        stop_reason=reason,
        selection="lowest regularized objective among screened evaluations",
        parameter_identifiability="not established; bounds and priors assumed",
    )
    (output / f"{label}_search.json").write_text(json.dumps(report, indent=2) + "\n")
    print(label, reason, "data MSE", best["data_mse"], flush=True)
    return best["runs"], report


def export(name, cfg, output, write):
    # Overlay the observed-state baseline, preserving new model/input values explicitly.
    parent = "flight_reconstructed_" + name
    fragment = dict(
        defaults=[parent, "_self_"],
        scenario=dict(
            name=f"flight_calibrated_{name}",
            description="Native 6-DOF held-in input/parameter identification",
            ic=OmegaConf.to_container(cfg.scenario.ic),
            horizon=OmegaConf.to_container(cfg.scenario.horizon),
            input_mode=str(cfg.scenario.input_mode),
            reference=OmegaConf.to_container(cfg.scenario.reference),
            reconstruction=dict(
                parameter_source="configs/analysis/flight_calibration.yaml",
                role="held_in_calibration_not_independent_validation",
            ),
        ),
        plant=dict(aero=dict(coeffs=OmegaConf.to_container(cfg.plant.aero.coeffs))),
    )
    if name == "landing":
        fragment["defaults"].insert(1, {"override /plant/actuator": str(cfg.plant.actuator.model)})
        fragment["gnc"] = dict(guidance=OmegaConf.to_container(cfg.gnc.guidance))
        fragment["scenario"]["description"] = (
            "Calibrated initial state; LCvx/PD closed-loop landing"
        )
    else:
        fragment["scenario"]["inputs"] = OmegaConf.to_container(cfg.scenario.inputs)
    fragment["scenario"]["reference"]["role"] = "held_in_native_dynamics_calibration_v1"
    text = "# @package _global_\n# Generated by scripts/calibrate_flight.py.\n"
    text += OmegaConf.to_yaml(fragment)
    (output / f"flight_calibrated_{name}.yaml").write_text(text)
    if write:
        (ROOT / f"configs/scenario/flight_calibrated_{name}.yaml").write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/verification/flight_calibration_20261110"
    )
    parser.add_argument("--write-configs", action="store_true")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Reuse completed searches after checking recipe, bounds and rollout loss",
    )
    parser.add_argument("--max-evaluations", type=int)
    parser.add_argument(
        "--reuse-input-searches",
        type=Path,
        help="Recheck and reuse completed ascent/recovery searches from this directory",
    )
    parser.add_argument("--phase", choices=["all", "ascent", "recovery", "landing"], default="all")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and not args.resume:
        raise FileExistsError(f"preserve previous results; choose a new --output: {output}")
    output.mkdir(parents=True, exist_ok=args.resume)
    recipe = OmegaConf.load(ROOT / "configs/analysis/flight_calibration.yaml")
    if args.max_evaluations is not None:
        recipe.optimizer.max_evaluations = args.max_evaluations
    if recipe.optimizer.max_evaluations < 1:
        raise ValueError("max evaluations must be positive")
    if args.resume:
        archived_recipe = OmegaConf.load(output / "recipe.yaml")
        if OmegaConf.to_container(archived_recipe) != OmegaConf.to_container(recipe):
            raise ValueError("resume requires the archived recipe and evaluation budget")
    else:
        OmegaConf.save(recipe, output / "recipe.yaml")
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        cfgs = [
            compose(config_name="config", overrides=[f"scenario=flight_reconstructed_{n}"])
            for n in NAMES
        ]
        before = propagate_candidates(cfgs)
        observations = [
            ObservationResidual(
                r,
                c.scenario.reference,
                recipe.objective_scales,
                include_initialization_support=True,
            )
            for c, r in before
        ]
        runs, searches = {}, {}
        if args.phase in ("all", "ascent"):
            fitted, searches["ascent"] = fit(
                "ascent",
                ascent_spec(cfgs[0], recipe),
                observations[:1],
                recipe,
                output,
                resume=args.resume or args.reuse_input_searches is not None,
                source=args.reuse_input_searches,
            )
            runs["ascent"] = fitted[0]
        if args.phase in ("all", "recovery"):
            fitted, searches["recovery"] = fit(
                "recovery",
                recovery_spec(cfgs[1:4], recipe),
                observations[1:4],
                recipe,
                output,
                couple_mass=True,
                resume=args.resume or args.reuse_input_searches is not None,
                source=args.reuse_input_searches,
            )
            runs.update(zip(NAMES[1:4], fitted, strict=True))
        if args.phase in ("all", "landing"):
            c, r = runs.get("powered_descent", before[3])
            cfg = closed_loop_landing_cfg(cfgs[4], c, r, recipe)
            _, result = propagate_candidates([cfg])[0]
            if not physical_screen(cfg, result):
                raise RuntimeError("calibrated LCvx/PD landing failed soft-contact/physics checks")
            runs["landing"] = (cfg, result)
            searches["landing"] = dict(
                method="LCvx boundary solve with live-state tracking and PD attitude control",
                success=bool(result.success),
                landing_class=result.landing_class,
                duration_s=float(result.t[-1]),
                contact_speed_mps=float(result.landing_speed),
                position_error_m=float(result.landing_position_error),
                fuel_remaining_kg=float(result.fuel_margin),
                solve_count=result.aux["solver"]["n_solves"],
            )
            OmegaConf.save(cfg, output / "best_landing.yaml")
    reports = {}
    fig, axes = plt.subplots(len(runs), 2, figsize=(12, 3 * len(runs)), squeeze=False)
    for row, (name, (cfg, result)) in enumerate(runs.items()):
        previous = before[NAMES.index(name)]
        old_report, old = compare_flight_reference(previous[1], previous[0].scenario.reference)
        cfg.scenario.reference.role = "held_in_native_dynamics_calibration_v1"
        report, arrays = compare_flight_reference(result, cfg.scenario.reference)
        reports[name] = dict(
            before=old_report,
            after=report,
            physics_screen=physical_screen(cfg, result),
            initial_mass_kg=float(result.m[0]),
            final_mass_kg=float(result.m[-1]),
            final_altitude_m=float(result.aux["per_tick"][-1]["altitude"]),
            final_speed_mps=float(np.linalg.norm(result.v_ned[-1])),
            duration_s=float(result.t[-1]),
        )
        np.savez_compressed(output / f"{name}_comparison.npz", **arrays)
        np.savez_compressed(
            output / f"{name}_trajectory.npz",
            t=result.t,
            r_ned=result.r_ned,
            v_ned=result.v_ned,
            q_bn=result.q_bn,
            w_body=result.w_body,
            mass=result.m,
        )
        common = {}
        for key, _ in observations[0].channels:
            mask = old[f"{key}_score_eligible"] & arrays[f"{key}_score_eligible"]
            common[key] = {
                label: masked_statistics(
                    data["reference_time_s"], data[f"error_{key}"], mask, max_gap_s=1.1
                )[0]
                for label, data in [("before", old), ("after", arrays)]
            }
        reports[name]["common_coverage_comparison"] = common
        export(name, cfg, output, args.write_configs)
        for col, (key, _) in enumerate(observations[0].channels):
            ax = axes[row, col]
            time = arrays["reference_time_s"]
            observed = arrays[f"reference_{key}"]
            eligible = arrays[f"{key}_score_eligible"]
            ax.plot(
                time[~eligible],
                observed[~eligible],
                ".",
                ms=2,
                color="0.7",
                label="Excluded from score",
            )
            ax.plot(
                time[eligible],
                observed[eligible],
                ".",
                ms=2,
                color="black",
                label="Webcast (scored)",
            )
            ax.plot(old["reference_time_s"], old[f"simulation_{key}"], label="Previous")
            ax.plot(arrays["reference_time_s"], arrays[f"simulation_{key}"], label="Calibrated")
            ax.set(
                title=name.replace("_", " "),
                xlabel="T+ [s]",
                ylabel="Altitude [m]" if key == "altitude_m" else "Speed [m/s]",
            )
            ax.grid(alpha=0.25)
            ax.legend(fontsize=8)
    fig.suptitle("Native 6-DOF input/parameter identification (held-in fit)")
    fig.tight_layout()
    fig.savefig(output / "comparison.png", dpi=160)
    fig.savefig(output / "comparison.pdf")
    plt.close(fig)
    source_paths = [
        ROOT / "configs/analysis/flight_calibration.yaml",
        Path(__file__),
        ROOT / "src/validation/flight_calibration.py",
    ]
    summary = dict(
        phases=reports,
        searches={k: {p: v for p, v in r.items() if p != "trace"} for k, r in searches.items()},
        role="held_in_reconstruction_and_closed_loop_landing_not_independent_prediction",
        fit_mask=recipe.fit_mask,
        evaluation_mask=recipe.evaluation_mask,
        state_handoffs="only mass/gas; position/velocity/attitude independently initialized",
        source_sha256={
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source_paths
        },
    )
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print("Results:", output, flush=True)


if __name__ == "__main__":
    main()
