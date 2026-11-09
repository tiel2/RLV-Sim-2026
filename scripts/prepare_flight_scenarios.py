"""Generate/check five CRS-11 scenario overlays and initialization-use masks.

No trajectory simulation or parameter calibration is performed.
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

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.open_loop import PrescribedInputs
from sim.schedule import build_schedule
from validation.flight_observations import coverage, prepare_observations
from validation.flight_scenarios import (
    initialization_mask,
    reconstruct_initial_states,
    scenario_fragment,
)

ROOT = Path(__file__).resolve().parents[1]


def inspect_configuration(cfg):
    """Build the actual plant and check its initial state/schedule, without stepping."""
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
    _, aux = bundle.native.create_session(state).observe()
    h = cfg.scenario.horizon
    prescribed = cfg.scenario.input_mode == "prescribed"
    schedule = build_schedule(
        cfg.plant_dt,
        h.t_max,
        h.input_dt if prescribed else h.ctrl_dt,
        None if prescribed else h.nav_dt,
        None if prescribed else h.guidance_dt,
        h.log_dt,
    )
    budget = None
    if prescribed:
        inputs = PrescribedInputs(cfg.scenario.inputs, bundle.params, float(h.t_max))
        epochs = list(inputs.rows)
        integral = 0.0
        for i, epoch in enumerate(epochs):
            end = epochs[i + 1] * float(cfg.plant_dt) if i + 1 < len(epochs) else float(h.t_max)
            integral += float(np.sum(inputs.rows[epoch]["command"].throttle)) * (
                end - epoch * float(cfg.plant_dt)
            )
        engines = cfg.plant.vehicle.engines
        budget = integral * engines.thrust_vac / (engines.isp * 9.80665)
        if budget > float(aux["main_propellant"]):
            raise ValueError("prescribed constant-flow planning screen exceeds initial fuel")
    else:
        from gnc.build import build_gnc, build_nav

        build_gnc(cfg, bundle.params)
        build_nav(cfg)
    vehicle = cfg.plant.vehicle
    fuel = float(aux["main_propellant"])
    masses = fuel * np.array([vehicle.tanks.of_ratio, 1]) / (1 + vehicle.tanks.of_ratio)
    capacity = (
        np.pi * np.array(vehicle.tanks.radius) ** 2 * vehicle.tanks.length * vehicle.tanks.density
    )
    if fuel <= 0 or np.any(masses > capacity) or cfg.scenario.ic.m > vehicle.mass.m_wet:
        raise ValueError("initial fuel/tank/wet capacity screen failed")
    return dict(
        status="initialized_not_propagated",
        radial_altitude_m=float(aux["altitude"]),
        main_propellant_kg=fuel,
        tank_fill_fraction=(masses / capacity).tolist(),
        x_cg_m=float(aux["x_cg"]),
        inertia_diag_kg_m2=np.asarray(aux["inertia_diag"]).tolist(),
        nominal_prescribed_consumption_kg=budget,
        budget_role="command-flow planning screen; not integrated flow or a landing guarantee",
        schedule=schedule.metadata(),
        actuator_model=str(cfg.plant.actuator.model),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--write-configs", action="store_true", help="write generated source YAML overlays"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/verification/public_flight_comparison/initialization",
    )
    args = parser.parse_args()
    path = ROOT / "configs/analysis/public_flight_comparison.yaml"
    policy = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    quality, arrays = prepare_observations(ROOT / "data/reference/crs11", policy)
    states, intervals = reconstruct_initial_states(policy, arrays)
    reserved = initialization_mask(arrays["reference_time_s"], intervals)
    report = dict(
        status="step_2_prepared_not_calibrated",
        policy=policy,
        snapshot=quality["snapshot"],
        initialization_exclusions=intervals,
        phases={},
    )
    sources = [
        path,
        Path(__file__),
        ROOT / "src/validation/flight_scenarios.py",
        ROOT / "src/validation/flight_comparison_policy.py",
        ROOT / "src/validation/flight_observations.py",
        ROOT / "src/validation/segment_spec.py",
        ROOT / "src/validation/flight_reference.py",
    ]
    args.output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(5, 2, figsize=(12, 15))
    lines = [
        "# Step 2: observation-informed initialization",
        "",
        "No full trajectory run or fit. Coverage excludes all inspected IC-support windows.",
        "",
        "| Phase | h0 [m] | speed0 [m/s] | N/E/D velocity [m/s] | "
        "Altitude/speed evaluation time [s] |",
        "|---|---:|---:|---|---|",
    ]
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        for index, (name, state) in enumerate(states.items()):
            spec = policy["reconstruction"]["phases"][name]
            base = compose(config_name="config", overrides=[f"scenario={spec['base_scenario']}"])
            fragment = scenario_fragment(name, policy, state, base)
            config_path = ROOT / f"configs/scenario/flight_comparison_{name}.yaml"
            content = (
                "# @package _global_\n"
                "# Generated by scripts/prepare_flight_scenarios.py; unfitted.\n"
                + OmegaConf.to_yaml(fragment)
            )
            if args.write_configs:
                config_path.write_text(content)
            elif not config_path.exists() or config_path.read_text() != content:
                raise ValueError(
                    f"generated overlay differs: {config_path}; review or use --write-configs"
                )
            sources.append(config_path)
            cfg = compose(config_name="config", overrides=[f"scenario=flight_comparison_{name}"])
            checks = inspect_configuration(cfg)
            resolved = OmegaConf.to_yaml(cfg, resolve=True)
            (args.output / f"flight_comparison_{name}.resolved.yaml").write_text(resolved)
            phase = dict(
                **state,
                initial_checks=checks,
                future_parameters=fragment["scenario"]["reconstruction"]["future_parameters"],
                resolved_config_sha256=hashlib.sha256(resolved.encode()).hexdigest(),
                evaluation_coverage={},
            )
            selected = arrays["phase_index"] == index
            for col, (channel, field, label) in enumerate(
                [("altitude", "altitude_m", "Altitude [m]"), ("speed", "speed_mps", "Speed [m/s]")]
            ):
                eligible = arrays[f"{channel}_eligible"] & ~reserved
                stats = coverage(
                    arrays["reference_time_s"],
                    arrays[field],
                    eligible,
                    selected,
                    max_gap_s=policy["quality"]["max_gap_s"],
                )
                phase["evaluation_coverage"][channel] = stats
                arrays[f"{name}_{channel}_evaluation_eligible"] = eligible & selected
                ax = axes[index, col]
                for mask, marker, color, legend in [
                    (selected & eligible, ".", "C0", "Evaluation / future fit"),
                    (selected & reserved, "x", "C1", "Used for initialization"),
                    (selected & ~arrays[f"{channel}_eligible"], ".", "0.6", "Quality excluded"),
                ]:
                    ax.plot(
                        arrays["reference_time_s"][mask],
                        arrays[field][mask],
                        marker,
                        ms=2,
                        color=color,
                        label=legend,
                    )
                ax.set(title=name, xlabel="Reference T+ [s]", ylabel=label)
                ax.grid(alpha=0.25)
                ax.legend(fontsize=7)
            report["phases"][name] = phase
            velocity = ", ".join(f"{v:.3f}" for v in state["initial_state"]["v_ned"])
            c = phase["evaluation_coverage"]
            lines.append(
                f"| {name} | {state['altitude_observation']['value']:.3f} | "
                f"{state['speed_observation']['value']:.3f} | {velocity} | "
                f"{c['altitude']['covered_duration_s']:.3f} / "
                f"{c['speed']['covered_duration_s']:.3f} |"
            )
    arrays["initialization_used"] = reserved
    handoffs = []
    for before, after in zip(list(states)[1:-1], list(states)[2:], strict=True):
        left, right = report["phases"][before], report["phases"][after]
        nominal_end_mass = (
            left["initial_state"]["m"] - left["initial_checks"]["nominal_prescribed_consumption_kg"]
        )
        handoffs.append(
            dict(
                before=before,
                after=after,
                nominal_end_mass_kg=nominal_end_mass,
                next_initial_mass_kg=right["initial_state"]["m"],
                difference_kg=right["initial_state"]["m"] - nominal_end_mass,
                status="independent_budgets_require_reconciliation_before_physical_acceptance",
                meaning="command-flow screen; actual actuator/fuel histories not yet integrated",
            )
        )
    report["mass_handoff_planning_diagnostics"] = handoffs
    lines += [
        "",
        "Mass budgets are independent initial assumptions. Adjacent-phase handoffs require",
        "reconciliation with integrated fuel use before final physical acceptance; see JSON.",
    ]
    np.savez_compressed(args.output / "evaluation_masks.npz", **arrays)
    report["source_sha256"] = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources
    }
    (args.output / "initialization_report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    (args.output / "summary.md").write_text("\n".join(lines) + "\n")
    fig.suptitle(
        "CRS-11 step 2: initialization use vs. descriptive evaluation\n"
        "Provisional datum/frame; no simulation or calibration"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(args.output / "evaluation_masks.png", dpi=160)
    plt.close(fig)
    print("\n".join(lines))
    print(f"Prepared and initialized five scenarios: {args.output}")


if __name__ == "__main__":
    main()
