"""Run all five unfitted flight scenarios and archive before-calibration evidence.

Use --analyze-only to rebuild summaries from preserved runs without propagating again.
Baseline regression is a separate check_project2_segments.py --compare-flight invocation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from omegaconf import OmegaConf

from core.interfaces import TerminationReason
from plant.dynamics import build_plant
from validation.flight_before import audit_history, braking_screen

ROOT = Path(__file__).resolve().parents[1]
PHASES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]


def read(path):
    return json.loads(path.read_text())


def summarize_case(folder, name):
    cfg = OmegaConf.load(folder / "resolved_config.yaml")
    p = np.load(folder / "physics.npz")
    actuators = np.load(folder / "actuators.npz")
    kpi, comparison = read(folder / "kpi.json"), read(folder / "flight_comparison.json")
    audit = audit_history(p, actuators, cfg)
    report = dict(
        scenario=cfg.scenario.name,
        duration_s=float(p["t"][-1]),
        requested_duration_s=float(cfg.scenario.horizon.t_max),
        termination=TerminationReason(kpi["termination_reason"]).name,
        landing_class=kpi["landing_class"],
        landing_success=kpi["success"] if name == "landing" else None,
        horizon_reached=bool(abs(p["t"][-1] - cfg.scenario.horizon.t_max) <= cfg.plant_dt),
        final_altitude_m=float(p["altitude_m"][-1]),
        final_speed_mps=float(p["speed_mps"][-1]),
        comparison=comparison,
        physics_screen=audit,
        full_physics_acceptance="pending_independent_numerical_checks_and_mass_handoff_reconciliation",
    )
    report["artifact_sha256"] = {
        file: hashlib.sha256((folder / file).read_bytes()).hexdigest()
        for file in (
            "resolved_config.yaml",
            "physics.npz",
            "actuators.npz",
            "kpi.json",
            "flight_comparison.json",
            "flight_comparison.npz",
        )
    }
    if name == "landing":
        bundle = build_plant(cfg)
        env = bundle.native.environment_diagnostics(float(p["altitude_m"][0]))
        engine = cfg.plant.vehicle.engines
        thrust = float(engine.thrust_vac - env["pressure"] * engine.nozzle_exit_area)
        down = float(np.dot(p["v_ned"][0], p["local_down_ned"][0]))
        horizontal = float(np.linalg.norm(p["v_ned"][0] - down * p["local_down_ned"][0]))
        report["initial_braking_screen"] = braking_screen(
            float(p["altitude_m"][0]),
            down,
            horizontal,
            float(p["mass"][0]),
            thrust,
            float(np.linalg.norm(p["gravity_ned"][0])),
            float(cfg.gnc.guidance.attitude_tilt_max_deg),
            float(p["main_propellant"][0]),
            float(engine.isp),
        )
        report["landing_kpi"] = kpi
        report["solver"] = read(folder / "solver_stats.json")
        report["control"] = read(folder / "control_stats.json")
    (folder / "before_assessment.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/verification/public_flight_comparison/step3_20261107/before",
    )
    parser.add_argument("--analyze-only", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    sources = (
        sorted((ROOT / "configs").rglob("*.yaml"))
        + sorted((ROOT / "src").rglob("*.py"))
        + [ROOT / "src/native/plant.cpp", ROOT / "scripts/run_single.py"]
    )
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    manifest = out / "run_sources.json"
    if not args.analyze_only:
        if manifest.exists():
            raise FileExistsError(
                "Preserve existing before runs; choose a new --output or --analyze-only"
            )
        manifest.write_text(json.dumps(hashes, indent=2) + "\n")
        for name in PHASES:
            folder = out / name
            folder.mkdir(parents=True, exist_ok=True)
            command = [
                sys.executable,
                str(ROOT / "scripts/run_single.py"),
                f"scenario=flight_comparison_{name}",
                f"hydra.run.dir={folder}",
                f"run.run_id=crs11_before_{name}",
            ]
            (folder / "command.json").write_text(json.dumps(command, indent=2) + "\n")
            print(f"Running unfitted {name}", flush=True)
            with (folder / "run.log").open("w") as log:
                completed = subprocess.run(
                    command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False
                )
            (folder / "exit_code.json").write_text(json.dumps(completed.returncode) + "\n")
    reports = {}
    for name in PHASES:
        folder = out / name
        if read(folder / "exit_code.json") != 0:
            reports[name] = dict(status="execution_error", log=str(folder / "run.log"))
            continue
        reports[name] = summarize_case(folder, name)
    handoffs = []
    for left, right in zip(PHASES[1:-1], PHASES[2:], strict=True):
        if "physics_screen" not in reports[left] or "physics_screen" not in reports[right]:
            continue
        a, b = (
            reports[left]["physics_screen"]["diagnostics"],
            reports[right]["physics_screen"]["diagnostics"],
        )
        handoffs.append(
            dict(
                before=left,
                after=right,
                preceding_final_mass_kg=a["final_mass_kg"],
                next_initial_mass_kg=b["initial_mass_kg"],
                difference_kg=b["initial_mass_kg"] - a["final_mass_kg"],
                status="independent_initializations_not_a_continuous_flight",
            )
        )
    summary = dict(
        status="before_runs_recorded"
        if all("comparison" in r for r in reports.values())
        else "execution_errors_recorded",
        calibrated=False,
        empirical_accuracy_verdict=None,
        run_sources_file="run_sources.json",
        source_changes_since_run=[k for k, v in hashes.items() if read(manifest).get(k) != v],
        phases=reports,
        mass_handoffs=handoffs,
    )
    summary["analysis_source_sha256"] = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (Path(__file__), ROOT / "src/validation/flight_before.py")
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    lines = [
        "# CRS-11 before calibration",
        "",
        "These are unfitted results, not validation passes.",
        "",
        "| Phase | Duration [s] | Termination | h RMSE [m] | speed RMSE [m/s] | "
        "h/speed coverage [s] | Logged screens |",
        "|---|---:|---|---:|---:|---|---|",
    ]
    fig, axes = plt.subplots(5, 2, figsize=(12, 15))
    for row, name in enumerate(PHASES):
        r = reports[name]
        if "comparison" not in r:
            lines.append(f"| {name} | — | execution error | — | — | — | — |")
            continue
        m = r["comparison"]["metrics"]["requested_overlap"]
        h, v = m["altitude_m"], m["speed_m_s"]
        lines.append(
            f"| {name} | {r['duration_s']:.3f} | {r['termination']} | {h['rmse']} | {v['rmse']} | "
            f"{h['covered_duration_s']:.3f}/{v['covered_duration_s']:.3f} | "
            f"{r['physics_screen']['status']} |"
        )
        arrays = np.load(out / name / "flight_comparison.npz")
        for col, (key, scale, label) in enumerate(
            [("altitude_m", 1000, "Altitude [km]"), ("speed_m_s", 1, "Speed [m/s]")]
        ):
            ax = axes[row, col]
            t, eligible = arrays["reference_time_s"], arrays[f"{key}_score_eligible"]
            ax.plot(
                t[eligible],
                arrays[f"reference_{key}"][eligible] / scale,
                ".",
                ms=2,
                label="Scored observation",
            )
            ax.plot(
                t[~eligible],
                arrays[f"reference_{key}"][~eligible] / scale,
                "x",
                ms=2,
                color="0.7",
                label="Excluded/no overlap",
            )
            ax.plot(t, arrays[f"simulation_{key}"] / scale, ".", ms=1, label="Before simulation")
            ax.set(title=f"{name}: {r['termination']}", xlabel="Reference T+ [s]", ylabel=label)
            ax.legend(fontsize=7)
            ax.grid(alpha=0.25)
    fig.suptitle(
        "CRS-11 unfitted comparison: assumed datum/frame\n"
        "No noise injection or parameter calibration"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out / "before_comparison.png", dpi=160)
    plt.close(fig)
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines), flush=True)
    if summary["status"] == "execution_errors_recorded":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
