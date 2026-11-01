"""Controlled LCvx planning-drag sweep for RLV-Sim-2026.

Copy this file to the repository root, activate your project virtual environment,
then run: python rlv_drag_sweep.py --values 0.1 0.25 0.5 1.0
No simulator source code is modified.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BASE_ARGS = [
    "plant/vehicle=falcon9_octaweb",
    "scenario=reentry_to_landing",
    "environment/atmosphere=ussa_poly",
    "gnc.scheduler.reentry_end_altitude=42500",
    "gnc.scheduler.switch_altitude=5000",
    "gnc.scheduler.powered_brake=true",
    "gnc.scheduler.brake_end_altitude=2000",
    "gnc.guidance.convex_replan=true",
    "plant/actuator=ideal",
]
HEADERS = [
    "drag_cd", "return_code", "kpi_success", "landing_class",
    "touchdown_speed_mps", "remaining_main_propellant_kg", "peak_q_pa",
    "brake_ignited", "landing_ignited", "n_solves", "n_solver_infeasible",
    "n_postcheck_rejected", "n_drag_refine", "n_drag_refine_feasible",
    "n_feasible_total", "output_dir",
]


def summarize(folder: Path, coefficient: float, exit_code: int | str = "") -> dict:
    report = {key: "" for key in HEADERS}
    report.update(drag_cd=coefficient, return_code=exit_code, output_dir=str(folder))

    trajectory = folder / "trajectory.csv"
    if trajectory.is_file():
        with trajectory.open(newline="", encoding="utf-8-sig") as handle:
            samples = list(csv.DictReader(handle))
        if samples:
            last = samples[-1]
            report["touchdown_speed_mps"] = round(float(last["speed_mps"]), 4)
            report["remaining_main_propellant_kg"] = round(float(last["main_propellant"]), 2)
            report["peak_q_pa"] = round(max(float(r["aero_q_dyn"]) for r in samples), 1)
            throttle_cols = [f"throttle{i}" for i in range(9)]
            for phase, key in [(2, "brake_ignited"), (3, "landing_ignited")]:
                selected = (r for r in samples if int(float(r["phase"])) == phase)
                report[key] = any(
                    any(float(r[col]) > 1e-5 for col in throttle_cols if col in r)
                    for r in selected
                )

    stats_path = folder / "solver_stats.json"
    if stats_path.is_file():
        stats = json.loads(stats_path.read_text(encoding="utf-8"))
        solves = stats.get("per_solve", [])
        refines = [s for s in solves if "drag_refine" in s.get("kind", "")]
        report.update(
            n_solves=stats.get("n_solves", ""),
            n_solver_infeasible=stats.get("n_solver_infeasible", ""),
            n_postcheck_rejected=stats.get("n_postcheck_rejected_solves", ""),
            n_drag_refine=len(refines),
            n_drag_refine_feasible=sum(s.get("feasible") is True for s in refines),
            n_feasible_total=sum(s.get("feasible") is True for s in solves),
        )

    kpi_path = folder / "kpi.json"
    if kpi_path.is_file():
        kpi = json.loads(kpi_path.read_text(encoding="utf-8"))
        report["kpi_success"] = kpi.get("success", "")
        report["landing_class"] = kpi.get("landing_class", "")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--values", type=float, nargs="+", default=[0.1, 0.25],
                        help="Planning drag coefficients to test (default: 0.1 0.25)")
    parser.add_argument("--inspect", type=Path, default=None,
                        help="Only summarize a completed results folder, no simulation run")
    args = parser.parse_args()

    if args.inspect is not None:
        folder = args.inspect.resolve()
        print(json.dumps(summarize(folder, float("nan")), indent=2))
        return

    if not (ROOT / "scripts" / "run_single.py").is_file():
        parser.error("Place this script at the RLV-Sim repository root next to 'scripts'.")

    reports = []
    for coefficient in args.values:
        if not (0.0 <= coefficient <= 5.0):
            parser.error("Provide nonnegative, reasonable planning drag coefficients (0 to 5).")
        label = f"cd_{coefficient:.3f}".replace(".", "p")
        output = ROOT / "results" / "drag_sweep" / label
        output.mkdir(parents=True, exist_ok=True)
        cmd = [sys.executable, "scripts/run_single.py", *BASE_ARGS,
               f"gnc.guidance.convex_drag_cd={coefficient}",
               f"hydra.run.dir=results/drag_sweep/{label}"]
        print(f"\nRunning drag Cd={coefficient} ...", flush=True)
        with (output / "console.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                    check=False)
        report = summarize(output, coefficient, result.returncode)
        reports.append(report)
        print("  touchdown speed:", report["touchdown_speed_mps"], "m/s")
        print("  brake/landing ignition:", report["brake_ignited"], report["landing_ignited"])
        print("  drag refinement feasible:", report["n_drag_refine_feasible"],
              "/", report["n_drag_refine"])
        if result.returncode != 0:
            print("  Nonzero return code: check", output / "console.log")

    summary_file = ROOT / "results" / "drag_sweep" / "summary.csv"
    with summary_file.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=HEADERS)
        writer.writeheader()
        writer.writerows(reports)
    print("\nSummary:", summary_file)
    print("Open each case's solver_stats.json and console.log for detailed failures.")


if __name__ == "__main__":
    main()
