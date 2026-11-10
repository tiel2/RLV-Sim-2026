"""Archive calibrated scenarios, check physical bookkeeping and independent integration.

Generates ordinary run_single artifacts, including replay.json, without fitting again.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from omegaconf import OmegaConf

from sim.run import run_configured
from validation.ascent import compare, reference
from validation.convergence import check_convergence, compare_results
from validation.flight_before import audit_history

ROOT = Path(__file__).resolve().parents[1]
NAMES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]
LIMITS = dict(r_ned=0.01, v_ned=0.001, w_body=1e-6, q_bn=1e-6, m=0.001)


def closed_loop_checks(cfg, folder):
    """Contact accuracy and refinement of the actual guidance/controller/actuator loop."""
    limits = dict(
        position_m=0.1,
        velocity_mps=0.05,
        attitude_rad=0.001,
        mass_kg=0.05,
        dynamic_pressure_pa=10.0,
        terminal_time_s=0.01,
    )
    runs = []
    for dt in [0.002, 0.001, 0.0005]:
        local = OmegaConf.create(OmegaConf.to_container(cfg, resolve=True))
        local.plant_dt = dt
        _, result = run_configured(local, record_telemetry=False)
        runs.append(result)
    pairs = []
    for coarse, fine in zip(runs[:-1], runs[1:], strict=True):
        row = compare_results(coarse, fine, float(cfg.scenario.horizon.log_dt))
        row["passed"] = (
            coarse.success
            and fine.success
            and all(row["errors"][key] <= value for key, value in limits.items())
        )
        pairs.append(row)
    kpi = json.loads((folder / "kpi.json").read_text())
    return dict(
        independent=dict(
            status="not_applied",
            passed=None,
            reason="Prescribed-input independent reference cannot verify feedback dynamics",
        ),
        convergence=dict(
            status="passed" if all(p["passed"] for p in pairs) else "failed",
            steps_s=[0.002, 0.001, 0.0005],
            limits=limits,
            comparisons=pairs,
            all_soft_landings=all(r.success and r.landing_class == "soft" for r in runs),
            contact_speeds_mps=[float(r.landing_speed) for r in runs],
        ),
        contact=dict(kpi=kpi, passed=kpi["success"] and kpi["landing_class"] == "soft"),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/verification/flight_calibration_closed_loop_20261110_verified",
    )
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        raise FileExistsError(f"preserve previous results; choose a new --output: {out}")
    out.mkdir(parents=True)
    reports, previous = {}, None
    paths = [
        *list((ROOT / "configs").rglob("*.yaml")),
        *list((ROOT / "src").rglob("*.py")),
        ROOT / "src/native/plant.cpp",
        Path(__file__),
    ]
    sources = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (out / "sources.json").write_text(json.dumps(sources, indent=2) + "\n")
    for name in NAMES:
        folder = out / name
        folder.mkdir()
        command = [
            sys.executable,
            str(ROOT / "scripts/run_single.py"),
            f"scenario=flight_calibrated_{name}",
            f"hydra.run.dir={folder}",
        ]
        print("archive", name, flush=True)
        (folder / "command.json").write_text(json.dumps(command, indent=2) + "\n")
        with (folder / "run.log").open("w") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        cfg = OmegaConf.load(folder / "resolved_config.yaml")
        with np.load(folder / "physics.npz") as stored:
            p = dict(stored)
        with np.load(folder / "actuators.npz") as stored:
            actuators = dict(stored)
        screens = audit_history(p, actuators, cfg)
        contact = None
        if cfg.scenario.input_mode == "closed_loop":
            print("closed-loop contact and step refinement", name, flush=True)
            checks = closed_loop_checks(cfg, folder)
            independent, convergence, contact = (
                checks["independent"],
                checks["convergence"],
                checks["contact"],
            )
            numerical_pass = contact["passed"]
        else:
            print("independent integration", name, flush=True)
            expected = reference(
                OmegaConf.to_container(cfg, resolve=True),
                p["t"],
                rtol=1e-10,
                allow_neutral_fins=True,
            )
            actual = SimpleNamespace(
                **{k: p[k] for k in ("r_ned", "v_ned", "q_bn", "w_body")}, m=p["mass"]
            )
            errors = compare(actual, expected)
            independent = dict(
                errors=errors,
                limits=LIMITS,
                passed=all(errors[k] <= tol for k, tol in LIMITS.items()),
                method="independent inertial-frame DOP853 at rtol=1e-10",
            )
            print("step refinement", name, flush=True)
            convergence = check_convergence(cfg, steps=(0.001, 0.0005, 0.00025))
            numerical_pass = independent["passed"]
        comparison = json.loads((folder / "flight_comparison.json").read_text())
        handoff = None
        if name != "ascent":
            if previous is not None:
                handoff = dict(
                    mass_difference_kg=float(p["mass"][0] - previous["mass"][-1]),
                    gas_difference_kg=float(p["rcs_fuel"][0] - previous["rcs_fuel"][-1]),
                )
                handoff["passed"] = all(abs(v) <= 1e-6 for v in handoff.values())
            previous = p
        row = dict(
            physics_screens=screens,
            independent=independent,
            convergence=convergence,
            contact=contact,
            mass_handoff=handoff,
            comparison=comparison,
            final_altitude_m=float(p["altitude_m"][-1]),
            final_speed_mps=float(p["speed_mps"][-1]),
        )
        row["passed"] = (
            screens["status"] == "passed_logged_screens"
            and numerical_pass
            and convergence["status"] == "passed"
            and (handoff is None or handoff["passed"])
        )
        (folder / "verification.json").write_text(json.dumps(row, indent=2) + "\n")
        reports[name] = row
        print(name, "passed", row["passed"], flush=True)
    summary = dict(
        phases=reports,
        passed=all(r["passed"] for r in reports.values()),
        scope="numerical/physics consistency, not real-vehicle parameter validation",
    )
    (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    if not summary["passed"]:
        raise SystemExit("one or more verification checks failed; inspect summary.json")


if __name__ == "__main__":
    main()
