"""Archive reconstructed runs, compare matched coverage, and verify numerical behavior."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from omegaconf import OmegaConf
from run_flight_before import summarize_case

from sim.run import run_configured
from validation.ascent import compare, reference
from validation.convergence import check_convergence, compare_results
from validation.flight_comparison_policy import masked_statistics

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/verification/public_flight_comparison/reconstruction_20261107/final"
BEFORE = ROOT / "results/verification/public_flight_comparison/step3_20261107/before"
NAMES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]
REF_LIMITS = dict(r_ned=0.01, v_ned=0.001, w_body=1e-6, q_bn=1e-6, m=0.001)
LANDING_LIMITS = dict(
    position_m=0.1,
    velocity_mps=0.05,
    attitude_rad=0.001,
    mass_kg=0.05,
    dynamic_pressure_pa=10.0,
    terminal_time_s=0.01,
)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    reports = {}
    before_hashes = {
        str(p.relative_to(BEFORE)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in BEFORE.rglob("*")
        if p.is_file()
    }
    for name in NAMES:
        folder = OUT / name
        if (folder / "physics.npz").exists():
            raise FileExistsError(f"preserve previous final run: {folder}")
        folder.mkdir(exist_ok=True)
        command = [
            sys.executable,
            str(ROOT / "scripts/run_single.py"),
            f"scenario=flight_reconstructed_{name}",
            f"hydra.run.dir={folder}",
            f"run.run_id=crs11_reconstructed_{name}",
        ]
        (folder / "command.json").write_text(json.dumps(command, indent=2) + "\n")
        print("archive", name, flush=True)
        with (folder / "run.log").open("w") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        report = summarize_case(folder, name)
        cfg = OmegaConf.load(folder / "resolved_config.yaml")
        physics = np.load(folder / "physics.npz")
        if name != "landing":
            print("independent equations + step refinement", name, flush=True)
            actual = SimpleNamespace(
                **{k: physics[k] for k in ["r_ned", "v_ned", "q_bn", "w_body"]}, m=physics["mass"]
            )
            constants = OmegaConf.to_container(cfg, resolve=True)
            expected = reference(constants, physics["t"], rtol=1e-11, allow_neutral_fins=True)
            tighter = reference(constants, physics["t"], rtol=1e-12, allow_neutral_fins=True)
            errors, stability = (
                compare(actual, tighter),
                compare(SimpleNamespace(**expected), tighter),
            )
            independent = dict(
                errors=errors,
                reference_refinement=stability,
                limits=REF_LIMITS,
                passed=all(
                    errors[k] <= tol and stability[k] <= tol / 10 for k, tol in REF_LIMITS.items()
                ),
            )
            convergence = check_convergence(cfg, steps=(0.001, 0.0005, 0.00025))
        else:
            runs = []
            for dt in [0.002, 0.001, 0.0005]:
                cfg.plant_dt = dt
                _, result = run_configured(cfg, record_telemetry=False)
                runs.append(result)
            pairs = []
            for left, right in zip(runs[:-1], runs[1:], strict=True):
                row = compare_results(left, right, cfg.scenario.horizon.log_dt)
                row["passed"] = bool(
                    left.success
                    and right.success
                    and all(row["errors"][k] <= v for k, v in LANDING_LIMITS.items())
                )
                pairs.append(row)
            convergence = dict(
                status="passed" if all(r["passed"] for r in pairs) else "failed",
                steps_s=[0.002, 0.001, 0.0005],
                limits=LANDING_LIMITS,
                comparisons=pairs,
                scope="closed-loop step sensitivity; not an independent dynamics integration",
                landing_speeds_mps=[float(r.landing_speed) for r in runs],
                all_soft_landings=all(r.success and r.landing_class == "soft" for r in runs),
            )
            independent = dict(
                status="not_applied",
                reason="closed-loop landing; prescribed reference inapplicable",
            )
        verification = dict(independent=independent, convergence=convergence)
        (folder / "numerical_verification.json").write_text(
            json.dumps(verification, indent=2, allow_nan=False) + "\n"
        )
        before = np.load(BEFORE / name / "flight_comparison.npz")
        after = np.load(folder / "flight_comparison.npz")
        common = {}
        np.testing.assert_array_equal(before["source_row"], after["source_row"])
        for key in ["altitude_m", "speed_m_s"]:
            mask = before[f"{key}_score_eligible"] & after[f"{key}_score_eligible"]
            common[key] = {
                label: masked_statistics(
                    data["reference_time_s"], data[f"error_{key}"], mask, max_gap_s=1.1
                )[0]
                for label, data in [("before", before), ("after", after)]
            }
        report["numerical_verification"] = verification
        report["common_coverage_comparison"] = common
        report["full_physics_acceptance"] = "scoped_model_checks_only_not_empirical_validation"
        reports[name] = report
    handoffs = []
    for left, right in zip(NAMES[1:-1], NAMES[2:], strict=True):
        delta = (
            reports[right]["physics_screen"]["diagnostics"]["initial_mass_kg"]
            - reports[left]["physics_screen"]["diagnostics"]["final_mass_kg"]
        )
        handoffs.append(
            dict(before=left, after=right, difference_kg=delta, passed=abs(delta) < 1e-6)
        )
    unchanged = all(
        hashlib.sha256((BEFORE / p).read_bytes()).hexdigest() == value
        for p, value in before_hashes.items()
    )
    summary = dict(
        phases=reports,
        mass_handoffs=handoffs,
        before_preserved=unchanged,
        scope="assumption-conditioned fitted reconstruction; no held-out empirical validation",
        recipe="configs/analysis/flight_reconstruction.yaml",
    )
    paths = [
        Path(__file__),
        ROOT / "scripts/reconstruct_flight.py",
        ROOT / "configs/analysis/flight_reconstruction.yaml",
    ]
    paths += list((ROOT / "configs/scenario").glob("flight_reconstructed_*.yaml"))
    summary["source_sha256"] = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    fig, axes = plt.subplots(5, 2, figsize=(12, 15))
    lines = [
        "# Bounded reconstruction: matched-coverage errors",
        "",
        "| Phase | h RMSE before / after [m] | speed RMSE before / after [m/s] | Landing |",
        "|---|---:|---:|---|",
    ]
    for row, name in enumerate(NAMES):
        r = reports[name]
        common = r["common_coverage_comparison"]
        lines.append(
            f"| {name} | {common['altitude_m']['before']['rmse']:.3f} / "
            f"{common['altitude_m']['after']['rmse']:.3f} | "
            f"{common['speed_m_s']['before']['rmse']:.3f} / "
            f"{common['speed_m_s']['after']['rmse']:.3f} | {r['landing_class']} |"
        )
        before = np.load(BEFORE / name / "flight_comparison.npz")
        after = np.load(OUT / name / "flight_comparison.npz")
        for col, (key, scale, label) in enumerate(
            [("altitude_m", 1000, "Altitude [km]"), ("speed_m_s", 1, "Speed [m/s]")]
        ):
            ax = axes[row, col]
            t = after["reference_time_s"]
            ax.plot(
                t,
                after[f"reference_{key}"] / scale,
                ".",
                ms=2,
                label="Webcast (includes excluded samples)",
            )
            ax.plot(t, before[f"simulation_{key}"] / scale, ".", ms=1, label="Before")
            ax.plot(t, after[f"simulation_{key}"] / scale, ".", ms=1, label="Reconstructed")
            ax.set(title=name, xlabel="Reference T+ [s]", ylabel=label)
            ax.grid(alpha=0.25)
            ax.legend(fontsize=7)
    fig.suptitle(
        "Bounded reconstruction with declared assumptions\n"
        "Matched-coverage scores; landing target is engineered"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT / "before_after.png", dpi=160)
    plt.close(fig)
    (OUT / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines), flush=True)


if __name__ == "__main__":
    main()
