"""Reproduce ten frozen before/after runs and assemble the five-phase comparison.

Default: calls run_single for both candidates in every phase, checks reproduction,
then joins immutable P3/P4 verification and sensitivity evidence. --reuse-archives
only rebuilds the report; it does not simulate. No search/refitting is performed.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from omegaconf import OmegaConf

from validation.flight_report import CHANNELS, phase_metrics, reproduction_errors
from validation.flight_sensitivity import physical_audit

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "results/verification/public_flight_comparison"
BEFORE = BASE / "step3_20261107/before"
AFTER = BASE / "reconstruction_20261107/final"
P4 = BASE / "p4_20261107"
NAMES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]
FILES = (
    "resolved_config.yaml",
    "physics.npz",
    "actuators.npz",
    "kpi.json",
    "flight_comparison.json",
    "flight_comparison.npz",
)


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_hashes(manifest):
    changed = [
        str(path)
        for path, digest in manifest.items()
        if not (ROOT / path).is_file() or sha(ROOT / path) != digest
    ]
    if changed:
        raise ValueError(f"evidence/source changed; refresh verification first: {changed}")


def copy_file(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def plot_phase(folder, name, before, after, before_physics, after_physics, report):
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    rt = after["reference_time_s"]
    epoch = report["window_s"][0]
    for col, (key, physics_key, ylabel) in enumerate(
        [("altitude_m", "altitude_m", "Altitude [m]"), ("speed_m_s", "speed_mps", "Speed [m/s]")]
    ):
        mask = after[f"{key}_quality_eligible"] & ~after["initialization_used"]
        axes[0, col].plot(
            rt[~mask],
            after[f"reference_{key}"][~mask],
            "x",
            color=".7",
            ms=3,
            label="Excluded observation",
        )
        axes[0, col].plot(
            rt[mask],
            after[f"reference_{key}"][mask],
            ".",
            color="black",
            ms=2,
            label="Eligible observation (may lack sim overlap)",
        )
        for label, data, p, color in (
            ("Before", before, before_physics, "#bd6337"),
            ("After", after, after_physics, "#197a97"),
        ):
            axes[0, col].plot(p["t"] + epoch, p[physics_key], color=color, label=label)
            # Points preserve missing/excluded intervals without drawing connecting lines.
            axes[1, col].plot(rt, data[f"error_{key}"], ".", ms=2, color=color, label=label)
        axes[0, col].set(ylabel=ylabel, xlabel="Reference T+ [s]")
        axes[1, col].set(ylabel=f"Sim - observation {ylabel}", xlabel="Reference T+ [s]")
        axes[1, col].axhline(0, color=".5", lw=0.7)
        for row in range(2):
            axes[row, col].set_xlim(report["window_s"])
            axes[row, col].legend(fontsize=7)
            axes[row, col].grid(alpha=0.25)
    valid = (
        after["altitude_m_quality_eligible"]
        & after["speed_m_s_quality_eligible"]
        & ~after["initialization_used"]
    )
    axes[0, 2].plot(
        after["reference_altitude_m"][valid],
        after["reference_speed_m_s"][valid],
        ".",
        ms=2,
        color="black",
        label="Eligible observation",
    )
    for label, p, color in (
        ("Before", before_physics, "#bd6337"),
        ("After", after_physics, "#197a97"),
    ):
        axes[0, 2].plot(p["altitude_m"], p["speed_mps"], color=color, label=label)
    axes[0, 2].set(xlabel="Altitude [m]", ylabel="Speed [m/s]", title="Speed–altitude")
    axes[0, 2].legend(fontsize=8)
    axes[0, 2].grid(alpha=0.25)
    full = report["metrics"]["full_overlap"]["after"]
    common = report["metrics"]["common_coverage"]
    lines = ["After eligible-observation coverage:"]
    for key in CHANNELS:
        m = full[key]
        lines.append(
            f"{key}: {m['covered_duration_s']:.3f} / {m['available_observation_duration_s']:.3f} s"
        )
    lines += ["", "RMSE before → after (common coverage):"]
    for key in CHANNELS:
        m = common[key]
        lines.append(f"{key}: {m['nominal']['rmse']:.3f} → {m['probe']['rmse']:.3f}")
    lines += [
        "",
        "Fitted reconstruction; no holdout accuracy claim.",
        "Model checks are separate from residuals.",
    ]
    if name == "landing":
        lines += [
            "Ideal-actuator soft landing is conditional.",
            "Unrepresented speed tail is not zero-filled.",
            "Realistic-actuator sensitivity case crashes.",
        ]
    axes[1, 2].axis("off")
    axes[1, 2].text(0, 1, "\n".join(lines), va="top", fontsize=10)
    fig.suptitle(f"CRS-11 {name}: fixed before/after reconstruction", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(folder / "comparison.png", dpi=150)
    fig.savefig(folder / "comparison.pdf")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=BASE / "integrated_20261107")
    parser.add_argument("--reuse-archives", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        raise FileExistsError(f"preserve previous report; choose a new --output: {out}")
    p4_manifest = read(P4 / "manifest.json")
    verify_hashes(p4_manifest["source_sha256"])
    verify_hashes(p4_manifest["frozen_artifact_sha256"])
    p3, p4 = read(AFTER / "summary.json"), read(P4 / "summary.json")
    if not (p4["nominal_scoped_physical_pass"] and p4["numerical_pass"]):
        raise ValueError("nominal verification did not pass")
    # All files consumed are pinned in this report, including P4 failed-case evidence.
    inputs = {
        str(p.relative_to(ROOT)): sha(p)
        for base in (BEFORE, AFTER, P4)
        for p in base.rglob("*")
        if p.is_file()
    }
    sources = list((ROOT / "src").rglob("*.py")) + list((ROOT / "configs").rglob("*.yaml"))
    sources += [p for p in (ROOT / "data/reference/crs11").rglob("*") if p.is_file()]
    sources += [
        Path(__file__),
        ROOT / "scripts/run_single.py",
        ROOT / "scripts/_plotting.py",
        ROOT / "src/native/plant.cpp",
        ROOT / "pyproject.toml",
        ROOT / "setup.py",
    ]
    source_hashes = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    out.mkdir(parents=True)
    write(out / "input_manifest.json", dict(artifacts=inputs, source_sha256=source_hashes))
    for src, dst in (
        (P4 / "summary.json", "verification/p4_summary.json"),
        (AFTER / "summary.json", "verification/p3_summary.json"),
        (P4 / "landing_input_replay.json", "verification/landing_input_replay.json"),
        (P4 / "sensitivity.png", "verification/sensitivity.png"),
        (P4 / "display_tolerance_sensitivity.json", "verification/display_sensitivity.json"),
        (ROOT / "data/reference/crs11/manifest.json", "verification/data_manifest.json"),
    ):
        copy_file(src, out / dst)
    for path in (ROOT / "configs/analysis").glob("*flight*.yaml"):
        copy_file(path, out / "policies" / path.name)
    phases = {}
    for name in NAMES:
        folder = out / name
        arrays, physics, versions = {}, {}, {}
        for label, archive, prefix in (
            ("before", BEFORE, "flight_comparison"),
            ("after", AFTER, "flight_reconstructed"),
        ):
            dest = folder / label
            dest.mkdir(parents=True)
            print(label, name, "copy" if args.reuse_archives else "run_single", flush=True)
            if args.reuse_archives:
                for filename in FILES:
                    copy_file(archive / name / filename, dest / filename)
            else:
                command = [
                    sys.executable,
                    str(ROOT / "scripts/run_single.py"),
                    f"scenario={prefix}_{name}",
                    f"hydra.run.dir={dest}",
                    f"run.run_id=integrated_{label}_{name}",
                    "run.save_plots=false",
                    "run.save_replay=false",
                ]
                write(dest / "command.json", command)
                with (dest / "run.log").open("w") as log:
                    subprocess.run(
                        command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True
                    )
            cfg = OmegaConf.load(dest / "resolved_config.yaml")
            archived_cfg = OmegaConf.load(archive / name / "resolved_config.yaml")
            for key in ("plant", "environment", "fault", "gnc", "scenario", "seed", "plant_dt"):
                if cfg[key] != archived_cfg[key]:
                    raise ValueError(f"{name}/{label} configuration changed: {key}")
            arrays[label] = dict(np.load(dest / "flight_comparison.npz"))
            physics[label] = dict(np.load(dest / "physics.npz"))
            reproduction = reproduction_errors(
                physics[label], dict(np.load(archive / name / "physics.npz"))
            )
            if not reproduction["passed"]:
                write(dest / "reproduction.json", reproduction)
                raise ValueError(f"{name}/{label} native reproduction differs; reverify candidate")
            current = read(dest / "flight_comparison.json")
            if current != read(archive / name / "flight_comparison.json"):
                raise ValueError(f"{name}/{label} comparison changed; regenerate verification")
            audit = physical_audit(physics[label], dict(np.load(dest / "actuators.npz")), cfg)
            versions[label] = dict(
                initial_conditions=OmegaConf.to_container(cfg.scenario.ic),
                assumptions=OmegaConf.to_container(cfg.scenario.reconstruction.provenance),
                control_inputs=OmegaConf.to_container(cfg.scenario.inputs)
                if "inputs" in cfg.scenario
                else dict(method="LCvx/PD", guidance=OmegaConf.to_container(cfg.gnc.guidance)),
                actuator_model=str(cfg.plant.actuator.model),
                dry_mass_kg=float(cfg.plant.vehicle.mass.m_dry),
                initial_altitude_m=float(physics[label]["altitude_m"][0]),
                initial_speed_mps=float(physics[label]["speed_mps"][0]),
                final_state={
                    key: physics[label][key][-1].tolist()
                    for key in ("t", "r_ned", "v_ned", "mass", "altitude_m", "speed_mps")
                },
                kpi=read(dest / "kpi.json"),
                physical_audit=audit,
                reproduction=reproduction,
                mode="archive_copy" if args.reuse_archives else "fresh_run",
            )
        window = current["requested_range_s"]
        sensitivity = folder / "sensitivity"
        sensitivity.mkdir()
        for label in p4["campaigns"]:
            source = P4 / label / name
            if source.is_dir():
                shutil.copytree(source, sensitivity / label)
        write(sensitivity / "observation_operator.json", p4["observation_sensitivity"][name])
        write(
            sensitivity / "summary.json",
            {label: campaign["phases"].get(name) for label, campaign in p4["campaigns"].items()},
        )
        report = dict(
            phase=name,
            window_s=window,
            versions=versions,
            metrics=phase_metrics(arrays["before"], arrays["after"], window[1] - window[0]),
            numerical_verification=p3["phases"][name]["numerical_verification"],
            p4_verdict=p4["phase_verdicts"][name],
            touchdown_truth=dict(
                time_s=None,
                velocity_mps=None,
                position=None,
                reason="not available in the pinned webcast observations",
            ),
        )
        if name == "landing":
            report["landing_independent_plant_replay"] = p4["landing_input_replay"]
        write(folder / "summary.json", report)
        plot_phase(
            folder,
            name,
            arrays["before"],
            arrays["after"],
            physics["before"],
            physics["after"],
            report,
        )
        phases[name] = report
    verify_hashes(inputs)
    verify_hashes(source_hashes)
    report = dict(
        status="integrated_report_complete_with_partial_landing_coverage",
        run_mode="verified_archive_report_only" if args.reuse_archives else "ten_fresh_frozen_runs",
        phases=phases,
        mass_handoffs=p3["mass_handoffs"],
        nominal_scoped_model_checks_passed=all(
            not r["versions"]["after"]["physical_audit"]["failed_checks"] for r in phases.values()
        ),
        empirical_accuracy_certified=False,
        robust_landing_demonstrated=False,
        full_landing_observation_history_reproduced=False,
        before_after_artifacts_preserved=True,
        limitations=[
            "independent phase initializations, not a connected whole flight",
            "same-data fit, not held-out prediction",
            "source datum/frame/latency unconfirmed",
            "ideal-actuator soft landing is conditional; realistic-actuator probe crashes",
            "public landing speed tail after simulated contact remains unrepresented",
            "no isolated disturbance, noise or slosh identification",
        ],
        software={
            name: importlib.metadata.version(name)
            for name in (
                "numpy",
                "scipy",
                "cvxpy",
                "clarabel",
                "hydra-core",
                "omegaconf",
                "matplotlib",
            )
        },
    )
    write(out / "summary.json", report)
    lines = [
        "# CRS-11 integrated comparison",
        "",
        "Ten frozen before/after candidates; scoped checks are not empirical certification.",
        "",
        "| Phase | h RMSE before → after [m] | v RMSE before → after [m/s] | "
        "After h / v coverage [s] |",
        "|---|---:|---:|---:|",
    ]
    for name, r in phases.items():
        common, full = r["metrics"]["common_coverage"], r["metrics"]["full_overlap"]["after"]
        values = [
            f"{common[k]['nominal']['rmse']:.3f} → {common[k]['probe']['rmse']:.3f}"
            for k in CHANNELS
        ]
        lines.append(
            f"| [{name}]({name}/comparison.png) | {values[0]} | {values[1]} | "
            f"{full['altitude_m']['covered_duration_s']:.3f} / "
            f"{full['speed_m_s']['covered_duration_s']:.3f} |"
        )
    lines += [
        "",
        "RMSE uses common coverage; coverage columns describe the full after run.",
        "Landing's remaining eligible speed observations are not reproduced.",
        "See each phase/summary.json for bias, max error, endpoints, assumptions and checks.",
        "Sensitivity variants and rejected cases are archived per phase.",
    ]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    manifest = {str(p.relative_to(out)): sha(p) for p in out.rglob("*") if p.is_file()}
    write(out / "artifact_manifest.json", manifest)
    print("\n".join(lines), flush=True)
    if not report["nominal_scoped_model_checks_passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
