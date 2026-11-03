"""Reproduce independent Project 2 segment observations, proposed states and figures.

This command prepares a specification; it does not run or fit a flight simulation.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from validation.segment_spec import estimate_components, load_snapshot, prepare_segments

ROOT = Path(__file__).resolve().parents[1]


def plots(report, recipe, data_root, output):
    _, time, height, speed, _ = load_snapshot(data_root)
    names = list(report["segments"])
    fig, axes = plt.subplots(4, 2, figsize=(12, 13))
    for row, name in enumerate(names):
        segment = report["segments"][name]
        start, end = segment["reference_window_s"]
        selected = (time >= start) & (time <= end)
        for col, (values, scale, label) in enumerate(
            ((height, 1000, "Observed altitude [km]"), (speed, 1, "Observed speed [m/s]"))
        ):
            ax = axes[row, col]
            ax.plot(time[selected], values[selected] / scale, ".", ms=1.5, color=f"C{row}")
            ax.set(
                title=f"{name}: T+{start:g} to {end:g} s", xlabel="Reference T+ [s]", ylabel=label
            )
            ax.grid(alpha=0.25)
    fig.suptitle("CRS-11 webcast observations: selected independent comparison windows")
    fig.tight_layout()
    for extension in ("png", "pdf"):
        fig.savefig(output / f"project2_reference_windows.{extension}", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(3, 4, figsize=(17, 10))
    fields = ["vertical_up_mps", "horizontal_speed_mps", "flight_path_angle_deg"]
    labels = [
        "Estimated vertical up [m/s]",
        "Estimated horizontal speed [m/s]",
        "Estimated flight-path angle [deg]",
    ]
    for col, name in enumerate(names):
        start, end = report["segments"][name]["reference_window_s"]
        epochs = np.unique(np.r_[start, np.arange(np.ceil(start), end, 1), end])
        for span_index, span in enumerate(recipe.reference.derivative_spans_s):
            color = f"C{span_index}"
            rows = [
                estimate_components(
                    time,
                    height,
                    speed,
                    t,
                    span,
                    max_gap_s=recipe.reference.max_gap_s,
                    tolerance=recipe.reference.component_tolerance_mps,
                )
                for t in epochs
            ]
            for row, field in enumerate(fields):
                values = [r[field] if r["status"] == "usable_estimate" else np.nan for r in rows]
                axes[row, col].plot(epochs, values, lw=1, color=color, label=f"Trailing {span:g} s")
            bad = np.array(
                [r["status"] in ("component_inconsistent", "altitude_floor") for r in rows]
            )
            if np.any(bad):
                axes[1, col].plot(
                    epochs[bad],
                    np.zeros(np.sum(bad)),
                    "x",
                    ms=4,
                    color=color,
                    label=f"Rejected {span:g} s",
                )
        axes[0, col].set_title(name)
        for row, label in enumerate(labels):
            axes[row, col].set(xlabel="Reference T+ [s]", ylabel=label, xlim=(start, end))
            axes[row, col].grid(alpha=0.25)
        axes[0, col].legend(fontsize=8)
        axes[1, col].legend(fontsize=7)
    fig.suptitle(
        "Trailing quadratic reconstruction: span sensitivity, not measured state truth\n"
        "Rejected components are not clamped; altitude-floor estimates are excluded"
    )
    fig.tight_layout()
    for extension in ("png", "pdf"):
        fig.savefig(output / f"project2_velocity_reconstruction.{extension}", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--recipe", type=Path, default=ROOT / "configs/analysis/project2_segments.yaml"
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results/verification/project2_segments"
    )
    args = parser.parse_args()
    recipe = OmegaConf.load(args.recipe)
    data_root = ROOT / "data/reference/crs11"
    with initialize_config_dir(config_dir=str(ROOT / "configs"), version_base=None):
        report = prepare_segments(
            data_root,
            recipe,
            lambda name: compose(config_name="config", overrides=[f"scenario={name}"]),
        )
    dependencies = [
        Path(__file__),
        ROOT / "src/validation/segment_spec.py",
        ROOT / "src/validation/references.py",
        args.recipe,
        data_root / "manifest.json",
        ROOT / "pyproject.toml",
    ]
    # Include the entire small YAML config tree: inherited defaults affect proposed states.
    dependencies += sorted((ROOT / "configs").rglob("*.yaml"))
    dependencies += [
        data_root / item["file"]
        for item in json.loads((data_root / "manifest.json").read_text())["files"]
    ]
    report["source_sha256"] = {
        (str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p.resolve())): hashlib.sha256(
            p.read_bytes()
        ).hexdigest()
        for p in dependencies
    }
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "boundary_observations.csv").open("w", newline="") as stream:
        fields = [
            "scenario",
            "boundary",
            "reference_time_s",
            "altitude_m",
            "speed_mps",
            "span_s",
            "estimate_status",
            "vertical_up_mps",
            "horizontal_speed_mps",
            "flight_path_angle_deg",
            "altitude_fit_rmse_m",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for name, segment in report["segments"].items():
            for label, boundary in segment["boundaries"].items():
                for estimate in boundary["velocity_estimates"]:
                    writer.writerow(
                        dict(
                            scenario=name,
                            boundary=label,
                            reference_time_s=boundary["reference_time_s"],
                            altitude_m=boundary["altitude_m"]["value"],
                            speed_mps=boundary["scalar_speed_mps"]["value"],
                            span_s=estimate["span_s"],
                            estimate_status=estimate["status"],
                            **{k: estimate.get(k) for k in fields[7:]},
                        )
                    )
    plots(report, recipe, data_root, args.output)
    (args.output / "reference_analysis.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    print(f"Specification and reference analysis written to {args.output}")
    for name, segment in report["segments"].items():
        s = segment["proposed_initial_state"]
        print(
            f"{name}: h={-s['r_ned'][2]:.3f} m, v_NED={np.round(s['v_ned'], 3)}, "
            f"mass={s['m']:.0f} kg; {segment['status']}"
        )


if __name__ == "__main__":
    main()
