"""Freeze CRS-11 observation windows/masks and produce an offline quality report.

Does not construct a plant, run a simulation, fit parameters, or infer touchdown.
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
from omegaconf import OmegaConf

from validation.flight_observations import prepare_observations

ROOT = Path(__file__).resolve().parents[1]


def plot_observations(report, arrays, output):
    fig, axes = plt.subplots(5, 2, figsize=(12, 15))
    time = arrays["reference_time_s"]
    for row, (name, phase) in enumerate(report["phases"].items()):
        selected = arrays["phase_index"] == phase["phase_index"]
        lo, hi = phase["requested_range_s"]
        for col, (channel, field, scale, label) in enumerate(
            (
                ("altitude", "altitude_m", 1000, "Altitude [km]"),
                ("speed", "speed_mps", 1, "Scalar speed [m/s]"),
            )
        ):
            ax = axes[row, col]
            usable = selected & arrays[f"{channel}_eligible"]
            excluded = selected & ~arrays[f"{channel}_eligible"]
            ax.plot(time[usable], arrays[field][usable] / scale, ".", ms=2, label="Eligible")
            if np.any(excluded):
                ax.plot(
                    time[excluded],
                    arrays[field][excluded] / scale,
                    "x",
                    ms=3,
                    color="0.6",
                    label="Excluded zero-altitude display",
                )
            for gap in report["gaps"]:
                if gap["left_s"] < hi and gap["right_s"] > lo:
                    ax.axvspan(
                        max(lo, gap["left_s"]),
                        min(hi, gap["right_s"]),
                        color="orange",
                        alpha=0.25,
                        label="Gap > cutoff",
                    )
            fraction = phase[channel]["coverage_fraction"]
            ax.set(
                title=f"{name}: {fraction:.1%} eligible time",
                xlim=(lo, hi),
                xlabel="Archived reference T+ [s]",
                ylabel=label,
            )
            ax.grid(alpha=0.25)
            ax.legend(fontsize=8)
    fig.suptitle(
        "CRS-11 observation policy v1: raw displays, no smoothing or simulation\n"
        "Datum/frame assumed; sample eligibility does not certify measurement accuracy"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    for extension in ("png", "pdf"):
        fig.savefig(output / f"observation_windows.{extension}", dpi=160)
    plt.close(fig)


def write_summary(report, output):
    lines = [
        "# CRS-11 observation preparation — step 1",
        "",
        "Generated offline from the pinned archive. No flight simulation or calibration.",
        "Eligibility uses the declared provisional datum/frame, not certified sensor truth.",
        "",
        "| Phase | Reference window [s] | Raw rows | Altitude eligible time [s] | "
        "Speed eligible time [s] | Altitude excluded rows |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for name, p in report["phases"].items():
        lo, hi = p["requested_range_s"]
        lines.append(
            f"| {name} | {lo:g}–{hi:g} | {p['raw_samples']} | "
            f"{p['altitude']['covered_duration_s']:.3f} | "
            f"{p['speed']['covered_duration_s']:.3f} | {p['altitude']['excluded_samples']} |"
        )
    lines += [
        "",
        "Intervals use original adjacent eligible rows in the same phase. No boundary "
        "extrapolation or bridging of excluded rows/gaps. Initialization-use masks are pending.",
        "",
        f"Landing split: T+{report['landing_split']['time_s']:.3f} s, "
        f"{report['landing_split']['altitude_m']:g} m display, "
        f"{report['landing_split']['speed_mps']:.3f} m/s scalar speed.",
        "This is a display-based partition, not an exact altitude-crossing measurement.",
        "",
        "## Limitations",
        "",
        *[f"- {item}" for item in report["limitations"]],
        "",
        "See observation_quality.json for provenance, policy, plateaus, events and file hashes.",
    ]
    (output / "summary.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy", type=Path, default=ROOT / "configs/analysis/public_flight_comparison.yaml"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results/verification/public_flight_comparison/observations",
    )
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    data_root = ROOT / "data/reference/crs11"
    policy = OmegaConf.load(args.policy)
    report, arrays = prepare_observations(data_root, policy)
    dependencies = [
        Path(__file__),
        ROOT / "src/validation/flight_observations.py",
        ROOT / "src/validation/segment_spec.py",
        args.policy,
        data_root / "manifest.json",
    ]
    report["preparation_sha256"] = {
        str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p.resolve()): hashlib.sha256(
            p.read_bytes()
        ).hexdigest()
        for p in dependencies
    }
    args.output.mkdir(parents=True, exist_ok=True)
    OmegaConf.save(policy, args.output / "observation_policy.yaml")
    (args.output / "observation_quality.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    np.savez_compressed(args.output / "observations.npz", **arrays)
    fields = list(arrays)
    with (args.output / "observations.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for row in zip(*(arrays[key] for key in fields), strict=True):
            writer.writerow(
                ["" if isinstance(x, (float, np.floating)) and np.isnan(x) else x for x in row]
            )
    write_summary(report, args.output)
    if not args.no_plots:
        plot_observations(report, arrays, args.output)
    print(f"Step 1 observation preparation complete: {args.output}")
    for name, p in report["phases"].items():
        print(
            f"{name}: {p['raw_samples']} rows, eligible altitude/speed time "
            f"{p['altitude']['covered_duration_s']:.3f}/"
            f"{p['speed']['covered_duration_s']:.3f} s"
        )


if __name__ == "__main__":
    main()
