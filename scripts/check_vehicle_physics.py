"""Run independent vehicle-model checks in order; no GNC or navigation in the loop.

python scripts/check_vehicle_physics.py --stage 1 --output results/vehicle_physics/<run>
python scripts/check_vehicle_physics.py --stage all
"""

import argparse
import csv
import hashlib
import json
import logging
import platform
import shutil
from datetime import datetime
from importlib.metadata import version
from pathlib import Path

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from validation import vehicle_physics as checks
from validation.references import attitude_error

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    "src/validation/vehicle_references.py",
    "src/validation/vehicle_physics.py",
    "src/validation/references.py",
    "src/validation/propulsion.py",
    "src/plant/dynamics.py",
    "src/plant/aero_data.py",
    "src/core/interfaces.py",
    "src/native/plant.cpp",
    "scripts/check_vehicle_physics.py",
    "tests/plant/test_vehicle_physics_validation.py",
]


def jsonable(value):
    if OmegaConf.is_config(value):
        return OmegaConf.to_container(value, resolve=True)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, default=jsonable, allow_nan=False) + "\n")


def save_stage(out, report, configs, series):
    directory = out / f"{report['stage']}_{report['name']}"
    directory.mkdir(parents=True, exist_ok=True)
    write_json(directory / "report.json", report)
    with (directory / "checks.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["case", "metric", "error", "absolute_tolerance", "passed"])
        for result in report["results"]:
            for metric, check in result["checks"].items():
                writer.writerow(
                    [result["name"], metric, check["error"], check["tolerance"], check["passed"]]
                )
    (directory / "settings").mkdir(exist_ok=True)
    for key, cfg in configs.items():
        (directory / "settings" / f"{key}.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    if series:
        (directory / "data").mkdir(exist_ok=True)
        for key, data in series.items():
            if isinstance(data, dict):
                np.savez_compressed(directory / "data" / f"{key}.npz", **data)
            else:
                np.savez_compressed(directory / "data" / f"{key}.npz", values=data)


def write_summary(out):
    reports = [json.loads(path.read_text()) for path in sorted(out.glob("[1-5]_*/report.json"))]
    matching_sources = len({json.dumps(r["source_sha256"], sort_keys=True) for r in reports}) == 1
    status = (
        "passed"
        if len(reports) == 5
        and {r["stage"] for r in reports} == set(range(1, 6))
        and matching_sources
        and all(r["status"] == "passed" for r in reports)
        else "incomplete_or_failed"
    )
    write_json(
        out / "summary.json",
        dict(
            status=status,
            stages=[{key: r[key] for key in ["stage", "name", "cases", "status"]} for r in reports],
        ),
    )


def plot_coupled(directory):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["STIXGeneral"],
            "mathtext.fontset": "stix",
            "font.size": 9,
            "axes.labelsize": 9,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes.linewidth": 0.6,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.top": True,
            "ytick.right": True,
            "legend.frameon": False,
            "axes.grid": False,
            "pdf.fonttype": 42,
            "savefig.dpi": 600,
        }
    )
    data_dir = directory / "5_coupled_6dof/data"
    data = np.load(data_dir / "coupled_burn_0.001.npz")
    t = data["t"]
    fig, axes = plt.subplots(1, 2, figsize=(7.05, 2.6))
    fig.subplots_adjust(left=0.09, right=0.985, bottom=0.19, top=0.82, wspace=0.38)
    axes[0].plot(t, data["reference_v_ned"][:, 2], color="black", lw=0.9)
    samples = np.arange(0, len(t), 100)
    axes[0].plot(
        t[samples],
        data["actual_v_ned"][samples, 2],
        ls="none",
        marker="o",
        ms=3,
        mew=0.65,
        mfc="none",
        color="#0072B2",
    )
    axes[0].set_ylabel(r"Down velocity, $v_D$ (m s$^{-1}$)")
    colors = ["#D55E00", "#0072B2", "#009E73"]
    styles = ["-", "--", "-."]
    for dt, color, style in zip([0.002, 0.001, 0.0005], colors, styles, strict=True):
        data = np.load(data_dir / f"coupled_burn_{dt:g}.npz")
        error = attitude_error(data["actual_q_bn"], data["reference_q_bn"])
        axes[1].semilogy(
            data["t"][1:],
            np.maximum(error[1:], 1e-16),
            color=color,
            ls=style,
            lw=0.8,
            label=f"{dt * 1000:g} ms",
        )
    axes[1].axhline(1e-8, color=".4", ls=":", lw=0.7)
    axes[1].set_ylabel(r"Attitude error, $e_R$ (rad)")
    axes[1].legend(loc="lower right", fontsize=8)
    for label, axis in zip(["(a)", "(b)"], axes, strict=True):
        axis.text(1.0, 1.05, label, transform=axis.transAxes, ha="right")
        axis.set(xlabel=r"Time, $t$ (s)", xlim=(0, 3))
    fig.legend(
        handles=[
            Line2D([], [], color="black", lw=0.9, label="Independent reference"),
            Line2D(
                [], [], color="#0072B2", ls="none", marker="o", ms=3, mfc="none", label="Simulation"
            ),
            Line2D([], [], color=".4", ls=":", lw=0.7, label=r"Attitude tolerance, $10^{-8}$ rad"),
        ],
        loc="upper center",
        bbox_to_anchor=(0.54, 1),
        ncol=3,
        fontsize=8,
    )
    out = directory / "figures"
    out.mkdir(exist_ok=True)
    for ext in ["png", "pdf"]:
        fig.savefig(out / f"coupled_comparison.{ext}")
    plt.close(fig)


def main():
    logging.getLogger("fontTools").setLevel(logging.WARNING)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["all", "1", "2", "3", "4", "5"], default="all")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    out = args.output or ROOT / "results/vehicle_physics" / datetime.now().strftime(
        "%Y-%m-%d/%H-%M-%S"
    )
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        cfg = compose(
            config_name="config", overrides=["scenario=vacuum_burn", "plant/thrust=vacuum"]
        )
    functions = {
        1: checks.check_fins,
        2: checks.check_rcs,
        3: checks.check_actuators,
        4: checks.check_aero_environment,
        5: checks.check_coupled,
    }
    stages = list(range(1, 6)) if args.stage == "all" else [int(args.stage)]
    hashes = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}
    passed = True
    for stage in stages:
        report, configs, series = functions[stage](cfg)
        report["source_sha256"] = hashes
        save_stage(out, report, configs, series)
        passed &= report["status"] == "passed"
        print(
            f"{stage}. {report['name']}: {report['status']} ({report['cases']} cases)", flush=True
        )
        for case in report["results"]:
            if not case["passed"]:
                print(json.dumps(case, default=jsonable), flush=True)
    for name in SOURCES:
        target = out / "source" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
        if hashlib.sha256(target.read_bytes()).hexdigest() != hashes[name]:
            raise RuntimeError(
                f"source changed during execution: {name}; rerun from stable sources"
            )
    write_json(out / "source_sha256.json", hashes)
    import _rlv_sim_native

    binary = Path(_rlv_sim_native.__file__)
    write_json(
        out / "environment.json",
        dict(
            python=platform.python_version(),
            packages={
                name: version(name)
                for name in ["numpy", "scipy", "hydra-core", "omegaconf", "matplotlib"]
            },
            native_binary=str(binary),
            native_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
            completed_local_time=datetime.now().isoformat(),
            command_stage=args.stage,
        ),
    )
    write_summary(out)
    if 5 in stages:
        plot_coupled(out)
    print(f"output: {out}", flush=True)
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
