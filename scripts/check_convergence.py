"""Deterministic plant-step refinement using the same configured episode runner.

python scripts/check_convergence.py scenario=coupled_6dof
"""

import json
from pathlib import Path

import hydra
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from omegaconf import OmegaConf

from core.seeding import write_manifest
from validation.convergence import check_convergence


@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg):
    options = cfg.get("convergence", {})
    resolved = OmegaConf.to_yaml(cfg, resolve=True)
    Path("resolved_config.yaml").write_text(resolved)
    write_manifest(Path.cwd(), cfg.seed, resolved, extra={"mode": "convergence"})
    report = check_convergence(
        cfg, options.get("steps", [0.002, 0.001, 0.0005]), options.get("limits")
    )
    Path("convergence.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    for ax, key in zip(axes.flat, report["limits"], strict=True):
        x = [row["coarse_dt"] for row in report["comparisons"]]
        y = [max(row["errors"][key], 1e-16) for row in report["comparisons"]]
        ax.loglog(x, y, "o-")
        ax.axhline(report["limits"][key], color="red", linestyle="--")
        ax.set(title=key, xlabel="coarser plant_dt [s]")
        ax.grid(True)
    fig.tight_layout()
    fig.savefig("convergence.png", dpi=120)
    plt.close(fig)
    print(json.dumps({k: v for k, v in report.items() if k != "resolved_configs"}, indent=2))
    print(f"output: {Path.cwd()}")
    if report["status"] == "failed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
