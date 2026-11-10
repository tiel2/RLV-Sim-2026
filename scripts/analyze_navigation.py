"""Run a fixed truth trajectory, then compare IMU-only INS and aided ESKF.

Usage:
    python scripts/analyze_navigation.py scenario=project2_landing
    python scripts/analyze_navigation.py gnc/navigation=synthetic_eskf scenario=project2_landing
"""

from __future__ import annotations

import json
from pathlib import Path

import hydra
from omegaconf import OmegaConf

from gnc.build import build_nav
from sim.run import run_configured
from validation.navigation import write_navigation_comparison


@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg):
    if str(cfg.scenario.get("input_mode", "closed_loop")) != "closed_loop":
        raise ValueError("navigation analysis requires a closed-loop truth scenario")

    selected_nav = OmegaConf.create(OmegaConf.to_container(cfg.gnc.navigation, resolve=True))
    if selected_nav.law._target_ != "gnc.navigation.synthetic_eskf.build":
        sensor_config = OmegaConf.load(
            Path(__file__).resolve().parents[1]
            / "configs"
            / "gnc"
            / "navigation"
            / "synthetic_eskf.yaml"
        )
    else:
        sensor_config = selected_nav

    # The same explicit timing is used by standalone, closed-loop, and report runs.
    cfg.gnc.navigation = sensor_config
    sensor = build_nav(cfg)  # validates the configured period before running truth
    cfg.gnc.navigation = OmegaConf.create({"law": {"_target_": "gnc.navigation.identity.build"}})
    _, truth_result = run_configured(cfg, progress=False, record_telemetry=True)
    if truth_result.navigation is None:
        raise RuntimeError("truth rollout did not record navigation-rate states")
    out_dir = Path.cwd()
    metadata = write_navigation_comparison(
        truth_result.navigation,
        sensor,
        int(cfg.seed),
        out_dir,
        scenario=str(cfg.scenario.name),
        sensor_configuration=OmegaConf.to_container(sensor_config, resolve=True),
    )
    print(json.dumps(metadata["metrics"], indent=2, allow_nan=False))
    print(f"saved: {out_dir / 'navigation_comparison.npz'}")
    print(f"saved: {out_dir / 'navigation_comparison.json'}")


if __name__ == "__main__":
    main()
