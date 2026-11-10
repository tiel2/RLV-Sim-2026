"""Run a fixed truth trajectory, then compare IMU-only INS and aided ESKF.

Usage:
    python scripts/analyze_navigation.py scenario=project2_landing
    python scripts/analyze_navigation.py gnc/navigation=synthetic_eskf scenario=project2_landing
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import hydra
import numpy as np
from omegaconf import OmegaConf

from gnc.context import GncContext
from gnc.navigation.synthetic_eskf import build as build_sensor_suite
from sim.run import run_configured
from validation.navigation import write_navigation_comparison


@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg):
    if str(cfg.scenario.get("input_mode", "closed_loop")) != "closed_loop":
        raise ValueError("navigation analysis requires a closed-loop truth scenario")

    selected_nav = OmegaConf.create(
        OmegaConf.to_container(cfg.gnc.navigation, resolve=True)
    )
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

    cfg.scenario.horizon.nav_dt = 1.0 / float(sensor_config.imu_rate_hz)
    cfg.gnc.navigation = OmegaConf.create(
        {"law": {"_target_": "gnc.navigation.identity.build"}}
    )
    _, truth_result = run_configured(cfg, progress=False, record_telemetry=True)
    if truth_result.navigation is None:
        raise RuntimeError("truth rollout did not record navigation-rate states")

    truth = truth_result.navigation.truth
    initial_state = SimpleNamespace(
        **{
            name: np.array(getattr(truth, name)[0], copy=True)
            for name in ("r_ned", "v_ned", "q_bn", "w_body", "m", "t")
        }
    )
    sensor = build_sensor_suite(GncContext(cfg=sensor_config, initial_state=initial_state))
    out_dir = Path.cwd()
    metadata = write_navigation_comparison(
        truth,
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
