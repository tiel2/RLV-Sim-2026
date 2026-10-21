"""Configuration composition and plant fixtures."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from omegaconf import OmegaConf

from plant.dynamics import build_plant

CONFIG_DIR = str(Path(__file__).resolve().parents[1] / "configs")


def compose_cfg(overrides=None):
    """Compose the Hydra config with optional override strings."""
    GlobalHydra.instance().clear()
    with initialize_config_dir(version_base=None, config_dir=CONFIG_DIR):
        return compose(config_name="config", overrides=overrides or [])


def build_bundle(overrides=None):
    return build_plant(compose_cfg(overrides))


def nominal_for(bundle, overrides=None):
    """GNC nominal (vehicle/plant constants) for a bundle — used by the allocation tests."""
    from gnc.build import default_nominal

    cfg = compose_cfg(overrides)
    return default_nominal(bundle.params, cfg.gnc.allocation, str(cfg.environment.atmosphere.model))


@pytest.fixture
def bundle():
    return build_bundle()


@pytest.fixture
def tabulated_aero_cfg(tmp_path):
    """Synthetic Falcon coefficient data for the generic file/fin model contracts."""
    cfg = compose_cfg()
    cos_grid = np.array([0.0, 0.5, 1.0])
    mach_grid = np.array([0.0, 1.0, 3.0])
    cone = np.broadcast_to(1.0 - cos_grid[:, None], (3, 3))
    path = tmp_path / "aero.npz"
    np.savez(
        path,
        cos_grid=cos_grid,
        mach_grid=mach_grid,
        cd_table=1.2 + 0.2 * cone + 0.03 * mach_grid[None, :],
        cl_table=2.0 * cone,
        xcp_table=np.full((3, 3), -20.0),
        cd_slope=np.full((3, 3), -0.2),
        cl_slope=np.full((3, 3), -2.0),
        xcp_slope=np.zeros((3, 3)),
        cm_table=0.1 * cone,
        cm_slope=np.full((3, 3), -0.1),
        x_cg_ref_m=-25.0,
    )
    OmegaConf.update(
        cfg,
        "plant.aero",
        {
            "backend": "file",
            "file": {"path": str(path)},
            "fins": {
                "incremental": True,
                "fin_phase_deg": 45.0,
                "mach_grid": mach_grid.tolist(),
                "cla_table": [2.0, 1.5, 1.0],
                "cd_delta2_table": [1.0, 0.8, 0.6],
            },
        },
        force_add=True,
    )
    return cfg
