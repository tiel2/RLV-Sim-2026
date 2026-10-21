"""Falcon configuration defaults and explicit single/nine-engine selections."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import build_bundle, compose_cfg
from omegaconf import OmegaConf


def test_default_matches_explicit_falcon_set():
    default = compose_cfg()
    explicit = compose_cfg(["vehicle_set=falcon9"])
    for group in ("plant", "environment", "scenario", "gnc"):
        assert OmegaConf.to_container(default[group], resolve=True) == OmegaConf.to_container(
            explicit[group], resolve=True
        )
    assert default.scenario.name == "pdg"
    assert default.plant.actuator.model == "realistic"


@pytest.mark.parametrize("vehicle,count", [("falcon9_stage1", 1), ("falcon9_octaweb", 9)])
def test_falcon_engine_geometry(vehicle, count):
    bundle = build_bundle([f"plant/vehicle={vehicle}"])
    positions = np.asarray(bundle.params.engines.positions)
    assert positions.shape == (count, 3)
    assert bundle.meta["n_gimbal"] == 1
    assert np.array_equal(positions[0, 1:], np.zeros(2))
    if count > 1:
        assert np.allclose(positions[1:, 1:].sum(axis=0), 0.0, atol=1e-12)
