"""Convex brake targets must follow the actual scheduler configuration."""

from types import SimpleNamespace

import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf

from core.interfaces import Phase
from gnc.build import build_gnc
from plant.dynamics import build_plant


@pytest.mark.parametrize("method", ["lcvx", "scvx"])
@pytest.mark.parametrize("scheduler", ["altitude", "guard"])
@pytest.mark.parametrize("height,sink_limit", [(800.0, 180.0), (2000.0, 90.0)])
def test_brake_target_and_transition_share_configuration(method, scheduler, height, sink_limit):
    cfg = compose_cfg(
        [
            "plant/vehicle=falcon9_octaweb",
            "scenario=reentry_to_landing",
            f"gnc/guidance={method}",
            f"gnc/scheduler={scheduler}",
            "gnc.scheduler.powered_brake=true",
            "gnc.scheduler.switch_altitude=5000",
            f"gnc.scheduler.brake_end_altitude={height}",
            f"gnc.scheduler.brake_terminal_vz_max_mps={sink_limit}",
            "gnc.guidance.convex_target_ned=[17,-9,-12]",
            "gnc.guidance.convex_terminal_vz_max_mps=2",
        ]
    )
    before = OmegaConf.to_container(cfg, resolve=True)
    stack = build_gnc(cfg, build_plant(cfg).params)
    dispatcher = stack.guidance
    brake = dispatcher.phase_guidance[int(Phase.POWERED_BRAKE)]._law
    landing = dispatcher.phase_guidance[int(Phase.POWERED_DESCENT)]._law
    assert brake.target == (17.0, -9.0, -height)
    assert brake.terminal_vz_max == sink_limit
    assert brake.n_active == 3
    assert landing.target == (17.0, -9.0, -12.0)
    assert landing.terminal_vz_max == 2.0
    assert landing.n_active == 1
    assert OmegaConf.to_container(cfg, resolve=True) == before

    schedule = dispatcher.scheduler
    memory = schedule.initial_memory()
    for altitude, expected in [
        (25000, Phase.REENTRY_BURN),
        (19999, Phase.AERO_DESCENT),
        (4999, Phase.POWERED_BRAKE),
        (height + 0.1, Phase.POWERED_BRAKE),
        (height - 0.1, Phase.POWERED_DESCENT),
    ]:
        phase, memory = schedule.phase(SimpleNamespace(r_ned=[17, -9, -altitude]), 0, memory)
        assert phase == int(expected)


def test_default_brake_handoff_matches_800m_scheduler():
    cfg = compose_cfg(
        [
            "plant/vehicle=falcon9_octaweb",
            "scenario=reentry_to_landing",
            "gnc.scheduler.powered_brake=true",
        ]
    )
    dispatcher = build_gnc(cfg, build_plant(cfg).params).guidance
    brake = dispatcher.phase_guidance[int(Phase.POWERED_BRAKE)]._law
    landing = dispatcher.phase_guidance[int(Phase.POWERED_DESCENT)]._law
    assert brake.target[2] == -cfg.gnc.scheduler.brake_end_altitude == -800.0
    assert landing.target == (0.0, 0.0, 0.0)
    assert landing.terminal_vz_max is None


@pytest.mark.parametrize("sink_limit", [0, -1, float("nan"), float("inf")])
def test_invalid_brake_sink_limit_rejected(sink_limit):
    cfg = compose_cfg(
        [
            "plant/vehicle=falcon9_octaweb",
            "scenario=reentry_to_landing",
            "gnc.scheduler.powered_brake=true",
        ]
    )
    cfg.gnc.scheduler.brake_terminal_vz_max_mps = sink_limit
    with pytest.raises(ValueError, match="brake_terminal_vz_max_mps"):
        build_gnc(cfg, build_plant(cfg).params)
