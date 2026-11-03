"""Replay export: the viz leg-deploy trigger time is the configured deploy altitude crossing."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import build_bundle, compose_cfg

from gnc.build import build_gnc
from gnc.navigation.identity import IdentityNav
from plant.dynamics import ic_from_scenario, initial_state
from sim.loop import run_closed_loop
from sim.replay import build_replay_doc
from sim.run import run_configured


@pytest.fixture(scope="module")
def episode():
    """One short powered-descent closed-loop rollout, shared across the replay tests."""
    cfg = compose_cfg(["scenario=pdg"])
    bundle = build_bundle(["scenario=pdg"])
    state0 = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
    h = cfg.scenario.horizon
    result = run_closed_loop(
        bundle,
        state0,
        build_gnc(cfg, bundle.params),
        IdentityNav(),
        t_max=h.t_max,
        ctrl_dt=h.ctrl_dt,
        plant_dt=cfg.plant_dt,
        nav_dt=h.nav_dt,
        guidance_dt=h.guidance_dt,
        log_dt=h.log_dt,
    )
    return result, bundle


def _alt(result):
    return np.array([float(tick["altitude"]) for tick in result.aux["per_tick"]])


def test_leg_deploy_is_first_below_configured_altitude(episode):
    result, bundle = episode
    cfg = compose_cfg(["scenario=pdg", "run.leg_deploy_alt_m=50"])
    t = build_replay_doc(result, bundle, cfg)["meta"]["leg_deploy"]["t"]
    alt = _alt(result)
    first = int(np.flatnonzero(alt < 50.0)[0])  # first tick below the deploy altitude
    assert t is not None
    assert np.isclose(t, float(result.t[first]))
    assert 0.0 < t <= float(result.t[-1])


def test_leg_deploy_altitude_is_cli_overridable(episode):
    result, bundle = episode
    t_lo = build_replay_doc(
        result, bundle, compose_cfg(["scenario=pdg", "run.leg_deploy_alt_m=30"])
    )["meta"]["leg_deploy"]["t"]
    t_hi = build_replay_doc(
        result, bundle, compose_cfg(["scenario=pdg", "run.leg_deploy_alt_m=200"])
    )["meta"]["leg_deploy"]["t"]
    assert t_hi <= t_lo  # a higher deploy altitude is crossed earlier in the descent


def test_airborne_aero_timeout_has_no_leg_deployment():
    cfg = compose_cfg(
        [
            "scenario=reentry_to_landing",
            "plant/vehicle=falcon9_octaweb",
            "scenario.start_phase=aero",
            "scenario.ic.r_ned=[0,0,-8000]",
            "scenario.ic.v_ned=[80,20,200]",
            "scenario.horizon.t_max=5",
        ]
    )
    bundle, result = run_configured(cfg)
    assert np.all(_alt(result) > cfg.run.leg_deploy_alt_m)
    assert result.landing_class is None
    doc = build_replay_doc(result, bundle, cfg)
    assert doc["meta"]["leg_deploy"]["t"] is None
    assert doc["meta"]["leg_deploy"]["visual_only"] is True


def test_rcs_display_preserves_short_burns_sign_and_failed_nozzles():
    cfg = compose_cfg(["scenario=free_rotation", "scenario.horizon.t_max=0.2"])
    cfg.scenario.horizon.log_dt = 0.1
    cfg.scenario.inputs = [
        {"t": 0, "rcs": [0, 0, 0]},
        {"t": 0.021, "rcs": [0.25, 0, 0]},
        {"t": 0.043, "rcs": [-0.25, 0, 0]},
        {"t": 0.065, "rcs": [0, 0, 0]},
    ]
    cfg.plant.rcs.thrust_scale = [0] + [1] * 11
    bundle, result = run_configured(cfg)
    doc = build_replay_doc(result, bundle, cfg, max_frames=2)
    rows = doc["meta"]["rcs_activity"]
    np.testing.assert_allclose([r["t"] for r in rows], [0, 0.021, 0.043, 0.065, 0.2])
    assert np.flatnonzero(rows[1]["duty"]).tolist() == [1]  # positive pair, one failed
    assert np.flatnonzero(rows[2]["duty"]).tolist() == [2, 3]  # opposite pair
    np.testing.assert_array_equal(rows[3]["duty"], 0)
    np.testing.assert_allclose(
        doc["meta"]["vehicle"]["rcs"]["positions"], bundle.params.rcs.positions
    )
    # Exhaust must oppose the force directions exported from the physical plant.
    np.testing.assert_allclose(
        doc["meta"]["vehicle"]["rcs"]["directions"], bundle.params.rcs.directions
    )


def test_empty_rcs_tank_suppresses_commanded_display_activity():
    cfg = compose_cfg(
        ["scenario=free_rotation", "scenario.horizon.t_max=0.1", "scenario.ic.rcs_fuel=0"]
    )
    cfg.scenario.inputs = [{"t": 0, "rcs": [1, 1, 1]}]
    bundle, result = run_configured(cfg)
    doc = build_replay_doc(result, bundle, cfg)
    assert all(not any(r["duty"]) for r in doc["meta"]["rcs_activity"])
    cfg.scenario.ic.rcs_fuel = 5
    bundle, result = run_configured(cfg)
    doc = build_replay_doc(result, bundle, cfg)
    assert any(doc["meta"]["rcs_activity"][-1]["duty"])  # timeout must not invent shutdown
