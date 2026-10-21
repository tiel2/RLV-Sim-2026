"""Exact periods, synchronous records, delay semantics, and true terminal timestamps."""

from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import pytest
from conftest import compose_cfg

from core.frames import quat_mul
from core.interfaces import ActuatorCommand, Command, Phase
from gnc.build import build_gnc, build_nav
from gnc.navigation.identity import IdentityNav
from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.loop import run_closed_loop
from sim.replay import build_replay_doc
from sim.schedule import build_schedule
from sim.telemetry import navigation_errors, write_telemetry


class ScheduledController:
    def __init__(self, bundle, throttle=None):
        self.bundle = bundle
        self.throttle = throttle or (lambda t: 0.0)

    def initial_memory(self):
        return {}

    def guide(self, state, t, memory):
        return Command(
            phase=int(Phase.POWERED_DESCENT),
            mode=0,
            thrust_vector=np.zeros(3),
            q_ref=state.q_bn,
            tau_body=np.zeros(3),
            active_engines=np.ones(self.bundle.meta["n_engines"]),
            w_ref=np.zeros(3),
        ), memory

    def control_allocate(self, command, state, memory, t):
        return ActuatorCommand(
            throttle=np.full(self.bundle.meta["n_engines"], self.throttle(t)),
            gimbal=np.zeros((self.bundle.meta["n_gimbal"], 2)),
            fin=np.zeros(4),
            rcs=np.zeros(3),
        ), memory


def _case(actuator="ideal"):
    cfg = compose_cfg([f"plant/actuator={actuator}"])
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
    return cfg, bundle, state


def test_nondivisible_gnc_periods_and_log_grid_are_exact():
    _, bundle, state = _case()
    state = state.replace(t=np.asarray(5.0))
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle),
        IdentityNav(),
        t_max=0.23,
        plant_dt=0.001,
        nav_dt=0.02,
        ctrl_dt=0.05,
        guidance_dt=0.1,
        log_dt=0.07,
    )
    events = result.aux["schedule"]["events_s"]
    np.testing.assert_allclose(events["control"], 5.0 + np.arange(5) * 0.05, atol=1e-12)
    np.testing.assert_allclose(events["navigation"], 5.0 + np.arange(12) * 0.02, atol=1e-12)
    np.testing.assert_allclose(events["guidance"], [5.0, 5.1, 5.2], atol=1e-12)
    np.testing.assert_allclose(result.t, [5.0, 5.07, 5.14, 5.21, 5.23], atol=1e-12)
    np.testing.assert_allclose([row["t"] for row in result.aux["per_tick"]], result.t)
    np.testing.assert_allclose(result.navigation.truth.t, result.navigation.measurement.t)
    np.testing.assert_allclose(result.navigation.truth.t, result.navigation.estimate.t)


@pytest.mark.parametrize(
    "key,value",
    [
        ("nav_dt", 0.0015),
        ("ctrl_dt", 0.0),
        ("guidance_dt", float("nan")),
        ("log_dt", -0.1),
        ("t_max", 0.0105),
        ("plant_dt", 0.0),
    ],
)
def test_unrepresentable_or_invalid_periods_are_rejected(key, value):
    kwargs = dict(plant_dt=0.001, t_max=0.1, ctrl_dt=0.02)
    kwargs[key] = value
    with pytest.raises(ValueError, match=key):
        build_schedule(**kwargs)


def test_delay_is_in_control_periods_and_logs_distinct_commands():
    _, bundle, state = _case()
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle, lambda t: 0.5 + t),
        IdentityNav(),
        t_max=0.08,
        plant_dt=0.001,
        nav_dt=0.01,
        ctrl_dt=0.02,
        guidance_dt=0.02,
        log_dt=0.03,
        delay_ticks=2,
    )
    history = result.actuators
    before = history.t_start < 0.04 - 1e-10
    assert np.allclose(history.held.throttle[before], 0.0)
    index = np.flatnonzero(np.isclose(history.t_start, 0.04))[0]
    assert np.isclose(history.commanded.throttle[index, 0], 0.54)
    assert np.isclose(history.held.throttle[index, 0], 0.5)
    assert np.isclose(history.t_held[index], 0.0)
    assert np.isclose(history.t_command[index], 0.04)
    np.testing.assert_allclose(history.effective.throttle, history.held.throttle)
    assert result.aux["schedule"]["delay_s"] == 0.04


def test_realistic_effective_throttle_records_lag():
    _, bundle, state = _case("realistic")
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle, lambda t: 0.7),
        IdentityNav(),
        t_max=0.1,
        ctrl_dt=0.02,
        nav_dt=0.01,
        plant_dt=0.001,
    )
    history = result.actuators
    assert np.allclose(history.held.throttle, 0.7)
    assert np.all(history.effective.throttle > 0.0)
    assert np.all(history.effective.throttle < history.held.throttle)


def test_early_contact_uses_native_time_not_requested_block_end():
    _, bundle, state = _case()
    state = state.replace(r_ned=np.array([0.0, 0.0, -0.002]), v_ned=np.array([0.0, 0.0, 1.0]))
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle),
        IdentityNav(),
        t_max=0.1,
        ctrl_dt=0.02,
        plant_dt=0.001,
        log_dt=0.1,
    )
    np.testing.assert_allclose(result.t, [0.0, 0.002], atol=1e-12)
    assert result.aux["per_tick"][-1]["t"] == result.t[-1]
    assert result.actuators.t_end[-1] == result.t[-1]


def test_navigation_export_is_full_rate_and_retains_known_error(tmp_path):
    _, bundle, state = _case()
    nav = IdentityNav(pos_bias=[1.0, 2.0, 3.0], vel_bias=[0.5, 0.0, 0.0])
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle),
        nav,
        t_max=0.1,
        ctrl_dt=0.02,
        nav_dt=0.01,
        log_dt=0.05,
        plant_dt=0.001,
    )
    paths = write_telemetry(result, tmp_path)
    with np.load(paths["navigation"]) as data:
        assert len(data["truth_t"]) == 10 > len(result.t)
        np.testing.assert_allclose(data["error_r_ned"], np.tile([1.0, 2.0, 3.0], (10, 1)))
        np.testing.assert_array_equal(data["truth_t"], data["estimate_t"])
        np.testing.assert_allclose(data["error_attitude_rad"], 0.0)
    stats = json.loads(paths["navigation_stats"].read_text())
    assert np.isclose(stats["r_ned"]["rmse"], np.sqrt(14.0))
    with np.load(paths["actuators"]) as data:
        assert len(data["t_end"]) == len(result.actuators.t_end)
        assert data["t_end"][-1] == result.t[-1]


def test_stale_estimate_epoch_is_not_compared_to_current_truth():
    class StaleNav(IdentityNav):
        def estimate(self, meas, act, memory):
            return meas.replace(t=0.0), memory

    _, bundle, state = _case()
    with pytest.raises(ValueError, match="epoch"):
        run_closed_loop(
            bundle,
            state,
            ScheduledController(bundle),
            StaleNav(),
            t_max=0.04,
            ctrl_dt=0.02,
            plant_dt=0.001,
        )


def test_attitude_error_handles_quaternion_sign_and_small_rotations():
    _, bundle, state = _case()
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle),
        IdentityNav(),
        t_max=0.04,
        ctrl_dt=0.02,
        plant_dt=0.001,
    )
    history = result.navigation
    signed = replace(history, estimate=replace(history.estimate, q_bn=-history.truth.q_bn))
    np.testing.assert_allclose(navigation_errors(signed)["attitude_rad"], 0.0, atol=1e-15)
    angle = 1e-7
    delta = np.array([np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0])
    rotated = np.stack([quat_mul(q, delta) for q in history.truth.q_bn])
    biased = replace(history, estimate=replace(history.estimate, q_bn=rotated))
    np.testing.assert_allclose(navigation_errors(biased)["attitude_rad"], angle, atol=1e-14)


def test_reference_pdg_lands_with_zero_identity_navigation_error(tmp_path):
    cfg = compose_cfg(["+baseline=pdg"])
    assert cfg.baseline_id == "falcon9_pdg_v2"
    assert cfg.seed == 0 and not cfg.scenario.dispersion.enable
    assert cfg.plant.actuator.model == "ideal"
    assert cfg.environment.wind.model == "none"
    assert cfg.scenario.ic.m == 29500.0
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, cfg.seed), bundle.params)
    h = cfg.scenario.horizon
    result = run_closed_loop(
        bundle,
        state,
        build_gnc(cfg, bundle.params),
        build_nav(cfg),
        t_max=h.t_max,
        ctrl_dt=h.ctrl_dt,
        nav_dt=h.nav_dt,
        guidance_dt=h.guidance_dt,
        log_dt=h.log_dt,
        plant_dt=cfg.plant_dt,
    )
    assert result.success and result.landing_class == "soft"
    assert result.landing_speed < 1.0
    assert result.t[0] == 0.0
    np.testing.assert_array_equal(result.r_ned[0], [0.0, 0.0, -300.0])
    for error in navigation_errors(result.navigation).values():
        np.testing.assert_allclose(error, 0.0, atol=1e-12)
    write_telemetry(result, tmp_path)
    replay = build_replay_doc(result, bundle, cfg)
    assert replay["frames"][0]["t"] == 0.0
    assert np.isclose(replay["meta"]["dt_nominal"], h.log_dt)
    np.testing.assert_allclose([row["alt"] for row in replay["frames"]], -result.r_ned[:, 2])
    assert max(row["thr"][0] for row in replay["frames"]) > 0.4
    repeat = run_closed_loop(
        bundle,
        state,
        build_gnc(cfg, bundle.params),
        build_nav(cfg),
        t_max=h.t_max,
        ctrl_dt=h.ctrl_dt,
        nav_dt=h.nav_dt,
        guidance_dt=h.guidance_dt,
        log_dt=h.log_dt,
        plant_dt=cfg.plant_dt,
        record_telemetry=False,
    )
    assert repeat.navigation is None and repeat.actuators is None
    for key in ("t", "r_ned", "v_ned", "q_bn", "w_body", "m"):
        np.testing.assert_array_equal(getattr(result, key), getattr(repeat, key))


def test_in_place_estimator_preserves_truth_and_original_measurement():
    class InPlaceNav(IdentityNav):
        def estimate(self, meas, act, memory):
            meas.r_ned[0] += 10.0
            return meas, memory

    _, bundle, state = _case()
    result = run_closed_loop(
        bundle,
        state,
        ScheduledController(bundle),
        InPlaceNav(),
        t_max=0.04,
        ctrl_dt=0.02,
        plant_dt=0.001,
    )
    history = result.navigation
    np.testing.assert_allclose(history.truth.r_ned[:, 0], 0.0, atol=1e-12)
    np.testing.assert_allclose(history.measurement.r_ned[:, 0], 0.0, atol=1e-12)
    np.testing.assert_allclose(history.estimate.r_ned[:, 0], 10.0)
    np.testing.assert_allclose(result.r_ned[:, 0], 0.0, atol=1e-12)
