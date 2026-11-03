"""Continuous mission, actual cutoff, fuel reserves, failed guards and reference geometry."""

from types import SimpleNamespace

import numpy as np
import pytest
from conftest import compose_cfg
from scipy.spatial.transform import Rotation

from core.interfaces import TerminationReason
from gnc.guidance.boostback import BoostbackMission, Slew, ballistic_projection, pointing_target
from plant.separation import Separation
from sim.replay import build_replay_doc
from sim.run import run_configured


@pytest.fixture(scope="module")
def episode():
    cfg = compose_cfg(["scenario=ascent_boostback"])
    bundle, result = run_configured(cfg, record_telemetry=False)
    return cfg, bundle, result


def handoff(episode):
    cfg, _, result = episode
    separation = Separation(cfg)
    event = next(e for e in result.aux["events"] if e["kind"] == "stage_separation")
    state = SimpleNamespace(
        **{k: np.asarray(v) if isinstance(v, list) else v for k, v in event["after"].items()}
    )
    state.key = np.array([0, 1], dtype=np.uint32)
    session = separation.bundle.native.create_session(state)
    session.set_fin_deployment(0)
    snapshot, aux = session.observe()
    mission = BoostbackMission(cfg.scenario.mission.copy(), separation.bundle, 0.02)
    return mission, SimpleNamespace(**snapshot), aux


def test_continuous_maneuver_burn_cutoff_and_reserves(episode):
    cfg, bundle, result = episode
    summary = result.aux["mission"]
    assert summary["completed"] and summary["failure"] is None
    assert summary["cutoff_reason"] == "main_reserve"
    assert not summary["target_projection_met"]  # insufficient return capability stays explicit
    assert not summary["landing_validated"]
    assert result.termination_reason == TerminationReason.MISSION_COMPLETE
    transitions = [e for e in result.aux["events"] if e["kind"] == "mission_transition"]
    assert [e["phase_after"] for e in transitions] == [10, 11, 12, 13, 14, 15, 16]
    assert np.all(np.diff(result.t) > 0)
    for event in transitions:
        k = np.argmin(abs(result.t - event["t"]))
        assert abs(result.t[k] - event["t"]) < 1e-7
        assert result.phase[k] == event["phase_after"]
        for key in ("r_ned", "v_ned", "q_bn", "w_body", "m"):
            np.testing.assert_allclose(getattr(result, key)[k], event["state"][key], atol=1e-10)
    records = result.aux["mission_records"]
    assert min(r["rcs_fuel_kg"] for r in records) >= cfg.scenario.mission.rcs_reserve_kg
    assert min(r["main_propellant_kg"] for r in records) >= cfg.scenario.mission.main_reserve_kg
    assert max(r["rate_rad_s"] for r in records) < 1.25 * cfg.scenario.mission.max_rate_rad_s
    ignite = next(e for e in transitions if e["phase_after"] == 13)
    before = [
        r for r in records if ignite["t"] - cfg.scenario.mission.hold_s <= r["t"] <= ignite["t"]
    ]
    assert max(r["attitude_error_rad"] for r in before) <= np.deg2rad(
        cfg.scenario.mission.attitude_tolerance_deg
    )
    assert max(r["rate_rad_s"] for r in before) <= cfg.scenario.mission.rate_tolerance_rad_s
    after = result.t >= next(e["t"] for e in transitions if e["phase_after"] == 15)
    assert (
        max(
            a["thrust_mag"]
            for a, selected in zip(result.aux["per_tick"], after, strict=True)
            if selected
        )
        < 1e-6
    )
    for a, t in zip(result.aux["per_tick"], result.t, strict=True):
        if ignite["t"] + 0.8 < t < ignite["t"] + 2:
            assert list(np.flatnonzero(np.linalg.norm(a["engine_force_body"], axis=1) > 1)) == [
                0,
                1,
                5,
            ]
        assert a["fin_deployment"] == 0
    replay = build_replay_doc(result, bundle, cfg, max_frames=50)
    assert replay["meta"]["guidance"] == "mission"
    assert not replay["meta"]["leg_deploy"]["enabled"]
    assert replay["meta"]["mission"]["completed"]
    for event in transitions:
        assert any(
            abs(f["t"] - event["t"]) < 1e-7 and f["phase"] == event["phase_after"]
            for f in replay["frames"]
        )


@pytest.mark.parametrize("failure", ["rcs_reserve", "maneuver_timeout", "angular_rate_limit"])
def test_failed_guard_cannot_ignite(episode, failure):
    mission, state, aux = handoff(episode)
    if failure == "rcs_reserve":
        aux["rcs_fuel"] = mission.c.rcs_reserve_kg
    elif failure == "angular_rate_limit":
        state.w_body = np.array([1.0, 0, 0])
    else:
        mission.update(state, aux)
        state.t += mission.c.maneuver_timeout_s + 0.02
    command = mission.update(state, aux)
    assert mission.failure == failure and mission.phase == 17
    np.testing.assert_array_equal(command.throttle, 0)
    np.testing.assert_array_equal(command.rcs, 0)


def test_pointing_and_slew_large_angle_and_antipodal_quaternion():
    initial = Rotation.from_euler("ZYX", [30, 60, 15], degrees=True).as_quat(scalar_first=True)
    target = pointing_target(initial, [-1, -0.5, 0])
    np.testing.assert_allclose(
        Rotation.from_quat(target, scalar_first=True).apply([1, 0, 0]),
        np.array([-1, -0.5, 0]) / np.sqrt(1.25),
        atol=1e-12,
    )
    profile = Slew(initial, -target, 0.014, 0.0007)
    samples = [profile.sample(t) for t in np.linspace(0, profile.duration, 1001)]
    assert max(np.linalg.norm(s[1]) for s in samples) <= 0.014 + 1e-14
    assert max(np.linalg.norm(s[2]) for s in samples) <= 0.0007 + 1e-14
    assert (
        Rotation.from_quat(samples[-1][0], scalar_first=True).inv()
        * Rotation.from_quat(target, scalar_first=True)
    ).magnitude() < 1e-12
    np.testing.assert_array_equal(samples[-1][1], 0)
    time, position = ballistic_projection([0, 0, -100], [10, 0, 0], 10, 20)
    assert time == pytest.approx(4)
    np.testing.assert_allclose(position, [40, 0])


def test_insufficient_slew_budget_reports_failure(episode):
    mission, state, aux = handoff(episode)
    aux["rcs_fuel"] = mission.c.rcs_reserve_kg + 5
    state.w_body = np.zeros(3)
    mission.update(state, aux)
    state.t += mission.c.hold_s + 0.02
    command = mission.update(state, aux)
    assert mission.failure == "insufficient_slew_budget"
    np.testing.assert_array_equal(command.throttle, 0)


@pytest.mark.parametrize("cutoff_mode", ["delta_v", "duration", "target"])
def test_burn_cutoff_modes_and_shutdown_are_distinct(episode, cutoff_mode):
    mission, state, aux = handoff(episode)
    mission.update(state, aux)
    mission.c.cutoff_mode = cutoff_mode
    mission.phase = 13
    mission.phase_start = mission.burn_start = state.t
    mission.q_ref = np.array(state.q_bn)
    mission.burn_dv = np.array([mission.c.delta_v_mps + 1, 0, 0])
    if cutoff_mode == "duration":
        state.t += mission.c.burn_timeout_s
    if cutoff_mode == "target":
        prediction = mission._projection(state, aux)
        mission.c.target_ne_m = prediction[1].tolist()
    aux["thrust_mag"] = 1000.0
    command = mission.update(state, aux)
    assert mission.phase == 14 and not mission.complete
    np.testing.assert_array_equal(command.throttle, 0)
    assert mission.cutoff_reason in (
        "prescribed_delta_v_reached",
        "prescribed_duration_reached",
        "projected_target_reached",
    )
    aux["thrust_mag"] = 0.0
    state.t += 0.02
    mission.update(state, aux)
    assert mission.phase == 15
