"""Continuous ascent handoff, event sampling, replay and fuel accounting."""

import json

import numpy as np
import pytest
from conftest import compose_cfg

from plant.dynamics import build_plant
from plant.separation import Separation
from sim.open_loop import PrescribedInputs
from sim.replay import build_replay_doc
from sim.run import run_configured
from sim.telemetry import write_physics
from validation.separation import event_balances


@pytest.fixture(scope="module")
def episode():
    cfg = compose_cfg(["scenario=ascent_separation"])
    bundle, result = run_configured(cfg, record_telemetry=False)
    return cfg, bundle, result


def test_ascent_separation_handoff_and_mass_accounting(episode, tmp_path):
    cfg, bundle, result = episode
    event = next(e for e in result.aux["events"] if e["kind"] == "stage_separation")
    assert max(event_balances(event).values()) < 1e-5
    assert result.t[-1] == pytest.approx(160, abs=1e-7)
    assert np.all(np.diff(result.t) > 0)
    assert result.landing_class is None
    k = np.argmin(abs(result.t - event["t"]))
    for key in ("r_ned", "v_ned", "q_bn", "w_body", "m"):
        np.testing.assert_allclose(getattr(result, key)[k], event["after"][key])
    assert result.aux["per_tick"][k - 1]["configuration"] == 0
    assert result.aux["per_tick"][k]["configuration"] == 1
    np.testing.assert_allclose(result.m[k:], result.m[k], atol=1e-8)
    assert result.fuel_margin == pytest.approx(event["before"]["main_propellant"])
    # Compare with the existing ascent's actual cutoff/coast state, not a new IC.
    ascent_cfg = compose_cfg(["scenario=ascent"])
    _, ascent = run_configured(ascent_cfg, record_telemetry=False)
    for key in ("r_ned", "v_ned", "q_bn", "w_body", "m"):
        np.testing.assert_allclose(getattr(ascent, key)[-1], event["before"][key], atol=1e-10)
    write_physics(result, tmp_path)
    metrics = json.loads((tmp_path / "metrics.json").read_text())
    assert metrics["separated_mass_kg"] == 120000
    assert metrics["propellant_consumed_kg"] == pytest.approx(400000 - result.fuel_margin)


def test_replay_retains_both_event_limits_when_downsampled(episode):
    cfg, bundle, result = episode
    replay = build_replay_doc(result, bundle, cfg, max_frames=12)
    frames = [f for f in replay["frames"] if abs(f["t"] - 150) < 1e-7]
    assert len(frames) == 2
    assert [f["configuration"] for f in frames] == [0, 1]
    assert frames[0]["mass"] - frames[1]["mass"] == pytest.approx(120000)
    assert len(replay["meta"]["vehicles"]) == 2
    stack, booster = replay["meta"]["vehicles"]
    assert stack["length"] == 70 and booster["length"] == 42
    assert booster["attached_component"] is None
    assert len(booster["engine_pos"]) == len(stack["engine_pos"]) == 9
    np.testing.assert_allclose(np.array(stack["engine_pos"]) + [28, 0, 0], booster["engine_pos"])


def test_separation_replay_uses_effective_ideal_actuators():
    cfg = compose_cfg(["scenario=ascent_separation", "plant/actuator=ideal"])
    cfg.scenario.ic.launch_pad = False
    cfg.scenario.ic.r_ned = [0, 0, -1000]
    cfg.scenario.horizon.t_max = 0.2
    cfg.scenario.inputs = [
        {"t": 0, "throttle": 0, "gimbal": 0.01, "fin_deployment": 0},
        {"t": 0.1, "separate": True},
    ]
    bundle, result = run_configured(cfg, record_telemetry=False)
    replay = build_replay_doc(result, bundle, cfg)
    pre = next(f for f in replay["frames"] if f.get("sample_side") == "before_event")
    np.testing.assert_array_equal(pre["thr"], np.zeros(9))
    np.testing.assert_allclose(pre["gim"], np.full((9, 2), 0.01))
    assert pre["thrust"] == 0


@pytest.mark.parametrize(
    "rows",
    [
        [{"t": 0}, {"t": 0.1, "separate": True}, {"t": 0.2, "separate": True}],
        [{"t": 0, "separate": True}],
        [{"t": 0}, {"t": 0.1, "separate": "true"}],
        [{"t": 0}],
    ],
)
def test_invalid_separation_schedules_fail_before_execution(rows):
    cfg = compose_cfg(["scenario=ascent_separation"])
    with pytest.raises(ValueError, match="separat"):
        PrescribedInputs(rows, build_plant(cfg).params, 1, separation=Separation(cfg))
