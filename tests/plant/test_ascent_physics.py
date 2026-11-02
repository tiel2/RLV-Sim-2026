"""Independent ascent equations, support reactions and delivered engine forces."""

import numpy as np
import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf

from core.interfaces import ActuatorCommand
from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.replay import build_replay_doc
from sim.run import run_configured
from validation.ascent import compare, reference
from validation.references import mass_properties


def test_attached_mass_matches_independent_distributed_mass_quadrature():
    cfg = compose_cfg(["scenario=ascent"])
    b = build_plant(cfg)
    for propellant in [0, 20000, 200000, 400000]:
        mass, gas = 145200 + propellant, 200
        cg, inertia = mass_properties(cfg.plant.vehicle, mass, gas)
        actual = b.native.mass_properties_diagnostics(mass, gas)
        assert actual["x_cg"] == pytest.approx(cg, abs=1e-12)
        np.testing.assert_allclose(actual["inertia"], np.diag(inertia), rtol=1e-14)


def test_pad_reactions_balance_forces_and_torques_with_burning_engines():
    cfg = compose_cfg(["scenario=ascent", "plant/actuator=ideal"])
    b = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), b.params)
    command = ActuatorCommand(
        throttle=np.full(9, 0.4),
        gimbal=np.tile([0.02, 0], (9, 1)),
        fin=np.zeros(4),
        rcs=np.zeros(3),
    )
    derivative, aux = b.native.rhs(state, command, np.zeros(3))
    assert derivative[5] < 0  # mass decreases even under rigid support
    np.testing.assert_allclose(derivative[1:3], 0, atol=1e-12)
    np.testing.assert_allclose(derivative[3], 0, atol=1e-12)
    np.testing.assert_allclose(derivative[4], 0, atol=1e-12)
    assert np.linalg.norm(aux["M_cg_body"]) > 1
    np.testing.assert_allclose(
        aux["F_body_total"] + aux["F_gravity_body"] + aux["F_pad_body"], 0, atol=1e-8
    )
    np.testing.assert_allclose(aux["M_cg_body"] + aux["M_pad_body"], 0, atol=1e-8)


def test_realistic_ascent_events_match_independent_matrix_integration():
    cfg = compose_cfg(["scenario=ascent"])
    cfg.scenario.horizon.t_max = 6.0
    cfg.scenario.horizon.log_dt = 0.02
    cfg.scenario.inputs = [
        {"t": 0, "throttle": 1, "fin_deployment": 0},
        {"t": 0.5, "gimbal": [[0.002, 0]] * 9},
        {"t": 1.3, "gimbal": 0},
        {"t": 2, "throttle": 0.4},
        {"t": 4, "throttle": 0},
    ]
    _, actual = run_configured(cfg)
    expected = reference(OmegaConf.to_container(cfg, resolve=True), actual.t, rtol=1e-12)
    errors = compare(actual, expected)
    assert errors["r_ned"] < 1e-4
    assert errors["v_ned"] < 1e-5
    assert errors["q_bn"] < 1e-8
    assert errors["w_body"] < 1e-8
    assert errors["m"] < 1e-4
    event = next(e for e in actual.aux["events"] if e["kind"] == "liftoff")
    assert event["t"] == 0
    assert event["time_uncertainty_s"] == cfg.plant_dt


def test_delivered_force_and_replay_stop_plumes_after_fuel_depletion():
    cfg = compose_cfg(["scenario=mass_depletion"])
    b, result = run_configured(cfg)
    doc = build_replay_doc(result, b, cfg)
    final = doc["frames"][-1]
    assert result.aux["per_tick"][-1]["main_propellant"] == 0
    assert max(final["thr"]) > 0  # effective throttle can remain commanded on
    np.testing.assert_array_equal(final["engine_force_body"], 0)
    for aux in result.aux["per_tick"]:
        np.testing.assert_allclose(
            np.sum(aux["engine_force_body"], axis=0), aux["T_body"], atol=1e-8
        )


def test_ascent_replay_keeps_stack_geometry_and_launch_alignment():
    cfg = compose_cfg(["scenario=ascent"])
    cfg.scenario.horizon.t_max = 0.2
    cfg.scenario.inputs = [cfg.scenario.inputs[0]]
    b, result = run_configured(cfg)
    replay = build_replay_doc(result, b, cfg)
    vehicle = replay["meta"]["vehicle"]
    assert vehicle["attached_component"]["aft_x"] == -28
    assert vehicle["gimbal_idx"] == list(range(9))
    np.testing.assert_allclose(vehicle["engine_pos"], b.params.engines.positions)
    assert replay["meta"]["scene"]["kind"] == "land"
    first = replay["frames"][0]
    assert replay["meta"]["scene"]["ground_y"] == pytest.approx(-70 - first["xcg"])
    assert not replay["meta"]["leg_deploy"]["enabled"]
    assert all(f["fin_deployment"] == 0 for f in replay["frames"])
