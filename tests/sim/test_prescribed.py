"""Physical and timing contracts of the shared prescribed-input runner."""

import numpy as np
import pytest
from conftest import compose_cfg

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.open_loop import PrescribedInputs
from sim.replay import build_replay_doc
from sim.run import run_configured
from sim.telemetry import write_physics, write_telemetry
from validation.analytical import validate_episode
from validation.convergence import check_convergence, compare_results


@pytest.mark.parametrize(
    "name",
    [
        "freefall_uniform",
        "freefall_inverse_square",
        "free_rotation",
        "vacuum_burn",
        "mass_depletion",
        "drag_descent",
    ],
)
def test_analytical_scenarios(name):
    cfg = compose_cfg([f"scenario={name}"])
    bundle, result = run_configured(cfg)
    report, _ = validate_episode(result, bundle, cfg)
    assert report["status"] == "passed", report
    assert result.navigation is None
    assert "solver" not in result.aux
    assert bundle.meta["n_engines"] == 9
    assert result.t[-1] == pytest.approx(cfg.scenario.horizon.t_max)


def test_event_between_logging_and_input_ticks_is_not_delayed(tmp_path):
    cfg = compose_cfg(["scenario=vacuum_burn", "scenario.horizon.t_max=0.1"])
    cfg.scenario.ic.throttle0 = 0
    cfg.scenario.inputs = [
        {"t": 0, "label": "off", "throttle": 0},
        {"t": 0.007, "label": "on", "throttle": [0.7] + [0] * 8},
        {"t": 0.023, "label": "off", "throttle": 0},
    ]
    bundle, r = run_configured(cfg)
    assert r.navigation is None
    assert r.aux["schedule"]["events_s"]["navigation"] == []
    np.testing.assert_allclose(
        r.aux["schedule"]["events_s"]["command_release"], [0, 0.007, 0.023], atol=1e-12
    )
    active = r.actuators.held.throttle[:, 0] > 0
    intervals = r.actuators.t_end - r.actuators.t_start
    assert intervals[active].sum() == pytest.approx(0.016, abs=1e-12)
    mdot = 0.7 * 981000 / (311 * 9.80665)
    assert r.m[0] - r.m[-1] == pytest.approx(mdot * 0.016, abs=1e-7)
    assert any(np.isclose(r.t, 0.007)) and any(np.isclose(r.t, 0.023))
    assert r.throttle_act[0, 0] == 0  # initial sample precedes first input
    doc = build_replay_doc(r, bundle, cfg, max_frames=2)
    assert doc["meta"]["guidance"] == "prescribed"
    assert doc["meta"]["phase_names"] == {0: "off", 1: "on", 2: "off"}
    assert any(np.isclose(f["t"], 0.007) for f in doc["frames"])
    write_telemetry(r, tmp_path)
    write_physics(r, tmp_path)
    assert not (tmp_path / "navigation.npz").exists()
    with np.load(tmp_path / "physics.npz") as data:
        assert data["inertia_diag"].shape == (len(r.t), 3)
        assert np.isnan(data["flight_path_angle_rad"][0])


@pytest.mark.parametrize(
    "rows",
    [
        [],
        [{"t": 0.001, "throttle": 0}],
        [{"t": 0}, {"t": 0.0005}],
        [{"t": 0}, {"t": 0}],
        [{"t": 0, "throttle": [0.7]}],
        [{"t": 0, "rcs": [2, 0, 0]}],
        [{"t": 0, "gimbal": float("nan")}],
        [{"t": 0, "fin_deployment": 0.5}],
        [{"t": 0, "throtle": 0.7}],
        [{"t": 0}, {"t": 1}],
    ],
)
def test_invalid_schedule_rejected(rows):
    bundle = build_plant(compose_cfg(["scenario=vacuum_burn"]))
    with pytest.raises(ValueError):
        PrescribedInputs(rows, bundle.params, 1.0)


def test_configuration_event_changes_force_without_jumping_state():
    cfg = compose_cfg(["scenario=aero_descent"])
    b = build_plant(cfg)
    s = initial_state(ic_from_scenario(cfg.scenario, 0), b.params)
    session = b.native.create_session(s)
    before, deployed = session.observe()
    session.set_fin_deployment(0)
    after, stowed = session.observe()
    for key in ("r_ned", "v_ned", "q_bn", "w_body", "m"):
        np.testing.assert_array_equal(before[key], after[key])
    np.testing.assert_array_equal(deployed["inertia_diag"], stowed["inertia_diag"])
    assert np.linalg.norm(deployed["F_fin_body"]) > 0
    np.testing.assert_array_equal(stowed["F_fin_body"], 0)
    np.testing.assert_allclose(
        np.asarray(deployed["M_body_total"]) - stowed["M_body_total"], deployed["M_fin_body"]
    )
    # Configuration is per session, and reset returns the standard deployed configuration.
    assert b.native.create_session(s).observe()[1]["fin_deployment"] == 1
    session.reset(s)
    assert session.observe()[1]["fin_deployment"] == 1


def test_deployment_replay_matches_physics():
    cfg = compose_cfg(["scenario=fin_deployment"])
    bundle, r = run_configured(cfg)
    d = np.array([a["fin_deployment"] for a in r.aux["per_tick"]])
    np.testing.assert_array_equal(d[r.t < 1 - 1e-8], 0)
    np.testing.assert_array_equal(d[r.t >= 1 - 1e-8], 1)
    doc = build_replay_doc(r, bundle, cfg, max_frames=10)
    assert doc["meta"]["configuration_events"]
    for f in doc["frames"]:
        assert f["fin_deployment"] == (0 if f["t"] < 1 - 1e-8 else 1)
    cfg.scenario.inputs[0].fin_deployment = 1
    _, deployed = run_configured(cfg)
    assert np.linalg.norm(r.r_ned[-1] - deployed.r_ned[-1]) > 0.01


def test_neutral_fin_table_cannot_be_stowed(tabulated_aero_cfg):
    b = build_plant(tabulated_aero_cfg)
    with pytest.raises(ValueError, match="neutral fins"):
        b.native.set_fin_deployment(0)


def test_initial_per_engine_throttle_and_rcs_budget():
    cfg = compose_cfg(["scenario=vacuum_burn", "scenario.ic.rcs_fuel=50"])
    b = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), b.params)
    assert state.rcs_fuel == 50
    np.testing.assert_array_equal(state.throttle_act, [0.7] + [0] * 8)
    assert b.native.create_session(state).observe()[1]["main_propellant"] == 14950


def test_refinement_and_quaternion_sign_equivalence():
    cfg = compose_cfg(["scenario=free_rotation", "scenario.horizon.t_max=0.2"])
    report = check_convergence(cfg)
    assert report["status"] == "passed"
    _, r = run_configured(cfg)
    import copy

    negated = copy.deepcopy(r)
    negated.q_bn *= -1
    assert (
        compare_results(r, negated, cfg.scenario.horizon.log_dt)["errors"]["attitude_rad"] < 1e-12
    )
    with pytest.raises(ValueError, match="decreasing"):
        check_convergence(cfg, [0.001, 0.002, 0.0005])


def test_analytical_validator_rejects_inapplicable_environment():
    cfg = compose_cfg(["scenario=freefall_uniform", "environment/atmosphere=ussa_poly"])
    b, r = run_configured(cfg)
    with pytest.raises(ValueError, match="vacuum"):
        validate_episode(r, b, cfg)
