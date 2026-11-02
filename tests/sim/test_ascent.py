"""Ascent integration: attached mass, one-way pad release, cutoff and observable errors."""

import numpy as np
import pytest
from conftest import compose_cfg

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.run import run_configured
from validation.flight_reference import compare_flight_reference, error_statistics


def test_attached_mass_is_not_propellant_and_parallel_axis_is_applied():
    cfg = compose_cfg(["scenario=ascent"])
    b = build_plant(cfg)
    dry, attached = 25000.0, 120000.0
    cg = (dry * -58 + attached * -14) / (dry + attached)
    expected_i = np.array([43000 + 194400, 3.7e6 + 7.94e6, 3.7e6 + 7.94e6])
    expected_i[1:] += dry * (-58 - cg) ** 2 + attached * (-14 - cg) ** 2
    assert b.params.mass.m_dry == dry + attached
    assert b.params.mass.x_cg_dry == pytest.approx(cg)
    np.testing.assert_allclose(b.params.mass.I_dry, expected_i)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), b.params)
    _, aux = b.native.create_session(state).observe()
    assert aux["main_propellant"] == 400000.0
    empty = b.native.mass_properties_diagnostics(dry + attached + 200, 200)
    assert empty["main_propellant"] == 0
    assert empty["x_cg"] < cg  # aft RCS reserve still contributes separately


def test_pad_hold_release_and_return_is_contact_not_second_hold():
    cfg = compose_cfg(["scenario=ascent", "plant/actuator=ideal"])
    cfg.scenario.ic.throttle0 = 0
    cfg.scenario.horizon.t_max = 3.0
    cfg.scenario.horizon.log_dt = 0.01
    cfg.scenario.inputs = [
        {"t": 0, "throttle": 0, "fin_deployment": 0},
        {"t": 0.1, "throttle": 1},
        {"t": 0.3, "throttle": 0},
    ]
    _, result = run_configured(cfg)
    on_pad = result.t <= 0.1 + 1e-9
    np.testing.assert_allclose(result.r_ned[on_pad, 2], 0, atol=1e-12)
    np.testing.assert_allclose(result.v_ned[on_pad], 0, atol=1e-12)
    assert np.min(result.r_ned[:, 2]) < -0.01
    assert result.landing_class is not None
    assert result.touchdown_index == len(result.t) - 1
    assert result.landing_speed > 0.1  # not the zero speed of the initial pad sample
    kinds = [event["kind"] for event in result.aux["events"]]
    assert kinds.count("liftoff") == 1
    assert kinds.count("ground_crossing") == 1


def test_no_liftoff_is_not_a_landing_and_pad_burn_consumes_fuel():
    cfg = compose_cfg(["scenario=ascent", "plant/actuator=ideal"])
    cfg.scenario.ic.throttle0 = 0.4
    cfg.scenario.horizon.t_max = 0.2
    cfg.scenario.inputs = [{"t": 0, "throttle": 0.4, "fin_deployment": 0}]
    _, result = run_configured(cfg)
    assert result.landing_class is None
    assert result.t[-1] == pytest.approx(0.2)
    np.testing.assert_allclose(result.r_ned[:, 2], 0, atol=1e-12)
    assert result.m[-1] < result.m[0]
    assert not any(event["kind"] == "liftoff" for event in result.aux["events"])


def test_ascent_cutoff_coast_and_reference_are_not_landing_validation():
    cfg = compose_cfg(["scenario=ascent"])
    _, result = run_configured(cfg)
    assert result.t[-1] == pytest.approx(150, abs=1e-7)
    assert result.landing_class is None
    assert result.navigation is None
    assert result.r_ned[-1, 2] < -1000
    assert all(aux["main_propellant"] > 0 for aux in result.aux["per_tick"])
    coast = result.t >= 147.0
    np.testing.assert_allclose(result.m[coast], result.m[coast][0], atol=1e-5)
    assert (
        max(
            aux["thrust_mag"]
            for t, aux in zip(result.t, result.aux["per_tick"], strict=True)
            if t >= 147
        )
        < 1e-6
    )
    report, arrays = compare_flight_reference(result, cfg.scenario.reference)
    assert report["status"] == "comparison_only"
    assert report["event_time_scores"] is None
    assert arrays["reference_time_s"][-1] <= 145
    assert report["checkpoints"]["meco_command"]["m"] > 145200


def test_time_weighted_reference_errors_exclude_missing_interval():
    # Both valid 1-second intervals have constant errors; the 9-second gap is excluded.
    stats = error_statistics([0, 1, 10, 11], [2, 2, 4, 4])
    assert stats["covered_duration_s"] == 2
    assert stats["bias"] == 3
    assert stats["rmse"] == pytest.approx(np.sqrt(10))
    assert error_statistics([0, 10], [1, 1])["rmse"] is None


def test_invalid_pad_initial_condition_is_rejected():
    cfg = compose_cfg(["scenario=ascent"])
    cfg.scenario.ic.v_ned = [0, 0, -1]
    with pytest.raises(ValueError, match="launch_pad"):
        run_configured(cfg)
