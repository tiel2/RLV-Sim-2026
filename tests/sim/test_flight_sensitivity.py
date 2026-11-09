"""Sensitivity isolation, missing-coverage handling and independent input replay."""

import numpy as np
import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf

from sim.run import run_configured
from sim.telemetry import physics_arrays
from validation.ascent import compare
from validation.flight_sensitivity import (
    actuator_arrays,
    common_metrics,
    perturb,
    physical_audit,
)
from validation.landing_input_reference import reference
from validation.references import G0


def test_probe_preserves_target_speed_norm_and_carries_mass_only_once():
    cfg = compose_cfg(["scenario=flight_reconstructed_landing"])
    original = OmegaConf.to_container(cfg, resolve=True)
    probes = OmegaConf.load("configs/analysis/flight_sensitivity.yaml").probes
    changed = perturb(cfg, {"mass": 1, "direction": 1}, probes, carried_mass=31000, carried_gas=199)
    assert changed.scenario.ic.m == 31000  # do not apply +1% at every handoff
    assert changed.scenario.ic.rcs_fuel == 199
    assert changed.scenario.ic.r_ned == cfg.scenario.ic.r_ned
    assert changed.gnc == cfg.gnc
    assert np.linalg.norm(changed.scenario.ic.v_ned) == pytest.approx(
        np.linalg.norm(cfg.scenario.ic.v_ned)
    )
    assert changed.scenario.ic.v_ned != cfg.scenario.ic.v_ned
    assert OmegaConf.to_container(cfg, resolve=True) == original


def test_common_metrics_do_not_bridge_a_missing_interval_or_hide_lost_coverage():
    nominal = dict(source_row=np.arange(4), reference_time_s=np.arange(4, dtype=float))
    for key in ("altitude_m", "speed_m_s"):
        nominal.update(
            {
                f"{key}_score_eligible": np.ones(4, dtype=bool),
                f"{key}_interval_eligible": np.array([False, True, True, True]),
                f"error_{key}": np.array([1.0, 1.0, 100.0, 100.0]),
            }
        )
    candidate = {k: v.copy() for k, v in nominal.items()}
    for key in ("altitude_m", "speed_m_s"):
        candidate[f"{key}_score_eligible"][2:] = False
    metric = common_metrics(nominal, candidate)["altitude_m"]
    assert metric["nominal"]["covered_duration_s"] == 1
    assert metric["nominal"]["rmse"] == 1
    candidate["altitude_m_interval_eligible"][1] = False
    assert common_metrics(nominal, candidate)["altitude_m"]["probe"]["rmse"] is None
    candidate["source_row"][0] = 99
    with pytest.raises(ValueError, match="source rows"):
        common_metrics(nominal, candidate)


@pytest.fixture(scope="module")
def short_landing():
    cfg = compose_cfg(["scenario=flight_reconstructed_landing"])
    cfg.scenario.horizon.t_max = 0.2
    _, result = run_configured(cfg)
    return cfg, result, physics_arrays(result), actuator_arrays(result.actuators)


def test_independent_replay_preserves_analytic_fuel_budget_and_native_states(short_landing):
    cfg, result, _, a = short_landing
    expected = reference(OmegaConf.to_container(cfg, resolve=True), a, result.t, rtol=1e-12)
    errors = compare(result, expected)
    assert errors["r_ned"] < 1e-5
    assert errors["v_ned"] < 1e-6
    assert errors["w_body"] < 1e-7
    assert errors["q_bn"] < 1e-7
    engine = cfg.plant.vehicle.engines
    flow_integral = np.dot(a["t_end"] - a["t_start"], a["held_throttle"][:, 0])
    expected_mass = cfg.scenario.ic.m - flow_integral * engine.thrust_vac / (G0 * engine.isp)
    assert expected["m"][-1] == pytest.approx(expected_mass, abs=1e-8)
    assert errors["m"] < 1e-7


def test_input_replay_rejects_command_gaps_and_unsupported_actuation(short_landing):
    cfg, result, _, a = short_landing
    constants = OmegaConf.to_container(cfg, resolve=True)
    altered = {k: v.copy() for k, v in a.items()}
    altered["t_start"][2] += 0.001
    with pytest.raises(ValueError, match="contiguous"):
        reference(constants, altered, result.t)
    constants["plant"]["actuator"]["model"] = "realistic"
    with pytest.raises(ValueError, match="requires healthy ideal"):
        reference(constants, a, result.t)


def test_physical_audit_detects_throttle_violation_without_enforcing_ideal_slew(short_landing):
    cfg, _, p, a = short_landing
    report = physical_audit(p, a, cfg)
    assert report["failed_checks"] == []
    assert not report["actuator_rates"]["throttle"]["enforced"]
    bad = {k: v.copy() for k, v in a.items()}
    bad["effective_throttle"][1] = 0.2
    assert "effective_throttle_floor" in physical_audit(p, bad, cfg)["failed_checks"]
    bad["held_throttle"][1] = 0.2
    assert "held_throttle_floor" in physical_audit(p, bad, cfg)["failed_checks"]
