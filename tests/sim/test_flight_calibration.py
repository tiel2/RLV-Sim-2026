"""Guard identification against missing-coverage rewards and inconsistent mass handoffs."""

from types import SimpleNamespace

import numpy as np
import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf

from sim.run import run_configured
from validation.flight_calibration import (
    ObservationResidual,
    physical_screen,
    propagate_candidates,
    recovery_spec,
    time_weights,
)
from validation.flight_reference import compare_flight_reference


def test_weights_do_not_bridge_excluded_rows_or_gaps():
    assert time_weights([0, 1, 2, 3, 10, 11], [True, True, False, True, True, True]) == (
        pytest.approx([0.25, 0.25, 0, 0, 0.25, 0.25])
    )
    with pytest.raises(ValueError, match="no eligible"):
        time_weights([0, 2], [True, True])


def test_residual_matches_frozen_policy_and_rejects_shortened_trajectory():
    cfg = compose_cfg(["scenario=flight_reconstructed_aero_descent"])
    _, result = run_configured(cfg, record_telemetry=False)
    residual = ObservationResidual(result, cfg.scenario.reference)
    values = residual(result)
    report, _ = compare_flight_reference(result, cfg.scenario.reference)
    assert values @ values / 2 == pytest.approx(report["normalized_reconstruction_mse"])
    shortened = SimpleNamespace(
        t=result.t[: len(result.t) // 2],
        v_ned=result.v_ned[: len(result.t) // 2],
        aux={"per_tick": result.aux["per_tick"][: len(result.t) // 2]},
    )
    with pytest.raises(ValueError, match="frozen observation support"):
        residual(shortened)


def test_recovery_preserves_scalar_speed_and_propagates_actual_fuel():
    recipe = OmegaConf.load("configs/analysis/flight_calibration.yaml")
    cfgs = [
        compose_cfg([f"scenario=flight_reconstructed_{name}"])
        for name in ["entry_burn", "aero_descent", "powered_descent"]
    ]
    x, _, make, _ = recovery_spec(cfgs, recipe)
    x[3:6] += 2.0
    candidates = make(x)
    for old, new in zip(cfgs, candidates, strict=True):
        assert np.linalg.norm(new.scenario.ic.v_ned) == pytest.approx(
            np.linalg.norm(old.scenario.ic.v_ned)
        )
    runs = propagate_candidates(candidates, couple_mass=True)
    for (cfg, r), (next_cfg, _) in zip(runs[:-1], runs[1:], strict=True):
        assert physical_screen(cfg, r)
        assert next_cfg.scenario.ic.m == r.m[-1]
        assert next_cfg.scenario.ic.rcs_fuel == r.aux["per_tick"][-1]["rcs_fuel"]


def test_identification_can_use_the_late_input_knot_without_changing_evaluation_policy():
    cfg = compose_cfg(["scenario=flight_reconstructed_powered_descent"])
    _, result = run_configured(cfg, record_telemetry=False)
    evaluation = ObservationResidual(result, cfg.scenario.reference)
    identification = ObservationResidual(
        result, cfg.scenario.reference, include_initialization_support=True
    )
    key = "speed_m_s"
    times = identification.arrays["simulation_time_s"]
    # The next phase's initialization support masks this phase's last six seconds
    # in descriptive evaluation. Offline identification must still constrain them.
    assert times[evaluation.weights[key] > 0].max() < 18
    assert times[identification.weights[key] > 0].max() > 23
    assert identification.weights[key][times == 0].sum() == 0


def test_exported_calibration_improves_both_channels_and_preserves_recovery_mass():
    previous, coefficients = None, None
    names = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]
    for name in names:
        original = compose_cfg([f"scenario=flight_reconstructed_{name}"])
        cfg = compose_cfg([f"scenario=flight_calibrated_{name}"])
        assert cfg.scenario.input_mode == ("closed_loop" if name == "landing" else "prescribed")
        assert cfg.plant.actuator.model == "realistic"
        if name != "ascent":
            if coefficients is not None:
                assert cfg.plant.aero.coeffs == coefficients
            coefficients = cfg.plant.aero.coeffs
            if previous is not None:
                assert cfg.scenario.ic.m == pytest.approx(previous.m[-1], abs=1e-6)
                assert cfg.scenario.ic.rcs_fuel == pytest.approx(
                    previous.aux["per_tick"][-1]["rcs_fuel"], abs=1e-6
                )
        _, before = run_configured(original, record_telemetry=False)
        _, after = run_configured(cfg, record_telemetry=False)
        assert physical_screen(cfg, after)
        old, _ = compare_flight_reference(before, original.scenario.reference)
        new, _ = compare_flight_reference(after, cfg.scenario.reference)
        for key in ["altitude_m", "speed_m_s"]:
            old_channel = old["metrics"]["requested_overlap"][key]
            new_channel = new["metrics"]["requested_overlap"][key]
            assert new_channel["rmse"] < old_channel["rmse"]
            assert new_channel["covered_duration_s"] >= old_channel["covered_duration_s"] - 1e-8
        if name != "ascent":
            previous = after
    assert after.success and after.landing_class == "soft"
    assert after.landing_speed < 1.0
    assert after.landing_position_error < 0.1
    assert after.aux["control"]["n_calls"] > 100
    assert after.aux["solver"]["n_solves"] > 0
    assert after.aux["plan_ignition"]["tf"] == pytest.approx(12.145)
    assert not cfg.gnc.guidance.convex_terminal_slack
    assert cfg.gnc.guidance.convex_terminal_vz_max_mps == 0.5
    assert cfg.gnc.guidance.landing_cutoff_impact_mps == 0
    assert "inputs" not in cfg.scenario
    assert not new["comparison_ends_early"]
    assert new["metrics"]["requested_overlap"]["speed_m_s"]["covered_duration_s"] > 12


@pytest.mark.parametrize("pitch", [89.0, 91.0])
def test_calibrated_closed_loop_recovers_from_initial_attitude_error(pitch):
    cfg = compose_cfg(["scenario=flight_calibrated_landing"])
    cfg.scenario.ic.pitch_deg = pitch
    _, result = run_configured(cfg, record_telemetry=False)
    assert result.success and result.landing_class == "soft"
    assert result.landing_speed < 1.0
    assert result.landing_position_error < 0.1
    assert physical_screen(cfg, result)


def test_calibrated_landing_commands_respond_to_current_velocity_and_rate():
    import copy

    from gnc.build import build_gnc
    from plant.dynamics import build_plant, ic_from_scenario, initial_state

    cfg = compose_cfg(["scenario=flight_calibrated_landing"])
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, cfg.seed), bundle.params)
    gnc = build_gnc(cfg, bundle.params)
    cmd, memory = gnc.guide(state, 0.0, gnc.initial_memory())
    nominal, _ = gnc.control_allocate(cmd, state, copy.deepcopy(memory), t=0.02)
    perturbed = state.replace(
        v_ned=state.v_ned + [0.0, 0.0, 1.0], w_body=state.w_body + [0.0, 0.01, 0.0]
    )
    corrected, _ = gnc.control_allocate(cmd, perturbed, copy.deepcopy(memory), t=0.02)
    assert corrected.throttle[0] > nominal.throttle[0] + 0.01
    assert np.linalg.norm(corrected.gimbal - nominal.gimbal) > 1e-5
