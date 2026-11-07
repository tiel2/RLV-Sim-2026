"""Analytic comparator anchors and real-data initialization contracts, without fitting."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.open_loop import PrescribedInputs
from sim.run import run_configured
from validation.flight_comparison_policy import interpolate_simulation, masked_statistics
from validation.flight_observations import prepare_observations
from validation.flight_reference import compare_flight_reference, write_flight_comparison
from validation.flight_scenarios import (
    initialization_mask,
    reconstruct_initial_states,
    scenario_fragment,
)

ROOT = Path(__file__).resolve().parents[2]
PHASES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]


@pytest.fixture
def prepared():
    policy = OmegaConf.to_container(
        OmegaConf.load(ROOT / "configs/analysis/public_flight_comparison.yaml")
    )
    _, arrays = prepare_observations(ROOT / "data/reference/crs11", policy)
    return policy, arrays


def synthetic_result(prepared, name, *, until=None):
    """Exactly known +20 m/+2 m/s biases at raw epochs; r_D deliberately is not altitude."""
    policy, arrays = prepared
    start = policy["phases"][name]["start_s"]
    selected = arrays["phase_index"] == PHASES.index(name)
    if until is not None:
        selected &= arrays["reference_time_s"] <= start + until
    t = arrays["reference_time_s"][selected] - start
    height, speed = arrays["altitude_m"][selected] + 20, arrays["speed_mps"][selected] + 2
    return SimpleNamespace(
        t=t,
        v_ned=np.column_stack([speed, np.zeros((len(t), 2))]),
        r_ned=np.full((len(t), 3), 1e8),
        aux={"per_tick": [{"altitude": h} for h in height]},
    )


def test_masked_statistics_preserve_adjacency_and_ignore_isolated_sample():
    stats, connected = masked_statistics(
        [0, 0.1, 0.2, 0.3, 2, 2.1, 3],
        [2, 2, 999, 999, 4, 4, 999],
        [True, True, False, True, True, True, False],
        max_gap_s=1.1,
    )
    assert stats["covered_duration_s"] == pytest.approx(0.2)
    assert stats["bias"] == pytest.approx(3)
    assert stats["rmse"] == pytest.approx(np.sqrt(10))
    assert stats["max_abs"] == 4
    np.testing.assert_array_equal(connected, [False, True, False, False, False, True, False])


def test_simulation_interpolation_rejects_extrapolation_missing_values_and_gaps():
    values = interpolate_simulation(
        [0, 1, 2, 4], [10, 20, np.nan, 40], [-1, 0, 0.5, 1.5, 3, 4, 5], max_gap_s=1.1
    )
    np.testing.assert_allclose(values, [np.nan, 10, 15, np.nan, np.nan, 40, np.nan], equal_nan=True)


def test_simulation_hole_between_exact_observation_epochs_cannot_be_bridged():
    predicted, segments = interpolate_simulation(
        [0, 0.1, 0.2], [10, np.nan, 20], [0, 0.2], max_gap_s=1.1, return_segments=True
    )
    stats, _ = masked_statistics(
        [0, 0.2],
        predicted,
        [True, True],
        max_gap_s=1.1,
        interval_eligible=[False, segments[0] == segments[1]],
    )
    assert stats["status"] == "insufficient_coverage"
    assert stats["rmse"] is None


@pytest.mark.parametrize("name", PHASES)
def test_policy_dispatch_radial_altitude_scalar_speed_masks_and_exact_bias(prepared, name):
    cfg = compose_cfg([f"scenario=flight_comparison_{name}"])
    report, arrays = compare_flight_reference(
        synthetic_result(prepared, name), cfg.scenario.reference
    )
    assert report["status"] == "comparison_only"
    metrics = report["metrics"]["requested_overlap"]
    assert metrics["altitude_m"]["bias"] == pytest.approx(20)
    assert metrics["speed_m_s"]["rmse"] == pytest.approx(2)
    assert report["normalized_reconstruction_mse"] == pytest.approx(0.04)
    assert not np.any(arrays["initialization_used"] & arrays["speed_m_s_score_eligible"])
    assert not np.any((arrays["reference_altitude_m"] == 0) & arrays["altitude_m_score_eligible"])
    if name == "landing":
        assert metrics["altitude_m"]["covered_duration_s"] == pytest.approx(4.672)
        assert metrics["speed_m_s"]["covered_duration_s"] == pytest.approx(12.112)
        assert arrays["speed_m_s_score_eligible"][-1]
    json.dumps(report, allow_nan=False)


def test_early_termination_empty_overlap_single_sample_and_missing_channel(prepared):
    cfg = compose_cfg(["scenario=flight_comparison_landing"])
    result = synthetic_result(prepared, "landing", until=2)
    report, arrays = compare_flight_reference(result, cfg.scenario.reference)
    assert report["comparison_ends_early"]
    assert np.all(np.isnan(arrays["simulation_speed_m_s"][arrays["simulation_time_s"] > 2]))
    assert report["metrics"]["requested_overlap"]["speed_m_s"]["covered_duration_s"] < 2
    result.aux = {}
    report, _ = compare_flight_reference(result, cfg.scenario.reference)
    assert report["channel_status"]["altitude_m"] == "not_comparable"
    assert report["normalized_reconstruction_mse"] is None
    result.t = result.t[1:2]
    result.v_ned = result.v_ned[1:2]
    report, _ = compare_flight_reference(result, cfg.scenario.reference)
    assert report["status"] == "insufficient_coverage"
    result.t = result.t + 100
    report, _ = compare_flight_reference(result, cfg.scenario.reference)
    assert report["status"] == "not_comparable"
    assert all(m["rmse"] is None for m in report["metrics"]["requested_overlap"].values())
    result.t = np.array([])
    result.v_ned = np.empty((0, 3))
    assert compare_flight_reference(result, cfg.scenario.reference)[0]["status"] == "not_comparable"


def test_offset_change_and_missing_snapshot_fail_closed(prepared, tmp_path):
    cfg = compose_cfg(["scenario=flight_comparison_landing"])
    result = synthetic_result(prepared, "landing")
    cfg.scenario.reference.time_offset_s *= -1
    with pytest.raises(ValueError, match="offsets/windows"):
        compare_flight_reference(result, cfg.scenario.reference)
    with pytest.raises(FileNotFoundError):
        compare_flight_reference(result, cfg.scenario.reference, data_root=tmp_path)


def test_initialization_rejects_excluded_altitude_and_preserves_sensitivity(prepared):
    policy, arrays = prepared
    states, intervals = reconstruct_initial_states(policy, arrays)
    assert states["entry_burn"]["derivative_estimates"][0]["status"] == "component_inconsistent"
    landing = states["landing"]
    assert landing["initial_state"]["v_ned"][2] == pytest.approx(58.07566277063896)
    assert np.linalg.norm(landing["initial_state"]["v_ned"]) == pytest.approx(76.111)
    # Neighboring phases' terminal samples used for subsequent ICs are also excluded.
    reserved = initialization_mask(arrays["reference_time_s"], intervals)
    assert np.all(
        reserved[(arrays["reference_time_s"] >= 446.485) & (arrays["reference_time_s"] <= 452.485)]
    )
    arrays["altitude_eligible"][np.searchsorted(arrays["reference_time_s"], 450)] = False
    with pytest.raises(ValueError, match="unusable nominal"):
        reconstruct_initial_states(policy, arrays)


@pytest.mark.parametrize("name", PHASES)
def test_generated_scenarios_match_recipe_and_initialize_native_plant(prepared, name):
    policy, arrays = prepared
    states, _ = reconstruct_initial_states(policy, arrays)
    spec = policy["reconstruction"]["phases"][name]
    base = compose_cfg([f"scenario={spec['base_scenario']}"])
    fragment = scenario_fragment(name, policy, states[name], base)
    saved = OmegaConf.to_container(
        OmegaConf.load(ROOT / f"configs/scenario/flight_comparison_{name}.yaml")
    )
    assert fragment == saved
    cfg = compose_cfg([f"scenario=flight_comparison_{name}"])
    for param in cfg.scenario.reconstruction.future_parameters:
        for path in param.paths:
            assert OmegaConf.select(cfg, path) == pytest.approx(param.initial)
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
    _, aux = bundle.native.create_session(state).observe()
    assert aux["altitude"] == pytest.approx(states[name]["altitude_observation"]["value"], abs=1e-8)
    assert aux["main_propellant"] > 0
    if name != "landing":
        PrescribedInputs(cfg.scenario.inputs, bundle.params, cfg.scenario.horizon.t_max)
    # A few ticks check the runtime entry point; this is not a phase-length before run.
    cfg.scenario.horizon.t_max = 0.04
    cfg.scenario.horizon.log_dt = 0.02
    if name != "landing":
        cfg.scenario.inputs = [cfg.scenario.inputs[0]]
    _, result = run_configured(cfg, record_telemetry=False)
    assert result.t[-1] == pytest.approx(0.04)
    assert np.all(np.isfinite(result.v_ned))


def test_policy_writer_emits_json_masks_and_plot(prepared, tmp_path):
    cfg = compose_cfg(["scenario=flight_comparison_landing"])
    report = write_flight_comparison(
        synthetic_result(prepared, "landing"), cfg.scenario.reference, tmp_path
    )
    assert json.loads((tmp_path / "flight_comparison.json").read_text()) == report
    assert (tmp_path / "plots/flight_comparison.png").stat().st_size > 0
    arrays = np.load(tmp_path / "flight_comparison.npz")
    assert "altitude_m_interval_eligible" in arrays
