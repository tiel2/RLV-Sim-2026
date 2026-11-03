"""Standalone runtime specifications, independent fin dynamics and local-time comparisons."""

import json
from pathlib import Path

import numpy as np
import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf

from sim.replay import build_replay_doc
from sim.run import run_configured
from validation.ascent import compare, reference
from validation.flight_reference import compare_flight_reference
from validation.segment_spec import prepare_segments

ROOT = Path(__file__).resolve().parents[2]
CASES = ("ascent", "entry_burn", "aero_descent", "powered_descent")


def short_config(name):
    cfg = compose_cfg([f"scenario=project2_{name}"])
    cfg.scenario.horizon.t_max = 1.0
    cfg.scenario.horizon.log_dt = 0.02
    cfg.scenario.inputs = [cfg.scenario.inputs[0]]
    return cfg


def test_runtime_initial_states_and_overrides_match_authored_specification():
    recipe = OmegaConf.load(ROOT / "configs/analysis/project2_segments.yaml")
    spec = prepare_segments(
        ROOT / "data/reference/crs11", recipe, lambda name: compose_cfg([f"scenario={name}"])
    )
    for name in CASES:
        cfg = compose_cfg([f"scenario=project2_{name}"])
        expected = spec["segments"][name]
        assert cfg.scenario.input_mode == "prescribed"
        assert cfg.scenario.horizon.t_max == expected["simulation_duration_s"]
        assert OmegaConf.to_container(cfg.scenario.ic) == expected["proposed_initial_state"]
        assert cfg.scenario.reference.time_offset_s == -expected["reference_window_s"][0]
        assert not cfg.scenario.reference.enabled
        assert cfg.scenario.reference.exclude_initial_sample
        assert cfg.scenario.reference.segment == name
        if name != "ascent":
            assert cfg.plant.vehicle.engines.thrust_vac == 914000
            assert list(cfg.plant.vehicle.tanks.bottom) == [-26, -40]
            for actual, row in zip(cfg.scenario.inputs, expected["proposed_inputs"], strict=True):
                for key, value in row.items():
                    if key != "label":
                        assert OmegaConf.to_container(actual)[key] == value
    # The generic component experiment retains its separate initial conditions.
    assert compose_cfg(["scenario=entry_burn"]).scenario.ic.r_ned[2] == -40000


@pytest.mark.parametrize("name", ["entry_burn", "aero_descent", "powered_descent"])
def test_realistic_neutral_fin_dynamics_match_independent_equations(name):
    cfg = short_config(name)
    cfg.scenario.ic.pitch_deg += 3.0  # nonzero incidence tests fin forces/moments
    if name != "aero_descent":
        cfg.scenario.inputs.append({"t": 0.5, "throttle": 0.0})
    _, result = run_configured(cfg, record_telemetry=False)
    assert np.linalg.norm(result.aux["per_tick"][0]["F_fin_body"]) > 0
    expected = reference(
        OmegaConf.to_container(cfg, resolve=True), result.t, rtol=1e-12, allow_neutral_fins=True
    )
    errors = compare(result, expected)
    assert errors["r_ned"] < 1e-4
    assert errors["v_ned"] < 1e-5
    assert errors["q_bn"] < 1e-7
    assert errors["w_body"] < 1e-7
    assert errors["m"] < 1e-4


@pytest.mark.parametrize("name", ["entry_burn", "aero_descent", "powered_descent"])
def test_reference_comparison_uses_local_epoch_and_reports_partial_coverage(name):
    cfg = short_config(name)
    cfg.scenario.reference.enabled = True  # comparisons are an explicit optional analysis
    bundle, result = run_configured(cfg)
    report, arrays = compare_flight_reference(result, cfg.scenario.reference)
    assert report["segment"] == name
    assert report["comparison_ends_early"]
    assert set(report["metrics"]) == {"requested_overlap", name}
    assert "meco_command" not in report["checkpoints"]
    assert report["initial_sample_excluded"]
    assert report["checkpoints"]["initial"]["reference_time_s"] == cfg.scenario.reference.start_s
    assert np.all(arrays["simulation_time_s"] > 0)
    assert np.all(arrays["simulation_time_s"] <= 1)
    np.testing.assert_allclose(
        arrays["reference_time_s"],
        arrays["simulation_time_s"] - cfg.scenario.reference.time_offset_s,
    )
    doc = build_replay_doc(result, bundle, cfg, max_frames=25)
    assert doc["meta"]["flight_reference"]["segment"] == name
    assert doc["meta"]["flight_reference"]["time_offset_s"] == cfg.scenario.reference.time_offset_s
    json.dumps(doc, allow_nan=False)


def test_default_ascent_reference_rejects_exposed_fins():
    cfg = short_config("entry_burn")
    with pytest.raises(ValueError, match="fin exposure"):
        reference(OmegaConf.to_container(cfg, resolve=True), np.array([0.0, 1.0]))


def test_powered_descent_demonstrates_deceleration_shutdown_inside_aero_domain():
    cfg = compose_cfg(["scenario=project2_powered_descent"])
    _, result = run_configured(cfg, record_telemetry=False)
    assert result.t[-1] == pytest.approx(22)
    assert result.v_ned[-1, 2] > 0  # still descending; no requirement to achieve touchdown
    assert np.linalg.norm(result.v_ned[-1]) < np.linalg.norm(result.v_ned[0])
    assert max(float(a["aero_alpha"]) for a in result.aux["per_tick"]) < np.deg2rad(90)
    assert result.aux["per_tick"][-1]["thrust_mag"] == 0
    assert result.landing_class is None
