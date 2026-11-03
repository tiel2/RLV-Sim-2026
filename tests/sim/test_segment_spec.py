"""Analytical reconstruction anchors and guards against invented flight states."""

import json
from pathlib import Path

import numpy as np
import pytest
from conftest import compose_cfg
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.open_loop import PrescribedInputs
from validation.segment_spec import (
    estimate_components,
    load_snapshot,
    observation,
    prepare_segments,
)

ROOT = Path(__file__).resolve().parents[2]


def test_trailing_derivative_recovers_quadratic_descent_without_future_altitude():
    t = np.linspace(0, 10, 301)
    h = 1000 - 20 * t - 2 * t**2
    speed = np.full_like(t, 100.0)
    result = estimate_components(t, h, speed, 5.0, 4.0)
    assert result["status"] == "usable_estimate"
    assert result["vertical_up_mps"] == pytest.approx(-40, abs=1e-10)
    assert result["horizontal_speed_mps"] == pytest.approx(np.sqrt(100**2 - 40**2))
    h[t > 5] += 1e6
    assert estimate_components(t, h, speed, 5.0, 4.0) == result


def test_irregular_sample_density_does_not_break_polynomial_anchor():
    t = np.unique(np.r_[np.linspace(0, 10, 80), np.linspace(3, 4, 800)])
    h = 2000 + 10 * t + 3 * t**2
    r = estimate_components(t, h, np.full_like(t, 200.0), 8.0, 6.0)
    assert r["vertical_up_mps"] == pytest.approx(58.0, abs=1e-10)


def test_inconsistent_components_are_rejected_instead_of_clamped():
    t = np.linspace(0, 6, 181)
    r = estimate_components(t, 1000 - 110 * t, np.full_like(t, 100), 6, 4)
    assert r["status"] == "component_inconsistent"
    assert r["vertical_up_mps"] == pytest.approx(-110)
    assert r["horizontal_speed_mps"] is None
    assert r["flight_path_angle_deg"] is None


def test_gap_and_support_guards_do_not_create_motion_through_missing_data():
    t = np.r_[np.arange(0, 3, 0.1), np.arange(5, 8, 0.1)]
    h, speed = 1000 - 10 * t, np.full_like(t, 20.0)
    with pytest.raises(ValueError, match="gap"):
        observation(t, h, 4)
    assert estimate_components(t, h, speed, 4, 4)["status"] == "reference_gap"
    assert estimate_components(t, h, speed, 6, 4)["status"] == "reference_gap"
    assert estimate_components(t, h, speed, 1, 4)["status"] == "insufficient_support"
    with pytest.raises(ValueError, match="extrapolate"):
        observation(t, h, -1)


def test_zero_altitude_display_does_not_become_a_trustworthy_touchdown_state():
    t = np.linspace(0, 6, 181)
    r = estimate_components(t, np.zeros_like(t), np.full_like(t, 4.0), 6, 4)
    assert r["status"] == "altitude_floor"
    assert r["horizontal_speed_mps"] is None


def test_snapshot_integrity_failure_is_not_ignored(tmp_path):
    (tmp_path / "stage1_raw.json").write_text("tampered")
    (tmp_path / "manifest.json").write_text(
        json.dumps({"files": [dict(file="stage1_raw.json", bytes=8, sha256="0" * 64)]})
    )
    with pytest.raises(ValueError, match="integrity"):
        load_snapshot(tmp_path)


def test_selected_reference_states_preserve_speed_and_retrograde_geometry():
    recipe = OmegaConf.load(ROOT / "configs/analysis/project2_segments.yaml")
    report = prepare_segments(
        ROOT / "data/reference/crs11", recipe, lambda name: compose_cfg([f"scenario={name}"])
    )
    assert report["snapshot_integrity"] == "passed"
    assert len(report["segments"]) == 4
    for name, segment in report["segments"].items():
        state = segment["proposed_initial_state"]
        speed = segment["boundaries"]["start"]["scalar_speed_mps"]["value"]
        assert np.linalg.norm(state["v_ned"]) == pytest.approx(speed, abs=1e-9)
        assert state["m"] == pytest.approx(
            segment["structural_mass_kg"]
            + segment["attached_mass_kg"]
            + segment["initial_main_propellant_kg"]
            + state["rcs_fuel"]
        )
        assert all(v is not False for v in segment["planning_checks"].values())
        if name != "ascent":
            attitude = Rotation.from_euler(
                "ZYX", [state["yaw_deg"], state["pitch_deg"], state["roll_deg"]], degrees=True
            )
            np.testing.assert_allclose(
                attitude.apply([1, 0, 0]), -np.asarray(state["v_ned"]) / speed, atol=1e-12
            )
            assert segment["horizontal_frame"] == "segment_start_ground_projection"
            assert all(
                row["t"] < segment["simulation_duration_s"] for row in segment["proposed_inputs"]
            )
    estimates = report["segments"]["entry_burn"]["boundaries"]["start"]["velocity_estimates"]
    assert estimates[0]["status"] == "component_inconsistent"
    assert estimates[1]["status"] == "usable_estimate"


def test_event_epoch_and_impossible_fuel_budgets_are_rejected():
    recipe = OmegaConf.load(ROOT / "configs/analysis/project2_segments.yaml")
    recipe.segments.entry_burn.start_s = 370
    with pytest.raises(ValueError, match="archived marker"):
        prepare_segments(
            ROOT / "data/reference/crs11", recipe, lambda name: compose_cfg([f"scenario={name}"])
        )
    recipe.segments.entry_burn.start_s = 372
    recipe.segments.powered_descent.mass_kg = 27000
    with pytest.raises(ValueError, match="mass/burn budget"):
        prepare_segments(
            ROOT / "data/reference/crs11", recipe, lambda name: compose_cfg([f"scenario={name}"])
        )


def test_proposed_states_and_input_schedules_can_initialize_the_actual_plant():
    recipe = OmegaConf.load(ROOT / "configs/analysis/project2_segments.yaml")
    report = prepare_segments(
        ROOT / "data/reference/crs11", recipe, lambda name: compose_cfg([f"scenario={name}"])
    )
    for name, segment in report["segments"].items():
        cfg = compose_cfg([f"scenario={name}"])
        if segment["vehicle_overrides"] is not None:
            cfg = OmegaConf.merge(cfg, segment["vehicle_overrides"])
        cfg.scenario.ic = segment["proposed_initial_state"]
        cfg.scenario.horizon.t_max = segment["simulation_duration_s"]
        if segment["proposed_inputs"] is not None:
            cfg.scenario.inputs = segment["proposed_inputs"]
        bundle = build_plant(cfg)
        initial = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
        _, aux = bundle.native.create_session(initial).observe()
        assert aux["main_propellant"] == pytest.approx(segment["initial_main_propellant_kg"])
        expected = segment["initial_mass_properties"]
        assert aux["x_cg"] == pytest.approx(expected["x_cg_m"], abs=1e-10)
        np.testing.assert_allclose(aux["inertia_diag"], expected["inertia_diag_kg_m2"], rtol=1e-12)
        PrescribedInputs(cfg.scenario.inputs, bundle.params, float(cfg.scenario.horizon.t_max))
