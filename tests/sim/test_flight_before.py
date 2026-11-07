"""Independent numerical anchors for logged screens and initial braking scales."""

import numpy as np
import pytest
from omegaconf import OmegaConf

from validation.flight_before import audit_history, braking_screen


def test_braking_screen_separates_vertical_lateral_and_impulse_fuel():
    result = braking_screen(300, 60, 40, 1000, 20000, 10, 30, 200, 300)
    assert result["initial_thrust_weight_ratio"] == 2
    assert result["vertical_stopping_distance_m"] == 180
    assert result["vertical_stopping_time_s"] == 6
    assert result["lateral_stopping_time_at_cone_s"] == pytest.approx(4)
    assert result["lateral_stopping_distance_at_cone_m"] == pytest.approx(80)
    assert 0 < result["ideal_propellant_for_speed_change_kg"] < 200


def test_insufficient_vertical_thrust_and_zero_cone_do_not_claim_stopping():
    result = braking_screen(100, 10, 10, 1000, 5000, 10, 0, 200, 300)
    assert result["vertical_stopping_distance_m"] is None
    assert result["lateral_stopping_time_at_cone_s"] is None


def fixture_history():
    cfg = OmegaConf.create(
        dict(
            plant=dict(
                actuator=dict(model="realistic"),
                vehicle=dict(
                    mass=dict(m_dry=4),
                    actuator=dict(tvc_deflection_max_rad=0.14, fin_deflection_max_rad=0.35),
                ),
                aero=dict(grid=dict(alpha_max_deg=90, mach_min=0, mach_max=8)),
            )
        )
    )
    p = dict(
        t=np.array([0, 1]),
        mass=np.array([10.0, 9.0]),
        main_propellant=np.array([5.0, 4.0]),
        rcs_fuel=np.ones(2),
        aero_alpha=np.zeros(2),
        mach=np.ones(2),
        q_bn=np.array([[1, 0, 0, 0], [1, 0, 0, 0]]),
        inertia_diag=np.ones((2, 3)),
        aero_q_dyn=np.zeros(2),
    )
    p.update({k: np.zeros((2, 3)) for k in ["r_ned", "v_ned", "w_body"]})
    actuators = dict(
        effective_throttle=np.array([[0.0], [0.1]]),
        effective_gimbal=np.zeros((2, 1, 2)),
        effective_fin=np.zeros((2, 4)),
    )
    return cfg, p, actuators


def test_legal_transient_below_throttle_floor_is_not_a_violation():
    cfg, p, actuators = fixture_history()
    result = audit_history(p, actuators, cfg)
    assert result["status"] == "passed_logged_screens"
    assert "independent_equations" in result["not_assessed"]


def test_mass_creation_domain_and_actuator_overruns_are_reported_separately():
    cfg, p, actuators = fixture_history()
    p["mass"][-1] = 11
    p["mach"][-1] = 9
    actuators["effective_gimbal"][-1, 0, 0] = 0.2
    result = audit_history(p, actuators, cfg)
    assert result["status"] == "failed_logged_screens"
    assert set(result["failed_checks"]) == {
        "mass_bookkeeping",
        "no_mass_creation",
        "body_lookup_domain",
        "tvc_deflection",
    }
    assert result["diagnostics"]["first_body_domain_violation_s"] == 1
