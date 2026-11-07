"""Integrated-observation reconstruction, spherical placement, and actual mass handoffs."""

import numpy as np
import pytest
from conftest import compose_cfg

from sim.run import run_configured
from validation.flight_reconstruction import direction_envelope, landing_state


def test_integrated_speed_recovers_known_direction_without_future_observations():
    t = np.linspace(0, 8, 81)
    h = 300 + 80 * (6 - t)
    speed = np.full_like(t, 100.0)
    result = direction_envelope(t, h, speed, epoch=6, span=6, height0=300, tolerance=1)
    assert result["nominal_fraction"] == pytest.approx(0.8)
    assert result["fraction_interval"] == pytest.approx([0.8 - 1 / 600, 0.8 + 1 / 600])
    h[t > 6] = 99999
    assert direction_envelope(t, h, speed, epoch=6, span=6, height0=300, tolerance=1) == result


def test_inconsistent_envelope_is_not_clipped_into_a_feasible_state():
    t = np.linspace(0, 6, 61)
    result = direction_envelope(
        t, 300 + 120 * (6 - t), np.full_like(t, 100), epoch=6, span=6, height0=300, tolerance=1
    )
    assert result["status"] == "infeasible_under_assumptions"
    assert result["nominal_fraction"] is None


@pytest.mark.parametrize("failure", ["gap", "zero_altitude", "missing_endpoint"])
def test_reconstruction_rejects_missing_or_excluded_support(failure):
    t = np.linspace(0, 6, 61)
    h = 300 + 80 * (6 - t)
    v = np.full_like(t, 100.0)
    if failure == "gap":
        keep = (t < 2) | (t > 4)
        t, h, v = t[keep], h[keep], v[keep]
    elif failure == "zero_altitude":
        h[20] = 0
    else:
        t, h, v = t[:-1], h[:-1], v[:-1]
    with pytest.raises(ValueError, match="gap, excluded altitude or missing endpoint"):
        direction_envelope(t, h, v, epoch=6, span=6, height0=300, tolerance=100)


def test_target_shift_preserves_radial_height_speed_and_forward_stopping_rule():
    radius = 6371000.0
    state = landing_state(0.99, 76.111, 300, radius)
    r = np.array(state["r_ned"])
    v = np.array(state["v_ned"])
    assert np.linalg.norm(r + [0, 0, -radius]) - radius == pytest.approx(300, abs=1e-8)
    assert np.linalg.norm(v) == pytest.approx(76.111)
    assert r[0] < 0 and v[0] > 0
    assert state["forward_target_distance_m"] == pytest.approx(v[0] * 300 / v[2])


def test_exported_recovery_configs_carry_actual_mass_and_land_without_relaxed_pointing():
    previous = None
    for name in ["entry_burn", "aero_descent", "powered_descent", "landing"]:
        cfg = compose_cfg([f"scenario=flight_reconstructed_{name}"])
        assert cfg.plant.vehicle.mass.m_dry == 25000
        if previous is not None:
            assert cfg.scenario.ic.m == pytest.approx(previous.m[-1], abs=1e-6)
            assert cfg.scenario.ic.rcs_fuel == pytest.approx(
                previous.aux["per_tick"][-1]["rcs_fuel"]
            )
        _, previous = run_configured(cfg, record_telemetry=False)
        assert np.all(previous.m >= 25200)
    assert cfg.gnc.guidance.attitude_tilt_max_deg == 15
    assert not cfg.gnc.guidance.convex_terminal_slack
    assert previous.success and previous.landing_class == "soft"
