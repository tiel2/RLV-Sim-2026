"""Verify that common-mode output errors cannot alter the independent reference."""

import copy
import csv
import json

import numpy as np
import pytest
from conftest import compose_cfg
from scipy.spatial.transform import Rotation

from sim.run import run_configured
from validation.analytical import validate_episode, write_validation
from validation.references import attitude_error, build_reference, mass_properties


@pytest.fixture(scope="module")
def cases():
    runs = {}
    for name in ("freefall_uniform", "vacuum_burn", "drag_descent", "free_rotation"):
        cfg = compose_cfg([f"scenario={name}", "scenario.horizon.t_max=1.0"])
        bundle, result = run_configured(cfg)
        runs[name] = cfg, bundle, result
    return runs


def test_initial_state_error_is_not_absorbed_into_reference(cases):
    cfg, bundle, original = cases["freefall_uniform"]
    result = copy.deepcopy(original)
    result.r_ned[:, 0] += 0.1  # internally consistent trajectory, wrong configured IC
    report, ref = validate_episode(result, bundle, cfg)
    assert report["status"] == "failed"
    assert report["checks"]["position"]["error"] == pytest.approx(0.1)
    np.testing.assert_array_equal(ref["r_ned"][0], cfg.scenario.ic.r_ned)


def test_correlated_mass_and_velocity_errors_cannot_pass_rocket_check(cases):
    cfg, bundle, original = cases["vacuum_burn"]
    result = copy.deepcopy(original)
    _, baseline = validate_episode(original, bundle, cfg)
    result.m = cfg.scenario.ic.m - 1.02 * (cfg.scenario.ic.m - result.m)
    axis = Rotation.from_quat(np.roll(original.q_bn[0], -1)).apply([1, 0, 0])
    # This corrupted velocity still satisfies the old check using simulated mass.
    result.v_ned = (311 * 9.80665 * np.log(cfg.scenario.ic.m / result.m))[:, None] * axis
    report, ref = validate_episode(result, bundle, cfg)
    assert not report["checks"]["velocity"]["passed"]
    assert not report["checks"]["mass_budget"]["passed"]
    np.testing.assert_array_equal(ref["mass"], baseline["mass"])
    np.testing.assert_array_equal(ref["v_ned"], baseline["v_ned"])


def test_density_and_coefficient_diagnostics_are_checked_not_used_as_reference(cases):
    cfg, bundle, original = cases["drag_descent"]
    _, baseline = validate_episode(original, bundle, cfg)
    result = copy.deepcopy(original)
    for sample in result.aux["per_tick"]:
        sample["rho"] *= 1.02
        sample["aero_cd"] *= 1.02
    report, ref = validate_episode(result, bundle, cfg)
    assert not report["checks"]["rho"]["passed"]
    assert not report["checks"]["aero_cd"]["passed"]
    np.testing.assert_array_equal(ref["r_ned"], baseline["r_ned"])


def test_rotation_and_inertia_errors_are_detected(cases):
    cfg, bundle, original = cases["free_rotation"]
    result = copy.deepcopy(original)
    attitude = Rotation.from_quat(np.roll(result.q_bn, -1, axis=1))
    result.q_bn = np.roll((Rotation.from_rotvec([0.01, 0, 0]) * attitude).as_quat(), 1, axis=1)
    for sample in result.aux["per_tick"]:
        sample["inertia_diag"] = np.array(sample["inertia_diag"]) * 1.01
    report, _ = validate_episode(result, bundle, cfg)
    assert not report["checks"]["attitude"]["passed"]
    assert not report["checks"]["inertia_relative"]["passed"]


def test_reference_uses_no_shared_frame_or_mass_property_helpers(cases, monkeypatch):
    import core.frames
    import plant.mass_properties

    def forbidden(*args, **kwargs):
        raise AssertionError("reference called a production calculation")

    monkeypatch.setattr(core.frames, "dcm_from_quat", forbidden)
    monkeypatch.setattr(core.frames, "quat_from_euler", forbidden)
    monkeypatch.setattr(plant.mass_properties, "dual_tank_properties", forbidden)
    for cfg, _, result in cases.values():
        ref, _ = build_reference(cfg, result.t)
        assert np.all(np.isfinite(ref["r_ned"]))


def test_numerical_reference_has_checked_refinement_and_sign_independent_attitude(cases):
    cfg, bundle, result = cases["free_rotation"]
    report, ref = validate_episode(result, bundle, cfg)
    assert report["checks"]["reference_stability_q_bn"]["passed"]
    assert report["checks"]["reference_stability_w_body"]["passed"]
    np.testing.assert_allclose(attitude_error(ref["q_bn"], -ref["q_bn"]), 0, atol=1e-15)
    # Empty tank limit is independently known without evaluating the Plant.
    cg, inertia = mass_properties(cfg.plant.vehicle, cfg.plant.vehicle.mass.m_dry, 0)
    assert cg == pytest.approx(cfg.plant.vehicle.mass.x_cg_dry)
    np.testing.assert_allclose(np.diag(inertia), cfg.plant.vehicle.mass.I_dry)


@pytest.mark.parametrize(
    "overrides,reason",
    [
        (["scenario=vacuum_burn", "scenario.ic.throttle0=0"], "initial throttle"),
        (["scenario=drag_descent", "plant.aero.coeffs.cd_alpha=1"], "constant drag"),
        (["scenario=free_rotation", "scenario.dispersion.enable=true"], "deterministic"),
        (["scenario=freefall_inverse_square", "scenario.ic.v_ned=[1,0,0]"], "vertical"),
    ],
)
def test_reference_rejects_unmodelled_conditions(overrides, reason):
    cfg = compose_cfg(overrides)
    with pytest.raises(ValueError, match=reason):
        build_reference(cfg, [0, 0.1])


def test_comparison_outputs_include_reference_actual_and_errors(cases, tmp_path):
    cfg, bundle, result = cases["freefall_uniform"]
    report, reference = validate_episode(result, bundle, cfg)
    write_validation(report, reference, tmp_path, plot=False)
    stored = json.loads((tmp_path / "validation.json").read_text())
    assert stored["status"] == "passed"
    assert "rms_error" in stored["checks"]["position"]
    with (tmp_path / "comparison.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(result.t)
    assert {"r_ned_reference_0", "r_ned_simulation_0", "r_ned_error"} <= rows[0].keys()
    with np.load(tmp_path / "reference.npz") as data:
        np.testing.assert_allclose(data["r_ned"], data["actual_r_ned"], atol=1e-5)
