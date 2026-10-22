"""Hand-computable anchors, enforced command/rate limits, and observed overshoot."""

import json

import numpy as np
import pytest
from conftest import compose_cfg

from validation.propulsion import (
    check_propulsion,
    clone,
    engine_reference,
    servo_reference,
)


@pytest.fixture(scope="module")
def cfg():
    return compose_cfg(["scenario=vacuum_burn", "plant/thrust=vacuum"])


@pytest.fixture(scope="module")
def result(cfg):
    return check_propulsion(cfg)


def test_exports_preserve_verdicts_and_full_histories_without_templates(result, tmp_path):
    from scripts.check_propulsion import write_results

    report, rows, series, configs = result
    write_results(report, rows, series, configs, tmp_path)
    assert json.loads((tmp_path / "report.json").read_text()) == json.loads(json.dumps(report))
    assert len(json.loads((tmp_path / "static_cases.json").read_text())) == len(rows)
    for name, expected in series.items():
        with np.load(tmp_path / "dynamic" / f"{name}.npz") as saved:
            for key, values in expected.items():
                np.testing.assert_array_equal(saved[key], values)
    assert len(list((tmp_path / "settings").glob("*.yaml"))) == len(configs)
    sources = json.loads((tmp_path / "source_sha256.json").read_text())
    assert all((tmp_path / "source" / path).is_file() for path in sources)
    assert not list(tmp_path.rglob("*.md"))


@pytest.mark.parametrize(
    "angles,direction",
    [
        ([[0, 0]], [1, 0, 0]),
        ([[np.pi / 2, 0]], [0, 0, -1]),
        ([[0, np.pi / 2]], [0, 1, 0]),
        ([[0, -np.pi / 2]], [0, -1, 0]),
    ],
)
def test_direction_convention_at_known_right_angles(cfg, angles, direction):
    ref = engine_reference(cfg, [0.7] + [0] * 8, angles, 40000.0, 10000.0)
    np.testing.assert_allclose(ref["force"], 0.7 * 981000 * np.array(direction), atol=3e-10, rtol=0)
    assert ref["flow"] == pytest.approx(0.7 * 981000 / (311 * 9.80665), rel=1e-14)


@pytest.mark.parametrize(
    "indices,moment",
    [
        ([1], [0, 0, -1]),
        ([3], [0, 1, 0]),
        ([1, 5], [0, 0, 0]),
    ],
)
def test_ring_moments_follow_hand_geometry(cfg, indices, moment):
    u = np.zeros(9)
    u[indices] = 0.7
    ref = engine_reference(cfg, u, [[0, 0]], 40000.0, 10000.0)
    np.testing.assert_allclose(
        ref["moment"], 0.7 * 981000 * 1.7 * np.array(moment), atol=1e-8, rtol=0
    )


def test_backpressure_changes_force_without_changing_vacuum_based_flow(cfg):
    c = clone(cfg)
    c.plant.thrust.model = "backpressure"
    c.environment.atmosphere.model = "exp"
    c.environment.atmosphere.p_sea = 101325.0
    c.environment.atmosphere.scale_height = 8500.0
    u = [0.7] + [0] * 8
    vac = engine_reference(cfg, u, [[0, 0]], 40000.0, 0.0)
    sea = engine_reference(c, u, [[0, 0]], 40000.0, 0.0)
    assert vac["force"][0] - sea["force"][0] == pytest.approx(101325 * 0.9)
    assert sea["flow"] == vac["flow"]


def test_native_static_cases_and_mass_dependent_moment(result):
    report, rows, _, _ = result
    assert report["static_cases"] == 183
    assert report["static_status"] == "passed"
    selected = [r for r in rows if r["variant"] == "vacuum" and r["name"] == "pitch_0.1"]
    assert len(selected) == 3
    moments = []
    for r in selected:
        moments.append(r["actual_moment_nm"][1])
        arm = -42 - r["cg_x_m"]
        expected = arm * 0.7 * 981000 * np.sin(0.1)
        assert r["actual_moment_nm"][1] == pytest.approx(expected, abs=1e-7, rel=0)
    assert np.ptp(moments) > 10000
    assert all(r["passed"] for r in rows if r["group"] == "ideal_command_clip")


def test_servo_small_signal_reference_has_correct_initial_state_and_linear_step(cfg):
    a = cfg.plant.vehicle.actuator
    t = np.array([0.0, 0.05, 0.5])
    actual = servo_reference(a, "simplified", [(0, [0.01, -0.01])], t)
    w, z = a.tvc_omega, a.tvc_zeta
    wd = w * np.sqrt(1 - z * z)
    expected = 0.01 * (1 - np.exp(-z * w * t) * (np.cos(wd * t) + z * w / wd * np.sin(wd * t)))
    np.testing.assert_allclose(actual[:, 0], expected, atol=1e-17)
    np.testing.assert_allclose(actual[:, 1], -expected, atol=1e-17)
    np.testing.assert_allclose(actual[0], 0, atol=1e-17)


def test_command_limit_and_actual_overshoot_have_distinct_roles(result):
    report, _, _, configs = result
    assert report["model_agreement"] == "passed", report
    assert report["status"] == "passed"
    assert report["constraint_status"] == "passed"
    assert report["acceptance_policy"]["mechanical_stop_status"] == "not_assessed"
    cases = [r for r in report["dynamic_cases"] if r["name"] == "realistic_limit_reversal"]
    assert len(cases) == 3
    for case in cases:
        assert case["model_agreement"] == "passed"
        assert case["constraint_status"] == "passed"
        assert case["enforced_constraints"]["command_clip_acceleration_rad_s2"]["passed"]
        assert case["enforced_constraints"]["rate_exceedance_rad_s"]["passed"]
        assert case["angle_observation"]["overshoot_rad"] > 0
        assert case["mechanical_stop_status"] == "not_assessed"
        # Numerical model agreement stays as strict as before the policy reassessment.
        assert case["checks"]["angle_rad"]["tolerance"] == 1e-7
        assert case["checks"]["rate_rad_s"]["tolerance"] == 2e-6
    # Saved settings must describe the unloaded component experiment, not the base burn.
    snapshot = configs["realistic_limit_reversal_0.001"]
    assert snapshot.scenario.ic.throttle0 == 0
    assert all(row.throttle == 0 for row in snapshot.scenario.inputs)
    assert snapshot.scenario.horizon.t_max == 2.0
    assert snapshot.scenario.horizon.log_dt == 0.001


def test_reference_does_not_use_production_geometry_or_frames(cfg, monkeypatch):
    import core.frames
    import plant.dynamics

    def forbidden(*args, **kwargs):
        raise AssertionError("reference called production calculations")

    monkeypatch.setattr(core.frames, "dcm_from_quat", forbidden)
    monkeypatch.setattr(plant.dynamics, "build_plant", forbidden)
    ref = engine_reference(cfg, [0.7] + [0] * 8, [[0.1, -0.1]], 40000.0, 10000.0)
    assert np.linalg.norm(ref["force"]) == pytest.approx(0.7 * 981000, abs=1e-9)


def test_reference_rejects_uncovered_throttle_and_faults(cfg):
    with pytest.raises(ValueError, match="above-minimum"):
        engine_reference(cfg, [0.2] + [0] * 8, [[0, 0]], 40000.0, 10000.0)
    c = clone(cfg)
    c.fault.engine_out_idx = 1
    with pytest.raises(ValueError, match="healthy"):
        engine_reference(c, [0.7] + [0] * 8, [[0, 0]], 40000.0, 10000.0)


def test_incorrect_full_rhs_force_cannot_hide_behind_correct_diagnostics(cfg, monkeypatch):
    import validation.propulsion as module

    original = module.build_plant

    class WrongRhsForce:
        def __init__(self, native):
            self.native = native

        def rhs(self, *args):
            derivative, aux = self.native.rhs(*args)
            aux["T_body"] = np.asarray(aux["T_body"]) * [1, -1, 1]
            return derivative, aux

        def engine_wrench_diagnostics(self, *args):
            return self.native.engine_wrench_diagnostics(*args)

    def broken(cfg):
        bundle = original(cfg)
        return bundle._replace(native=WrongRhsForce(bundle.native))

    monkeypatch.setattr(module, "build_plant", broken)
    rows, _ = module.check_static(cfg)
    assert any(not row["checks"]["force_n"]["passed"] for row in rows)


def test_incorrect_native_command_clipping_still_fails(cfg, monkeypatch):
    import validation.propulsion as module

    original = module.build_plant
    limit = cfg.plant.vehicle.actuator.tvc_deflection_max_rad

    class WrongCommandClip:
        def __init__(self, native):
            self.native = native

        def create_session(self, state):
            return self.native.create_session(state)

        def rhs(self, state, cmd, wind):
            derivative, aux = self.native.rhs(state, cmd, wind)
            derivative = list(derivative)
            if np.any(np.abs(cmd.gimbal) > limit):
                derivative[7] = np.asarray(derivative[7]) + 1.0
            return derivative, aux

    def broken(cfg):
        bundle = original(cfg)
        return bundle._replace(native=WrongCommandClip(bundle.native))

    monkeypatch.setattr(module, "build_plant", broken)
    report, _, _ = module.check_servo(
        cfg, "clip_failure", "realistic", [(0.0, [0.3, -0.3])], 0.1, 0.001
    )
    assert report["model_agreement"] == "passed"
    assert report["constraint_status"] == "failed"
    assert not report["enforced_constraints"]["command_clip_acceleration_rad_s2"]["passed"]
