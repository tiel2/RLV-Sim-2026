"""Hand anchors and deliberate corruption checks for the independent verifier."""

import json
import sys

import numpy as np
import pytest
from conftest import compose_cfg
from scipy.spatial.transform import Rotation

from validation import vehicle_physics as checks
from validation import vehicle_references as ref
from validation.propulsion import clone
from validation.references import attitude_error, mass_properties


@pytest.fixture(scope="module")
def cfg():
    return compose_cfg(["scenario=vacuum_burn", "plant/thrust=vacuum"])


@pytest.mark.parametrize(
    "function,count",
    [
        (checks.check_fins, 325),
        (checks.check_rcs, 1162),
        (checks.check_actuators, 7),
        (checks.check_aero_environment, 1064),
    ],
)
def test_component_experiments_pass(cfg, function, count):
    report, _, _ = function(cfg)
    assert report["cases"] == count
    assert report["status"] == "passed", [r for r in report["results"] if not r["passed"]]


def test_coupled_trajectories_reference_refinement_and_timesteps_pass(cfg):
    report, configs, _ = checks.check_coupled(cfg)
    assert report["cases"] == 24
    assert report["status"] == "passed", [r for r in report["results"] if not r["passed"]]
    # The saved scenario and the live-session test must both represent stowed fins.
    assert configs["coupled_burn_stowed_0.001"].scenario.inputs[0].fin_deployment == 0.0


def test_single_fin_local_rotation_drag_has_hand_computable_moment(cfg):
    c = clone(cfg)
    c.environment.atmosphere.model = "exp"
    c.environment.atmosphere.rho0 = 1.225
    c.plant.aero.fins.n_fins = 1
    c.plant.aero.fins.fin_cla = 0.0
    c.plant.aero.fins.fin_cd_alpha2 = 0.0
    c.plant.aero.fins.fin_transonic_mach = 5.0
    actual = ref.fins(c, [-100.0, 0, 0], [0.2, 0, 0], [0.3], -30.0, 0.0)
    # arm=[27,2,0]; omega x arm=[0,0,0.4]. Drag opposes this LOCAL flow.
    velocity = np.array([-100.0, 0.0, 0.4])
    expected = -0.5 * 1.225 * np.linalg.norm(velocity) * 2 * 0.1 * velocity
    np.testing.assert_allclose(actual["force"], expected, rtol=0, atol=1e-10)
    np.testing.assert_allclose(actual["moment"], np.cross([27, 2, 0], expected), rtol=0, atol=1e-9)


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("sense", [-1, 1])
def test_paired_rcs_matches_known_pure_couple(cfg, axis, sense):
    c = clone(cfg)
    c.plant.rcs.mode = "averaged"
    actual = ref.rcs(c, sense * np.eye(3)[axis], 0.0, -27.0)
    np.testing.assert_allclose(actual["force"], 0.0, rtol=0, atol=1e-12)
    np.testing.assert_allclose(actual["moment"], sense * 7400 * np.eye(3)[axis], rtol=0, atol=1e-10)
    assert actual["flow"] == pytest.approx(4000 / (60 * 9.80665), abs=1e-14)


@pytest.mark.parametrize("dt,expected_ticks", [(0.002, 3), (0.001, 7), (0.0005, 13)])
def test_pwm_rounding_is_discrete_not_continuous_duty(cfg, dt, expected_ticks):
    c = clone(cfg)
    c.plant_dt = dt
    ticks = round(0.02 / dt)
    on = [ref.rcs(c, [0.33, 0, 0], k * dt, -27.0)["on"][0] for k in range(ticks)]
    assert sum(on) == expected_ticks
    assert set(on) == {0.0, 1.0}


def test_linear_actuator_matches_exponential_throttle_and_step_superposition(cfg):
    times = np.linspace(0, 0.5, 101)
    response = ref.actuators(cfg, "simplified", [(0.0, 0.7, [0.01] * 4)], times)
    double = ref.actuators(cfg, "simplified", [(0.0, 0.7, [0.02] * 4)], times)
    np.testing.assert_allclose(
        response[:, 0], 0.7 * (1 - np.exp(-times / 0.25)), rtol=0, atol=1e-15
    )
    np.testing.assert_allclose(double[:, 1:], 2 * response[:, 1:], rtol=0, atol=1e-15)
    np.testing.assert_allclose(response[0], 0, rtol=0, atol=1e-15)


@pytest.mark.parametrize("mass,gas", [(40000.0, 200.0), (30000.0, 10.0)])
def test_analytic_inertia_rate_matches_mass_quadrature_derivative(cfg, mass, gas):
    v, main_flow, gas_flow = cfg.plant.vehicle, 200.0, 0.3
    # Differentiate along the actual depletion direction, using a much larger
    # mass perturbation than native's per-coordinate 1e-3 kg finite differences.
    eps = 0.001
    plus = mass_properties(v, mass + (main_flow + gas_flow) * eps, gas + gas_flow * eps)[1]
    minus = mass_properties(v, mass - (main_flow + gas_flow) * eps, gas - gas_flow * eps)[1]
    expected = np.diag(minus - plus) / (2 * eps)
    np.testing.assert_allclose(
        ref.inertia_rate(v, mass, gas, main_flow, gas_flow), expected, rtol=2e-9, atol=1e-5
    )


def test_standard_atmosphere_at_sea_level_and_first_layer_boundary(cfg):
    c = clone(cfg)
    c.environment.atmosphere.model = "ussa_poly"
    c.plant.thrust.model = "backpressure"
    sea = ref.environment(c, 0)
    assert sea["pressure"] == 101325.0
    assert sea["rho"] == pytest.approx(101325 / (287.05287 * 288.15), abs=1e-14)
    assert sea["a_sound"] == pytest.approx(np.sqrt(1.4 * 287.05287 * 288.15), abs=1e-12)
    h = 6356766 * 11000 / (6356766 - 11000)
    layer = ref.environment(c, h + 0.000001)
    assert layer["pressure"] == pytest.approx(22632.06, abs=1e-5)
    assert layer["a_sound"] == pytest.approx(np.sqrt(1.4 * 287.05287 * 216.65), abs=1e-9)


def test_body_table_matches_closed_coefficients_at_known_grid_node(cfg):
    cd, cl, cp = ref.body_table(cfg)(np.cos(np.pi / 4), 1.0)
    assert cd == pytest.approx(1.0 + 0.9 + 0.8 / (1 + np.exp(3)) + 1.0, abs=1e-14)
    assert cl == pytest.approx(1.25, abs=1e-14)
    assert cp == pytest.approx(-31.92, abs=1e-13)


def test_coupled_reference_reduces_to_analytic_rocket_equation(cfg):
    c = checks.base_config(cfg)
    c.plant.rcs.mode = "averaged"
    c.environment.gravity.model = "uniform"
    c.environment.atmosphere.g0 = 0.0
    c.scenario.ic.v_ned = [0.0, 0.0, 0.0]
    c.scenario.ic.w_body = [0.0, 0.0, 0.0]
    inputs = dict(throttle=[0.7] + [0.0] * 8, gimbal=[[0.0, 0.0]], fin=[0.0] * 4, rcs=[0.0] * 3)
    times = np.linspace(0, 1.0, 21)
    result = ref.coupled(c, times, inputs, 0.0)
    flow = 0.7 * 981000 / (311 * 9.80665)
    mass = 40000 - flow * times
    speed = 311 * 9.80665 * np.log(40000 / mass)
    distance = 311 * 9.80665 * (times - mass / flow * np.log(40000 / mass))
    axis = Rotation.from_euler("ZYX", [-27.0, 63.0, 18.0], degrees=True).apply([1.0, 0.0, 0.0])
    np.testing.assert_allclose(result["m"], mass, rtol=0, atol=1e-9)
    np.testing.assert_allclose(result["v_ned"], speed[:, None] * axis, rtol=0, atol=1e-10)
    np.testing.assert_allclose(
        result["r_ned"], np.array(c.scenario.ic.r_ned) + distance[:, None] * axis, rtol=0, atol=1e-9
    )
    assert np.max(np.linalg.norm(result["w_body"], axis=1)) < 1e-14
    assert np.max(attitude_error(result["q_bn"], result["q_bn"][:1])) < 1e-14


def test_references_do_not_call_production_aero_geometry_or_native(cfg, monkeypatch):
    import plant.aero_data
    import plant.dynamics

    def forbidden(*args, **kwargs):
        raise AssertionError("production calculation entered reference")

    monkeypatch.setattr(plant.aero_data, "generate_table", forbidden)
    monkeypatch.setattr(plant.dynamics, "build_plant", forbidden)
    ref.body_table(cfg)(0.5, 1.2)
    ref.fins(cfg, [-200.0, 0, 0], [0.0, 0, 0], [0.1] * 4, -27.0, 10000.0)
    ref.rcs(cfg, [1.0, 0, 0], 0.0, -27.0)
    _, local, inputs, deployment = checks.coupled_experiments(cfg)[0]
    ref.coupled(local, np.array([0.0, 0.01]), inputs, deployment)


def test_wrong_full_rhs_fin_force_is_detected_even_with_correct_diagnostic(cfg, monkeypatch):
    original = checks.build_plant

    class Broken:
        def __init__(self, native):
            self.native = native

        def __getattr__(self, name):
            return getattr(self.native, name)

        def rhs(self, *args):
            d, aux = self.native.rhs(*args)
            aux["F_fin_body"] = -np.asarray(aux["F_fin_body"])
            return d, aux

    def broken(c):
        bundle = original(c)
        return bundle._replace(native=Broken(bundle.native))

    monkeypatch.setattr(checks, "build_plant", broken)
    report, _, _ = checks.check_fins(cfg)
    assert report["status"] == "failed"
    assert any(not r["checks"]["force_n"]["passed"] for r in report["results"])


def test_limits_reject_nan_and_attitude_error_is_sign_invariant():
    assert not checks.row("nan", dict(x=np.nan), dict(x=1e-6))["passed"]
    q = np.array([[1.0, 0, 0, 0]])
    assert attitude_error(q, -q)[0] == 0.0
    assert not checks.row("small_but_excessive", dict(x=2e-6), dict(x=1e-6))["passed"]


@pytest.mark.parametrize("variant", ["complete", "missing", "failed", "mixed_sources"])
def test_run_summary_requires_all_stages_to_pass_from_matching_sources(tmp_path, variant):
    from scripts.check_vehicle_physics import write_json, write_summary

    for stage in range(1, 6 if variant != "missing" else 5):
        directory = tmp_path / f"{stage}_experiment"
        directory.mkdir()
        report = dict(
            stage=stage,
            name="experiment",
            cases=1,
            status="passed",
            source_sha256={"reference.py": "same_hash"},
        )
        if stage == 5 and variant == "failed":
            report["status"] = "failed"
        if stage == 5 and variant == "mixed_sources":
            report["source_sha256"] = {"reference.py": "different_hash"}
        write_json(directory / "report.json", report)
    write_summary(tmp_path)
    summary = json.loads((tmp_path / "summary.json").read_text())
    expected = "passed" if variant == "complete" else "incomplete_or_failed"
    assert summary["status"] == expected


def test_cli_preserves_failure_artifacts_and_exit_code_without_templates(tmp_path, monkeypatch):
    from scripts import check_vehicle_physics as runner

    failed = dict(
        stage=1,
        name="grid_fins",
        cases=1,
        status="failed",
        results=[checks.row("injected_failure", {"force_n": 1.0}, {"force_n": 1e-6})],
    )
    monkeypatch.setattr(runner.checks, "check_fins", lambda cfg: (failed, {}, {}))
    monkeypatch.setattr(
        sys, "argv", ["check_vehicle_physics.py", "--stage", "1", "--output", str(tmp_path)]
    )
    with pytest.raises(SystemExit) as exit_info:
        runner.main()
    assert exit_info.value.code == 2
    saved = json.loads((tmp_path / "1_grid_fins/report.json").read_text())
    assert saved["status"] == "failed"
    assert not saved["results"][0]["checks"]["force_n"]["passed"]
    assert (tmp_path / "1_grid_fins/checks.csv").is_file()
    assert json.loads((tmp_path / "summary.json").read_text())["status"] == "incomplete_or_failed"
    assert (tmp_path / "source_sha256.json").is_file()
    assert not list(tmp_path.rglob("*.md"))
