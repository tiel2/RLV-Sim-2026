"""Physical sensor channels, rotating-frame INS, initialization, and error reset."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from conftest import compose_cfg
from hydra.errors import InstantiationException
from scipy.spatial.transform import Rotation

from gnc.build import build_nav
from gnc.navigation.synthetic_eskf import (
    NavigationFrame,
    SyntheticESKF,
    dq_body,
    left_jacobian,
    qmul,
    rot,
)
from validation.rotating_earth import vacuum_setup


def sample_state(t=0.0):
    return SimpleNamespace(
        t=t,
        r_ned=np.array([0.0, 0.0, -100.0]),
        v_ned=np.zeros(3),
        q_bn=np.array([1.0, 0.0, 0.0, 0.0]),
        w_body=np.zeros(3),
        m=40000.0,
    )


def test_physical_imu_zero_in_freefall_and_nonzero_on_pad():
    cfg = compose_cfg(
        [
            "scenario=project2_landing",
            "gnc/navigation=synthetic_eskf",
            "environment/atmosphere=vacuum",
        ]
    )
    bundle, state, command = vacuum_setup(cfg, 0.001)
    nav = build_nav(cfg)
    nav.sa = nav.sg = nav.alignment_sigma = 0
    nav.ba[:] = nav.bg[:] = 0
    for pad in (False, True):
        s = state.replace(
            r_ned=np.array([0.0, 0.0, 0.0 if pad else -1000.0]),
            v_ned=np.zeros(3),
            w_body=np.zeros(3),
            launch_pending=np.asarray(pad),
        )
        _, aux = bundle.native.rhs(s, command, np.zeros(3))
        obs = nav.measure_native(s, 1, aux)
        if pad:
            expected = -rot(s.q_bn).T @ (aux["gravity_ned"] + aux["a_centrifugal_ned"])
            np.testing.assert_allclose(obs.accel_body, expected, atol=1e-12)
        else:
            np.testing.assert_allclose(obs.accel_body, 0, atol=1e-12)
        np.testing.assert_allclose(
            obs.gyro_body, rot(s.q_bn).T @ bundle.params.earth.omega_ned, atol=1e-14
        )
        assert np.linalg.norm(obs.gyro_body) > 7e-5


def test_stationary_rotating_ins_uses_inertial_gyro_and_returns_relative_rate():
    omega = np.array([6.4e-5, 0.0, -3.5e-5])
    frame = NavigationFrame(rotating=True, omega=tuple(omega))
    nav = SyntheticESKF(
        frame=frame,
        accel_bias=(0, 0, 0),
        gyro_bias_deg=(0, 0, 0),
        imu_accel_sigma=0,
        imu_gyro_sigma_deg=0,
        alignment_sigma_deg=0,
        gnss_dropout=0,
    )
    s = sample_state()
    s.r_ned = np.array([500000.0, 200000.0, -100000.0])
    s.q_bn = dq_body([0.3, -0.2, 0.5])
    R = s.r_ned + [0, 0, -frame.radius]
    acceleration = -frame.mu * R / np.linalg.norm(R) ** 3 - np.cross(omega, np.cross(omega, R))
    mem = {}
    for k in range(1001):
        s.t = k * 0.01
        obs = nav.measure(
            s,
            1,
            specific_force_body=-rot(s.q_bn).T @ acceleration,
            inertial_rate_body=rot(s.q_bn).T @ omega,
        )
        # Exact initialization isolates noiseless INS propagation, with no aiding afterward.
        obs = replace(
            obs, r_ned=s.r_ned.copy(), v_ned=s.v_ned.copy(), gnss_valid=k == 0, radar_valid=False
        )
        est, mem = nav.estimate(obs, None, mem)
    np.testing.assert_allclose(est.r_ned, s.r_ned, atol=1e-7)
    np.testing.assert_allclose(est.v_ned, 0, atol=1e-9)
    np.testing.assert_allclose(est.w_body, 0, atol=1e-13)
    np.testing.assert_allclose(rot(est.q_bn), rot(s.q_bn), atol=1e-12)


def test_startup_outage_uses_declared_prior_then_acquires_gnss_without_nan():
    nav = SyntheticESKF(
        initial_position=[10.0, -8.0, -95.0],
        initial_velocity=[0.5, -0.4, 0.3],
        gnss_outage_start=0,
        gnss_outage_end=1,
        gnss_dropout=0,
    )
    mem = {}
    for k in range(301):
        s = sample_state(k * 0.01)
        obs = nav.measure(
            s, [1, 2], specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0]
        )
        est, mem = nav.estimate(obs, None, mem)
        assert np.all(np.isfinite(np.r_[est.r_ned, est.v_ned, est.q_bn]))
        if k == 0:
            np.testing.assert_allclose(est.r_ned[:2], [10, -8])
            assert mem["initialization_source"] == "declared_prior"
    assert mem["first_gnss_time"] == pytest.approx(1.1)
    assert np.linalg.norm(est.r_ned - s.r_ned) < 2.0
    assert np.linalg.eigvalsh(mem["P"])[0] > 0


def test_missing_gnss_and_missing_prior_rejected_before_feedback():
    nav = SyntheticESKF(gnss_dropout=1)
    obs = nav.measure(
        sample_state(), 1, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0]
    )
    with pytest.raises(ValueError, match="explicit position/velocity prior"):
        nav.estimate(obs, None, {})


def test_initial_alignment_is_uncertain_and_truth_quaternion_not_reused():
    nav = SyntheticESKF(alignment_sigma_deg=1, gnss_dropout=0)
    s = sample_state()
    obs = nav.measure(s, 1, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0])
    assert not np.allclose(obs.q_bn, s.q_bn)
    _, mem = nav.estimate(obs, None, {})
    s.t = 0.01
    obs = nav.measure(s, 1, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0])
    assert not obs.alignment_valid and np.all(np.isnan(obs.q_bn))
    a, _ = nav.estimate(obs, None, mem)
    b, _ = nav.estimate(replace(obs, q_bn=dq_body([1.0, 2.0, 3.0])), None, mem)
    np.testing.assert_array_equal(a.q_bn, b.q_bn)


def test_initial_gnss_sample_not_fused_twice_and_input_not_mutated():
    nav = SyntheticESKF(gnss_dropout=0)
    obs = nav.measure(
        sample_state(), 1, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0]
    )
    obs = replace(obs, radar_valid=False)
    initial = obs.r_ned.copy()
    _, mem = nav.estimate(obs, None, {})
    np.testing.assert_array_equal(np.diag(mem["P"])[:3], np.full(3, nav.sp**2))
    np.testing.assert_array_equal(obs.r_ned, initial)
    assert mem["innovations"] == []


def test_radial_altimeter_and_gravity_jacobian_away_from_origin():
    frame = NavigationFrame(rotating=True, omega=(6.4e-5, 0, -3.5e-5))
    r = np.array([800000.0, 300000.0, -100000.0])
    h, H = frame.height_gradient(r)
    assert h != pytest.approx(-r[2])
    v = np.array([1200.0, 300.0, -500.0])
    _, J = frame.acceleration_gradient(r, v)
    for i in range(3):
        d = np.eye(3)[i]
        num_h = (frame.height_gradient(r + d)[0] - frame.height_gradient(r - d)[0]) / 2
        num_g = (
            frame.acceleration_gradient(r + d, v)[0] - frame.acceleration_gradient(r - d, v)[0]
        ) / 2
        assert num_h == pytest.approx(H[i], abs=1e-9)
        np.testing.assert_allclose(num_g, J[:, i], atol=1e-12)


def test_global_error_reset_jacobian_matches_finite_rotation_difference():
    phi = np.array([0.2, -0.15, 0.1])
    J = left_jacobian(phi)
    for i in range(3):
        eps = np.eye(3)[i] * 1e-7
        plus = qmul(dq_body(phi + eps), dq_body(-phi))
        minus = qmul(dq_body(phi - eps), dq_body(-phi))
        numeric = (
            Rotation.from_quat(plus, scalar_first=True).as_rotvec()
            - Rotation.from_quat(minus, scalar_first=True).as_rotvec()
        ) / 2e-7
        np.testing.assert_allclose(numeric, J[:, i], atol=1e-8)


def test_noiseless_native_ballistic_ins_across_changing_gravity():
    cfg = compose_cfg(
        [
            "scenario=project2_landing",
            "gnc/navigation=synthetic_eskf",
            "environment/atmosphere=vacuum",
        ]
    )
    bundle, initial, command = vacuum_setup(cfg, 0.001)
    initial = initial.replace(
        r_ned=np.array([200000.0, 100000.0, -300000.0]),
        v_ned=np.array([1400.0, -300.0, -500.0]),
        w_body=np.zeros(3),
    )
    session = bundle.native.create_session(initial)
    nav = build_nav(cfg)
    nav.sa = nav.sg = nav.alignment_sigma = 0
    nav.ba[:] = nav.bg[:] = 0
    mem = {}
    for k in range(501):
        snap, aux = session.observe() if k == 0 else session.advance(command, 10)
        state = SimpleNamespace(**snap)
        obs = nav.measure_native(state, 3, aux)
        obs = replace(
            obs,
            gnss_valid=k == 0,
            radar_valid=False,
            r_ned=state.r_ned.copy(),
            v_ned=state.v_ned.copy(),
        )
        est, mem = nav.estimate(obs, None, mem)
    np.testing.assert_allclose(est.r_ned, state.r_ned, atol=1e-6, rtol=0)
    np.testing.assert_allclose(est.v_ned, state.v_ned, atol=1e-7, rtol=0)
    np.testing.assert_allclose(rot(est.q_bn), rot(state.q_bn), atol=1e-8, rtol=0)


def test_native_sensor_channels_export_at_navigation_rate(tmp_path):
    from sim.run import run_configured
    from sim.telemetry import write_telemetry

    cfg = compose_cfg(["scenario=project2_landing", "gnc/navigation=synthetic_eskf"])
    cfg.scenario.horizon.t_max = 0.05
    _, result = run_configured(cfg)
    paths = write_telemetry(result, tmp_path)
    data = np.load(paths["navigation"])
    assert len(data["truth_specific_force_body"]) == len(data["truth_t"])
    assert len(data["sensor_accel_body"]) == len(data["truth_t"])
    assert data["sensor_alignment_valid"].tolist() == [True, False, False, False, False]
    assert np.isnan(data["measurement_q_bn"][1:]).all()
    assert np.isfinite(data["estimate_q_bn"]).all()
    assert not np.allclose(data["sensor_gyro_body"], data["truth_w_body"])


def test_optional_scale_bias_walk_and_rate_contract():
    nav = SyntheticESKF(
        imu_accel_sigma=0,
        imu_gyro_sigma_deg=0,
        accel_bias=(0, 0, 0),
        gyro_bias_deg=(0, 0, 0),
        accel_scale_factor=(0, 0, 0.1),
        accel_bias_random_walk=0.1,
    )
    state = sample_state()
    first = nav.measure(state, 1, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0])
    assert first.accel_body[2] == pytest.approx(-1.1 * 9.82)
    state.t = 0.01
    second = nav.measure(state, 2, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0])
    assert not np.array_equal(first.accel_body, second.accel_body)
    state.t = 0.03
    with pytest.raises(ValueError, match="does not match configured period"):
        nav.measure(state, 3, specific_force_body=[0, 0, -9.82], inertial_rate_body=[0, 0, 0])
    cfg = compose_cfg(
        [
            "scenario=project2_landing",
            "gnc/navigation=synthetic_eskf",
            "gnc.navigation.imu_rate_hz=200",
        ]
    )
    with pytest.raises(InstantiationException, match="nav_dt to match"):
        build_nav(cfg)
    prescribed = compose_cfg(["scenario=project2_ascent", "gnc/navigation=synthetic_eskf"])
    assert build_nav(prescribed).imu_rate_hz == 100


def test_gnss_outlier_gate_rejects_gross_error_but_nominal_filter_accepts_it():
    from copy import deepcopy

    sensor = SyntheticESKF(
        gnss_outlier_times=(0.2,), gnss_dropout=0, gnss_gate_threshold=16.812, alignment_sigma_deg=0
    )
    unguarded = deepcopy(sensor)
    unguarded.gnss_gate = None
    memories = [{}, {}]
    for k in range(21):
        obs = sensor.measure(
            sample_state(k * 0.01),
            [1, 2],
            specific_force_body=[0, 0, -9.82],
            inertial_rate_body=[0, 0, 0],
        )
        for j, nav in enumerate((sensor, unguarded)):
            estimate, memories[j] = nav.estimate(deepcopy(obs), None, memories[j])
    assert obs.gnss_scheduled and obs.gnss_valid and obs.gnss_outlier
    assert memories[0]["gnss_nis"] > 16.812
    assert not memories[0]["gnss_update_accepted"]
    assert memories[1]["gnss_update_accepted"]
    assert np.linalg.norm(memories[0]["r"][:2]) < 3
    assert np.linalg.norm(memories[1]["r"][:2]) > 10


def test_unfiltered_ins_uses_same_initial_observation_and_constant_acceleration():
    from gnc.navigation.synthetic_eskf import UnfilteredINS

    nav = SyntheticESKF(
        imu_accel_sigma=0,
        imu_gyro_sigma_deg=0,
        accel_bias=(0, 0, 0),
        gyro_bias_deg=(0, 0, 0),
        alignment_sigma_deg=0,
        gnss_dropout=0,
    )
    ins, memory = UnfilteredINS(nav), {}
    g = nav.frame.mu / (nav.frame.radius + 100) ** 2
    for k in range(101):
        state = sample_state(k * 0.01)
        state.r_ned += [0.5 * state.t**2, 0, 0]
        state.v_ned = np.array([state.t, 0.0, 0.0])
        obs = nav.measure(state, k, specific_force_body=[1, 0, -g], inertial_rate_body=[0, 0, 0])
        obs = replace(obs, r_ned=state.r_ned.copy(), v_ned=state.v_ned.copy(), radar_valid=False)
        estimate, memory = ins.estimate(obs, None, memory)
    np.testing.assert_allclose(estimate.r_ned, [0.5, 0, -100], atol=1e-9)
    np.testing.assert_allclose(estimate.v_ned, [1, 0, 0], atol=1e-9)


def test_shared_native_replay_is_deterministic_and_does_not_use_truth_initialization(tmp_path):
    from sim.run import run_configured
    from validation.navigation import compare_navigation, write_navigation_comparison

    cfg = compose_cfg(["scenario=project2_landing", "gnc/navigation=synthetic_eskf"])
    cfg.scenario.horizon.t_max = 0.3
    _, result = run_configured(cfg)
    sensor = build_nav(cfg)
    a, metrics = compare_navigation(result.navigation, sensor, 12)
    b, metrics2 = compare_navigation(result.navigation, sensor, 12)
    assert metrics == metrics2
    np.testing.assert_array_equal(a["measurement_accel_body"], b["measurement_accel_body"])
    np.testing.assert_array_equal(a["filtered_r_ned"][0], a["unfiltered_r_ned"][0])
    np.testing.assert_array_equal(a["filtered_q_bn"][0], a["unfiltered_q_bn"][0])
    assert not np.allclose(a["filtered_q_bn"][0], a["truth_q_bn"][0])
    assert metrics["sensors"]["imu_rate_hz"] == 100
    assert metrics["sensors"]["gnss_outliers"] == 0
    with pytest.raises(ValueError, match="requires native"):
        compare_navigation(result.navigation.truth, sensor, 12)
    metadata = write_navigation_comparison(
        result.navigation, sensor, 12, tmp_path, scenario="test", sensor_configuration={}
    )
    assert metadata["metrics"] == metrics
    assert (tmp_path / "navigation_comparison.json").exists()
    assert all(__import__("pathlib").Path(p).exists() for p in metadata["plots"].values())
