from types import SimpleNamespace

import numpy as np

from core.interfaces import StateHistory
from gnc.navigation.synthetic_eskf import SyntheticESKF, UnfilteredINS, build, rot
from validation.navigation import compare_navigation


def test_rotation_identity():
    assert np.allclose(rot([1,0,0,0]),np.eye(3))


def test_stationary_navigation_and_dropout():
    nav=SyntheticESKF(imu_rate_hz=50,accel_bias=(0,0,0),gyro_bias_deg=(0,0,0),
       gnss_pos_sigma=2.5,gnss_vel_sigma=0.15,gnss_dropout=0,radar_sigma=0.5,
       radar_period=0.1)
    mem={}
    for i in range(101):
        t=i*0.02
        # rigid body at rest; a=0, so accelerometer must measure -g in NED
        state=SimpleNamespace(t=t,r_ned=np.array([0.,0.,-100.]),
          v_ned=np.zeros(3),q_bn=np.array([1.,0.,0.,0.]),w_body=np.zeros(3),m=40000.)
        obs=nav.measure(state,np.array([12,34],dtype=np.uint32))
        est,mem=nav.estimate(obs,None,mem)
    assert np.linalg.norm(est.r_ned-np.array([0,0,-100]))<3.0
    assert np.linalg.norm(est.v_ned)<0.5
    assert np.isclose(np.linalg.norm(est.q_bn),1)
    assert mem['P'].shape==(15,15)
    assert np.all(np.linalg.eigvalsh(mem['P']) >= -1e-8)


def test_gnss_outage():
    nav=SyntheticESKF(gnss_dropout=0,gnss_outage_start=1.,gnss_outage_end=2.)
    state=SimpleNamespace(t=1.5,r_ned=np.array([0.,0.,-2000.]),v_ned=np.zeros(3),
       q_bn=np.array([1.,0.,0.,0.]),w_body=np.zeros(3),m=40000.)
    obs=nav.measure(state,[1,2])
    assert obs.gnss_scheduled
    assert not obs.gnss_valid
    assert obs.radar_scheduled
    assert not obs.radar_valid


def test_stationary_accelerometer_reports_specific_force_without_gravity_leak():
    nav = SyntheticESKF(
        imu_accel_sigma=0,
        imu_gyro_sigma_deg=0,
        accel_bias=(0, 0, 0),
        gyro_bias_deg=(0, 0, 0),
        gnss_dropout=0,
    )
    state = SimpleNamespace(
        t=0.0,
        r_ned=np.array([0.0, 0.0, -100.0]),
        v_ned=np.zeros(3),
        q_bn=np.array([1.0, 0.0, 0.0, 0.0]),
        w_body=np.zeros(3),
        m=40000.0,
    )
    obs = nav.measure(state, 1)
    altitude = 100.0
    gravity = 9.80665 * (6371000.0 / (6371000.0 + altitude)) ** 2
    assert np.allclose(obs.accel_body, [0.0, 0.0, -gravity])
    assert np.array_equal(obs.q_bn, [1.0, 0.0, 0.0, 0.0])


def test_unfiltered_ins_tracks_known_constant_acceleration():
    dt = 0.005
    acceleration = np.array([1.0, 0.0, 0.0])
    initial = SimpleNamespace(
        t=0.0,
        r_ned=np.array([0.0, 0.0, -100.0]),
        v_ned=np.zeros(3),
        q_bn=np.array([1.0, 0.0, 0.0, 0.0]),
        w_body=np.zeros(3),
        m=40000.0,
    )
    nav = SyntheticESKF(
        initial_state=initial,
        imu_accel_sigma=0,
        imu_gyro_sigma_deg=0,
        accel_bias=(0, 0, 0),
        gyro_bias_deg=(0, 0, 0),
        gnss_period=10.0,
        radar_range=1.0,
        gnss_dropout=0,
    )
    ins = UnfilteredINS(initial)
    memory = ins.initial_memory()
    estimate = None
    for index in range(201):
        t = index * dt
        state = SimpleNamespace(
            t=t,
            r_ned=initial.r_ned + 0.5 * acceleration * t**2,
            v_ned=acceleration * t,
            q_bn=initial.q_bn,
            w_body=initial.w_body,
            m=initial.m,
        )
        measurement = nav.measure(state, index)
        estimate, memory = ins.estimate(measurement, None, memory)
    assert np.allclose(estimate.v_ned, acceleration * 1.0, atol=dt)
    assert np.allclose(
        estimate.r_ned,
        initial.r_ned + 0.5 * acceleration,
        atol=dt,
    )


def test_sensor_packet_does_not_expose_truth_attitude_or_mass():
    nav = SyntheticESKF(initial_attitude=[1.0, 0.0, 0.0, 0.0], initial_mass=40000.0)
    state = SimpleNamespace(
        t=0.0,
        r_ned=np.array([0.0, 0.0, -100.0]),
        v_ned=np.zeros(3),
        q_bn=np.array([0.0, 0.0, 1.0, 0.0]),
        w_body=np.zeros(3),
        m=39000.0,
    )
    observation = nav.measure(state, 1)
    assert np.array_equal(observation.q_bn, [1.0, 0.0, 0.0, 0.0])
    assert observation.m == 40000.0


def test_sensor_rate_scale_factor_and_bias_random_walk():
    nav = SyntheticESKF(
        imu_accel_sigma=0,
        imu_gyro_sigma_deg=0,
        accel_bias=(0, 0, 0),
        gyro_bias_deg=(0, 0, 0),
        accel_scale_factor=(0, 0, 0.1),
        accel_bias_random_walk=0.1,
        gnss_dropout=0,
    )
    state = SimpleNamespace(
        t=0.0,
        r_ned=np.array([0.0, 0.0, -100.0]),
        v_ned=np.zeros(3),
        q_bn=np.array([1.0, 0.0, 0.0, 0.0]),
        w_body=np.zeros(3),
        m=40000.0,
    )
    first = nav.measure(state, 1)
    state.t = 0.005
    second = nav.measure(state, 2)
    gravity = 9.80665 * (6371000.0 / (6371000.0 + 100.0)) ** 2
    assert np.isclose(first.accel_body[2], -1.1 * gravity)
    assert not np.array_equal(first.accel_body, second.accel_body)

    state.t = 0.015
    with np.testing.assert_raises_regex(ValueError, "does not match configured period"):
        nav.measure(state, 3)


def test_sensor_rate_must_match_simulation_navigation_period():
    context = SimpleNamespace(
        cfg={"imu_rate_hz": 200.0},
        initial_state=None,
        nav_dt=0.01,
    )
    with np.testing.assert_raises_regex(ValueError, "nav_dt to match"):
        build(context)


def test_comparison_reuses_sensor_samples_and_reports_all_navigation_errors():
    count = 2001
    t = np.arange(count) * 0.005
    truth = StateHistory(
        t=t,
        r_ned=np.tile([0.0, 0.0, -100.0], (count, 1)),
        v_ned=np.zeros((count, 3)),
        q_bn=np.tile([1.0, 0.0, 0.0, 0.0], (count, 1)),
        w_body=np.zeros((count, 3)),
        m=np.full(count, 40000.0),
    )
    initial = SimpleNamespace(
        t=t[0],
        r_ned=truth.r_ned[0],
        v_ned=truth.v_ned[0],
        q_bn=truth.q_bn[0],
        w_body=truth.w_body[0],
        m=truth.m[0],
    )
    config = dict(
        initial_state=initial,
        accel_bias=(0.03, -0.02, 0.04),
        gyro_bias_deg=(0.005, -0.006, 0.004),
        gnss_dropout=0.0,
        gnss_outage_start=4.0,
        gnss_outage_end=6.0,
        gnss_outlier_times=(2.0,),
    )
    first, first_metrics = compare_navigation(truth, SyntheticESKF(**config), seed=12)
    second, second_metrics = compare_navigation(truth, SyntheticESKF(**config), seed=12)

    assert np.array_equal(first["measurement_accel_body"], second["measurement_accel_body"])
    assert np.array_equal(first["measurement_gnss_valid"], second["measurement_gnss_valid"])
    assert np.allclose(first["filtered_r_ned"], second["filtered_r_ned"])
    assert np.allclose(first["unfiltered_r_ned"], second["unfiltered_r_ned"])
    assert first_metrics == second_metrics
    for estimator in ("unfiltered", "filtered"):
        assert set(first_metrics[estimator]) == {
            "position_m",
            "velocity_mps",
            "attitude_rad",
            "attitude_euler_deg",
            "angular_rate_radps",
        } | (
            {"attitude_vector_rad", "gyro_bias_radps", "accel_bias_mps2"}
            if estimator == "filtered"
            else set()
        )
        assert first[f"{estimator}_error_attitude_rad"].shape == (count,)
    assert first_metrics["sensors"]["gnss_samples"] == 80
    assert first_metrics["sensors"]["gnss_scheduled_samples"] == 101
    assert first_metrics["sensors"]["gnss_dropouts"] == 0
    assert first_metrics["sensors"]["gnss_outage_scheduled_samples"] == 21
    assert first_metrics["sensors"]["gnss_outage_samples"] == 21
    assert first_metrics["sensors"]["gnss_outliers"] == 1
    assert first_metrics["innovation"]["gnss_rejected"] == 1
    assert np.count_nonzero(first["gnss_update_accepted"]) == 79
    assert (
        first_metrics["filtered"]["position_m"]["rmse"]
        < first_metrics["unfiltered"]["position_m"]["rmse"]
    )
    assert (
        first_metrics["filtered"]["velocity_mps"]["rmse"]
        < first_metrics["unfiltered"]["velocity_mps"]["rmse"]
    )
