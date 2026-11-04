"""Independent inertial-frame and closed-form checks for a spherical rotating Earth.

Kepler ellipses test the complete rotating-frame propagation, not a second copy of
its fictitious-force RHS. Torque-free reference uses inertial rotation matrices.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from core.interfaces import ActuatorCommand
from plant.dynamics import build_plant, ic_from_scenario
from validation.references import mass_properties


def vacuum_setup(cfg, dt):
    """Caller supplies a resolved rotating/vacuum/ideal/no-wind/no-slosh config."""
    cfg = cfg.copy()
    cfg.plant_dt = dt
    bundle = build_plant(cfg)
    state = bundle.make_initial_state(ic_from_scenario(cfg.scenario, cfg.seed))
    state = state.replace(
        throttle_act=np.zeros(bundle.meta["n_engines"]),
        w_body=np.zeros(3),
        launch_pending=np.asarray(False),
        done=np.asarray(False),
    )
    command = ActuatorCommand(
        throttle=np.zeros(bundle.meta["n_engines"]),
        gimbal=np.zeros((bundle.meta["n_gimbal"], 2)),
        fin=np.zeros(bundle.meta["n_fins"]),
        rcs=np.zeros(3),
    )
    return bundle, state, command


def sample_native(bundle, state, command, duration, sample_dt):
    dt = float(bundle.params.plant_dt)
    total = round(duration / dt)
    stride = max(1, round(sample_dt / dt))
    ticks = np.unique(np.r_[np.arange(0, total + 1, stride), total])
    session = bundle.native.create_session(state)
    snapshots = [session.snapshot(True)]
    for count in np.diff(ticks):
        session.advance(command, int(count))
        snapshots.append(session.snapshot(True))
    return {k: np.array([row[k] for row in snapshots]) for k in snapshots[0]}


def inertial_history(history, radius, omega):
    """Reconstruct geocentric inertial state from Earth-fixed origin-axis records."""
    Q = Rotation.from_rotvec(history["t"][:, None] * omega).as_matrix()
    r_fixed = history["r_ned"] + [0, 0, -radius]
    position = np.einsum("nij,nj->ni", Q, r_fixed)
    velocity = np.einsum("nij,nj->ni", Q, history["v_ned"] + np.cross(omega, r_fixed))
    C_fixed = Rotation.from_quat(history["q_bn"], scalar_first=True).as_matrix()
    C = Q @ C_fixed
    rates = history["w_body"] + np.einsum("nji,j->ni", C_fixed, omega)
    return position, velocity, C, rates


def kepler_ellipse(times, mu, perigee, eccentricity, heading):
    """Closed-form conic in a fixed inertial plane, initialized at perigee."""
    e = eccentricity
    a = perigee / (1 - e)
    n = np.sqrt(mu / a**3)
    mean = n * np.asarray(times)
    anomaly = mean.copy()
    for _ in range(12):
        anomaly -= (anomaly - e * np.sin(anomaly) - mean) / (1 - e * np.cos(anomaly))
    p = np.array([0.0, 0.0, -1.0])
    q = np.array([np.sin(heading), np.cos(heading), 0.0])
    position = a * (
        (np.cos(anomaly) - e)[:, None] * p + (np.sqrt(1 - e**2) * np.sin(anomaly))[:, None] * q
    )
    velocity = (a * n / (1 - e * np.cos(anomaly)))[:, None] * (
        -np.sin(anomaly)[:, None] * p + (np.sqrt(1 - e**2) * np.cos(anomaly))[:, None] * q
    )
    return position, velocity


def orbit_case(cfg, *, eccentricity=0.0, heading_deg=0.0, dt=0.5):
    bundle, state, command = vacuum_setup(cfg, dt)
    mu, radius = float(bundle.params.gravity.mu), float(bundle.params.gravity.radius)
    omega = np.asarray(bundle.params.earth.omega_ned)
    perigee = radius + 400000.0
    heading = np.deg2rad(heading_deg)
    initial_r, initial_v = kepler_ellipse([0.0], mu, perigee, eccentricity, heading)
    C0 = Rotation.from_quat(state.q_bn, scalar_first=True).as_matrix()
    state = state.replace(
        r_ned=initial_r[0] - [0, 0, -radius],
        v_ned=initial_v[0] - np.cross(omega, initial_r[0]),
        w_body=-C0.T @ omega,  # zero INERTIAL spin, not zero Earth-relative spin
    )
    period = 2 * np.pi * np.sqrt((perigee / (1 - eccentricity)) ** 3 / mu)
    history = sample_native(bundle, state, command, period, 10.0)
    r, v, C, rates = inertial_history(history, radius, omega)
    expected_r, expected_v = kepler_ellipse(history["t"], mu, perigee, eccentricity, heading)
    energy = np.sum(v * v, axis=1) / 2 - mu / np.linalg.norm(r, axis=1)
    momentum = np.cross(r, v)
    errors = {
        "position_m": float(np.max(np.linalg.norm(r - expected_r, axis=1))),
        "velocity_mps": float(np.max(np.linalg.norm(v - expected_v, axis=1))),
        "relative_specific_energy": float(np.max(abs((energy - energy[0]) / energy[0]))),
        "relative_angular_momentum": float(
            np.max(np.linalg.norm(momentum - momentum[0], axis=1) / np.linalg.norm(momentum[0]))
        ),
        "inertial_attitude_rad": float(np.max(Rotation.from_matrix(C @ C0.T).magnitude())),
        "inertial_rate_rad_s": float(np.max(np.linalg.norm(rates, axis=1))),
    }
    limits = dict(
        position_m=0.005,
        velocity_mps=1e-5,
        relative_specific_energy=1e-10,
        relative_angular_momentum=1e-10,
        inertial_attitude_rad=1e-9,
        inertial_rate_rad_s=1e-10,
    )
    completed = not np.any(history["done"]) and abs(history["t"][-1] - period) <= dt / 2 + 1e-9
    report = dict(
        status="passed" if completed and all(errors[k] <= limits[k] for k in errors) else "failed",
        eccentricity=eccentricity,
        heading_deg=heading_deg,
        latitude_deg=float(cfg.environment.earth.latitude_deg),
        dt_s=dt,
        duration_s=float(history["t"][-1]),
        completed=bool(completed),
        errors=errors,
        limits=limits,
        # A whole orbit must survive positive reference-D coordinates (below the tangent plane).
        maximum_reference_down_m=float(np.max(history["r_ned"][:, 2])),
    )
    return report, dict(
        t=history["t"],
        position_error_m=np.linalg.norm(r - expected_r, axis=1),
        velocity_error_mps=np.linalg.norm(v - expected_v, axis=1),
        inertial_position_m=r,
        expected_position_m=expected_r,
    )


def torque_free_case(cfg, *, dt=0.01, duration=120.0):
    bundle, state, command = vacuum_setup(cfg, dt)
    omega = np.asarray(bundle.params.earth.omega_ned)
    radius = float(bundle.params.gravity.radius)
    C0 = Rotation.from_quat(state.q_bn, scalar_first=True).as_matrix()
    inertial_rate0 = np.array([0.017, -0.023, 0.031])
    state = state.replace(
        r_ned=np.array([12000.0, -18000.0, -400000.0]), w_body=inertial_rate0 - C0.T @ omega
    )
    history = sample_native(bundle, state, command, duration, 1.0)
    _, _, C, rates = inertial_history(history, radius, omega)
    _, inertia = mass_properties(cfg.plant.vehicle, float(state.m), float(state.rcs_fuel))

    def rhs(_, y):
        matrix, w = y[:9].reshape(3, 3), y[9:]
        wx, wy, wz = w
        skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
        return np.r_[(matrix @ skew).ravel(), np.linalg.solve(inertia, -np.cross(w, inertia @ w))]

    reference = solve_ivp(
        rhs,
        (0, duration),
        np.r_[C0.ravel(), inertial_rate0],
        t_eval=history["t"],
        method="DOP853",
        rtol=1e-12,
        atol=1e-14,
    )
    if not reference.success:
        raise RuntimeError(reference.message)
    expected_C = reference.y[:9].T.reshape(-1, 3, 3)
    angular = np.einsum("nij,nj->ni", C, rates @ inertia.T)
    energy = np.einsum("ni,ij,nj->n", rates, inertia, rates) / 2
    errors = dict(
        attitude_rad=float(
            np.max(Rotation.from_matrix(C @ expected_C.transpose(0, 2, 1)).magnitude())
        ),
        rate_rad_s=float(np.max(np.linalg.norm(rates - reference.y[9:].T, axis=1))),
        relative_angular_momentum=float(
            np.max(np.linalg.norm(angular - angular[0], axis=1) / np.linalg.norm(angular[0]))
        ),
        relative_rotational_energy=float(np.max(abs((energy - energy[0]) / energy[0]))),
    )
    limits = dict(
        attitude_rad=1e-7,
        rate_rad_s=1e-9,
        relative_angular_momentum=1e-9,
        relative_rotational_energy=1e-9,
    )
    return dict(
        status="passed" if all(errors[k] <= limits[k] for k in errors) else "failed",
        duration_s=duration,
        dt_s=dt,
        errors=errors,
        limits=limits,
    )
