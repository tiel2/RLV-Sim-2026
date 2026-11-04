"""Physical invariants and independent inertial solutions for rotating Earth."""

import numpy as np
import pytest
from conftest import build_bundle, compose_cfg

from plant.earth import altitude
from validation.rotating_earth import orbit_case, torque_free_case, vacuum_setup

VACUUM = [
    "environment/earth=rotating",
    "environment.earth.latitude_deg=28.6",
    "environment/atmosphere=vacuum",
    "plant/thrust=vacuum",
    "plant/actuator=ideal",
    "plant/slosh=none",
    "environment/wind=none",
    "plant.rcs.mode=averaged",
]


@pytest.mark.parametrize(
    ("eccentricity", "heading", "latitude"), [(0.0, 0.0, 0.0), (0.2, 63.0, -37.0)]
)
def test_whole_kepler_orbit_and_inertial_invariants(eccentricity, heading, latitude):
    cfg = compose_cfg(VACUUM)
    cfg.environment.earth.latitude_deg = latitude
    report, _ = orbit_case(cfg, eccentricity=eccentricity, heading_deg=heading)
    assert report["status"] == "passed", report
    assert report["maximum_reference_down_m"] > 1e7


def test_torque_free_asymmetric_body_matches_inertial_euler_dynamics():
    cfg = compose_cfg(VACUUM)
    cfg.plant.vehicle.mass.I_dry[2] *= 1.15
    cfg.plant.vehicle.mass.inertia_products = [100.0, -200.0, 300.0]
    report = torque_free_case(cfg)
    assert report["status"] == "passed", report


@pytest.mark.parametrize("latitude", [-90.0, -37.0, 0.0, 28.6, 90.0])
def test_stationary_point_acceleration_is_geocentric_gravity_plus_centrifugal(latitude):
    cfg = compose_cfg(VACUUM)
    cfg.environment.earth.latitude_deg = latitude
    bundle, state, command = vacuum_setup(cfg, 0.001)
    R = float(bundle.params.gravity.radius)
    mu = float(bundle.params.gravity.mu)
    omega = np.asarray(bundle.params.earth.omega_ned)
    state = state.replace(r_ned=np.array([900000.0, -700000.0, -300000.0]), v_ned=np.zeros(3))
    derivative, aux = bundle.native.rhs(state, command, np.zeros(3))
    geocentric = state.r_ned + [0, 0, -R]
    expected_gravity = -mu * geocentric / np.linalg.norm(geocentric) ** 3
    assert np.allclose(
        derivative[2],
        expected_gravity - np.cross(omega, np.cross(omega, geocentric)),
        atol=1e-13,
        rtol=1e-13,
    )
    assert aux["altitude"] == pytest.approx(np.linalg.norm(geocentric) - R, abs=1e-8)
    assert altitude(state.r_ned, bundle.params) == pytest.approx(aux["altitude"], abs=1e-8)


@pytest.mark.parametrize(
    "override",
    ["environment/gravity=uniform", "environment.earth.include_differential_centrifugal=false"],
)
def test_reject_incomplete_rotating_earth_configuration(override):
    with pytest.raises(ValueError, match="full centrifugal"):
        build_bundle([*VACUUM, override])


def test_spherical_contact_occurs_above_the_old_tangent_plane():
    cfg = compose_cfg(VACUUM)
    bundle, state, command = vacuum_setup(cfg, 0.001)
    R = float(bundle.params.gravity.radius)
    north = 100000.0
    surface_D = R - np.sqrt(R**2 - north**2)
    state = state.replace(
        r_ned=np.array([north, 0.0, surface_D - 0.01]), v_ned=np.array([0.0, 0.0, 10.0])
    )
    session = bundle.native.create_session(state)
    assert not session.snapshot()["done"]
    snapshot, aux = session.advance(command, 10)
    assert snapshot["done"] and aux["altitude"] <= 0
    assert snapshot["t"] <= 0.002


def test_rotating_pad_reaction_supports_inertial_angular_momentum_change():
    cfg = compose_cfg(VACUUM)
    bundle, state, command = vacuum_setup(cfg, 0.001)
    state = state.replace(
        r_ned=np.zeros(3), v_ned=np.zeros(3), w_body=np.zeros(3), launch_pending=np.asarray(True)
    )
    derivative, aux = bundle.native.rhs(state, command, np.zeros(3))
    inertia = np.diag(aux["inertia_diag"])
    omega = aux["omega_inertial_body"]
    assert np.allclose(aux["M_pad_body"] + aux["M_cg_body"], np.cross(omega, inertia @ omega))
    assert np.array_equal(derivative[4], np.zeros(3))
