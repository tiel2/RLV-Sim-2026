"""Mass-property & propellant-budget anchors."""

from __future__ import annotations

import numpy as np
from conftest import build_bundle

from core.interfaces import ActuatorCommand
from plant.dynamics import initial_state

G0 = 9.80665
FALCON_SET = ["vehicle_set=falcon9", "environment/atmosphere=exp"]


def test_prescribed_cg_inertia_monotone_continuous():
    bundle = build_bundle([*FALCON_SET, "plant.vehicle.massprops_model=prescribed"])
    mp = bundle.params.mass
    ms = np.linspace(float(mp.m_dry), float(mp.m_wet), 200)
    props = [bundle.native.mass_properties_diagnostics(mass) for mass in ms]
    xcg = np.array([float(value["x_cg"]) for value in props])
    Ixx = np.array([float(value["inertia"][0]) for value in props])
    Iyy = np.array([float(value["inertia"][1]) for value in props])
    # monotone increasing with mass
    assert np.all(np.diff(xcg) >= -1e-12)
    assert np.all(np.diff(Ixx) >= -1e-12)
    assert np.all(np.diff(Iyy) >= -1e-12)
    # continuous: bounded step-to-step change
    assert np.max(np.abs(np.diff(xcg))) < 1.0
    assert np.max(np.abs(np.diff(Iyy))) / float(mp.I_wet[1]) < 0.05


def test_dual_tank_cg_continuous_and_bounded():
    bundle = build_bundle([*FALCON_SET, "plant.vehicle.massprops_model=dual_tank"])
    mp = bundle.params.mass
    ms = np.linspace(float(mp.m_dry), float(mp.m_wet), 200)
    xcg = np.array([float(bundle.native.mass_properties_diagnostics(mass)["x_cg"]) for mass in ms])
    assert np.max(np.abs(np.diff(xcg))) < 1.0  # continuous
    # CG stays within the vehicle (between base and nose)
    assert np.all(xcg <= 0.0) and np.all(xcg >= -float(bundle.params.geom.length))


def test_dual_tank_endpoints_are_derived_from_geometry():
    """The dry endpoint is structural; wet values are derived, not fitted to old constants."""
    dual = build_bundle(FALCON_SET)
    mp = dual.params.mass
    dry = dual.native.mass_properties_diagnostics(float(mp.m_dry), 0.0)
    wet = dual.native.mass_properties_diagnostics(float(mp.m_wet))
    assert np.isclose(dry["x_cg"], mp.x_cg_dry)
    assert np.allclose(dry["inertia"], mp.I_dry)
    assert np.isclose(wet["x_cg"], mp.x_cg_wet)
    assert np.allclose(wet["inertia"], mp.I_wet)
    # No arbitrary correction forces the cylindrical tanks back to the old wet endpoint.
    prescribed = build_bundle([*FALCON_SET, "plant.vehicle.massprops_model=prescribed"])
    assert not np.isclose(wet["x_cg"], prescribed.params.mass.x_cg_wet)
    assert not np.allclose(wet["inertia"], prescribed.params.mass.I_wet)


def test_tsiolkovsky_delta_v():
    """1-D burn, no gravity/drag => Δv = Isp·g0·ln(m0/m1) (rocket equation)."""
    bundle = build_bundle(
        [
            *FALCON_SET,
            "environment/gravity=uniform",
            "environment.atmosphere.g0=0.0",
            "environment.atmosphere.rho0=0.0",
            "environment.atmosphere.p_sea=0.0",
        ]
    )
    p = bundle.params
    ic = dict(
        r_ned=[0.0, 0.0, -8000.0],
        v_ned=[0.0, 0.0, 0.0],
        roll=0.0,
        pitch=np.pi / 2,
        yaw=0.0,
        w_body=[0, 0, 0],
        m=50000.0,
        throttle0=1.0,
        seed=0,
    )
    s0 = initial_state(ic, p)
    n, dt = 10000, float(p.plant_dt)  # 10 s
    cmd = ActuatorCommand(
        throttle=np.ones(1), gimbal=np.zeros((1, 2)), fin=np.zeros(4), rcs=np.zeros(3)
    )
    session = bundle.native.create_session(s0)
    final, _ = session.advance(cmd, n)

    m1 = float(final["m"])
    dv_numeric = float(np.linalg.norm(final["v_ned"]))
    dv_tsiolkovsky = float(p.engines.isp[0]) * G0 * np.log(50000.0 / m1)
    assert np.isclose(dv_numeric, dv_tsiolkovsky, rtol=1e-4)
    # propellant budget: burned mass = ∫ṁ dt = mdot·t
    mdot = float(p.engines.thrust_vac[0]) / (float(p.engines.isp[0]) * G0)
    assert np.isclose(50000.0 - m1, mdot * n * dt, rtol=1e-4)


def test_main_mass_floor_preserves_rcs_gas():
    """Main fuel exhaustion leaves RCS gas aboard and the ballistic simulation continues."""
    bundle = build_bundle(
        [
            *FALCON_SET,
            "environment/gravity=uniform",
            "environment.atmosphere.g0=0.0",
            "environment.atmosphere.rho0=0.0",
            "environment.atmosphere.p_sea=0.0",
        ]
    )
    p = bundle.params
    ic = dict(
        r_ned=[0.0, 0.0, -8000.0],
        v_ned=[0.0, 0.0, 0.0],
        roll=0.0,
        pitch=np.pi / 2,
        yaw=0.0,
        w_body=[0, 0, 0],
        m=50000.0,
        throttle0=1.0,
        seed=0,
    )
    s0 = initial_state(ic, p)
    n = 120000  # way past fuel exhaustion
    cmd = ActuatorCommand(
        throttle=np.ones(1), gimbal=np.zeros((1, 2)), fin=np.zeros(4), rcs=np.zeros(3)
    )
    session = bundle.native.create_session(s0)
    final, _ = session.advance(cmd, n)
    assert float(final["m"]) >= float(p.mass.m_dry) - 1e-6
    assert np.isclose(float(final["m"]), float(p.mass.m_dry + p.actuator.rcs_fuel0), atol=1e-6)
    assert not bool(final["done"])
