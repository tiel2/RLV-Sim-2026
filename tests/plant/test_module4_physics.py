"""Physical anchors for Module 4 pp.11–23, independent of trajectory tuning."""

import numpy as np
import pytest
from conftest import build_bundle

from core.frames import dcm_from_quat, quat_from_euler
from core.interfaces import ActuatorCommand
from plant.dynamics import initial_state


def case(extra=()):
    bundle = build_bundle(
        [
            "environment/atmosphere=vacuum",
            "plant/thrust=vacuum",
            "plant/actuator=ideal",
            *extra,
        ]
    )
    state = initial_state(
        dict(
            r_ned=[0, 0, -60000],
            v_ned=[0, 0, 0],
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            m=40000.0,
            seed=0,
        ),
        bundle.params,
    )
    command = ActuatorCommand(
        throttle=np.zeros(1),
        gimbal=np.zeros((1, 2)),
        fin=np.zeros(4),
        rcs=np.zeros(3),
    )
    return bundle, state, command


@pytest.mark.parametrize("altitude", [0.0, 10000.0, 60000.0, 84900.0])
def test_inverse_square_gravity_and_body_force(altitude):
    b, s, c = case()
    s = s.replace(r_ned=np.array([0, 0, -altitude]), q_bn=quat_from_euler(0.3, -0.7, 1.2))
    d, a = b.native.rhs(s, c, np.zeros(3))
    g = float(b.params.gravity.mu) / (float(b.params.gravity.radius) + altitude) ** 2
    assert np.allclose(d[2], [0, 0, g], rtol=1e-13)
    assert np.allclose(dcm_from_quat(s.q_bn) @ a["F_gravity_body"], [0, 0, float(s.m) * g])
    assert np.isclose(np.linalg.norm(a["F_gravity_body"]), float(s.m) * g)


def test_inverse_square_freefall_conserves_specific_energy():
    b, s, c = case()
    session = b.native.create_session(s.replace(v_ned=np.array([0.0, 0.0, 300.0])))
    mu, radius = float(b.params.gravity.mu), float(b.params.gravity.radius)
    initial = 0.5 * 300**2 - mu / (radius + 60000)
    energies = []
    for _ in range(5):
        state, _ = session.advance(c, 1000)
        energies.append(
            0.5 * np.dot(state["v_ned"], state["v_ned"]) - mu / (radius - state["r_ned"][2])
        )
    assert np.allclose(energies, initial, rtol=0, atol=2e-6)


def test_dual_tank_cylinders_match_mass_integrals():
    b, _, _ = case()
    p = b.params.mass
    dry, gas, prop = float(p.m_dry), 80.0, 7000.0
    mass = dry + gas + prop
    result = b.native.mass_properties_diagnostics(mass, gas)
    ratio = float(p.tank_of_ratio)
    liquids = np.array([ratio, 1.0]) * prop / (1 + ratio)
    heights = liquids / (np.pi * np.asarray(p.tank_radius) ** 2 * np.asarray(p.tank_density))
    centers = np.asarray(p.tank_bottom) + heights / 2
    masses = np.r_[dry, liquids, gas]
    xs = np.r_[float(p.x_cg_dry), centers, float(p.rcs_gas_x)]
    cg = masses @ xs / mass
    # Integrate x^2 numerically over each uniformly filled cylinder, not endpoint interpolation.
    inertia = np.array(p.I_dry, copy=True)
    inertia[1:] += dry * (float(p.x_cg_dry) - cg) ** 2
    nodes, weights = np.polynomial.legendre.leggauss(4)
    for m, h, x, r in zip(liquids, heights, centers, p.tank_radius, strict=True):
        x_samples = x + nodes * h / 2
        inertia[0] += m * r * r / 2
        inertia[1:] += m * (r * r / 4 + np.sum(weights * (x_samples - cg) ** 2) / 2)
    inertia[0] += gas * float(p.rcs_gas_radius) ** 2 / 2
    inertia[1:] += gas * (
        (float(p.rcs_gas_radius) ** 2) / 4
        + float(p.rcs_gas_length) ** 2 / 12
        + (float(p.rcs_gas_x) - cg) ** 2
    )
    assert np.isclose(liquids[0] / liquids[1], 2.56)
    assert np.allclose(result["liquid_masses"], liquids)
    assert np.allclose(result["liquid_heights"], heights)
    assert np.isclose(result["x_cg"], cg)
    assert np.allclose(result["inertia"], inertia, rtol=1e-12)
    empty = b.native.mass_properties_diagnostics(dry, 0)
    assert np.allclose(empty["inertia"], p.I_dry)
    assert np.isclose(empty["x_cg"], p.x_cg_dry)


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("sense", [-1, 1])
def test_rcs_couples_have_zero_force_and_correct_torque_and_flow(axis, sense):
    b, s, c = case()
    c = c.replace(rcs=sense * np.eye(3)[axis])
    d, a = b.native.rhs(s, c, np.zeros(3))
    thrust, arm = float(b.params.actuator.rcs_thrust), float(b.params.actuator.rcs_arm)
    expected_moment = sense * np.eye(3)[axis] * 2 * thrust * arm
    flow = 2 * thrust / (float(b.params.actuator.rcs_isp) * 9.80665)
    assert np.allclose(a["F_rcs_body"], 0, atol=1e-12)
    assert np.allclose(a["M_rcs_body"], expected_moment, atol=1e-9)
    assert np.count_nonzero(a["rcs_thruster_on"]) == 2
    assert np.isclose(-d[5], flow)
    assert np.isclose(d[11], d[5])  # total and RCS mass decrease by the same amount


def test_four_nozzle_layout_only_has_roll_authority():
    b, s, c = case(["plant/rcs=roll_four"])
    _, a = b.native.rhs(s, c.replace(rcs=np.array([0, 1, 1.0])), np.zeros(3))
    assert len(a["rcs_thruster_on"]) == 4
    assert np.allclose(a["M_rcs_body"], 0)
    assert a["rcs_mdot"] == 0


def test_failed_member_of_pair_generates_translation():
    b, s, c = case(["plant/rcs=roll_four", "plant.rcs.thrust_scale=[1,0,1,1]"])
    _, a = b.native.rhs(s, c.replace(rcs=np.array([1.0, 0, 0])), np.zeros(3))
    thrust = float(b.params.actuator.rcs_thrust)
    assert np.allclose(a["F_rcs_body"], [0, 0, thrust])
    assert np.isclose(a["M_rcs_body"][0], thrust * float(b.params.actuator.rcs_arm))
    assert np.isclose(a["rcs_mdot"], thrust / (float(b.params.actuator.rcs_isp) * 9.80665))


def test_pwm_is_on_off_and_conserves_impulse_and_propellant():
    b, s, c = case(["environment/gravity=uniform", "environment.atmosphere.g0=0"])
    c = c.replace(rcs=np.array([0.25, 0, 0]))
    n = int(b.params.rcs.pulse_ticks)
    flags = []
    for i in range(n):
        _, a = b.native.rhs(s.replace(t=np.array(i * float(b.params.plant_dt))), c, np.zeros(3))
        flags.append(a["rcs_thruster_on"][0])
    assert set(flags) == {0, 1}
    assert sum(flags) == n / 4
    session = b.native.create_session(s)
    end, _ = session.advance(c, n)
    full = session.snapshot(full=True)
    duration = n * float(b.params.plant_dt)
    flow = 2 * float(b.params.actuator.rcs_thrust) / (float(b.params.actuator.rcs_isp) * 9.80665)
    used = duration * 0.25 * flow
    assert np.isclose(float(s.m) - end["m"], used, atol=1e-8)
    assert np.isclose(float(s.rcs_fuel) - full["rcs_fuel"], used, atol=1e-10)
    assert np.allclose(end["v_ned"], 0, atol=1e-12)
    I = b.native.mass_properties_diagnostics(end["m"], full["rcs_fuel"])["inertia"]
    angular_impulse = (
        2 * float(b.params.actuator.rcs_thrust) * float(b.params.actuator.rcs_arm) * duration * 0.25
    )
    assert np.isclose(I[0] * end["w_body"][0], angular_impulse, rtol=1e-8)


def test_rcs_empty_tank_cannot_consume_main_propellant():
    b, s, c = case()
    s = s.replace(rcs_fuel=np.array(0.001))
    session = b.native.create_session(s)
    end, _ = session.advance(c.replace(rcs=np.array([1.0, 0, 0])), 100)
    full = session.snapshot(full=True)
    assert np.isclose(float(s.m) - end["m"], 0.001, atol=1e-8)
    assert full["rcs_fuel"] == 0
    _, a = session.observe()
    assert a["rcs_mdot"] == 0
    assert np.allclose(a["M_rcs_body"], 0)


def test_main_cutoff_preserves_rcs_reserve_and_continues_ballistically():
    b, s, c = case(["environment/gravity=uniform", "environment.atmosphere.g0=0"])
    s = s.replace(m=np.array(float(b.params.mass.m_dry) + float(s.rcs_fuel) + 1))
    session = b.native.create_session(s)
    end, a = session.advance(c.replace(throttle=np.ones(1)), 100)
    assert np.isclose(end["m"], float(b.params.mass.m_dry) + float(s.rcs_fuel), atol=1e-8)
    assert a["thrust_mag"] == 0
    assert not end["done"]
    later, _ = session.advance(c.replace(throttle=np.ones(1)), 100)
    assert np.allclose(end["v_ned"], later["v_ned"])
    assert np.isclose(later["t"], 0.2)


def test_fin_force_is_dissipative_and_body_moment_is_lever_cross_force():
    b = build_bundle(["plant.aero.fins.n_fins=1"])
    mass = 40000.0
    cg = b.native.mass_properties_diagnostics(mass)["x_cg"]
    arm = np.array(b.params.aero.fin_arm[0]) - [cg, 0, 0]
    velocity = np.array([-200.0, 30.0, -20.0])
    for delta in [-0.1, 0, 0.1]:
        result = b.native.fin_wrench_diagnostics(velocity, np.zeros(3), [delta], mass, 0.0)
        assert np.dot(result["force"], velocity) <= 0
        assert np.allclose(result["moment"], np.cross(arm, result["force"]))


@pytest.mark.parametrize(
    "override",
    [
        "environment.gravity.radius=0",
        "environment.gravity.mu=-1",
        "plant.vehicle.tanks.of_ratio=0",
        "plant.vehicle.tanks.length=[0.01,0.01]",
        "plant.rcs.pulse_period=0.0015",
        "plant.rcs.thrust_scale=-1",
    ],
)
def test_invalid_physical_parameters_are_rejected(override):
    with pytest.raises(ValueError):
        build_bundle([override])
