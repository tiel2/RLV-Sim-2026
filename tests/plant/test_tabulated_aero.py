"""Vehicle-independent coefficient and fin contracts using synthetic Falcon tables."""

from __future__ import annotations

import numpy as np
import pytest

from core.interfaces import Command, Phase
from gnc.allocation import allocate
from gnc.build import build_gnc, default_nominal
from gnc.navigation.identity import IdentityNav
from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.loop import run_closed_loop


def test_file_coefficients_and_reference_cg_moment(tabulated_aero_cfg):
    bundle = build_plant(tabulated_aero_cfg)
    alpha, mach, mass = np.deg2rad(20.0), 0.6, 40000.0
    sound = float(bundle.native.environment_diagnostics(0.0)["a_sound"])
    velocity = mach * sound * np.array([-np.cos(alpha), 0.0, np.sin(alpha)])
    result = bundle.native.aero_wrench_diagnostics(velocity, np.zeros(3), mass, 0.0)

    cone = 1.0 - np.cos(alpha)
    assert np.isclose(result["cd"], 1.2 + 0.2 * cone + 0.03 * mach)
    assert np.isclose(result["cl"], 2.0 * cone)
    # The file's C_m is about its reference CG; translate that moment to the live vehicle CG.
    force_cross = np.cross([1.0, 0.0, 0.0], result["force"])
    moment_ref = (
        -float(bundle.params.geom.d_ref)
        * (0.1 * cone)
        * float(result["q_dyn"])
        * float(bundle.params.geom.s_ref)
        * force_cross
        / np.linalg.norm(force_cross)
    )
    cg = float(bundle.native.mass_properties_diagnostics(mass)["x_cg"])
    expected = moment_ref + (-25.0 - cg) * force_cross
    assert np.allclose(result["moment"], expected, rtol=1e-12, atol=1e-8)


def test_file_moment_is_finite_at_zero_incidence(tabulated_aero_cfg):
    bundle = build_plant(tabulated_aero_cfg)
    result = bundle.native.aero_wrench_diagnostics([-200.0, 0.0, 0.0], np.zeros(3), 40000.0, 0.0)
    assert np.isfinite(result["x_cp"])
    assert np.allclose(result["moment"], 0.0, atol=1e-9)


def test_neutral_fin_increment_vanishes_at_static_incidence(tabulated_aero_cfg):
    bundle = build_plant(tabulated_aero_cfg)
    result = bundle.native.fin_wrench_diagnostics(
        [-200.0, 0.0, 60.0], np.zeros(3), np.zeros(4), 40000.0, 0.0
    )
    assert np.allclose(result["force"], 0.0, atol=1e-9)
    assert np.allclose(result["moment"], 0.0, atol=1e-9)


def test_fin_increment_matches_module4_lift_and_quadratic_drag(tabulated_aero_cfg):
    """At fixed incidence, ΔCL=CLα δ and ΔCD=CDα²(δ²-2δφ), slide 23."""
    cfg = tabulated_aero_cfg
    cfg.plant.aero.fins.n_fins = 1
    cfg.plant.aero.fins.fin_phase_deg = 0.0
    bundle = build_plant(cfg)
    flow = np.array([-200.0, 0.0, 50.0])
    speed = np.linalg.norm(flow)
    env = bundle.native.environment_diagnostics(0.0)
    mach = speed / env["a_sound"]
    area = float(bundle.params.aero.fin_area)
    q = 0.5 * env["rho"] * speed**2
    cla = np.interp(mach, cfg.plant.aero.fins.mach_grid, cfg.plant.aero.fins.cla_table)
    cd2 = np.interp(mach, cfg.plant.aero.fins.mach_grid, cfg.plant.aero.fins.cd_delta2_table)
    phi, delta = np.arctan2(50.0, 200.0), 0.1
    vhat = flow / speed
    lift = np.array([0.0, 0.0, 1.0]) - vhat[2] * vhat
    lift /= np.linalg.norm(lift)
    result = bundle.native.fin_wrench_diagnostics(flow, np.zeros(3), [delta], 40000.0, 0.0)
    assert np.isclose(np.dot(result["force"], lift), q * area * cla * delta)
    assert np.isclose(-np.dot(result["force"], vhat), q * area * cd2 * (delta**2 - 2 * delta * phi))


@pytest.mark.parametrize("axis", [1, 2])
def test_geometry_allocation_produces_requested_fin_moment(tabulated_aero_cfg, axis):
    cfg = tabulated_aero_cfg
    bundle = build_plant(cfg)
    nominal = default_nominal(bundle.params, cfg.gnc.allocation, cfg.environment.atmosphere.model)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params).replace(
        r_ned=np.array([0.0, 0.0, -3000.0]),
        v_ned=np.array([0.0, 0.0, 150.0]),
        m=bundle.params.mass.m_wet,
    )
    torque = np.zeros(3)
    torque[axis] = 2.0e4
    command = Command(
        phase=int(Phase.AERO_DESCENT),
        mode=1,
        thrust_vector=np.zeros(3),
        q_ref=state.q_bn,
        tau_body=torque,
        active_engines=np.zeros(bundle.meta["n_engines"]),
        w_ref=np.zeros(3),
    )
    act = allocate(command, state, nominal)
    assert np.linalg.norm(act.fin) > 1e-3
    assert not np.allclose(act.fin, act.fin[0])
    assert np.max(np.abs(act.fin)) <= float(bundle.params.actuator.fin_deflection_max_rad)
    actual = bundle.native.fin_wrench_diagnostics(
        [-150.0, 0.0, 0.0], np.zeros(3), act.fin, float(state.m), 3000.0
    )["moment"]
    assert actual[axis] > 0.0
    assert np.linalg.norm(np.delete(actual, axis)) < 1e-2 * actual[axis]


def test_file_aero_closed_loop_descends(tabulated_aero_cfg):
    cfg = tabulated_aero_cfg
    cfg.scenario.start_phase = "aero"
    cfg.scenario.ic.r_ned = [0.0, 0.0, -3000.0]
    cfg.scenario.ic.v_ned = [0.0, 0.0, 150.0]
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
    result = run_closed_loop(
        bundle,
        state,
        build_gnc(cfg, bundle.params),
        IdentityNav(),
        t_max=3.0,
        ctrl_dt=0.05,
        plant_dt=float(cfg.plant_dt),
    )
    assert np.all(np.isfinite(result.r_ned))
    assert np.all(np.isfinite(result.w_body))
    assert result.phase[0] == int(Phase.AERO_DESCENT)
    assert result.r_ned[-1, 2] > result.r_ned[0, 2]
