"""Configuration regression tests for the experimental phase-specific LCvx targets."""
from gnc.guidance.convex.guidance import LcvxGuidance


def make_guidance(phase):
    nominal = dict(
        thrust_vac=981000.0, throttle_min=0.4, isp=311.0, g0=9.80665,
        gravity_ned=[0.0, 0.0, 9.80665], m_dry=25200.0,
        p_sea=101325.0, nozzle_exit_area=0.9, scale_height=8500.0,
        rho0=1.225, drag_cd_s=10.52,
        scale_pos=1000.0, scale_vel=100.0, scale_accel=40.0,
    )
    return LcvxGuidance(
        nominal=nominal, active_engines=[1, 1, 0, 0, 0, 1, 0, 0, 0] if phase == 2 else [1],
        phase=phase, N=20, target=(0, 0, 0), theta_max_deg=15,
        gamma_gs_deg=20, solver='CLARABEL', tf_search_iters=8,
        grid_fallback=True, tilt_max_deg=15, k_r=1, k_v=4,
        tf_floor=0.5, landing_cutoff_impact_mps=0, terminal_slack=False,
        terminal_pos_weight=100.0, terminal_vel_weight=100.0,
    )


def test_brake_is_a_handoff_not_a_touchdown():
    guidance = make_guidance(phase=2)
    assert guidance.target == (0.0, 0.0, -2000.0)
    assert guidance.terminal_vz_max == 180.0
    assert guidance.rho_max == 3 * 981000.0


def test_final_landing_keeps_ground_target():
    guidance = make_guidance(phase=3)
    assert guidance.target == (0.0, 0.0, 0.0)
    assert guidance.terminal_vz_max is None
