"""Regression: three-engine brake plan must not survive landing phase transition."""
from gnc.guidance.convex.guidance import LcvxGuidance


def make_guidance(phase):
    nominal = dict(
        thrust_vac=981000.0, throttle_min=0.4, isp=311.0, g0=9.80665,
        gravity_ned=[0.0, 0.0, 9.80665], m_dry=25200.0,
        p_sea=101325.0, nozzle_exit_area=0.9, scale_height=8500.0,
        rho0=1.225, drag_cd_s=10.52, scale_pos=1000.0,
        scale_vel=100.0, scale_accel=40.0,
    )
    return LcvxGuidance(
        nominal=nominal,
        active_engines=[1, 1, 0, 0, 0, 1, 0, 0, 0] if phase == 2 else [1],
        phase=phase, N=20, target=(0, 0, 0), theta_max_deg=15,
        gamma_gs_deg=20, solver="CLARABEL", tf_search_iters=8,
        grid_fallback=True, tilt_max_deg=15, k_r=1, k_v=4,
        tf_floor=0.5, landing_cutoff_impact_mps=0, terminal_slack=False,
        terminal_pos_weight=100.0, terminal_vel_weight=100.0,
    )


def test_brake_to_landing_resets_plan_and_keeps_logs():
    brake = make_guidance(2)
    landing = make_guidance(3)
    mem = brake.initial_memory()
    old_plan = {"tf": 11.0, "N": 20, "r": [], "v": []}
    mem.update(ignited=True, frozen=True, cutoff=True,
               plan=old_plan, t_ignite=50.0, t_ig0=50.0, tf0=11.0)
    mem["solve_log"].append({"t": 50.0, "kind": "ignite_search"})
    mem["outer_scheduler_key"] = "preserve"
    new = landing._memory(mem)
    assert new["_planner_phase_owner"] == 3
    assert new["phase_reset_count"] == 1
    assert new["ignited"] is False
    assert new["frozen"] is False
    assert new["cutoff"] is False
    assert new["plan"] is None
    assert new["t_ignite"] is None
    assert new["tf0"] is None
    assert len(new["solve_log"]) == 1
    assert new["outer_scheduler_key"] == "preserve"
    assert mem["plan"] is old_plan  # input not mutated


def test_no_reset_within_same_phase():
    brake = make_guidance(2)
    mem = brake.initial_memory()
    mem.update(ignited=True, frozen=True, cutoff=True,
               plan={"tf": 11.0}, t_ignite=50.0)
    same = brake._memory(mem)
    assert same["ignited"] is True
    assert same["frozen"] is True
    assert same["cutoff"] is True
    assert same["plan"] is mem["plan"]
    assert same["phase_reset_count"] == 0


def test_final_phase_has_ground_target_and_single_engine():
    landing = make_guidance(3)
    assert landing.target == (0.0, 0.0, 0.0)
    assert landing.n_active == 1
