"""Logged-history screens for unfitted runs; not independent numerical verification."""

from __future__ import annotations

import numpy as np


def audit_history(p, actuators, cfg):
    """Check recorded samples and native-block actuator endpoints, not unlogged extrema."""
    a = cfg.plant.vehicle.actuator
    main, gas = p["main_propellant"], p["rcs_fuel"]
    vehicle = cfg.plant.vehicle
    dry = vehicle.mass.m_dry + vehicle.get("attached_component", {}).get("mass", 0)
    mass_error = float(np.max(np.abs(p["mass"] - dry - main - gas)))
    alpha = np.rad2deg(p["aero_alpha"])
    grid = cfg.plant.aero.grid
    outside = (
        (alpha > grid.alpha_max_deg + 1e-8)
        | (p["mach"] > grid.mach_max)
        | (p["mach"] < grid.mach_min)
    )
    checks = dict(
        finite_state=all(
            np.all(np.isfinite(p[k])) for k in ("t", "r_ned", "v_ned", "q_bn", "w_body", "mass")
        ),
        quaternion=float(np.max(abs(np.linalg.norm(p["q_bn"], axis=1) - 1))) <= 1e-12,
        mass_bookkeeping=mass_error <= 0.001,
        positive_mass=bool(np.all(p["mass"] > 0)),
        propellant_nonnegative=bool(np.all(main >= 0) and np.all(gas >= 0)),
        no_mass_creation=bool(np.max(np.diff(p["mass"]), initial=0) <= 1e-8),
        positive_inertia=bool(np.all(p["inertia_diag"] > 0)),
        body_lookup_domain=not np.any(outside),
        throttle_range=bool(
            np.all(
                (actuators["effective_throttle"] >= -1e-10)
                & (actuators["effective_throttle"] <= 1 + 1e-10)
            )
        ),
        tvc_deflection=bool(
            np.max(abs(actuators["effective_gimbal"]), initial=0) <= a.tvc_deflection_max_rad + 1e-8
        ),
        fin_deflection=bool(
            np.max(abs(actuators["effective_fin"]), initial=0) <= a.fin_deflection_max_rad + 1e-8
        ),
    )
    checks = {k: bool(v) for k, v in checks.items()}
    return dict(
        status="passed_logged_screens" if all(checks.values()) else "failed_logged_screens",
        scope="logged states and actuator endpoints; not independent or empirical validation",
        checks=checks,
        failed_checks=[k for k, v in checks.items() if not v],
        diagnostics=dict(
            mass_bookkeeping_max_error_kg=mass_error,
            maximum_alpha_deg=float(alpha.max()),
            maximum_mach=float(p["mach"].max()),
            body_domain_outside_samples=int(outside.sum()),
            first_body_domain_violation_s=float(p["t"][np.flatnonzero(outside)[0]])
            if np.any(outside)
            else None,
            minimum_main_propellant_kg=float(main.min()),
            initial_main_propellant_kg=float(main[0]),
            final_main_propellant_kg=float(main[-1]),
            main_consumed_kg=float(main[0] - main[-1]),
            initial_mass_kg=float(p["mass"][0]),
            final_mass_kg=float(p["mass"][-1]),
            maximum_dynamic_pressure_pa=float(p["aero_q_dyn"].max()),
        ),
        not_assessed=[
            "independent_equations",
            "step_refinement",
            "real_vehicle_accuracy",
            "unlogged_extrema",
        ],
        minimum_throttle_note="realistic throttle may cross below its floor during transients",
        actuator_model=str(cfg.plant.actuator.model),
    )


def braking_screen(
    height, down_speed, horizontal_speed, mass, max_thrust, gravity, cone_deg, fuel, isp
):
    """Constant initial-mass, no-drag capability scales; not a feasible trajectory proof."""
    acceleration = max_thrust / mass
    net_up = acceleration - gravity
    angle = np.deg2rad(cone_deg)
    lateral_accel = acceleration * np.sin(angle)
    return dict(
        scope="initial mass, no drag; omit slew, coupling and changing mass",
        initial_altitude_m=height,
        initial_down_speed_mps=down_speed,
        initial_horizontal_speed_mps=horizontal_speed,
        maximum_thrust_n_at_initial_pressure=max_thrust,
        initial_thrust_weight_ratio=acceleration / gravity,
        vertical_stopping_distance_m=down_speed**2 / (2 * net_up) if net_up > 0 else None,
        vertical_stopping_time_s=down_speed / net_up if net_up > 0 else None,
        lateral_stopping_time_at_cone_s=horizontal_speed / lateral_accel
        if lateral_accel > 0
        else None,
        lateral_stopping_distance_at_cone_m=horizontal_speed**2 / (2 * lateral_accel)
        if lateral_accel > 0
        else None,
        cone_deg=cone_deg,
        initial_fuel_kg=fuel,
        ideal_propellant_for_speed_change_kg=mass
        * (1 - np.exp(-np.hypot(down_speed, horizontal_speed) / (isp * 9.80665))),
        fuel_estimate_role="impulse only; excludes gravity/drag/turn losses, not a landing budget",
    )
