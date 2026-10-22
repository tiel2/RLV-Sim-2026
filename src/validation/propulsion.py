"""Independent propulsion component references and native-Plant comparisons.

The reference functions take configuration constants only. Fixed-state tests isolate
engine forces/moments; unloaded TVC sessions isolate servo dynamics from vehicle motion.
Model agreement and enforced command/rate constraints have separate verdicts.
The angle parameter bounds the command; actual-angle overshoot is observational.
"""

import numpy as np
from omegaconf import OmegaConf
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from core.interfaces import ActuatorCommand
from plant.dynamics import build_plant, ic_from_scenario, initial_state
from validation.references import G0, mass_properties

STATIC_LIMITS = {
    "force_n": 1e-7,
    "moment_nm": 1e-6,
    "flow_kg_s": 1e-10,
    "acceleration_mps2": 1e-9,
    "angular_acceleration_rad_s2": 1e-10,
}
SERVO_LIMITS = {"angle_rad": 1e-7, "rate_rad_s": 2e-6}
BOUND_ATOL = 1e-8


def clone(cfg):
    return OmegaConf.create(OmegaConf.to_container(cfg, resolve=True))


def engine_reference(cfg, throttle, angles, mass, altitude):
    """Sum rotated engine vectors and their CG moments without Plant geometry helpers.

    Angles are effective gimbal angles, not commands. This anchor intentionally excludes
    faults and below-minimum nonzero throttle. Mass flow uses vacuum Isp, including in
    the back-pressure variant, matching the declared propulsion model.
    """
    vehicle = cfg.plant.vehicle
    e = vehicle.engines
    n = int(e.n_engines)
    if e.layout != "octaweb" or n != 9 or list(e.gimbal_mask) != [1] + [0] * 8:
        raise ValueError("propulsion reference requires center-gimballed nine-engine octaweb")
    if (
        cfg.fault.engine_out_idx is not None
        or not np.allclose(cfg.fault.thrust_scale, 1)
        or not np.allclose(cfg.fault.tvc_bias, 0)
        or not np.allclose(vehicle.mass.cg_lateral, 0)
    ):
        raise ValueError("propulsion reference requires healthy engines and axial CG")
    u = np.asarray(throttle, dtype=float)
    floor = np.broadcast_to(np.asarray(e.throttle_min), (n,))
    if u.shape != (n,) or np.any((u != 0) & ((u < floor) | (u > 1))):
        raise ValueError("reference uses zero or above-minimum throttle only")
    gas = float(cfg.scenario.ic.rcs_fuel)
    cg, inertia = mass_properties(vehicle, mass, gas)
    # Construct ring locations independently using the roots of unity.
    circle = np.exp(2j * np.pi * np.arange(8) / 8) * float(e.ring_radius)
    positions = np.zeros((n, 3))
    positions[:, 0] = e.x_eng
    positions[1:, 1], positions[1:, 2] = circle.real, circle.imag
    directions = np.tile([1.0, 0.0, 0.0], (n, 1))
    pitch, yaw = np.asarray(angles).reshape(2)
    directions[0] = Rotation.from_rotvec([0, 0, yaw]).apply(
        Rotation.from_rotvec([0, pitch, 0]).apply([1.0, 0.0, 0.0])
    )
    atmosphere = cfg.environment.atmosphere
    if cfg.plant.thrust.model == "vacuum" or atmosphere.model == "vacuum":
        pressure = 0.0
    elif cfg.plant.thrust.model == "backpressure" and atmosphere.model == "exp":
        pressure = float(atmosphere.p_sea) * np.exp(-max(altitude, 0) / atmosphere.scale_height)
    else:
        raise ValueError("reference supports vacuum or exponential back-pressure only")
    vacuum = u * np.broadcast_to(np.asarray(e.thrust_vac), (n,))
    magnitudes = np.maximum(vacuum - pressure * np.asarray(e.nozzle_exit_area), 0)
    forces = magnitudes[:, None] * directions
    arms = positions - np.array([cg, 0, 0])
    return {
        "force": np.sum(forces, axis=0),
        "moment": np.sum(np.cross(arms, forces), axis=0),
        "flow": float(np.sum(vacuum / (np.asarray(e.isp) * G0))),
        "cg": cg,
        "inertia": inertia,
        "pressure_pa": float(pressure),
    }


def servo_reference(actuator, model, schedule, times, *, rtol=1e-12):
    """Independent analytic linear response or nonlinear DOP853, split at commands."""
    omega, zeta = float(actuator.tvc_omega), float(actuator.tvc_zeta)
    rate_limit, limit = float(actuator.tvc_rate_max_rad_s), float(actuator.tvc_deflection_max_rad)
    if model not in ("simplified", "realistic") or not 0 < zeta < 1:
        raise ValueError("reference requires simplified/realistic underdamped servo")
    times = np.asarray(times)
    output = np.zeros((len(times), 4))
    initial = np.zeros(4)  # pitch, yaw, pitch rate, yaw rate
    for i, (start, command) in enumerate(schedule):
        end = schedule[i + 1][0] if i + 1 < len(schedule) else float(times[-1])
        target = np.clip(command, -limit, limit)
        mask = (times >= start) & (times <= end)
        local = times[mask] - start
        if model == "simplified":
            wd = omega * np.sqrt(1 - zeta * zeta)
            a = initial[:2] - target
            b = (initial[2:] + zeta * omega * a) / wd

            def analytic(t, target=target, a=a, b=b, wd=wd):
                cos, sin = np.cos(wd * t)[:, None], np.sin(wd * t)[:, None]
                exp = np.exp(-zeta * omega * t)[:, None]
                angle = target + exp * (cos * a + sin * b)
                rate = exp * (
                    cos * (-zeta * omega * a + wd * b) + sin * (-zeta * omega * b - wd * a)
                )
                return np.column_stack([angle, rate])

            output[mask] = analytic(local)
            initial = analytic(np.array([end - start]))[0]
        else:

            def rhs(_, y, target=target):
                velocity_target = rate_limit * np.tanh(
                    omega * (target - y[:2]) / (2 * zeta * rate_limit)
                )
                return np.r_[y[2:], 2 * zeta * omega * (velocity_target - y[2:])]

            sol = solve_ivp(
                rhs,
                (0, end - start),
                initial,
                method="DOP853",
                rtol=rtol,
                atol=rtol * 0.01,
                dense_output=True,
            )
            if not sol.success:
                raise ValueError(f"servo reference failed: {sol.message}")
            output[mask] = sol.sol(local).T
            initial = sol.y[:, -1]
    return output


def static_cases():
    """Small deterministic set covering axes, layout symmetry, and CG variation."""
    cases = []

    def add(group, name, engines, pitch=0.0, yaw=0.0, throttle=0.7):
        u = np.zeros(9)
        u[engines] = throttle
        cases.append(
            {"group": group, "name": name, "throttle": u.tolist(), "gimbal": [[pitch, yaw]]}
        )

    for u in [0.0, 0.4, 0.7, 1.0]:
        add("axial", f"center_u{u}", [0], throttle=u)
    for axis in ("pitch", "yaw"):
        for angle in [-0.1, -0.05, 0.0, 0.05, 0.1]:
            add("tvc_axes", f"{axis}_{angle}", [0], **{axis: angle})
    for p, y in [(-0.1, -0.1), (-0.1, 0.1), (0.1, -0.1), (0.1, 0.1)]:
        add("tvc_combined", f"p{p}_y{y}", [0], p, y)
    for i in range(1, 9):
        add("ring_single", f"engine_{i}", [i])
    for i in range(1, 5):
        add("symmetry", f"opposite_{i}_{i + 4}", [i, i + 4])
    add("symmetry", "all_engines", list(range(9)))
    add("mixed", "center_ring", [0, 1, 2], 0.07, -0.04)
    add("fixed_ring", "ring_ignores_center_gimbal", [1], 0.1, -0.1)
    for axis in ("pitch", "yaw"):
        for angle in [-0.3, 0.3]:
            add("ideal_command_clip", f"{axis}_{angle}", [0], **{axis: angle})
    return cases


def check_values(errors, limits):
    """Explicit absolute tolerances; no hidden NumPy default relative tolerance."""
    return {
        key: {
            "error": float(value),
            "tolerance": limits[key],
            "passed": bool(np.isfinite(value) and value <= limits[key]),
        }
        for key, value in errors.items()
    }


def check_static(cfg):
    c = clone(cfg)
    c.plant.actuator.model = "ideal"
    c.scenario.ic.throttle0 = 0.0
    c.scenario.validation.kind = "none"
    variants = {"vacuum": c}
    bp = clone(c)
    bp.environment.atmosphere.model = "exp"
    bp.environment.atmosphere.p_sea = 101325.0
    bp.environment.atmosphere.rho0 = 1.225
    bp.environment.atmosphere.scale_height = 8500.0
    bp.plant.thrust.model = "backpressure"
    variants["backpressure"] = bp
    rows = []
    for variant, local in variants.items():
        b = build_plant(local)
        for mass in [25700.0, 40000.0, 50000.0]:
            for case in static_cases():
                altitudes = [0.0, 10000.0, 40000.0] if variant == "backpressure" else [10000.0]
                # Pressure at several altitudes is needed only for axial/combined cases.
                if variant == "backpressure" and case["group"] not in ("axial", "tvc_combined"):
                    continue
                for altitude in altitudes:
                    u, command_angle = np.array(case["throttle"]), np.array(case["gimbal"])
                    angle = np.clip(
                        command_angle,
                        -local.plant.vehicle.actuator.tvc_deflection_max_rad,
                        local.plant.vehicle.actuator.tvc_deflection_max_rad,
                    )
                    ref = engine_reference(local, u, angle, mass, altitude)
                    ic = ic_from_scenario(local.scenario, local.seed)
                    ic.update(
                        m=mass,
                        r_ned=[0, 0, -altitude],
                        v_ned=[0, 0, 0],
                        w_body=[0, 0, 0],
                        throttle0=0.0,
                    )
                    state = initial_state(ic, b.params)
                    cmd = ActuatorCommand(u, command_angle, np.zeros(4), np.zeros(3))
                    deriv, aux = b.native.rhs(state, cmd, np.zeros(3))
                    R = Rotation.from_euler(
                        "ZYX",
                        [
                            local.scenario.ic.yaw_deg,
                            local.scenario.ic.pitch_deg,
                            local.scenario.ic.roll_deg,
                        ],
                        degrees=True,
                    )
                    wrench = b.native.engine_wrench_diagnostics(u, angle, mass, altitude)
                    errors = {
                        "force_n": max(
                            np.linalg.norm(aux["T_body"] - ref["force"]),
                            np.linalg.norm(wrench["force"] - ref["force"]),
                        ),
                        "moment_nm": max(
                            np.linalg.norm(aux["M_body_total"] - ref["moment"]),
                            np.linalg.norm(wrench["moment"] - ref["moment"]),
                        ),
                        "flow_kg_s": max(
                            abs(-float(deriv[5]) - ref["flow"]), abs(wrench["mdot"] - ref["flow"])
                        ),
                        "acceleration_mps2": np.linalg.norm(
                            deriv[2] - R.apply(ref["force"]) / mass
                        ),
                        "angular_acceleration_rad_s2": np.linalg.norm(
                            deriv[4] - np.linalg.solve(ref["inertia"], ref["moment"])
                        ),
                    }
                    checks = check_values(errors, STATIC_LIMITS)
                    rows.append(
                        {
                            **case,
                            "variant": variant,
                            "mass_kg": mass,
                            "altitude_m": altitude,
                            "cg_x_m": ref["cg"],
                            "pressure_pa": ref["pressure_pa"],
                            "effective_gimbal": angle.tolist(),
                            "reference_force_n": ref["force"].tolist(),
                            "actual_force_n": np.asarray(aux["T_body"]).tolist(),
                            "reference_moment_nm": ref["moment"].tolist(),
                            "actual_moment_nm": np.asarray(aux["M_body_total"]).tolist(),
                            "reference_flow_kg_s": ref["flow"],
                            "actual_flow_kg_s": -float(deriv[5]),
                            "checks": checks,
                            "passed": all(x["passed"] for x in checks.values()),
                        }
                    )
    return rows, variants


SERVO_CASES = [
    ("linear_pulse", "simplified", [(0.0, [0.01, -0.007]), (0.25, [0.0, 0.0])], 0.8),
    ("realistic_pulse", "realistic", [(0.0, [0.01, -0.007]), (0.25, [0.0, 0.0])], 0.8),
    ("realistic_limit_reversal", "realistic", [(0.0, [0.3, -0.3]), (0.75, [-0.3, 0.3])], 2.0),
]


def check_servo(cfg, name, model, schedule, duration, dt):
    c = clone(cfg)
    c.plant.actuator.model = model
    c.plant_dt = dt
    c.scenario.name = name
    c.scenario.ic.m = 40000.0
    c.scenario.ic.v_ned = [0.0, 0.0, 0.0]
    c.scenario.ic.w_body = [0.0, 0.0, 0.0]
    c.scenario.ic.throttle0 = 0.0
    c.scenario.horizon.t_max = duration
    c.scenario.horizon.input_dt = dt
    c.scenario.horizon.log_dt = dt
    c.scenario.validation.kind = "none"
    c.scenario.inputs = [
        {"t": time, "throttle": 0.0, "gimbal": [angle], "fin": 0.0, "rcs": 0.0}
        for time, angle in schedule
    ]
    b = build_plant(c)
    ic = ic_from_scenario(c.scenario, c.seed)
    ic.update(m=40000.0, v_ned=[0, 0, 0], w_body=[0, 0, 0], throttle0=0.0)
    state0 = initial_state(ic, b.params)
    session = b.native.create_session(state0)
    t = np.arange(round(duration / dt) + 1) * dt
    if any(abs(time / dt - round(time / dt)) > 1e-8 for time, _ in schedule):
        raise ValueError("servo input changes must lie on each integration grid")
    actual = np.zeros((len(t), 4))
    command_clip_error = 0.0
    limit = float(c.plant.vehicle.actuator.tvc_deflection_max_rad)
    # Check native command clipping, rather than merely clipping our own reference.
    for _, requested in schedule:
        cmd = ActuatorCommand(np.zeros(9), np.array([requested]), np.zeros(4), np.zeros(3))
        clipped = cmd.replace(gimbal=np.clip(cmd.gimbal, -limit, limit))
        raw_derivative, _ = b.native.rhs(state0, cmd, np.zeros(3))
        clipped_derivative, _ = b.native.rhs(state0, clipped, np.zeros(3))
        command_clip_error = max(
            command_clip_error,
            float(
                np.max(np.abs(np.asarray(raw_derivative[7]) - np.asarray(clipped_derivative[7])))
            ),
        )
    releases = {round(time / dt): angle for time, angle in schedule}
    angle = [0.0, 0.0]
    for k in range(len(t) - 1):
        if k in releases:
            angle = releases[k]
        cmd = ActuatorCommand(np.zeros(9), np.array([angle]), np.zeros(4), np.zeros(3))
        session.advance(cmd, 1)
        s = session.snapshot(True)
        if abs(float(s["t"]) - t[k + 1]) > 1e-10:
            raise ValueError("native and reference time grids disagree")
        actual[k + 1] = np.r_[np.asarray(s["tvc"])[0], np.asarray(s["tvc_rate"])[0]]
    a = c.plant.vehicle.actuator
    ref = servo_reference(a, model, schedule, t)
    errors = {
        "angle_rad": float(np.max(np.abs(actual[:, :2] - ref[:, :2]))),
        "rate_rad_s": float(np.max(np.abs(actual[:, 2:] - ref[:, 2:]))),
    }
    checks = check_values(errors, SERVO_LIMITS)
    stability = None
    if model == "realistic":
        coarse = servo_reference(a, model, schedule, t, rtol=1e-10)
        stability = check_values(
            {
                "angle_rad": float(np.max(np.abs(ref[:, :2] - coarse[:, :2]))),
                "rate_rad_s": float(np.max(np.abs(ref[:, 2:] - coarse[:, 2:]))),
            },
            {k: v * 0.01 for k, v in SERVO_LIMITS.items()},
        )
    peak_angle, peak_rate = (
        float(np.max(np.abs(actual[:, :2]))),
        float(np.max(np.abs(actual[:, 2:]))),
    )
    constraints = check_values(
        {"command_clip_acceleration_rad_s2": command_clip_error},
        {"command_clip_acceleration_rad_s2": 1e-10},
    )
    if model == "realistic":
        constraints.update(
            check_values(
                {"rate_exceedance_rad_s": max(peak_rate - a.tvc_rate_max_rad_s, 0)},
                {"rate_exceedance_rad_s": BOUND_ATOL},
            )
        )
    agreement = all(x["passed"] for x in checks.values()) and (
        stability is None or all(x["passed"] for x in stability.values())
    )
    report = {
        "name": name,
        "model": model,
        "plant_dt_s": dt,
        "duration_s": duration,
        "schedule": schedule,
        "checks": checks,
        "reference_stability": stability,
        "model_agreement": "passed" if agreement else "failed",
        "enforced_constraints": constraints,
        "peak_angle_rad": peak_angle,
        "peak_rate_rad_s": peak_rate,
        "angle_observation": {
            "command_limit_rad": limit,
            "overshoot_rad": max(peak_angle - limit, 0),
            "overshoot_deg": float(np.rad2deg(max(peak_angle - limit, 0))),
            "role": "model characteristic; does not gate model verification",
        },
        "mechanical_stop_status": "not_assessed",
        "constraint_status": "passed"
        if all(x["passed"] for x in constraints.values())
        else "failed",
    }
    return report, {"t": t, "reference": ref, "actual": actual}, c


def check_propulsion(cfg):
    if (
        cfg.environment.gravity.model != "uniform"
        or cfg.environment.atmosphere.g0 != 0
        or cfg.environment.atmosphere.model != "vacuum"
        or cfg.plant.thrust.model != "vacuum"
        or cfg.environment.earth.model != "fixed"
        or cfg.environment.wind.model != "none"
        or cfg.plant.slosh.model != "none"
        or cfg.scenario.dispersion.enable
        or cfg.scenario.ic.rcs_fuel != cfg.plant.vehicle.actuator.rcs_fuel0
    ):
        raise ValueError("use scenario=vacuum_burn plant/thrust=vacuum with nominal RCS reserve")
    rows, configs = check_static(cfg)
    dynamics, series = [], {}
    for name, model, schedule, duration in SERVO_CASES:
        for dt in [0.002, 0.001, 0.0005]:
            report, data, local = check_servo(cfg, name, model, schedule, duration, dt)
            key = f"{name}_{dt:g}"
            dynamics.append(report)
            series[key] = data
            configs[key] = local
    static_pass = all(row["passed"] for row in rows)
    model_pass = static_pass and all(r["model_agreement"] == "passed" for r in dynamics)
    bounds_pass = all(r["constraint_status"] == "passed" for r in dynamics)
    report = {
        "status": "passed" if model_pass and bounds_pass else "failed",
        "scope": "fixed-state engine/TVC wrench and unloaded servo verification",
        "acceptance_policy": {
            "version": 2,
            "angle_limit": "per-axis command clipping; not an actual-angle hard stop",
            "actual_angle_overshoot": "recorded model transient; not a failure criterion",
            "enforced": ["native command clipping", "realistic rate bound"],
            "mechanical_stop_status": "not_assessed",
        },
        "static_cases": len(rows),
        "static_status": "passed" if static_pass else "failed",
        "static_max_errors": {k: max(r["checks"][k]["error"] for r in rows) for k in STATIC_LIMITS},
        "static_limits": STATIC_LIMITS,
        "model_agreement": "passed" if model_pass else "failed",
        "constraint_status": "passed" if bounds_pass else "failed",
        "dynamic_cases": dynamics,
    }
    return report, rows, series, configs
