"""Sequential component and coupled checks with explicit absolute error budgets."""

import numpy as np
from scipy.spatial.transform import Rotation

from core.interfaces import ActuatorCommand
from plant.dynamics import build_plant, ic_from_scenario, initial_state
from validation import vehicle_references as ref
from validation.propulsion import check_values, clone
from validation.references import attitude_error, mass_properties

WRENCH_LIMITS = dict(
    force_n=1e-6, moment_nm=1e-5, acceleration_mps2=1e-9, angular_acceleration_rad_s2=1e-10
)


def base_config(cfg):
    c = clone(cfg)
    c.plant.actuator.model = "ideal"
    c.plant.slosh.model = "none"
    c.environment.wind.model = "none"
    c.environment.earth.model = "fixed"
    # The base scenario uses vacuum (rho0=p_sea=0); restore standard constants
    # before any experiment switches the model name to an air atmosphere.
    c.environment.atmosphere.rho0 = 1.225
    c.environment.atmosphere.p_sea = 101325.0
    c.scenario.ic.throttle0 = 0.0
    c.scenario.inputs = [
        dict(t=0.0, throttle=0.0, gimbal=[[0.0, 0.0]], fin=[0.0] * 4, rcs=[0.0] * 3)
    ]
    c.scenario.validation.kind = "none"
    c.scenario.ic.roll_deg, c.scenario.ic.pitch_deg, c.scenario.ic.yaw_deg = 18.0, 63.0, -27.0
    return c


def state_for(cfg, bundle, **overrides):
    ic = ic_from_scenario(cfg.scenario, cfg.seed)
    ic.update(overrides)
    return initial_state(ic, bundle.params)


def command(cfg, throttle=0.0, gimbal=None, fin=None, rcs=None):
    return ActuatorCommand(
        np.broadcast_to(
            np.asarray(throttle, dtype=float), (cfg.plant.vehicle.engines.n_engines,)
        ).copy(),
        np.zeros((1, 2)) if gimbal is None else np.array(gimbal, dtype=float),
        np.zeros(cfg.plant.aero.fins.n_fins) if fin is None else np.array(fin, dtype=float),
        np.zeros(3) if rcs is None else np.array(rcs, dtype=float),
    )


def row(name, errors, limits, **details):
    checks = check_values(errors, limits)
    return dict(
        name=name, checks=checks, passed=all(c["passed"] for c in checks.values()), **details
    )


def finish(stage, name, rows, configs, series=None):
    return (
        dict(
            stage=stage,
            name=name,
            status="passed" if all(r["passed"] for r in rows) else "failed",
            cases=len(rows),
            results=rows,
        ),
        configs,
        series or {},
    )


def check_fins(cfg):
    c = base_config(cfg)
    c.environment.atmosphere.model = "exp"
    c.environment.atmosphere.rho0 = 1.225
    c.plant.thrust.model = "backpressure"
    b = build_plant(c)
    R = Rotation.from_euler(
        "ZYX",
        [c.scenario.ic.yaw_deg, c.scenario.ic.pitch_deg, c.scenario.ic.roll_deg],
        degrees=True,
    )
    wind = np.array([12.0, -7.0, 3.0])
    rows = []
    inputs = [
        np.zeros(4),
        np.full(4, 0.1),
        np.full(4, -0.1),
        np.array([0.12, 0, -0.12, 0]),
        np.array([0, -0.12, 0, 0.12]),
        np.array([0.35, -0.35, 0.35, -0.35]),
    ]
    for altitude in [0.0, 10000.0, 40000.0]:
        sound = ref.environment(c, altitude)["a_sound"]
        flows = [
            (np.zeros(3), np.zeros(3)),
            (np.array([-200.0, 0, 0]), np.zeros(3)),
            (np.array([-sound, 0, 0]), np.zeros(3)),
            (np.array([-2 * sound, 30, -20]), np.array([0.12, -0.03, 0.04])),
            (np.array([-200.0, 30, -20]), np.array([0.0, 0.05, -0.04])),
            (np.array([-200.0, 0, 0]), np.array([0.2, 0, 0])),
        ]
        for mass in [25700.0, 40000.0, 50000.0]:
            cg, inertia = mass_properties(c.plant.vehicle, mass, c.scenario.ic.rcs_fuel)
            for j, (velocity, rates) in enumerate(flows):
                state = state_for(
                    c,
                    b,
                    m=mass,
                    r_ned=[0, 0, -altitude],
                    v_ned=R.apply(velocity) + wind,
                    w_body=rates,
                )
                for k, angles in enumerate(inputs):
                    cmd = command(c, fin=angles)
                    expected = ref.fins(c, velocity, rates, angles, cg, altitude)
                    diagnostic = b.native.fin_wrench_diagnostics(
                        velocity, rates, angles, mass, altitude
                    )
                    b.native.set_fin_deployment(0)
                    off_d, off = b.native.rhs(state, cmd, wind)
                    b.native.set_fin_deployment(1)
                    on_d, on = b.native.rhs(state, cmd, wind)
                    errors = dict(
                        force_n=max(
                            np.linalg.norm(diagnostic["force"] - expected["force"]),
                            np.linalg.norm(on["F_fin_body"] - expected["force"]),
                            np.linalg.norm(off["F_fin_body"]),
                        ),
                        moment_nm=max(
                            np.linalg.norm(diagnostic["moment"] - expected["moment"]),
                            np.linalg.norm(on["M_fin_body"] - expected["moment"]),
                            np.linalg.norm(off["M_fin_body"]),
                            np.linalg.norm(
                                on["M_body_total"] - off["M_body_total"] - expected["moment"]
                            ),
                        ),
                        acceleration_mps2=np.linalg.norm(
                            on_d[2] - off_d[2] - R.apply(expected["force"]) / mass
                        ),
                        angular_acceleration_rad_s2=np.linalg.norm(
                            on_d[4] - off_d[4] - np.linalg.solve(inertia, expected["moment"])
                        ),
                    )
                    rows.append(
                        row(
                            f"h{altitude:g}_m{mass:g}_flow{j}_fin{k}",
                            errors,
                            WRENCH_LIMITS,
                            altitude_m=altitude,
                            mass_kg=mass,
                            velocity_body=velocity,
                            rates_body=rates,
                            angles_rad=angles,
                            reference_force_n=expected["force"],
                            actual_force_n=on["F_fin_body"],
                            reference_moment_nm=expected["moment"],
                            actual_moment_nm=on["M_fin_body"],
                        )
                    )
    # Hand-computable structural invariants, separate from component-model duplication.
    cg, _ = mass_properties(c.plant.vehicle, 40000, c.scenario.ic.rcs_fuel)
    neutral = ref.fins(c, [-200, 0, 0], [0, 0, 0], np.zeros(4), cg, 0)
    positive = ref.fins(c, [-200, 0, 0], [0, 0, 0], np.full(4, 0.1), cg, 0)
    negative = ref.fins(c, [-200, 0, 0], [0, 0, 0], np.full(4, -0.1), cg, 0)
    damping = ref.fins(c, [-200, 0, 0], [0.2, 0, 0], np.zeros(4), cg, 0)
    rows.append(
        row(
            "symmetry_and_damping",
            {
                "neutral_moment_nm": np.linalg.norm(neutral["moment"]),
                "collective_lateral_n": np.linalg.norm(positive["force"][1:]),
                "collective_pitch_yaw_nm": np.linalg.norm(positive["moment"][1:]),
                "antisymmetric_roll_nm": abs(positive["moment"][0] + negative["moment"][0]),
                "roll_damping_sign": float(damping["moment"][0] >= 0),
            },
            dict(
                neutral_moment_nm=1e-8,
                collective_lateral_n=1e-8,
                collective_pitch_yaw_nm=1e-7,
                antisymmetric_roll_nm=1e-7,
                roll_damping_sign=0,
            ),
        )
    )
    return finish(1, "grid_fins", rows, {"experiment": c}, {"wind_ned": wind})


def check_rcs(cfg):
    c = base_config(cfg)
    rows, configs, series = [], {}, {}
    static_limits = dict(
        force_n=1e-9,
        moment_nm=1e-8,
        flow_kg_s=1e-12,
        acceleration_mps2=1e-11,
        angular_acceleration_rad_s2=1e-10,
        nozzle_activity=1e-14,
    )
    variants = [
        ("paired_pwm", "paired_3axis", "pulsed", 1.0),
        ("paired_averaged", "paired_3axis", "averaged", 1.0),
        ("roll_only", "roll_four", "pulsed", 1.0),
        ("roll_failed_nozzle", "roll_four", "averaged", [1.0, 0.0, 1.0, 1.0]),
    ]
    for name, layout, mode, scale in variants:
        local = clone(c)
        local.plant.rcs.layout, local.plant.rcs.mode, local.plant.rcs.thrust_scale = (
            layout,
            mode,
            scale,
        )
        configs[name] = local
        b = build_plant(local)
        for mass in [25700.0, 40000.0, 50000.0]:
            cg, inertia = mass_properties(local.plant.vehicle, mass, local.scenario.ic.rcs_fuel)
            state = state_for(local, b, m=mass, w_body=[0, 0, 0], v_ned=[0, 0, 0])
            R = Rotation.from_quat(state.q_bn, scalar_first=True)
            requests = [np.zeros(3), np.array([0.33, -0.25, 0.5]), np.array([1.2, -1.3, 1.0])]
            requests += [s * np.eye(3)[axis] for axis in range(3) for s in [-1, 1]]
            count = round(local.plant.rcs.pulse_period / local.plant_dt)
            for index, requested in enumerate(requests):
                for tick in range(count if mode == "pulsed" else 1):
                    time = tick * local.plant_dt
                    expected = ref.rcs(local, requested, time, cg)
                    derivative, aux = b.native.rhs(
                        state.replace(t=np.array(time)), command(local, rcs=requested), np.zeros(3)
                    )
                    # Zero rate isolates wrench/CG/frame assembly; dynamics below exercise Idot.
                    errors = dict(
                        force_n=np.linalg.norm(aux["F_rcs_body"] - expected["force"]),
                        moment_nm=np.linalg.norm(aux["M_rcs_body"] - expected["moment"]),
                        flow_kg_s=max(
                            abs(aux["rcs_mdot"] - expected["flow"]),
                            abs(derivative[5] + expected["flow"]),
                            abs(derivative[11] + expected["flow"]),
                        ),
                        acceleration_mps2=np.linalg.norm(
                            derivative[2] - R.apply(expected["force"]) / mass
                        ),
                        angular_acceleration_rad_s2=np.linalg.norm(
                            derivative[4] - np.linalg.solve(inertia, expected["moment"])
                        ),
                        nozzle_activity=np.max(np.abs(aux["rcs_thruster_on"] - expected["on"])),
                    )
                    rows.append(
                        row(
                            f"{name}_m{mass:g}_input{index}_tick{tick}",
                            errors,
                            static_limits,
                            request=requested,
                            time_s=time,
                            mass_kg=mass,
                            reference_force_n=expected["force"],
                            actual_force_n=aux["F_rcs_body"],
                            reference_moment_nm=expected["moment"],
                            actual_moment_nm=aux["M_rcs_body"],
                            reference_flow_kg_s=expected["flow"],
                            actual_flow_kg_s=aux["rcs_mdot"],
                        )
                    )
    # Time-integrated pure couples: changing I*w must equal applied angular impulse.
    for dt in [0.002, 0.001, 0.0005]:
        for axis in range(3):
            for duty in [0.25, 0.33, -0.6]:
                local = clone(c)
                local.plant_dt = dt
                b = build_plant(local)
                state = state_for(local, b, m=40000.0, w_body=[0, 0, 0], v_ned=[0, 0, 0])
                session = b.native.create_session(state)
                requested = duty * np.eye(3)[axis]
                duration = 0.4
                local.scenario.horizon.t_max = duration
                local.scenario.horizon.input_dt = dt
                local.scenario.horizon.log_dt = dt
                local.scenario.inputs = [
                    dict(
                        t=0.0,
                        throttle=[0.0] * 9,
                        fin=[0.0] * 4,
                        gimbal=[[0.0, 0.0]],
                        rcs=requested.tolist(),
                    )
                ]
                ticks = round(local.plant.rcs.pulse_period / dt)
                quantized = np.floor(abs(duty) * ticks + 0.5) / ticks
                p = local.plant.vehicle.actuator
                impulse = np.sign(duty) * 2 * p.rcs_thrust * p.rcs_arm * duration * quantized
                used = 2 * p.rcs_thrust / (p.rcs_isp * ref.G0) * duration * quantized
                end, _ = session.advance(command(local, rcs=requested), round(duration / dt))
                full = session.snapshot(True)
                _, inertia = mass_properties(local.plant.vehicle, full["m"], full["rcs_fuel"])
                errors = dict(
                    gas_kg=abs(state.rcs_fuel - full["rcs_fuel"] - used),
                    total_mass_kg=abs(state.m - full["m"] - used),
                    angular_impulse_nms=np.linalg.norm(
                        inertia @ full["w_body"] - impulse * np.eye(3)[axis]
                    ),
                    translation_mps=np.linalg.norm(full["v_ned"]),
                    main_propellant_kg=abs(
                        (full["m"] - full["rcs_fuel"]) - (state.m - state.rcs_fuel)
                    ),
                )
                key = f"pwm_axis{axis}_duty{duty:g}_{dt:g}"
                rows.append(
                    row(
                        key,
                        errors,
                        dict(
                            gas_kg=1e-9,
                            total_mass_kg=1e-7,
                            angular_impulse_nms=1e-5,
                            translation_mps=1e-12,
                            main_propellant_kg=1e-7,
                        ),
                        duration_s=duration,
                        plant_dt_s=dt,
                        request=requested,
                        effective_abs_duty=quantized,
                        reference_gas_kg=used,
                        reference_angular_impulse_nms=impulse,
                    )
                )
                configs[key] = local
    # A partial final pulse must consume only remaining RCS gas, preserving main fuel.
    local = clone(c)
    local.scenario.ic.rcs_fuel = 0.001
    local.scenario.horizon.t_max = 0.1
    local.scenario.horizon.input_dt = local.plant_dt
    local.scenario.horizon.log_dt = local.plant_dt
    local.scenario.inputs = [
        dict(t=0.0, throttle=[0.0] * 9, fin=[0.0] * 4, gimbal=[[0.0, 0.0]], rcs=[1.0, 0.0, 0.0])
    ]
    b = build_plant(local)
    tiny = 0.001
    state = state_for(local, b, m=40000.0, rcs_fuel=tiny, w_body=[0, 0, 0], v_ned=[0, 0, 0])
    session = b.native.create_session(state)
    cmd = command(local, rcs=[1, 0, 0])
    session.advance(cmd, 100)
    end = session.snapshot(True)
    _, aux = session.observe()
    _, inertia = mass_properties(local.plant.vehicle, end["m"], end["rcs_fuel"])
    impulse = (
        tiny * local.plant.vehicle.actuator.rcs_isp * ref.G0 * local.plant.vehicle.actuator.rcs_arm
    )
    rows.append(
        row(
            "gas_exhaustion",
            dict(
                remaining_gas_kg=abs(end["rcs_fuel"]),
                mass_used_kg=abs(state.m - end["m"] - tiny),
                angular_impulse_nms=np.linalg.norm(inertia @ end["w_body"] - [impulse, 0, 0]),
                post_cutoff_flow_kg_s=abs(aux["rcs_mdot"]),
                post_cutoff_moment_nm=np.linalg.norm(aux["M_rcs_body"]),
            ),
            dict(
                remaining_gas_kg=1e-12,
                mass_used_kg=1e-8,
                angular_impulse_nms=1e-5,
                post_cutoff_flow_kg_s=1e-12,
                post_cutoff_moment_nm=1e-10,
            ),
        )
    )
    configs["gas_exhaustion"] = local
    return finish(2, "rcs", rows, configs, series)


ACTUATOR_SCHEDULE = [
    (0.0, 1.2, [1.0, -1.0, 0.6, -0.6]),
    (0.6, 0.2, [0.0, 0.0, 0.0, 0.0]),
    (1.2, 0.7, [-1.0, 1.0, -0.6, 0.6]),
    (2.4, 0.0, [0.0, 0.0, 0.0, 0.0]),
]


def check_actuators(cfg):
    c = base_config(cfg)
    rows, configs, series = [], {}, {}
    limits = dict(throttle=1e-8, fin_angle_rad=1e-7, fin_rate_rad_s=2e-6)
    for model in ["simplified", "realistic"]:
        for dt in [0.002, 0.001, 0.0005]:
            local = clone(c)
            local.plant.actuator.model = model
            local.plant_dt = dt
            local.scenario.horizon.t_max = 4.0
            local.scenario.horizon.input_dt = dt
            local.scenario.horizon.log_dt = dt
            local.scenario.inputs = [
                dict(t=t, throttle=[u] + [0.0] * 8, fin=f, gimbal=[[0.0, 0.0]], rcs=[0.0, 0.0, 0.0])
                for t, u, f in ACTUATOR_SCHEDULE
            ]
            b = build_plant(local)
            state = state_for(local, b)
            session = b.native.create_session(state)
            times = np.arange(round(4.0 / dt) + 1) * dt
            expected = ref.actuators(local, model, ACTUATOR_SCHEDULE, times)
            coarse = ref.actuators(local, model, ACTUATOR_SCHEDULE, times, rtol=1e-10)
            actual = np.zeros_like(expected)
            release = {round(t / dt): (u, f) for t, u, f in ACTUATOR_SCHEDULE}
            requested_u, requested_fin = 0.0, np.zeros(4)
            clipping_error, unused_throttle = 0.0, 0.0
            for _, u, fin in ACTUATOR_SCHEDULE:
                cmd = command(local, throttle=[u] + [0.0] * 8, fin=fin)
                corrected_u = (
                    np.clip(u, 0, 1) if u >= local.plant.vehicle.engines.throttle_min else 0
                )
                clipped = command(
                    local,
                    throttle=[corrected_u] + [0.0] * 8,
                    fin=np.clip(
                        fin,
                        -local.plant.vehicle.actuator.fin_deflection_max_rad,
                        local.plant.vehicle.actuator.fin_deflection_max_rad,
                    ),
                )
                raw_d, _ = b.native.rhs(state, cmd, np.zeros(3))
                clipped_d, _ = b.native.rhs(state, clipped, np.zeros(3))
                clipping_error = max(
                    clipping_error,
                    np.max(np.abs(raw_d[10] - clipped_d[10])),
                    np.max(np.abs(raw_d[8] - clipped_d[8])),
                )
            for k in range(len(times) - 1):
                if k in release:
                    requested_u, requested_fin = release[k]
                cmd = command(local, throttle=[requested_u] + [0.0] * 8, fin=requested_fin)
                session.advance(cmd, 1)
                full = session.snapshot(True)
                actual[k + 1] = np.r_[full["throttle_act"][0], full["fin"], full["fin_rate"]]
                unused_throttle = max(unused_throttle, np.max(np.abs(full["throttle_act"][1:])))
            # Secants bound the integrated throttle speed; the independent reference
            # comparison separately checks the continuous response.
            max_throttle_rate = np.max(np.abs(np.diff(actual[:, 0]) / dt))
            errors = dict(
                throttle=np.max(np.abs(actual[:, 0] - expected[:, 0])),
                fin_angle_rad=np.max(np.abs(actual[:, 1:5] - expected[:, 1:5])),
                fin_rate_rad_s=np.max(np.abs(actual[:, 5:] - expected[:, 5:])),
            )
            stability = dict(
                throttle=np.max(np.abs(coarse[:, 0] - expected[:, 0])),
                fin_angle_rad=np.max(np.abs(coarse[:, 1:5] - expected[:, 1:5])),
                fin_rate_rad_s=np.max(np.abs(coarse[:, 5:] - expected[:, 5:])),
            )
            for key, value in stability.items():
                errors[f"reference_stability_{key}"] = value
            local_limits = {
                **limits,
                **{f"reference_stability_{key}": value * 0.01 for key, value in limits.items()},
                "native_command_clipping": 1e-10,
                "unused_engines": 1e-14,
            }
            errors.update(native_command_clipping=clipping_error, unused_engines=unused_throttle)
            if model == "realistic":
                errors.update(
                    fin_rate_exceedance_rad_s=max(
                        np.max(np.abs(actual[:, 5:]))
                        - local.plant.vehicle.actuator.fin_rate_max_rad_s,
                        0,
                    ),
                    throttle_rate_exceedance=max(
                        max_throttle_rate - local.plant.vehicle.actuator.throttle_rate_max, 0
                    ),
                )
                local_limits.update(fin_rate_exceedance_rad_s=1e-8, throttle_rate_exceedance=1e-8)
            name = f"{model}_{dt:g}"
            rows.append(
                row(
                    name,
                    errors,
                    local_limits,
                    plant_dt_s=dt,
                    duration_s=4.0,
                    schedule=ACTUATOR_SCHEDULE,
                    peak_fin_angle_rad=np.max(np.abs(actual[:, 1:5])),
                    fin_command_limit_rad=local.plant.vehicle.actuator.fin_deflection_max_rad,
                    actual_angle_hard_stop="not_assessed",
                    pure_transport_delay="not_modelled",
                )
            )
            configs[name] = local
            series[name] = dict(t=times, reference=expected, actual=actual)
    # Ideal path must use clipped effective commands, irrespective of the stored states.
    local = clone(c)
    local.environment.atmosphere.model = "exp"
    local.scenario.ic.v_ned = [0.0, 0.0, 200.0]
    local.scenario.inputs = [
        dict(
            t=0.0,
            throttle=[0.2] + [0.0] * 8,
            fin=[1.0, -1.0, 0.8, -0.8],
            gimbal=[[0.0, 0.0]],
            rcs=[0.0] * 3,
        )
    ]
    b = build_plant(local)
    state = state_for(local, b, v_ned=[0, 0, 200.0], w_body=[0, 0, 0])
    cmd = command(local, throttle=[0.2] + [0.0] * 8, fin=[1.0, -1.0, 0.8, -0.8])
    d, aux = b.native.rhs(state, cmd, np.zeros(3))
    cg, _ = mass_properties(local.plant.vehicle, state.m, state.rcs_fuel)
    R = Rotation.from_quat(state.q_bn, scalar_first=True)
    fi = ref.fins(
        local,
        R.inv().apply(state.v_ned),
        state.w_body,
        np.clip(cmd.fin, -0.35, 0.35),
        cg,
        -state.r_ned[2],
    )
    rows.append(
        row(
            "ideal_clipping_and_throttle_cutoff",
            dict(
                fin_force_n=np.linalg.norm(aux["F_fin_body"] - fi["force"]),
                fin_moment_nm=np.linalg.norm(aux["M_fin_body"] - fi["moment"]),
                below_minimum_thrust_n=np.linalg.norm(aux["T_body"]),
                below_minimum_flow_kg_s=abs(d[5]),
                ideal_fin_derivative=np.linalg.norm(d[9]),
            ),
            dict(
                fin_force_n=1e-6,
                fin_moment_nm=1e-5,
                below_minimum_thrust_n=1e-10,
                below_minimum_flow_kg_s=1e-12,
                ideal_fin_derivative=1e-12,
            ),
        )
    )
    configs["ideal"] = local
    return finish(3, "actuators", rows, configs, series)


def check_aero_environment(cfg):
    c = base_config(cfg)
    c.environment.gravity.model = "inverse_square"
    c.environment.atmosphere.g0 = 9.80665
    c.plant.thrust.model = "backpressure"
    rows, configs = [], {}
    limits = dict(rho_kg_m3=1e-12, sound_mps=1e-10, pressure_pa=1e-7, gravity_mps2=1e-12)
    # Include both sides of every geopotential layer boundary, converted to geometric h.
    hb = np.array([11000, 20000, 32000, 47000, 51000, 71000.0])
    geometric = 6356766 * hb / (6356766 - hb)
    altitudes = [-100.0, 0.0, 1000.0, 10000.0, 40000.0, 60000.0, 84852.0]
    altitudes += [h + d for h in geometric for d in [-0.001, 0, 0.001]]
    for model, delta, scale in [
        ("vacuum", 0.0, 1.0),
        ("exp", 0.0, 1.0),
        ("ussa_poly", 0.0, 1.0),
        ("ussa_poly", 15.0, 1.1),
    ]:
        for gravity in ["uniform", "inverse_square"]:
            local = clone(c)
            local.environment.atmosphere.model = model
            local.environment.atmosphere.delta_T = delta
            local.environment.atmosphere.density_scale = scale
            local.environment.gravity.model = gravity
            name = f"{model}_dT{delta:g}_scale{scale:g}_{gravity}"
            configs[name] = local
            b = build_plant(local)
            for altitude in altitudes:
                expected = ref.environment(local, altitude)
                actual = b.native.environment_diagnostics(altitude)
                errors = dict(
                    rho_kg_m3=abs(actual["rho"] - expected["rho"]),
                    sound_mps=abs(actual["a_sound"] - expected["a_sound"]),
                    pressure_pa=abs(actual["pressure"] - expected["pressure"]),
                    gravity_mps2=np.linalg.norm(actual["gravity_ned"] - expected["gravity_ned"]),
                )
                rows.append(
                    row(
                        f"environment_{name}_h{altitude:.6f}",
                        errors,
                        limits,
                        altitude_m=altitude,
                        reference=expected,
                        actual=actual,
                    )
                )
    # Body forces at nodes, between nodes and outside the table. Fins/engines/RCS off.
    local = clone(c)
    local.environment.atmosphere.model = "ussa_poly"
    local.environment.atmosphere.delta_T, local.environment.atmosphere.density_scale = 0.0, 1.0
    b = build_plant(local)
    b.native.set_fin_deployment(0)
    lookup = ref.body_table(local)
    rotation = Rotation.from_euler(
        "ZYX",
        [local.scenario.ic.yaw_deg, local.scenario.ic.pitch_deg, local.scenario.ic.roll_deg],
        degrees=True,
    )
    wind = np.array([15.0, -8.0, 4.0])
    force_limits = {
        **WRENCH_LIMITS,
        "cd": 1e-12,
        "cl": 1e-12,
        "cp_m": 1e-11,
        "relative_flow_mps": 1e-10,
        "mach": 1e-12,
        "alpha_rad": 1e-7,
    }
    for altitude in [0.0, 10000.0, 40000.0]:
        env = ref.environment(local, altitude)
        for mass in [25700.0, 50000.0]:
            cg, inertia = mass_properties(local.plant.vehicle, mass, local.scenario.ic.rcs_fuel)
            for mach in [0.0, 0.2, 0.9, 1.0, 1.13, 3.0, 8.0, 9.0]:
                for alpha_deg in [0.0, 2.5, 5.0, 17.0, 45.0, 89.0, 90.0, 110.0, 180.0]:
                    alpha = np.deg2rad(alpha_deg)
                    for azimuth, rates in [
                        (0.0, np.zeros(3)),
                        (0.7, np.array([0.03, -0.04, 0.05])),
                    ]:
                        velocity = (
                            mach
                            * env["a_sound"]
                            * np.array(
                                [
                                    -np.cos(alpha),
                                    np.sin(alpha) * np.cos(azimuth),
                                    np.sin(alpha) * np.sin(azimuth),
                                ]
                            )
                        )
                        expected = ref.body(local, lookup, velocity, rates, cg, altitude)
                        diagnostic = b.native.aero_wrench_diagnostics(
                            velocity, rates, mass, altitude
                        )
                        state = state_for(
                            local,
                            b,
                            m=mass,
                            r_ned=[0, 0, -altitude],
                            v_ned=rotation.apply(velocity) + wind,
                            w_body=rates,
                        )
                        d, aux = b.native.rhs(state, command(local), wind)
                        expected_wdot = np.linalg.solve(
                            inertia, expected["moment"] - np.cross(rates, inertia @ rates)
                        )
                        errors = dict(
                            force_n=max(
                                np.linalg.norm(diagnostic["force"] - expected["force"]),
                                np.linalg.norm(aux["F_aero_body"] - expected["force"]),
                            ),
                            moment_nm=max(
                                np.linalg.norm(diagnostic["moment"] - expected["moment"]),
                                np.linalg.norm(aux["M_body_total"] - expected["moment"]),
                            ),
                            acceleration_mps2=np.linalg.norm(
                                d[2] - env["gravity_ned"] - rotation.apply(expected["force"]) / mass
                            ),
                            angular_acceleration_rad_s2=np.linalg.norm(d[4] - expected_wdot),
                            cd=max(
                                abs(diagnostic["cd"] - expected["cd"]),
                                abs(aux["aero_cd"] - expected["cd"]),
                            ),
                            cl=max(
                                abs(diagnostic["cl"] - expected["cl"]),
                                abs(aux["aero_cl"] - expected["cl"]),
                            ),
                            cp_m=abs(diagnostic["x_cp"] - expected["x_cp"]),
                            relative_flow_mps=np.linalg.norm(aux["v_air_body"] - velocity),
                            mach=abs(aux["mach"] - expected["mach"]),
                            alpha_rad=abs(aux["aero_alpha"] - expected["alpha"]),
                        )
                        name = (
                            f"body_h{altitude:g}_m{mass:g}_M{mach:g}"
                            f"_alpha{alpha_deg:g}_az{azimuth:g}"
                        )
                        rows.append(
                            row(
                                name,
                                errors,
                                force_limits,
                                altitude_m=altitude,
                                mass_kg=mass,
                                velocity_body=velocity,
                                rates_body=rates,
                                wind_ned=wind,
                                reference=expected,
                                actual=diagnostic,
                            )
                        )
    configs["body_table"] = local
    return finish(4, "aero_environment", rows, configs)


TRAJECTORY_LIMITS = dict(
    position_m=1e-5,
    velocity_mps=1e-6,
    attitude_rad=1e-8,
    angular_rate_rad_s=1e-8,
    mass_kg=1e-7,
    gas_kg=1e-8,
)


def trajectory_errors(actual, expected):
    return dict(
        position_m=float(np.max(np.linalg.norm(actual["r_ned"] - expected["r_ned"], axis=1))),
        velocity_mps=float(np.max(np.linalg.norm(actual["v_ned"] - expected["v_ned"], axis=1))),
        attitude_rad=float(np.max(attitude_error(actual["q_bn"], expected["q_bn"]))),
        angular_rate_rad_s=float(
            np.max(np.linalg.norm(actual["w_body"] - expected["w_body"], axis=1))
        ),
        mass_kg=float(np.max(np.abs(actual["m"] - expected["m"]))),
        gas_kg=float(np.max(np.abs(actual["rcs_fuel"] - expected["rcs_fuel"]))),
    )


def coupled_experiments(cfg):
    c = base_config(cfg)
    c.plant.rcs.mode = "averaged"
    c.plant.thrust.model = "backpressure"
    c.environment.gravity.model = "inverse_square"
    c.environment.atmosphere.model = "ussa_poly"
    c.environment.atmosphere.g0 = 9.80665
    c.scenario.ic.r_ned = [100.0, -70.0, -8000.0]
    c.scenario.ic.v_ned = [30.0, -10.0, 200.0]
    c.scenario.ic.w_body = [0.03, -0.02, 0.015]
    c.scenario.ic.pitch_deg = 80.0
    c.scenario.horizon.t_max = 3.0
    cases = []
    u = [0.6, 0.4, 0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0]
    for name, atmosphere, throttle, deployment in [
        ("vacuum_asymmetric_burn", "vacuum", [0.7, 0.5] + [0.0] * 7, 0.0),
        ("aero_coast", "ussa_poly", [0.0] * 9, 1.0),
        ("coupled_burn", "ussa_poly", u, 1.0),
        ("coupled_burn_stowed", "ussa_poly", u, 0.0),
    ]:
        local = clone(c)
        local.environment.atmosphere.model = atmosphere
        inputs = dict(
            throttle=throttle,
            gimbal=[[0.025, -0.018]],
            fin=[0.08, 0.03, -0.06, -0.02],
            rcs=[0.1, -0.15, 0.2],
        )
        local.scenario.name = name
        local.scenario.ic.throttle0 = throttle
        local.scenario.inputs = [dict(t=0.0, fin_deployment=deployment, **inputs)]
        cases.append((name, local, inputs, deployment))
    return cases


def check_coupled(cfg):
    rows, configs, series = [], {}, {}
    fine_times = np.arange(6001) * 0.0005
    for name, local, inputs, deployment in coupled_experiments(cfg):
        print(f"  independent 6-DOF reference: {name}", flush=True)
        fine = ref.coupled(local, fine_times, inputs, deployment)
        coarse = ref.coupled(local, fine_times, inputs, deployment, rtol=1e-11)
        stability = trajectory_errors(coarse, fine)
        rows.append(
            row(
                f"{name}_reference_refinement",
                stability,
                {key: value * 0.01 for key, value in TRAJECTORY_LIMITS.items()},
                method="DOP853 matrix attitude; rtol 1e-11 versus 1e-13; max_step 0.01 s",
            )
        )
        outputs = {}
        for dt in [0.002, 0.001, 0.0005]:
            c = clone(local)
            c.plant_dt = dt
            c.scenario.horizon.input_dt = dt
            c.scenario.horizon.log_dt = dt
            b = build_plant(c)
            state = state_for(c, b)
            session = b.native.create_session(state)
            # reset/create_session initializes exposure to deployed; set the live session.
            session.set_fin_deployment(deployment)
            _, initial_aux = session.observe()
            if initial_aux["fin_deployment"] != deployment:
                raise RuntimeError("live session deployment differs from reference condition")
            cmd = command(c, **inputs)
            stride = round(dt / 0.0005)
            times = fine_times[::stride]
            expected = {key: value[::stride] for key, value in fine.items()}
            actual = {key: np.empty_like(value) for key, value in expected.items()}
            full = session.snapshot(True)
            for key in actual:
                actual[key][0] = full[key]
            for k in range(len(times) - 1):
                session.advance(cmd, 1)
                full = session.snapshot(True)
                for key in actual:
                    actual[key][k + 1] = full[key]
                if abs(full["t"] - times[k + 1]) > 1e-9 or full["done"]:
                    raise RuntimeError("coupled experiment ended early or time grids disagree")
            key = f"{name}_{dt:g}"
            rows.append(
                row(
                    key,
                    trajectory_errors(actual, expected),
                    TRAJECTORY_LIMITS,
                    plant_dt_s=dt,
                    duration_s=3.0,
                    fin_deployment=deployment,
                    initial_state=dict(
                        r_ned=c.scenario.ic.r_ned,
                        v_ned=c.scenario.ic.v_ned,
                        w_body=c.scenario.ic.w_body,
                        mass_kg=c.scenario.ic.m,
                    ),
                    inputs=inputs,
                )
            )
            configs[key] = c
            outputs[dt] = actual
            series[key] = dict(
                t=times,
                **{f"actual_{k}": v for k, v in actual.items()},
                **{f"reference_{k}": v for k, v in expected.items()},
            )
        # Adjacent native timesteps at shared epochs; bounded agreement, not an order claim
        # when roundoff and finite-difference Idot dominate truncation error.
        for large, small in [(0.002, 0.001), (0.001, 0.0005)]:
            aligned = {key: value[::2] for key, value in outputs[small].items()}
            rows.append(
                row(
                    f"{name}_dt_{large:g}_versus_{small:g}",
                    trajectory_errors(outputs[large], aligned),
                    TRAJECTORY_LIMITS,
                    purpose="shared-epoch timestep agreement",
                )
            )
    return finish(5, "coupled_6dof", rows, configs, series)
