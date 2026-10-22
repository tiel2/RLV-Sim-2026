"""Independent references for restricted vehicle experiments.

Only configuration constants and requested sample times enter this module. No Plant,
native frames, simulated state, or simulated diagnostics are used. Numerical references
integrate a separate RHS with DOP853 twice; rotation uses matrices rather than the
Plant's quaternion RK4 equation. These are implementation checks, not flight validation.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

G0 = 9.80665


def attitude_error(actual, expected):
    """Shortest relative rotation in radians; scalar-first quaternions, sign invariant."""
    a = Rotation.from_quat(np.roll(actual, -1, axis=-1))
    b = Rotation.from_quat(np.roll(expected, -1, axis=-1))
    return (b.inv() * a).magnitude()


def mass_properties(vehicle, total, gas):
    """CG and full inertia from configuration; integrate cylinder axial moments.

    Gauss quadrature of the distributed mass is independent of the Plant's closed-form
    tank formula. Prescribed mass properties remain an explicitly separate model.
    """
    p = vehicle.mass
    if vehicle.massprops_model == "prescribed":
        fraction = np.clip((total - p.m_dry) / (p.m_wet - p.m_dry), 0, 1)
        cg = p.x_cg_dry + fraction * (p.x_cg_wet - p.x_cg_dry)
        diagonal = np.array(p.I_dry) + fraction * (np.array(p.I_wet) - p.I_dry)
    elif vehicle.massprops_model == "dual_tank":
        tanks = vehicle.tanks
        propellant = total - p.m_dry - gas
        if propellant < -1e-9 or gas < 0:
            raise ValueError("reference mass must include structure, main propellant, and gas")
        propellant = max(propellant, 0)
        liquid = propellant * np.array([tanks.of_ratio, 1]) / (1 + tanks.of_ratio)
        radii = np.r_[tanks.radius, tanks.rcs_gas_radius]
        heights = np.r_[
            liquid / (np.pi * np.array(tanks.radius) ** 2 * tanks.density),
            tanks.rcs_gas_length,
        ]
        centers = np.r_[np.array(tanks.bottom) + heights[:2] / 2, tanks.rcs_gas_x]
        masses = np.r_[liquid, gas]
        cg = (p.m_dry * p.x_cg_dry + masses @ centers) / total
        diagonal = np.array(p.I_dry, dtype=float)
        diagonal[1:] += p.m_dry * (p.x_cg_dry - cg) ** 2
        nodes, weights = np.polynomial.legendre.leggauss(4)
        for mass, radius, height, center in zip(masses, radii, heights, centers, strict=True):
            axial_positions = center + height * nodes / 2
            diagonal[0] += mass * radius**2 / 2
            diagonal[1:] += mass * (radius**2 / 4 + weights @ (axial_positions - cg) ** 2 / 2)
    else:
        raise ValueError("unsupported reference mass-properties model")
    inertia = np.diag(diagonal)
    inertia[0, 1] = inertia[1, 0] = p.inertia_products[0]
    inertia[0, 2] = inertia[2, 0] = p.inertia_products[1]
    inertia[1, 2] = inertia[2, 1] = p.inertia_products[2]
    return float(cg), inertia


def _integrate(rhs, initial, times, unpack):
    """Return the tighter solution and differences after tightening tolerances 100x."""
    solutions = []
    for tolerance in (1e-10, 1e-12):
        if times[-1] == 0:
            values = np.asarray(initial)[None, :]
        else:
            sol = solve_ivp(
                rhs,
                (0, times[-1]),
                initial,
                method="DOP853",
                t_eval=times,
                rtol=tolerance,
                atol=tolerance * 0.01,
            )
            if not sol.success or not np.all(np.isfinite(sol.y)):
                raise ValueError(f"independent reference integration failed: {sol.message}")
            values = sol.y.T
        solutions.append(unpack(values))
    coarse, fine = solutions
    differences = {}
    for key in fine:
        if key == "q_bn":
            error = attitude_error(coarse[key], fine[key])
        else:
            error = np.linalg.norm(coarse[key] - fine[key], axis=1)
        differences[key] = float(np.max(error))
    return fine, {
        "method": "DOP853",
        "rtol": [1e-10, 1e-12],
        "atol": [1e-12, 1e-14],
        "max_refinement_difference": differences,
    }


def build_reference(cfg, times):
    """Compute expected trajectories from the scenario specification, never its output."""
    kind = str(cfg.scenario.validation.kind)
    times = np.asarray(times, dtype=float)

    def require(condition, reason):
        if not condition:
            raise ValueError(f"{kind} is not applicable: {reason}")

    require(
        times.ndim == 1
        and len(times) > 0
        and np.all(np.isfinite(times))
        and times[0] == 0
        and np.all(np.diff(times) > 0),
        "invalid reference sample times",
    )
    env, plant, scenario = cfg.environment, cfg.plant, cfg.scenario
    require(scenario.input_mode == "prescribed", "requires prescribed inputs")
    require(not scenario.dispersion.enable, "requires deterministic initial conditions")
    require(env.earth.model == "fixed", "requires fixed local frame")
    require(env.wind.model == "none", "requires no wind")
    require(plant.slosh.model == "none", "requires slosh disabled")
    require(
        env.atmosphere.model == ("exp" if kind == "constant_drag" else "vacuum"),
        "requires exponential atmosphere" if kind == "constant_drag" else "requires vacuum",
    )
    require(
        cfg.fault.engine_out_idx is None
        and np.allclose(cfg.fault.thrust_scale, 1)
        and np.allclose(cfg.fault.tvc_bias, 0),
        "requires healthy engines",
    )
    require(np.allclose(plant.vehicle.mass.cg_lateral, 0), "requires axial CG")
    for row in scenario.inputs:
        for field in ("gimbal", "fin", "rcs"):
            require(np.allclose(row.get(field, 0), 0), f"requires zero {field} inputs")
    ic, vehicle = scenario.ic, plant.vehicle
    r0, v0, w0 = (np.array(ic[k], dtype=float) for k in ("r_ned", "v_ned", "w_body"))
    m0 = float(ic.m)
    gas = float(ic.get("rcs_fuel", vehicle.actuator.rcs_fuel0))
    rotation = Rotation.from_euler("ZYX", [ic.yaw_deg, ic.pitch_deg, ic.roll_deg], degrees=True)
    q0 = np.roll(rotation.as_quat(), 1)
    count = len(times)
    ref = {
        "t": times.copy(),
        "r_ned": r0 + times[:, None] * v0,
        "v_ned": np.tile(v0, (count, 1)),
        "mass": np.full(count, m0),
        "q_bn": np.tile(q0, (count, 1)),
        "w_body": np.tile(w0, (count, 1)),
        "thrust_mag": np.zeros(count),
        "rcs_fuel": np.full(count, gas),
    }
    info = {
        "source": "resolved configuration and sample times only",
        "method": "closed-form solution",
        "kind": kind,
        "initial_state": {
            "r_ned": r0.tolist(),
            "v_ned": v0.tolist(),
            "q_bn": q0.tolist(),
            "w_body": w0.tolist(),
            "mass_kg": m0,
        },
    }
    burn = kind in ("rocket_equation", "mass_depletion")
    if not burn:
        require(
            np.allclose(ic.throttle0, 0)
            and all(np.allclose(row.get("throttle", 0), 0) for row in scenario.inputs),
            "requires engines off",
        )
    if kind != "free_rotation":
        require(np.allclose(w0, 0), "requires no initial rotation")
    if kind in ("freefall_uniform", "constant_drag", "free_rotation") or burn:
        require(env.gravity.model == "uniform", "requires uniform gravity")
    g = float(env.atmosphere.g0)
    if kind == "freefall_uniform":
        ref["r_ned"][:, 2] += 0.5 * g * times**2
        ref["v_ned"][:, 2] += g * times
    elif kind == "inverse_square_energy":
        require(env.gravity.model == "inverse_square", "requires inverse-square gravity")
        require(np.allclose(v0[:2], 0), "reference requires vertical free fall")
        mu, radius = float(env.gravity.mu), float(env.gravity.radius)

        def rhs(_, y):
            return [y[1], mu / (radius - y[0]) ** 2]

        def unpack(y):
            r, v = np.tile(r0, (len(y), 1)), np.tile(v0, (len(y), 1))
            r[:, 2], v[:, 2] = y[:, 0], y[:, 1]
            return {"r_ned": r, "v_ned": v}

        solution, details = _integrate(rhs, [r0[2], v0[2]], times, unpack)
        ref.update(solution)
        info.update(method="independent vertical ODE", numerical_reference=details)
        e0 = 0.5 * v0 @ v0 - mu / (radius - r0[2])
        ref["expected_specific_energy"] = np.full(count, e0)
    elif kind == "free_rotation":
        require(g == 0, "requires zero gravity")
        _, inertia = mass_properties(vehicle, m0, gas)
        require(np.linalg.norm(w0) > 0, "requires nonzero angular velocity")

        def rhs(_, y):
            C, w = y[:9].reshape(3, 3), y[9:]
            wx, wy, wz = w
            skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
            return np.r_[(C @ skew).ravel(), np.linalg.solve(inertia, -np.cross(w, inertia @ w))]

        def unpack(y):
            q = Rotation.from_matrix(y[:, :9].reshape(-1, 3, 3)).as_quat()
            return {"q_bn": np.roll(q, 1, axis=1), "w_body": y[:, 9:]}

        solution, details = _integrate(rhs, np.r_[rotation.as_matrix().ravel(), w0], times, unpack)
        ref.update(solution)
        info.update(method="independent matrix/Euler rigid-body ODE", numerical_reference=details)
        info["inertia_kg_m2"] = inertia.tolist()
        H0 = rotation.apply(inertia @ w0)
        ref["expected_angular_momentum"] = np.tile(H0, (count, 1))
        ref["expected_rotational_energy"] = np.full(count, 0.5 * w0 @ inertia @ w0)
    elif burn:
        require(g == 0 and plant.actuator.model == "ideal", "requires zero gravity/ideal actuation")
        require(len(scenario.inputs) == 1, "requires a constant command")
        engines = vehicle.engines
        throttle = np.broadcast_to(np.asarray(scenario.inputs[0].throttle), (engines.n_engines,))
        require(np.allclose(ic.throttle0, throttle), "initial throttle must match the command")
        # Restrict this straight-flight reference to an axial center engine.
        require(
            throttle[0] >= engines.throttle_min and np.all(throttle[1:] == 0),
            "reference requires only the center engine, above minimum throttle",
        )
        require(np.allclose(engines.get("cant", 0), 0), "requires axial thrust")
        thrust = float(throttle[0] * engines.thrust_vac)
        exhaust = float(engines.isp) * G0
        flow = thrust / exhaust
        floor = float(vehicle.mass.m_dry) + gas
        depletion = (m0 - floor) / flow
        require(depletion > 0, "requires initial main propellant")
        burn_time = np.minimum(times, depletion)
        mass = m0 - flow * burn_time
        log_ratio = -np.log1p(-flow * burn_time / m0)
        dv = exhaust * log_ratio
        # Integral of log(m0 / (m0 - flow*t)), followed by post-depletion coast.
        distance = exhaust * (burn_time - mass / flow * log_ratio)
        distance += (times - burn_time) * dv
        axis = rotation.apply([1, 0, 0])
        ref["mass"] = mass
        ref["v_ned"] += dv[:, None] * axis
        ref["r_ned"] += distance[:, None] * axis
        ref["thrust_mag"] = np.where(times < depletion, thrust, 0)
        info.update(
            mass_flow_kg_s=flow,
            expected_depletion_s=depletion,
            thrust_n=thrust,
            exhaust_speed_mps=exhaust,
        )
    elif kind == "constant_drag":
        aero, atmosphere = plant.aero, env.atmosphere
        require(
            aero.model == "table" and aero.backend == "analytic", "requires analytic drag table"
        )
        require(aero.fins.fin_area == 0, "requires zero fin area")
        require(
            all(aero.coeffs[k] == 0 for k in ("cd_transonic", "cd_hyper", "cd_alpha", "cn_alpha")),
            "requires constant drag coefficient and zero lift",
        )
        require(np.allclose(v0[:2], 0), "requires vertical descent")
        forward = rotation.apply(vehicle.geom.fwd_axis)
        require(
            np.linalg.norm(np.cross(forward, [0, 0, 1])) < 1e-12,
            "requires axial descent without aerodynamic torque",
        )
        # Reference density is evaluated from input altitude, not the Plant's diagnostics.
        rho = float(atmosphere.rho0) * np.exp(-max(-r0[2], 0) / atmosphere.scale_height)
        cd, area = float(aero.coeffs.cd_base), float(vehicle.geom.s_ref)
        require(g > 0 and rho > 0 and cd > 0, "requires positive gravity, density, and CD")
        terminal = np.sqrt(2 * m0 * g / (rho * area * cd))
        require(0 <= v0[2] < terminal, "requires subterminal initial descent")
        offset = np.arctanh(v0[2] / terminal)
        arg = g * times / terminal + offset
        ref["v_ned"][:, 2] = terminal * np.tanh(arg)
        log_cosh = lambda x: np.logaddexp(x, -x) - np.log(2)
        ref["r_ned"][:, 2] = r0[2] + terminal**2 / g * (log_cosh(arg) - log_cosh(offset))
        rho_path = float(atmosphere.rho0) * np.exp(
            -np.maximum(-ref["r_ned"][:, 2], 0) / atmosphere.scale_height
        )
        variation = float(np.max(np.abs(rho_path / rho - 1)))
        require(variation < 1e-7, "density must be effectively constant along the reference")
        ref["rho"] = rho_path
        ref["aero_cd"] = np.full(count, cd)
        ref["drag_force_n"] = 0.5 * rho * area * cd * ref["v_ned"][:, 2] ** 2
        info.update(
            terminal_speed_mps=float(terminal),
            reference_density_kg_m3=rho,
            relative_density_variation=variation,
            approximation="density frozen at initial altitude for the analytical trajectory",
        )
    else:
        raise ValueError(f"unknown validation kind {kind!r}")
    return ref, info
