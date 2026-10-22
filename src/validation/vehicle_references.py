"""Configuration-only references; no production geometry, aero, frames or native calls.

These verify the declared models, not measured vehicle fidelity. Body tables are
rebuilt from their analytic specification using SciPy's PCHIP (native uses Hermite).
The trajectory reference integrates rotation matrices rather than quaternions.
"""

from collections.abc import Mapping, Sequence
from types import SimpleNamespace

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import PchipInterpolator
from scipy.spatial.transform import Rotation
from scipy.special import expit

from validation.references import G0, mass_properties


def _constants(value):
    """Read configuration once; avoid resolver overhead in thousands of ODE calls."""
    if isinstance(value, Mapping):
        return SimpleNamespace(**{key: _constants(item) for key, item in value.items()})
    if isinstance(value, Sequence) and not isinstance(value, str):
        return [_constants(item) for item in value]
    return value


def environment(cfg, altitude):
    a, grav = cfg.environment.atmosphere, cfg.environment.gravity
    h = max(float(altitude), 0.0)
    if a.model == "vacuum":
        rho, sound, pressure = 0.0, 340.3, 0.0
    elif a.model == "exp":
        temperature = 288.15 - 0.0065 * h if h <= 11000 else 216.65
        rho = a.rho0 * np.exp(-h / a.scale_height)
        sound = np.sqrt(1.4 * 287.05 * temperature)
        pressure = a.p_sea * np.exp(-h / a.scale_height)
    elif a.model == "ussa_poly":
        base = np.array([0, 11000, 20000, 32000, 47000, 51000, 71000])
        lapse = np.array([-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002])
        tb = np.array([288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65])
        pb = np.array([101325, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956420])
        geopotential = 6356766 * h / (6356766 + h)
        i = min(np.searchsorted(base, geopotential, side="right") - 1, 6)
        dh = geopotential - base[i]
        temperature = tb[i] + lapse[i] * dh
        pressure = pb[i] * (
            np.exp(-G0 * dh / (287.05287 * tb[i]))
            if lapse[i] == 0
            else (temperature / tb[i]) ** (-G0 / (lapse[i] * 287.05287))
        )
        actual_temperature = max(temperature + a.delta_T, 1)
        rho = a.density_scale * pressure / (287.05287 * actual_temperature)
        sound = np.sqrt(1.4 * 287.05287 * actual_temperature)
    else:
        raise ValueError("unsupported reference atmosphere")
    if cfg.plant.thrust.model == "vacuum":
        pressure = 0.0
    elif cfg.plant.thrust.model == "sea_level":
        pressure = float(a.p_sea)
    g = a.g0 if grav.model == "uniform" else grav.mu / (grav.radius + altitude) ** 2
    return dict(
        rho=float(rho),
        a_sound=float(sound),
        pressure=float(pressure),
        gravity_ned=np.array([0.0, 0.0, g]),
    )


def unit(x):
    norm = np.linalg.norm(x)
    return x / norm if norm > 1e-8 else np.zeros_like(x)


def fins(cfg, velocity, rates, angles, cg, altitude):
    """Independent per-fin local-flow forces, retaining individual contributions."""
    p = cfg.plant.aero.fins
    if p.incremental:
        raise ValueError("reference covers analytic component fins only")
    env = environment(cfg, altitude)
    forces, moments, flows = [], [], []
    phase = np.deg2rad(p.fin_phase_deg)
    for i, delta in enumerate(angles):
        theta = phase + 2 * np.pi * i / p.n_fins
        radial = np.array([0, np.cos(theta), np.sin(theta)])
        normal = np.cross([1, 0, 0], radial)
        arm = np.array([p.fin_x - cg, 0, 0]) + p.fin_radius * radial
        flow = np.asarray(velocity) + np.cross(rates, arm)
        speed = np.linalg.norm(flow)
        direction = flow / speed if speed > 1e-6 else np.zeros(3)
        mach = speed / env["a_sound"]
        incidence = np.arctan2(
            flow @ normal, max(abs(flow @ cfg.plant.vehicle.geom.fwd_axis), 1e-9)
        )
        stall = np.deg2rad(p.fin_stall_deg)
        alpha = stall * np.tanh((delta - incidence) / stall)
        lift_axis = unit(normal - (normal @ direction) * direction)
        efficiency = np.clip(((1 - mach**2) ** 2 + 0.04) ** -0.25, 0.35, 1)
        efficiency *= 1 - 0.45 * expit((mach - 0.75) / 0.08) * expit((1.6 - mach) / 0.08)
        drag_rise = 1 + 2 * np.exp(-(((mach - p.fin_transonic_mach) / 0.15) ** 2))
        qarea = 0.5 * env["rho"] * speed**2 * p.fin_area
        force = qarea * (
            p.fin_cla * efficiency * alpha * lift_axis
            - (p.fin_cd0 * drag_rise + p.fin_cd_alpha2 * efficiency * alpha**2) * direction
        )
        forces.append(force)
        moments.append(np.cross(arm, force))
        flows.append(flow)
    return dict(
        force=np.sum(forces, axis=0),
        moment=np.sum(moments, axis=0),
        individual_force=np.array(forces),
        individual_moment=np.array(moments),
        local_flow=np.array(flows),
    )


def rcs(cfg, command, time, cg):
    """Explicit paired-nozzle geometry and integer-tick PWM, including quantization."""
    p, a = cfg.plant.rcs, cfg.plant.vehicle.actuator
    duty = np.clip(command, -1, 1)
    if p.mode == "pulsed":
        count = round(p.pulse_period / cfg.plant_dt)
        phase = int(np.floor(time / cfg.plant_dt + 1e-7)) % count
        duty = np.sign(duty) * (phase < np.floor(np.abs(duty) * count + 0.5))
    # Axes of offset and force specified independently as Cartesian vectors.
    offsets = [[0, 1, 0], [0, 0, 1], [0, 1, 0]]
    directions = [[0, 0, 1], [1, 0, 0], [-1, 0, 0]]
    axes = range(1 if p.layout == "roll_four" else 3)
    n = 4 * len(axes)
    scales = np.broadcast_to(np.asarray(p.thrust_scale), (n,))
    force, moment, on = np.zeros(3), np.zeros(3), []
    flow = 0.0
    index = 0
    for axis in axes:
        for sign in [1, -1]:
            for side in [1, -1]:
                firing = max(sign * duty[axis], 0)
                arm = np.array([p.center_x - cg, 0, 0]) + side * a.rcs_arm * np.array(offsets[axis])
                f = firing * scales[index] * a.rcs_thrust * side * sign * np.array(directions[axis])
                force += f
                moment += np.cross(arm, f)
                flow += firing * scales[index] * a.rcs_thrust / (a.rcs_isp * G0)
                on.append(firing)
                index += 1
    return dict(force=force, moment=moment, flow=flow, on=np.array(on))


def body_table(cfg):
    """Reconstruct from constants, never read the production coefficient arrays."""
    p, g = cfg.plant.aero.coeffs, cfg.plant.aero.grid
    if cfg.plant.aero.backend != "analytic" or cfg.plant.aero.model != "table":
        raise ValueError("reference covers analytic-generated body tables")
    alpha = np.linspace(0, np.deg2rad(g.alpha_max_deg), g.n_alpha)[::-1]
    mach = np.linspace(g.mach_min, g.mach_max, g.n_mach)
    cd0 = (
        p.cd_base
        + p.cd_transonic * np.exp(-(((mach - p.transonic_mach) / p.transonic_width) ** 2))
        + p.cd_hyper / (1 + np.exp(-2 * (mach - p.hyper_mach) / p.hyper_width))
    )
    s2 = np.sin(alpha)[:, None] ** 2
    tables = [
        cd0 + p.cd_alpha * s2,
        np.broadcast_to(
            p.cn_alpha * np.sin(alpha)[:, None] * np.cos(alpha)[:, None], s2.shape[:1] + mach.shape
        ),
        np.broadcast_to(
            -cfg.plant.vehicle.geom.length * (p.xcp_frac_base + p.xcp_frac_alpha * s2),
            s2.shape[:1] + mach.shape,
        ),
    ]
    interpolators = [PchipInterpolator(np.cos(alpha), t, axis=0) for t in tables]

    def lookup(cosine, m):
        c = np.clip(cosine, np.cos(alpha[0]), np.cos(alpha[-1]))
        return np.array([np.interp(m, mach, interp(c)) for interp in interpolators])

    return lookup


def body(cfg, lookup, velocity, rates, cg, altitude):
    env, geom = environment(cfg, altitude), cfg.plant.vehicle.geom
    speed = np.linalg.norm(velocity)
    direction = np.asarray(velocity) / speed if speed > 1e-6 else np.zeros(3)
    cosine = np.clip(np.dot(geom.fwd_axis, direction), -1, 1) if speed > 1e-6 else 1.0
    cd, cl, cp = lookup(cosine, speed / env["a_sound"])
    q = env["rho"] * speed**2 / 2
    lift_axis = (
        unit(np.asarray(geom.fwd_axis) - cosine * direction) if speed > 1e-6 else np.zeros(3)
    )
    force = q * geom.s_ref * (-cd * direction + cl * lift_axis)
    moment = np.cross([cp - cg, 0, 0], force)
    if speed > 1e-6:
        moment -= q * geom.s_ref * geom.length**2 / (4 * speed) * np.array([0, rates[1], rates[2]])
    return dict(
        force=force,
        moment=moment,
        cd=cd,
        cl=cl,
        x_cp=cp,
        alpha=np.arccos(cosine),
        mach=speed / env["a_sound"],
        q_dyn=q,
    )


def actuators(cfg, model, schedule, times, rtol=1e-12):
    """Throttle and four-fin response, split exactly at command changes.

    Throttle command below minimum switches off. Angle bounds clip commands only.
    Simplified response has a first-order exponential and closed-form 2nd order servo.
    """
    a = cfg.plant.vehicle.actuator
    result = np.zeros((len(times), 9))  # u, fin(4), rate(4)
    state = np.zeros(9)
    for i, (start, throttle, fin) in enumerate(schedule):
        end = schedule[i + 1][0] if i + 1 < len(schedule) else times[-1]
        target_u = (
            np.clip(throttle, 0, 1) if throttle >= cfg.plant.vehicle.engines.throttle_min else 0
        )
        target = np.clip(fin, -a.fin_deflection_max_rad, a.fin_deflection_max_rad)
        mask = (times >= start) & (times <= end)
        local = times[mask] - start
        if model == "simplified":
            w, z = a.fin_omega, a.fin_zeta
            wd = w * np.sqrt(1 - z * z)
            c = state[1:5] - target
            s = (state[5:] + z * w * c) / wd

            def exact(t, c=c, s=s, u=state[0], target=target, target_u=target_u, z=z, w=w, wd=wd):
                t = np.asarray(t)
                decay = np.exp(-z * w * t)[:, None]
                co, si = np.cos(wd * t)[:, None], np.sin(wd * t)[:, None]
                return np.column_stack(
                    [
                        target_u + (u - target_u) * np.exp(-t / a.throttle_tau),
                        target + decay * (c * co + s * si),
                        decay * ((-z * w * c + wd * s) * co + (-z * w * s - wd * c) * si),
                    ]
                )

            result[mask] = exact(local)
            state = exact([end - start])[0]
        elif model == "realistic":

            def rhs(_, y, target=target, target_u=target_u):
                du = a.throttle_rate_max * np.tanh(
                    (target_u - y[0]) / (a.throttle_tau * a.throttle_rate_max)
                )
                desired = a.fin_rate_max_rad_s * np.tanh(
                    a.fin_omega * (target - y[1:5]) / (2 * a.fin_zeta * a.fin_rate_max_rad_s)
                )
                return np.r_[du, y[5:], 2 * a.fin_zeta * a.fin_omega * (desired - y[5:])]

            sol = solve_ivp(
                rhs,
                (0, end - start),
                state,
                method="DOP853",
                rtol=rtol,
                atol=rtol * 0.01,
                dense_output=True,
            )
            if not sol.success:
                raise RuntimeError(sol.message)
            result[mask] = sol.sol(local).T
            state = sol.y[:, -1]
        else:
            raise ValueError("dynamic actuator reference requires simplified or realistic")
    return result


def inertia_rate(vehicle, mass, gas, main_flow, gas_flow):
    """Analytic derivative about moving CG, independent of native finite differences."""
    if vehicle.massprops_model != "dual_tank":
        raise ValueError("coupled reference requires dual tanks")
    p, t = vehicle.mass, vehicle.tanks
    fractions = np.array([t.of_ratio, 1]) / (1 + t.of_ratio)
    liquid = (mass - p.m_dry - gas) * fractions
    radii = np.array(t.radius)
    k = 1 / (np.pi * radii**2 * np.array(t.density))
    cg = mass_properties(vehicle, mass, gas)[0]
    # I_trans = I_origin - Q^2/M; d(Q^2/M)=2*x_cg*dQ-x_cg^2*dM.
    axial_origin_derivative = (
        radii**2 / 4
        + np.array(t.bottom) ** 2
        + 2 * np.array(t.bottom) * k * liquid
        + (k * liquid) ** 2
    )
    transverse = axial_origin_derivative - 2 * cg * (np.array(t.bottom) + k * liquid) + cg**2
    dmain = np.array([fractions @ (radii**2 / 2), fractions @ transverse, fractions @ transverse])
    gas_trans = t.rcs_gas_radius**2 / 4 + t.rcs_gas_length**2 / 12 + (t.rcs_gas_x - cg) ** 2
    dgas = np.array([t.rcs_gas_radius**2 / 2, gas_trans, gas_trans])
    return -main_flow * dmain - gas_flow * dgas


def coupled(cfg, times, command, deployment, rtol=1e-13):
    """Matrix/Euler/DOP853 reference, ideal constant inputs and no fuel exhaustion.

    Includes gravity, aero, fins, backpressure, gas use, I-dot and jet damping.
    RCS uses averaged mode so this smooth reference does not cross PWM switches.
    """
    cfg = _constants(cfg)
    if cfg.plant.actuator.model != "ideal" or cfg.plant.rcs.mode != "averaged":
        raise ValueError("coupled reference requires ideal actuation and averaged RCS")
    ic, v = cfg.scenario.ic, cfg.plant.vehicle
    lookup = body_table(cfg)
    matrix = Rotation.from_euler(
        "ZYX", [ic.yaw_deg, ic.pitch_deg, ic.roll_deg], degrees=True
    ).as_matrix()
    initial = np.r_[ic.r_ned, ic.v_ned, matrix.ravel(), ic.w_body, ic.m, ic.rcs_fuel]
    e = v.engines
    n = e.n_engines
    u = np.clip(command["throttle"], 0, 1)
    phi = np.clip((u - (e.throttle_min - 0.1)) / 0.1, 0, 1)
    phi = phi**2 * (3 - 2 * phi)
    positions = np.zeros((n, 3))
    positions[:, 0] = e.x_eng
    if n == 9:
        z = e.ring_radius * np.exp(2j * np.pi * np.arange(8) / 8)
        positions[1:, 1], positions[1:, 2] = z.real, z.imag
    elif n != 1:
        raise ValueError("unsupported engine layout")
    pitch, yaw = np.clip(
        np.array(command["gimbal"])[0],
        -v.actuator.tvc_deflection_max_rad,
        v.actuator.tvc_deflection_max_rad,
    )
    directions = np.tile([1.0, 0, 0], (n, 1))
    directions[0] = Rotation.from_euler("ZY", [yaw, pitch]).apply([1, 0, 0])
    flow = float(np.sum(phi * u * e.thrust_vac / (e.isp * G0)))

    def rhs(time, y):
        r, velocity, C, w, mass, gas = y[:3], y[3:6], y[6:15].reshape(3, 3), y[15:18], y[18], y[19]
        if mass <= v.mass.m_dry + gas or gas <= 0:
            raise ValueError("coupled experiment must avoid fuel exhaustion")
        altitude = -r[2]
        env = environment(cfg, altitude)
        cg, inertia = mass_properties(v, mass, gas)
        air = C.T @ velocity
        ae = body(cfg, lookup, air, w, cg, altitude)
        fi = fins(cfg, air, w, np.array(command["fin"]), cg, altitude)
        reaction = rcs(cfg, command["rcs"], time, cg)
        thrusts = (phi * np.maximum(u * e.thrust_vac - env["pressure"] * e.nozzle_exit_area, 0))[
            :, None
        ] * directions
        force = thrusts.sum(axis=0) + ae["force"] + deployment * fi["force"] + reaction["force"]
        moment = (
            np.cross(positions - [cg, 0, 0], thrusts).sum(axis=0)
            + ae["moment"]
            + deployment * fi["moment"]
            + reaction["moment"]
        )
        moment -= flow * (v.geom.x_eng - cg) ** 2 * np.array([0, w[1], w[2]])
        idot = inertia_rate(v, mass, gas, flow, reaction["flow"])
        wdot = np.linalg.solve(inertia, moment - np.cross(w, inertia @ w) - idot * w)
        wx, wy, wz = w
        skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
        return np.r_[
            velocity,
            env["gravity_ned"] + C @ force / mass,
            (C @ skew).ravel(),
            wdot,
            -flow - reaction["flow"],
            -reaction["flow"],
        ]

    sol = solve_ivp(
        rhs,
        (0, times[-1]),
        initial,
        method="DOP853",
        t_eval=times,
        rtol=rtol,
        atol=rtol * 0.01,
        max_step=0.01,
    )
    if not sol.success:
        raise RuntimeError(sol.message)
    y = sol.y.T
    q = Rotation.from_matrix(y[:, 6:15].reshape(-1, 3, 3)).as_quat(scalar_first=True)
    return dict(
        r_ned=y[:, :3], v_ned=y[:, 3:6], q_bn=q, w_body=y[:, 15:18], m=y[:, 18], rcs_fuel=y[:, 19]
    )
