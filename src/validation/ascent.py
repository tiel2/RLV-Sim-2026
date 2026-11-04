"""Independent attached-stack reference and Newton–Euler balance audit.

Reference uses config constants, distributed-mass quadrature, rotation matrices,
analytic inertia derivatives and DOP853. No production RHS or flight data is used.
The recorded-load balance audit is complementary, not an independent force model.
"""

import numpy as np
from scipy.integrate import cumulative_trapezoid, solve_ivp
from scipy.spatial.transform import Rotation

from validation.references import G0, attitude_error, mass_properties
from validation.vehicle_references import (
    _constants,
    body,
    body_table,
    environment,
    fins,
    inertia_rate,
)


def reference(cfg, times, *, rtol=1e-10, allow_neutral_fins=False):
    """Prescribed realistic nine-engine dynamics; recovery allows neutral exposed fins.

    Initial state and commands come from configuration, never native output. Reference
    time samples may end at native contact, but no contact constraint is applied here.
    """
    cfg = _constants(cfg)
    v, ic = cfg.plant.vehicle, cfg.scenario.ic
    if (
        cfg.environment.earth.model not in {"fixed", "rotating"}
        or cfg.environment.wind.model != "none"
        or cfg.plant.slosh.model != "none"
        or cfg.plant.actuator.model != "realistic"
        or v.massprops_model != "dual_tank"
    ):
        raise ValueError(
            "ascent reference requires fixed or rotating/no-wind/no-slosh realistic dual-tank model"
        )
    if (
        cfg.fault.engine_out_idx is not None
        or not np.allclose(cfg.fault.thrust_scale, 1)
        or not np.allclose(cfg.fault.tvc_bias, 0)
    ):
        raise ValueError("ascent reference requires healthy unbiased engines")
    if not np.allclose(v.mass.cg_lateral, 0) or not np.allclose(v.mass.inertia_products, 0):
        raise ValueError("ascent reference requires coaxial principal axes")
    e, a = v.engines, v.actuator
    n = e.n_engines
    slots = np.flatnonzero(e.gimbal_mask)
    ng = len(slots)
    if n != 9:
        raise ValueError("ascent reference currently covers the nine-engine stack")
    positions = np.zeros((n, 3))
    positions[:, 0] = e.x_eng
    theta = np.arange(8) * np.pi / 4
    positions[1:, 1:] = e.ring_radius * np.column_stack([np.cos(theta), np.sin(theta)])
    lookup = body_table(cfg)
    C = Rotation.from_euler(
        "ZYX", [ic.yaw_deg, ic.pitch_deg, ic.roll_deg], degrees=True
    ).as_matrix()
    rotating = cfg.environment.earth.model == "rotating"
    radius = cfg.environment.gravity.radius
    origin = np.array([0.0, 0.0, -radius])
    latitude = np.deg2rad(cfg.environment.earth.latitude_deg)
    omega = (
        cfg.environment.earth.rotation_rate_rad_s
        * np.array([np.cos(latitude), 0, -np.sin(latitude)])
        if rotating
        else np.zeros(3)
    )
    # Rotating cases are independently propagated in an INERTIAL frame. No
    # production geometry or fictitious-acceleration routines are reused.
    position0 = np.asarray(ic.r_ned) + origin if rotating else ic.r_ned
    velocity0 = np.asarray(ic.v_ned) + np.cross(omega, position0) if rotating else ic.v_ned
    rates0 = np.asarray(ic.w_body) + C.T @ omega if rotating else ic.w_body
    # r, v, C, omega, mass, throttle[N], gimbal[Ng,2], gimbal_rate[Ng,2]
    initial = np.r_[
        position0,
        velocity0,
        C.ravel(),
        rates0,
        ic.m,
        np.broadcast_to(ic.throttle0, n),
        np.zeros(4 * ng),
    ]
    gas = ic.rcs_fuel
    dry = v.mass.m_dry + getattr(getattr(v, "attached_component", None), "mass", 0)
    current_u, current_g = np.zeros(n), np.zeros((ng, 2))
    deployment = 1
    result = np.empty((len(times), len(initial)))
    rows = cfg.scenario.inputs
    for k, row in enumerate(rows):
        start = row.t
        if start >= times[-1]:
            break
        end = min(rows[k + 1].t if k + 1 < len(rows) else times[-1], times[-1])
        if hasattr(row, "throttle"):
            current_u = np.broadcast_to(row.throttle, (n,)).copy()
        if hasattr(row, "gimbal"):
            current_g = np.broadcast_to(row.gimbal, (ng, 2)).copy()
        deployment = getattr(row, "fin_deployment", deployment)
        if (
            (deployment != 0 and not allow_neutral_fins)
            or not np.allclose(getattr(row, "rcs", 0), 0)
            or not np.allclose(getattr(row, "fin", 0), 0)
            or (allow_neutral_fins and cfg.plant.aero.fins.incremental)
        ):
            raise ValueError("reference requires neutral fins/no RCS and declared fin exposure")

        def rhs(t, y, current_u=current_u, current_g=current_g, deployment=deployment):
            r, vel, C, w, mass = y[:3], y[3:6], y[6:15].reshape(3, 3), y[15:18], y[18]
            u = y[19 : 19 + n]
            g = y[19 + n : 19 + n + 2 * ng].reshape(ng, 2)
            rate = y[19 + n + 2 * ng :].reshape(ng, 2)
            if mass <= dry + gas:
                raise ValueError("reference must not cross fuel exhaustion")
            altitude = np.linalg.norm(r) - radius if rotating else -r[2]
            relative_velocity = vel - np.cross(omega, r) if rotating else vel
            relative_rates = w - C.T @ omega if rotating else w
            env = environment(cfg, altitude)
            cg, I = mass_properties(v, mass, gas)
            ae = body(cfg, lookup, C.T @ relative_velocity, relative_rates, cg, altitude)
            fi = (
                fins(cfg, C.T @ relative_velocity, relative_rates, np.zeros(4), cg, altitude)
                if deployment
                else dict(force=np.zeros(3), moment=np.zeros(3))
            )
            duty = np.clip(u, 0, 1)
            ignition = np.clip((duty - e.throttle_min + 0.1) / 0.1, 0, 1)
            ignition = ignition**2 * (3 - 2 * ignition)
            flow = np.sum(ignition * duty) * e.thrust_vac / (G0 * e.isp)
            axes = np.tile([1.0, 0, 0], (n, 1))
            axes[slots] = Rotation.from_euler("ZY", g[:, ::-1]).apply([1, 0, 0])
            thrust = (
                ignition * np.maximum(duty * e.thrust_vac - env["pressure"] * e.nozzle_exit_area, 0)
            )[:, None] * axes
            force = thrust.sum(axis=0) + ae["force"] + deployment * fi["force"]
            moment = (
                np.cross(positions - [cg, 0, 0], thrust).sum(axis=0)
                + ae["moment"]
                + deployment * fi["moment"]
            )
            moment -= flow * (e.x_eng - cg) ** 2 * np.array([0, w[1], w[2]])
            idot = inertia_rate(v, mass, gas, flow, 0)
            wdot = np.linalg.solve(I, moment - np.cross(w, I @ w) - idot * w)
            wx, wy, wz = w
            skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
            du = a.throttle_rate_max * np.tanh(
                (current_u - u) / (a.throttle_tau * a.throttle_rate_max)
            )
            desired = a.tvc_rate_max_rad_s * np.tanh(
                a.tvc_omega * (current_g - g) / (2 * a.tvc_zeta * a.tvc_rate_max_rad_s)
            )
            ddg = 2 * a.tvc_zeta * a.tvc_omega * (desired - rate)
            return np.r_[
                vel,
                (
                    -cfg.environment.gravity.mu * r / np.linalg.norm(r) ** 3
                    if rotating
                    else env["gravity_ned"]
                )
                + C @ force / mass,
                (C @ skew).ravel(),
                wdot,
                -flow,
                du,
                rate.ravel(),
                ddg.ravel(),
            ]

        if k == 0 and getattr(ic, "launch_pad", False) and rhs(start, initial)[5] >= 0:
            raise ValueError("independent ascent reference starts at liftoff, after pad support")
        sol = solve_ivp(
            rhs,
            (start, end),
            initial,
            method="DOP853",
            rtol=rtol,
            atol=rtol * 0.01,
            max_step=0.2,
            dense_output=True,
        )
        if not sol.success:
            raise RuntimeError(sol.message)
        selected = (times >= start - 1e-9) & (times <= end + 1e-9)
        result[selected] = sol.sol(np.clip(times[selected], start, end)).T
        initial = sol.y[:, -1]
    matrices = result[:, 6:15].reshape(-1, 3, 3)
    if rotating:
        Q = Rotation.from_rotvec(np.asarray(times)[:, None] * omega).as_matrix()
        matrices = np.einsum("nji,njk->nik", Q, matrices)
        position = np.einsum("nji,nj->ni", Q, result[:, :3])
        result[:, 3:6] = np.einsum("nji,nj->ni", Q, result[:, 3:6]) - np.cross(omega, position)
        result[:, :3] = position - origin
        result[:, 15:18] -= np.einsum("nji,j->ni", matrices, omega)
    q = Rotation.from_matrix(matrices).as_quat(scalar_first=True)
    return dict(
        r_ned=result[:, :3],
        v_ned=result[:, 3:6],
        q_bn=q,
        w_body=result[:, 15:18],
        m=result[:, 18],
        throttle=result[:, 19 : 19 + n],
    )


def compare(actual, expected):
    errors = {
        key: float(np.max(np.linalg.norm(getattr(actual, key) - expected[key], axis=1)))
        for key in ("r_ned", "v_ned", "w_body")
    }
    errors["q_bn"] = float(np.max(attitude_error(actual.q_bn, expected["q_bn"])))
    errors["m"] = float(np.max(np.abs(actual.m - expected["m"])))
    return errors


def balances(result):
    """Inertial open-system balances: d(mv)/dt=F+v*dm/dt, d(C I w)/dt=C M_effective.

    Jet angular-momentum flux is already included in M_cg_body. Gravity and pad
    reactions are added explicitly. Diagnostic quadrature is not a force-model proof.
    """
    aux = result.aux["per_tick"]
    series = lambda key: np.stack([a[key] for a in aux])
    C = Rotation.from_quat(result.q_bn, scalar_first=True).as_matrix()
    earth = result.aux.get("earth", {})
    rotating = earth.get("model") == "rotating"
    velocity = result.v_ned
    rates = result.w_body
    Q = np.broadcast_to(np.eye(3), C.shape)
    if rotating:
        omega = np.asarray(earth["omega_reference_rad_s"])
        Q = Rotation.from_rotvec(result.t[:, None] * omega).as_matrix()
        position = result.r_ned + [0, 0, -earth["radius_m"]]
        velocity = np.einsum("nij,nj->ni", Q, velocity + np.cross(omega, position))
        rates = series("omega_inertial_body")
        C = Q @ C
    body_to_ned = lambda vector: np.einsum("nij,nj->ni", C, vector)
    flow = series("mdot")
    force = body_to_ned(series("F_body_total"))
    force += result.m[:, None] * np.einsum("nij,nj->ni", Q, series("gravity_ned"))
    momentum = result.m[:, None] * velocity
    torque = body_to_ned(series("M_cg_body"))
    angular = body_to_ned(series("inertia_diag") * rates)
    integral = lambda values: cumulative_trapezoid(values, result.t, axis=0, initial=0)

    # Integrate the support reaction only up to its exact native release epoch.
    # The initial sample precedes release; smearing that jump over one log interval
    # would introduce a spurious half-interval reaction impulse.
    def supported_integral(load, reaction):
        dt = np.diff(result.t)
        pending = series("launch_pending").astype(bool)
        release = series("liftoff_time")[1:]
        held_dt = np.where(pending[1:], dt, np.clip(release - result.t[:-1], 0, dt))
        held_dt = np.where(pending[:-1], held_dt, 0)
        boundary = load[:-1] + (load[1:] - load[:-1]) * (held_dt / dt)[:, None]
        reaction_end = np.where(pending[1:, None], reaction[1:], -boundary)
        impulse = 0.5 * (reaction[:-1] + reaction_end) * held_dt[:, None]
        return integral(load) + np.vstack([np.zeros(3), np.cumsum(impulse, axis=0)])

    dm = result.m - result.m[0] + integral(flow)
    dp = (
        momentum
        - momentum[0]
        - supported_integral(force, body_to_ned(series("F_pad_body")))
        + integral(flow[:, None] * velocity)
    )
    dh = angular - angular[0] - supported_integral(torque, body_to_ned(series("M_pad_body")))
    return {
        "mass_kg": float(np.max(np.abs(dm))),
        "linear_momentum_kg_m_s": float(np.max(np.linalg.norm(dp, axis=1))),
        "angular_momentum_kg_m2_s": float(np.max(np.linalg.norm(dh, axis=1))),
    }
