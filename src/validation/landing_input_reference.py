"""Independent ideal single-engine plant integration using recorded held commands.

Initial state is configured; all states propagate continuously without resetting to logs.
An inertial-frame DOP853 reference uses rotation matrices and independent force/mass
functions. It verifies the plant for the recorded inputs, not the feedback controller.
Contact detection is excluded: integrate to the archived final native sample epoch.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from validation.references import G0, mass_properties
from validation.vehicle_references import (
    _constants,
    body,
    body_table,
    environment,
    fins,
    inertia_rate,
)


def reference(config, actuators, times, *, rtol=1e-10):
    cfg = _constants(config)
    v, ic = cfg.plant.vehicle, cfg.scenario.ic
    e, gas = v.engines, ic.rcs_fuel
    if not (
        cfg.plant.actuator.model == "ideal"
        and e.n_engines == 1
        and e.layout == "single"
        and e.gimbal_mask == [1]
        and cfg.environment.earth.model == "rotating"
        and cfg.environment.gravity.model == "inverse_square"
        and cfg.environment.earth.include_differential_centrifugal
        and cfg.environment.wind.model == "none"
        and cfg.plant.slosh.model == "none"
        and v.massprops_model == "dual_tank"
        and cfg.fault.engine_out_idx is None
        and np.allclose(cfg.fault.thrust_scale, 1)
        and np.allclose(cfg.fault.tvc_bias, 0)
        and np.allclose(cfg.fault.fin_lock, 0)
        and np.allclose(v.mass.cg_lateral, 0)
        and np.allclose(v.mass.inertia_products, 0)
        and np.allclose(actuators["held_rcs"], 0)
        and not ic.launch_pad
    ):
        raise ValueError("requires healthy ideal rotating single-engine/no-wind/no-slosh/no-RCS")
    times = np.asarray(times)
    starts, ends = actuators["t_start"], actuators["t_end"]
    if not (
        times[0] == starts[0] == 0
        and np.all(np.diff(times) > 0)
        and np.all(ends > starts)
        and np.allclose(starts[1:], ends[:-1], atol=1e-10, rtol=0)
        and abs(times[-1] - ends[-1]) < 1e-10
    ):
        raise ValueError("reference requires complete contiguous held commands")
    C = Rotation.from_euler(
        "ZYX", [ic.yaw_deg, ic.pitch_deg, ic.roll_deg], degrees=True
    ).as_matrix()
    radius = cfg.environment.gravity.radius
    origin = np.array([0.0, 0, -radius])
    lat = np.deg2rad(cfg.environment.earth.latitude_deg)
    omega = cfg.environment.earth.rotation_rate_rad_s * np.array([np.cos(lat), 0, -np.sin(lat)])
    r = np.asarray(ic.r_ned) + origin
    y = np.r_[
        r,
        np.asarray(ic.v_ned) + np.cross(omega, r),
        C.ravel(),
        np.asarray(ic.w_body) + C.T @ omega,
        ic.m,
    ]
    lookup = body_table(cfg)
    out = np.full((len(times), 19), np.nan)
    out[0] = y
    a = v.actuator
    for i, (start, end) in enumerate(zip(starts, ends, strict=True)):
        u = float(np.clip(actuators["held_throttle"][i, 0], 0, 1))
        u = u if u >= e.throttle_min else 0.0
        g = np.clip(
            actuators["held_gimbal"][i, 0], -a.tvc_deflection_max_rad, a.tvc_deflection_max_rad
        )
        angles = np.clip(
            actuators["held_fin"][i], -a.fin_deflection_max_rad, a.fin_deflection_max_rad
        )
        axis = Rotation.from_euler("ZY", g[::-1]).apply([1, 0, 0])
        flow = u * e.thrust_vac / (G0 * e.isp)

        def rhs(t, state, u=u, angles=angles, axis=axis, flow=flow):
            r, vel, C = state[:3], state[3:6], state[6:15].reshape(3, 3)
            w, mass = state[15:18], state[18]
            if mass <= v.mass.m_dry + gas:
                raise ValueError("reference does not cover fuel exhaustion")
            h = np.linalg.norm(r) - radius
            relative_v, relative_w = vel - np.cross(omega, r), w - C.T @ omega
            env = environment(cfg, h)
            cg, I = mass_properties(v, mass, gas)
            ae = body(cfg, lookup, C.T @ relative_v, relative_w, cg, h)
            fi = fins(cfg, C.T @ relative_v, relative_w, angles, cg, h)
            thrust = max(u * e.thrust_vac - env["pressure"] * e.nozzle_exit_area, 0) * axis
            if u == 0:
                thrust = np.zeros(3)
            force = thrust + ae["force"] + fi["force"]
            torque = np.cross([e.x_eng - cg, 0, 0], thrust) + ae["moment"] + fi["moment"]
            torque -= flow * (e.x_eng - cg) ** 2 * np.array([0, w[1], w[2]])
            idot = inertia_rate(v, mass, gas, flow, 0)
            wdot = np.linalg.solve(I, torque - np.cross(w, I @ w) - idot * w)
            wx, wy, wz = w
            skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
            return np.r_[
                vel,
                -cfg.environment.gravity.mu * r / np.linalg.norm(r) ** 3 + C @ force / mass,
                (C @ skew).ravel(),
                wdot,
                -flow,
            ]

        sol = solve_ivp(
            rhs,
            (start, end),
            y,
            method="DOP853",
            rtol=rtol,
            atol=rtol * 0.01,
            dense_output=True,
            max_step=0.01,
        )
        if not sol.success:
            raise RuntimeError(sol.message)
        selected = (times > start + 1e-10) & (times <= end + 1e-10)
        if np.any(selected):
            out[selected] = sol.sol(np.clip(times[selected], start, end)).T
        y = sol.y[:, -1]
    if not np.all(np.isfinite(out)):
        raise ValueError("reference output has uncovered epochs")
    Q = Rotation.from_rotvec(times[:, None] * omega).as_matrix()
    C = np.einsum("nji,njk->nik", Q, out[:, 6:15].reshape(-1, 3, 3))
    position = np.einsum("nji,nj->ni", Q, out[:, :3])
    velocity = np.einsum("nji,nj->ni", Q, out[:, 3:6]) - np.cross(omega, position)
    return dict(
        r_ned=position - origin,
        v_ned=velocity,
        q_bn=Rotation.from_matrix(C).as_quat(scalar_first=True),
        w_body=out[:, 15:18] - np.einsum("nji,j->ni", C, omega),
        m=out[:, 18],
    )
