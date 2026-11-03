"""Independent local integrations of recorded mission inputs (not controller validation).

Uses configuration-only force models, cylinder quadrature, analytic inertia rates,
rotation matrices and DOP853. PWM changes and command holds are explicit boundaries.
No production RHS is called. Each selected 1 s window starts from a recorded full state.
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
    inertia_rate,
    rcs,
)


def replay_window(config, rows, rtol=1e-12):
    cfg = _constants(config)
    if (
        cfg.plant.actuator.model != "realistic"
        or cfg.plant.slosh.model != "none"
        or cfg.environment.wind.model != "none"
        or cfg.environment.earth.model != "fixed"
    ):
        raise ValueError("reference requires realistic actuators, no slosh/wind, fixed Earth")
    v = cfg.plant.vehicle
    e, a = v.engines, v.actuator
    n = e.n_engines
    lookup = body_table(cfg)
    s = rows[0]["state"]
    y = np.r_[
        s["r_ned"],
        s["v_ned"],
        Rotation.from_quat(s["q_bn"], scalar_first=True).as_matrix().ravel(),
        s["w_body"],
        s["m"],
        s["rcs_fuel"],
        s["throttle_act"],
        np.asarray(s["tvc"]).ravel(),
        np.asarray(s["tvc_rate"]).ravel(),
    ]
    positions = np.zeros((n, 3))
    positions[:, 0] = e.x_eng
    angles = np.arange(n - 1) * 2 * np.pi / (n - 1)
    positions[1:, 1:] = e.ring_radius * np.column_stack([np.cos(angles), np.sin(angles)])
    history = [y.copy()]
    for current, next_row in zip(rows[:-1], rows[1:], strict=True):
        command = current["command"]
        start, end = current["state"]["t"], next_row["state"]["t"]
        # Reproduce the native integration grid's held PWM activity, but integrate
        # the continuous equations independently between actual switches.
        steps = round((end - start) / cfg.plant_dt)
        times = start + np.arange(steps + 1) * cfg.plant_dt
        # Native sessions use an integer integration grid, also across separation.
        reactions = [rcs(cfg, command.rcs, t, 0) for t in times[:-1]]
        boundaries = (
            [0]
            + [
                i
                for i in range(1, steps)
                if not np.array_equal(reactions[i]["on"], reactions[i - 1]["on"])
            ]
            + [steps]
        )
        for lo, hi in zip(boundaries[:-1], boundaries[1:], strict=True):
            reaction = reactions[lo]  # symmetric couples: net force zero, moment independent of CG

            def rhs(_, y, reaction=reaction, command=command):
                r, vel, C, w = y[:3], y[3:6], y[6:15].reshape(3, 3), y[15:18]
                mass, gas = y[18:20]
                u = y[20 : 20 + n]
                g = y[20 + n : 20 + 3 * n].reshape(n, 2)
                rate = y[20 + 3 * n :].reshape(n, 2)
                env = environment(cfg, -r[2])
                cg, I = mass_properties(v, mass, gas)
                ae = body(cfg, lookup, C.T @ vel, w, cg, -r[2])
                duty = np.clip(u, 0, 1)
                ignition = np.clip((duty - e.throttle_min + 0.1) / 0.1, 0, 1)
                ignition = ignition**2 * (3 - 2 * ignition)
                flow = np.sum(ignition * duty) * e.thrust_vac / (G0 * e.isp)
                axes = Rotation.from_euler("ZY", g[:, ::-1]).apply([1, 0, 0])
                thrust = (
                    ignition
                    * np.maximum(duty * e.thrust_vac - env["pressure"] * e.nozzle_exit_area, 0)
                )[:, None] * axes
                force = thrust.sum(axis=0) + ae["force"] + reaction["force"]
                moment = (
                    np.cross(positions - [cg, 0, 0], thrust).sum(axis=0)
                    + ae["moment"]
                    + reaction["moment"]
                )
                moment -= flow * (e.x_eng - cg) ** 2 * np.array([0, w[1], w[2]])
                idot = inertia_rate(v, mass, gas, flow, reaction["flow"])
                wdot = np.linalg.solve(I, moment - np.cross(w, I @ w) - idot * w)
                wx, wy, wz = w
                skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
                target = np.where(command.throttle >= e.throttle_min, command.throttle, 0)
                du = a.throttle_rate_max * np.tanh(
                    (target - u) / (a.throttle_tau * a.throttle_rate_max)
                )
                desired = a.tvc_rate_max_rad_s * np.tanh(
                    a.tvc_omega * (command.gimbal - g) / (2 * a.tvc_zeta * a.tvc_rate_max_rad_s)
                )
                ddg = 2 * a.tvc_zeta * a.tvc_omega * (desired - rate)
                return np.r_[
                    vel,
                    env["gravity_ned"] + C @ force / mass,
                    (C @ skew).ravel(),
                    wdot,
                    -flow - reaction["flow"],
                    -reaction["flow"],
                    du,
                    rate.ravel(),
                    ddg.ravel(),
                ]

            sol = solve_ivp(
                rhs,
                (times[lo], times[hi]),
                y,
                method="DOP853",
                rtol=rtol,
                atol=rtol * 0.01,
                max_step=0.002,
            )
            if not sol.success:
                raise RuntimeError(sol.message)
            y = sol.y[:, -1]
        history.append(y.copy())
    values = np.array(history)
    return dict(
        r_ned=values[:, :3],
        v_ned=values[:, 3:6],
        q_bn=Rotation.from_matrix(values[:, 6:15].reshape(-1, 3, 3)).as_quat(scalar_first=True),
        w_body=values[:, 15:18],
        m=values[:, 18],
        rcs_fuel=values[:, 19],
        throttle=values[:, 20 : 20 + n],
        tvc=values[:, 20 + n : 20 + 3 * n],
    )
