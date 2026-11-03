"""Independent release balances and unpowered booster reference integration."""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation

from validation.references import mass_properties
from validation.vehicle_references import _constants, body, body_table, environment


def event_balances(event):
    """Total momentum of both bodies about the pre-release system CG.

    Include each body's spin and orbital angular momentum. No production frame or
    mass-properties routines are used here.
    """
    before, after, removed = (event[k] for k in ("before", "after", "removed"))
    rot = Rotation.from_quat(before["q_bn"], scalar_first=True).as_matrix()
    origin = np.array(before["r_ned"])
    velocity = np.array(before["v_ned"])
    momentum = np.zeros(3)
    angular = np.zeros(3)
    centroid = np.zeros(3)
    for state in (after, removed):
        displacement = np.array(state["r_ned"]) - origin
        relative_v = np.array(state["v_ned"]) - velocity
        momentum += state["m"] * relative_v
        centroid += state["m"] * displacement
        angular += rot @ (np.array(state["inertia_diag"]) * state["w_body"])
        angular += np.cross(displacement, state["m"] * relative_v)
    angular -= rot @ (np.array(before["inertia_diag"]) * before["w_body"])
    return {
        "mass_kg": abs(before["m"] - after["m"] - removed["m"]),
        "centroid_m": float(np.linalg.norm(centroid) / before["m"]),
        "linear_momentum_kg_m_s": float(np.linalg.norm(momentum)),
        "angular_momentum_kg_m2_s": float(np.linalg.norm(angular)),
        "main_propellant_kg": abs(before["main_propellant"] - after["main_propellant"]),
        "rcs_propellant_kg": abs(before["rcs_fuel"] - after["rcs_fuel"]),
    }


def coast_reference(cfg, initial, times, *, rtol=1e-11):
    """DOP853 rotation-matrix reference for a stowed, unpowered, fixed-Earth coast."""
    cfg = _constants(cfg)
    if (
        cfg.environment.earth.model != "fixed"
        or cfg.environment.wind.model != "none"
        or cfg.plant.slosh.model != "none"
    ):
        raise ValueError("coast reference requires fixed Earth, no wind and no slosh")
    cg, inertia = mass_properties(cfg.plant.vehicle, initial["m"], initial["rcs_fuel"])
    lookup = body_table(cfg)
    rotation = Rotation.from_quat(initial["q_bn"], scalar_first=True).as_matrix()
    y0 = np.r_[initial["r_ned"], initial["v_ned"], rotation.ravel(), initial["w_body"]]

    def rhs(_, y):
        r, velocity, C, w = y[:3], y[3:6], y[6:15].reshape(3, 3), y[15:18]
        env = environment(cfg, -r[2])
        aero = body(cfg, lookup, C.T @ velocity, w, cg, -r[2])
        wx, wy, wz = w
        skew = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
        return np.r_[
            velocity,
            env["gravity_ned"] + C @ aero["force"] / initial["m"],
            (C @ skew).ravel(),
            np.linalg.solve(inertia, aero["moment"] - np.cross(w, inertia @ w)),
        ]

    sol = solve_ivp(
        rhs,
        (times[0], times[-1]),
        y0,
        t_eval=times,
        method="DOP853",
        rtol=rtol,
        atol=rtol * 0.01,
        max_step=0.1,
    )
    if not sol.success:
        raise RuntimeError(sol.message)
    y = sol.y.T
    return dict(
        r_ned=y[:, :3],
        v_ned=y[:, 3:6],
        w_body=y[:, 15:18],
        q_bn=Rotation.from_matrix(y[:, 6:15].reshape(-1, 3, 3)).as_quat(scalar_first=True),
        m=np.full(len(times), initial["m"]),
    )
