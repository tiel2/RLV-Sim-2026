"""Analytical anchors for explicitly restricted physics experiments.

These checks establish implementation consistency, not flight-data validation.
Tolerances come from the scenario and are included in every result.
"""

import json
from pathlib import Path

import numpy as np

from core.frames import dcm_from_quat


def validate_episode(result, bundle, cfg):
    spec = cfg.scenario.get("validation", {})
    kind = str(spec.get("kind", "none"))
    report = {
        "kind": kind,
        "status": "not_requested",
        "checks": {},
        "scope": "numerical/analytical verification; not flight-data validation",
    }
    reference = {"t": result.t}
    if kind == "none":
        return report, reference
    p = bundle.params
    t = result.t - result.t[0]
    r, v, m = result.r_ned, result.v_ned, result.m
    aux = result.aux["per_tick"]

    def require(condition, message):
        if not condition:
            raise ValueError(f"{kind} is not applicable: {message}")

    def check(name, value, tolerance, unit):
        report["checks"][name] = {
            "error": float(value),
            "tolerance": float(tolerance),
            "unit": unit,
            "passed": bool(np.isfinite(value) and value <= tolerance),
        }

    require(bundle.meta["earth_model"] == "fixed", "requires fixed local frame")
    if kind != "constant_drag":
        require(bundle.meta["atmosphere_model"] == "vacuum", "requires vacuum atmosphere")
    require(bundle.meta["slosh_model"] == "none", "requires slosh disabled")
    require(np.allclose([a["rcs_mdot"] for a in aux], 0), "requires RCS off")
    check(
        "quaternion_norm",
        np.max(np.abs(np.linalg.norm(result.q_bn, axis=1) - 1)),
        1e-12,
        "dimensionless",
    )
    if kind in ("freefall_uniform", "inverse_square_energy", "free_rotation"):
        require(np.allclose(m, m[0], atol=1e-8, rtol=0), "requires constant mass")
        require(np.allclose([a["thrust_mag"] for a in aux], 0), "requires engines off")
    if kind == "freefall_uniform":
        require(not bool(p.gravity.inverse_square), "requires uniform gravity")
        g = np.asarray(p.g_ned)
        reference["r_ned"] = r[0] + t[:, None] * v[0] + 0.5 * t[:, None] ** 2 * g
        reference["v_ned"] = v[0] + t[:, None] * g
        check(
            "position",
            np.max(np.linalg.norm(r - reference["r_ned"], axis=1)),
            spec["position_atol_m"],
            "m",
        )
        check(
            "velocity",
            np.max(np.linalg.norm(v - reference["v_ned"], axis=1)),
            spec["velocity_atol_mps"],
            "m/s",
        )
    elif kind == "constant_drag":
        require(not bool(p.gravity.inverse_square), "requires uniform gravity")
        require(np.allclose(m, m[0], rtol=0, atol=1e-8), "requires constant mass")
        require(np.allclose([a["thrust_mag"] for a in aux], 0), "requires engines off")
        require(float(p.aero.fin_area) == 0, "requires fin forces disabled")
        require(
            np.allclose(result.w_body, 0, atol=1e-10) and np.allclose(v[:, :2], 0, atol=1e-9),
            "requires vertical nonrotating flight",
        )
        cd = np.array([a["aero_cd"] for a in aux])
        rho = np.array([a["rho"] for a in aux])
        require(np.allclose(cd, cd[0], atol=1e-12, rtol=0), "requires constant CD")
        require(
            np.max(np.abs(rho - rho[0])) / rho[0] < 1e-7, "requires effectively constant density"
        )
        g = float(p.g_ned[2])
        require(g > 0 and cd[0] > 0 and rho[0] > 0, "requires positive gravity and drag")
        terminal = np.sqrt(2 * m[0] * g / (rho[0] * float(p.geom.s_ref) * cd[0]))
        require(0 <= v[0, 2] < terminal, "anchor uses subterminal initial descent")
        offset = np.arctanh(v[0, 2] / terminal)
        arg = g * t / terminal + offset
        reference["v_ned"] = np.tile(v[0], (len(t), 1))
        reference["v_ned"][:, 2] = terminal * np.tanh(arg)
        reference["r_ned"] = np.tile(r[0], (len(t), 1))
        reference["r_ned"][:, 2] += (
            terminal**2 / g * (np.log(np.cosh(arg)) - np.log(np.cosh(offset)))
        )
        report["terminal_speed_mps"] = float(terminal)
        check(
            "position",
            np.max(np.linalg.norm(r - reference["r_ned"], axis=1)),
            spec["position_atol_m"],
            "m",
        )
        check(
            "velocity",
            np.max(np.linalg.norm(v - reference["v_ned"], axis=1)),
            spec["velocity_atol_mps"],
            "m/s",
        )
    elif kind == "inverse_square_energy":
        require(bool(p.gravity.inverse_square), "requires inverse-square gravity")
        energy = 0.5 * np.sum(v * v, axis=1) - float(p.gravity.mu) / (
            float(p.gravity.radius) - r[:, 2]
        )
        reference["specific_energy"] = energy
        reference["expected_specific_energy"] = np.full_like(t, energy[0])
        check(
            "specific_energy_relative",
            np.max(np.abs(energy - energy[0])) / abs(energy[0]),
            spec["relative_tolerance"],
            "dimensionless",
        )
    elif kind == "free_rotation":
        inertia = np.asarray(aux[0]["inertia_diag"])
        products = np.asarray(p.mass.inertia_products)
        I = np.diag(inertia)
        I[0, 1] = I[1, 0] = products[0]
        I[0, 2] = I[2, 0] = products[1]
        I[1, 2] = I[2, 1] = products[2]
        H = np.array(
            [dcm_from_quat(q) @ I @ w for q, w in zip(result.q_bn, result.w_body, strict=True)]
        )
        energy = 0.5 * np.einsum("ni,ij,nj->n", result.w_body, I, result.w_body)
        require(np.linalg.norm(H[0]) > 0 and energy[0] > 0, "requires nonzero angular velocity")
        reference.update(
            angular_momentum=H,
            rotational_energy=energy,
            expected_angular_momentum=np.tile(H[0], (len(t), 1)),
            expected_rotational_energy=np.full_like(t, energy[0]),
        )
        tol = spec["relative_tolerance"]
        check(
            "angular_momentum_relative",
            np.max(np.linalg.norm(H - H[0], axis=1)) / np.linalg.norm(H[0]),
            tol,
            "dimensionless",
        )
        check(
            "rotational_energy_relative",
            np.max(np.abs(energy - energy[0])) / energy[0],
            tol,
            "dimensionless",
        )
    elif kind in ("rocket_equation", "mass_depletion"):
        require(
            not bool(p.gravity.inverse_square) and np.allclose(p.g_ned, 0), "requires zero gravity"
        )
        require(np.allclose(result.w_body, 0, atol=1e-10), "requires straight nonrotating flight")
        require(
            bundle.meta["actuator_model"] == "ideal", "requires ideal actuation for flow anchor"
        )
        require(result.aux.get("input_mode") == "prescribed", "requires prescribed input")
        require(len(cfg.scenario.inputs) == 1, "requires a constant command")
        throttle = np.asarray(
            result.actuators.held.throttle[0]
            if result.actuators is not None
            else cfg.scenario.inputs[0].throttle,
            dtype=float,
        )
        throttle = np.broadcast_to(throttle, p.engines.thrust_vac.shape)
        on = throttle > 0
        require(np.any(on), "requires an active engine")
        require(
            np.all(throttle[on] >= np.asarray(p.engines.throttle_min)[on]),
            "requires firing above minimum throttle",
        )
        require(
            np.allclose(p.engines.isp[on], p.engines.isp[on][0]),
            "requires common active-engine Isp",
        )
        require(
            np.allclose(p.engines.thrust_scale, 1) and np.allclose(p.engines.engine_on, 1),
            "requires healthy engines",
        )
        require(
            np.allclose(result.tvc, 0)
            and np.allclose(p.engines.cant, 0)
            and np.allclose(p.engines.tvc_bias, 0),
            "requires axial thrust",
        )
        isp = float(p.engines.isp[on][0])
        axis = dcm_from_quat(result.q_bn[0]) @ np.array([1.0, 0.0, 0.0])
        reference["v_ned"] = v[0] + (isp * 9.80665 * np.log(m[0] / m))[:, None] * axis
        dv_error = np.max(np.linalg.norm(v - reference["v_ned"], axis=1))
        check("rocket_velocity", dv_error, float(spec.get("velocity_atol_mps", 1e-4)), "m/s")
        flow = float(
            np.sum(
                throttle * np.asarray(p.engines.thrust_vac) / (np.asarray(p.engines.isp) * 9.80665)
            )
        )
        floor = float(p.mass.m_dry) + float(aux[0]["rcs_fuel"])
        reference["mass"] = np.maximum(m[0] - flow * t, floor)
        check("mass_budget", np.max(np.abs(m - reference["mass"])), spec["mass_atol_kg"], "kg")
        if kind == "mass_depletion":
            depletion = (m[0] - floor) / flow
            report["expected_depletion_s"] = depletion
            check("remaining_main_fuel", aux[-1]["main_propellant"], spec["mass_atol_kg"], "kg")
            check("final_thrust", aux[-1]["thrust_mag"], 1e-8, "N")
            check(
                "rcs_reserve",
                abs(float(aux[-1]["rcs_fuel"]) - float(aux[0]["rcs_fuel"])),
                1e-8,
                "kg",
            )
            check(
                "continued_after_depletion",
                0.0 if t[-1] > depletion + 1.0 else 1.0,
                0.0,
                "boolean_error",
            )
    else:
        raise ValueError(f"unknown validation kind {kind!r}")
    report["status"] = "passed" if all(x["passed"] for x in report["checks"].values()) else "failed"
    return report, reference


def write_validation(report, reference, directory, *, plot=True):
    out = Path(directory)
    (out / "validation.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    if report["status"] == "not_requested":
        return
    np.savez_compressed(out / "reference.npz", **reference)
    if plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        checks = report["checks"]
        fig, ax = plt.subplots(figsize=(9, 4))
        names = list(checks)
        ratios = [
            max(checks[k]["error"] / max(checks[k]["tolerance"], 1e-30), 1e-16) for k in names
        ]
        ax.bar(names, ratios)
        ax.axhline(1, color="red", linestyle="--", label="tolerance")
        ax.set(yscale="log", ylabel="error / tolerance", title=report["kind"])
        ax.tick_params(axis="x", rotation=20)
        ax.legend()
        fig.tight_layout()
        (out / "plots").mkdir(exist_ok=True)
        fig.savefig(out / "plots" / "validation.png", dpi=120)
        plt.close(fig)
