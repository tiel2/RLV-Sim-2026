"""Compare restricted vehicle runs against independently computed references."""

import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from validation.references import attitude_error, build_reference, mass_properties


def validate_episode(result, bundle, cfg):
    """Keep the shared-runner API; reference construction never receives ``bundle``."""
    spec = cfg.scenario.get("validation", {})
    kind = str(spec.get("kind", "none"))
    report = {
        "kind": kind,
        "status": "not_requested",
        "checks": {},
        "scope": "numerical/analytical verification; not flight-data validation",
    }
    if kind == "none":
        return report, {"t": result.t}
    reference, method = build_reference(cfg, result.t)
    report["reference"] = method
    aux = result.aux["per_tick"]

    def check(name, error, tolerance, unit):
        values = np.atleast_1d(error).astype(float)
        finite = bool(np.all(np.isfinite(values)))
        peak = float(np.max(np.abs(values))) if finite else None
        report["checks"][name] = {
            "error": peak,
            "tolerance": float(tolerance),
            "unit": unit,
            "rms_error": float(np.sqrt(np.mean(values**2))) if finite else None,
            "final_error": float(abs(values[-1])) if finite else None,
            "passed": bool(finite and peak <= tolerance),
        }

    def compare(key, actual, tolerance, unit, name=None):
        actual = np.asarray(actual)
        expected = reference[key]
        delta = actual - expected
        error = np.linalg.norm(delta, axis=1) if delta.ndim == 2 else np.abs(delta)
        reference["actual_" + key] = actual
        reference["error_" + key] = error
        check(name or key, error, tolerance, unit)

    check(
        "quaternion_norm", np.abs(np.linalg.norm(result.q_bn, axis=1) - 1), 1e-12, "dimensionless"
    )
    compare("r_ned", result.r_ned, spec["position_atol_m"], "m", "position")
    compare("v_ned", result.v_ned, spec["velocity_atol_mps"], "m/s", "velocity")
    compare("mass", result.m, spec.get("mass_atol_kg", 1e-8), "kg", "mass_budget")
    compare("rcs_fuel", [a["rcs_fuel"] for a in aux], 1e-8, "kg", "rcs_reserve")
    angle = attitude_error(result.q_bn, reference["q_bn"])
    reference["actual_q_bn"] = result.q_bn
    reference["error_q_bn"] = angle
    angle_tol, rate_tol = (
        float(spec.get("attitude_atol_rad", 1e-8)),
        float(spec.get("angular_velocity_atol_rad_s", 1e-9)),
    )
    check("attitude", angle, angle_tol, "rad")
    compare("w_body", result.w_body, rate_tol, "rad/s", "angular_velocity")

    # Check the independently integrated reference before trusting its comparison.
    if "numerical_reference" in method:
        limits = {
            "r_ned": spec["position_atol_m"],
            "v_ned": spec["velocity_atol_mps"],
            "q_bn": angle_tol,
            "w_body": rate_tol,
        }
        units = {"r_ned": "m", "v_ned": "m/s", "q_bn": "rad", "w_body": "rad/s"}
        for key, value in method["numerical_reference"]["max_refinement_difference"].items():
            check("reference_stability_" + key, value, 0.01 * limits[key], units[key])

    # Verify mass properties using reference mass, not simulated mass or diagnostics.
    properties = [
        mass_properties(cfg.plant.vehicle, m, gas)
        for m, gas in zip(reference["mass"], reference["rcs_fuel"], strict=True)
    ]
    reference["x_cg"] = np.array([x[0] for x in properties])
    reference["inertia_diag"] = np.array([np.diag(x[1]) for x in properties])
    compare("x_cg", [a["x_cg"] for a in aux], 1e-8, "m")
    actual_inertia = np.array([a["inertia_diag"] for a in aux])
    reference["actual_inertia_diag"] = actual_inertia
    inertia_error = np.linalg.norm(actual_inertia - reference["inertia_diag"], axis=1)
    reference["error_inertia_diag"] = inertia_error
    check(
        "inertia_relative",
        inertia_error / np.linalg.norm(reference["inertia_diag"], axis=1),
        1e-10,
        "dimensionless",
    )

    thrust = np.array([a["thrust_mag"] for a in aux])
    reference["actual_thrust_mag"] = thrust
    thrust_error = np.abs(thrust - reference["thrust_mag"])
    reference["error_thrust_mag"] = thrust_error
    mask = np.ones(len(result.t), dtype=bool)
    if kind in ("rocket_equation", "mass_depletion"):
        depletion = method["expected_depletion_s"]
        report["expected_depletion_s"] = depletion
        # The native last burn spreads remaining impulse over one plant step.
        # Compare instantaneous thrust away from that explicitly identified interval.
        mask = np.abs(result.t - depletion) > float(cfg.plant_dt) + 1e-12
        report["thrust_boundary_exclusion"] = {
            "half_width_s": float(cfg.plant_dt),
            "excluded_samples": int(np.sum(~mask)),
            "reason": "last partial burn is averaged over one plant integration step",
        }
        if kind == "mass_depletion":
            check("remaining_main_fuel", aux[-1]["main_propellant"], spec["mass_atol_kg"], "kg")
            check("final_thrust", thrust[-1], 1e-8, "N")
            check(
                "continued_after_depletion",
                0 if result.t[-1] > depletion + 1 else 1,
                0,
                "boolean_error",
            )
            empty = np.flatnonzero(np.array([a["main_propellant"] for a in aux]) <= 1e-8)
            bracket = None
            if len(empty) and empty[0] > 0:
                idx = int(empty[0])
                bracket = [float(result.t[idx - 1]), float(result.t[idx])]
            report["depletion_observation"] = {
                "sample_bracket_s": bracket,
                "note": "logging bracket, not an exact measured event time",
            }
            distance = (
                max(bracket[0] - depletion, depletion - bracket[1], 0)
                if bracket is not None
                else float("inf")
            )
            check("depletion_bracket", distance, float(cfg.plant_dt), "s")
    reference["thrust_comparison_mask"] = mask
    check("thrust", thrust_error[mask], 1e-6, "N")

    if kind == "inverse_square_energy":
        grav = cfg.environment.gravity
        energy = 0.5 * np.sum(result.v_ned**2, axis=1) - grav.mu / (
            grav.radius - result.r_ned[:, 2]
        )
        expected = reference["expected_specific_energy"]
        reference["specific_energy"] = energy
        check(
            "specific_energy_relative",
            np.abs(energy - expected) / abs(expected[0]),
            spec["relative_tolerance"],
            "dimensionless",
        )
    elif kind == "free_rotation":
        inertia = np.asarray(method["inertia_kg_m2"])
        C = Rotation.from_quat(np.roll(result.q_bn, -1, axis=1)).as_matrix()
        H = np.einsum("nij,nj->ni", C, result.w_body @ inertia.T)
        energy = 0.5 * np.einsum("ni,ij,nj->n", result.w_body, inertia, result.w_body)
        reference.update(angular_momentum=H, rotational_energy=energy)
        H0, E0 = reference["expected_angular_momentum"], reference["expected_rotational_energy"]
        check(
            "angular_momentum_relative",
            np.linalg.norm(H - H0, axis=1) / np.linalg.norm(H0[0]),
            spec["relative_tolerance"],
            "dimensionless",
        )
        check(
            "rotational_energy_relative",
            np.abs(energy - E0) / E0[0],
            spec["relative_tolerance"],
            "dimensionless",
        )
    elif kind == "constant_drag":
        report["terminal_speed_mps"] = method["terminal_speed_mps"]
        compare("rho", [a["rho"] for a in aux], 1e-10, "kg/m^3")
        compare("aero_cd", [a["aero_cd"] for a in aux], 1e-12, "dimensionless")
        # Constant-density trajectory is approximate; permit its documented 1e-7
        # relative density variation in the force check as well as roundoff.
        force = np.array([np.linalg.norm(a["F_aero_body"]) for a in aux])
        compare("drag_force_n", force, 1e-7 * np.max(reference["drag_force_n"]) + 1e-6, "N")
    report["status"] = "passed" if all(c["passed"] for c in report["checks"].values()) else "failed"
    return report, reference


def write_validation(report, reference, directory, *, plot=True):
    out = Path(directory)
    (out / "validation.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    if report["status"] == "not_requested":
        return
    np.savez_compressed(out / "reference.npz", **reference)
    # Side-by-side values and per-sample errors are readable without a plotting tool.
    columns = {"t_s": reference["t"]}
    for key in reference:
        if not key.startswith("actual_"):
            continue
        name = key.removeprefix("actual_")
        for label, values in (("reference", reference[name]), ("simulation", reference[key])):
            values = np.asarray(values)
            if values.ndim == 1:
                columns[f"{name}_{label}"] = values
            else:
                for index in range(values.shape[1]):
                    columns[f"{name}_{label}_{index}"] = values[:, index]
        if "error_" + name in reference:
            columns[name + "_error"] = reference["error_" + name]
    columns["thrust_compared"] = reference["thrust_comparison_mask"]
    with (out / "comparison.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values(), strict=True))
    if not plot:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    checks = report["checks"]
    fig, ax = plt.subplots(figsize=(14, 5))
    names = list(checks)
    ratios = [
        max(
            (checks[k]["error"] if checks[k]["error"] is not None else 1e30)
            / max(checks[k]["tolerance"], 1e-30),
            1e-16,
        )
        for k in names
    ]
    ax.bar(names, ratios)
    ax.axhline(1, color="red", linestyle="--", label="tolerance")
    ax.set(yscale="log", ylabel="error / tolerance", title=report["kind"])
    ax.tick_params(axis="x", rotation=75)
    ax.legend()
    fig.tight_layout()
    (out / "plots").mkdir(exist_ok=True)
    fig.savefig(out / "plots" / "validation.png", dpi=120)
    plt.close(fig)

    quantities = [
        ("r_ned", "Position [m]"),
        ("v_ned", "Velocity [m/s]"),
        ("mass", "Mass [kg]"),
        ("q_bn", "Attitude from initial [rad]"),
        ("w_body", "Angular velocity [rad/s]"),
    ]
    fig, axes = plt.subplots(len(quantities), 2, figsize=(13, 14))
    t = reference["t"]
    for (key, label), (left, right) in zip(quantities, axes, strict=True):
        for prefix, style, title in (("", "--", "reference"), ("actual_", ":", "simulation")):
            values = reference[prefix + key]
            if key == "q_bn":
                values = attitude_error(values, np.tile(reference[key][0], (len(t), 1)))
            if values.ndim == 1:
                left.plot(t, values, style, label=title)
            else:
                for i, axis in enumerate("xyz"):
                    left.plot(t, values[:, i], style, color=f"C{i}", label=f"{title} {axis}")
        left.set(ylabel=label)
        left.legend(fontsize=7, ncol=2)
        right.semilogy(t, np.maximum(reference["error_" + key], 1e-16))
        right.set(ylabel="Error (norm for vectors)")
        left.grid(True)
        right.grid(True)
    axes[-1, 0].set_xlabel("Time [s]")
    axes[-1, 1].set_xlabel("Time [s]")
    fig.suptitle(report["kind"] + ": independent reference vs simulation")
    fig.tight_layout()
    fig.savefig(out / "plots" / "reference_comparison.png", dpi=140)
    plt.close(fig)
