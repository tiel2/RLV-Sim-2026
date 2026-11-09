"""Frozen-candidate probes and scoped physical audits; no parameter fitting."""

from dataclasses import fields

import numpy as np
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation

from validation.flight_before import audit_history
from validation.flight_comparison_policy import interpolate_simulation, masked_statistics


def actuator_arrays(history):
    arrays = {key: getattr(history, key) for key in ("t_start", "t_end")}
    for kind in ("commanded", "held", "effective"):
        command = getattr(history, kind)
        arrays.update({f"{kind}_{f.name}": getattr(command, f.name) for f in fields(command)})
    return arrays


def perturb(config, changes, probes, *, carried_mass=None, carried_gas=None):
    """Copy a candidate. Recovery mass is changed once at entry, then actually carried."""
    cfg = OmegaConf.create(OmegaConf.to_container(config, resolve=True))
    ic = cfg.scenario.ic
    if carried_mass is None:
        ic.m *= 1 + changes.get("mass", 0) * probes.initial_mass_fraction
    else:
        ic.m, ic.rcs_fuel = float(carried_mass), float(carried_gas)
    v = np.array(ic.v_ned, dtype=float)
    speed = np.linalg.norm(v)
    if speed > 0:
        v *= (speed + changes.get("speed", 0) * probes.speed_mps) / speed
        # A rotation in the north/down plane preserves speed. Target stays fixed.
        v = Rotation.from_euler(
            "y", changes.get("direction", 0) * probes.direction_deg, degrees=True
        ).apply(v)
        ic.v_ned = v.tolist()
    ic.pitch_deg += changes.get("pitch", 0) * probes.pitch_deg
    cfg.fault.thrust_scale = 1 + changes.get("thrust", 0) * probes.thrust_fraction
    drag = 1 + changes.get("drag", 0) * probes.drag_fraction
    for key in ("cd_base", "cd_transonic", "cd_hyper", "cd_alpha"):
        cfg.plant.aero.coeffs[key] *= drag
    for key in ("fin_cd0", "fin_cd_alpha2"):
        cfg.plant.aero.fins[key] *= drag
    if changes.get("wind", 0):
        cfg.environment.wind.model = "steady"
        cfg.environment.wind.mean0 = [changes["wind"] * probes.wind_north_mps, 0.0, 0.0]
    if changes.get("realistic", False):
        cfg.plant.actuator.model = "realistic"
    return cfg


def physical_audit(p, a, cfg):
    report = audit_history(p, a, cfg)
    checks = report["checks"]
    floor = cfg.plant.vehicle.engines.throttle_min
    held, effective = a["held_throttle"], a["effective_throttle"]
    checks["held_throttle_floor"] = bool(np.all((held <= 1e-10) | (held >= floor - 1e-10)))
    checks["finite_actuators"] = bool(all(np.all(np.isfinite(v)) for v in a.values()))
    ideal = cfg.plant.actuator.model == "ideal"
    if ideal:
        checks["effective_throttle_floor"] = bool(
            np.all((effective <= 1e-10) | (effective >= floor - 1e-10))
        )
    rate_diagnostics = {}
    settings = cfg.plant.vehicle.actuator
    for key, limit, initial in (
        (
            "throttle",
            settings.throttle_rate_max,
            np.broadcast_to(cfg.scenario.ic.throttle0, effective.shape[1:]),
        ),
        ("gimbal", settings.tvc_rate_max_rad_s, np.zeros(a["effective_gimbal"].shape[1:])),
        ("fin", settings.fin_rate_max_rad_s, np.zeros(a["effective_fin"].shape[1:])),
    ):
        values = np.concatenate([np.asarray(initial)[None], a[f"effective_{key}"]])
        dt = np.diff(np.r_[a["t_start"][0], a["t_end"]])
        rate = np.diff(values, axis=0) / dt.reshape((-1,) + (1,) * (values.ndim - 1))
        maximum = float(np.max(abs(rate), initial=0))
        rate_diagnostics[key] = dict(
            maximum_endpoint_average_rate=maximum, limit=float(limit), enforced=not ideal
        )
        if not ideal:
            checks[f"{key}_endpoint_average_rate"] = maximum <= limit + 1e-7
    thrust_bound = (
        cfg.plant.vehicle.engines.n_engines
        * cfg.plant.vehicle.engines.thrust_vac
        * float(np.max(np.asarray(cfg.fault.thrust_scale)))
    )
    checks["thrust_nonnegative_and_bounded"] = bool(
        np.all(p["thrust_mag"] >= 0) and np.max(p["thrust_mag"]) <= thrust_bound + 1e-6
    )
    checks["atmosphere_finite_nonnegative"] = bool(
        np.all(np.isfinite(p["rho"])) and np.all(p["rho"] >= 0)
    )
    h = p["altitude_m"]
    # Domain of the layered lower-atmosphere implementation; contact is one native step late.
    gp = 6356766 * np.maximum(h, 0) / (6356766 + np.maximum(h, 0))
    checks["lower_atmosphere_domain"] = bool(np.max(gp) <= 84852)
    main = p["main_propellant"]
    tank = cfg.plant.vehicle.tanks
    liquids = main[:, None] * np.array([tank.of_ratio, 1]) / (1 + tank.of_ratio)
    fill = liquids / (np.pi * np.array(tank.radius) ** 2 * np.array(tank.density))
    checks["tank_capacity"] = bool(np.all(fill <= np.array(tank.length) + 1e-8))
    report.update(
        status="passed_scoped_checks" if all(checks.values()) else "failed_scoped_checks",
        failed_checks=[k for k, v in checks.items() if not v],
        actuator_rates=rate_diagnostics,
        effective_below_floor_samples=int(
            np.count_nonzero((effective > 1e-10) & (effective < floor - 1e-10))
        ),
        minimum_altitude_m=float(h.min()),
        maximum_altitude_m=float(h.max()),
        maximum_tank_fill_fraction=float(np.max(fill / np.array(tank.length))),
        scope="logged states and actuator endpoints; no assertion about unlogged extrema",
    )
    return report


def common_metrics(nominal, candidate):
    """Compare on identical original rows and adjacent supported intervals."""
    if not np.array_equal(nominal["source_row"], candidate["source_row"]):
        raise ValueError("source rows differ")
    report = {}
    for key in ("altitude_m", "speed_m_s"):
        mask = nominal[f"{key}_score_eligible"] & candidate[f"{key}_score_eligible"]
        intervals = nominal[f"{key}_interval_eligible"] & candidate[f"{key}_interval_eligible"]
        report[key] = {
            label: masked_statistics(
                data["reference_time_s"],
                data[f"error_{key}"],
                mask,
                max_gap_s=1.1,
                interval_eligible=intervals,
            )[0]
            for label, data in (("nominal", nominal), ("probe", candidate))
        }
    return report


def observation_probe(p, nominal, epoch, *, datum_m=0.0, time_shift_s=0.0):
    """Diagnostic observation operator only; frozen data, IC and nominal scores unchanged.

    Positive datum adds to simulated altitude; positive shift samples the simulation later.
    """
    candidate = {k: v.copy() for k, v in nominal.items()}
    epochs = nominal["reference_time_s"] - epoch + time_shift_s
    for key, values in (("altitude_m", p["altitude_m"] + datum_m), ("speed_m_s", p["speed_mps"])):
        prediction, segments = interpolate_simulation(
            p["t"], values, epochs, max_gap_s=1.1, return_segments=True
        )
        candidate[f"simulation_{key}"] = prediction
        candidate[f"error_{key}"] = prediction - nominal[f"reference_{key}"]
        candidate[f"{key}_score_eligible"] &= np.isfinite(prediction)
        candidate[f"{key}_interval_eligible"] &= np.r_[
            False, (segments[1:] == segments[:-1]) & (segments[1:] >= 0)
        ]
    return common_metrics(nominal, candidate)
