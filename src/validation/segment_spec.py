"""Reference-only segment preparation; no native trajectory or parameter fitting.

The derivative is an unconstrained, trailing local quadratic. Inconsistent speed
components are flagged rather than projected onto a fabricated feasible velocity.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from omegaconf import OmegaConf

from validation.references import mass_properties


def load_snapshot(root):
    """Read only after verifying every pinned file's size and SHA-256."""
    root = Path(root)
    manifest = json.loads((root / "manifest.json").read_text())
    for item in manifest["files"]:
        content = (root / item["file"]).read_bytes()
        if len(content) != item["bytes"] or hashlib.sha256(content).hexdigest() != item["sha256"]:
            raise ValueError(f"reference integrity failure: {item['file']}")
    raw = json.loads((root / "stage1_raw.json").read_text())
    time = np.asarray(raw["time"], dtype=float)
    height = np.asarray(raw["altitude"], dtype=float) * 1000
    speed = np.asarray(raw["velocity"], dtype=float)
    if (
        time.ndim != 1
        or len(time) < 2
        or height.shape != time.shape
        or speed.shape != time.shape
        or not np.all(np.isfinite([time, height, speed]))
        or np.any(np.diff(time) <= 0)
        or np.any(speed < 0)
    ):
        raise ValueError("invalid reference arrays")
    return manifest, time, height, speed, json.loads((root / "events.json").read_text())


def observation(time, values, epoch, max_gap_s=1.1):
    """Bracketed linear interpolation; reject gaps and extrapolation explicitly."""
    time, values = np.asarray(time), np.asarray(values)
    if not np.isfinite(epoch) or epoch < time[0] or epoch > time[-1]:
        raise ValueError("observation would extrapolate")
    right = int(np.searchsorted(time, epoch))
    if right < len(time) and time[right] == epoch:
        left = right
    else:
        left = right - 1
    if time[right] - time[left] > max_gap_s:
        raise ValueError("observation crosses a reference gap")
    value = float(np.interp(epoch, time[left : right + 1], values[left : right + 1]))
    return dict(
        value=value,
        bracket_time_s=[float(time[left]), float(time[right])],
        bracket_values=[float(values[left]), float(values[right])],
        interpolated=bool(left != right),
    )


def estimate_components(time, height, speed, epoch, span_s, *, max_gap_s=1.1, tolerance=0.01):
    """Fit only [epoch-span, epoch]; estimates are not independent measurements.

    Trapezoidal time weights reduce dependence on extraction sampling density.
    The fitted intercept is diagnostic; initial height uses the raw observation.
    No statistical uncertainty is inferred from repeated/quantized video samples.
    """
    if not np.isfinite(span_s) or span_s <= 0:
        raise ValueError("derivative span must be positive")
    time, height = np.asarray(time), np.asarray(height)
    report = dict(
        span_s=float(span_s),
        status="insufficient_support",
        vertical_up_mps=None,
        horizontal_speed_mps=None,
        flight_path_angle_deg=None,
    )
    if not np.isfinite(epoch) or epoch < time[0] or epoch > time[-1]:
        raise ValueError("derivative epoch would extrapolate")
    try:
        velocity = observation(time, speed, epoch, max_gap_s)["value"]
    except ValueError:
        report["status"] = "reference_gap"
        return report
    lo = epoch - span_s
    mask = (time >= lo) & (time <= epoch)
    t, h = time[mask], height[mask]
    if lo < time[0] or len(t) < 4:
        return report
    dt = np.diff(t)
    report.update(
        fit_range_s=[float(t[0]), float(t[-1])],
        samples=len(t),
        leading_support_gap_s=float(t[0] - lo),
        trailing_support_gap_s=float(epoch - t[-1]),
    )
    if (
        np.max(dt) > max_gap_s
        or t[0] - lo > max_gap_s
        or epoch - t[-1] > max_gap_s
        or t[-1] - t[0] < 0.8 * span_s
    ):
        report["status"] = "reference_gap"
        return report
    weights = np.r_[dt[0], dt[:-1] + dt[1:], dt[-1]] / 2
    x = (t - epoch) / span_s
    design = np.column_stack([np.ones_like(x), x, x * x])
    coefficients = np.linalg.lstsq(
        design * np.sqrt(weights[:, None]), h * np.sqrt(weights), rcond=None
    )[0]
    up = float(coefficients[1] / span_s)
    report.update(
        fitted_altitude_m=float(coefficients[0]),
        vertical_up_mps=up,
        altitude_fit_rmse_m=float(
            np.sqrt(np.average((design @ coefficients - h) ** 2, weights=weights))
        ),
        scalar_speed_mps=velocity,
        component_excess_mps=float(max(abs(up) - velocity, 0)),
    )
    if np.all(h == 0):
        report["status"] = "altitude_floor"
    elif abs(up) > velocity:
        # Never silently clamp a negative radicand or call the result a measured state.
        report["status"] = (
            "component_inconsistent" if abs(up) > velocity + tolerance else "component_roundoff"
        )
    elif velocity == 0:
        report["status"] = "zero_speed"
    else:
        horizontal = float(np.sqrt(velocity**2 - up**2))
        report.update(
            status="usable_estimate",
            horizontal_speed_mps=horizontal,
            flight_path_angle_deg=float(np.rad2deg(np.arctan2(up, horizontal))),
        )
    return report


def prepare_segments(root, recipe, compose):
    """Build observations and proposed states using authored, unfitted assumptions.

    ``compose(name)`` returns a resolved base Hydra configuration. Returned overrides
    are proposals for later implementation, not newly installed runtime scenarios.
    """
    manifest, time, height, speed, events = load_snapshot(root)
    options = recipe.reference
    spans = list(options.derivative_spans_s)
    if options.polynomial_degree != 2 or options.nominal_span_s not in spans:
        raise ValueError("recipe requires quadratic fits and a listed nominal span")
    segments = {}
    for name, spec in recipe.segments.items():
        start, end = float(spec.start_s), float(spec.end_s)
        if end <= start:
            raise ValueError("segment end must follow start")
        experiment_duration = float(spec.experiment_duration_s)
        burn_duration = float(spec.burn_duration_s)
        if (
            not np.isfinite(experiment_duration)
            or experiment_duration <= 0
            or not np.isfinite(burn_duration)
            or burn_duration < 0
            or burn_duration >= experiment_duration
            or (spec.active_engines and burn_duration <= 0)
        ):
            raise ValueError("experiment duration must be positive and contain the burn/cutoff")
        for key, epoch in ((spec.start_event, start), (spec.end_event, end)):
            if (0.0 if key == "liftoff" else events[key]) != epoch:
                raise ValueError(f"segment epoch does not match archived marker: {key}")
        boundaries = {}
        for label, epoch in (("start", start), ("end", end)):
            boundaries[label] = dict(
                reference_time_s=epoch,
                altitude_m=observation(time, height, epoch, options.max_gap_s),
                scalar_speed_mps=observation(time, speed, epoch, options.max_gap_s),
                velocity_estimates=[
                    estimate_components(
                        time,
                        height,
                        speed,
                        epoch,
                        span,
                        max_gap_s=options.max_gap_s,
                        tolerance=options.component_tolerance_mps,
                    )
                    for span in spans
                ],
            )
        boundary = boundaries["start"]
        baseline = next(
            x for x in boundary["velocity_estimates"] if x["span_s"] == options.nominal_span_s
        )
        if spec.attitude == "vertical":
            if start != 0 or boundary["scalar_speed_mps"]["value"] != 0:
                raise ValueError("vertical pad initialization requires the zero-speed launch epoch")
            velocity = [0.0, 0.0, 0.0]
            pitch, yaw = 90.0, float(spec.velocity_heading_deg)
        elif spec.attitude == "initial_retrograde" and baseline["status"] == "usable_estimate":
            heading = np.deg2rad(spec.velocity_heading_deg)
            horizontal, up = baseline["horizontal_speed_mps"], baseline["vertical_up_mps"]
            velocity = [
                float(horizontal * np.cos(heading)),
                float(horizontal * np.sin(heading)),
                -up,
            ]
            pitch = float(np.rad2deg(np.arctan2(-up, horizontal)))
            yaw = float((spec.velocity_heading_deg + 180) % 360)
        else:
            raise ValueError(f"no usable initial-state recipe: {name}")
        cfg = compose(spec.base_scenario)
        if spec.attitude != "vertical":
            cfg = OmegaConf.merge(cfg, recipe.recovery_overrides)
        v = cfg.plant.vehicle
        attached = float(v.get("attached_component", {}).get("mass", 0))
        main = spec.mass_kg - v.mass.m_dry - attached - spec.rcs_fuel_kg
        liquid = main * np.array([v.tanks.of_ratio, 1]) / (1 + v.tanks.of_ratio)
        capacity = np.pi * np.array(v.tanks.radius) ** 2 * v.tanks.length * v.tanks.density
        throttle = np.zeros(v.engines.n_engines)
        inputs = None
        if spec.input_policy != "retain_existing_ascent_schedule":
            if spec.active_engines:
                if not v.engines.throttle_min <= spec.throttle <= 1:
                    raise ValueError("proposed throttle outside configured operating range")
                throttle[list(spec.active_engines)] = spec.throttle
            inputs = [
                dict(
                    t=0.0,
                    throttle=throttle.tolist(),
                    gimbal=0.0,
                    fin=0.0,
                    rcs=0.0,
                    fin_deployment=float(spec.fin_deployment),
                )
            ]
            if spec.active_engines:
                inputs.append(dict(t=burn_duration, throttle=0.0, label="cutoff"))
            # Conservative full commanded flow held through a lag+rate shutdown allowance.
            duration = burn_duration
            mdot = float(throttle.sum() * v.engines.thrust_vac / (v.engines.isp * 9.80665))
            tail = float(v.actuator.throttle_tau + 1 / v.actuator.throttle_rate_max)
            budget = dict(
                nominal_burn_flow_kg_s=mdot,
                nominal_main_consumption_kg=mdot * duration,
                shutdown_screen_allowance_s=tail,
                shutdown_screen_extra_kg=mdot * tail,
                remaining_after_screen_kg=main - mdot * (duration + tail),
                meaning="planning screen, not integrated actuator flow or proven recovery reserve",
            )
        else:
            budget = dict(meaning="existing ascent budget; no additional burn fitted here")
        state = dict(
            r_ned=[*map(float, spec.horizontal_position_m), -boundary["altitude_m"]["value"]],
            v_ned=velocity,
            roll_deg=float(spec.roll_deg),
            pitch_deg=pitch,
            yaw_deg=yaw,
            w_body=[0.0, 0.0, 0.0],
            m=float(spec.mass_kg),
            rcs_fuel=float(spec.rcs_fuel_kg),
            throttle0=cfg.scenario.ic.throttle0 if spec.attitude == "vertical" else 0.0,
            launch_pad=bool(spec.attitude == "vertical"),
        )
        checks = dict(
            positive_main_propellant=bool(main > 0),
            tank_capacity=bool(np.all(liquid <= capacity)),
            wet_capacity=bool(spec.mass_kg <= v.mass.m_wet),
            burn_and_shutdown_screen=(
                bool(budget["remaining_after_screen_kg"] >= 0)
                if "remaining_after_screen_kg" in budget
                else None
            ),
        )
        if any(value is False for value in checks.values()):
            raise ValueError(f"inconsistent proposed mass/burn budget: {name}: {checks}")
        cg, inertia = mass_properties(v, float(spec.mass_kg), float(spec.rcs_fuel_kg))
        selected = (time >= start) & (time <= end)
        local_dt = np.diff(time[selected])
        segments[name] = dict(
            base_scenario=spec.base_scenario,
            reference_window_s=[start, end],
            simulation_duration_s=experiment_duration,
            prescribed_burn_duration_s=burn_duration,
            time_mapping=f"reference_time = simulation_time + {start:g} s",
            boundaries=boundaries,
            proposed_initial_state=state,
            horizontal_frame=spec.horizontal_frame,
            nominal_derivative_span_s=float(options.nominal_span_s),
            structural_mass_kg=float(v.mass.m_dry),
            attached_mass_kg=attached,
            initial_main_propellant_kg=float(main),
            tank_fill_fraction=(liquid / capacity).tolist(),
            initial_mass_properties=dict(
                x_cg_m=cg,
                inertia_diag_kg_m2=np.diag(inertia).tolist(),
                method="configuration-only cylinder quadrature",
            ),
            field_provenance=dict(
                initial_altitude="webcast interpolation; datum equality assumed",
                scalar_speed="webcast interpolation; ground-relative frame assumed",
                velocity_components="altitude derivative plus assumed heading; launch at rest",
                horizontal_position="assumed coordinates/gauge, not reconstructed ground track",
                body_attitude_and_rates="assumed initial alignment and zero rate",
                mass_and_fuel="assumed independent budgets, not measured flight histories",
                cg_and_inertia="calculated from assumed structure and tank geometry",
                inputs="assumed prescribed commands, not observed actuator histories",
            ),
            initial_fin_deployment=float(spec.fin_deployment),
            input_policy=spec.input_policy,
            proposed_inputs=inputs,
            vehicle_overrides=(
                OmegaConf.to_container(recipe.recovery_overrides, resolve=True)
                if spec.attitude != "vertical"
                else None
            ),
            burn_budget_screen=budget,
            planning_checks=checks,
            initial_actuators="zero TVC/fin angles and rates; throttle0 above; zero RCS command",
            reference_samples=int(np.sum(selected)),
            raw_covered_duration_s=float(np.sum(local_dt[local_dt <= options.max_gap_s])),
            reference_gaps_s=[
                [float(time[i]), float(time[i + 1])]
                for i in np.flatnonzero(np.diff(time) > options.max_gap_s)
                if time[i] < end and time[i + 1] > start
            ],
            status="nominal_physics_specification",
        )
    return dict(
        status="reference_analysis_and_proposed_specification",
        reference_commit=manifest["commit"],
        snapshot_integrity="passed",
        observation_fields=["time", "altitude", "scalar speed"],
        method="trailing time-weighted quadratic; no future altitude in boundary derivatives",
        sensitivity="2/4/6 s span variation, not statistical confidence bounds",
        limitations=[
            "Assumed launch-relative altitude and ground-relative speed; datum/frame unresolved.",
            "Heading, horizontal position, body attitude/rates, mass/fuel and inputs are assumed.",
            "Analysis recipe is separate from runtime YAML; this command only prepares data.",
            "Initial-state reconstruction is not an independent validation output.",
            "No numerical flight-accuracy acceptance limit or landing-success claim is assigned.",
        ],
        segments=segments,
    )
