"""Observation-informed initial states and authored inputs, without trajectory fitting."""

from __future__ import annotations

import numpy as np
from omegaconf import OmegaConf

from validation.segment_spec import estimate_components, observation


def reconstruct_initial_states(policy, arrays):
    """Keep scalar observations fixed; never clamp an inconsistent velocity estimate."""
    p = policy
    recipe = p["reconstruction"]
    spans = recipe["derivative_spans_s"]
    nominal = recipe["nominal_span_s"]
    if nominal not in spans or recipe["derivative_method"] != "trailing_time_weighted_quadratic":
        raise ValueError("unsupported initial velocity recipe")
    if (
        recipe["initialization_exclusion"]
        != "union_of_all_phase_derivative_support_and_ic_brackets_inclusive"
        or recipe["fit_window"] != "each_phase_window_minus_initialization_and_quality_masks"
        or recipe["residual_weighting"] != "adjacent_interval_trapezoidal_time_weighted"
    ):
        raise ValueError("unsupported initialization or evaluation policy")
    scales = list(recipe["normalization_scales"].values())
    weights = list(recipe["channel_weights"].values())
    if not np.all(np.isfinite(scales + weights)) or min(scales + weights) <= 0:
        raise ValueError("normalization scales and channel weights must be finite and positive")
    time, height, speed = (arrays[k] for k in ("reference_time_s", "altitude_m", "speed_mps"))
    gap = p["quality"]["max_gap_s"]
    states, intervals = {}, []
    for name, phase in p["phases"].items():
        epoch = phase["start_s"]
        spec = recipe["phases"][name]
        h = observation(time, height, epoch, gap)
        v = observation(time, speed, epoch, gap)
        estimates = []
        if name == "ascent":
            if epoch != 0 or v["value"] != 0:
                raise ValueError("pad initialization requires the zero-speed launch epoch")
            velocity = [0.0, 0.0, 0.0]
            pitch, yaw = 90.0, 45.0
            support = [0.0, max(h["bracket_time_s"] + v["bracket_time_s"])]
        else:
            bracket = (time >= h["bracket_time_s"][0]) & (time <= h["bracket_time_s"][1])
            if not np.all(arrays["altitude_eligible"][bracket]):
                raise ValueError(f"ineligible altitude at initialization: {name}")
            for span in spans:
                selected = (time >= epoch - span) & (time <= epoch)
                if not np.all(arrays["altitude_eligible"][selected]):
                    estimates.append(dict(span_s=span, status="excluded_altitude_support"))
                else:
                    estimates.append(
                        estimate_components(time, height, speed, epoch, span, max_gap_s=gap)
                    )
            chosen = next(row for row in estimates if row["span_s"] == nominal)
            if chosen["status"] != "usable_estimate":
                raise ValueError(f"unusable nominal velocity reconstruction: {name}: {chosen}")
            horizontal, up = chosen["horizontal_speed_mps"], chosen["vertical_up_mps"]
            heading = float(recipe["fixed"]["heading_deg"])
            angle = np.deg2rad(heading)
            velocity = [float(horizontal * np.cos(angle)), float(horizontal * np.sin(angle)), -up]
            if spec["attitude"] == "initial_retrograde":
                pitch, yaw = float(np.rad2deg(np.arctan2(-up, horizontal))), (heading + 180) % 360
            elif spec["attitude"] == "vertical":
                pitch, yaw = 90.0, heading
            else:
                raise ValueError("unsupported initial attitude assumption")
            # Conservatively reserve every inspected derivative span, including rejected ones.
            support = [epoch - max(spans), max(h["bracket_time_s"] + v["bracket_time_s"])]
        intervals.append(dict(phase=name, range_s=support))
        states[name] = dict(
            initial_state=dict(
                r_ned=[0.0, 0.0, -h["value"]],
                v_ned=velocity,
                roll_deg=0.0,
                pitch_deg=pitch,
                yaw_deg=yaw,
                w_body=[0.0, 0.0, 0.0],
                m=spec["mass_kg"],
                rcs_fuel=200.0,
                throttle0=1.0 if name == "ascent" else 0.0,
                launch_pad=name == "ascent",
            ),
            altitude_observation=h,
            speed_observation=v,
            derivative_estimates=estimates,
            nominal_span_s=None if name == "ascent" else nominal,
            initialization_use_range_s=support,
            field_provenance=dict(
                altitude="bracketed webcast altitude; pad zero is an engineering condition",
                scalar_speed="bracketed webcast speed under Earth-relative frame assumption",
                velocity_components="trailing altitude derivative and assumed heading; pad at rest",
                position_heading_attitude_rates="assumed origin, heading, alignment and zero rates",
                mass_propellant_actuators="authored independent budgets and actuator initial state",
                inputs="authored commands or LCvx/PD; actual flight inputs unavailable",
                landing_target="initial ground projection, engineering target, not measured LZ-1",
            ),
        )
    return states, intervals


def initialization_mask(time, intervals):
    mask = np.zeros(len(time), dtype=bool)
    for row in intervals:
        lo, hi = row["range_s"]
        mask |= (time >= lo) & (time <= hi)
    return mask


def scenario_fragment(name, policy, state, base):
    """Return a Hydra overlay plus concrete future search bounds; no optimizer runs."""
    phase = policy["phases"][name]
    spec = policy["reconstruction"]["phases"][name]
    start, end = phase["start_s"], phase["end_s"]
    reference = dict(
        enabled=True,
        dataset="crs11",
        segment=name,
        policy="crs11_observations_v1",
        policy_path="configs/analysis/public_flight_comparison.yaml",
        start_s=start,
        end_s=end,
        time_offset_s=-start,
        altitude_offset_m=0.0,
        exclude_initial_sample=True,
        conventions="Provisional equal radial-altitude datum and Earth-relative scalar speed",
        role="observation_informed_uncalibrated_reconstruction",
    )
    scenario = dict(
        name=f"flight_comparison_{name}",
        description="Independent CRS-11 reconstruction, unfitted",
        ic=state["initial_state"],
        horizon=dict(t_max=spec["duration_s"]),
        reference=reference,
        reconstruction=dict(
            policy_version=1,
            provenance=state["field_provenance"],
            initialization_use_range_s=state["initialization_use_range_s"],
        ),
    )
    parameters = [
        dict(
            name="initial_mass_kg",
            paths=["scenario.ic.m"],
            initial=spec["mass_kg"],
            bounds=spec["mass_bounds_kg"],
            constraint="tank/wet/fuel checks required",
        )
    ]
    fragment = dict(defaults=[spec["base_scenario"], "_self_"], scenario=scenario)
    if name == "ascent":
        rows = OmegaConf.to_container(base.scenario.inputs, resolve=True)
        scenario["inputs"] = [r for r in rows if r["t"] < spec["duration_s"]]
        for i, row in enumerate(scenario["inputs"]):
            if "throttle" in row:
                parameters.append(
                    dict(
                        name=f"throttle_knot_{i}",
                        paths=[f"scenario.inputs.{i}.throttle"],
                        initial=row["throttle"],
                        bounds=spec["throttle_bounds"],
                    )
                )
            if isinstance(row.get("gimbal"), list):
                parameters.append(
                    dict(
                        name="pitch_kick_rad",
                        initial=row["gimbal"][0][0],
                        bounds=spec["pitch_kick_bounds_rad"],
                        paths=[f"scenario.inputs.{i}.gimbal.{j}.0" for j in range(9)],
                    )
                )
    elif name == "landing":
        # Match the recovery engine/tank assumptions; keep the declared ideal-actuator baseline.
        fragment["plant"] = dict(
            vehicle=dict(
                engines=dict(thrust_vac=914000.0, nozzle_exit_area=0.68),
                tanks=dict(
                    density=[1230.0, 810.0],
                    radius=[1.8, 1.8],
                    bottom=[-26.0, -40.0],
                    length=[25.0, 14.0],
                ),
            )
        )
        fragment["gnc"] = dict(
            guidance=dict(
                convex_target_ned=policy["reconstruction"]["fixed"]["landing_target_ned_m"],
                convex_thrust_rate_max_nps=914000.0,
            )
        )
        parameters.append(
            dict(
                name="pointing_cone_deg",
                paths=["gnc.guidance.attitude_tilt_max_deg"],
                initial=float(base.gnc.guidance.attitude_tilt_max_deg),
                bounds=spec["pointing_cone_bounds_deg"],
            )
        )
    else:
        throttle = np.zeros(base.plant.vehicle.engines.n_engines)
        throttle[spec["active_engines"]] = spec["throttle"]
        scenario["inputs"] = [
            dict(
                t=0.0,
                throttle=throttle.tolist(),
                gimbal=0.0,
                fin=0.0,
                rcs=0.0,
                fin_deployment=1.0,
                label=name,
            )
        ]
        if spec["active_engines"]:
            parameters.append(
                dict(
                    name="active_engine_throttle",
                    initial=spec["throttle"],
                    bounds=spec["throttle_bounds"],
                    paths=[f"scenario.inputs.0.throttle.{j}" for j in spec["active_engines"]],
                )
            )
        else:
            parameters.append(
                dict(
                    name="common_fin_angle_rad",
                    paths=["scenario.inputs.0.fin"],
                    initial=0.0,
                    bounds=spec["fin_bounds_rad"],
                )
            )
    scenario["reconstruction"]["future_parameters"] = parameters
    for parameter in parameters:
        lo, hi = parameter["bounds"]
        if not lo <= parameter["initial"] <= hi:
            raise ValueError(f"initial parameter outside declared bounds: {parameter['name']}")
    return fragment
