"""Channel-specific comparison with original-row adjacency and explicit exclusions."""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
from omegaconf import OmegaConf

from validation.flight_observations import channel_segments, prepare_observations
from validation.flight_scenarios import initialization_mask, reconstruct_initial_states


def masked_statistics(time, error, eligible, *, max_gap_s, interval_eligible=None):
    """Do not compress the arrays before identifying eligible integration intervals."""
    time, error = np.asarray(time), np.asarray(error)
    eligible = np.asarray(eligible, dtype=bool) & np.isfinite(error)
    _, connected = channel_segments(time, eligible, max_gap_s=max_gap_s)
    if interval_eligible is not None:
        connected &= np.asarray(interval_eligible, dtype=bool)
    weights = np.zeros(len(time))
    half_dt = np.diff(time) * connected[1:] / 2
    weights[:-1] += half_dt
    weights[1:] += half_dt
    used = weights > 0
    duration = float(weights.sum())
    report = dict(
        status="insufficient_coverage",
        reason="no_adjacent_eligible_interval",
        eligible_samples=int(eligible.sum()),
        weighted_samples=int(used.sum()),
        covered_duration_s=duration,
        bias=None,
        rmse=None,
        max_abs=None,
    )
    if duration > 0:
        report.update(
            status="comparison_only",
            reason=None,
            bias=float(np.sum(weights[used] * error[used]) / duration),
            rmse=float(np.sqrt(np.sum(weights[used] * error[used] ** 2) / duration)),
            max_abs=float(np.max(np.abs(error[used]))),
        )
    return report, connected


def interpolate_simulation(time, values, epochs, *, max_gap_s, return_segments=False):
    """Only finite adjacent simulation samples support interpolation; never extrapolate."""
    time, values, epochs = np.asarray(time), np.asarray(values), np.asarray(epochs)
    if (
        time.ndim != 1
        or values.shape != time.shape
        or not np.all(np.isfinite(time))
        or np.any(np.diff(time) <= 0)
    ):
        raise ValueError("invalid simulation channel/time")
    output = np.full(len(epochs), np.nan)
    groups = np.full(len(epochs), -1, dtype=int)
    segments, _ = channel_segments(time, np.isfinite(values), max_gap_s=max_gap_s)
    if not len(time):
        return (output, groups) if return_segments else output
    right = np.searchsorted(time, epochs)
    in_range = right < len(time)
    exact = in_range & (time[np.minimum(right, len(time) - 1)] == epochs)
    output[exact] = values[right[exact]]
    groups[exact] = segments[right[exact]]
    between = ~exact & (right > 0) & in_range
    rows = np.flatnonzero(between)
    r, l = right[rows], right[rows] - 1
    valid = np.isfinite(values[l]) & np.isfinite(values[r]) & (time[r] - time[l] <= max_gap_s)
    rows, l, r = rows[valid], l[valid], r[valid]
    fraction = (epochs[rows] - time[l]) / (time[r] - time[l])
    output[rows] = values[l] + fraction * (values[r] - values[l])
    groups[rows] = segments[l]
    return (output, groups) if return_segments else output


def compare_policy_reference(result, config, *, data_root=None):
    root = Path(__file__).resolve().parents[2]
    policy_path = Path(config.policy_path)
    if not policy_path.is_absolute():
        policy_path = root / policy_path
    p = OmegaConf.to_container(OmegaConf.load(policy_path), resolve=True)
    if config.policy != p["policy_id"] or config.dataset != p["reference"]["dataset"]:
        raise ValueError("comparison policy/dataset mismatch")
    observation_report, observed = prepare_observations(
        data_root or root / "data/reference/crs11", p
    )
    _, intervals = reconstruct_initial_states(p, observed)
    name = str(config.segment)
    phase = p["phases"][name]
    start, end = phase["start_s"], phase["end_s"]
    if (
        float(config.start_s),
        float(config.end_s),
        float(config.time_offset_s),
        float(config.altitude_offset_m),
    ) != (start, end, -start, 0.0):
        raise ValueError("comparison offsets/windows differ from the frozen policy")
    if (
        p["observables"]["altitude"]["altitude_offset_m"] != 0
        or p["observables"]["altitude"]["comparison_assumption"]
        != "simulator_radial_altitude_plus_fixed_offset"
        or p["observables"]["speed"]["comparison_assumption"]
        != "earth_relative_scalar_speed_norm_v_ned"
    ):
        raise ValueError("unsupported observable conventions")
    phase_index = list(p["phases"]).index(name)
    selected = observed["phase_index"] == phase_index
    rt = observed["reference_time_s"][selected]
    st = rt - start
    reserved = initialization_mask(observed["reference_time_s"], intervals)[selected]
    t = np.asarray(result.t, dtype=float)
    if t.ndim != 1 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError("invalid simulation time axis")
    overlap = (st >= t[0]) & (st <= t[-1]) if len(t) else np.zeros(len(st), dtype=bool)
    gap = p["quality"]["max_gap_s"]
    arrays = dict(
        reference_time_s=rt,
        simulation_time_s=st,
        source_row=observed["source_row"][selected],
        initialization_used=reserved,
        simulation_overlap=overlap,
    )
    metrics = {}
    per_tick = result.aux.get("per_tick", [])
    altitude = np.array([row.get("altitude", np.nan) for row in per_tick])
    if altitude.shape != t.shape:
        altitude = np.full(len(t), np.nan)
    velocity = np.asarray(result.v_ned)
    speed = (
        np.linalg.norm(velocity, axis=1)
        if velocity.shape == (len(t), 3)
        else np.full(len(t), np.nan)
    )
    for channel, key, field, simulated in (
        ("altitude", "altitude_m", "altitude_m", altitude),
        ("speed", "speed_m_s", "speed_mps", speed),
    ):
        quality = observed[f"{channel}_eligible"][selected]
        actual = observed[field][selected]
        predicted, sim_segments = interpolate_simulation(
            t, simulated, st, max_gap_s=gap, return_segments=True
        )
        usable = quality & ~reserved & overlap & np.isfinite(predicted)
        error = predicted - actual
        sim_connected = np.r_[
            False, (sim_segments[1:] == sim_segments[:-1]) & (sim_segments[1:] >= 0)
        ]
        stats, connected = masked_statistics(
            rt, error, usable, max_gap_s=gap, interval_eligible=sim_connected
        )
        if not np.any(quality & ~reserved):
            stats.update(
                status="not_comparable", reason="no_eligible_observations_after_initialization"
            )
        elif not np.any(overlap):
            stats.update(status="not_comparable", reason="no_simulation_overlap")
        elif not np.any(np.isfinite(predicted)):
            stats.update(status="not_comparable", reason="observable_unavailable_in_simulation")
        stats["coverage_fraction_of_phase"] = stats["covered_duration_s"] / (end - start)
        stats["exclusions"] = dict(
            quality=int((~quality).sum()),
            initialization=int((quality & reserved).sum()),
            outside_simulation=int((quality & ~reserved & ~overlap).sum()),
            missing_simulation_channel_or_gap=int(
                (quality & ~reserved & overlap & ~np.isfinite(predicted)).sum()
            ),
        )
        metrics[key] = stats
        arrays.update(
            {
                f"reference_{key}": actual,
                f"simulation_{key}": predicted,
                f"error_{key}": np.where(usable, error, np.nan),
                f"{key}_quality_eligible": quality,
                f"{key}_score_eligible": usable,
                f"{key}_interval_eligible": connected,
            }
        )
    statuses = [m["status"] for m in metrics.values()]
    status = (
        "comparison_only"
        if "comparison_only" in statuses
        else "not_comparable"
        if all(s == "not_comparable" for s in statuses)
        else "insufficient_coverage"
    )
    recipe = p["reconstruction"]
    objective = None
    if all(s == "comparison_only" for s in statuses):
        weights = recipe["channel_weights"]
        scales = recipe["normalization_scales"]
        objective = float(
            (
                weights["altitude"] * (metrics["altitude_m"]["rmse"] / scales["altitude_m"]) ** 2
                + weights["speed"] * (metrics["speed_m_s"]["rmse"] / scales["speed_mps"]) ** 2
            )
            / sum(weights.values())
        )
    report = dict(
        status=status,
        segment=name,
        role=str(config.role),
        reference_commit=p["reference"]["commit"],
        policy_id=p["policy_id"],
        policy_sha256=hashlib.sha256(policy_path.read_bytes()).hexdigest(),
        source="stage1_raw.json",
        conventions=p["observables"],
        time_mapping="simulation_time = reference_time - phase_start",
        time_offset_s=-start,
        altitude_offset_m=0.0,
        requested_range_s=[start, end],
        simulation_reference_range_s=[float(t[0] + start), float(t[-1] + start)]
        if len(t)
        else None,
        comparison_ends_early=bool(not len(t) or t[-1] + start < end - 1e-8),
        metrics={"requested_overlap": metrics},
        channel_status={k: v["status"] for k, v in metrics.items()},
        normalized_reconstruction_mse=objective,
        normalization=recipe["normalization_scales"],
        normalization_role=recipe["scales_role"],
        initialization_exclusions=intervals,
        initialization_exclusion_scope=recipe["initialization_exclusion"],
        fit_window=[start, end],
        fit_mask="channel score eligibility; no held-out prediction claim",
        event_time_scores=None,
        touchdown_reference=None,
        checkpoints={},
        error_sign="simulation minus reference",
        max_gap_s=gap,
        weighting="trapezoidal over original adjacent eligible rows, per channel",
        limitations=observation_report["limitations"][:6]
        + [
            "All inspected initialization windows are excluded across all five phases.",
            "Fit and descriptive evaluation use the same declared mask; not predictive validation.",
            "Physical acceptance and successful landing require separate full-run checks.",
            "Partial overlap or a missing channel never constitutes full-phase completion.",
        ],
    )
    return report, arrays
