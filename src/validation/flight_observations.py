"""Prepare audited webcast observations without running or fitting a simulation.

Eligibility means usable under the declared comparison assumptions, not certified
sensor accuracy. Preserve rows and adjacency so exclusions cannot be bridged.
"""

from __future__ import annotations

import numpy as np
from omegaconf import OmegaConf

from validation.segment_spec import load_snapshot


def channel_segments(time, eligible, *, max_gap_s):
    """Give eligible contiguous rows an ID; invalid rows have ID -1."""
    time, eligible = np.asarray(time, dtype=float), np.asarray(eligible, dtype=bool)
    if (
        time.ndim != 1
        or eligible.shape != time.shape
        or not np.all(np.isfinite(time))
        or np.any(np.diff(time) <= 0)
        or not np.isfinite(max_gap_s)
        or max_gap_s <= 0
    ):
        raise ValueError("invalid observation time, eligibility or gap threshold")
    connected = np.zeros(len(time), dtype=bool)
    connected[1:] = eligible[:-1] & eligible[1:] & (np.diff(time) <= max_gap_s)
    starts = eligible & ~connected
    return np.where(eligible, np.cumsum(starts) - 1, -1), connected


def phase_membership(time, phases):
    """Disjoint [start, end) windows; only the last phase includes its end."""
    time = np.asarray(time)
    membership = np.full(len(time), -1, dtype=int)
    previous_end = -np.inf
    for index, spec in enumerate(phases.values()):
        start, end = float(spec["start_s"]), float(spec["end_s"])
        if not np.all(np.isfinite([start, end])) or start >= end or start < previous_end:
            raise ValueError("phase windows must be finite, ordered and nonoverlapping")
        selected = (time >= start) & ((time <= end) if index == len(phases) - 1 else (time < end))
        membership[selected] = index
        previous_end = end
    return membership


def coverage(time, values, eligible, selected, *, max_gap_s):
    """Integrate only original adjacent eligible rows in the selected window."""
    time, values = np.asarray(time), np.asarray(values)
    eligible, selected = np.asarray(eligible, dtype=bool), np.asarray(selected, dtype=bool)
    usable = eligible & selected
    _, connected = channel_segments(time, usable, max_gap_s=max_gap_s)
    rows = np.flatnonzero(usable)
    return {
        "eligible_samples": len(rows),
        "excluded_samples": int(np.count_nonzero(selected & ~eligible)),
        "covered_duration_s": float(np.sum(np.diff(time)[connected[1:]])),
        "eligible_time_range_s": [float(time[rows[0]]), float(time[rows[-1]])]
        if len(rows)
        else None,
        "eligible_value_range": [float(np.min(values[rows])), float(np.max(values[rows]))]
        if len(rows)
        else None,
        "status": "eligible_under_assumptions" if np.any(connected) else "insufficient_coverage",
    }


def constant_runs(time, values, selected, *, max_gap_s):
    """Audit constant display runs, stopping at gaps or nonselected samples."""
    selected = np.asarray(selected, dtype=bool)
    same = np.zeros(len(time), dtype=bool)
    same[1:] = (
        selected[:-1] & selected[1:] & (np.diff(time) <= max_gap_s) & (values[1:] == values[:-1])
    )
    starts = np.flatnonzero(selected & ~same)
    stops = np.flatnonzero(selected & ~np.r_[same[1:], False])
    return [
        {
            "first_source_row": int(first),
            "last_source_row": int(last),
            "first_time_s": float(time[first]),
            "last_time_s": float(time[last]),
            "samples": int(last - first + 1),
            "value": float(values[first]),
        }
        for first, last in zip(starts, stops, strict=True)
    ]


def prepare_observations(root, policy):
    """Return an evidence report and SI arrays under the frozen CRS-11 v1 policy."""
    p = OmegaConf.to_container(policy, resolve=True) if OmegaConf.is_config(policy) else policy
    manifest, time, height, speed, events = load_snapshot(root)
    if (
        p["schema_version"] != 1
        or p["policy_id"] != "crs11_observations_v1"
        or p["reference"]["dataset"] != "crs11"
        or p["reference"]["stage"] != manifest["stage"]
        or p["reference"]["commit"] != manifest["commit"]
        or p["reference"]["raw_file"] != "stage1_raw.json"
        or p["reference"]["event_file"] != "events.json"
        or p["reference"]["input_units"] != {"time": "s", "altitude": "km", "velocity": "m/s"}
        or p["reference"]["output_units"] != {"time_s": "s", "altitude_m": "m", "speed_mps": "m/s"}
        or p["observables"]["altitude"]["input_scale_to_si"] != 1000
        or p["observables"]["speed"]["input_scale_to_si"] != 1
    ):
        raise ValueError("policy does not match supported snapshot or units")
    if np.any(height < 0):
        raise ValueError("negative webcast altitude requires explicit review")
    if (
        p["quality"]["zero_altitude"] != "exclude_from_altitude_score_and_derivatives"
        or p["quality"]["zero_speed"] != "retain_with_flag_not_exact_touchdown_truth"
        or p["quality"]["zero_altitude_does_not_mask_speed"] is not True
        or p["quality"]["repeated_values"] != "retain_with_flag_use_time_weighting"
        or p["quality"]["outlier_deletion"] != "none_without_independent_evidence"
        or p["quality"]["smoothing"] != "none"
        or p["time"]["observation_resampling"] != "none"
        or p["quality"]["coverage"]
        != "sum_only_adjacent_eligible_same_phase_intervals_not_crossing_gaps"
        or p["phase_partition"]["boundary_rule"] != "left_closed_right_open_last_phase_right_closed"
        or p["phase_partition"]["shared_boundary_owner"] != "later_phase"
        or p["phase_partition"]["landing_split"]["rule"]
        != "first_positive_altitude_at_or_below_threshold_after_landing_start"
    ):
        raise ValueError("unsupported observation processing policy")
    gap = float(p["quality"]["max_gap_s"])
    partition = p["phase_partition"]
    split = partition["landing_split"]
    candidates = np.flatnonzero(
        (time > events["landing_start"]) & (height > 0) & (height <= split["threshold_m"])
    )
    if not len(candidates) or time[candidates[0]] != split["fixed_time_s"]:
        raise ValueError("frozen landing split does not match its observation rule")
    if partition["landing_end"]["fixed_time_s"] != time[-1]:
        raise ValueError("frozen archive end does not match observations")
    anchors = {
        **events,
        "archive_launch_epoch": float(time[0]),
        "landing_split": float(time[candidates[0]]),
        "archive_end": float(time[-1]),
    }
    phases = p["phases"]
    if list(phases) != ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]:
        raise ValueError("policy requires the five declared phases in chronological order")
    for spec in phases.values():
        for side in ("start", "end"):
            if spec[f"{side}_s"] != anchors[spec[f"{side}_anchor"]]:
                raise ValueError("phase endpoint does not match its declared anchor")
    membership = phase_membership(time, phases)
    dt = np.diff(time)
    arrays = {
        "source_row": np.arange(len(time)),
        "reference_time_s": time,
        "altitude_m": height,
        "speed_mps": speed,
        "phase_index": membership,
        "gap_before": np.r_[False, dt > gap],
        "zero_altitude": height == 0,
        "zero_speed": speed == 0,
    }
    phase_reports = {name: {} for name in phases}
    for channel, values, eligible in (
        ("altitude", height, height > 0),
        ("speed", speed, np.ones(len(time), dtype=bool)),
    ):
        segments, connected = channel_segments(time, eligible, max_gap_s=gap)
        arrays[f"{channel}_eligible"] = eligible
        arrays[f"{channel}_segment_id"] = segments
        arrays[f"{channel}_repeat_previous"] = np.r_[False, values[1:] == values[:-1]]
        arrays[f"{channel}_interval_eligible"] = (
            connected & np.r_[False, (membership[1:] == membership[:-1]) & (membership[1:] >= 0)]
        )
        for index, (name, spec) in enumerate(phases.items()):
            selected = membership == index
            stats = coverage(time, values, eligible, selected, max_gap_s=gap)
            stats["coverage_fraction"] = stats["covered_duration_s"] / (
                spec["end_s"] - spec["start_s"]
            )
            stats["zero_display_samples"] = int(np.count_nonzero(selected & (values == 0)))
            stats["repeated_adjacent_samples"] = int(
                np.count_nonzero(selected & arrays[f"{channel}_repeat_previous"])
            )
            phase_reports[name][channel] = stats
    arrays["phase_local_time_s"] = np.full(len(time), np.nan)
    for index, (name, spec) in enumerate(phases.items()):
        selected = membership == index
        arrays["phase_local_time_s"][selected] = time[selected] - spec["start_s"]
        phase_reports[name].update(
            phase_index=index,
            requested_range_s=[spec["start_s"], spec["end_s"]],
            time_offset_s=-spec["start_s"],
            raw_samples=int(np.count_nonzero(selected)),
            boundary_rule="[start,end]" if index == len(phases) - 1 else "[start,end)",
        )
    split_row = int(candidates[0])
    report = {
        "status": "observation_policy_prepared_with_declared_assumptions",
        "scope": "Step 1 only; no simulation, calibration, or empirical accuracy verdict.",
        "policy": p,
        "snapshot": {
            "integrity": "passed",
            "manifest": manifest,
            "raw_samples": len(time),
            "raw_range_s": [float(time[0]), float(time[-1])],
            "median_step_s": float(np.median(dt)),
            "maximum_step_s": float(np.max(dt)),
        },
        "gaps": [
            {"left_s": float(time[i]), "right_s": float(time[i + 1]), "duration_s": float(dt[i])}
            for i in np.flatnonzero(dt > gap)
        ],
        "events": {
            "source": "events.json",
            "times_s": events,
            "frame_verified": False,
            "touchdown_time_s": None,
            "landing_end_role": "rounded_burn_end_marker_not_measured_contact",
        },
        "landing_split": {
            "time_s": float(time[split_row]),
            "source_row": split_row,
            "altitude_m": float(height[split_row]),
            "speed_mps": float(speed[split_row]),
            "previous_sample": {
                "time_s": float(time[split_row - 1]),
                "altitude_m": float(height[split_row - 1]),
                "speed_mps": float(speed[split_row - 1]),
            },
            "role": split["meaning"],
        },
        "altitude_zero_runs": constant_runs(time, height, height == 0, max_gap_s=gap),
        "landing_altitude_plateaus": constant_runs(
            time, height, (time > events["landing_start"]) & (height <= 600), max_gap_s=gap
        ),
        "landing_zero_speed_runs": constant_runs(
            time, speed, (time > events["landing_start"]) & (speed == 0), max_gap_s=gap
        ),
        "phases": phase_reports,
        "limitations": [
            "Source altitude datum, speed frame, filtering and latency remain unverified.",
            "No original-video frame crosscheck; this policy uses the pinned archive.",
            "Altitude zero is masked separately from speed; displayed zero is not exact contact.",
            "Repeated values are retained; extraction samples are not independent measurements.",
            "No statistical measurement error bars or physical accuracy threshold is assigned.",
            "Step 1 observation masks alone do not exclude initialization/calibration use.",
            "The policy comparator additionally masks initialization and simulation coverage.",
            "Step 1 coverage is observation availability, not a simulation comparison result.",
        ],
    }
    return report, arrays
