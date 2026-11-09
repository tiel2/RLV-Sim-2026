"""Coverage-aware summaries for the public-flight report, without new fitting."""

import numpy as np

from validation.flight_comparison_policy import masked_statistics
from validation.flight_sensitivity import common_metrics

CHANNELS = ("altitude_m", "speed_m_s")


def channel_summary(arrays, key, phase_duration):
    """Report available observation duration separately from simulated overlap."""
    t = arrays["reference_time_s"]
    quality = arrays[f"{key}_quality_eligible"] & ~arrays["initialization_used"]
    available = masked_statistics(t, np.zeros(len(t)), quality, max_gap_s=1.1)[0]
    eligible = arrays[f"{key}_score_eligible"]
    metrics = masked_statistics(
        t,
        arrays[f"error_{key}"],
        eligible,
        max_gap_s=1.1,
        interval_eligible=arrays[f"{key}_interval_eligible"],
    )[0]
    duration = available["covered_duration_s"]
    coverage = metrics["covered_duration_s"]
    metrics.update(
        available_observation_duration_s=duration,
        phase_duration_s=float(phase_duration),
        fraction_of_phase=coverage / phase_duration,
        fraction_of_available_observations=coverage / duration if duration else None,
        unrepresented_eligible_duration_s=max(0.0, duration - coverage),
        all_eligible_samples_represented=bool(np.all(~quality | eligible)),
    )
    return metrics


def common_endpoint(before, after, key):
    """Last common scored observation is not the true phase endpoint or touchdown."""
    mask = before[f"{key}_score_eligible"] & after[f"{key}_score_eligible"]
    rows = np.flatnonzero(mask)
    if not len(rows):
        return dict(status="unavailable", reason="no_common_scored_observation")
    first, last = int(rows[0]), int(rows[-1])
    result = dict(
        status="last_common_scored_observation_not_touchdown_truth",
        first_time_s=float(after["reference_time_s"][first]),
        last_time_s=float(after["reference_time_s"][last]),
        source_row=int(after["source_row"][last]),
        reference_final=float(after[f"reference_{key}"][last]),
        reference_change=float(after[f"reference_{key}"][last] - after[f"reference_{key}"][first]),
    )
    for name, data in (("before", before), ("after", after)):
        result[name] = dict(
            final=float(data[f"simulation_{key}"][last]),
            residual=float(data[f"error_{key}"][last]),
            change=float(data[f"simulation_{key}"][last] - data[f"simulation_{key}"][first]),
        )
    return result


def phase_metrics(before, after, phase_duration):
    common = common_metrics(before, after)
    return dict(
        full_overlap={
            label: {key: channel_summary(data, key, phase_duration) for key in CHANNELS}
            for label, data in (("before", before), ("after", after))
        },
        common_coverage=common,
        changes_on_common_coverage={
            key: {
                metric: common[key]["probe"][metric] - common[key]["nominal"][metric]
                if common[key]["probe"][metric] is not None
                and common[key]["nominal"][metric] is not None
                else None
                for metric in ("bias", "rmse", "max_abs")
            }
            for key in CHANNELS
        },
        common_endpoints={key: common_endpoint(before, after, key) for key in CHANNELS},
        common_coverage_labels={"nominal": "before", "probe": "after"},
    )


def reproduction_errors(actual, archived):
    """Strict deterministic run check before reusing archived numerical evidence."""
    limits = dict(
        t=1e-10,
        r_ned=1e-6,
        v_ned=1e-6,
        q_bn=1e-8,
        w_body=1e-8,
        mass=1e-6,
        altitude_m=1e-6,
        speed_mps=1e-6,
    )
    errors = {}
    for key in limits:
        if actual[key].shape != archived[key].shape:
            return dict(passed=False, reason=f"shape_changed:{key}", limits=limits, errors=errors)
        errors[key] = float(np.max(abs(actual[key] - archived[key])))
    return dict(
        passed=all(np.isfinite(errors[k]) and errors[k] <= v for k, v in limits.items()),
        errors=errors,
        limits=limits,
        scope="native reproduction; independent numerical checks are separately archived",
    )
