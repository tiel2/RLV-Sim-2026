"""Report regression: coverage loss and unavailable endpoints remain explicit."""

import numpy as np
import pytest

from validation.flight_report import (
    CHANNELS,
    channel_summary,
    common_endpoint,
    phase_metrics,
    reproduction_errors,
)


def sample():
    a = dict(
        reference_time_s=np.arange(5, dtype=float),
        source_row=np.arange(5),
        initialization_used=np.array([True, False, False, False, False]),
    )
    for key in CHANNELS:
        a.update(
            {
                f"{key}_quality_eligible": np.ones(5, dtype=bool),
                f"{key}_score_eligible": np.array([False, True, True, False, False]),
                f"{key}_interval_eligible": np.array([False, False, True, False, False]),
                f"reference_{key}": np.arange(5, dtype=float),
                f"simulation_{key}": np.array([1.0, 2.0, 3.0, np.nan, np.nan]),
                f"error_{key}": np.array([np.nan, 1.0, 1.0, np.nan, np.nan]),
            }
        )
    return a


def test_missing_tail_has_nonzero_unrepresented_duration_not_zero_error():
    report = channel_summary(sample(), "speed_m_s", 4)
    assert report["available_observation_duration_s"] == 3
    assert report["covered_duration_s"] == 1
    assert report["unrepresented_eligible_duration_s"] == 2
    assert report["fraction_of_phase"] == 0.25
    assert report["fraction_of_available_observations"] == pytest.approx(1 / 3)
    assert not report["all_eligible_samples_represented"]
    assert report["rmse"] == 1


def test_endpoints_use_same_scored_time_not_each_runs_different_terminal():
    before, after = sample(), sample()
    for key in CHANNELS:
        after[f"{key}_score_eligible"][3:] = True
        after[f"{key}_interval_eligible"][3:] = True
        after[f"simulation_{key}"][1:] = [3, 4, 20, 40]
        after[f"error_{key}"][1:] = [2, 2, 17, 36]
    report = phase_metrics(before, after, 4)
    endpoint = report["common_endpoints"]["altitude_m"]
    assert endpoint["last_time_s"] == 2
    assert endpoint["after"]["residual"] == 2
    assert endpoint["reference_change"] == 1
    assert report["changes_on_common_coverage"]["altitude_m"]["rmse"] == 1
    assert report["full_overlap"]["after"]["altitude_m"]["covered_duration_s"] == 3


def test_unavailable_channels_remain_null_and_no_endpoint_is_invented():
    a = sample()
    a["altitude_m_quality_eligible"][:] = False
    a["altitude_m_score_eligible"][:] = False
    r = channel_summary(a, "altitude_m", 4)
    assert r["rmse"] is None
    assert r["fraction_of_available_observations"] is None
    assert common_endpoint(a, a, "altitude_m")["status"] == "unavailable"


def test_reproduction_rejects_different_termination_or_nonfinite_states():
    keys = ("t", "r_ned", "v_ned", "q_bn", "w_body", "mass", "altitude_m", "speed_mps")
    base = {k: np.ones(3) for k in keys}
    changed = {k: v.copy() for k, v in base.items()}
    assert reproduction_errors(base, changed)["passed"]
    changed["t"] = np.ones(2)
    assert reproduction_errors(base, changed)["reason"] == "shape_changed:t"
    changed = {k: v.copy() for k, v in base.items()}
    changed["mass"][1] = np.nan
    assert not reproduction_errors(base, changed)["passed"]
