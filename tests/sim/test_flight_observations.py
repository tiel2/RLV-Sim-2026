"""Observation masks must preserve missing time and independent channel eligibility."""

import copy
import json
import shutil
from pathlib import Path

import numpy as np
import pytest
from omegaconf import OmegaConf

from validation.flight_observations import (
    channel_segments,
    constant_runs,
    coverage,
    phase_membership,
    prepare_observations,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def policy():
    return OmegaConf.to_container(
        OmegaConf.load(ROOT / "configs/analysis/public_flight_comparison.yaml")
    )


def test_coverage_does_not_bridge_a_short_excluded_sample_or_long_gap():
    time = np.array([0.0, 0.1, 0.2, 0.3, 2.0, 2.1])
    eligible = np.array([True, True, False, True, True, True])
    segments, connected = channel_segments(time, eligible, max_gap_s=1.1)
    np.testing.assert_array_equal(segments, [0, 0, -1, 1, 2, 2])
    np.testing.assert_array_equal(connected, [False, True, False, False, False, True])
    stats = coverage(time, time, eligible, np.ones(6, dtype=bool), max_gap_s=1.1)
    assert stats["covered_duration_s"] == pytest.approx(0.2)
    assert stats["excluded_samples"] == 1


def test_empty_and_singleton_coverage_are_not_success():
    for mask in ([False, False], [False, True]):
        stats = coverage(
            np.array([0, 1]),
            np.array([10, 20]),
            np.array(mask),
            np.ones(2, dtype=bool),
            max_gap_s=1.1,
        )
        assert stats["covered_duration_s"] == 0
        assert stats["status"] == "insufficient_coverage"


@pytest.mark.parametrize(
    "time,gap", [([0, 0], 1), ([1, 0], 1), ([0, np.nan], 1), ([0, 1], 0), ([0, 1], np.inf)]
)
def test_bad_time_or_gap_fails_closed(time, gap):
    with pytest.raises(ValueError):
        channel_segments(time, [True, True], max_gap_s=gap)


def test_boundary_sample_owned_only_by_later_phase_and_unselected_gaps_remain():
    phases = {
        "a": {"start_s": 0, "end_s": 1},
        "b": {"start_s": 1, "end_s": 2},
        "c": {"start_s": 3, "end_s": 4},
    }
    np.testing.assert_array_equal(phase_membership(np.arange(5), phases), [0, 1, -1, 2, 2])
    phases["b"]["start_s"] = 0.5
    with pytest.raises(ValueError, match="nonoverlapping"):
        phase_membership(np.arange(5), phases)


def test_equal_display_plateaus_are_split_at_gaps():
    runs = constant_runs(
        np.array([0, 0.1, 2, 2.1]), np.zeros(4), np.ones(4, dtype=bool), max_gap_s=1.1
    )
    assert [r["samples"] for r in runs] == [2, 2]
    assert [r["first_source_row"] for r in runs] == [0, 2]


def test_pinned_archive_policy_units_masks_windows_and_provenance(policy):
    report, arrays = prepare_observations(ROOT / "data/reference/crs11", policy)
    time = arrays["reference_time_s"]
    assert report["snapshot"]["integrity"] == "passed"
    assert len(time) == 13489
    assert len(report["gaps"]) == 1
    assert report["gaps"][0]["duration_s"] == pytest.approx(1.302)
    i = np.flatnonzero(time == 452.485).item()
    assert arrays["altitude_m"][i] == 300
    assert arrays["speed_mps"][i] == 76.111
    assert arrays["phase_index"][i] == 4
    assert arrays["phase_local_time_s"][i] == 0
    assert not arrays["speed_interval_eligible"][i]  # no crossing the phase boundary
    i = np.flatnonzero(time == 457.223).item()
    assert not arrays["altitude_eligible"][i]
    assert arrays["altitude_segment_id"][i] == -1
    assert arrays["speed_eligible"][i] and arrays["speed_mps"][i] == 32.5
    assert arrays["speed_eligible"][-1]  # displayed zero is flagged, not invented truth
    assert arrays["zero_speed"][-1]
    assert arrays["phase_index"][-1] == 4  # archive end included
    assert not arrays["altitude_eligible"][0]  # launch zero also has finite display resolution
    assert report["events"]["touchdown_time_s"] is None
    assert report["policy"]["observables"]["speed"]["source_frame_status"] == "unknown"
    for phase in report["phases"].values():
        assert phase["time_offset_s"] == -phase["requested_range_s"][0]
        for channel in ("altitude", "speed"):
            assert 0 < phase[channel]["coverage_fraction"] <= 1
    json.dumps(report, allow_nan=False)


def test_frozen_split_cannot_be_silently_moved(policy):
    changed = copy.deepcopy(policy)
    changed["phase_partition"]["landing_split"]["fixed_time_s"] += 1
    with pytest.raises(ValueError, match="frozen landing split"):
        prepare_observations(ROOT / "data/reference/crs11", changed)


@pytest.mark.parametrize(
    "section,key,value",
    [
        ("quality", "zero_altitude_does_not_mask_speed", False),
        ("quality", "smoothing", "moving_average"),
        ("time", "observation_resampling", "uniform"),
        ("phase_partition", "shared_boundary_owner", "earlier_phase"),
    ],
)
def test_unimplemented_processing_is_rejected_instead_of_misreported(policy, section, key, value):
    policy[section][key] = value
    with pytest.raises(ValueError, match="unsupported observation processing"):
        prepare_observations(ROOT / "data/reference/crs11", policy)


def test_raw_tampering_is_rejected_before_preparation(tmp_path, policy):
    root = tmp_path / "reference"
    shutil.copytree(ROOT / "data/reference/crs11", root)
    raw = root / "stage1_raw.json"
    raw.write_bytes(raw.read_bytes() + b" ")
    with pytest.raises(ValueError, match="integrity"):
        prepare_observations(root, policy)
