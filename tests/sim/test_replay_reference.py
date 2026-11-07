"""Observed altitude is a conditional overlay, never a measured 3D trajectory."""

import copy
import json

import numpy as np
import pytest
from omegaconf import OmegaConf

from sim.replay_reference import attach_flight_reference


def data(times=None):
    t = np.arange(6.0) if times is None else np.asarray(times, dtype=float)
    return dict(
        simulation_time_s=t,
        reference_time_s=t + 372,
        reference_altitude_m=np.full(len(t), 100.0),
        reference_speed_m_s=np.full(len(t), 20.0),
        source_row=np.arange(len(t)),
    )


def config(**changes):
    return OmegaConf.create(dict(dataset="crs11", time_offset_s=-372, **changes))


@pytest.mark.parametrize("cfg", [None, config(enabled=False)])
def test_absent_or_disabled_reference_leaves_replay_unchanged(cfg):
    doc = dict(meta={"name": "ordinary"}, frames=[{"t": 0, "p": [0, 100, 0]}])
    original = copy.deepcopy(doc)
    assert attach_flight_reference(doc, None, None, cfg, 6371000) == original


def test_projection_preserves_observed_radial_height_without_modifying_vehicle():
    radius = 6371000
    physics = dict(t=np.array([0.0, 1.0]), r_ned=np.array([[40000, 20000, -1], [42000, 22000, -2]]))
    doc = dict(frames=[{"p": [40000, 1, 20000]}])
    frames = copy.deepcopy(doc["frames"])
    attach_flight_reference(doc, physics, data([0.5]), config(altitude_offset_m=10), radius)
    overlay = doc["flight_reference"]
    north, up, east = overlay["segments"][0][0]["p"]
    assert (north, east) == (41000, 21000)
    assert np.linalg.norm([north, radius + up, east]) - radius == pytest.approx(90, abs=1e-8)
    assert up != pytest.approx(90)
    assert not overlay["measured_3d_trajectory"]
    assert doc["frames"] == frames
    json.dumps(doc, allow_nan=False)


def test_zero_height_and_terminal_boundary_break_line_without_changing_observations():
    physics = dict(t=np.arange(5.0), r_ned=np.zeros((5, 3)))
    obs = data()
    obs["reference_altitude_m"][2] = 0
    obs["initialization_used"] = np.ones(6, dtype=bool)
    original = copy.deepcopy(obs)
    doc = attach_flight_reference({}, physics, obs, config(), 6371000)
    ref = doc["flight_reference"]
    assert [[p["t"] for p in s] for s in ref["segments"]] == [[0, 1], [3, 4]]
    assert "samples" not in ref
    for key in original:
        np.testing.assert_array_equal(obs[key], original[key])


@pytest.mark.parametrize("gap", ["time", "source_row", "quality", "simulation"])
def test_missing_data_is_not_connected(gap):
    obs = data([0, 1, 3, 4] if gap == "time" else [0, 1, 2, 3])
    physics = dict(t=np.arange(5.0), r_ned=np.zeros((5, 3)))
    if gap == "source_row":
        obs["source_row"][2:] += 1
    if gap == "quality":
        obs["altitude_m_quality_eligible"] = np.array([True, False, True, True])
    if gap == "simulation":
        physics = dict(t=np.array([0, 3, 4]), r_ned=np.zeros((3, 3)))
    ref = attach_flight_reference({}, physics, obs, config(), 6371000)["flight_reference"]
    expected = [[0, 1], [3, 4]] if gap == "time" else [[0, 1], [2, 3]]
    if gap == "quality":
        expected = [[0], [2, 3]]
    if gap == "simulation":
        expected = [[0], [3]]
    assert [[p["t"] for p in s] for s in ref["segments"]] == expected
