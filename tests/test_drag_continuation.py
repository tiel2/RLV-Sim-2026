"""Regression tests for LCvx drag-template and continuation modifications.

From repository root: python -m pytest tests/test_drag_continuation.py -q
"""
import types

import numpy as np

from gnc.guidance.convex import lcvx
from gnc.guidance.convex.guidance import LcvxGuidance


def make_bare_guidance():
    """Avoid building a full vehicle: these methods need only three parameters."""
    obj = LcvxGuidance.__new__(LcvxGuidance)
    obj.rho0 = 1.225
    obj.drag_cd_s = 10.52
    obj.scale_height = 8500.0
    return obj


def sample_plan():
    return {
        "tf": 10.0,
        "N": 2,
        "r": np.array([[0., 0., -5000.], [0., 0., -2500.], [0., 0., 0.]]),
        "v": np.array([[0., 0., 200.], [0., 0., 100.], [0., 0., 0.]]),
        "m": np.array([38000., 37000., 36000.]),
    }


def test_drag_template_normalized_time_and_fraction():
    guidance = make_bare_guidance()
    plan = sample_plan()
    # Both candidates describe the same normalized source shape. Previously,
    # the longer grid would extrapolate the final velocity (zero) for most nodes.
    short = guidance._drag_template(plan, 0.0, 10.0, 4)
    long = guidance._drag_template(plan, 0.0, 30.0, 4)
    assert np.allclose(short, long)
    assert long[-1] > 0.0
    half = guidance._drag_template(plan, 0.0, 30.0, 4, drag_fraction=0.5)
    assert np.allclose(half, 0.5 * long)


def test_drag_continuation_reaches_full_coefficient():
    guidance = make_bare_guidance()
    calls = []

    def fake_plan(self, *args, drag_ref, drag_fraction):
        calls.append((drag_fraction, drag_ref[0]["id"]))
        return {"id": len(calls)}

    guidance._plan = types.MethodType(fake_plan, guidance)
    def logged(kind, func, fraction=None):
        assert kind == "ignite_drag_refine"
        assert fraction is not None
        return func()
    result = guidance._drag_continuation(None, None, None, {"id": 0}, logged, "ignite_drag_refine")
    assert result is not None
    assert [f for f, _ in calls] == [0.25, 0.5, 0.75, 1.0]
    assert [src for _, src in calls] == [0, 1, 2, 3]


def test_drag_continuation_never_accepts_partial_drag():
    guidance = make_bare_guidance()
    attempted = []

    def fake_plan(self, *args, drag_ref, drag_fraction):
        attempted.append(drag_fraction)
        if drag_fraction > 0.6:
            return None
        return {"id": len(attempted)}

    guidance._plan = types.MethodType(fake_plan, guidance)
    result = guidance._drag_continuation(
        None, None, None, {"id": 0}, lambda kind, func, fraction=None: func(), "ignite_drag_refine"
    )
    assert result is None
    assert any(f > 0.6 for f in attempted)
    assert any(0.5 < f <= 0.6 for f in attempted)


def test_drag_fallback_scans_narrow_feasible_window(monkeypatch):
    attempts = []

    def fake_solve(*args, **kwargs):
        tf = args[8]
        attempts.append(tf)
        feasible = abs(tf - 15.0) < 0.2
        return {"status": "optimal" if feasible else "infeasible",
                "feasible": feasible, "fuel": 1.0 + abs(tf - 15.0), "tf": tf}

    monkeypatch.setattr(lcvx, "solve_pdg", fake_solve)
    result = lcvx.min_fuel_tf(
        np.array([0., 0., -100.]), np.array([0., 0., 10.]), 30000.0,
        np.array([0., 0., 9.81]), 1/3000, 100000, 200000, 25000,
        N=20, iters=4, grid_fallback=True,
        drag_seq_fn=lambda tf, n: np.zeros(n),
    )
    assert result["feasible"]
    assert abs(result["tf"] - 15.0) < 0.2
    assert result["grid_fallback_used"]
    assert len(attempts) >= 41


def test_fixed_time_drag_continuation_keeps_the_terminal_clock():
    guidance = make_bare_guidance()
    calls = []

    def fixed_plan(self, r0, v0, m0, tf, *, drag_ref, drag_fraction):
        calls.append((tf, drag_fraction, drag_ref[0]["id"]))
        return {"id": len(calls)}

    def unexpected_free_search(*args, **kwargs):
        raise AssertionError("drag refinement changed the specified terminal time")

    guidance._fixed_plan = types.MethodType(fixed_plan, guidance)
    guidance._plan = unexpected_free_search
    result = guidance._drag_continuation(
        None,
        None,
        None,
        {"id": 0},
        lambda kind, func, fraction=None: func(),
        "ignite_drag_refine",
        fixed_tf=12.145,
    )
    assert result is not None
    assert [tf for tf, _, _ in calls] == [12.145] * 4
    assert [fraction for _, fraction, _ in calls] == [0.25, 0.5, 0.75, 1.0]
