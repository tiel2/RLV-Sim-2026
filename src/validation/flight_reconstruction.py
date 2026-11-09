"""Assumption-conditioned landing direction and exactly coupled recovery mass budgets."""

from __future__ import annotations

import numpy as np
from scipy.integrate import cumulative_trapezoid


def direction_envelope(time, altitude, speed, *, epoch, span, height0, tolerance, max_gap=1.1):
    """Fit h(t)=h0+f*integral_t^epoch speed, 0<=f<=1, on the trailing support.

    Constant direction and the display envelope are engineering assumptions.
    Infeasible envelopes remain infeasible; never enlarge tolerance automatically.
    """
    time, altitude, speed = map(np.asarray, (time, altitude, speed))
    if time.ndim != 1 or altitude.shape != time.shape or speed.shape != time.shape:
        raise ValueError("invalid observation shapes")
    if (
        not np.all(np.isfinite([time, altitude, speed]))
        or np.any(np.diff(time) <= 0)
        or np.any(speed < 0)
        or span <= 0
        or tolerance < 0
        or max_gap <= 0
    ):
        raise ValueError("invalid envelope inputs")
    selected = (time >= epoch - span) & (time <= epoch)
    t, h, v = time[selected], altitude[selected], speed[selected]
    if (
        len(t) < 2
        or t[-1] != epoch
        or t[0] - (epoch - span) > max_gap
        or np.max(np.diff(t)) > max_gap
        or np.any(h <= 0)
    ):
        raise ValueError("landing envelope has a gap, excluded altitude or missing endpoint")
    integral = cumulative_trapezoid(v, t, initial=0)
    integral = integral[-1] - integral
    usable = integral > 0
    lower = float(max(0, np.max((h[usable] - height0 - tolerance) / integral[usable])))
    upper = float(min(1, np.min((h[usable] - height0 + tolerance) / integral[usable])))
    feasible = lower <= upper and abs(h[-1] - height0) <= tolerance
    fraction = (lower + upper) / 2 if feasible else None
    return dict(
        status="feasible_under_assumptions" if feasible else "infeasible_under_assumptions",
        fraction_interval=[lower, upper] if feasible else None,
        nominal_fraction=fraction,
        tolerance_m=tolerance,
        source_range_s=[float(t[0]), float(t[-1])],
        samples=len(t),
        speed0_mps=float(v[-1]),
        maximum_altitude_discrepancy_m=float(np.max(abs(height0 + fraction * integral - h)))
        if feasible
        else None,
        interpretation="constant direction and display tolerance assumed, not measurement bounds",
    )


def landing_state(fraction, speed, height, radius):
    """Origin at the engineered target; maintain radial height exactly on the sphere."""
    if not 0 < fraction <= 1 or speed <= 0 or height <= 0 or radius <= 0:
        raise ValueError("invalid landing reconstruction")
    down = speed * fraction
    horizontal = speed * np.sqrt(1 - fraction * fraction)
    distance = horizontal * height / down
    if distance >= radius + height:
        raise ValueError("forward target exceeds spherical coordinate support")
    position = [
        -float(distance),
        0.0,
        float(radius - np.sqrt((radius + height) ** 2 - distance**2)),
    ]
    return dict(
        r_ned=position,
        v_ned=[float(horizontal), 0.0, float(down)],
        forward_target_distance_m=float(distance),
        vertical_fraction=float(fraction),
    )
