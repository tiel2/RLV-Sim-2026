"""Optional webcast observations for the existing replay viewer; never vehicle truth."""

import numpy as np

from validation.flight_comparison_policy import interpolate_simulation


def attach_flight_reference(doc, physics, observations, config, radius):
    """Project observed altitude onto simulated N/E, without extrapolation or gap filling.

    The comparison uses simulation altitude + altitude_offset_m against observations,
    so remove that offset when mapping observations into the simulation coordinates.
    Initialization-used observations remain visible: this is a display, not an error score.
    """
    if config is None or not config.get("enabled", True):
        return doc
    t = np.asarray(observations["simulation_time_s"])
    h = np.asarray(observations["reference_altitude_m"])
    gap = 1.1  # Same maximum interpolation/observation gap as the public-flight comparison.
    n = interpolate_simulation(physics["t"], physics["r_ned"][:, 0], t, max_gap_s=gap)
    e = interpolate_simulation(physics["t"], physics["r_ned"][:, 1], t, max_gap_s=gap)
    quality = np.asarray(observations.get("altitude_m_quality_eligible", h > 0))
    quality = quality & np.isfinite(h) & (h > 0)
    heights = h - float(config.get("altitude_offset_m", 0))
    squared = (radius + heights) ** 2 - n**2 - e**2
    valid = quality & np.isfinite(n) & np.isfinite(e) & (squared >= 0)
    # Stable sqrt((R+h)^2 - N^2 - E^2) - R: radial height differs from tangent Up.
    up = (2 * radius * heights + heights**2 - n**2 - e**2) / (
        np.sqrt(np.maximum(squared, 0)) + radius
    )
    rows = observations.get("source_row", np.arange(len(t)))
    segments, segment = [], []
    previous = None
    for i in range(len(t)):
        connected = (
            previous is not None
            and i == previous + 1
            and 0 < t[i] - t[previous] <= gap
            and rows[i] == rows[previous] + 1
        )
        if not valid[i] or not connected:
            if segment:
                segments.append(segment)
            segment = []
        if valid[i]:
            segment.append(dict(t=float(t[i]), p=[float(n[i]), float(up[i]), float(e[i])]))
            previous = i
        else:
            previous = None
    if segment:
        segments.append(segment)
    doc["flight_reference"] = dict(
        dataset=str(config.dataset),
        kind="observed_altitude_on_simulated_horizontal_position",
        measured_3d_trajectory=False,
        radius_m=float(radius),
        altitude_offset_m=float(config.get("altitude_offset_m", 0)),
        time_offset_s=float(config.time_offset_s),
        simulation_end_s=float(physics["t"][-1]),
        note="관측 고도 + 모의 수평 위치 · 실제 3D 관측 경로 아님",
        segments=segments,
    )
    return doc
