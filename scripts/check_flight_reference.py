"""Audit the pinned CRS-11 reference snapshot offline; never modify upstream data."""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1] / "data/reference/crs11"
    manifest = json.loads((root / "manifest.json").read_text())
    for item in manifest["files"]:
        content = (root / item["file"]).read_bytes()
        if len(content) != item["bytes"] or hashlib.sha256(content).hexdigest() != item["sha256"]:
            raise ValueError(f"Snapshot integrity failure: {item['file']}")

    report = {
        "mission": manifest["mission"],
        "commit": manifest["commit"],
        "snapshot_integrity": "passed",
        "scope": "Structural checks and quality flags only; not flight-model validation.",
        "units": {"time": "s", "velocity": "m/s", "altitude": "km"},
        "thresholds": {
            "gap_flag_s": 1.1,
            "component_excess_flag_m_s": 0.01,
            "meaning": "Audit thresholds, not measurement uncertainties or acceptance limits.",
        },
        "series": {},
    }
    series = {}
    for name in ("stage1_raw.json", "analysed.json"):
        data = json.loads((root / name).read_text())
        times = data["time"]
        if len(times) < 2:
            raise ValueError(f"Insufficient samples: {name}")
        for key, values in data.items():
            if len(values) != len(times) or not all(math.isfinite(v) for v in values):
                raise ValueError(f"Invalid array: {name}:{key}")
        steps = [b - a for a, b in zip(times, times[1:], strict=False)]
        if min(steps) <= 0:
            raise ValueError(f"Non-increasing time: {name}")
        report["series"][name] = {
            "samples": len(times),
            "time_range_s": [times[0], times[-1]],
            "median_step_s": statistics.median(steps),
            "maximum_step_s": max(steps),
            "gaps_over_1_1_s": [
                {"start_s": a, "end_s": b, "duration_s": round(b - a, 6)}
                for a, b in zip(times, times[1:], strict=False)
                if b - a > report["thresholds"]["gap_flag_s"]
            ],
            "fields": list(data),
        }
        series[name] = data

    analysed = series["analysed.json"]
    excess = [
        {"t_s": t, "speed_m_s": v, "vertical_velocity_m_s": vy}
        for t, v, vy in zip(
            analysed["time"], analysed["velocity"], analysed["velocity_y"], strict=True
        )
        if abs(vy) > v + report["thresholds"]["component_excess_flag_m_s"]
    ]
    report["derived_data_flags"] = {
        "vertical_component_exceeds_speed": excess,
        "final_downrange_km": analysed["downrange_distance"][-1],
        "interpretation": (
            "Derived channels are not an independent, consistent state-vector truth. "
            "Downrange must not be equated with distance to the landing target."
        ),
    }
    events = json.loads((root / "events.json").read_text())
    report["events_s"] = events
    report["missing_reference_information"] = [
        "Stage-separation time and fin-deployment time",
        "Altitude datum and speed reference frame",
        "Launch/landing coordinates and heading",
        "Attitude, angular rates, mass/propellant and actuator histories",
        "Measurement uncertainties and original display update rate",
    ]
    output = root / "quality_report.json"
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"Snapshot integrity and array structure passed; report: {output}")
    print(f"Derived vertical-component excess flags: {len(excess)} (not a validation verdict)")


if __name__ == "__main__":
    main()
