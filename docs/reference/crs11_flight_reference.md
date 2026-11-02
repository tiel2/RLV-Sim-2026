# CRS-11 flight reference

Selected on 2026-11-02 for the full-flight integration work in [TODO](../TODO.md).
Use CRS-11 stage-1 webcast altitude and speed as the primary trajectory reference.
This selection does not complete the ascent configuration, full-flight implementation,
initial-state reconstruction, or flight-data validation. A provisional ascent → MECO →
attached-stack coast scenario is now available; see the [ascent specification](crs11_ascent.md)
for its explicit assumptions and unresolved reference conventions.

## Source and reproducibility

Source: [shahar603/Telemetry-Data](https://github.com/shahar603/Telemetry-Data),
commit `b245d3b81aa36b7941ec10f3f4b508999d106a6d`.
The source describes telemetry extracted from public SpaceX launch webcasts, not onboard
sensor logs. Its launch catalogue identifies the CRS-11 analysed stage as stage 1.

The [local snapshot](../../data/reference/crs11/manifest.json) records upstream paths,
commit-pinned download URLs, file sizes, SHA-256 hashes, and retrieval date.
Upstream data, README, launch catalogue, and [Unlicense](../../data/reference/crs11/LICENSE)
are preserved byte for byte. `stage1 raw.json` is locally named `stage1_raw.json`.
Generated audit output is separate from the upstream files.

| File | Content | Use |
|---|---|---|
| [stage1_raw.json](../../data/reference/crs11/stage1_raw.json) | 13,489 samples, T+0–464.630 s; time [s], speed [m/s], altitude [km] | Primary altitude/speed observations |
| [analysed.json](../../data/reference/crs11/analysed.json) | 465 samples, T+0–464 s at 1 s spacing; calculated trajectory channels | Auxiliary trend comparison only |
| [events.json](../../data/reference/crs11/events.json) | Named event times [s], rounded to the nearest second according to the upstream README | Prescribed timing inputs, with their role recorded |
| [stages.json](../../data/reference/crs11/stages.json) | Beginning of stage-specific webcast telemetry | Stage attribution; not a physical separation event |

Reproduce the offline integrity/structure checks and quality flags from the repository root:

```bash
python3 scripts/check_flight_reference.py
```

The command writes [quality_report.json](../../data/reference/crs11/quality_report.json).
It does not clean data, reconstruct missing measurements, or certify flight-model accuracy.

## Selected phase sequence and event roles

Use ascent → cutoff → separation/coast/attitude maneuver → boostback → coast →
reentry burn → aerodynamic descent → landing burn → touchdown. Boostback is required
for this reference mission. Separation and maneuver timing still require additional evidence.

| Event | T+ [s] | Role in the initial prescribed-input reconstruction |
|---|---:|---|
| Throttle-down start/end | 50 / 74 | Timing anchors; throttle magnitudes remain unknown |
| Max-Q | 65 | Comparison landmark if predicted independently; not a phase trigger |
| MECO | 145 | Prescribed ascent cutoff |
| Boostback start/end | 161 / 211 | Prescribed burn window |
| Apogee | 248 | Comparison landmark if predicted independently; not a phase trigger |
| Entry burn start/end | 372 / 384 | Prescribed burn window |
| Landing burn start/end | 429 / 462 | Prescribed burn window; end is not an exact touchdown measurement |
| Stage separation / fin deployment | Unavailable | Obtain separately; do not substitute `stages.json` or MECO |

These are reference inputs for a future runnable scenario, not an implemented engine schedule.
If an event is prescribed, matching its time is not independent validation. A later
condition-triggered model may compare its predicted event time with this reference.

## Quality and uncertainty

- Raw timestamps are strictly increasing. The largest interval is 1.302 s between
  T+70.870 and 72.172 s. Mark missing coverage; do not bridge it silently when resampling.
- Approximately 30 samples per second describe extraction cadence, not independent
  measurements or the original instrument/display update rate. Quantization, repeated
  display values, extraction errors, and timing latency require consideration.
- Derived vertical speed exceeds total speed by more than 0.01 m/s at 52 analysed epochs.
  At T+462 s, total speed is 4.282 m/s while vertical speed is −6.053 m/s. These flags
  indicate that the channels must not be assembled directly into a truth state vector.
  The 0.01 m/s threshold is an audit setting, not an uncertainty estimate.
- Derived downrange ends at −6.534 km. Establish its definition and sign before comparison;
  it is not automatically distance to the landing target or the simulator's unsigned
  horizontal displacement from its initial position.
- The source does not supply numerical measurement uncertainty. Event rounding alone
  contributes up to about 0.5 s, with additional extraction/timing errors unquantified.
  Do not assign sensor-grade error bars or exact touchdown speed from the displayed zeros.
- Derived acceleration, angle, and `q` need their computation and conventions checked
  before quantitative use. The upstream README labels aerodynamic pressure with `N`;
  do not silently treat this unit label as a verified pressure definition.

## Comparison contract

1. Preserve raw observations and keep preprocessing/masks in separate artifacts. Use
   stage-1 data explicitly rather than a generic file that could mix stages on other missions.
2. Align the simulated launch epoch with T+0. Convert altitude from km to m. Resolve
   the source altitude datum before comparing with `-r_ned[2]` in the landing frame;
   record any datum offset. Resolve the speed reference frame before comparing with
   `norm(v_ned)`. Do not silently fit time/height offsets to minimize validation error.
3. Compare only overlapping, valid coverage. Interpolate the simulated trajectory to
   observation epochs; do not extrapolate observations or invent data across gaps.
   Record sampling/weighting so dense repeated webcast samples do not imply extra evidence.
4. Report altitude and speed error histories, bias, RMSE, and maximum absolute error per
   phase, together with coverage and preprocessing. Report event-time errors only for
   events predicted independently. Define acceptance limits after assessing uncertainty.
5. Treat attitude, angular rate, propellant, aerodynamic loads, and actuator histories as
   simulation diagnostics unless an independent measurement is obtained. Webcast telemetry
   is not IMU/GNSS truth for validating INS/EKF performance.
6. If parameters are fitted to CRS-11, call those results calibration. Freeze parameters
   before evaluating a separately selected flight or withheld interval. Record the split
   before fitting; another mission must be audited before adopting it as a holdout.

## Inputs still needed

| Classification | Current evidence / remaining task |
|---|---|
| Known as published observations | Stage-1 altitude, scalar speed, timestamps, and named event times, with the limitations above |
| To obtain from additional sources | Launch/landing coordinates and elevations, flight heading, separation timing, and speed/altitude conventions |
| To estimate with documented bounds | Vehicle and attached-stage mass, propellant budget, thrust/throttle schedules, and aerodynamic coefficients |
| To state as model assumptions | Unobserved attitude/rate initial conditions, separation impulse, wind, and actuator/deployment approximations |

Do not populate a complete NED initial state from scalar speed or derived downrange alone.
The existing standalone recovery initial conditions remain engineering experiments until
the selected mission's upstream trajectory supplies their handoff state.
