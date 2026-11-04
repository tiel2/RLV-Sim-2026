# CRS-11 flight reference

Selected on 2026-11-02; scope updated on 2026-11-03 as optional background for
independently initialized Project 2 physics experiments in [TODO](../TODO.md).
Current scenarios use engineering inputs and do not seek to reconstruct CRS-11.
Comparison is disabled by default and RMSE is not a physics acceptance criterion.
When explicitly comparing, use stage-1 webcast altitude and speed over the selected window.
The archive covers an RTLS (return-to-launch-site) flight, but a continuous
launch-to-touchdown reconstruction is an optional extension. Reference selection
does not complete initial-state reconstruction or flight-data validation.
The provisional ascent → MECO → attached-stack coast scenario is available; see
the [ascent specification](crs11_ascent.md) for its assumptions and unresolved conventions.

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

The reference flight follows ascent → cutoff → separation/coast/attitude maneuver →
boostback → coast → reentry burn → aerodynamic descent → landing burn → touchdown.
Boostback is part of this RTLS mission; reproducing every phase or connecting the
phases is not required for the selected Project 2 scenario set. Select independent
comparison windows with documented initial states and local-to-reference time mapping.
Separation and maneuver timing still require additional evidence.

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

These are reference markers for selecting comparison windows, not an implemented
CRS-11 engine schedule. Existing generic engineering scenarios are not automatically
reconstructions of these windows. If an event is prescribed, matching its time is not
independent validation. A condition-triggered model may compare its predicted event
time with this reference.

### Independent comparison windows

| Segment | Approximate reference T+ [s] | Evidence / limits |
|---|---:|---|
| Powered ascent | 0–145 | Altitude and scalar-speed observations through the MECO marker. |
| Cutoff / short coast, including separation and attitude activity | 145–161 | Altitude/speed coverage; no exact separation or flip timing/state. |
| Boostback burn | 161–211 | Burn markers and altitude/speed observations; no per-engine throttle, thrust or attitude telemetry. |
| Post-boostback coast | 211–372 | Includes ascent to and descent from apogee; apogee marker is 248 s. |
| Entry burn | 372–384 | Burn markers and altitude/speed observations. |
| Aerodynamic descent | 384–429 | Between entry-burn end and landing-burn start; fin state/history is not supplied. |
| Terminal descent / landing burn | 429–462 | Burn markers and altitude/speed observations; 462 s is not an exact touchdown measurement. |

Raw observations continue to 464.630 s. Coverage within a window does not supply
the full 6-DOF state, mass, propellant, actuator state or configuration needed to
initialize a simulation. Reconstruct or assume those inputs explicitly. Source
event times are rounded and should not be mixed with preflight planned times as
though they were one measured timeline. The table partitions the reference for
analysis; individual scenarios may use a shorter window or include adjacent events.

## Public observations, reconstructed states and simulation truth

The four Project 2 physics experiments (ascent, entry burn, aerodynamic descent and
terminal powered descent) use documented engineering initial conditions. Their
current values were partly inspired by this archive, but may be changed within
physical constraints. For a later claimed flight reconstruction, constrain initial
conditions using the available observations. Use the following distinction:

| Category | Available evidence / treatment |
|---|---|
| Webcast observations | Time, altitude, scalar speed and named event markers in the preserved archive; retain extraction/rounding and convention limitations. |
| Reconstructed quantities | Vertical/horizontal speed magnitudes and flight-path angle may be estimated from altitude/speed under explicit frame assumptions and suitable smoothing. They are derived estimates, not independent measurements or a complete NED velocity. |
| Additional estimates / assumptions | Heading, horizontal position, body attitude/rates, mass/inertia, remaining propellant, throttle/gimbal/fin histories and aerodynamic parameters need separate evidence or documented assumptions and sensitivity analysis. |
| Simulation truth | The state propagated by the declared plant model, used later to generate synthetic sensors and assess navigation error. It is not measured CRS-11 flight truth. |

The [five-phase specification](project2_scenario_spec.md) records the boundary
observations, proposed initial states/inputs, processing sensitivity and planned
verification criteria. Its [analysis recipe](../../configs/analysis/project2_segments.yaml)
is separate from runnable scenario YAML; it does not change existing experiments.
Those boundary observations and the recipe cover the four prescribed experiments. The fifth
phase, `scenario=project2_landing`, has an independent engineering initialization at 300 m;
no corresponding CRS-11 landing window or observed complete initial state is assigned to it.

Public-source check on 2026-11-03: the original
[Telemetry-Data documentation](https://github.com/shahar603/Telemetry-Data) and its
successor [Launch Dashboard API documentation](https://github.com/shahar603/Launch-Dashboard-API)
describe webcast observations and calculated trajectory channels. Their documented
channels do not provide a measured complete 6-DOF CRS-11 state, mass/fuel history or
actuator/sensor log. No such complete truth dataset was identified in this check;
this is not a claim that no additional data exists elsewhere. The API documentation
was inspected for provenance/fields; its live service was not used to replace or
extend the pinned local archive.

In `analysed.json`, `angle` is a derived velocity/flight-path angle, not body pitch;
`q` denotes the aerodynamic-pressure channel, not a quaternion. Preserve the unit
and derivation caveats below. Parameters estimated from a comparison window belong
to calibration; validate on separately declared withheld data or independent evidence.

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
The existing standalone recovery initial conditions remain engineering assumptions
until justified for a selected reference window. They may be estimated or assumed
with documented bounds; an upstream simulation is not required. Reusing a reference
sample to initialize a run is an initial condition, not an independently predicted result.
