# Comparison of Public CRS-11 Flight Data and Simulation

[한국어](public_flight_comparison_report_ko.md)

Five phases were configured using public CRS-11 flight data. The resulting trajectories were compared with the observations and checked against the model constraints. Altitude RMSE across the phases was approximately 51–599 m, and speed RMSE was 11–66 m/s. Landing succeeded under specific initial conditions and ideal actuation, but the later part of the observed landing speed history was not reproduced.

The comparison uses trajectories calculated from observation-informed initial conditions and assumed simple commands. No trajectory optimization or observation-tracking controller was developed to jointly design the flight direction, attitude, and command history of each phase to minimize observation errors.

## 1. How the trajectories were constructed

The data consists of time, altitude, and speed values extracted from the CRS-11 webcast display. It comes from an actual flight, but it is not a raw sensor log. Planned trajectories, target states, control inputs, and complete position and attitude information are unavailable. The [source investigation](../reference/crs11_data_provenance_ko.md) and [observation and initialization rules](../reference/crs11_flight_reference.md) document the evidence and processing assumptions.

1. **Initial altitude and total speed:** Samples at the start of each phase were used, with interpolation between adjacent valid samples where necessary. The observations were assumed to use the same altitude datum and Earth-relative speed convention as the model. Simulation time zero was aligned with the start of the phase. The actual source altitude datum and speed reference frame remain unconfirmed.
2. **Other initial states:** Vertical velocity was estimated from altitude changes, and horizontal speed was calculated from total speed. Horizontal heading, position, vehicle attitude, and angular rates were assumed separately. Landing used a different method to account for the coarse low-altitude display.
3. **Model and inputs:** The vehicle model and command forms from the existing scenarios were adopted. A few throttle, gimbal-amplitude, and mass candidates were compared through repeated simulations. Position, velocity, and attitude were propagated using the translational and rotational equations of motion. The first four phases have no controller that corrects deviations from the observations; landing is controlled toward a separately defined ground target.

The five phases are **independently initialized runs**. During recovery, terminal mass and remaining RCS gas are passed to the next phase, but position, velocity, and attitude are not connected into a continuous flight. The direction and attitude shown in the animation follow the assumptions and model calculations. Public altitude and speed alone cannot verify agreement with the actual 3D path or attitude.

## 2. Final configuration

### Initial conditions and model

| Phase | Analysis window after launch [s] | Initial altitude [m] | Initial total speed [m/s] | Initial mass [kg] |
|---|---:|---:|---:|---:|
| Ascent | 0–145 | 0 | 0 | 545,200.0 |
| Entry burn | 372–384 | 47,500 | 1,260.3 | 40,000.0 |
| Aerodynamic descent | 384–429 | 35,000 | 953.9 | 34,996.8 |
| Terminal powered descent | 429–452.485 | 4,700 | 319.7 | 34,996.8 |
| Landing | Observations from 452.485–464.630 | 300 | 76.1 | 31,998.6 |

For entry burn, aerodynamic descent, and terminal powered descent, a time-weighted quadratic was fitted to altitude over the preceding **4 seconds** to estimate vertical velocity at the start. Horizontal speed was calculated as `horizontal speed = √(total speed² − vertical velocity²)`, with a northward heading assumed. Estimates with vertical speed exceeding total speed were rejected. Initial attitude was set so that the thrust axis pointed opposite to the initial velocity.

Ascent starts at rest with the vehicle upright and an initial heading of 45°. Landing starts upright with a heading of 0°. Initial roll and angular rates are zero in every phase. **Heading, initial attitude, and angular rates were chosen for the simulation; they were neither confirmed by flight observations nor selected to minimize observation errors.** Except for landing, each phase uses the ground projection of its starting point as the origin, with north and east positions set to zero. These coordinates do not reconstruct the actual latitude and longitude.

The model includes Earth rotation, altitude-dependent gravity and atmosphere, body and fin aerodynamics, and mass loss during engine operation. Recovery uses a structural mass of 25,000 kg, initial RCS gas of 200 kg, and vacuum thrust and specific impulse of 914 kN and 311 s per engine. Mass and vehicle coefficients are model assumptions rather than flight measurements. Aerodynamic coefficients were fixed, and the reference runs did not add wind, sensor noise, or slosh.

### Inputs and control

| Phase | Applied inputs | Basis and limitations of the command form |
|---|---|---|
| Ascent | 9 engines. Throttle **0.908026** at 0–50 s, **0.778307** at 50–74 s, and **0.925023** at 74–145 s. Gimbal command **0.001869 rad (approximately 0.107°)** at 10–12 s, then zero | Uses the existing ascent scenario's throttle intervals and brief pitch-kick command. The 50 and 74 s times refer to public events; the 10–12 s pitch kick is an engineering choice. Command timing and form were fixed |
| Entry burn | 3 engines, each at throttle **0.484908** for 12 s. Zero fin deflection | Assumes simple constant-throttle deceleration over the published burn window. Actual per-engine command histories are unknown |
| Aerodynamic descent | Engines OFF; fins deployed with zero deflection. Gimbal and RCS commands also zero | Calculates aerodynamic motion with neutral fins. No fin or attitude controller was configured to follow the observations |
| Terminal powered descent | 1 engine at throttle **0.436689** for 23.485 s. Zero fin deflection | Assumes constant-throttle deceleration using one engine. No constraint requires the observed altitude and speed to be reached at the end of the phase |
| Landing | 1 engine, LCvx guidance and PD control. **15°** pointing limit, ideal actuation, and navigation using the exact simulation state | Uses the existing controller to land at an assumed ground target. It does not control the vehicle to reduce errors against the observed time histories |

The first four phases use prescribed inputs with actuator response delays and rate limits. Throttle is a command between 0 and 1; it is distinct from the delivered thrust fraction after accounting for atmospheric pressure. Landing uses closed-loop control toward the configured ground target, rather than tracking the observed altitude and speed histories.

**Scope of the input-candidate comparison:** Under the [search configuration](../../configs/analysis/flight_reconstruction.yaml), only three throttle levels and one gimbal amplitude were varied for ascent. Recovery varied only the entry initial mass and the throttle levels of the two burn phases. Each run was scored by equally averaging `(altitude RMSE / 100 m)²` and `(speed RMSE / 10 m/s)²`. Recovery then averaged the scores of entry burn, aerodynamic descent, and terminal powered descent. A candidate could therefore be selected even if one phase or one observable became worse. These weighting scales were not derived from measurement uncertainty.

A Powell numerical search was performed, but the [ascent record](../../results/verification/public_flight_comparison/reconstruction_20261107/ascent_search.json) and [recovery record](../../results/verification/public_flight_comparison/reconstruction_20261107/recovery_search.json) both show termination at the respective **80- and 60-evaluation limits**, without confirmed convergence. The table lists the lowest-scoring candidates among the evaluated attempts that passed the screening checks. Command timing, full command histories, flight direction, aerodynamic coefficients, and fin control were not searched jointly. **The result cannot be interpreted as a trajectory or command history that minimizes the error against flight data.** Aerodynamic descent was affected only indirectly by the mass passed from the preceding phase.

### Landing direction and target

Altitude is displayed in 100 m increments, making short finite differences unsuitable for determining the initial landing direction. The ratio of downward speed to total speed was assumed constant over the preceding **6 seconds**. Speed was integrated to obtain an altitude history, and the midpoint of the ratio range that kept this history **within 100 m** of the observations was selected. The resulting initial horizontal speed was **1.242 m/s**, and downward speed was **76.101 m/s**. The 100 m allowance is an engineering tolerance, not a sensor error bound.

The ground target was placed **4.898 m ahead in the direction of travel**, using `horizontal speed × initial altitude / downward speed`. This assumes horizontal and vertical speeds decrease linearly over the same duration; it does not identify the actual landing site. The target was used as the origin, with the initial position 4.898 m south of it at a radial altitude of 300 m.

## 3. Differences from the observations

RMSE was calculated by interpolating simulated values to observation times and weighting errors by the intervals between adjacent valid observations. Data used for initialization, zero-altitude display values, and gaps exceeding 1.1 s were excluded from the relevant evaluation. Altitude and speed validity were assessed separately.

| Phase | Altitude RMSE [m] | Speed RMSE [m/s] | Altitude comparison duration [s] | Speed comparison duration [s] |
|---|---:|---:|---:|---:|
| Ascent | 51.277 | 28.615 | 135.635 | 143.643 |
| Entry burn | 136.654 | 25.959 | 5.939 | 5.939 |
| Aerodynamic descent | 599.301 | 66.365 | 38.939 | 38.939 |
| Terminal powered descent | 273.751 | 32.884 | 17.418 | 17.418 |
| Landing | 72.712 | 10.944 | 4.672 | 6.040 |

Ascent achieved an altitude RMSE of approximately 51 m while climbing to approximately 62 km, but speed at the last evaluated instant was about 107 m/s above the observation. Aerodynamic descent had the largest differences. Terminal powered descent ended at an altitude of 694.693 m and speed of 108.030 m/s, which do not connect to the next phase's landing initial conditions of 300 m and 76.111 m/s.

Landing covered all 4.672 s of eligible altitude observations, but only 6.040 s out of 12.112 s of eligible speed observations. The simulation ended first, so **the remaining 6.072 s of observed speed was not reproduced.** Values after simulation termination were not filled with zeros.

In the figures, black points are observations, blue lines are simulation results, and gray marks are observations excluded from scoring. Differences are calculated as simulation minus observation.

![Ascent observations and simulation](../../results/verification/public_flight_comparison/final_report_20261107/ascent/comparison.png)

## 4. Landing result and interpretation

The landing met the simulator's soft-landing criteria at **6.091 s** after phase start, with a touchdown speed of **0.998 m/s**, an error of **0.455 m** relative to the assumed target, and approximately **5.33 t** of remaining main propellant. Actual touchdown time, speed, and position are unknown, so these values are not errors against actual touchdown measurements.

![Landing observations and simulation](../../results/verification/public_flight_comparison/final_report_20261107/landing/comparison.png)

The five reference runs passed the logged model-constraint checks, independent integration checks, and time-step checks. However, changing the initial velocity direction by ±1° with the same controller failed the soft-landing criteria. Reducing thrust by 2% caused a crash at **12.54 m/s**, and applying actuator response delays and rate limits caused a crash at **24.62 m/s**. The [physical, numerical, and sensitivity assessment](public_flight_sensitivity_ko.md) records the conditions and evidence.

The checks establish **whether the calculations are consistent under the assumed initial conditions and simple commands, and how far they differ from the observations**. Matching initial conditions or passing model-constraint checks does not guarantee agreement with the subsequent actual flight. No observation-tracking controller was configured for aerodynamic descent. Connected states across phase boundaries, tracking of the complete landing observations, and soft landing with realistic actuation were also not achieved. Some inputs were selected and evaluated using the same data, so this is not independent predictive validation. Remaining errors include differences between the assumed direction and commands and those of the actual flight; there is no basis for attributing all errors to disturbances, noise, or slosh.

## 5. Running the simulations and viewing results

For each phase, run the simulation first, then open the existing animator using the next command. `--system` is the external-browser option of `view_replay.py`.

Flight-comparison runs add only an **orange dashed observed-altitude line**. It places **observed altitude at the simulated horizontal position at the same time**, rather than showing a measured 3D flight path. Zero-altitude values and missing data are not connected, and the line is not extended beyond simulation termination. Ordinary scenarios do not display it. Zoom out if the observation line lies outside the view. A `replay.json` generated before this feature was added must be regenerated once to include the line.

Ascent:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_ascent \
  hydra.run.dir=results/final_flight_repeat/ascent

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/ascent --system
```

Entry burn:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_entry_burn \
  hydra.run.dir=results/final_flight_repeat/entry_burn

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/entry_burn --system
```

Aerodynamic descent:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_aero_descent \
  hydra.run.dir=results/final_flight_repeat/aero_descent

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/aero_descent --system
```

Terminal powered descent:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_powered_descent \
  hydra.run.dir=results/final_flight_repeat/powered_descent

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/powered_descent --system
```

Landing:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_landing \
  hydra.run.dir=results/final_flight_repeat/landing

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/landing --system
```

### Result files

| Phase | Observation–simulation figures | Final metrics, settings, and checks |
|---|---|---|
| Ascent | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/ascent/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/ascent/comparison.pdf) | [Ascent results](../../results/verification/public_flight_comparison/final_report_20261107/ascent/summary.json) |
| Entry burn | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/entry_burn/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/entry_burn/comparison.pdf) | [Entry burn results](../../results/verification/public_flight_comparison/final_report_20261107/entry_burn/summary.json) |
| Aerodynamic descent | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/aero_descent/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/aero_descent/comparison.pdf) | [Aerodynamic descent results](../../results/verification/public_flight_comparison/final_report_20261107/aero_descent/summary.json) |
| Terminal powered descent | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/powered_descent/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/powered_descent/comparison.pdf) | [Powered descent results](../../results/verification/public_flight_comparison/final_report_20261107/powered_descent/summary.json) |
| Landing | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/landing/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/landing/comparison.pdf) | [Landing results](../../results/verification/public_flight_comparison/final_report_20261107/landing/summary.json) |

Figures, final metrics, settings, and checks are stored in `results/verification/public_flight_comparison/final_report_20261107/`, together with the [combined results](../../results/verification/public_flight_comparison/final_report_20261107/summary.json), [input hashes](../../results/verification/public_flight_comparison/final_report_20261107/input_manifest.json), and [generated-file hashes](../../results/verification/public_flight_comparison/final_report_20261107/artifact_manifest.json). Since `results/` is not tracked by git, these artifacts must be transferred or regenerated in another environment.

The [figure-generation script](../../scripts/plot_final_flight_comparison.py) displays only the final runs from the archived verification records. It does not run new simulations or search inputs.

```bash
.venv/bin/python scripts/plot_final_flight_comparison.py \
  --output results/verification/public_flight_comparison/final_report_repeat
```
