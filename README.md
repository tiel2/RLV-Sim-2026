# RLV-Sim

A modular 6-DOF Falcon 9 first-stage descent/landing simulator with native C++ dynamics and swappable Python GNC.

> Research software: the Falcon 9 model is approximate, and convex guidance currently assumes
> ideal thrust actuation. Realistic-actuator runs are model-mismatch experiments, not validated landings.

## Install

Requires Python 3.10+ and a C++ compiler.

```bash
python -m pip install -e ".[dev]"
```

This installs the runtime dependencies, including pandas for sensor CSV generation and plotting.
After updating an existing checkout, rerun this command in the project virtual environment to
install newly added dependencies.

The optional `[qoco]` extra enables QOCO for LCvx comparisons; SCvx bundles its native solver.

## Run

```bash
python scripts/run_single.py plant/actuator=ideal                      # one landing run
python scripts/run_montecarlo.py plant/actuator=ideal run=montecarlo run.n_episodes=32 # Monte Carlo
python scripts/view_replay.py                                           # latest trajectory replay
python scripts/view_frames.py                                           # coordinate-frame explorer
```

Results are written to `results/`. Commands accept Hydra overrides: use `group=name` to swap a
component and `key=value` to change a field.

```bash
python scripts/run_single.py plant/actuator=ideal gnc/guidance=scvx
python scripts/run_single.py plant/actuator=ideal scenario.dispersion.enable=true
```

The default vehicle is `plant/vehicle=falcon9_stage1`: one center engine for the `pdg` landing
scenario, analytic table aerodynamics, inverse-square gravity, and layered US-standard atmosphere.
`vehicle_set=falcon9` remains
an equivalent explicit selection. Actuators default to `realistic`; the examples above select
`ideal` for the nominal landing regressions.

The nine-engine `falcon9_octaweb` configuration supports the post-flip reentry sequence:

```bash
python scripts/run_single.py plant/vehicle=falcon9_octaweb scenario=reentry_to_landing environment/atmosphere=ussa_poly
```

This is a reentry/descent experiment, not an end-to-end landing validation. Atmosphere, wind,
thrust, actuator, slosh, and guidance choices remain available for both Falcon configurations.
Generic file-backed aerodynamic tables and geometry-based fin allocation are also retained.

A provisional ascent experiment now covers pad departure, prescribed MECO at 145 s,
and attached-stack coast to 150 s:

```bash
python scripts/run_single.py scenario=ascent
python scripts/check_ascent_physics.py
python scripts/check_convergence.py scenario=ascent
```

It uses a separately configured attached upper-stage mass and nose-first aerodynamics,
with no sensors/navigation. The independent physics check covers the full 150 s trajectory
and open-system mass/momentum balances. Replay shows the attached stack, configured engine
count and actual thrust directions on a schematic land pad. A separate `ascent_separation` experiment releases the upper stack at an assumed 150 s
and continues the retained booster to 160 s with zero separation impulse. Full-flight
recovery remains pending. Development proceeds through physics verification, rendering and
the [project guide](docs/3_project_guide_ko.md#ascent-physics), before flight-data validation.
```bash
python scripts/run_single.py scenario=ascent_separation hydra.run.dir=results/ascent_separation
python scripts/check_separation_physics.py
python scripts/view_replay.py results/ascent_separation
```

Separation preserves live actuator and fuel state, transforms the tracked CG, and updates
mass properties and body aerodynamics. Event records include both limits and the removed
body at release; only the retained booster is propagated afterward. See the
[separation guide](docs/3_project_guide_ko.md#separation-physics).

The `ascent_boostback` mission continues the same native state through an RCS attitude
maneuver, three-engine boostback, actual thrust shutdown and a 10 s coast. It preserves
the existing 200 kg RCS supply and main propellant budget. The sequence completes, but
the provisional trajectory **does not meet the return-target criterion**; the main-fuel
reserve triggers cutoff. `mission.json` distinguishes sequence completion from target
and landing success. See the [mission guide](docs/3_project_guide_ko.md#boostback-physics).

```bash
python scripts/run_single.py scenario=ascent_boostback hydra.run.dir=results/ascent_boostback
python scripts/check_boostback_physics.py
python scripts/view_replay.py results/ascent_boostback
```

CRS-11 comparison is disabled by default; the archived comparison tool remains available with
`scenario.reference.enabled=true`. See [ascent assumptions](docs/reference/crs11_ascent.md).

## CRS-11 trajectory calibration

The `flight_calibrated_*` scenarios identify bounded, time-varying inputs and uncertain
recovery parameters through native 6-DOF rollouts. They preserve the original physics
and LCvx landing regressions. These are held-in reconstructions; altitude/speed alone do
not identify the real 3D trajectory or unique actuator history. The calibrated landing
uses LCvx guidance, live-state position/velocity feedback, PD attitude control, and realistic
actuators. It reaches soft contact while covering the full eligible speed observation window.
See [method, assumptions, results, and replay commands](docs/reference/crs11_calibration_ko.md).

```bash
.venv/bin/python scripts/run_single.py scenario=flight_calibrated_ascent
.venv/bin/python scripts/view_replay.py --system
# Re-identify inputs/parameters; choose a fresh output directory:
.venv/bin/python scripts/calibrate_flight.py --output results/calibration_repeat --write-configs
.venv/bin/python scripts/verify_flight_calibration.py --output results/calibration_verified_repeat
```

## Vehicle physics (Module 4 pp.7–23)

- Gravity: `mu / (radius + h)^2` down in NED, with the corresponding quaternion-rotated body force.
- Mass: settled cylindrical LOX/RP-1 tanks, O/F = 2.56, fill-dependent CG and parallel-axis inertia.
  Total mass includes RCS gas; structural `m_dry` excludes both propellant systems.
- RCS: individual nozzle forces, couples, PWM, failures and propellant use. `plant/rcs=paired`
  provides three-axis couples; `plant/rcs=roll_four` provides four roll-only nozzles.
- Fins: four local-flow force/moment models, linear small-angle lift and quadratic drag,
  with neutral-fin subtraction when the body database already includes the fins.
- Main fuel exhaustion cuts thrust and preserves the RCS reserve; ballistic flight continues.

Tank geometry and aerodynamic coefficients remain explicit approximations, not validated Falcon 9
flight data. See [the model equations and assumptions](docs/3_project_guide_ko.md#plant).
RCS layout selection does not enable control allocation: set `gnc.allocation.rcs_enabled=true`.
The default `plant.rcs.mode=pulsed` uses a 20 ms PWM period, independently of `plant/actuator`.
Its period must be an integer multiple of `plant_dt`; `plant.rcs.mode=averaged` selects mean thrust.

```bash
python scripts/run_single.py +baseline=pdg
python scripts/run_single.py plant/rcs=paired gnc.allocation.rcs_enabled=true
python scripts/run_single.py plant/rcs=roll_four gnc.allocation.rcs_enabled=true
# Constant gravity is an explicit analytic-test approximation:
python scripts/run_single.py environment/gravity=uniform
```

## Reference case and records

```bash
python scripts/run_single.py +baseline=pdg
```

`falcon9_pdg_v2` starts at 300 m, descending at 25 m/s with mass 29,500 kg. It selects the single
engine, ideal actuation, LCvx, identity navigation, no wind/fault/slosh, no initial dispersion,
and seed 0. Version 2 uses geometric liquid tanks, separately accounted RCS gas, inverse-square
gravity and layered atmosphere; its trajectory is not numerically interchangeable with v1.
Plant/nav/control/guidance/log periods are 1/10/20/1000/100 ms. The resolved config
and manifest saved with each run identify the exact model parameters used for comparisons.

At initialization, 29,500 kg consists of 25,600 kg structure, 200 kg RCS gas and 3,700 kg main
propellant. The `fuel_margin_kg` KPI excludes RCS gas. For `dual_tank`, wet CG/inertia are derived
from tank geometry in `PlantParams.mass`; YAML `x_cg_wet` and `I_wet` are prescribed-model inputs.

Update periods and the horizon must be positive integer multiples of `plant_dt`. The scheduler
uses their greatest common divisor: 20 ms navigation and 50 ms control both run exactly, with
10 ms native blocks. `gnc.delay_ticks` always counts control periods. Trajectory times start at
the initial state and include regular log endpoints plus the native terminal time, even when
contact occurs inside a requested block.

Single runs additionally write:

| File | Meaning |
|---|---|
| `timing.json` | Exact periods, actual update times, delay, and start/end times |
| `navigation.npz` | `truth_*`, `measurement_*`, `estimate_*`, and `error_*` at every navigation update |
| `navigation_stats.json` | Navigation error RMSE and maximum magnitudes |
| `actuators.npz` | Timestamped `commanded_*`, delayed `held_*`, and plant `effective_*` inputs |
| `telemetry_schema.json` | Frames, units, sample meanings, and timestamp conventions |

Navigation records use NED position/velocity and scalar-first body-to-NED quaternions. The current
navigation seam is synchronous: measurement and estimate epochs match truth, and held estimates
between updates are not treated as fresh samples. Legacy modules without timestamps are stamped
by the loop. All records are independent copies. Identity navigation has zero error in the
reference case; `synthetic_eskf` provides the sensor-based INS/filter described below.

Actuator effective values are endpoint inputs after limits/lag, including ideal command tracking;
engine health multipliers and fixed nozzle cant/bias act downstream. The initial truth sample
precedes the first control application. Monte Carlo workers retain trajectory/KPI computation
and omit the dense navigation/actuator histories.
For pulsed RCS, the effective axis channel is the instantaneous endpoint pulse state, not the
mean duty requested by GNC. Individual nozzle states are available in native diagnostics.

## Navigation status and evaluation

The default `IdentityNav` passes plant truth to GNC. Select the research prototype
`gnc/navigation=synthetic_eskf` for synthetic IMU/GNSS/radar measurements and a 15-error-state
ESKF with position, velocity, attitude, and IMU-bias estimation:

```bash
.venv/bin/python scripts/run_single.py scenario=flight_calibrated_landing gnc/navigation=synthetic_eskf
.venv/bin/python report_assets/generate_navigation.py
```

The report campaign compares identity and ESKF feedback within two landing scenarios and
evaluates IMU-only/ESKF/GNSS-outage variants on identical truth and sensor samples.
See [navigation evidence and reproduction](report_assets/README.md#navigation-extension-10-november-2026).
IMU channels use native non-gravitational force and inertial angular rate. The ESKF
propagates central gravity, Coriolis and centrifugal acceleration, and Earth-relative attitude.
Initial alignment has 1 degree per-axis uncertainty; absent initial GNSS uses an explicit
position/velocity prior. Mass remains an ideal auxiliary measurement. The campaign includes
30 landing trials and 15 ascent replays (150 s each), startup/in-flight GNSS outages, and
NEES/NIS diagnostics. These finite trials reveal landing sensitivity and do not establish
global covariance consistency or operational reliability. The separate `gnc/navigation=ekf` placeholder still raises `NotImplementedError`.

See [Development status](docs/TODO.md) for remaining model-fidelity and qualification work.

Project 2 selects **five independently initialized phases**: ascent, entry burn, aerodynamic descent,
terminal powered descent, and landing. Their [scenario specification](docs/reference/project2_scenario_spec.md)
records engineering initial conditions, prescribed-input physics checks and the closed-loop landing
regression. Run `scenario=project2_ascent`,
`scenario=project2_entry_burn`, `scenario=project2_aero_descent`, `scenario=project2_powered_descent`
or `scenario=project2_landing`
with `scripts/run_single.py` for trajectories, loads, actuator/propellant histories and replay.
The fifth phase is a single-engine closed-loop soft landing from 300 m, with ideal actuation,
truth pass-through navigation and rotating Earth. It reuses PDG guidance/control, includes
surface centrifugal acceleration in the planning gravity and uses slower lateral tracking gains.
The historical `+baseline=pdg` remains a separate fixed-Earth regression.

```bash
.venv/bin/python scripts/run_single.py scenario=project2_landing
.venv/bin/python scripts/view_replay.py --system
```

The same viewer command opens the most recent run of any of the five phases.
All five cases enable spherical rotating Earth at an assumed reference latitude of 28.6°;
the ascent origin is the launch point. Stored `r_ned/v_ned` are Earth-fixed Cartesian axes
aligned with origin NED, with radial altitude/contact. `w_body` is reference-relative;
`omega_inertial_body` is the inertial-rate truth used by the synthetic gyro. Run
`python scripts/check_rotating_earth.py` for full Kepler orbits, inertial energy/momentum,
torque-free attitude and rotation sensitivity. See [equations and results](docs/verification/physics_implementation_report_en.md#11-spherical-rotating-earth-verification).
`python scripts/check_project2_segments.py` checks independent equations, step refinement and body-table
applicability for the four prescribed cases, plus a separate soft-touchdown/bookkeeping regression for
landing. Use `--scenario landing` to check only landing. The terminal experiment uses 20 s of burn
and 2 s of shutdown observation; touchdown is
not its acceptance criterion. See [physics results](docs/verification/project2_segments/physics_results.md).
CRS-11 is optional context. Enable `scenario.reference.enabled=true` for a single comparison, or
pass `--compare-flight` to the checker; RMSE is not a physics acceptance criterion.
`python scripts/analyze_project2_reference.py` reproduces the optional initial-state source analysis.
Generated verification records and figures go to ignored `results/verification/`; documentation links
to those local artifacts become available after running the corresponding commands. Code, authored
configurations, tests and manual reports remain in the repository.

The existing update schedule, truth/measurement/estimate records, and error statistics remain
used by the sensor-based navigation implementation. The reference case continues to use identity
navigation; its zero estimation error does not validate a sensor-based navigation algorithm.

The final submission document is [`report.tex`](report.tex), compiled to [`report.pdf`](report.pdf).
`main.tex` is a separate working document; the final report is `report.tex`. The nominal navigation
configuration uses 100 Hz IMU and a 10 ms navigation period, 10 Hz GNSS, and 20 Hz radial
height aiding. Runners reject inconsistent rates. No GNSS outage/outlier or innovation
gate is enabled in nominal settings; explicit stress experiments configure them.

The standalone comparison and its four diagnostic plots use the same physical sensor
implementation as online navigation:

```bash
.venv/bin/python scripts/analyze_navigation.py scenario=project2_landing
.venv/bin/python scripts/analyze_navigation.py scenario=project2_landing gnc/navigation=synthetic_eskf gnc.navigation.gnss_outlier_times=[4.0] gnc.navigation.gnss_gate_threshold=16.812
```

Scale factors, sensor-bias random walks, and GNSS/radar NIS gates remain optional
experiments. These options are disabled in the nominal report campaign. Comparison
metrics/plots in a standalone run are a separate replay using one common sensor sequence,
not the online estimation errors that influenced its flight trajectory.

## Layout

```text
src/native/   C++ plant and SCvx solver
src/plant/    plant configuration and initial state
src/gnc/      guidance, navigation, control, and allocation
src/sim/      rollout, KPIs, Monte Carlo, and replay export
configs/      Hydra configuration
viz/          replay viewer and Frame Studio
```

To add a GNC module, copy the relevant `template.py` and YAML pair. See
[Architecture](docs/1_architecture.md), [Adding a module](docs/2_adding_a_module.md), and the
[literature ledger](docs/reference/review/papers.md).

For a Korean guide covering code flow, component interfaces, configuration, and mathematical
models, see [코드 구조와 수학 모델](docs/3_project_guide_ko.md).

## Verify

Current verification (2026-11-10): **484 tests**, `ruff check src tests scripts`, all
five representative phase checks, standalone native-sensor comparison, online synthetic
navigation and report compilation passed. Navigation data/source hashes were regenerated
for the merged implementation. The nominal seed/ascent results reproduce the report's
recorded values; finite-seed landing sensitivity and covariance limits remain documented.

Historical physics milestone (2026-11-03, including `project2_landing`): **400 tests**, Ruff,
all **five phase checks**, and **25 WebGL state/geometry checks** passed. The rotating-Earth
landing achieved 0.086245 m/s contact speed and 0.019430 m position error. See
[current results](docs/verification/project2_segments/physics_results.md) and
[the detailed verification report](docs/verification/physics_implementation_report_en.md#12-landing-added-as-the-fifth-representative-phase).

```bash
python -m pip install -e ".[dev,qoco]" # full suite includes Python QOCO comparisons
python -m pytest tests/ -q
ruff check src tests scripts
```

Historical regression record from 2026-10-21: **230 tests passed**, and `ruff check src tests`
passed. The PDG v2 command produced a soft landing at approximately **0.4884 m/s**, with
**1,417.922 kg** main propellant remaining. The reentry command above continued after main fuel
exhaustion and reached the ground at approximately **173.5069 m/s** (`crash`, zero main fuel).
These are simulation regression records, not flight-data validation. Commands and model assumptions
are recorded in [the Korean guide](docs/3_project_guide_ko.md#extension).

## Prescribed-input vehicle experiments

The same `run_single.py` entry point can now advance the plant without constructing GNC or
navigation. Existing closed-loop and Monte Carlo commands remain available. These scenarios
are independent engineering experiments, not reconstructed Falcon 9 flights or guaranteed landings.

```bash
python scripts/run_single.py scenario=entry_burn
python scripts/run_single.py scenario=aero_descent
python scripts/run_single.py scenario=powered_descent
python scripts/run_single.py scenario=fin_deployment
python scripts/run_single.py scenario=vacuum_burn
# Sequential Hydra sweep; not Monte Carlo:
python scripts/run_single.py -m scenario=freefall_uniform,freefall_inverse_square,free_rotation,vacuum_burn,mass_depletion,drag_descent
python scripts/check_convergence.py scenario=coupled_6dof
```

All new scenarios use the same nine-engine vehicle (25,000 kg structure, initially 200 kg RCS gas).
Engine activation is prescribed per engine; it does not swap vehicle mass properties.
Initial positions have zero North/East components. Velocities below are NED, angles are degrees.

| Scenario | Altitude [m] | Velocity N/E/D [m/s] | Mass [kg] | Roll/pitch/yaw [deg] | Duration [s] |
|---|---:|---|---:|---|---:|
| `entry_burn` | 40,000 | 600 / 0 / 1,200 | 45,000 | 0 / 63.435 / 180 | 20 |
| `aero_descent` | 8,000 | 80 / 20 / 200 | 35,000 | 0 / 72 / -165.964 | 5 |
| `powered_descent` | 1,500 | 0 / 0 / 100 | 30,000 | 0 / 90 / 0 | 15 |

`fin_response` and `fin_deployment` reuse the atmospheric descent initial state.
`actuator_response` exercises throttle, TVC and RCS in vacuum; `coupled_6dof` adds initial
translation, tilt and angular rates. `freefall_uniform`, `freefall_inverse_square`,
`free_rotation`, `vacuum_burn`, `mass_depletion`, and `drag_descent` supply analytical checks.
Vacuum burn and free rotation disable gravity intentionally. The constant-drag anchor uses
an effectively constant density (exponential scale height 1e12 m), constant CD, and no fin force.
An analytical check rejects incompatible model settings; changing a verification scenario into
another experiment may require `scenario.validation.kind=none`.

`scenario.input_mode=prescribed` selects a piecewise-constant `scenario.inputs` list. Every row
has a time relative to the initial epoch and may update `throttle`, `gimbal`, `fin`, `rcs`,
`fin_deployment`, and `label`. Angles are radians. Omitted channels hold their previous values;
a scalar broadcasts to the channel shape. The first row must be at zero, subsequent times must
increase, and every event must lie on the plant grid before `t_max`. Events between regular
record/input periods split the native advance exactly. `horizon.input_dt` limits observation
block size, not the resolution of command changes. `ic.throttle0` accepts a per-engine list;
`ic.rcs_fuel` specifies the initial gas reserve included in total mass.

Fin deployment is an **instantaneous aerodynamic-exposure approximation**: 0 suppresses fin
forces/moments, 1 enables the configured fin model. Mass and inertia remain unchanged; stowed-fin
drag, hinge motion, deployment impulse, and structural dynamics are excluded. This is supported
only for body coefficient tables excluding neutral fins; incompatible incremental tables are
rejected. The event affects the plant and replay. Both hull renderers use dynamic grid fins; static
GLB fins are hidden. Stowed fins remain visible folded along the hull, and switch to the deployed
pose at the recorded event without an invented deployment duration. Landing-leg animation remains cosmetic and is disabled
for prescribed experiments. Native sessions reset to deployed fins unless the schedule specifies
otherwise.

Additional outputs:

- `physics.npz`: truth, forces/moments, dynamic pressure, Mach, alpha, CG, diagonal inertia,
  main/RCS propellant, and fin exposure at trajectory epochs.
- `events.json`: input/configuration events and detected main-fuel depletion/ground crossing.
  Fuel-depletion detection includes its observation-block uncertainty.
- `metrics.json`: execution outcome and physical extrema, separate from landing performance.
- `validation.json`: analytical errors, tolerances and `passed`/`failed`/`not_requested` status.
  Failed checks give exit code 2 after exporting results.
- `reference.npz`, `plots/physics.png`, `plots/validation.png`, and analytical comparison plots
  where applicable. Existing trajectory CSV, actuator records and replay remain available.

Downrange is horizontal distance **from the initial position**, not ground-track arclength.
Flight-path angle is positive in climb and NaN at rest. Configuration diagnostics at an event
are post-event; actuator samples are endpoint values before the next command takes effect.
No navigation or solver statistics are fabricated for prescribed runs. `kpi.json` remains a
landing assessment; `success=false` does not mean a free-fall or rotation verification failed.
Noncontact landing speed/position/tilt are null in exported KPI/replay metadata.

`check_convergence.py` compares 2/1/0.5 ms steps on shared log epochs with identical commands.
It records every resolved configuration, adjacent-step errors and explicit numerical tolerances
in `convergence.json`, plus `convergence.png`. Stochastic wind and initial dispersion are rejected.
Steps can be changed with `+convergence.steps=[0.004,0.002,0.001]`; all command times, horizons,
record periods and RCS pulse periods must remain representable. Passing checks establish numerical
agreement, not flight-data accuracy or fourth-order convergence across discontinuities.
