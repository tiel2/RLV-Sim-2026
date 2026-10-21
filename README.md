# RLV-Sim

A modular 6-DOF Falcon 9 first-stage descent/landing simulator with native C++ dynamics and swappable Python GNC.

> Research software: the Falcon 9 model is approximate, and convex guidance currently assumes
> ideal thrust actuation. Realistic-actuator runs are model-mismatch experiments, not validated landings.

## Install

Requires Python 3.10+ and a C++ compiler.

```bash
python -m pip install -e ".[dev]"
```

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
reference case; a sensor-based INS/filter remains future work.

Actuator effective values are endpoint inputs after limits/lag, including ideal command tracking;
engine health multipliers and fixed nozzle cant/bias act downstream. The initial truth sample
precedes the first control application. Monte Carlo workers retain trajectory/KPI computation
and omit the dense navigation/actuator histories.
For pulsed RCS, the effective axis channel is the instantaneous endpoint pulse state, not the
mean duty requested by GNC. Individual nozzle states are available in native diagnostics.

## Navigation status and TODO

Sensor-based navigation is deferred. The working baseline is `IdentityNav`, which passes plant
truth to GNC as the estimated state. Its optional state-level noise/bias helpers are synthetic
perturbations, not IMU/GNSS/altimeter models. The navigation template also passes truth through;
selecting `gnc/navigation=ekf` raises `NotImplementedError`.

The following navigation work remains TODO:

- [ ] IMU, GNSS, and altimeter measurement models, including sensor errors and availability.
- [ ] Sensor sampling/latency contracts for asynchronous measurements.
- [ ] INS propagation, initialization/alignment, and aided state estimation (e.g. EKF).
- [ ] Sensor/estimator validation and navigation-error Monte Carlo evaluation.

The existing update schedule, truth/measurement/estimate records, and error statistics remain
available as infrastructure for this future work. The reference case continues to use identity
navigation; its zero estimation error does not validate a sensor-based navigation algorithm.

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

```bash
python -m pip install -e ".[dev,qoco]" # full suite includes Python QOCO comparisons
python -m pytest tests/ -q
ruff check src tests scripts
```

The physics update was checked on 2026-10-21: **230 tests passed**, and `ruff check src tests`
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
