# Architecture

How the testbed is wired, and where the seams are. See [`2_adding_a_module.md`](2_adding_a_module.md) for
the step-by-step plug-in guide.

## Top-level packages (`src/`)

| Package | Role |
|---|---|
| `core/`  | **Contracts.** Frozen NumPy-backed state/command/parameter records (`interfaces.py`), native frame/quaternion conventions (`frames.py`), and deterministic seed derivation. |
| `plant/` | **Native-plant construction.** Resolves configuration, preprocesses coefficient tables, and creates initial state plus the native C++ session. The 6-DOF EoM, RK4, actuator, wind, and component models have one implementation in `native/plant.cpp`. |
| `gnc/`   | **The subsystem you plug into.** Guidance + Navigation + Control + Allocation, a phase scheduler, and the assembly (`stack.py`, `build.py`) that wires them from config. |
| `sim/`   | **The harness.** Closed-loop multirate rollout (`loop.py`), KPI scoring, Monte-Carlo aggregation, replay export, open-loop schedules. |

## The closed-loop tick

The supported vehicles are Falcon 9 first-stage approximations: `falcon9_stage1` has one center
engine for isolated landing-burn experiments, and `falcon9_octaweb` has nine engines for the
post-flip reentry sequence. The root configuration and `vehicle_set=falcon9` both select the
single-engine `pdg` scenario. Reentry runs select `plant/vehicle=falcon9_octaweb`,
`scenario=reentry_to_landing`, and `environment/atmosphere=ussa_poly` together.

Aerodynamic model infrastructure remains vehicle-independent. The default analytic tables can be
replaced with a file-backed coefficient table; `C_m` moments and incremental, geometry-based fin
models are covered by synthetic-table tests without requiring an external vehicle dataset.

`sim/loop.py` drives one integer-tick schedule for both single- and multi-rate operation. Navigation,
guidance, and control fire at their own exact periods; commands are held between control updates:

```
state (plant truth)
  │  nav.measure(state, key)         -> Measurement      (sensor model)
  │  nav.estimate(meas, act, mem)    -> x_hat            (state estimate)        ◄── NAVIGATION
  │  guidance(x_hat, t, mem)         -> Command          (thrust vector / q_ref) ◄── GUIDANCE
  │  controller(x_hat, Command, mem) -> Command.tau_body (body torque)           ◄── CONTROL
  │  allocate(Command, x_hat)        -> ActuatorCommand  (throttle/gimbal/fin/RCS)◄── ALLOCATION
  ▼  native.advance(act, n_substeps) -> timestamped endpoint state and diagnostics
```

The four GNC modules are the seams. `Command` is filled progressively: guidance sets the reference
(`thrust_vector` or `q_ref`), control adds `tau_body`, allocation turns the whole thing into actuator
deflections. The plant never sees a `Command` — only the `ActuatorCommand`.

The data types crossing each seam are defined in `core/interfaces.py`: `Measurement`, `Command`,
`ActuatorCommand`, `PlantState`, and the `*Params` structs.

## Assembly: config → modules (`gnc/build.py`)

The stack is assembled by **name**, via Hydra `instantiate`, so adding a module never touches the
wiring code:

1. `default_nominal(...)` derives vehicle / plant constants into a `nominal` dict. Controller gains
   are NOT here — each controller sizes its own in its `build(ctx)`.
2. `build_scheduler(...)` builds the latching phase scheduler (`gnc/guidance/phase.py`) from the
   `gnc/scheduler` config group.
3. For the powered-descent guidance, control, and navigation, `build_gnc` / `build_nav` construct a
   **`GncContext`** and call `instantiate(cfg.<seam>.law, ctx)`. Each module's config carries
   `law._target_: <module>.build`, a factory taking the context.

```python
ctx = GncContext(cfg=gd, nominal=nominal, params=params, active_engines=active, phase=phase)
guidance = instantiate(gd.law, ctx)        # gd.law._target_ -> build(ctx)
```

### `GncContext` — the one bundle every module receives

```python
@dataclass(frozen=True)
class GncContext:
    cfg:         # this module's resolved config node (read your own params here)
    nominal:     # vehicle/plant constants and helpers — built by gnc/build.py:default_nominal
    params:      # PlantParams (numeric vehicle/environment leaves)
    active_engines: # (N,) engines this guidance phase fires    (guidance only; else None)
    phase:       # int Phase tag this guidance instance runs in  (guidance only; else -1)
```

A factory reads only what its module needs and ignores the rest. A new guidance law and a future
navigation estimator use the same construction path with no special cases in `build.py`.

## The four seams (interface contracts)

Each seam ships a set of provided modules (siblings of the framework files under `src/gnc/<seam>/`)
and (for the config-selected seams) a config group under `configs/gnc/`; the method signature a module
must satisfy is in the table below and demonstrated in the seam's `template.py`.

| Seam | Contract (method signature) | Selected by | Provided modules |
|---|---|---|---|
| Guidance   | `__call__(x_hat, t, memory) -> (thrust_ned, memory)` (or a `Command`; + `initial_memory`) | `gnc/guidance` `law._target_` | reentry_burn, aero_descent, convex/{lcvx, scvx}, template |
| Navigation | `measure(state, key) -> Measurement`; `estimate(meas, act, memory) -> (x_hat, memory)` | `gnc/navigation` `law._target_` | identity baseline, template scaffold; EKF is TODO (selection raises) |
| Control    | `__call__(x_hat, Command, memory) -> (Command, memory)` (fills `tau_body`) | `gnc/control` `law._target_` | pd, template |
| Allocation | `allocate(Command, x_hat) -> ActuatorCommand` | fixed (`gnc/allocation/default.py`) | scheduled gimbal/fin/RCS |

`guidance` also holds the mission-sequencing framework: `phase.py` (the latching `SequentialPhaseScheduler`
+ guard factories) and `scheduled.py` (the `ScheduledGuidance` dispatcher that routes each phase to its
guidance law). Reentry/aero are fixed per-phase laws wired directly in `build.py`; the **powered-descent
method** is the config-selected one.

## Phases

`Phase` (core/interfaces.py) is a pass-through KPI tag — the plant never branches on it. The scheduler
advances `REENTRY_BURN → AERO_DESCENT → [POWERED_BRAKE] → POWERED_DESCENT` on single-crossing guards
(altitude thresholds, or a generic ψ(x; θ) ≥ 0 with `gnc/scheduler=guard`). `scenario.start_phase`
starts a run mid-sequence for isolated module testing.

## Native plant boundary and determinism

`build_plant` constructs an immutable native model once. Each episode calls `create_session(state0)`;
the session then owns the complete mutable plant state, actuator internals, slosh modes, wind filter,
and RNG. Each native block crosses the language boundary as follows:

| Direction | Per native block | Deliberately not transferred |
|---|---|---|
| Python -> C++ | one held `ActuatorCommand` and integer `n_substeps` per native block | parameters and current plant state |
| C++ -> Python | actual time, pose/velocity/mass, effective actuator inputs including RCS duty, `done`, endpoint diagnostics | servo rates, slosh, wind state/filter, RNG, RK4 stage values |

All `n_substeps` RK4 updates therefore run behind one boundary call. Sessions are separate objects,
so concurrent Monte-Carlo episodes do not share mutable state. The array-based native diagnostic
methods exercise that same C++ implementation in focused physics tests; there is no Python copy of
the time-integrated plant equations. `plant/mass_properties.py` also evaluates the settled-tank
mass geometry during construction and for GNC nominal CG; the actual state evolution stays native.

The native RNG state remains inside the session. A seed enters once through the initial state, which
makes a repeated seed deterministic without returning random keys at every substep.

`observe()` samples the current state and diagnostics at the same epoch without advancing time,
wind, or RNG. `advance()` returns that observation after integrating, possibly stopping early
inside the block. `snapshot(full=True)` remains the internal servo-state diagnostic; ideal servo
internals are inert while public actuator channels show effective clamped commands. The focused
`rhs()`/`step()` APIs retain start-state diagnostics for analytical physics tests.

## Time and record contracts

`sim/schedule.py` validates every period and the duration against `plant_dt`; off-grid or nonpositive
values raise an error. Native blocks use the greatest common divisor of nav/guidance/control/log
periods in integer plant ticks. Updates fire at `state0.t + k*period`, before integration. The last
block may be shorter to end at the horizon. No period is rounded to another GNC period.
`delay_ticks` counts control updates; startup holds the supplied initial actuator state.

`EpisodeResult.t` and `aux['per_tick'][i]['t']` identify the same truth endpoint. Logs include the
initial state, exact log-period endpoints, and the native terminal sample. Contact inside a block
is never labelled with the uncompleted block's requested end time.

`NavigationHistory` contains independent `StateHistory` records for truth, measurements, and
estimates at every navigation update, regardless of `log_dt`. `Measurement.t` is its state epoch;
the loop stamps synchronous legacy modules and rejects mismatched epochs. Estimation receives the
previously held actuator command. Between nav updates, GNC uses the held estimate with its original
epoch. Error statistics compare synchronous records and handle quaternion sign equivalence.

Sensor models, INS propagation, and aided state estimation are deferred TODOs (see README).
These records provide a validation harness; the current baseline passes truth through to GNC.

`ActuatorHistory` records each native interval `[t_start,t_end]`, the latest requested command,
the command held after transport delay, and effective endpoint inputs. `t_command`/`t_held` are
their respective creation times. Endpoint values precede any new command at that endpoint.
Engine health and fixed nozzle cant/bias remain downstream wrench effects.

`sim/telemetry.py` exports these timelines as NPZ, a JSON schema, navigation error statistics,
and timing metadata. Monte Carlo opts out of dense histories with `record_telemetry=False`,
which leaves scheduling and physics unchanged. `+baseline=pdg` selects the versioned Falcon
landing reference described in README, before introducing sensor/navigation errors.

## Vehicle physics and fuel contracts

The defaults are inverse-square gravity, layered `ussa_poly` atmosphere, geometric `dual_tank`
mass properties, and a paired three-axis RCS layout. The equations and their correspondence with
Module 4 pp.7–23 are documented in [the Korean physics guide](3_project_guide_ko.md#plant).

| Component | Configuration and state | Native result |
|---|---|---|
| Gravity | `environment/gravity`, altitude and quaternion | NED acceleration and body gravity force |
| Tanks | `plant.vehicle.tanks`, total mass, remaining RCS gas | LOX/RP-1 fill heights, CG, inertia and inertia rate |
| RCS | `plant/rcs`, three signed axis duties, remaining gas | Per-nozzle pulses, resultant force/torque and mass flow |
| Grid fins | Fin geometry, actual deflections, relative wind and body rate | Local lift/drag and moment about the current CG |

Total mass is structure plus main propellant plus RCS gas. Structural `params.mass.m_dry`
excludes both propellant systems. Main thrust stops at `m = m_dry + rcs_fuel`; its exhaustion
does not set `done`. Remaining RCS operation and ballistic flight continue until ground contact
or the loop horizon. The `fuel_margin` KPI counts main propellant only, interpolated at contact.

`plant/rcs=paired` has 12 unidirectional nozzles forming three bidirectional couples;
`plant/rcs=roll_four` has four nozzles and roll authority only. The command interface remains
three signed axis duties, not a per-nozzle vector. `mode=pulsed` uses integer-tick PWM and holds
the nozzle states throughout each RK4 step; `mode=averaged` uses continuous mean thrust.
This selection is independent of the throttle/TVC/fin actuator fidelity setting. GNC allocation
is disabled by default and requires `gnc.allocation.rcs_enabled=true`.

Endpoint diagnostics include `gravity_ned`, `F_gravity_body`, `F_rcs_body`, `M_rcs_body`,
`rcs_thruster_on`, `rcs_mdot` and `main_propellant`. They are exposed through native observation
and `EpisodeResult.aux['per_tick']`; they are not all exported as columns in `trajectory.csv`.
The public `rcs_act` channel reports axis activity at the endpoint, not pulse-averaged impulse.

The baseline is now `falcon9_pdg_v2`. Its physical assumptions and trajectory differ from v1;
the resolved configuration and the implementation version are needed for reproduction.

## Earth model

The state position and velocity are a displacement and derivative in a fixed-origin local NED frame.
The default `environment/gravity=inverse_square` uses `g(h)=mu/(radius+h)^2`, straight down in
NED. The body gravity force is `m C_bn.T @ g_ned(h)`; NED integration adds `g_ned(h)` directly.
`g_ned` in the parameters is surface gravitation, not apparent gravity. Standard gravity for Isp
conversion remains 9.80665 m/s². `environment/gravity=uniform` is an explicit analytic-test approximation.

`environment/earth=fixed` omits rotation. Selecting `environment/earth=rotating` and setting the site
latitude adds Coriolis plus centrifugal acceleration at the origin and (optionally) its displacement:

`a_rot = -2 omega_ie x v_ned - omega_ie x (omega_ie x (R_origin + chi*r_ned))`,

where `R_origin=[0,0,-radius]` and `chi` enables the differential term. Origin centrifugal acceleration
is no longer hidden in the gravity parameter. These terms execute inside the C++ RK4 stages.

This switch intentionally does not claim spherical/ellipsoidal trajectory propagation. A fixed local
NED frame has no moving-frame transport rate or Earth-curvature kinematics. Missions for which those
effects matter need an LLA/ECEF navigation state and ECEF plant propagation, rather than an additional
correction term in this local plant.
