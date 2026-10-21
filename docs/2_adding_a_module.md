# Adding a GNC module

Guidance, navigation, and control use the same extension path: copy a runnable module and YAML pair,
rename them, implement the contract, and select the new Hydra group. `gnc/build.py` normally needs
no edit.

Sensor-based navigation is currently deferred as TODO. Identity navigation and the navigation
template pass truth through; physical sensors, INS, and filter work are listed in README.
The contracts below describe the extension interface, not an implemented sensor-based estimator.

| Seam | Module | Config | Selection |
|---|---|---|---|
| Guidance | `src/gnc/guidance/template.py` | `configs/gnc/guidance/template.yaml` | `gnc/guidance=<name>` |
| Navigation | `src/gnc/navigation/template.py` | `configs/gnc/navigation/template.yaml` | `gnc/navigation=<name>` |
| Control | `src/gnc/control/template.py` | `configs/gnc/control/template.yaml` | `gnc/control=<name>` |

The templates are executable before modification:

```bash
python scripts/run_single.py gnc/guidance=template scenario=pdg
python scripts/run_single.py gnc/navigation=template
python scripts/run_single.py gnc/control=template
```

## Construction

Each YAML file points `law._target_` to a module-level `build(ctx)` factory. The factory returns the
object used at runtime. `ctx` is a `gnc.context.GncContext`:

```python
ctx.cfg             # this module's resolved config
ctx.nominal         # derived vehicle and environment constants
ctx.params          # numeric PlantParams
ctx.active_engines  # guidance only
ctx.phase           # guidance only
```

Read only the fields the module needs. Estimated state `x_hat` provides `r_ned`, `v_ned`, `q_bn`,
`w_body`, `m`, and its epoch `t` in simulation seconds. Navigation measurements and estimates
currently share the truth epoch; the loop stamps synchronous legacy modules that omit `t` and
rejects mismatched epochs. Held estimates keep their original epoch between navigation updates.

### Physical quantities in `ctx.nominal` and `ctx.params`

The physics update changes several meanings that new GNC modules must preserve:

| Quantity | Meaning |
|---|---|
| `x_hat.m` | Total mass, including main propellant and RCS gas |
| `params.mass.m_dry` | Structural mass, excluding both propellant systems |
| `nominal["m_dry"]` | Planning lower bound: structural mass plus initial RCS gas |
| `nominal["g0"]` | Standard gravity, 9.80665 m/s², for Isp-to-mass-flow conversion |
| `nominal["gravity_ned"]` | Surface gravitation used as a fixed approximation by the convex planner |
| `params.gravity` | Inverse-square model flag, `mu` and Earth `radius` used by the plant |
| `nominal["mass_cg"](m)` | CG from the selected mass model; assumes initial RCS gas remains aboard |
| `nominal["rcs_torque_scale"]` | Three-axis nominal couple authority, not a scalar single-nozzle torque |

The plant recomputes gravity with altitude. A new guidance module should distinguish its own
gravity approximation from the plant and keep the standard Isp conversion constant separate.
The current navigation interface does not estimate RCS gas independently, so the nominal CG
and planning mass floor retain the initial gas reserve assumption.

`ActuatorCommand.rcs` remains shape `(3,)`: signed roll/pitch/yaw duties in `[-1,1]`.
The native model maps these to the selected nozzles. `roll_four` has zero pitch/yaw authority;
allocation must not divide by those zero entries. `plant/actuator=ideal` does not disable RCS PWM.

See [the mass and actuator equations](3_project_guide_ko.md#plant) for the model assumptions.

## Runtime contracts

| Seam | Required methods | Result |
|---|---|---|
| Guidance | `__call__(x_hat, t, memory)` | `(thrust_ned, memory)` or `(Command, memory)` |
| Navigation | `measure(state, key)` and `estimate(meas, act, memory)` | `Measurement` and `(x_hat, memory)` |
| Control | `__call__(x_hat, command, memory)` | `(command, memory)` with `tau_body` filled |

`initial_memory()` is optional. Use the returned `memory` for a warm start, filter state, or
integrator; return it unchanged for a stateless module.

Guidance normally returns a NED thrust vector. The framework attaches phase, active engines, and a
thrust-aligned attitude reference. Return a full `Command` when the law owns attitude or cutoff.
A guidance law may also implement `track(x_hat, t, memory)` for control-rate tracking between slower
guidance updates.

Allocation is currently a single implementation in `gnc/allocation/default.py`, not a selected
plug-in seam. Reentry and aerodynamic-descent laws are wired in `gnc/build.py`; only powered-descent
guidance is selected by `gnc/guidance=<name>`.

A compiled implementation can sit behind the same Python object through an extension module or a
thin `ctypes` wrapper. Add that boundary only when the Python implementation is actually too slow.

## Checklist

1. Copy and rename the module and config pair.
2. Set `law._target_: gnc.<seam>.<name>.build` and add only keys the implementation reads.
3. Keep the existing method signatures and return types.
4. Run the selected module once, then add a focused regression test under `tests/`.
5. Run `python -m pytest tests/ -q` and `ruff check src tests scripts`.

See [Architecture](1_architecture.md) for the complete stack and native-plant boundary.
