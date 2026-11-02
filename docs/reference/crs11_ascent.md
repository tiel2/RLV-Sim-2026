# CRS-11 ascent configuration and assumptions

The first integration milestone is implemented as `scenario=ascent`: a pre-ignited
attached stack departs the pad, follows prescribed throttle/gimbal commands, receives
MECO at T+145 s, and propagates to T+150 s. There is no stage-separation event yet.
The last five seconds exercise cutoff and unpowered propagation of the **attached stack**;
they are not a reconstruction of the separated first stage.

Current priority is physics verification → rendering → project-guide documentation.
Flight-data validation and parameter fitting are deferred, and comparison is disabled by
default. The [reference-data contract](crs11_flight_reference.md) remains available for that
later work. Independent dynamics results are in
[report Section 6](../verification/physics_implementation_report_en.md#6-ascent-physics-and-rendering-verification).
Section 5 preserves the earlier, uncalibrated flight-data comparison as a historical record.

## Reproduce

```bash
python -m pip install -e .
python scripts/run_single.py scenario=ascent hydra.run.dir=results/ascent_physics_replay
python scripts/check_ascent_physics.py
python scripts/check_convergence.py scenario=ascent hydra.run.dir=results/ascent_convergence
```

The usual resolved configuration, physics/actuator records, input events, trajectory,
plots, and replay are exported. Only an explicit `scenario.reference.enabled=true` additionally
exports `flight_comparison.json`, `flight_comparison.npz`, and `plots/flight_comparison.png`.
That comparison JSON contains
MECO-command and final state diagnostics, including attitude, rates, mass, remaining
propellant, effective throttle/gimbal, CG, and inertia. These are diagnostic checkpoints,
not complete native restart files. No navigation or sensor model runs in this scenario.

## Evidence and boundary conditions

[NASA's launch record](https://www.nasa.gov/blogs/spacestation/2017/06/03/dragon-launches-to-station-cygnus-departs-sunday/)
identifies LC-39A as the launch site on 3 June 2017.
[JAXA's postflight overview, p. 2](https://iss.jaxa.jp/iss/flight/dragon_spx11/overview_spx11.pdf)
identifies LZ-1 as the first-stage recovery site. These sources establish the sites;
they do not establish the exact coordinates/elevations or webcast measurement conventions.

The [SpaceX preflight press kit, p. 2 (mirror)](https://spacex.com.pl/files/2017-10/crs11presskit.pdf)
lists approximate planned MECO at 142 s and separation at 145 s. The pinned webcast
event file instead assigns MECO to 145 s. Do not combine the planned separation time
with observed MECO as if they described simultaneous actual events. Actual separation
timing remains unresolved. The prescribed cutoff uses the webcast event file consistently.

| Item | Value / convention used | Classification |
|---|---|---|
| Launch / landing sites | LC-39A / LZ-1 | Known from the cited mission records |
| Time origin | Simulated liftoff at reference T+0; no fitted time offset | Assumed alignment |
| Landing-frame origin | `[0,0,0]` NED, flat local surface | Model definition |
| Initial position | `[14000,-5000,0]` m relative to landing origin | Assumed approximate displacement; not surveyed |
| Altitude datum | Launch and landing surfaces at equal height; webcast altitude compared to `-r_D` | Assumed; source datum remains unverified |
| Initial velocity / angular rate | Zero | Assumed pre-release state |
| Initial attitude | Roll/pitch/yaw = `[0,90,45]` degrees | Assumed vertical orientation and northeast tipover plane |
| Scalar speed | Ground-relative `norm(v_ned)` | Assumed source convention; not confirmed |
| Earth/environment | Fixed local NED, inverse-square vertical gravity, US-standard atmosphere, no wind/slosh | Model assumptions |
| Throttle-down / restoration / MECO | T+50 / 74 / 145 s | Published webcast timing used as input |

The position state is a point-mass trajectory reference. Pad support at `r_D=0` does
not resolve engine-plane, CG-height, landing-leg contact, pad elevation, or vehicle flexure.
The body coordinate origin used for forces and mass properties is separate from this
ground-contact convention.

## Mass, geometry, and inputs

[falcon9_ascent.yaml](../../configs/plant/vehicle/falcon9_ascent.yaml) keeps booster dry
structure and the attached component separate in configuration. The builder combines their
CG and principal inertias with the parallel-axis theorem into an equivalent inert body.
The attached component includes upper-stage propellant that is **not available to the first-stage engines**.
It stays fixed until a future separation implementation removes it.

| Budget / geometry | Assumed value |
|---|---:|
| Booster dry structure | 25,000 kg |
| Attached upper stage + its fuel + Dragon/payload | 120,000 kg |
| First-stage main propellant | 400,000 kg |
| RCS gas | 200 kg |
| Initial total | 545,200 kg |
| Stack length / diameter | 70 / 3.66 m |
| Body origin / engine plane | Stack nose / x = −70 m |
| Booster structure CG / attached CG | x = −58 / −14 m |
| LOX / RP-1 density | 1230 / 810 kg/m³ |
| Tank radius; usable lengths | 1.8 m; 25 / 14 m |

These are engineering assumptions, not a measured CRS-11 mass reconstruction. The existing
settled dual-tank model computes CG/inertia as fuel is consumed. All component coordinates
use the stack nose origin; the booster nose is at x = −28 m. Separation will require an
explicit body-origin/state mapping and the correct recovery geometry.

Nine assumed 914 kN-vacuum engines use 311 s vacuum Isp and 0.68 m² effective nozzle exit
area each. All nine engines gimbal. Initial throttle is 1, representing ignition before T+0;
the startup sequence is not reconstructed. The assumed throttle fraction is 0.8 from
50–74 s and 1 otherwise until cutoff. A +0.002 rad pitch-gimbal pulse from 10–12 s
initiates a gravity turn. There is no feedback or inferred flight attitude input.
The realistic actuator model delays effective shutdown after the 145 s command.

[ascent.yaml aerodynamics](../../configs/plant/aero/ascent.yaml) uses a nose-first `+x`
reference axis and separate assumed coefficients, rather than the recovery base-first table.
Its numerical table spans Mach 0–12 and incidence 0–180 degrees; this is a lookup domain,
not an empirically validated envelope. Fins remain stowed. Coefficients, CP, thrust,
mass, heading, and control inputs have not been fitted to CRS-11.

## Pad and event behavior

`scenario.ic.launch_pad=true` requires zero initial altitude, velocity, and angular rate.
While held, the integrator constrains position/attitude and rates; actuators and propellant
continue evolving. Release occurs once the free vertical acceleration is upward at a native
step boundary. The release latch is part of the native state; subsequent ground contact
terminates normally instead of reattaching the rocket to the pad. Existing scenarios default
to the original free-flight mode.

This is a rigid support approximation. While held, `rhs` constrains the motion and diagnostics
record balancing `F_pad_body` and `M_pad_body` separately from free vehicle loads.
Release uses unconstrained upward acceleration. A `liftoff` event records the native release
time (one plant-step resolution) separately from `detected_at_s`, the observation endpoint.
Initial pad samples are excluded from touchdown assessment; a horizon reached while held is
not a landing. Replay offsets the schematic ground to the engine plane for visual alignment;
this does not add geometric contact or CG-height physics.

## Limits and next work

The earlier initial run was an **uncalibrated comparison**, with no pass/fail flight-accuracy verdict.
No coefficients were adjusted using CRS-11 error metrics. This release therefore has no
calibration/holdout split yet; choose that split before fitting any inputs or coefficients.
The raw altitude/speed comparison ends at 145 s and excludes intervals across the identified
reference-data gap from time-weighted metrics. Timing-anchor agreement is not scored.

When flight validation resumes, evaluate sensitivity to thrust/mass and steering before attributing errors to aerodynamics.
The MECO flight path is too steep and the predicted speed too high in the initial run.
The scalar observations cannot uniquely identify all mass, thrust, steering, and aerodynamic
parameters. Exact launch/landing coordinates, speed frame, altitude datum, and attitude
evidence remain needed for a geographic reconstruction. Fixed vertical gravity and a flat
local frame also need an error budget over the simulated range; numerical timestep agreement
does not validate these assumptions. Next implement separation with mass/configuration
changes, then connect coast/attitude maneuvers and boostback without resetting trajectory states.
