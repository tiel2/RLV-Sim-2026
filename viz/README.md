# RLV Flight Replay — 3D Viewer

A web viewer for closed-loop and prescribed-input reusable-launch-vehicle episodes. It follows
the logged CG trajectory and attitude, using the configured engine count, positions and gimbal
axes. Actual per-engine force controls plume direction/intensity and fuel-depletion shutdown.
Recovery uses a first-stage glTF hull or procedural booster; attached-stack ascent uses separate
schematic booster and upper-stack geometry. The HUD shows flight/actuator telemetry, engine
status, recorded phases and playback controls. Closely spaced timeline labels remain available
as marker tooltips. The actuator card displays the first gimbal; all gimbals move in the 3D scene.

Launch-pad episodes use a schematic land scene at the recorded launch position. Recovery
retains the ocean/drone-ship scene. Ground/deck alignment is a visual offset to the engine plane,
not a geometric contact model. Separation switches the recorded configuration; the
removed upper body's subsequent flight is not propagated or animated.

Everything is driven by a single `replay.json` exported from the simulator. The viewer runs out of
the box with the procedural booster (**no licensing strings attached**); drop in a glTF for the
photoreal hull (see below).

## Current Project 2 phases

All five representative phases use rotating Earth. Select one argument below with the shared
runner, then open the most recent result using the same viewer command:

| Phase | Runner argument |
|---|---|
| Ascent | `scenario=project2_ascent` |
| Entry burn | `scenario=project2_entry_burn` |
| Aerodynamic descent | `scenario=project2_aero_descent` |
| Terminal powered descent | `scenario=project2_powered_descent` |
| Landing | `scenario=project2_landing` |

```bash
.venv/bin/python scripts/run_single.py scenario=project2_landing
.venv/bin/python scripts/view_replay.py --system
```

The first four use prescribed inputs; landing uses closed-loop LCvx/PD with ideal actuators and
truth navigation. Physics propagates spherical altitude/contact, but terrain/ocean scenery remains
a tangent-plane visual approximation. The current five replays pass 25 WebGL pose/actuator checks;
these do not verify physical globe rendering or landing-leg contact. See the
[scenario specification](../docs/reference/project2_scenario_spec.md) and
[current results](../docs/verification/project2_segments/physics_results.md).

## Ascent and rendering checks

Generate the three regression replays with the same native build:

```bash
python scripts/run_single.py scenario=ascent hydra.run.dir=results/ascent_physics_replay
python scripts/run_single.py +baseline=pdg run.save_plots=false hydra.run.dir=results/replay_landing_regression
python scripts/run_single.py scenario=mass_depletion run.save_plots=false hydra.run.dir=results/replay_fuel_depletion
python scripts/view_replay.py results/ascent_physics_replay
```

Optional browser assertions use Playwright and Chromium (or `--browser /path/to/chrome`):

```bash
python -m pip install playwright
python -m playwright install chromium
python scripts/check_replay_browser.py
```

The checker starts a temporary local server and inspects six WebGL checkpoints, including
reverse seeking and zero-thrust shutdown. Reports/screenshots go to
`results/visual_checks/ascent/`. Three.js imports require network access. The `?inspect` URL
flag exposes scene objects only for these explicit checks.

For independent physics verification, run `python scripts/check_ascent_physics.py`.
The [project guide](../docs/3_project_guide_ko.md#ascent-physics) separates verified model behavior,
schematic rendering and still-unimplemented full-flight events.

### Separation replay

`ascent_separation` extends the attached ascent with an assumed impulse-free release at
150 s and retained-booster coast to 160 s. Both configurations use procedural geometry
so the body origin, engine positions and fin mounts follow the same recorded configuration.
`meta.vehicles` contains the two configurations; `frame.configuration` selects one.
The replay contains the left and right limits at the same release timestamp. Binary
search selects the right limit at the event and interpolation never crosses the CG jump.
Reverse seeking restores the attached stack. The removed upper body is not propagated
or shown after release; only its release state is recorded in the event log.

```bash
python scripts/run_single.py scenario=ascent_separation hydra.run.dir=results/ascent_separation
python scripts/view_replay.py results/ascent_separation
# Also uses the three baseline replays generated above:
python scripts/check_replay_browser.py --browser /opt/google/chrome/chrome \
  --separation results/ascent_separation/replay.json --output results/visual_checks/separation
```

The extended browser run checks five separation checkpoints (including reverse seeking)
and continuity of the retained engine's world position at the event, alongside the six
baseline checkpoints. See [the guide](../docs/3_project_guide_ko.md#separation-physics).

### Attitude maneuver and boostback replay

`ascent_boostback` continues from separation through rate damping, an RCS maneuver,
aligned three-engine ignition, actual thrust shutdown and a 10 s coast. Replay uses
recorded attitude and forces; the timeline exposes each transition. This run completes
the sequence but misses the approximate return target and stops on the main-propellant
reserve guard. It is not a complete landing simulation.

```bash
python scripts/run_single.py scenario=ascent_boostback hydra.run.dir=results/ascent_boostback
python scripts/view_replay.py results/ascent_boostback
# Uses the baseline and separation replays above:
python scripts/check_replay_browser.py --browser /opt/google/chrome/chrome \
  --separation results/ascent_separation/replay.json \
  --boostback results/ascent_boostback/replay.json --output results/visual_checks/boostback
```

The combined browser run checks 28 states/continuity assertions, including RCS on/off,
reverse seeks, exhaust directions, and chase framing at 1x/8x/32x. It also checks that
the sky follows the camera without writing terrain depth. See the
[mission guide](../docs/3_project_guide_ko.md#boostback-physics) for assumptions and results.

### Cold-gas display and camera tracking

RCS nozzles use configured body-frame mounts. Large white puffs with a pale blue halo
travel opposite to each nozzle's force direction. Their brightness and size emphasize
even a 5% duty for visibility, with activity averaged over its PWM period so short
pulses remain visible at normal display rates. Off, failed and fuel-starved nozzles
stay dark. The actuator card has a dedicated RCS section: an ON/OFF badge, active/total
nozzle count, maximum recorded duty and a numbered status grid with per-nozzle duty
bars/tooltips. The numeric values follow the recorded duty without visual amplification.
The cloud shape and size are illustrative. RCS duty transitions have their own timeline
and are not interpolated or lost when pose frames are downsampled.
Browser assertions compare these HUD values to the recorded nozzle activity. They also
compare rendered pixels with and without the gas sprites at the same state, requiring
at least 500 clearly changed pixels at both 5% and higher duty in the standard chase view.

New runs export these channels automatically. To add them to an older saved run using
its `resolved_config.yaml`, `physics.npz` and `actuators.npz`, without rerunning dynamics:

```bash
python scripts/refresh_replay_rcs.py results/ascent_boostback
```

Reload the viewer after updating. Legacy JSON files without nozzle metadata still
load, but cannot show RCS jets until refreshed. The current ascent/landing/depletion,
separation and boostback regression replays have been refreshed.

Orbit and chase follow the current interpolated CG without position smoothing lag,
including scrub/reset/backward seeks. Chase initially fits the full vehicle and keeps
user orbit/zoom adjustments. The sky is camera-centered and does not write depth;
logarithmic depth and the expanded far plane stabilize distant ground and sky edges.
The ground extends beyond the viewing volume and fades before the far clip, hiding
the finite square's corners in high-altitude shots.
It is subdivided into a 64×64 grid: using just two enormous triangles made the
horizon jump during chase playback around T+05:36 despite passing static checks.

The temporal background regression renders 181 consecutive frames from 335 to 338 s
for orbit and chase, then compares a background-only pixel region between frames.
The optional legacy control reproduces the former two-triangle mesh in the same browser:

```bash
python scripts/check_replay_background.py --legacy-control
```

Reports and screenshots go to `results/visual_checks/background_temporal/`. The test
rejects a jump larger than 25/255 in any RGB channel on over 1% of the sampled background.
This supplements geometry assertions with a check of visible frame continuity.

## 1. Get a replay

**Normal workflow — every `run_single` writes its own replay.** A rollout auto-exports
`replay.json` into its Hydra run dir (`results/run_single/<date>/<time>/`), alongside `kpi.json` /
`trajectory.csv`. No separate export step, and runs never overwrite each other (disable with
`run.save_replay=false`):

```bash
PYTHONPATH=src python scripts/run_single.py gnc/scheduler=guard
#  -> results/run_single/2026-06-09/14-30-22/replay.json
```

For a custom scenario, just pass overrides to `run_single.py` (each run writes its own `replay.json`).

## 2. View it

**Point the viewer at a run** (serves the repo root, loads that run's `replay.json` straight from
its folder — nothing is copied; default = the newest replay under `results/`):

```bash
PYTHONPATH=src python scripts/view_replay.py                                 # latest run
PYTHONPATH=src python scripts/view_replay.py results/run_single/2026-06-09/14-30-22
# opens http://localhost:8000/viz/index.html?replay=/results/.../replay.json
```

**Bare viewer** (drag-drop) — serve this folder over HTTP (`file://` fetch is blocked by the
browser), then drop a `replay.json` onto the page:

```bash
cd viz && python3 -m http.server 8000
# open http://localhost:8000/
```

The page loads `?replay=<url>` if present, else the co-located `./replay.json`. With no server (or
either load failing) the loader lets you **drop a `replay.json`** onto the page (or pick one).

**Controls:** drag to orbit · scroll to zoom · `space` play/pause · `←/→` step · `C` cycle camera
(Orbit / Chase / Pad) · the bottom bar has play, reset, a scrubber, speed (0.25–4×), and the camera
toggle. Coordinate-frame geometry is intentionally kept out of trajectory replay.

### Frame Studio: standard coordinate-frame explorer

Frame Studio is intentionally separate from trajectory replay. It contains no trajectory, landing
HUD, timeline, or engine telemetry: only a spacecraft, geometric state controls, coordinate frames,
a selected plane, and vector projection. Start it directly from the repository root; no simulation
or `replay.json` is required:

```bash
uv run python scripts/view_frames.py --system
```

The initial view shows the ground-velocity frame and enables the fixed NED origin, LOS target, and
LOS ray. The five built-ins are:

- **VEL:** `x`<sub>V</sub> follows ground-relative velocity **V**<sub>G</sub>,
  `y`<sub>V</sub> = **D** × `x`<sub>V</sub>, and `z`<sub>V</sub> =
  `x`<sub>V</sub> × `y`<sub>V</sub>.
- **LOS:** `x`<sub>L</sub> points from the vehicle to the target; its other axes use the same
  direction-frame construction.
- **ENU:** fixed East, North, Up local-navigation axes.
- **NED:** fixed North, East, Down native-navigation axes.
- **BODY:** vehicle-fixed FRD axes: `x`<sub>B</sub> forward, `y`<sub>B</sub> right, and
  `z`<sub>B</sub> down.

Here **D** is local down. VEL and LOS use the same equations, vertical-line North datum, and
singularity thresholds as the native C++ direction-frame transforms. A zero direction is undefined.

The bundled Falcon 9 first stage is normalized to FRD Body axes and centered at the state origin.
Body roll φ<sub>B</sub> is an Euler attitude coordinate. Lift-vector bank σ<sub>L</sub> uses the
standard Wind-frame lift direction
−`z`<sub>W</sub> = sin(α)`x`<sub>B</sub> − cos(α)`z`<sub>B</sub>. It is the signed rotation from
the zero-bank lift-up direction to −`z`<sub>W</sub> about air-relative +`x`<sub>W</sub>. It is
undefined at zero airspeed, for vertical air-relative velocity, or when air-relative velocity is
purely lateral in Body axes. Velocity-frame axes retain their deterministic North datum in vertical
flight, but that coordinate convention is not reported as a physical bank angle.

Air-relative velocity is **V**<sub>A</sub> = **V**<sub>G</sub> − **V**<sub>W</sub>. In FRD body
components [*u*<sub>B</sub>, *v*<sub>B</sub>, *w*<sub>B</sub>], Frame Studio computes
α = atan2(*w*<sub>B</sub>, *u*<sub>B</sub>) and
β = atan2(*v*<sub>B</sub>, √(*u*<sub>B</sub>² + *w*<sub>B</sub>²)), equivalently
asin(*v*<sub>B</sub> / |**V**<sub>A</sub>|). The α/β geometry layer draws the body
`x`<sub>B</sub>–`z`<sub>B</sub> plane, the longitudinal projection, lateral residual, and the same
signed arcs. α is undefined for a purely lateral air-relative velocity; both angles are undefined at
zero airspeed.

Ground velocity **V**<sub>G</sub> and wind **V**<sub>W</sub> are independent editable 3D vectors.
Drag either arrow-tip handle to change direction and magnitude in the current camera plane; orbit the
camera and drag again to alter the remaining depth component. Manual **V**<sub>G</sub> edits preserve
**V**<sub>W</sub>. The body component *w*<sub>B</sub> is not wind velocity.

Use the local-axis gimbal in the lower-right dock for constrained CAD-style attitude editing, or
switch to direct drag for screen-relative free rotation. Dragging empty space orbits the camera.
Coordinate-frame axes use long, thin dashed shafts and compact axis tags; editable physical vectors
use solid thick shafts with circular grab handles. Panel definitions and sign conventions are kept in
delayed native hover tooltips so the controls remain uncluttered.
Slider-backed attitude, trajectory, and vehicle-position values also accept direct numeric entry.
Wind accepts direct North/Up/East components. The editable FRD values `u`<sub>B</sub>,
`v`<sub>B</sub>, and `w`<sub>B</sub> specify air-relative velocity; editing them preserves attitude
and wind, reconstructs the air-relative vector in local N–U–E with the Body-to-local attitude, then applies
**V**<sub>G</sub> = **V**<sub>A</sub> + **V**<sub>W</sub>. Speed, flight-path angle, and course remain
synchronized.
Click any two axes from one visible frame to display their plane, then project
**V**<sub>G</sub>, **V**<sub>A</sub>, **V**<sub>W</sub>, LOS, or `x`<sub>B</sub> onto it. The
original vector, in-plane projection, component construction, out-of-plane residual, and signed
components are shown together.

Frame Studio places the fixed NED origin at the LOS target and accepts the vehicle position
**r**<sub>V/O</sub><sup>NED</sup> = [*N*<sub>V</sub>, *E*<sub>V</sub>, *D*<sub>V</sub>]. The NED basis is
fixed independently of LOS; the LOS direction is derived from the relative position and points from
the vehicle to that origin. The target can be any aimpoint required by a frame definition and need
not represent a landing site. For a stable comparison view, the renderer translates the scene so the
vehicle and all vehicle-attached frames remain at the display origin. In display N–U–E coordinates,
the NED origin is therefore drawn at [−*N*<sub>V</sub>, *D*<sub>V</sub>, −*E*<sub>V</sub>]. This is a
visual translation only and does not change any relative vector, angle, or frame definition.
Rotating the vehicle changes body-frame LOS components, not the NED origin or world-space LOS ray.
The Descent, Crosswind, and Vertical scenario presets replace attitude, ground velocity, and wind
while preserving vehicle position. **Align x**<sub>B</sub> **to V**<sub>A</sub> is an attitude-only operation: it
preserves **V**<sub>G</sub>, **V**<sub>W</sub>, and body roll while setting α = β = 0. It is unavailable
at zero air-relative speed; for vertical **V**<sub>A</sub>, the existing body heading is retained.

New paper conventions can be loaded with **Import JSON**. The schema is documented in
[`viz/frames/README.md`](frames/README.md), with an importable example in
[`example_paper_frame.json`](frames/example_paper_frame.json). Frame axes are declarative expressions
over standard directions, body axes, velocity, air-relative velocity, wind, and LOS, so a new paper
normally requires a small definition rather than new rendering code.

## Frame convention

The plant works in NED (z down); `src/sim/replay.py` does the frame change **once** so the
viewer stays simple:

```
world = [North, Up, East] = [r_ned[0], -r_ned[2], r_ned[1]]      # proper rotation, det = +1
q     = body→world = q_vn ⊗ q_bn,   q_vn = [√2/2, √2/2, 0, 0]    # NED→world, +90° about X
```

Quaternions are emitted in three.js order `[x, y, z, w]`. The rocket mesh is built in **body**
coordinates (local +X = nose, base at x = −L); the pivot group carries `q` and the CG world
position, and an inner group offsets by `−x_cg` so the integrated center of mass lands exactly on
the pivot. Engine positions and grid-fin arms come straight from the plant params, so plumes/fins
sit where the physics puts them.

## Photoreal glTF model (bundled — loads automatically)

The glTF loader is **already wired** in `app.js` (`tryLoadPhotorealModel`) and the viewer ships with a
hull, so it loads on start for recovery vehicles; attached stacks and separation runs use schematic geometry.
With no GLB it falls back to the procedural booster. The bundled
default is **`viz/resources/falcon9_landing.glb`** — the Forest Katsch *Falcon 9* **landing** scene
(a first-stage booster with legs, topped by a Dragon 2 capsule on a pad). On start the loader:

1. **Isolates the first-stage booster** — hides the Dragon 2 capsule/trunk and the pad scenery
   (ground, railings, signs) **by name**, so only the booster is measured and shown.
2. **Collects the 4 landing legs** into a deploy group, hidden until the **leg-deploy trigger**, when
   they snap out. `run_single` writes that time to `meta.leg_deploy.t` as the moment the booster first
   drops below `run.leg_deploy_alt_m` (default 60 m; CLI-overridable, e.g.
   `run.leg_deploy_alt_m=40`). A null/missing event leaves the legs stowed: episode end is not
   evidence of touchdown. The procedural hull follows the same event with a discrete pose change.
   This is a cosmetic representation; leg hinge/contact dynamics are not simulated. Prescribed-input
   experiments disable leg deployment through `meta.leg_deploy.enabled=false`.
3. **Re-fits the visible booster** to the real length `L` (long axis → body +X, base at `x=-L`,
   interstage top at `x=0`), measuring a *visible-only* bounding box so the capsule/pad never skew it.
4. **Hides the procedural hull and legs** (kept as the no-asset fallback) and the GLB's static
   `grid fin*` and `m1ds` engine assembly. Dynamic fins and engine bells/plumes use the replay's
   configured count and mounting positions, so a single-engine configuration shows one engine.
   Current exports use actual per-engine force; older exports fall back to throttle channels.

### Grid-fin state and geometry

Both the GLB hull and the procedural fallback use the same animated grid fins. The bundled
asset's static folded fins are hidden so they cannot contradict the recorded state. Each fin
uses the exported mount position and effective `frame.fin[i]` steering angle (radians), plus a
separate fold hinge for `frame.fin_deployment`. The open lattice is illustrative geometry,
not the aerodynamic model or a measured hinge/structural model.

- `fin_deployment=0`: fins remain visible, folded aft along the hull; physical fin force/moment is off.
- `fin_deployment=1`: fins extend radially; steering follows the effective actuator angle.
- Deployment switches **instantaneously at the recorded event**, including when seeking backwards.
  No travel duration or deployment impulse is simulated, so the viewer adds no gradual animation.
- The actuator card reports **STOWED / DEPLOYED**; hover over a fin bar for its effective angle.
  Legacy replays without the exposure field default to deployed, matching the plant default.

`fin_response` starts deployed and commands only 0.01 rad (0.573°) on the first fin from 1 to 2 s.
The movement is deliberately small; the realistic actuator has a transient and slight overshoot.
`fin_deployment` starts stowed and switches at 1 s, with zero steering throughout. Reloading the
updated viewer fixes already-exported replays containing these states; no new physics run is needed.
Custom GLBs must identify/hide their own static fin meshes if their node names do not match `grid fin*`.

**Swapping the model.** Both Forest Katsch hulls are bundled with asset-specific licenses:
`falcon9_landing.glb` (legs, the default) is CC BY 4.0, while `falcon9.glb` (the legs-stowed flight
stack) is CC BY-NC 4.0. Attribution is shown in-app and in Credits. To fly the stowed hull, point
`MODEL.url` at `./resources/falcon9.glb`; the loader's name
filter also hides that model's second stage / fairing / M-Vac so only the first stage shows. To use a
*different* booster, find one on Sketchfab (e.g. search **"Forest Katsch Falcon 9"**; a free account is
needed to download), verify its license, save it under `viz/resources/`, and point `MODEL.url` at it.

Tuning knobs are the `MODEL` object at the top of `app.js`:

```js
const MODEL = {
  url: "./resources/falcon9_landing.glb", // landing hull (legs); ./resources/falcon9.glb = flight (stowed)
  rotation: [0, 0, -Math.PI / 2],         // align the model's long axis to body +X (Y-up model default)
  flip: false,                            // set true if the nose ends up pointing the wrong way
  credit: "Falcon 9 model: Forest Katsch (Sketchfab) · CC BY 4.0",
};
```

Most models need no tuning (the loader bounding-box-fits and centers automatically); if the rocket
lands on its side or upside-down, adjust `rotation`/`flip`. Any other CC0/CC-BY booster glTF works
too — just update `url` and `credit` (keep a visible attribution for CC-BY).

### Drone-ship model (optional)

The sea scene's drone-ship is **bundled** as `viz/resources/droneship.glb` — the CC BY 4.0 *SpaceX drone barge*
by **Yakudami** (Sketchfab), auto-fit to `DRONESHIP.lengthM` and centered on the landing target. If
the file is absent it falls back to a fully procedural barge (no asset needed). To swap in a different
model, replace `viz/resources/droneship.glb` and tune the `DRONESHIP` object at the top of `app.js`
(`lengthM`, `rotation`, `yOffset`, `sinkFrac`, `seaSinkFrac`, `credit`); if the deck loads tilted or
sunk, adjust `rotation`, `yOffset`, and the `sinkFrac`/`seaSinkFrac` seating knobs.

### Credits

- Viewer + procedural booster + procedural drone-ship: this repo (runs with no third-party asset).
- [three.js](https://threejs.org) (MIT), loaded from the unpkg CDN; `Sky`, `Water`, and
  `GLTFLoader` from its `examples/jsm` addons.
- `waternormals.jpg` — three.js example texture (MIT), bundled in `viz/resources/` so the ocean needs no CDN.
- **Replay hull — *SpaceX Falcon 9 and Dragon 2* by Forest Katsch (Sketchfab), CC BY 4.0**, bundled
  as `viz/resources/falcon9_landing.glb`. Attribution is shown in-app (bottom-right) and here.
- **Drone-ship — *SpaceX drone barge* by Yakudami (Sketchfab), CC BY 4.0**, bundled as
  `viz/resources/droneship.glb`. Attribution shown in-app and here.
- **Frame Studio booster — *SpaceX Falcon 9 Block 4.5* by Forest Katsch (Sketchfab),
  CC BY-NC 4.0**, bundled as `viz/resources/falcon9.glb`. Frame Studio isolates the first stage and
  shows the attribution when it is loaded.
- CC BY 4.0: <https://creativecommons.org/licenses/by/4.0/>.
- CC BY-NC 4.0: <https://creativecommons.org/licenses/by-nc/4.0/>.
