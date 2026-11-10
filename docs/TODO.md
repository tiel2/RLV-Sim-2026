# RLV-Sim Development TODO

Status as of 2026-11-10. Scope: complete [Project 2](../materials/RSLV_GNC_Project2.pdf) through independently initialized representative flight scenarios and documented sensor/navigation analysis. Project 2 p.5 asks for selected representative phases; a continuous launch-to-touchdown mission is an optional extension.

**Current status:** The five-phase public-flight comparison and conditional model checks are documented in the [report](verification/public_flight_comparison_report_ko.md). Sensor-based INS/ESKF and matched navigation-error analysis are implemented as a synthetic research model; their hardware fidelity and report integration remain open.

**Sequence:** Select representative physics experiments → Define engineering initial/boundary conditions and prescribed inputs → Verify dynamics and numerical agreement → Inspect physical histories and rendering → Document assumptions and useful variants → Implement and compare sensor-derived navigation → Complete the report, including a scoped public-flight-data comparison when its conventions and input correspondence are understood.

**Physics milestone baseline (2026-11-03):** Demonstrate five `project2_*` phases on rotating Earth: four prescribed physics experiments plus `scenario=project2_landing` with closed-loop LCvx/PD control. Inspect trajectories, loads, attitude/rates, propellant and actuator response; require soft touchdown for landing. See [current results](verification/project2_segments/physics_results.md). CRS-11 comparison is opt-in background and RMSE reduction is not a physics acceptance criterion. Terminal powered descent uses a 20 s burn and 2 s shutdown observation; that experiment does not require touchdown. Landing uses ideal actuation, truth navigation, rotation-aware nominal planning gravity and separate lateral/vertical tracking gains. Additional variants should exercise useful physics within model limits. Synthetic sensor/navigation analysis is implemented as a separate research workflow and remains required for final Project 2 reporting; it does not constitute calibrated flight sensors or a rotating-Earth navigation solution. Continuous full-flight recovery remains optional.

## Completed Work

- [x] Independently verify the existing 6-DOF equations, mass properties, and canonical motion cases.
- [x] Verify engine, TVC, grid-fin, RCS, actuator, atmosphere, and body-aerodynamic components.
- [x] Verify coupled motion and timestep agreement with ideal actuation and averaged RCS.
- [x] Complete the [physical implementation verification report](verification/physics_implementation_report_en.md) and remove redundant explanatory files.
- [x] Independently integrate the 150 s attached-stack ascent with realistic actuators; verify mass and open-system linear/angular momentum balances and rigid-pad support.
- [x] Verify ascent, single-engine landing, and fuel-depletion rendering in Chrome, including reverse seeking, engine count, thrust axes, stack geometry, and pad alignment.
- [x] Document these implemented capabilities, reproducible checks, and remaining scope in the [project guide](3_project_guide_ko.md#ascent-physics).
- [x] Connect ascent to impulse-free stage separation at an assumed 150 s and retained-booster coast to 160 s; verify release balances, independent coast dynamics, configuration replay, and guide.
- [x] Connect separation → RCS attitude maneuver → three-engine boostback → actual shutdown → coast in `ascent_boostback`; preserve original fuel budgets. Sequence completes, but return-target projection fails and main-fuel reserve triggers cutoff.
- [x] Implement and execute four independent `project2_*` physics experiments; full-segment equations, 2/1/0.5 ms comparisons and body-table checks pass. Keep optional CRS-11 analysis separately from physics acceptance. Terminal experiment: 20 s burn plus 2 s shutdown observation.
- [x] Enable spherical rotating Earth in the four prescribed Project 2 experiments at an assumed reference latitude of 28.6°; use central gravity, radial altitude/contact and consistent inertial/reference angular rates. Verify full Kepler orbits, inertial energy/angular momentum, asymmetric torque-free rotation, pad/contact behavior and full-phase inertial-frame reference agreement. Landing is extended below. See [report Section 11](verification/physics_implementation_report_en.md#11-spherical-rotating-earth-verification).
- [x] Provide `scenario=project2_landing` as the fifth rotating-Earth phase, derived from PDG with ideal actuation and truth navigation. Include surface centrifugal acceleration in nominal planning gravity and separate lateral/vertical tracking gains. Verify soft touchdown from 300 m, a small lateral-IC perturbation, the five-case checker and replay. Preserve `+baseline=pdg` as the separate fixed-Earth reference.

Flight scenarios cover post-flip reentry, descent, and landing, plus attached-stack ascent → MECO → short coast and ascent → separation → booster-coast and ascent → attitude maneuver → boostback experiments. Preserve these connected experiments as verification/regression cases; connecting all of them is not a Project 2 completion criterion. Identity navigation remains the default; synthetic IMU/GNSS/radar navigation and the aided ESKF are available as an explicit research configuration.

**Status convention:** Existing capabilities are listed separately from pending scenario qualification and optional integration. An unchecked task does not mean its underlying physics or controller is unimplemented. Independent initialization is explicit and allowed; state continuity is required within each scenario and across any events it actually includes. Existing segment implementations do not establish flight-data validation or a validated full-flight scenario.

## 1. Define the Landing Frame, Initial Conditions, and Assumptions

The selected five phases use Earth-fixed Cartesian axes aligned with NED at their spherical surface origin; altitude/contact are radial. The selected landing target is position `[0, 0, 0]`, velocity `[0, 0, 0]`. The older PDG baseline retains flat local NED. Independent prescribed recovery segments use their starting point's ground projection as a horizontal origin when geographic position is unavailable, as declared in the [scenario specification](reference/project2_scenario_spec.md). Such a local origin is not an observed landing target and does not support a flight-referenced horizontal touchdown-error score. Record frame definitions and any reference mapping explicitly.

- [x] Select **CRS-11 stage 1** as the reference flight; preserve the commit-pinned webcast data and record observables, coverage, provenance, quality flags, and uncertainty limitations in the [flight-reference specification](reference/crs11_flight_reference.md).
- [ ] For each selected reference window, resolve the altitude datum, speed frame, and any coordinates, heading or event timing needed by that scenario. Exact separation timing is needed for a flight-referenced separation case, not every experiment. Define numerical uncertainty bounds where supported; retain unknowns explicitly.
- [x] Specify the five selected phases' independent initial states, duration/end condition, propellant, actuator state and configuration in the [scenario specification](reference/project2_scenario_spec.md), distinguishing the four prescribed-input experiments from the rotating-Earth closed-loop landing. Source classification and uncertainty limitations are explicit; flight validation remains pending.
- [x] Define provisional ascent initial conditions, parameter classifications, and their limits in the [ascent specification](reference/crs11_ascent.md). Exact geographic/datum reconstruction remains open above.
- [x] Represent the attached upper stage as a fixed equivalent component during ascent; retain its mass/CG/inertia separately in configuration. Impulse-free runtime separation and first-stage-only coast are implemented; downstream recovery handoffs are optional.
- [ ] Classify vehicle, environment, and input parameters as **Known / Estimated / Assumed**.

**Completion criterion:** Each selected independent scenario has a reproducible initial state, frame/target definition, phase/event scope, observables, end condition and principal modeling assumptions. Its initial state need not be generated by an upstream simulation.

**Optional reference:** CRS-11 stage 1 provides webcast altitude/scalar speed over T+0–464.630 s. It is context for plausible scales and a later scoped comparison, not complete state truth or a mandatory runtime schedule. Current experiments use independent engineering durations/inputs. See the [reference specification](reference/crs11_flight_reference.md); no isolated navigation-error record is supplied.

## 2. Qualify Vehicle and Environment Models for Each Scenario

- [x] Configure provisional ascent mass, propellant, tanks, CG, and inertia in `plant/vehicle=falcon9_ascent`; retain recovery configurations separately. Values are documented assumptions, not measured CRS-11 properties.
- [x] Account for attached upper-stage mass, CG, and inertia separately from main propellant, using an equivalent dry-body combination and the parallel-axis theorem.
- [x] Add a separate nose-first aerodynamic configuration and numerical lookup domain; keep recovery's engine-first reference axis and coefficients.
- [ ] Establish empirical aerodynamic validity and uncertainty bounds for both configurations; the assumed ascent coefficient table is not flight-validated.
- [x] Distinguish rigid-pad hold/release from ground-contact termination and exclude initial pad samples from touchdown assessment.
- [x] Replace the rotating mode's flat/vertical-gravity approximation with spherical central-gravity propagation; verify orbital and attitude mechanics and quantify rotation sensitivity in the selected experiments. Empirical limitations (spherical Earth, no J2/gravity-gradient torque) remain explicit; older fixed-mode scenarios retain their declared approximations.
- [ ] For each selected segment, check vehicle mass/propellant consistency, thrust and RCS authority, aerodynamic incidence/Mach domain, atmospheric altitude range and applicable actuator/deployment assumptions. Use documented hardware estimates rather than changing limits solely to match an event time.

**Completion criterion:** Each selected scenario's mass budget, propulsion/control authority, aerodynamics, environment and initial conditions are internally consistent and their empirical limitations are stated.

## 3. Qualify Independent Representative Flight Scenarios

**Selected Project 2 demonstration set (five phases):** powered ascent, entry burn, aerodynamic descent, terminal powered descent, and closed-loop landing, all selected with `scenario=project2_<phase>` and all using rotating Earth. Their purposes, local durations, inputs and engineering initial states are in the [scenario specification](reference/project2_scenario_spec.md). The first four use prescribed inputs; landing uses ideal actuation and requires soft touchdown. Generic cases and the original fixed-Earth PDG baseline remain available. Cutoff/separation is supporting verification; coast/boostback/full-flight connection are additional cases. Default runs do not compare against CRS-11.

### Reference windows and reusable implementations

The table below records optional historical CRS-11 windows and reusable capabilities, not the current experiment schedule or required coverage. Current prescribed durations are 150 / 14 / 45 / 22 s; the independent 300 m landing ends at contact, with a 40 s maximum. No observed CRS-11 window is assigned to this landing baseline. The archived observations end at T+464.630 s. Preserve local/reference time mapping only when explicitly comparing.

| Segment / reference window [s] | Existing reusable implementation | Work for an independent reference-based scenario |
|---|---|---|
| **Powered ascent: 0–145** | [ascent](../configs/scenario/ascent.yaml): attached-stack mass, nose-first aerodynamics, pad release, prescribed nine-engine inputs; verified through 150 s including MECO/coast. | Review ascent inputs and initial/vehicle assumptions; compare altitude and scalar speed over the selected ascent window. |
| **Cutoff / separation / short coast: 145–161** | [ascent_separation](../configs/scenario/ascent_separation.yaml): assumed zero-impulse separation at 150 s and booster coast to 160 s; event balances verified. | Exact separation/maneuver timing is absent from the archive. Declare event assumptions and independently initialize a segment if selected; do not label 150 s as measured separation. |
| **Boostback: 161–211** | [ascent_boostback](../configs/scenario/ascent_boostback.yaml): verified connected maneuver/restart/cutoff experiment; current ignition is 335.18 s and return projection fails. | Add an independently initialized boostback scenario if selected. Review RCS/pointing, engine inputs, mass/fuel and initial-state feasibility; existing experiment is not a reconstruction of this reference window. |
| **Post-boostback coast: 211–372; apogee marker 248** | Native ballistic/rotational propagation, canonical freefall/rotation checks and existing short coast segments. | Configure an independently initialized coast segment if selected. Model the rise and fall without requiring an upstream burn; treat apogee as an observable, not an extra burn phase. |
| **Entry burn: 372–384** | [entry_burn](../configs/scenario/entry_burn.yaml): independent prescribed-input experiment; reentry guidance also exists. | Qualify initial conditions, burn inputs and environment for a selected entry window. Existing generic inputs/duration are not CRS-11 reconstruction. |
| **Aerodynamic descent: 384–429** | [aero_descent](../configs/scenario/aero_descent.yaml), fin-response/deployment experiments and existing aero guidance. | Qualify descent state, fin exposure, aerodynamic validity and selected observation window; existing short tests are component/segment experiments. |
| **Terminal descent / landing burn: 429–462** | [powered_descent](../configs/scenario/powered_descent.yaml): prescribed burn; [pdg](../configs/scenario/pdg.yaml): closed-loop landing. | Select a defensible initial state and fuel budget. Distinguish prescribed burn verification, flight-data comparison and successful closed-loop touchdown. Burn-end marker 462 s is not an exact measured touchdown. |

The raw altitude/speed observations cover these intervals, but do not measure each phase's complete state or directly resolve separation/attitude/deployment events. Existing [reentry_to_landing](../configs/scenario/reentry_to_landing.yaml) remains an optional connected recovery experiment with its own initial conditions; a connected successful landing is not required for every segment. Fin deployment remains an instantaneous aerodynamic-exposure approximation, without deployment dynamics or mass/inertia changes.

### Immediate scenario work

- [x] Select five representative phases: powered ascent, entry burn, aerodynamic descent, terminal powered descent, and the existing PDG landing (confirmed by the user on 2026-11-03). Selection is separate from implementation/validation completion.
- [x] Maintain the [five-phase specification](reference/project2_scenario_spec.md): purpose, applicable reference window/time mapping, frame/target, complete nominal initial state/configuration, inputs, end condition, parameter provenance and verification criteria. Landing has an engineering initialization and no flight-reference mapping. Flight-accuracy pass/fail limits remain unassigned pending uncertainty evidence.
- [x] Extract boundary observations and reconstruct velocity with 2/4/6 s trailing-quadratic sensitivity; reject gaps, inconsistent components and zero-altitude plateaus. Record assumed heading/attitude, mass/fuel and inputs separately. See [reproducible analysis](../scripts/analyze_project2_reference.py), [recipe](../configs/analysis/project2_segments.yaml) and [results](../results/verification/project2_segments/reference_analysis.json). No measured complete 6-DOF truth is claimed.
- [x] Add separate `project2_*` YAML using shared runners and common recovery hardware assumptions; preserve generic experiments and compare observations without claiming a calibrated flight reconstruction.
- [x] Check the prescribed event timelines and continuously propagated state/fuel/actuators against configuration-only full-segment integration. These runs do not construct GNC or reset state to reference samples; controller-memory handoffs are not exercised.
- [x] Export truth, loads, propellant, events, labels and replay under `results/project2_physics/<case>/`; place generated checks/configurations/plots under `results/verification/project2_segments/physics/`. Generated evidence and historical runs are local ignored outputs; retain source, authored settings and concise reports in Git.
- [x] Produce reproducible uncalibrated comparisons and plots for all four reference windows after implementation checks. See [results](../results/verification/project2_segments/runtime_results.md); large ascent error and terminal incidence-domain failure remain explicit.
- [x] Set CRS-11 comparison OFF by default in all four runtime scenarios and the checker; use explicit `scenario.reference.enabled=true` or `--compare-flight` for optional analysis. Keep RMSE outside physics acceptance.
- [x] Define terminal powered descent as a 20 s burn / 22 s experiment. Verify continued descent, deceleration, actual shutdown and body-table applicability; no touchdown requirement. Preserve the earlier 35 s experiment as historical evidence.
- [ ] Add initial-state/input/model variants only where they improve the physical demonstration or assess relevant assumptions. Reject invalid mass/fuel or model-domain configurations. Fitting to CRS-11 is not a current objective.

**Completion criterion:** All five selected phases run reproducibly from documented engineering initial conditions and have interpretable physical histories. The four prescribed experiments pass their equation/numerical/body-table/rendering checks without a touchdown requirement. The separate landing regression must achieve soft touchdown and pass bookkeeping/domain checks. Neither CRS-11 matching nor full-flight connection is required for this milestone. Later public-data comparison and sensors/navigation remain part of final Project 2 reporting.

### Optional extension: continuous full-flight recovery

The existing ascent/separation/maneuver/boostback connections are completed capabilities, preserved above and in the verification reports. The following work is outside the current Project 2 scenario milestone:

- [ ] Establish full-flight return feasibility across ascent trajectory, attitude maneuver timing and main/RCS fuel allocation before claiming return capability.
- [ ] Connect actual boostback exit state through recovery-oriented attitude/coast, entry burn, fin deployment, descent and landing without standalone-state resets.
- [ ] Extend orchestration, controller handoffs, transition records, physics checks and replay across the entire sequence; distinguish sequence completion from target return and soft-landing success.
- [ ] Add nonzero separation impulse or removed-body propagation/contact only if required by the selected extension.

## 4. Verify the Dynamics Before Flight-Data Comparison

- [x] Compare the full 0–150 s ascent, realistic engine cutoff, and attached-stack coast against independent DOP853 integration using rotation matrices and mass quadrature. Check reference-tolerance refinement.
- [x] Audit open-system mass and linear/angular momentum balances using recorded loads; halve diagnostic sampling and verify decreasing integration residuals. Include momentum carried by consumed propellant and the modeled angular momentum flux.
- [x] Record rigid-pad reaction force/moment and exact native release time; test fixed pose/rates with evolving fuel and actuators while supported.
- [x] Test attached inert mass/CG/inertia, actual per-engine forces, fuel cutoff, and replay metadata. Ascent milestone suite: 321 passed; after separation extension: 352 passed on 2026-11-02.
- [x] Check mass, inertia, material-point continuity, and two-body momentum balances across impulse-free stage separation. Independently integrate the retained coast and compare 1/0.5 ms full runs; see [separation results](../results/verification/separation_physics_results.json).
- [x] Check ascent/separation/maneuver/boostback/coast boundaries for state resets, propellant budgets and aligned restart.
- [ ] Optional: extend recorded-load momentum/flow integral audits to the recovery experiments if further conservation evidence is needed. Full-segment independent integration, mass bookkeeping and shutdown checks are already complete; cross-scenario handoffs remain an optional full-flight extension.
- [x] Assess realistic-actuator and PWM operation in the attitude/boostback experiment with independent local integrations and 1/0.5 ms runs; hold PWM command resolution fixed. Latest suite: 363 tests; browser: 18 checks (2026-11-02). See [boostback results](../results/verification/boostback_physics_results.json) and [report Section 8](verification/physics_implementation_report_en.md#8-attitude-maneuver-and-boostback-connection).
- [x] Independently integrate all four selected prescribed segments, including realistic throttle/TVC and exposed neutral fins, with reference-tolerance refinement and 2/1/0.5 ms native comparisons. Check the declared body-table domain as part of physics acceptance. See [current verification summary](../results/verification/project2_segments/physics/summary.json).

**Evidence:** [Ascent physics results](verification/ascent_physics_results.json), [verification report](verification/physics_implementation_report_en.md) and [current physics results](verification/project2_segments/physics_results.md). After rotating-Earth landing was added, the full suite passed **400 tests**, Ruff passed, all five phase checks passed and **25 WebGL checks** passed on 2026-11-03. Earlier counts above belong to their named historical campaigns. These are declared-model implementation checks, separate from empirical vehicle fidelity or navigation accuracy.

**Completion criterion:** Each implemented extension obeys its declared equations, event balances, and numerical tolerances. Unimplemented phases remain explicit.

## 5. Verify Rendering and Maintain the Project Guide

- [x] Render the attached stack separately from the recovery booster; use configured engine positions/counts and all gimbal axes.
- [x] Drive plumes from actual per-engine force, including fuel-depletion shutdown; preserve stowed fins and reverse-seek state.
- [x] Show a schematic land launch pad and frame the launch site; document the cosmetic ground offset separately from contact physics.
- [x] Run six browser checkpoints across ascent, single-engine landing, and fuel depletion; record screenshots and transform/state assertions.
- [x] Update the Korean project guide with dynamics, verification results, rendering contracts, commands, and implemented/deferred distinctions.
- [x] Extend rendering and guide for separation: per-frame configuration, both event limits, reverse seeking, and engine-point continuity.
- [x] Extend rendering and guide for the connected attitude/boostback experiment, including failed target projection.
- [x] Show recorded RCS cold-gas activity; remove chase-camera position lag and sky/terrain depth overlap. Replay regressions: 21 tests; combined browser run: 26 checks, including 1x/8x/32x tracking and backward seeking.
- [x] Fix the remaining T+05:36 chase-background flicker by subdividing the giant terrain mesh. Add 335–338 s pixel comparisons at 60 Hz for orbit/chase, with the former two-triangle mesh as a diagnostic control; static scene checks alone did not catch it.
- [x] Make RCS usage prominent in Actuators with an ON/OFF badge, active count, maximum duty and per-nozzle indicators. Enlarge/brighten gas puffs and verify visible pixel changes even at 5% duty; combined browser run: 28 checks.
- [x] Verify all five current phase replays at initial/middle/final times and backward seeks: **25 WebGL checks** passed. Preserve [browser checks and hashes](../results/verification/project2_segments/physics/browser_checks.json) and screenshots. This establishes recorded pose/actuator agreement, not empirical vehicle fidelity or spherical scenery/contact geometry.

**Completion criterion:** Rendered geometry and actuator states agree with recorded simulation state, and the guide describes only implemented and verified behavior.

## 6. Public-Flight Comparison

- [x] Preserve CRS-11 observations and define units, time mapping, quality exclusions and initialization rules in the [reference](reference/crs11_flight_reference.md).
- [x] Run five independently initialized final configurations; compare altitude/speed and carry recovery mass budgets between phases.
- [x] Check model constraints, independent integration, time-step sensitivity and [assumption sensitivity](verification/public_flight_sensitivity_ko.md).
- [x] Document final settings, errors and per-phase commands in the [report](verification/public_flight_comparison_report_ko.md).
- [x] Show an observed-altitude line in the existing animator only for flight-comparison runs.

**Scope and limitations:** Inputs were fitted within a small parameter set; aerodynamic-descent fins remain at zero deflection. The nominal ideal-actuator landing reaches 0.998 m/s, but 6.072 s of eligible speed observations are not reproduced. Reduced thrust and unchanged realistic actuators can cause a crash. These results do not establish full observation tracking, real-flight accuracy or independent predictive validation. Further fitting/control changes are deferred by user choice.

Existing `project2_*` physics experiments keep their separate acceptance criteria. Public-flight comparison is opt-in and does not change ordinary runs.

## 7. Sensor Models and Navigation

- [x] Define position, velocity, attitude, and angular-rate estimates and select a synthetic IMU, GNSS, and radar-altimeter suite.
- [x] Define specific-force/gyro and aiding measurement records, NED/body frames, sample timestamps, availability flags, and estimator contracts.
- [x] Implement specific-force, gyro, GNSS position/velocity, and local radar-altitude equations.
- [x] Model deterministic white noise, constant bias, optional bias random walk and scale factor, the representative 200 Hz IMU / 10 Hz GNSS / 20 Hz radar rates, a zero-latency synchronous interface, GNSS dropouts/outliers, and a scheduled outage.
- [x] Implement known-initial-state alignment, strapdown propagation, and a 15-state GNSS/radar-aided ESKF.
- [x] Gate GNSS/radar innovations using configured chi-square thresholds and save innovations, acceptance flags, bias estimates, and posterior covariance.
- [x] Add `python scripts/analyze_navigation.py` to generate an identity-navigation fixed truth history on the 200 Hz IMU grid and evaluate the sensor/INS chain separately from flight-model feedback.
- [x] Generate truth-vs-INS-vs-ESKF flight-path/state plots, navigation errors with component-wise ±3σ bounds, and normalized innovation/bias plots.

The implementation is a research example, not a sensor specification. Accelerometer truth is approximated by differentiating velocity at navigation ticks (the first sample assumes zero translational acceleration); the mechanization uses a fixed-origin NED gravity approximation; initial position, velocity, attitude, and mass are known; mass is held constant; and sensor latency is fixed at zero. Truth sets the declared initial conditions and is used by sensor generation/error scoring, but is not consumed as a state update. Generated assumptions and limitations are saved with the comparison.

The default seeded landing comparison reduces position/velocity RMSE with aiding, but attitude/angular-rate RMSE increase. The mean NEES and 100% component-wise 3σ coverage show a conservative, uncalibrated covariance model; these diagnostics are recorded as results, not claimed as filter consistency.

**Completion criterion:** The estimator derives position, velocity, attitude, and angular rate from IMU/aiding measurements and explicit initialization; truth is consumed only by sensor generation and error scoring.

## 8. Unfiltered and Filtered Errors

- [x] Define the unfiltered INS-only strapdown solution as the baseline for the same IMU/GNSS/radar sensor suite.
- [x] Compare the INS-only and aided ESKF using one generated measurement sequence, identical initial state, and deterministic sensor/noise seeds.
- [x] Evaluate position, velocity, attitude, and angular-rate RMSE, maximum vector errors, per-axis RMSE, and timestamped error histories.
- [x] Exercise constant biases, optional scale-factor/random-walk errors, seeded noise, a GNSS outlier and scheduled outage, and radar availability; report RMSE, maximum errors, normalized innovations, NEES, and 3σ coverage.

**Completion criterion:** Truth/estimate histories, flight-path plots, covariance bounds, and quantitative errors are reproducible from `scripts/analyze_navigation.py`; metrics are synthetic-model results, not flight validation.

## 9. Complete the Report and Reproducible Runs

- [x] Document current ascent configuration, dynamics verification, environment, replay, assumptions, and deferred flight-data use in the project guide; preserve these historical verification records.
- [x] Document selected independent scenarios, initial conditions, reference windows, reproducible runs and comparison results. Keep remaining vehicle-fidelity/domain work and final navigation requirements explicit.
- [ ] Incorporate the synthetic sensor/navigation architecture and error-analysis results into the final verification report.
- [ ] Reuse the existing physical implementation verification report and incorporate extension verification, flight-data validation, and navigation-error analysis.
- [ ] Assess the principal modeling and navigation errors that could affect subsequent GNC design.
- [x] Provide reproducible configurations, commands, verdicts and essential figures for all five selected phases in the [specification](reference/project2_scenario_spec.md) and [results](verification/project2_segments/physics_results.md). Sensor/navigation reproducibility remains part of the later deliverable.

**Completion criterion:** Code, configurations, results, and the report describe the same models and experimental conditions.

## Maintenance Principles

- Keep generated JSON/CSV/NPZ, plots/screenshots/PDFs and resolved run configurations under ignored `results/`, including `results/verification/`. Keep source code, authored YAML, tests, raw reference snapshots and manual reports in Git. Existing committed evidence is historical; new executions should not populate `docs/verification/` with generated files.
- Implement features in shared RLV modules and existing runners; use general flight or physics-experiment names for scenarios.
- Reuse existing phase physics, controllers, event records, and replay support. Track implemented capabilities, pending scenario qualification and optional integration separately; do not mark an entire phase unimplemented because full-flight integration remains open.
- Keep independent scenario initialization explicit. Require continuity within each scenario, but do not make upstream-generated initial states or successful end-to-end return a prerequisite for Project 2 segment demonstrations.
- Consolidate verification equations and result discussions in the English verification report; maintain task status and completion criteria here.
- Preserve existing functionality and verification coverage. Treat fidelity extensions unused by the selected mission as follow-up work.
