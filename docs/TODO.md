# RLV-Sim Development TODO

Status as of 2026-11-03. Scope: complete [Project 2](../materials/RSLV_GNC_Project2.pdf) through independently initialized representative flight scenarios, followed by sensor/navigation development. Project 2 p.5 asks for selected representative phases; a continuous launch-to-touchdown mission is an optional extension.

**Sequence:** Select representative physics experiments → Define engineering initial/boundary conditions and prescribed inputs → Verify dynamics and numerical agreement → Inspect physical histories and rendering → Document assumptions and useful variants → Later implement sensors/navigation and error analysis → Complete the report, including a scoped public-flight-data comparison when its conventions and input correspondence are understood.

**Current priority:** Use the four `project2_*` scenarios as approximate independent physics experiments: inspect trajectories, loads, attitude/rates, propellant and actuator response under documented engineering inputs. See [current physics results](verification/project2_segments/physics_results.md). CRS-11 comparison is opt-in background and RMSE reduction is not a current acceptance criterion. Terminal powered descent now uses a 20 s burn and 2 s shutdown observation within its body-table domain; touchdown is not required. Additional scenario variants should exercise useful physics within model limits. Sensor/navigation remains later work by user choice and is required for final Project 2 completion. Continuous full-flight recovery remains optional.

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

Flight scenarios cover post-flip reentry, descent, and landing, plus attached-stack ascent → MECO → short coast and ascent → separation → booster-coast and ascent → attitude maneuver → boostback experiments. Preserve these connected experiments as verification/regression cases; connecting all of them is not a Project 2 completion criterion. Navigation provides truth pass-through and synthetic state perturbations; physical sensor models, INS, and EKF remain deferred.

**Status convention:** Existing capabilities are listed separately from pending scenario qualification and optional integration. An unchecked task does not mean its underlying physics or controller is unimplemented. Independent initialization is explicit and allowed; state continuity is required within each scenario and across any events it actually includes. Existing segment implementations do not establish flight-data validation or a validated full-flight scenario.

## 1. Define the Landing Frame, Initial Conditions, and Assumptions

The existing landing cases use local NED with the landing point at the origin: nominal touchdown position `[0, 0, 0]` and velocity `[0, 0, 0]`. Independent prescribed recovery segments may use their starting point's ground projection as a horizontal origin when geographic position is unavailable, as declared in the [scenario specification](reference/project2_scenario_spec.md). Such a local origin is not an observed landing target and does not support a flight-referenced horizontal touchdown-error score. Record frame definitions and any reference mapping explicitly.

- [x] Select **CRS-11 stage 1** as the reference flight; preserve the commit-pinned webcast data and record observables, coverage, provenance, quality flags, and uncertainty limitations in the [flight-reference specification](reference/crs11_flight_reference.md).
- [ ] For each selected reference window, resolve the altitude datum, speed frame, and any coordinates, heading or event timing needed by that scenario. Exact separation timing is needed for a flight-referenced separation case, not every experiment. Define numerical uncertainty bounds where supported; retain unknowns explicitly.
- [x] Specify and implement the four selected scenarios' start epoch, duration/end condition, position, velocity, attitude, angular rate, mass, main/RCS propellant, actuator state, and configuration in the [scenario specification](reference/project2_scenario_spec.md). Source classification and uncertainty limitations are explicit; flight validation remains pending.
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
- [ ] Evaluate the coordinate-frame, altitude, and gravity approximations over the selected flight domain and determine necessary extensions.
- [ ] For each selected segment, check vehicle mass/propellant consistency, thrust and RCS authority, aerodynamic incidence/Mach domain, atmospheric altitude range and applicable actuator/deployment assumptions. Use documented hardware estimates rather than changing limits solely to match an event time.

**Completion criterion:** Each selected scenario's mass budget, propulsion/control authority, aerodynamics, environment and initial conditions are internally consistent and their empirical limitations are stated.

## 3. Qualify Independent Representative Flight Scenarios

**Selected Project 2 demonstration set:** powered ascent, entry burn, aerodynamic descent, and terminal powered descent, implemented as `project2_*`. Their purposes, local durations, inputs and engineering initial states are in the [physics specification](reference/project2_scenario_spec.md). Existing generic cases and the `pdg` landing regression remain separate. Cutoff/separation is supporting verification; coast/boostback/full-flight connection are additional cases. Default runs do not compare against CRS-11.

### Reference windows and reusable implementations

The table below records optional historical CRS-11 windows and reusable capabilities, not the current experiment schedule or required coverage. Current local experiment durations are 150 / 14 / 45 / 22 s. The archived observations end at T+464.630 s. Preserve local/reference time mapping only when explicitly comparing.

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

- [x] Select the Project 2 demonstration set: powered ascent, entry burn, aerodynamic descent, and terminal powered descent (confirmed by the user on 2026-11-03). Selection is separate from implementation/validation completion.
- [x] Create the [four-scenario specification](reference/project2_scenario_spec.md): purpose, reference window, time mapping, frame/target, complete nominal initial state/configuration, inputs, end condition, parameter provenance and planned verification criteria. Flight-accuracy pass/fail limits remain unassigned pending uncertainty evidence.
- [x] Extract boundary observations and reconstruct velocity with 2/4/6 s trailing-quadratic sensitivity; reject gaps, inconsistent components and zero-altitude plateaus. Record assumed heading/attitude, mass/fuel and inputs separately. See [reproducible analysis](../scripts/analyze_project2_reference.py), [recipe](../configs/analysis/project2_segments.yaml) and [results](../results/verification/project2_segments/reference_analysis.json). No measured complete 6-DOF truth is claimed.
- [x] Add separate `project2_*` YAML using shared runners and common recovery hardware assumptions; preserve generic experiments and compare observations without claiming a calibrated flight reconstruction.
- [x] Check the prescribed event timelines and continuously propagated state/fuel/actuators against configuration-only full-segment integration. These runs do not construct GNC or reset state to reference samples; controller-memory handoffs are not exercised.
- [x] Export truth, loads, propellant, events, labels and replay under `results/project2_physics/<case>/`; place generated checks/configurations/plots under `results/verification/project2_segments/physics/`. Generated evidence and historical runs are local ignored outputs; retain source, authored settings and concise reports in Git.
- [x] Produce reproducible uncalibrated comparisons and plots for all four reference windows after implementation checks. See [results](../results/verification/project2_segments/runtime_results.md); large ascent error and terminal incidence-domain failure remain explicit.
- [x] Set CRS-11 comparison OFF by default in all four runtime scenarios and the checker; use explicit `scenario.reference.enabled=true` or `--compare-flight` for optional analysis. Keep RMSE outside physics acceptance.
- [x] Define terminal powered descent as a 20 s burn / 22 s experiment. Verify continued descent, deceleration, actual shutdown and body-table applicability; no touchdown requirement. Preserve the earlier 35 s experiment as historical evidence.
- [ ] Add initial-state/input/model variants only where they improve the physical demonstration or assess relevant assumptions. Reject invalid mass/fuel or model-domain configurations. Fitting to CRS-11 is not a current objective.

**Completion criterion:** Each selected experiment runs reproducibly from documented engineering initial conditions, exercises its declared inputs/events, has interpretable physical histories, and passes applicable equation/numerical/body-table/rendering checks. Neither CRS-11 matching, touchdown nor full-flight connection is required for this vehicle-experiment milestone. Later public-data comparison and sensors/navigation remain part of final Project 2 reporting.

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
- [ ] Apply state/event/propellant balance checks to boundaries included in each selected independent scenario; cross-scenario recovery handoffs belong to the optional full-flight extension.
- [x] Assess realistic-actuator and PWM operation in the attitude/boostback experiment with independent local integrations and 1/0.5 ms runs; hold PWM command resolution fixed. Latest suite: 363 tests; browser: 18 checks (2026-11-02). See [boostback results](../results/verification/boostback_physics_results.json) and [report Section 8](verification/physics_implementation_report_en.md#8-attitude-maneuver-and-boostback-connection).
- [x] Independently integrate all four selected prescribed segments, including realistic throttle/TVC and exposed neutral fins, with reference-tolerance refinement and 2/1/0.5 ms native comparisons. Check the declared body-table domain as part of physics acceptance. See [current verification summary](../results/verification/project2_segments/physics/summary.json).

**Evidence:** [Ascent physics results](verification/ascent_physics_results.json), [verification report](verification/physics_implementation_report_en.md) and [current physics results](verification/project2_segments/physics_results.md). The full regression suite passed 383 tests on 2026-11-03. These are declared-model implementation checks, separate from empirical vehicle fidelity or navigation accuracy.

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
- [x] Verify current physics replays at initial/middle/final times and backward seeks: 20 WebGL checks passed. Preserve [browser checks and hashes](../results/verification/project2_segments/physics/browser_checks.json) and screenshots. This establishes rendering agreement, not empirical vehicle fidelity.

**Completion criterion:** Rendered geometry and actuator states agree with recorded simulation state, and the guide describes only implemented and verified behavior.

## 6. Later: Scoped Public-Flight Comparison and Optional Parameter Fitting

Use archived observations as optional background during the physics-experiment milestone. Default runs and physics checks do not load webcast data or optimize RMSE. For final Project 2 reporting, define a meaningful comparison after documenting observation conventions and the correspondence of initial conditions/inputs; quantify differences without calling them isolated model or navigation errors.

- [x] Archive and audit the CRS-11 reference snapshot: file integrity, array structure, time coverage, gaps, and derived-speed consistency flags. This is data preparation, not flight-model validation.
- [x] Implement raw altitude/speed comparison with explicit offsets/conventions, overlap-only interpolation, gap-aware time weighting, per-window metrics, and comparison plots. Run the initial uncalibrated ascent and 2/1/0.5 ms timestep comparison; record results in the [verification report](verification/physics_implementation_report_en.md#5-ascent-integration-and-initial-flight-data-comparison).
- [x] Apply explicit local/reference epoch offsets, units, initial-sample exclusion, overlap masks and gap-aware time weighting for each selected scenario. Record the still-assumed altitude datum/speed frame under the [comparison contract](reference/crs11_flight_reference.md#comparison-contract); evidence needed to resolve those assumptions remains open in Section 1.
- [x] Compare raw altitude and scalar speed for the four segments, with coverage, bias, RMSE, maximum error and histories. No prescribed event-time scores or empirical flight-accuracy pass verdicts are assigned.
- [ ] Present downrange, attitude, dynamic-pressure, angle-of-attack, and propellant histories as simulation diagnostics unless independent observations support error calculations; assess derived reference channels separately.
- [ ] Before parameter fitting, select and record calibration versus withheld intervals or an independently audited second flight. Freeze fitted parameters before validation and report remaining uncertainty.

**Completion criterion:** A later scoped comparison states the observation uncertainty, initial-condition/input correspondence and modeling assumptions. It is separate from physics acceptance and from navigation-error evaluation. Parameter fitting is optional and requires a declared calibration/validation distinction; RMSE reduction is not a prerequisite for using the current experiments.

## 7. Deferred: Sensor Models and Navigation

Implementation is deferred until the independent vehicle-scenario milestone by user choice. It remains required for final Project 2 completion; truth pass-through is not sensor-based navigation.

- [ ] Define position, velocity, attitude, and angular-rate estimation requirements and select the sensors.
- [ ] Define sensor measurement types, coordinate frames, timestamps, availability, and estimator input/output contracts.
- [ ] Implement the selected IMU, GNSS, altimeter, or other measurement equations. Accelerometers must measure specific force, excluding gravity.
- [ ] Model noise, bias, sampling periods, latency, and measurement dropouts.
- [ ] Implement initialization, alignment, INS propagation, and an aiding filter.
- [ ] Evaluate navigation on a fixed truth trajectory to separate flight-model errors from estimation errors.

**Completion criterion:** The estimator derives states from sensor measurements and initialization information, with truth used for error evaluation.

## 8. Deferred: Unfiltered and Filtered Errors

- [ ] Define an unfiltered baseline appropriate to the sensor configuration.
- [ ] Compare unfiltered and filtered results using identical truth, measurements, initial conditions, and noise seeds.
- [ ] Evaluate position, velocity, attitude, and angular-rate RMSE, maximum errors, and error histories.
- [ ] Assess estimation performance and limitations under bias and measurement dropouts.

**Completion criterion:** Filter improvements and residual errors are supported by quantitative comparisons under identical conditions.

## 9. Complete the Report and Reproducible Runs

- [x] Document current ascent configuration, dynamics verification, environment, replay, assumptions, and deferred flight-data use in the project guide; preserve these historical verification records.
- [x] Document selected independent scenarios, initial conditions, reference windows, reproducible runs and comparison results. Keep remaining vehicle-fidelity/domain work and final navigation requirements explicit.
- [ ] Describe the sensor and navigation architecture and error-processing methods.
- [ ] Reuse the existing physical implementation verification report and incorporate extension verification, flight-data validation, and navigation-error analysis.
- [ ] Assess the principal modeling and navigation errors that could affect subsequent GNC design.
- [ ] Provide reproducible configurations, execution commands, verdicts, and essential figures for the selected scenarios.

**Completion criterion:** Code, configurations, results, and the report describe the same models and experimental conditions.

## Maintenance Principles

- Keep generated JSON/CSV/NPZ, plots/screenshots/PDFs and resolved run configurations under ignored `results/`, including `results/verification/`. Keep source code, authored YAML, tests, raw reference snapshots and manual reports in Git. Existing committed evidence is historical; new executions should not populate `docs/verification/` with generated files.
- Implement features in shared RLV modules and existing runners; use general flight or physics-experiment names for scenarios.
- Reuse existing phase physics, controllers, event records, and replay support. Track implemented capabilities, pending scenario qualification and optional integration separately; do not mark an entire phase unimplemented because full-flight integration remains open.
- Keep independent scenario initialization explicit. Require continuity within each scenario, but do not make upstream-generated initial states or successful end-to-end return a prerequisite for Project 2 segment demonstrations.
- Consolidate verification equations and result discussions in the English verification report; maintain task status and completion criteria here.
- Preserve existing functionality and verification coverage. Treat fidelity extensions unused by the selected mission as follow-up work.
