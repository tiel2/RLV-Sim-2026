# RLV-Sim Development TODO

Status as of 2026-11-02. Prioritize vehicle simulation and navigation development required by [Project 2](../materials/RSLV_GNC_Project2.pdf). The listed flight phases are representative examples; the implementation scope will be determined by the scenario and its boundary conditions.

**Sequence:** Define model assumptions → Implement and independently verify dynamics → Verify rendering against simulation state → Update the project guide → Integrate the next phase with the same checks → Later validate against flight-reference data → Sensors/navigation and error analysis → Final report.

**Current priority:** Mathematical and physical consistency comes first. Flight-reference matching, parameter fitting, sensor models, and navigation errors are deferred. Archived CRS-11 data and comparison tools remain available; `scenario=ascent` disables flight comparison by default. Verification of an assumed model does not establish the accuracy of its real-vehicle parameters.

## Completed Work

- [x] Independently verify the existing 6-DOF equations, mass properties, and canonical motion cases.
- [x] Verify engine, TVC, grid-fin, RCS, actuator, atmosphere, and body-aerodynamic components.
- [x] Verify coupled motion and timestep agreement with ideal actuation and averaged RCS.
- [x] Complete the [physical implementation verification report](verification/physics_implementation_report_en.md) and remove redundant explanatory files.
- [x] Independently integrate the 150 s attached-stack ascent with realistic actuators; verify mass and open-system linear/angular momentum balances and rigid-pad support.
- [x] Verify ascent, single-engine landing, and fuel-depletion rendering in Chrome, including reverse seeking, engine count, thrust axes, stack geometry, and pad alignment.
- [x] Document these implemented capabilities, reproducible checks, and remaining scope in the [project guide](3_project_guide_ko.md#ascent-physics).

Flight scenarios cover post-flip reentry, descent, and landing, plus a provisional attached-stack ascent → MECO → short coast milestone. They are not yet connected into one full flight. Navigation provides truth pass-through and synthetic state perturbations; physical sensor models, INS, and EKF remain deferred.

**Status convention:** Existing capabilities are listed separately from pending work. An unchecked integration or verification task does not mean its underlying physics or controller is unimplemented. Existing segment implementations do not establish a validated full-flight scenario.

## 1. Define the Landing Frame, Initial Conditions, and Assumptions

Use a local NED frame with the landing point at the origin: nominal touchdown position `[0, 0, 0]` and velocity `[0, 0, 0]`. Express the launch location and initial states relative to this origin. Map this frame to the reference flight when comparing with flight data.

- [x] Select **CRS-11 stage 1** as the reference flight; preserve the commit-pinned webcast data and record observables, coverage, provenance, quality flags, and uncertainty limitations in the [flight-reference specification](reference/crs11_flight_reference.md).
- [ ] Resolve altitude datum, speed reference frame, launch/landing coordinates, heading, and stage-separation timing using additional sources. Define numerical uncertainty bounds where supported; retain unknowns explicitly.
- [ ] Specify the launch location, initial position, velocity, attitude, angular rate, and mass in the landing frame; select representative phases consistent with these conditions and the landing target.
- [x] Define provisional ascent initial conditions, parameter classifications, and their limits in the [ascent specification](reference/crs11_ascent.md). Exact geographic/datum reconstruction remains open above.
- [x] Represent the attached upper stage as a fixed equivalent component during ascent; retain its mass/CG/inertia separately in configuration. Runtime separation and the first-stage-only handoff remain open in Section 3.
- [ ] Classify vehicle, environment, and input parameters as **Known / Estimated / Assumed**.

**Completion criterion:** The landing frame, launch location, initial state, phase sequence, observables, and principal modeling assumptions are defined.

**Selected reference:** CRS-11 uses ascent → cutoff → separation/coast/attitude maneuver → boostback → coast → reentry burn → aerodynamic descent → landing burn → touchdown. Compare primarily against raw stage-1 altitude and scalar speed. Use published burn times as prescribed inputs initially; matching prescribed event times is not independent validation. Derived channels are auxiliary references, not a complete truth state vector. The reference is archived and a provisional ascent initial state is implemented; full-flight integration and measured initial-condition reconstruction remain pending.

## 2. Configure the Ascent Vehicle and Environment

- [x] Configure provisional ascent mass, propellant, tanks, CG, and inertia in `plant/vehicle=falcon9_ascent`; retain recovery configurations separately. Values are documented assumptions, not measured CRS-11 properties.
- [x] Account for attached upper-stage mass, CG, and inertia separately from main propellant, using an equivalent dry-body combination and the parallel-axis theorem.
- [x] Add a separate nose-first aerodynamic configuration and numerical lookup domain; keep recovery's engine-first reference axis and coefficients.
- [ ] Establish empirical aerodynamic validity and uncertainty bounds for both configurations; the assumed ascent coefficient table is not flight-validated.
- [x] Distinguish rigid-pad hold/release from ground-contact termination and exclude initial pad samples from touchdown assessment.
- [ ] Evaluate the coordinate-frame, altitude, and gravity approximations over the selected flight domain and determine necessary extensions.

**Completion criterion:** Mass budgets, propulsion, aerodynamics, and initial conditions are consistent for the ascent configuration.

## 3. Integrate Existing Flight Phases and Implement Missing Events

### Existing implementation and remaining scope

| Phase / capability | Already implemented and reusable | Not yet implemented or integrated into the full flight |
|---|---|---|
| **Powered ascent** | [ascent](../configs/scenario/ascent.yaml) combines the attached-stack mass model, nose-first aerodynamics, pad departure, nine-engine throttle/gimbal inputs, and independently verified 6-DOF propagation through prescribed MECO/coast. | State handoff into separation and recovery. Flight calibration/validation is deferred. |
| **Engine cutoff** | Scheduled throttle-off, fuel exhaustion cutoff, and continued unpowered propagation. Ascent commands MECO at 145 s and continues to 150 s with the stack attached. | Separation handoff, boostback cutoff, and any independently condition-triggered ascent termination. |
| **Stage separation** | Propellant-dependent mass, CG, and inertia provide the existing mass-properties foundation. | Attached-component removal, separation-induced mass/CG/inertia and aerodynamic changes, and any modeled separation impulse. Propellant depletion does not implement stage separation. |
| **Coast / attitude maneuvers** | Unpowered propagation and actuator-driven rotational dynamics; [actuator_response](../configs/scenario/actuator_response.yaml) exercises TVC and RCS inputs. | Post-separation coast and flip/maneuver sequences with target attitudes, completion conditions, and downstream handoffs. |
| **Boostback** | Shared thrust, propellant, and attitude dynamics can support retropropulsion. | A dedicated boostback sequence, target-dependent maneuver and burn inputs, and restart/cutoff conditions. Required for the selected CRS-11 mission. |
| **Reentry / aerodynamic descent** | [reentry_to_landing](../configs/scenario/reentry_to_landing.yaml) already connects post-flip reentry burn → aerodynamic descent → landing burn through the [phase scheduler](../src/gnc/build.py). [fin_deployment](../configs/scenario/fin_deployment.yaml) separately implements prescribed instantaneous fin exposure and replay state. | Handoff from preceding mission phases and mission-specific restart/fin-deployment conditions integrated with the recovery sequence. |
| **Terminal descent / landing** | Powered-descent guidance/control, phase-dependent engine selection, and touchdown assessment; standalone landing and connected recovery scenarios exist. | Full-flight fuel allocation and verification that the actual incoming state permits the landing sequence to succeed. |
| **Events and replay** | Prescribed input/configuration events, detected fuel depletion/ground crossing, trajectory phase labels, and fin-deployment replay state. | Consistent full-flight transition records, including condition-triggered transitions and new separation/configuration events, with matching replay representation. |

The existing recovery sequence starts after the flip; it does not simulate launch, separation, or boostback. Its connected phases do not establish a successful end-to-end landing. Fin deployment currently changes aerodynamic exposure instantaneously, without deployment dynamics or mass/inertia changes.

### Pending integration of existing capabilities

- [x] **Powered ascent milestone:** Reuse the shared plant/runner for prescribed ascent and continue through a short attached-stack coast. Independent dynamics verification and state-driven replay are complete for this assumed configuration.
- [ ] **Powered ascent integration:** Hand the continuous cutoff state into stage separation using explicit assumptions. Parameter fitting is not a prerequisite for verifying the event dynamics.
- [ ] **Engine cutoff:** Reuse throttle-off and unpowered propagation; add the selected ascent/boostback termination conditions and connect each cutoff to the next phase.
- [ ] **Coast / attitude maneuvers:** Reuse unpowered and rotational dynamics with prescribed actuator inputs or existing controllers; define coast duration, target attitudes, maneuver-completion conditions, and handoffs.
- [ ] **Reentry / aerodynamic descent:** Feed the preceding phase's state into the existing recovery sequence; integrate mission-specific engine restart and fin-deployment conditions using the existing engine and fin-exposure mechanisms.
- [ ] **Terminal descent / landing:** Reuse landing guidance/control, engine selection, and touchdown assessment; allocate remaining main/RCS propellant and verify landing from the actual upstream state.
- [ ] **Continuous state and records:** Carry time, position, velocity, attitude, angular rate, main/RCS propellant, actuator state, and deployment/configuration state through phase boundaries. Reconcile controller memory at handoffs. Do not reset to standalone segment initial conditions; apply only explicitly modeled event changes.
- [ ] **Events and replay:** Extend existing records with transition time, trigger/reason, and before/after configuration for the full sequence; reflect phase, deployment, and separation states consistently in replay.

### Pending new implementation

- [ ] **Stage separation:** Implement attached-component removal and updates to mass, CG, inertia, and aerodynamic configuration. Define the separation-impulse assumption and account for any modeled state jump.
- [ ] **Boostback (required for CRS-11):** Build the missing mission sequence using existing attitude and propulsion models; define maneuver, restart, burn, cutoff, and exit conditions for the landing target. Use declared experiment inputs first; defer matching the archived T+161–211 s window until flight validation.
- [ ] **Full-flight orchestration:** Extend the existing post-flip scheduler and/or prescribed-event runner to cover the selected ascent, cutoff, separation, coast/maneuver, boostback, and recovery sequence. Add the missing phase/configuration events and connect them to the reusable capabilities above.

**Completion criterion:** One scenario propagates the selected launch-to-touchdown sequence with a consistent state history, explicit configuration changes, and traceable transitions. Existing physics and controllers are reused; missing mission logic is implemented. Phase-boundary and landing checks distinguish successful integration from successful landing, with verification tracked in Section 4.

## 4. Verify the Dynamics Before Flight-Data Comparison

- [x] Compare the full 0–150 s ascent, realistic engine cutoff, and attached-stack coast against independent DOP853 integration using rotation matrices and mass quadrature. Check reference-tolerance refinement.
- [x] Audit open-system mass and linear/angular momentum balances using recorded loads; halve diagnostic sampling and verify decreasing integration residuals. Include momentum carried by consumed propellant and the modeled angular momentum flux.
- [x] Record rigid-pad reaction force/moment and exact native release time; test fixed pose/rates with evolving fuel and actuators while supported.
- [x] Test attached inert mass/CG/inertia, actual per-engine forces, fuel cutoff, and replay metadata. Full suite: 321 passed on 2026-11-02.
- [ ] Check mass, inertia, state consistency, and momentum balances across stage separation.
- [ ] Check phase boundaries for unintended state resets, missing propellant, and incorrect restarts.
- [ ] Assess coupled operation with nonlinear actuators and PWM, including timestep sensitivity.

**Evidence:** [Ascent physics results](verification/ascent_physics_results.json), [verification report Section 6](verification/physics_implementation_report_en.md#6-ascent-physics-and-rendering-verification). Realistic nine-engine ascent is checked; this does not complete the pending full-flight/PWM cases above.

**Completion criterion:** Each implemented extension obeys its declared equations, event balances, and numerical tolerances. Unimplemented phases remain explicit.

## 5. Verify Rendering and Maintain the Project Guide

- [x] Render the attached stack separately from the recovery booster; use configured engine positions/counts and all gimbal axes.
- [x] Drive plumes from actual per-engine force, including fuel-depletion shutdown; preserve stowed fins and reverse-seek state.
- [x] Show a schematic land launch pad and frame the launch site; document the cosmetic ground offset separately from contact physics.
- [x] Run six browser checkpoints across ascent, single-engine landing, and fuel depletion; record screenshots and transform/state assertions.
- [x] Update the Korean project guide with dynamics, verification results, rendering contracts, commands, and implemented/deferred distinctions.
- [ ] Extend rendering and guide together with future separation, maneuver, and boostback implementations.

**Completion criterion:** Rendered geometry and actuator states agree with recorded simulation state, and the guide describes only implemented and verified behavior.

## 6. Deferred: Flight-Reference Validation and Parameter Fitting

- [x] Archive and audit the CRS-11 reference snapshot: file integrity, array structure, time coverage, gaps, and derived-speed consistency flags. This is data preparation, not flight-model validation.
- [x] Implement raw altitude/speed comparison with explicit offsets/conventions, overlap-only interpolation, gap-aware time weighting, per-window metrics, and comparison plots. Run the initial uncalibrated ascent and 2/1/0.5 ms timestep comparison; record results in the [verification report](verification/physics_implementation_report_en.md#5-ascent-integration-and-initial-flight-data-comparison).
- [ ] Apply the [CRS-11 comparison contract](reference/crs11_flight_reference.md#comparison-contract): align time origins, altitude datum, coordinate frames, units, and speed definitions; record preprocessing, masks, and sampling/weighting.
- [ ] Compare raw altitude and scalar speed by phase, reporting coverage, bias, RMSE, maximum absolute error, and error histories. Compare event times only when predicted independently; do not score prescribed timing inputs as validation outputs.
- [ ] Present downrange, attitude, dynamic-pressure, angle-of-attack, and propellant histories as simulation diagnostics unless independent observations support error calculations; assess derived reference channels separately.
- [ ] Before parameter fitting, select and record calibration versus withheld intervals or an independently audited second flight. Freeze fitted parameters before validation and report remaining uncertainty.

**Completion criterion:** After physics/rendering/documentation, interpret flight-data discrepancies in terms of measurement uncertainty, initial conditions, and modeling assumptions. Do not tune to reference data to conceal a dynamics error.

## 7. Deferred: Sensor Models and Navigation

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

- [x] Document current ascent configuration, dynamics verification, environment, replay, assumptions, and deferred flight-data use in the project guide; extend this as new phases are implemented.
- [ ] Describe the sensor and navigation architecture and error-processing methods.
- [ ] Reuse the existing physical implementation verification report and incorporate extension verification, flight-data validation, and navigation-error analysis.
- [ ] Assess the principal modeling and navigation errors that could affect subsequent GNC design.
- [ ] Provide reproducible configurations, execution commands, verdicts, and essential figures for the selected scenarios.

**Completion criterion:** Code, configurations, results, and the report describe the same models and experimental conditions.

## Maintenance Principles

- Implement features in shared RLV modules and existing runners; use general flight or physics-experiment names for scenarios.
- Reuse existing phase physics, controllers, event records, and replay support. Track implemented capabilities, pending integration/verification, and missing features separately; do not mark an entire phase unimplemented because full-flight integration remains open.
- Consolidate verification equations and result discussions in the English verification report; maintain task status and completion criteria here.
- Preserve existing functionality and verification coverage. Treat fidelity extensions unused by the selected mission as follow-up work.
