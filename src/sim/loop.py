"""Shared prescribed-input and closed-loop rollout of the native C++ plant.

All update periods use exact integer plant ticks. Truth logs contain initial, scheduled, and
terminal endpoints; navigation and actuator histories have independent timestamped records.
"""

from __future__ import annotations

import copy
import sys
import time
from collections import deque
from types import SimpleNamespace

import numpy as np

from core.frames import dcm_from_quat
from core.interfaces import (
    ActuatorCommand,
    ActuatorHistory,
    EpisodeResult,
    NavigationHistory,
    StateHistory,
    TerminationReason,
)
from core.seeding import rng_from_seed
from plant.dynamics import termination_reason
from sim.schedule import build_schedule


class _Progress:
    """Single-line live progress on stderr: sim-time bar + state + wall-clock ETA.

    Auto-disabled when stderr is not a TTY (pipes, background tasks, CI stay clean)."""

    _WIDTH = 24

    def __init__(self, t_max: float, enabled: bool):
        self.t_max = float(t_max)
        self.enabled = bool(enabled) and sys.stderr.isatty()
        self.t0 = time.perf_counter()
        self.last = 0.0

    def update(self, t: float, state) -> None:
        if not self.enabled:
            return
        now = time.perf_counter()
        if now - self.last < 0.1:  # ~10 Hz refresh cap
            return
        self.last = now
        frac = min(max(t, 0.0) / self.t_max, 1.0)
        wall = now - self.t0
        eta = wall * (1.0 - frac) / max(frac, 1e-9)
        r = np.asarray(state.r_ned)
        v = np.asarray(state.v_ned)
        fill = int(round(self._WIDTH * frac))
        bar = "█" * fill + "░" * (self._WIDTH - fill)
        sys.stderr.write(
            f"\r{bar} t={t:6.1f}/{self.t_max:.0f} s  alt={-float(r[2]):8.1f} m  "
            f"vz={float(v[2]):7.1f} m/s  ETA {eta:5.0f} s "
        )
        sys.stderr.flush()

    def close(self) -> None:
        if self.enabled:
            sys.stderr.write("\r" + " " * 100 + "\r")
            sys.stderr.flush()


def _tilt_from_vertical(q_bn) -> float:
    """Angle [rad] of body-x (nose) from NED up (-z)."""
    bx = np.asarray(dcm_from_quat(q_bn))[:, 0]  # body-x in NED
    return float(np.arccos(np.clip(-bx[2], -1.0, 1.0)))


def _slerp(q0, q1, s):
    """Normalized-lerp of two adjacent log quaternions (with antipodal-sign correction). The two
    samples are one log step apart so the rotation between them is tiny — nlerp is exact enough for
    the touchdown ground-crossing interpolation and avoids a full slerp's trig."""
    q0, q1 = np.asarray(q0), np.asarray(q1)
    if float(np.dot(q0, q1)) < 0.0:
        q1 = -q1  # take the short way (q and -q are the same rotation)
    q = q0 + s * (q1 - q0)
    return q / (np.linalg.norm(q) + 1e-12)


def _log_step(log, aux_log, state, phase, aux):
    log["t"].append(float(state.t))
    log["r_ned"].append(np.asarray(state.r_ned))
    log["v_ned"].append(np.asarray(state.v_ned))
    log["q_bn"].append(np.asarray(state.q_bn))
    log["w_body"].append(np.asarray(state.w_body))
    log["m"].append(float(state.m))
    log["phase"].append(int(phase))
    # actuator truth (for replay/visualization; cheap to log alongside the pose)
    log["throttle_act"].append(np.asarray(state.throttle_act))
    log["tvc"].append(np.asarray(state.tvc))
    log["fin"].append(np.asarray(state.fin))
    aux_log.append({key: np.asarray(value) for key, value in aux.items()})


# Structural-survival contact limits: reaching the ground within these counts as a
# (possibly imperfect) LANDING; exceeding any of them is a CRASH. Distinct from the tighter
# soft-landing SUCCESS tolerances. Grounded in legged-lander limits: F9-class legs tolerate a few
# m/s sink and a few degrees of tilt; > ~6 m/s sink / large tilt / spin tips or collapses it.
_CRASH_SINK = 6.0  # vertical sink rate [m/s]
_CRASH_HORIZ = 4.0  # horizontal velocity [m/s] (tip-over on contact)
_CRASH_TILT_DEG = 15.0  # attitude tilt from vertical [deg]
_CRASH_RATE = 0.35  # body angular rate [rad/s] (~20 deg/s; tumbling arrival)

# Genuine SOFT-landing limits (the upper end of a real set-down): F9/Apollo-LM nominal touchdown is
# ~1–2 m/s sink, <1 m/s lateral, near-vertical, near-zero spin. A contact within the CRASH limits
# but ABOVE these is 'hard' — structurally survivable (legs absorb it) but NOT soft. (2022 IJASS
# cites 5 m/s as the *allowable* bound; this is the *good*-landing bar, deliberately tighter.)
_SOFT_SINK = 2.0  # vertical sink rate [m/s]
_SOFT_HORIZ = 1.0  # horizontal velocity [m/s]
_SOFT_TILT_DEG = 5.0  # attitude tilt from vertical [deg]
_SOFT_RATE = 0.1  # body angular rate [rad/s] (~6 deg/s)


def _finalize(log, aux_log, final_state, bundle, hit_max, pos_tol, speed_tol, tilt_tol_deg):
    """Assemble an EpisodeResult + KPIs from the logged trajectory (shared by both rate paths)."""
    t_arr = np.asarray(log["t"])
    r = np.stack(log["r_ned"])
    v = np.stack(log["v_ned"])
    q = np.stack(log["q_bn"])
    w = np.stack(log["w_body"])
    m = np.asarray(log["m"])
    phase = np.asarray(log["phase"])
    throttle_act = np.stack(log["throttle_act"])
    tvc = np.stack(log["tvc"])
    fin_act = np.stack(log["fin"])

    # touchdown index = first altitude <= 0, else last
    alt = -r[:, 2]
    below = np.where(alt <= 0.0)[0]
    td = int(below[0]) if below.size else len(alt) - 1

    # interpolate the state to the EXACT ground crossing alt=0: the logged ``td`` is
    # the first SUB-ground sample, so reading KPIs there biases them; lerp between the last-above
    # and first-below samples. (No crossing, or td==0 => use td as-is.)
    if 0 < td < len(alt) and alt[td] < 0.0 <= alt[td - 1]:
        s = float(alt[td - 1] / (alt[td - 1] - alt[td]))  # in [0,1]
        r_td = r[td - 1] + s * (r[td] - r[td - 1])
        v_td = v[td - 1] + s * (v[td] - v[td - 1])
        w_td = w[td - 1] + s * (w[td] - w[td - 1])
        m_td = float(m[td - 1] + s * (m[td] - m[td - 1]))
        q_td = _slerp(q[td - 1], q[td], s)
    else:
        r_td, v_td, w_td, m_td, q_td = r[td], v[td], w[td], float(m[td]), q[td]

    landing_position_error = float(np.linalg.norm(r_td[:2]))
    landing_speed = float(np.linalg.norm(v_td))
    landing_speed_vertical = float(abs(v_td[2]))  # sink rate
    landing_speed_horizontal = float(np.linalg.norm(v_td[:2]))  # lateral
    landing_rate = float(np.linalg.norm(w_td))  # spin at contact
    landing_tilt = _tilt_from_vertical(q_td)
    fuel_margin = m_td - float(bundle.params.mass.m_dry)
    # RCS gas cannot be burned by the main engine. Use the native main-propellant
    # budget rather than counting the separate gas reserve as landing fuel.
    if aux_log and "main_propellant" in aux_log[td]:
        fuel_margin = float(aux_log[td]["main_propellant"])
        if 0 < td < len(alt) and alt[td] < 0.0 <= alt[td - 1]:
            previous = float(aux_log[td - 1]["main_propellant"])
            fuel_margin = previous + s * (fuel_margin - previous)
    reason = termination_reason(final_state, bundle.params, hit_max)

    # CRASH reclassification: if the predicate says we reached the ground (TOUCHDOWN) but any
    # structural-survival limit is exceeded, it is a CRASH, not a landing — a 15 m/s slam must not
    # be recorded identically to a 0.5 m/s set-down.
    if reason == int(TerminationReason.TOUCHDOWN) and (
        landing_speed_vertical > _CRASH_SINK
        or landing_speed_horizontal > _CRASH_HORIZ
        or landing_tilt > np.deg2rad(_CRASH_TILT_DEG)
        or landing_rate > _CRASH_RATE
    ):
        reason = int(TerminationReason.CRASH)

    # 3-tier contact quality: soft (genuine set-down) / hard (survivable but firm) /
    # crash (>structural). None if the run never made a controlled contact (fuel-out/timeout/etc.).
    if reason == int(TerminationReason.CRASH):
        landing_class = "crash"
    elif reason == int(TerminationReason.TOUCHDOWN):
        landing_class = (
            "soft"
            if (
                landing_speed_vertical <= _SOFT_SINK
                and landing_speed_horizontal <= _SOFT_HORIZ
                and landing_tilt <= np.deg2rad(_SOFT_TILT_DEG)
                and landing_rate <= _SOFT_RATE
            )
            else "hard"
        )
    else:
        landing_class = None

    violations = {
        "position": bool(landing_position_error > pos_tol),
        "speed": bool(landing_speed > speed_tol),
        "tilt": bool(landing_tilt > np.deg2rad(tilt_tol_deg)),
        "rate": bool(landing_rate > _CRASH_RATE),  # tumbling-but-upright is not a success
        "fuel": bool(fuel_margin <= 0.0),
    }
    # SUCCESS requires a genuine SOFT landing (sink≤2 / lateral≤1 m/s, etc. — landing_class) ON
    # the pad (position) with fuel remaining. A 'hard' set-down (survivable but firm, 2–6 m/s) is
    # not a success.
    success = landing_class == "soft" and not any(violations.values())

    return EpisodeResult(
        t=t_arr,
        r_ned=r,
        v_ned=v,
        q_bn=q,
        w_body=w,
        m=m,
        phase=phase,
        aux={"per_tick": aux_log},
        termination_reason=reason,
        touchdown_index=td,
        landing_position_error=landing_position_error,
        landing_speed=landing_speed,
        landing_tilt=landing_tilt,
        fuel_margin=fuel_margin,
        success=success,
        violation_flags=violations,
        throttle_act=throttle_act,
        tvc=tvc,
        fin=fin_act,
        landing_speed_vertical=landing_speed_vertical,
        landing_speed_horizontal=landing_speed_horizontal,
        landing_rate=landing_rate,
        landing_class=landing_class,
    )


def _find_mem_key(mem, key):
    """Recursively find the first ``key`` value in the (nested) GNC memory."""
    if isinstance(mem, dict):
        if mem.get(key) is not None:
            return mem[key]
        for v in mem.values():
            if isinstance(v, (dict, list)):
                found = _find_mem_key(v, key)
                if found is not None:
                    return found
    elif isinstance(mem, list):
        for v in mem:
            found = _find_mem_key(v, key)
            if found is not None:
                return found
    return None


def _collect_mem_lists(mem, key):
    """Recursively gather every list stored under ``key`` in the (nested) GNC memory and concat it.

    Used for guidance ``solve_log`` records and ``control_timing`` samples."""
    out: list = []
    if isinstance(mem, dict):
        for k, v in mem.items():
            if k == key and isinstance(v, list):
                out.extend(v)
            elif isinstance(v, (dict, list)):
                out.extend(_collect_mem_lists(v, key))
    elif isinstance(mem, list):
        for v in mem:
            out.extend(_collect_mem_lists(v, key))
    return out


def _percentiles(a) -> dict:
    """mean/std/p50/p95/p99/max(=WCET)/min of a 1-D sample array."""
    return {
        "mean": float(a.mean()),
        "std": float(a.std()),
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "p99": float(np.percentile(a, 99)),
        "max": float(a.max()),  # WCET
        "min": float(a.min()),
    }


def _solver_stats(logs):
    """Aggregate per-solve guidance records into timing/feasibility stats (RTI/WCET relevant)."""
    solves = [record for record in logs if record.get("record_type", "solve") == "solve"]
    outcomes = [record for record in logs if record.get("record_type") == "outcome"]
    stats = {
        "n_solves": len(solves),
        "n_ignition": int(sum(r["kind"].startswith("ignite") for r in solves)),
        "n_resolve": int(sum(r["kind"].startswith("resolve") for r in solves)),
        "n_reanchor": int(sum(r["kind"].startswith("reanchor") for r in solves)),
        "n_solver_infeasible": int(
            sum(not r.get("solver_feasible", r.get("feasible", False)) for r in solves)
        ),
        "n_outcomes": len(outcomes),
        "n_failed_outcomes": int(
            sum(not r.get("mission_feasible", r["feasible"]) for r in outcomes)
        ),
        "n_postcheck_rejected_solves": int(
            sum(record.get("mission_feasible") is False for record in solves)
        ),
        "n_grid_fallback": int(sum(r.get("grid_fallback", False) for r in logs)),
    }
    if solves:
        # Per-attempt wall includes model stuffing, dispatch, and solve. solve_ms is the backend's
        # own timer and its exact scope is backend-dependent.
        wall_ms = [r["wall_ms"] for r in solves if r.get("wall_ms") is not None]
        solve_ms = [r["solve_ms"] for r in solves if r.get("solve_ms") is not None]
        stats["wall_ms"] = _percentiles(np.asarray(wall_ms, float)) if wall_ms else None
        stats["solve_ms"] = _percentiles(np.asarray(solve_ms, float)) if solve_ms else None
        stats["n_backend_timed"] = len(solve_ms)
        stats["iters"] = _percentiles(np.array([r["iters"] for r in solves], float))
    stats["per_solve"] = solves
    stats["outcomes"] = outcomes
    return stats


def _guidance_degradation(mem, solve_logs):
    """Expose guidance degraded-mode counters that otherwise die with controller memory."""
    keys = (
        "replan_failures",
        "cached_plan_fallbacks",
        "reanchor_count",
        "reanchor_time_added",
        "terminal_slack_uses",
        "terminal_pos_slack",
        "terminal_vel_slack",
        "template_refinement_failures",
        "plan_exhausted_ticks",
        "proj_downthrust_clamps",
        "last_replan_failed",
    )
    out = {key: value for key in keys if (value := _find_mem_key(mem, key)) is not None}
    out["grid_fallbacks"] = int(sum(log.get("grid_fallback", False) for log in solve_logs))
    return out


def _attach_gnc_aux(result, gnc_mem):
    """Attach final GNC diagnostics identically for single- and multi-rate runners."""
    solve_logs = _collect_mem_lists(gnc_mem, "solve_log")
    result.aux["solver"] = _solver_stats(solve_logs)
    result.aux["guidance_degradation"] = _guidance_degradation(gnc_mem, solve_logs)
    result.aux["control"] = _control_stats(_collect_mem_lists(gnc_mem, "control_timing"))
    result.aux["plan_ignition"] = _find_mem_key(gnc_mem, "plan_ignition")
    result.aux["t_ignition"] = _find_mem_key(gnc_mem, "t_ignition")


def _control_stats(samples):
    """Per-call tracking-controller compute time [ms] — the control LAW only (allocation excluded).

    Controller-agnostic: timed at the stack layer, so any plugged-in controller (PD or otherwise) is
    measured. The first call is logged separately as warm-up (first-touch numpy / tracing). min/p50
    approximate the true compute (jitter only ADDS time); max/p95/p99 are the WCET/jitter real-time
    analysis cares about. This is pure Python wall time — a C-ported controller exposing its own
    solver timer can additionally report that (cf. the solver's solve_ms vs wall_ms split)."""
    stats = {"n_calls": len(samples)}
    if samples:
        stats["warmup_ms"] = float(samples[0])  # first call: first-touch numpy / tracing
    if len(samples) > 1:
        stats["wall_ms"] = _percentiles(np.array(samples[1:], float))  # warm-up call dropped
    return stats


_STATE_FIELDS = ("t", "r_ned", "v_ned", "q_bn", "w_body", "m")
_ACT_FIELDS = ("throttle", "gimbal", "fin", "rcs")


def _copy_command(command):
    return ActuatorCommand(
        **{key: np.array(getattr(command, key), copy=True) for key in _ACT_FIELDS}
    )


def _stamp_epoch(record, t):
    """The current navigation seam is synchronous; reject stale state epochs explicitly."""
    epoch = getattr(record, "t", None)
    if epoch is not None and not np.isclose(float(epoch), t, rtol=0.0, atol=1e-8):
        raise ValueError("navigation state epoch must match current truth time")
    if hasattr(record, "replace"):
        return record.replace(t=t)
    out = copy.copy(record)
    out.t = t
    return out


def _state_record(state):
    return {key: np.array(getattr(state, key), copy=True) for key in _STATE_FIELDS}


def _state_history(rows):
    return StateHistory(**{key: np.stack([row[key] for row in rows]) for key in _STATE_FIELDS})


def _command_history(rows):
    return ActuatorCommand(
        **{key: np.stack([getattr(row, key) for row in rows]) for key in _ACT_FIELDS}
    )


def run_simulation(
    bundle,
    state0,
    controller,
    nav,
    t_max: float,
    ctrl_dt: float,
    plant_dt: float,
    success_pos_tol: float = 10.0,
    success_speed_tol: float = 5.0,
    success_tilt_tol_deg: float = 15.0,
    delay_ticks: int = 0,
    nav_dt: float | None = None,
    guidance_dt: float | None = None,
    log_dt: float | None = None,
    progress: bool = False,
    record_telemetry: bool = True,
    prescribed=None,
) -> EpisodeResult:
    """Advance for at most t_max seconds from state0.t on an exact fixed-step grid.

    Each period and t_max must be a positive integer multiple of plant_dt. GNC updates run
    at t0+k*period, before integration. Trajectory samples are at t0, k*log_dt, and the actual
    native terminal time. Navigation records are synchronous, full-rate truth/meas/estimate
    triples. Actuator rows describe each native block and separate requested, delayed, and
    effective endpoint inputs. delay_ticks is always measured in control periods.
    """
    if prescribed is not None:
        if controller is not None or nav is not None or delay_ticks:
            raise ValueError("prescribed execution uses no GNC, navigation, or command delay")
        if not np.isclose(prescribed.dt, plant_dt, atol=1e-12, rtol=0):
            raise ValueError("input schedule and plant_dt must match")
        nav_dt = guidance_dt = ctrl_dt
    elif controller is None or nav is None:
        raise ValueError("closed-loop execution requires controller and navigation")
    schedule = build_schedule(plant_dt, t_max, ctrl_dt, nav_dt, guidance_dt, log_dt)
    if not np.isclose(float(bundle.params.plant_dt), plant_dt, rtol=0.0, atol=1e-12):
        raise ValueError("plant_dt must match the native plant's configured integration step")
    if not isinstance(delay_ticks, (int, np.integer)) or delay_ticks < 0:
        raise ValueError("delay_ticks must be a non-negative integer of control periods")
    split = hasattr(controller, "guide") and hasattr(controller, "control_allocate")
    if not split and schedule.guidance != schedule.control:
        raise ValueError("independent guidance_dt requires guide/control_allocate methods")

    session = bundle.native.create_session(state0)
    if prescribed is not None:
        session.set_fin_deployment(prescribed.rows[0]["fin_deployment"])
    snapshot, aux = session.observe()
    state = SimpleNamespace(**snapshot)
    t0 = float(state.t)
    if not np.isfinite(t0):
        raise ValueError("initial state time must be finite")
    nav_mem = nav.initial_memory() if hasattr(nav, "initial_memory") else {}
    gnc_mem = controller.initial_memory() if hasattr(controller, "initial_memory") else {}
    nav_rng = rng_from_seed(state0.key)
    held = ActuatorCommand(
        throttle=np.array(state.throttle_act, copy=True),
        gimbal=np.array(state.tvc, copy=True),
        fin=np.array(state.fin, copy=True),
        rcs=np.zeros(3),
    )
    requested = _copy_command(held)
    t_command = t_held = t0
    pending = deque()
    log = {
        key: []
        for key in (
            "t",
            "r_ned",
            "v_ned",
            "q_bn",
            "w_body",
            "m",
            "phase",
            "throttle_act",
            "tvc",
            "fin",
        )
    }
    aux_log = []
    nav_log = {key: [] for key in ("truth", "measurement", "estimate")}
    act_log = {
        key: []
        for key in ("t_start", "t_end", "t_command", "t_held", "commanded", "held", "effective")
    }
    events = {key: [] for key in ("navigation", "guidance", "control", "command_release")}
    x_hat = cmd = None
    phase = -1
    prog = _Progress(t_max, progress)

    if bool(state.done):
        _log_step(log, aux_log, state, phase, aux)
    boundaries = set(range(0, schedule.horizon, schedule.block)) | {schedule.horizon}
    if prescribed is not None:
        if max(prescribed.rows) >= schedule.horizon:
            raise ValueError("input event lies outside the simulation horizon")
        boundaries.update(prescribed.rows)
    boundaries = sorted(boundaries)
    input_events = []
    physical_events = []
    last_main = float(aux["main_propellant"])
    for tick, next_tick in zip(boundaries[:-1], boundaries[1:], strict=True):
        if bool(state.done):
            break
        t = t0 + tick * schedule.plant_dt
        if not np.isclose(float(state.t), t, rtol=0.0, atol=1e-8):
            raise RuntimeError("native time diverged from the integer-tick schedule")
        if nav is not None and tick % schedule.navigation == 0:
            seed = int(nav_rng.bit_generator.random_raw())
            # Keep independent records even if an estimator updates its input arrays in place.
            truth_row = _state_record(state) if record_telemetry else None
            meas = _stamp_epoch(nav.measure(copy.deepcopy(state), seed), float(state.t))
            measurement_row = _state_record(meas) if record_telemetry else None
            x_hat, nav_mem = nav.estimate(meas, held, nav_mem)
            x_hat = _stamp_epoch(x_hat, float(state.t))
            events["navigation"].append(float(state.t))
            if record_telemetry:
                nav_log["truth"].append(truth_row)
                nav_log["measurement"].append(measurement_row)
                nav_log["estimate"].append(_state_record(x_hat))
        if split and tick % schedule.guidance == 0:
            cmd, gnc_mem = controller.guide(x_hat, t, gnc_mem)
            phase = int(cmd.phase)
            events["guidance"].append(float(state.t))
        if prescribed is not None and tick in prescribed.rows:
            row = prescribed.rows[tick]
            requested = _copy_command(row["command"])
            held = requested
            t_command = t_held = float(state.t)
            phase = row["phase"]
            session.set_fin_deployment(row["fin_deployment"])
            # Configuration changes affect diagnostics at the event epoch, not the pose.
            snapshot, aux = session.observe()
            state = SimpleNamespace(**snapshot)
            if log["t"] and np.isclose(log["t"][-1], state.t, atol=1e-9, rtol=0):
                aux_log[-1] = {key: np.asarray(value) for key, value in aux.items()}
                log["phase"][-1] = phase
            input_events.append(
                {
                    "t": float(state.t),
                    "kind": "input",
                    "label": row["label"],
                    "fin_deployment": row["fin_deployment"],
                }
            )
            events["command_release"].append(float(state.t))
        if prescribed is None and tick % schedule.control == 0:
            if split:
                act, gnc_mem = controller.control_allocate(cmd, x_hat, gnc_mem, t)
            else:
                act, gnc_mem, phase = controller(x_hat, t, gnc_mem)
                events["guidance"].append(float(state.t))
            requested = _copy_command(act)
            t_command = float(state.t)
            pending.append((tick + delay_ticks * schedule.control, t_command, requested))
            events["control"].append(float(state.t))
        while pending and pending[0][0] <= tick:
            _, t_held, held = pending.popleft()
            events["command_release"].append(float(state.t))

        # Initial truth is the supplied state, before the first command takes effect.
        if tick == 0:
            _log_step(log, aux_log, state, phase, aux)
        start = float(state.t)
        steps = next_tick - tick
        snapshot, aux = session.advance(held, steps)
        state = SimpleNamespace(**snapshot)
        main = float(aux["main_propellant"])
        if last_main > 1e-8 and main <= 1e-8:
            physical_events.append(
                {
                    "t": float(state.t),
                    "kind": "main_fuel_depleted",
                    "time_uncertainty_s": steps * plant_dt,
                }
            )
        last_main = main
        if bool(state.done):
            physical_events.append(
                {"t": float(state.t), "kind": "ground_crossing", "time_uncertainty_s": plant_dt}
            )
        if record_telemetry:
            for key, value in (
                ("t_start", start),
                ("t_end", float(state.t)),
                ("t_command", t_command),
                ("t_held", t_held),
                ("commanded", _copy_command(requested)),
                ("held", _copy_command(held)),
                (
                    "effective",
                    ActuatorCommand(
                        throttle=np.array(state.throttle_act, copy=True),
                        gimbal=np.array(state.tvc, copy=True),
                        fin=np.array(state.fin, copy=True),
                        rcs=np.array(state.rcs_act, copy=True),
                    ),
                ),
            ):
                act_log[key].append(value)
        end_tick = tick + steps
        if (
            end_tick % schedule.logging == 0
            or bool(state.done)
            or end_tick == schedule.horizon
            or (prescribed is not None and end_tick in prescribed.rows)
        ):
            _log_step(log, aux_log, state, phase, aux)
        prog.update(float(state.t) - t0, state)
    prog.close()

    result = _finalize(
        log,
        aux_log,
        state,
        bundle,
        not bool(state.done),
        success_pos_tol,
        success_speed_tol,
        success_tilt_tol_deg,
    )
    if prescribed is None:
        _attach_gnc_aux(result, gnc_mem)
    else:
        result.aux["phase_names"] = prescribed.phase_names
    result.aux["input_mode"] = "prescribed" if prescribed is not None else "closed_loop"
    result.aux["events"] = sorted(input_events + physical_events, key=lambda row: row["t"])
    result.aux["schedule"] = {
        **schedule.metadata(),
        "t_start_s": t0,
        "t_end_s": float(state.t),
        "delay_control_ticks": int(delay_ticks),
        "delay_s": delay_ticks * schedule.control * schedule.plant_dt,
        "events_s": events,
    }
    if nav_log["truth"]:
        result.navigation = NavigationHistory(
            **{key: _state_history(rows) for key, rows in nav_log.items()}
        )
    if act_log["t_start"]:
        result.actuators = ActuatorHistory(
            **{
                key: np.asarray(act_log[key]) for key in ("t_start", "t_end", "t_command", "t_held")
            },
            **{key: _command_history(act_log[key]) for key in ("commanded", "held", "effective")},
        )
    return result


# Public compatibility name: both modes share the same stepping and recording implementation.
run_closed_loop = run_simulation
