"""Truth-fed separation-to-boostback experiment; no landing or flight-data claims.

All state changes remain in the shared plant. This supervisor only selects actuator
commands, traces guards, and reserves fuel. The return predictor is an explicitly
approximate constant-local-gravity ballistic projection to a descending altitude.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from core.interfaces import ActuatorCommand, Command
from gnc.control.pd import PDController

PHASES = {
    10: "rate_damping",
    11: "attitude_maneuver",
    12: "alignment_hold",
    13: "boostback",
    14: "cutoff_settle",
    15: "post_boostback_coast",
    16: "mission_complete",
    17: "mission_abort",
}


def ballistic_projection(r, v, gravity, altitude):
    """Position at descending altitude under constant local vertical g, no aero.

    This is a targeting diagnostic, never an independent validation or a landing prediction.
    """
    r, v = np.asarray(r), np.asarray(v)
    disc = v[2] ** 2 + 2 * gravity * (-altitude - r[2])
    if gravity <= 0 or disc <= 0:
        return None
    time = (-v[2] + np.sqrt(disc)) / gravity
    if time <= 0:
        return None
    return float(time), r[:2] + time * v[:2]


class Slew:
    """Shortest quaternion rotation with bounded trapezoidal angular speed."""

    def __init__(self, initial, target, rate, acceleration):
        self.initial = Rotation.from_quat(initial, scalar_first=True)
        delta = (self.initial.inv() * Rotation.from_quat(target, scalar_first=True)).as_rotvec()
        self.angle = float(np.linalg.norm(delta))
        self.axis = delta / self.angle if self.angle > 1e-12 else np.array([1.0, 0, 0])
        self.rate = min(rate, np.sqrt(self.angle * acceleration))
        self.acceleration = acceleration
        self.ramp = self.rate / acceleration
        self.cruise = max(0.0, self.angle / max(self.rate, 1e-12) - self.ramp)
        self.duration = 2 * self.ramp + self.cruise

    def sample(self, time):
        t = np.clip(time, 0, self.duration)
        if t < self.ramp:
            angle, rate, acc = (
                0.5 * self.acceleration * t**2,
                self.acceleration * t,
                self.acceleration,
            )
        elif t < self.ramp + self.cruise:
            angle = 0.5 * self.rate * self.ramp + self.rate * (t - self.ramp)
            rate, acc = self.rate, 0.0
        elif t < self.duration:
            remaining = self.duration - t
            angle = self.angle - 0.5 * self.acceleration * remaining**2
            rate, acc = self.acceleration * remaining, -self.acceleration
        else:
            angle, rate, acc = self.angle, 0.0, 0.0
        q = (self.initial * Rotation.from_rotvec(self.axis * angle)).as_quat(scalar_first=True)
        return q, self.axis * rate, self.axis * acc


def pointing_target(q, direction):
    """Rotate body +x to direction with the minimum additional inertial rotation."""
    rotation = Rotation.from_quat(q, scalar_first=True)
    current = rotation.apply([1, 0, 0])
    direction = np.asarray(direction) / np.linalg.norm(direction)
    axis = np.cross(current, direction)
    norm = np.linalg.norm(axis)
    angle = np.arctan2(norm, np.clip(current @ direction, -1, 1))
    if norm < 1e-12:
        axis = rotation.apply([0, 1, 0])
    else:
        axis /= norm
    return (Rotation.from_rotvec(axis * angle) * rotation).as_quat(scalar_first=True)


class BoostbackMission:
    def __init__(self, config, bundle, control_dt):
        self.c = config
        self.bundle = bundle
        self.p = bundle.params
        self.dt = control_dt
        self.phase = None
        self.events = []
        self.records = []
        self.complete = False
        self.failure = None
        self.aligned_since = None
        self.slew = None
        self.burn_start = None
        self.cutoff_reason = None
        self.target_met = False
        self.burn_dv = np.zeros(3)
        self.last_t = None
        self.last_accel = np.zeros(3)
        self.feasibility = {}
        self.reference_windows = {}
        self.q_ref = np.array([1.0, 0, 0, 0])
        self.w_ref = np.zeros(3)
        self._validate()
        self.active = np.zeros(len(self.p.engines.positions))
        self.active[list(config.engines)] = 1
        moments = np.cross(
            self.p.rcs.positions, float(self.p.actuator.rcs_thrust) * self.p.rcs.directions
        )
        self.authority = np.array(
            [moments[np.asarray(self.p.rcs.channels)[:, i] > 0, i].sum() for i in range(3)]
        )
        if np.any(self.authority <= 0) or not np.allclose(self.p.rcs.thrust_scale, 1):
            raise ValueError("boostback requires healthy paired three-axis RCS")
        if not np.allclose(self.p.engines.engine_on, 1) or not np.allclose(
            self.p.engines.thrust_scale, 1
        ):
            raise ValueError("boostback experiment requires healthy main engines")
        if not np.allclose(self.p.engines.tvc_bias, 0) or not np.allclose(self.p.engines.cant, 0):
            raise ValueError("boostback experiment requires unbiased uncanted engines")
        if bundle.meta["n_gimbal"] != bundle.meta["n_engines"]:
            raise ValueError("boostback experiment requires all engines gimballed")
        if not np.allclose(np.sum(self.p.engines.positions[self.active > 0, 1:], axis=0), 0):
            raise ValueError("boostback engines must form a symmetric cluster")

    def _validate(self):
        c = self.c
        positive = [
            "max_rate_rad_s",
            "max_accel_rad_s2",
            "attitude_bandwidth",
            "burn_bandwidth",
            "attitude_tolerance_deg",
            "rate_tolerance_rad_s",
            "hold_s",
            "maneuver_timeout_s",
            "burn_timeout_s",
            "cutoff_timeout_s",
            "coast_s",
            "interface_altitude_m",
            "target_tolerance_m",
            "delta_v_mps",
            "thrust_off_n",
            "burn_abort_angle_deg",
        ]
        if any(not np.isfinite(float(c[k])) or float(c[k]) <= 0 for k in positive):
            raise ValueError("mission times, limits and gains must be finite and positive")
        if c.cutoff_mode not in ("target", "delta_v", "duration"):
            raise ValueError("cutoff_mode must be target, delta_v or duration")
        if not 0 < c.throttle <= 1 or c.throttle < max(self.p.engines.throttle_min):
            raise ValueError("mission throttle must be within the engine operating range")
        if any(
            not np.isfinite(float(c[k])) or c[k] < 0 for k in ("main_reserve_kg", "rcs_reserve_kg")
        ):
            raise ValueError("fuel reserves must be finite and nonnegative")
        quantum = float(c.rcs_duty_quantum)
        if (
            not np.isfinite(quantum)
            or not 0 < quantum <= 1
            or not np.isclose(1 / quantum, round(1 / quantum))
        ):
            raise ValueError("RCS duty quantum must divide one")
        if bool(self.p.rcs.pulsed) and not np.isclose(
            quantum * float(self.p.rcs.pulse_ticks), round(quantum * float(self.p.rcs.pulse_ticks))
        ):
            raise ValueError("RCS duty quantum must be representable on the plant grid")
        engines = list(c.engines)
        if (
            not engines
            or len(set(engines)) != len(engines)
            or any(
                not isinstance(i, int) or not 0 <= i < len(self.p.engines.positions)
                for i in engines
            )
        ):
            raise ValueError("mission engine indices must be unique and valid")
        if np.shape(c.target_ne_m) != (2,) or not np.all(np.isfinite(c.target_ne_m)):
            raise ValueError("target_ne_m must contain two finite coordinates")

    def _transition(self, phase, reason, state, aux):
        previous = self.phase
        self.phase = phase
        self.phase_start = float(state.t)
        self.aligned_since = None
        self.events.append(
            dict(
                t=float(state.t),
                kind="mission_transition",
                phase_before=previous,
                phase_after=phase,
                label=PHASES[phase],
                trigger=reason,
                configuration_before=1,
                configuration_after=1,
                time_uncertainty_s=self.dt,
                state={
                    k: np.asarray(getattr(state, k)).tolist()
                    for k in ("r_ned", "v_ned", "q_bn", "w_body", "m")
                },
                main_propellant_kg=float(aux["main_propellant"]),
                rcs_fuel_kg=float(aux["rcs_fuel"]),
            )
        )

    def _abort(self, reason, state, aux):
        self.failure = reason
        self._transition(17, reason, state, aux)

    def _projection(self, state, aux):
        return ballistic_projection(
            state.r_ned, state.v_ned, float(aux["gravity_ned"][2]), self.c.interface_altitude_m
        )

    def _error(self, q):
        return float(
            (
                Rotation.from_quat(q, scalar_first=True).inv()
                * Rotation.from_quat(self.q_ref, scalar_first=True)
            ).magnitude()
        )

    def _settled(self, state, error):
        good = (
            error <= np.deg2rad(self.c.attitude_tolerance_deg)
            and np.linalg.norm(state.w_body) <= self.c.rate_tolerance_rad_s
        )
        if not good:
            self.aligned_since = None
        elif self.aligned_since is None:
            self.aligned_since = float(state.t)
        return good and state.t - self.aligned_since >= self.c.hold_s - 1e-8

    def update(self, state, aux):
        c, p = self.c, self.p
        time = float(state.t)
        inertia = np.asarray(aux["inertia_diag"])
        C = Rotation.from_quat(state.q_bn, scalar_first=True).as_matrix()
        if self.phase is None:
            self.start = time
            self.initial_main = float(aux["main_propellant"])
            self.initial_rcs = float(aux["rcs_fuel"])
            self.q_ref = np.array(state.q_bn)
            self._transition(10, "separation_handoff", state, aux)
        dt = 0 if self.last_t is None else time - self.last_t
        accel = C @ np.asarray(aux["T_body"]) / state.m
        self.burn_dv += 0.5 * (accel + self.last_accel) * dt
        self.last_accel, self.last_t = accel, time
        projection = self._projection(state, aux)
        miss = float(np.linalg.norm(projection[1] - c.target_ne_m)) if projection else None
        if self.phase in (10, 11, 12, 13):
            if np.linalg.norm(state.w_body) > 1.25 * c.max_rate_rad_s:
                self._abort("angular_rate_limit", state, aux)
            elif aux["rcs_fuel"] <= c.rcs_reserve_kg + self._rcs_guard():
                self._abort("rcs_reserve", state, aux)
            elif -state.r_ned[2] <= c.interface_altitude_m:
                self._abort("below_targeting_interface", state, aux)
            elif self.phase != 13 and time - self.start > c.maneuver_timeout_s:
                self._abort("maneuver_timeout", state, aux)
        if self.phase == 10 and self._settled(state, 0.0):
            if projection is None:
                self._abort("no_descending_projection", state, aux)
            else:
                desired_velocity = (np.asarray(c.target_ne_m) - state.r_ned[:2]) / projection[0]
                delta = desired_velocity - state.v_ned[:2]
                if np.linalg.norm(delta) < 1e-8:
                    self._abort("no_boostback_direction", state, aux)
                else:
                    self.q_ref = pointing_target(state.q_bn, np.r_[delta, 0.0])
                    self.slew = Slew(state.q_bn, self.q_ref, c.max_rate_rad_s, c.max_accel_rad_s2)
                    # Torque impulse / lever arm / exhaust speed, including both ramps.
                    estimate = np.sum(2 * inertia * np.abs(self.slew.axis) * self.slew.rate) / (
                        float(p.actuator.rcs_arm) * float(p.actuator.rcs_isp) * 9.80665
                    )
                    self.feasibility = dict(
                        rotation_deg=float(np.rad2deg(self.slew.angle)),
                        nominal_slew_s=self.slew.duration,
                        ideal_slew_rcs_kg=float(estimate),
                        available_rcs_kg=float(aux["rcs_fuel"] - c.rcs_reserve_kg),
                        required_horizontal_delta_v_mps=float(np.linalg.norm(delta)),
                        ideal_main_delta_v_with_reserve_mps=float(
                            float(p.engines.isp[0])
                            * 9.80665
                            * np.log(
                                state.m
                                / (float(p.mass.m_dry) + aux["rcs_fuel"] + c.main_reserve_kg)
                            )
                        ),
                    )
                    if estimate > aux["rcs_fuel"] - c.rcs_reserve_kg:
                        self._abort("insufficient_slew_budget", state, aux)
                    else:
                        self.final_q = self.q_ref.copy()
                        self._transition(11, "rate_settled", state, aux)
        a_ref = np.zeros(3)
        self.w_ref = np.zeros(3)
        if self.phase == 11:
            self.q_ref, self.w_ref, a_ref = self.slew.sample(time - self.phase_start)
            if time - self.phase_start >= self.slew.duration:
                self.q_ref = self.final_q.copy()
                self._transition(12, "reference_slew_complete", state, aux)
        error = self._error(state.q_bn)
        if self.phase == 12 and self._settled(state, error):
            if aux["main_propellant"] <= c.main_reserve_kg + self._main_guard():
                self._abort("insufficient_main_budget", state, aux)
            else:
                self.burn_start = time
                self.initial_miss = miss
                self._transition(13, "attitude_aligned", state, aux)
        if self.phase == 13:
            reason = None
            if error > np.deg2rad(c.burn_abort_angle_deg):
                self.failure = reason = "burn_pointing_error"
            elif aux["main_propellant"] <= c.main_reserve_kg + self._main_guard():
                reason = "main_reserve"
            elif projection is None:
                self.failure = reason = "no_descending_projection"
            elif c.cutoff_mode == "target" and miss <= c.target_tolerance_m:
                reason = "projected_target_reached"
            elif c.cutoff_mode == "delta_v" and np.linalg.norm(self.burn_dv) >= c.delta_v_mps:
                reason = "prescribed_delta_v_reached"
            elif time - self.burn_start >= c.burn_timeout_s:
                reason = (
                    "prescribed_duration_reached" if c.cutoff_mode == "duration" else "burn_timeout"
                )
            if reason:
                self.cutoff_reason = reason
                self._transition(14, reason, state, aux)
        if self.phase == 14:
            if aux["thrust_mag"] <= c.thrust_off_n:
                self.target_met = bool(miss is not None and miss <= c.target_tolerance_m)
                self.cutoff_miss = miss
                self._transition(15, "actual_thrust_off", state, aux)
            elif time - self.phase_start > c.cutoff_timeout_s:
                self._abort("cutoff_timeout", state, aux)
        if self.phase == 15 and time - self.phase_start >= c.coast_s - 1e-8:
            self._transition(16, "coast_complete", state, aux)
            self.complete = True
        if self.phase == 17:
            # Propagate the real shutdown transient even after a failed guard.
            self.complete = bool(aux["thrust_mag"] <= c.thrust_off_n)
        throttle = self.active * c.throttle if self.phase == 13 else np.zeros_like(self.active)
        gimbal = np.zeros((len(p.engines.tvc_bias), 2))
        rcs = np.zeros(3)
        if self.phase in (10, 11, 12, 13, 14):
            # Live inertia; stowed fins have no allocation authority. Main TVC authority
            # uses delivered per-engine forces, so ignition/shutdown cannot invent torque.
            wn = c.burn_bandwidth if self.phase in (13, 14) else c.attitude_bandwidth
            law = PDController(2 * inertia * wn**2, 2 * 0.9 * inertia * wn)
            command = Command(
                self.phase,
                int(self.phase != 13),
                np.zeros(3),
                self.q_ref,
                np.zeros(3),
                self.active,
                self.w_ref,
            )
            command, _ = law(state, command, {})
            tau = (
                command.tau_body + inertia * a_ref + np.cross(state.w_body, inertia * state.w_body)
            )
            arm = float(aux["x_cg"]) - np.asarray(p.engines.positions)[:, 0]
            force = np.asarray(aux["engine_force_body"])
            authority = float(np.sum(np.linalg.norm(force, axis=1) * arm))
            realized = np.zeros(3)
            if authority > 1.0:
                # For aft nozzles, pitch torque is -arm*T*sin(pitch), yaw is -arm*T*sin(yaw).
                pitch = np.clip(
                    -tau[1] / authority,
                    -float(p.actuator.tvc_deflection_max_rad),
                    float(p.actuator.tvc_deflection_max_rad),
                )
                yaw = np.clip(
                    -tau[2] / authority,
                    -float(p.actuator.tvc_deflection_max_rad),
                    float(p.actuator.tvc_deflection_max_rad),
                )
                gimbal[:] = [pitch, yaw]
                realized[1:] = [
                    -authority * np.sin(pitch),
                    -authority * np.cos(pitch) * np.sin(yaw),
                ]
            rcs = np.clip((tau - realized) / self.authority, -1, 1)
            if bool(p.rcs.pulsed):
                # Hold physical PWM resolution fixed when refining the integration grid.
                quantum = float(c.rcs_duty_quantum)
                rcs = np.sign(rcs) * np.floor(np.abs(rcs) / quantum + 0.5) * quantum
            if aux["rcs_fuel"] <= c.rcs_reserve_kg + self._rcs_guard():
                rcs[:] = 0
        self.records.append(
            dict(
                t=time,
                phase=self.phase,
                q_ref=self.q_ref.copy(),
                w_ref=self.w_ref.copy(),
                attitude_error_rad=error,
                rate_rad_s=float(np.linalg.norm(state.w_body)),
                projected_miss_m=miss,
                main_propellant_kg=float(aux["main_propellant"]),
                rcs_fuel_kg=float(aux["rcs_fuel"]),
                thrust_delta_v_mps=float(np.linalg.norm(self.burn_dv)),
            )
        )
        return ActuatorCommand(throttle, gimbal, np.zeros(len(p.aero.fin_arm)), rcs)

    def capture(self, session, command):
        if not self.c.get("capture_reference_windows", False):
            return
        if self.phase in (10, 11, 13, 14) and self.phase not in self.reference_windows:
            self.reference_windows[self.phase] = []
        for rows in self.reference_windows.values():
            if not rows or (rows[-1]["state"]["t"] - rows[0]["state"]["t"] < 1.0 - 1e-8):
                rows.append(dict(state=session.snapshot(full=True), command=command))

    def _rcs_guard(self):
        return (
            6
            * float(self.p.actuator.rcs_thrust)
            / (float(self.p.actuator.rcs_isp) * 9.80665)
            * self.dt
        )

    def _main_guard(self):
        # Upper bound on burning time after cutoff: throttle starts at <=1 and
        # stops producing flow below min_throttle-0.1. Include one control interval.
        a = self.p.actuator
        threshold = float(np.min(self.p.engines.throttle_min)) - 0.1
        shutdown_s = (1 - threshold) / (
            float(a.throttle_rate_max)
            * np.tanh(threshold / (float(a.throttle_tau) * float(a.throttle_rate_max)))
        )
        flow = np.sum(
            np.asarray(self.p.engines.thrust_vac)
            * self.active
            / (np.asarray(self.p.engines.isp) * 9.80665)
        )
        return float(flow * (shutdown_s + self.dt))

    def summary(self):
        return dict(
            completed=self.phase == 16,
            failure=self.failure,
            phase=PHASES.get(self.phase, "not_started"),
            cutoff_reason=self.cutoff_reason,
            target_projection_met=self.target_met,
            landing_validated=False,
            projection_model=(
                "constant local gravity, no drag, descending interface; diagnostic only"
            ),
            feasibility=self.feasibility,
            projected_miss_at_ignition_m=getattr(self, "initial_miss", None),
            projected_miss_at_thrust_off_m=getattr(self, "cutoff_miss", None),
            thrust_delta_v_mps=float(np.linalg.norm(self.burn_dv)),
        )
