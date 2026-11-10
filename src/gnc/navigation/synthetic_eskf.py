"""Synthetic CG-mounted IMU/GNSS/radial altimeter and rotating-frame 15-state ESKF.

Only the sensor generator receives native force/rate truth. The estimator consumes
samples, an uncertain alignment observation, and a declared position/velocity prior.
Mass is an explicitly ideal auxiliary measurement, not an estimated filter state.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, replace

import numpy as np

from core.interfaces import Measurement


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)


def qnorm(q):
    a = np.asarray(q, float)
    n = np.linalg.norm(a)
    if a.shape != (4,) or not np.isfinite(n) or n == 0:
        raise ValueError("attitude must be a finite nonzero quaternion")
    return a / n


def qmul(a, b):
    w, x, y, z = a
    W, X, Y, Z = b
    return np.array(
        [
            w * W - x * X - y * Y - z * Z,
            w * X + x * W + y * Z - z * Y,
            w * Y - x * Z + y * W + z * X,
            w * Z + x * Y - y * X + z * W,
        ]
    )


def dq_body(dtheta):
    angle = np.linalg.norm(dtheta)
    if angle < 1e-12:
        return qnorm(np.r_[1.0, 0.5 * np.asarray(dtheta)])
    return np.r_[np.cos(angle / 2), np.sin(angle / 2) * np.asarray(dtheta) / angle]


def rot(q):
    w, x, y, z = qnorm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def left_jacobian(phi):
    """SO(3) reset for a left (reference-frame) attitude error."""
    angle = np.linalg.norm(phi)
    A = skew(phi)
    if angle < 1e-5:
        return np.eye(3) + 0.5 * A + A @ A / 6
    return (
        np.eye(3)
        + (1 - np.cos(angle)) / angle**2 * A
        + (angle - np.sin(angle)) / angle**3 * (A @ A)
    )


@dataclass(frozen=True)
class NavigationFrame:
    """Declared reference constants, independent of realized plant state/faults."""

    rotating: bool = False
    omega: tuple = (0.0, 0.0, 0.0)
    radius: float = 6371000.0
    mu: float = 3.986004418e14
    inverse_square: bool = True
    uniform_g: float = 9.80665

    def height_gradient(self, r):
        r = np.asarray(r, float)
        if not self.rotating:
            return -r[2], np.array([0.0, 0.0, -1.0])
        R = r + [0.0, 0.0, -self.radius]
        distance = np.linalg.norm(R)
        # Rationalized altitude avoids cancellation near the surface.
        h = (r @ r - 2 * self.radius * r[2]) / (distance + self.radius)
        return h, R / distance

    def acceleration_gradient(self, r, v):
        r = np.asarray(r, float)
        W = skew(self.omega)
        if self.rotating:
            R = r + [0.0, 0.0, -self.radius]
            d = np.linalg.norm(R)
            g = -self.mu * R / d**3
            J = self.mu * (3 * np.outer(R, R) / d**5 - np.eye(3) / d**3)
            return g - 2 * W @ v - W @ W @ R, J - W @ W
        if self.inverse_square:
            # Fixed-Earth plant uses a vertical field and clamps subsurface height.
            h = max(-r[2], 0.0)
            g = self.mu / (self.radius + h) ** 2
            J = np.zeros((3, 3))
            if r[2] < 0:
                J[2, 2] = 2 * self.mu / (self.radius + h) ** 3
            return np.array([0.0, 0.0, g]), J
        return np.array([0.0, 0.0, self.uniform_g]), np.zeros((3, 3))


@dataclass(frozen=True)
class SensorMeasurement(Measurement):
    accel_body: object = None
    gyro_body: object = None  # inertial body rate; not body/reference rate
    gnss_valid: bool = False
    radar_alt_m: float = float("nan")
    radar_valid: bool = False
    alignment_valid: bool = False
    gnss_scheduled: bool = False
    radar_scheduled: bool = False
    gnss_outlier: bool = False


class SyntheticESKF:
    def __init__(
        self,
        *,
        imu_accel_sigma=0.08,
        imu_gyro_sigma_deg=0.025,
        accel_bias=(0.025, -0.02, 0.035),
        gyro_bias_deg=(0.005, -0.006, 0.004),
        gnss_pos_sigma=2.5,
        gnss_vel_sigma=0.15,
        gnss_dropout=0.03,
        gnss_period=0.1,
        radar_sigma=0.5,
        radar_period=0.05,
        radar_range=1500.0,
        gnss_outage_start=-1.0,
        gnss_outage_end=-1.0,
        seed_offset=0,
        frame=None,
        alignment_sigma_deg=1.0,
        alignment_error_deg=(0.0, 0.0, 0.0),
        initial_position=None,
        initial_velocity=None,
        prior_position_sigma=25.0,
        prior_velocity_sigma=2.0,
        gyro_bias_rw_deg=0.0005,
        accel_bias_rw=0.0005,
        imu_rate_hz=100.0,
        accel_scale_factor=(0.0, 0.0, 0.0),
        gyro_scale_factor=(0.0, 0.0, 0.0),
        accel_bias_random_walk=0.0,
        gyro_bias_random_walk_deg=0.0,
        gnss_outlier_times=(),
        gnss_outlier_position_m=(100.0, -100.0, 50.0),
        gnss_outlier_velocity_mps=(10.0, -10.0, 5.0),
        gnss_gate_threshold=None,
        radar_gate_threshold=None,
    ):
        self.sa = float(imu_accel_sigma)
        self.sg = float(np.deg2rad(imu_gyro_sigma_deg))
        self.ba = np.asarray(accel_bias, float)
        self.bg = np.deg2rad(np.asarray(gyro_bias_deg, float))
        self.sp, self.sv = float(gnss_pos_sigma), float(gnss_vel_sigma)
        self.drop = float(gnss_dropout)
        self.gperiod, self.rperiod = float(gnss_period), float(radar_period)
        self.srad, self.rrange = float(radar_sigma), float(radar_range)
        self.outage0, self.outage1 = float(gnss_outage_start), float(gnss_outage_end)
        self.seed_offset = int(seed_offset)
        self.frame = frame or NavigationFrame()
        self.alignment_sigma = float(np.deg2rad(alignment_sigma_deg))
        self.alignment_error = np.deg2rad(np.asarray(alignment_error_deg, float))
        self.initial_position = (
            None if initial_position is None else np.array(initial_position, dtype=float)
        )
        self.initial_velocity = (
            None if initial_velocity is None else np.array(initial_velocity, dtype=float)
        )
        self.prior_sp, self.prior_sv = float(prior_position_sigma), float(prior_velocity_sigma)
        self.sbg, self.sba = np.deg2rad(gyro_bias_rw_deg), float(accel_bias_rw)
        self.imu_rate_hz = float(imu_rate_hz)
        if not np.isfinite(self.imu_rate_hz) or self.imu_rate_hz <= 0:
            raise ValueError("imu_rate_hz must be finite and positive")
        self.imu_period = 1.0 / self.imu_rate_hz
        self.accel_scale = np.asarray(accel_scale_factor, float)
        self.gyro_scale = np.asarray(gyro_scale_factor, float)
        self.accel_walk = float(accel_bias_random_walk)
        self.gyro_walk = float(np.deg2rad(gyro_bias_random_walk_deg))
        self.outlier_times = np.asarray(gnss_outlier_times, float)
        self.outlier_position = np.asarray(gnss_outlier_position_m, float)
        self.outlier_velocity = np.asarray(gnss_outlier_velocity_mps, float)
        self.gnss_gate = None if gnss_gate_threshold is None else float(gnss_gate_threshold)
        self.radar_gate = None if radar_gate_threshold is None else float(radar_gate_threshold)
        self._walk_ba = np.zeros(3)
        self._walk_bg = np.zeros(3)
        self._sensor_epoch = None
        if (
            min(
                self.gperiod,
                self.rperiod,
                self.sp,
                self.sv,
                self.srad,
                self.prior_sp,
                self.prior_sv,
            )
            <= 0
            or not 0 <= self.drop <= 1
        ):
            raise ValueError(
                "sensor periods/aiding uncertainties must be positive; dropout in [0,1]"
            )
        if min(self.sa, self.sg, self.alignment_sigma, self.sbg, self.sba) < 0:
            raise ValueError("noise amplitudes must be nonnegative")

        vectors = (
            self.ba,
            self.bg,
            self.alignment_error,
            self.accel_scale,
            self.gyro_scale,
            self.outlier_position,
            self.outlier_velocity,
        )
        if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in vectors):
            raise ValueError(
                "sensor bias, scale, alignment and outlier parameters must be finite 3-vectors"
            )
        scalars = (
            self.sa,
            self.sg,
            self.sp,
            self.sv,
            self.srad,
            self.gperiod,
            self.rperiod,
            self.rrange,
            self.alignment_sigma,
            self.sbg,
            self.sba,
            self.accel_walk,
            self.gyro_walk,
            self.prior_sp,
            self.prior_sv,
        )
        if not np.all(np.isfinite(scalars)) or min(self.accel_walk, self.gyro_walk) < 0:
            raise ValueError("sensor parameters must be finite and random walk nonnegative")
        for period in (self.gperiod, self.rperiod):
            if not np.isclose(period / self.imu_period, round(period / self.imu_period), atol=1e-9):
                raise ValueError("GNSS/radar periods must be integer multiples of the IMU period")
        for gate in (self.gnss_gate, self.radar_gate):
            if gate is not None and (not np.isfinite(gate) or gate <= 0):
                raise ValueError("innovation gates must be finite positive thresholds or null")
        if (
            self.outlier_times.ndim != 1
            or not np.all(np.isfinite(self.outlier_times))
            or np.any(self.outlier_times < 0)
            or any(not self._due(t, self.gperiod) for t in self.outlier_times)
        ):
            raise ValueError("outlier times must lie on the nonnegative GNSS grid")

    def initial_memory(self):
        return {}

    def _rng(self, seed, t, channel):
        ints = [int(v) for v in np.asarray(seed).reshape(-1)]
        ss = np.random.SeedSequence(
            [
                *(x & 0xFFFFFFFF for x in ints),
                int(round(t * 1e6)) & 0xFFFFFFFF,
                int(channel),
                self.seed_offset & 0xFFFFFFFF,
            ]
        )
        return np.random.default_rng(ss)

    @staticmethod
    def _due(t, period):
        return abs(t / period - round(t / period)) < 1e-5

    def measure_native(self, state, seed, diagnostics):
        """Specific force at the modeled CG includes support force, excludes gravitation."""
        return self.measure(
            state,
            seed,
            specific_force_body=(
                np.asarray(diagnostics["F_body_total"]) + np.asarray(diagnostics["F_pad_body"])
            )
            / float(state.m),
            inertial_rate_body=diagnostics["omega_inertial_body"],
        )

    def measure(self, state, seed, *, specific_force_body, inertial_rate_body):
        """Sample supplied physical channels; never infer acceleration from state differences."""
        t = float(state.t)
        p, v = np.asarray(state.r_ned, float), np.asarray(state.v_ned, float)
        reset = self._sensor_epoch is None or t <= self._sensor_epoch
        if reset:
            self._walk_ba = np.zeros(3)
            self._walk_bg = np.zeros(3)
        else:
            dt = t - self._sensor_epoch
            if not np.isclose(dt, self.imu_period, rtol=0, atol=1e-8):
                raise ValueError("sensor epoch spacing does not match configured period")
            self._walk_ba += (
                self.accel_walk * np.sqrt(dt) * self._rng(seed, t, 8).standard_normal(3)
            )
            self._walk_bg += self.gyro_walk * np.sqrt(dt) * self._rng(seed, t, 9).standard_normal(3)
        accel = (
            np.asarray(specific_force_body, float) * (1 + self.accel_scale)
            + self.ba
            + self._walk_ba
            + self.sa * self._rng(seed, t, 1).standard_normal(3)
        )
        gyro = (
            np.asarray(inertial_rate_body, float) * (1 + self.gyro_scale)
            + self.bg
            + self._walk_bg
            + self.sg * self._rng(seed, t, 2).standard_normal(3)
        )
        gps_on = self._due(t, self.gperiod)
        gps_on &= not (self.outage0 <= t <= self.outage1 and self.outage1 > self.outage0)
        if gps_on:
            gps_on = self._rng(seed, t, 3).random() >= self.drop
        gp = (
            p + self.sp * self._rng(seed, t, 4).standard_normal(3) if gps_on else np.full(3, np.nan)
        )
        gv = (
            v + self.sv * self._rng(seed, t, 5).standard_normal(3) if gps_on else np.full(3, np.nan)
        )
        outlier = bool(gps_on and np.any(np.isclose(t, self.outlier_times, rtol=0, atol=1e-8)))
        if outlier:
            gp += self.outlier_position
            gv += self.outlier_velocity
        height, _ = self.frame.height_gradient(p)
        radar_on = self._due(t, self.rperiod) and 0 <= height <= self.rrange
        ra = height + self.srad * self._rng(seed, t, 6).standard_normal() if radar_on else np.nan
        alignment = self._sensor_epoch is None or t <= self._sensor_epoch
        q = np.full(4, np.nan)
        if alignment:
            error = self.alignment_error + self.alignment_sigma * self._rng(
                seed, t, 7
            ).standard_normal(3)
            q = qnorm(qmul(dq_body(error), state.q_bn))
        self._sensor_epoch = t
        return SensorMeasurement(
            r_ned=gp,
            v_ned=gv,
            q_bn=q,
            w_body=gyro,
            m=float(state.m),
            t=t,
            accel_body=accel,
            gyro_body=gyro,
            gnss_valid=bool(gps_on),
            radar_alt_m=float(ra),
            radar_valid=bool(radar_on),
            alignment_valid=alignment,
            gnss_scheduled=self._due(t, self.gperiod),
            radar_scheduled=self._due(t, self.rperiod),
            gnss_outlier=outlier,
        )

    @staticmethod
    def _update(mem, z, h, H, R, channel, gate=None):
        P = mem["P"]
        innovation = z - h
        S = H @ P @ H.T + R
        nis = float(innovation @ np.linalg.solve(S, innovation))
        accepted = gate is None or nis <= gate
        normalized = innovation / np.sqrt(np.diag(S))
        mem[f"{channel}_nis"] = nis
        mem[f"{channel}_normalized_innovation"] = normalized
        mem[f"{channel}_update_accepted"] = accepted
        mem["innovations"].append(dict(channel=channel, dof=len(z), nis=nis, accepted=accepted))
        if not accepted:
            return False
        K = np.linalg.solve(S, (P @ H.T).T).T
        dx = K @ innovation
        mem["r"] += dx[:3]
        mem["v"] += dx[3:6]
        mem["q"] = qnorm(qmul(dq_body(dx[6:9]), mem["q"]))
        mem["bg"] += dx[9:12]
        mem["ba"] += dx[12:15]
        A = np.eye(15) - K @ H
        posterior = A @ P @ A.T + K @ R @ K.T
        reset = np.eye(15)
        reset[6:9, 6:9] = left_jacobian(dx[6:9])
        posterior = reset @ posterior @ reset.T
        mem["P"] = (posterior + posterior.T) / 2
        return True

    def _initialize(self, meas):
        if not meas.alignment_valid:
            raise ValueError("navigation initialization requires an alignment observation")
        if meas.gnss_valid:
            r, v = np.array(meas.r_ned, float), np.array(meas.v_ned, float)
            sp, sv, source = self.sp, self.sv, "gnss"
        else:
            if self.initial_position is None or self.initial_velocity is None:
                raise ValueError("GNSS unavailable: supply an explicit position/velocity prior")
            r, v = self.initial_position.copy(), self.initial_velocity.copy()
            sp, sv, source = self.prior_sp, self.prior_sv, "declared_prior"
        if not (r.shape == v.shape == (3,) and np.all(np.isfinite(np.r_[r, v]))):
            raise ValueError("initial position and velocity must be finite 3-vectors")
        P = np.diag(
            [sp**2] * 3
            + [sv**2] * 3
            + [max(self.alignment_sigma, 1e-8) ** 2] * 3
            + [np.deg2rad(0.05) ** 2] * 3
            + [0.1**2] * 3
        )
        return dict(
            r=r,
            v=v,
            q=qnorm(meas.q_bn),
            bg=np.zeros(3),
            ba=np.zeros(3),
            P=P,
            t=float(meas.t),
            initialization_source=source,
            first_gnss_time=None,
            previous_accel=np.array(meas.accel_body),
            previous_gyro=np.array(meas.gyro_body),
            innovations=[],
        )

    def linearization(self, r, v, C, f):
        _, J = self.frame.acceleration_gradient(r, v)
        W = skew(self.frame.omega)
        F = np.zeros((15, 15))
        F[:3, 3:6] = np.eye(3)
        F[3:6, :3], F[3:6, 3:6] = J, -2 * W
        F[3:6, 6:9], F[3:6, 12:15] = -skew(C @ f), -C
        F[6:9, 6:9], F[6:9, 9:12] = -W, -C
        return F

    def _propagate(self, mem, meas):
        dt = float(meas.t) - mem["t"]
        if dt <= 0:
            raise ValueError("navigation timestamps must increase")
        w = (mem["previous_gyro"] + np.asarray(meas.gyro_body)) / 2 - mem["bg"]
        f = (mem["previous_accel"] + np.asarray(meas.accel_body)) / 2 - mem["ba"]
        omega = np.asarray(self.frame.omega)
        qmid = qmul(dq_body(-omega * dt / 2), qmul(mem["q"], dq_body(w * dt / 2)))
        C = rot(qmid)
        force = C @ f

        # RK4 of translation with midpoint specific force and full frame acceleration.
        def rhs(x):
            a, _ = self.frame.acceleration_gradient(x[:3], x[3:])
            return np.r_[x[3:], force + a]

        x = np.r_[mem["r"], mem["v"]]
        k1 = rhs(x)
        k2 = rhs(x + dt * k1 / 2)
        k3 = rhs(x + dt * k2 / 2)
        k4 = rhs(x + dt * k3)
        F = self.linearization(mem["r"], mem["v"], C, f)
        x += dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        mem["r"], mem["v"] = x[:3], x[3:]
        mem["q"] = qnorm(qmul(dq_body(-omega * dt), qmul(mem["q"], dq_body(w * dt))))
        Phi = np.eye(15) + F * dt + (F @ F) * (dt * dt / 2)
        G = np.zeros((15, 12))
        G[3:6, :3], G[6:9, 3:6] = C, C
        G[9:12, 6:9], G[12:15, 9:12] = np.eye(3), np.eye(3)
        B = (np.eye(15) + F * dt / 2) @ G
        variance = np.array(
            [self.sa**2 * dt**2] * 3
            + [self.sg**2 * dt**2] * 3
            + [self.sbg**2 * dt] * 3
            + [self.sba**2 * dt] * 3
        )
        P = Phi @ mem["P"] @ Phi.T + (B * variance) @ B.T
        mem["P"] = (P + P.T) / 2
        mem["t"] = float(meas.t)
        mem["previous_accel"] = np.array(meas.accel_body)
        mem["previous_gyro"] = np.array(meas.gyro_body)

    def estimate(self, meas, act, memory):
        initializing = "P" not in memory
        mem = (
            self._initialize(meas)
            if initializing
            else {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in memory.items()
            }
        )
        mem["innovations"] = []
        for channel, count in (("gnss", 6), ("radar", 1)):
            mem[f"{channel}_nis"] = float("nan")
            mem[f"{channel}_normalized_innovation"] = np.full(count, np.nan)
            mem[f"{channel}_update_accepted"] = False
        if not initializing:
            self._propagate(mem, meas)
        if meas.gnss_valid:
            accepted = True
            if not initializing:  # the initialization sample is used exactly once
                H = np.zeros((6, 15))
                H[:6, :6] = np.eye(6)
                accepted = self._update(
                    mem,
                    np.r_[meas.r_ned, meas.v_ned],
                    np.r_[mem["r"], mem["v"]],
                    H,
                    np.diag([self.sp**2] * 3 + [self.sv**2] * 3),
                    "gnss",
                    self.gnss_gate,
                )
            else:
                mem["gnss_update_accepted"] = True
            if accepted and mem["first_gnss_time"] is None:
                mem["first_gnss_time"] = float(meas.t)
        if meas.radar_valid:
            height, gradient = self.frame.height_gradient(mem["r"])
            H = np.zeros((1, 15))
            H[0, :3] = gradient
            self._update(
                mem,
                np.array([meas.radar_alt_m]),
                np.array([height]),
                H,
                np.array([[self.srad**2]]),
                "radar",
                self.radar_gate,
            )
        # Control uses Earth-relative body rate; the physical gyro observes inertial rate.
        w = np.asarray(meas.gyro_body) - mem["bg"] - rot(mem["q"]).T @ self.frame.omega
        return Measurement(
            r_ned=mem["r"].copy(),
            v_ned=mem["v"].copy(),
            q_bn=mem["q"].copy(),
            w_body=w,
            m=meas.m,
            t=float(meas.t),
        ), mem


class UnfilteredINS:
    """Same initialized mechanization, with all aiding disabled after the first sample."""

    def __init__(self, navigation):
        self.navigation = copy.deepcopy(navigation)

    def initial_memory(self):
        return {}

    def estimate(self, measurement, act, memory):
        if "P" in memory:
            measurement = replace(measurement, gnss_valid=False, radar_valid=False)
        return self.navigation.estimate(measurement, act, memory)


def build(ctx):
    cfg = ctx.cfg
    kwargs = {
        k: cfg[k]
        for k in (
            "imu_rate_hz",
            "accel_scale_factor",
            "gyro_scale_factor",
            "accel_bias_random_walk",
            "gyro_bias_random_walk_deg",
            "gnss_outlier_times",
            "gnss_outlier_position_m",
            "gnss_outlier_velocity_mps",
            "gnss_gate_threshold",
            "radar_gate_threshold",
            "imu_accel_sigma",
            "imu_gyro_sigma_deg",
            "accel_bias",
            "gyro_bias_deg",
            "gnss_pos_sigma",
            "gnss_vel_sigma",
            "gnss_dropout",
            "gnss_period",
            "radar_sigma",
            "radar_period",
            "radar_range",
            "gnss_outage_start",
            "gnss_outage_end",
            "seed_offset",
            "alignment_sigma_deg",
            "alignment_error_deg",
            "prior_position_sigma",
            "prior_velocity_sigma",
            "gyro_bias_rw_deg",
            "accel_bias_rw",
        )
        if k in cfg
    }
    period = getattr(ctx, "nav_dt", None)
    if period is not None and not np.isclose(
        period, 1 / float(kwargs.get("imu_rate_hz", 100)), rtol=0, atol=1e-10
    ):
        raise ValueError("synthetic_eskf requires nav_dt to match 1 / imu_rate_hz")
    reference = ctx.navigation_reference
    if reference is None:
        raise ValueError("synthetic_eskf requires declared navigation frame and initial prior")
    kwargs.update(
        frame=NavigationFrame(**reference["frame"]),
        initial_position=np.asarray(reference["position"])
        + np.asarray(cfg.get("prior_position_offset", [0, 0, 0])),
        initial_velocity=np.asarray(reference["velocity"])
        + np.asarray(cfg.get("prior_velocity_offset", [0, 0, 0])),
    )
    return SyntheticESKF(**kwargs)
