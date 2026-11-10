"""Synthetic IMU/GNSS/radar sensors and a 15-state error-state INS filter.

The sensor model is a deterministic research example, not a hardware specification.
It assumes a fixed-origin NED frame, zero sensor latency, known initial state/alignment,
and constant mass. Accelerometer samples are specific force in body axes.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.interfaces import Measurement


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def qnorm(quaternion: np.ndarray) -> np.ndarray:
    value = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(value)
    if value.shape != (4,) or not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("attitude quaternion must be a finite nonzero 4-vector")
    return value / norm


def qmul(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w, x, y, z = left
    W, X, Y, Z = right
    return np.array(
        [
            w * W - x * X - y * Y - z * Z,
            w * X + x * W + y * Z - z * Y,
            w * Y - x * Z + y * W + z * X,
            w * Z + x * Y - y * X + z * W,
        ]
    )


def dq_body(rotation: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation))
    if angle < 1e-12:
        return qnorm(np.r_[1.0, 0.5 * np.asarray(rotation, dtype=float)])
    axis = np.asarray(rotation, dtype=float) / angle
    return np.r_[np.cos(angle / 2.0), np.sin(angle / 2.0) * axis]


def rot(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = qnorm(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _vector(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return result.copy()


def _sigma(value: float, name: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return result


@dataclass(frozen=True)
class SensorMeasurement(Measurement):
    """Synchronous sensor packet; GNSS position/velocity are NaN when unavailable."""

    accel_body: np.ndarray | None = None
    gyro_body: np.ndarray | None = None
    gnss_scheduled: bool = False
    gnss_valid: bool = False
    gnss_outlier: bool = False
    radar_scheduled: bool = False
    radar_alt_m: float = float("nan")
    radar_valid: bool = False


class SyntheticESKF:
    """Deterministic IMU, GNSS, radar, and aided INS sensor/estimator module."""

    def __init__(
        self,
        *,
        imu_rate_hz: float = 200.0,
        imu_accel_sigma: float = 0.08,
        imu_gyro_sigma_deg: float = 0.025,
        accel_bias=(0.025, -0.02, 0.035),
        gyro_bias_deg=(0.005, -0.006, 0.004),
        accel_scale_factor=(0.0, 0.0, 0.0),
        gyro_scale_factor=(0.0, 0.0, 0.0),
        accel_bias_random_walk: float = 0.0,
        gyro_bias_random_walk_deg: float = 0.0,
        gnss_pos_sigma: float = 2.5,
        gnss_vel_sigma: float = 0.15,
        gnss_dropout: float = 0.03,
        gnss_period: float = 0.1,
        gnss_outlier_times=(),
        gnss_outlier_position_m=(100.0, -100.0, 50.0),
        gnss_outlier_velocity_mps=(10.0, -10.0, 5.0),
        gnss_gate_threshold: float = 16.812,
        radar_sigma: float = 0.5,
        radar_period: float = 0.05,
        radar_range: float = 1500.0,
        radar_gate_threshold: float = 6.635,
        gnss_outage_start: float = -1.0,
        gnss_outage_end: float = -1.0,
        seed_offset: int = 0,
        initial_position_sigma_m: float = 5.0,
        initial_velocity_sigma_mps: float = 2.0,
        initial_attitude_sigma_deg: float = 1.0,
        initial_gyro_bias_sigma_deg: float = 0.05,
        initial_accel_bias_sigma_mps2: float = 0.1,
        initial_state=None,
        initial_position=None,
        initial_velocity=None,
        initial_attitude=None,
        initial_mass: float | None = None,
    ):
        self.imu_rate_hz = self._positive(imu_rate_hz, "imu_rate_hz")
        self.imu_period = 1.0 / self.imu_rate_hz
        self.sa = _sigma(imu_accel_sigma, "imu_accel_sigma")
        self.sg = np.deg2rad(_sigma(imu_gyro_sigma_deg, "imu_gyro_sigma_deg"))
        self.ba = _vector(accel_bias, "accel_bias")
        self.bg = np.deg2rad(_vector(gyro_bias_deg, "gyro_bias_deg"))
        self.accel_scale = _vector(accel_scale_factor, "accel_scale_factor")
        self.gyro_scale = _vector(gyro_scale_factor, "gyro_scale_factor")
        self.ba_rw = _sigma(accel_bias_random_walk, "accel_bias_random_walk")
        self.bg_rw = np.deg2rad(
            _sigma(gyro_bias_random_walk_deg, "gyro_bias_random_walk_deg")
        )
        self.sp = _sigma(gnss_pos_sigma, "gnss_pos_sigma")
        self.sv = _sigma(gnss_vel_sigma, "gnss_vel_sigma")
        self.drop = float(gnss_dropout)
        if not np.isfinite(self.drop) or not 0.0 <= self.drop <= 1.0:
            raise ValueError("gnss_dropout must be between zero and one")
        self.gperiod = self._positive(gnss_period, "gnss_period")
        gnss_ratio = self.gperiod / self.imu_period
        if not np.isclose(gnss_ratio, round(gnss_ratio), rtol=0.0, atol=1e-8):
            raise ValueError("gnss_period must be an integer multiple of the IMU period")
        self.outlier_times = tuple(float(value) for value in gnss_outlier_times)
        if not np.all(np.isfinite(self.outlier_times)):
            raise ValueError("gnss_outlier_times must contain only finite times")
        self.outlier_position = _vector(
            gnss_outlier_position_m, "gnss_outlier_position_m"
        )
        self.outlier_velocity = _vector(
            gnss_outlier_velocity_mps, "gnss_outlier_velocity_mps"
        )
        self.gnss_gate = self._positive(gnss_gate_threshold, "gnss_gate_threshold")
        self.srad = _sigma(radar_sigma, "radar_sigma")
        self.rperiod = self._positive(radar_period, "radar_period")
        radar_ratio = self.rperiod / self.imu_period
        if not np.isclose(radar_ratio, round(radar_ratio), rtol=0.0, atol=1e-8):
            raise ValueError("radar_period must be an integer multiple of the IMU period")
        self.rrange = self._positive(radar_range, "radar_range")
        self.radar_gate = self._positive(radar_gate_threshold, "radar_gate_threshold")
        self.outage0 = float(gnss_outage_start)
        self.outage1 = float(gnss_outage_end)
        if not np.isfinite(self.outage0) or not np.isfinite(self.outage1):
            raise ValueError("GNSS outage endpoints must be finite")
        self.initial_sigmas = np.array(
            [
                *([_sigma(initial_position_sigma_m, "initial_position_sigma_m")] * 3),
                *([_sigma(initial_velocity_sigma_mps, "initial_velocity_sigma_mps")] * 3),
                *(
                    [np.deg2rad(_sigma(initial_attitude_sigma_deg, "initial_attitude_sigma_deg"))]
                    * 3
                ),
                *(
                    [np.deg2rad(_sigma(initial_gyro_bias_sigma_deg, "initial_gyro_bias_sigma_deg"))]
                    * 3
                ),
                *([_sigma(initial_accel_bias_sigma_mps2, "initial_accel_bias_sigma_mps2")] * 3),
            ]
        )
        self.seed_offset = int(seed_offset)
        self.initial_position = self._optional_vector(initial_position, "initial_position")
        self.initial_velocity = self._optional_vector(initial_velocity, "initial_velocity")
        self.initial_attitude = (
            qnorm(initial_attitude) if initial_attitude is not None else np.array([1.0, 0, 0, 0])
        )
        self.initial_mass = None if initial_mass is None else float(initial_mass)
        if self.initial_mass is not None and (
            not np.isfinite(self.initial_mass) or self.initial_mass <= 0.0
        ):
            raise ValueError("initial_mass must be finite and positive")
        if initial_state is not None:
            self.initial_position = _vector(initial_state.r_ned, "initial_state.r_ned")
            self.initial_velocity = _vector(initial_state.v_ned, "initial_state.v_ned")
            self.initial_attitude = qnorm(initial_state.q_bn)
            self.initial_mass = float(initial_state.m)
        self._prev_truth: tuple[float, np.ndarray] | None = None
        self._walk_ba = np.zeros(3)
        self._walk_bg = np.zeros(3)

    @staticmethod
    def _positive(value: float, name: str) -> float:
        result = float(value)
        if not np.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return result

    @staticmethod
    def _optional_vector(value, name: str) -> np.ndarray | None:
        return None if value is None else _vector(value, name)

    @staticmethod
    def _due(t: float, period: float) -> bool:
        return abs(t / period - round(t / period)) < 1e-5

    def _rng(self, seed, t: float, channel: int) -> np.random.Generator:
        values = np.asarray(seed, dtype=np.uint64).reshape(-1)
        if not values.size:
            raise ValueError("sensor seed must contain at least one integer")
        words = [int(value) & 0xFFFFFFFF for value in values]
        sequence = np.random.SeedSequence(
            [*words, int(round(t * 1e6)) & 0xFFFFFFFF, channel, self.seed_offset & 0xFFFFFFFF]
        )
        return np.random.default_rng(sequence)

    def measure(self, state, seed) -> SensorMeasurement:
        t = float(state.t)
        if not np.isfinite(t):
            raise ValueError("sensor state time must be finite")
        p = _vector(state.r_ned, "state.r_ned")
        v = _vector(state.v_ned, "state.v_ned")
        q_truth = qnorm(state.q_bn)
        w = _vector(state.w_body, "state.w_body")
        if self._prev_truth is None:
            # The first sample has no velocity history; initialize with zero translational accel.
            acceleration_ned = np.zeros(3)
        else:
            previous_t, previous_v = self._prev_truth
            dt = t - previous_t
            if dt <= 0.0:
                raise ValueError("sensor truth timestamps must increase")
            if not np.isclose(dt, self.imu_period, rtol=0.0, atol=1e-8):
                raise ValueError(
                    f"IMU sample interval {dt:g} s does not match configured "
                    f"period {self.imu_period:g} s"
                )
            acceleration_ned = (v - previous_v) / dt
            self._walk_ba += (
                self.ba_rw
                * np.sqrt(dt)
                * self._rng(seed, t, 10).standard_normal(3)
            )
            self._walk_bg += (
                self.bg_rw
                * np.sqrt(dt)
                * self._rng(seed, t, 11).standard_normal(3)
            )
        self._prev_truth = (t, v.copy())

        altitude = max(-p[2], 0.0)
        gravity = 9.80665 * (6_371_000.0 / (6_371_000.0 + altitude)) ** 2
        gravity_ned = np.array([0.0, 0.0, gravity])
        specific_force = rot(q_truth).T @ (acceleration_ned - gravity_ned)
        accel = (
            (1.0 + self.accel_scale) * specific_force
            + self.ba
            + self._walk_ba
            + self.sa * self._rng(seed, t, 1).standard_normal(3)
        )
        gyro = (
            (1.0 + self.gyro_scale) * w
            + self.bg
            + self._walk_bg
            + self.sg * self._rng(seed, t, 2).standard_normal(3)
        )

        gnss_scheduled = self._due(t, self.gperiod)
        gnss_valid = gnss_scheduled
        gnss_valid &= not (
            self.outage0 <= t <= self.outage1 and self.outage1 > self.outage0
        )
        if gnss_valid:
            gnss_valid = self._rng(seed, t, 3).random() >= self.drop
        if gnss_valid:
            position = p + self.sp * self._rng(seed, t, 4).standard_normal(3)
            velocity = v + self.sv * self._rng(seed, t, 5).standard_normal(3)
        else:
            position = np.full(3, np.nan)
            velocity = np.full(3, np.nan)
        gnss_outlier = gnss_valid and any(
            np.isclose(t, event_time, rtol=0.0, atol=1e-8)
            for event_time in self.outlier_times
        )
        if gnss_outlier:
            position += self.outlier_position
            velocity += self.outlier_velocity

        radar_scheduled = self._due(t, self.rperiod)
        radar_valid = radar_scheduled and 0.0 <= -p[2] <= self.rrange
        radar_altitude = (
            -p[2] + self.srad * self._rng(seed, t, 6).standard_normal()
            if radar_valid
            else float("nan")
        )
        if self.initial_mass is None:
            self.initial_mass = float(state.m)

        return SensorMeasurement(
            r_ned=position,
            v_ned=velocity,
            q_bn=self.initial_attitude.copy(),
            w_body=gyro.copy(),
            m=self.initial_mass,
            t=t,
            accel_body=accel,
            gyro_body=gyro,
            gnss_scheduled=bool(gnss_scheduled),
            gnss_valid=bool(gnss_valid),
            gnss_outlier=bool(gnss_outlier),
            radar_scheduled=bool(radar_scheduled),
            radar_alt_m=float(radar_altitude),
            radar_valid=bool(radar_valid),
        )

    def initial_memory(self):
        return {}

    def _initial_filter_state(self, measurement: SensorMeasurement) -> dict:
        position = self.initial_position
        velocity = self.initial_velocity
        if position is None:
            position = (
                np.nan_to_num(np.asarray(measurement.r_ned, dtype=float))
                if measurement.gnss_valid
                else np.zeros(3)
            )
        if velocity is None:
            velocity = (
                np.nan_to_num(np.asarray(measurement.v_ned, dtype=float))
                if measurement.gnss_valid
                else np.zeros(3)
            )
        mass = self.initial_mass if self.initial_mass is not None else float(measurement.m)
        return {
            "r": np.array(position, dtype=float, copy=True),
            "v": np.array(velocity, dtype=float, copy=True),
            "q": self.initial_attitude.copy(),
            "bg": np.zeros(3),
            "ba": np.zeros(3),
            "m": float(mass),
            "t": float(measurement.t),
        }

    def _propagate(self, memory: dict, measurement: SensorMeasurement) -> dict:
        mem = {
            key: value.copy() if isinstance(value, np.ndarray) else value
            for key, value in memory.items()
        }
        mem["previous_t"] = mem["t"]
        mem["previous_q"] = mem["q"].copy()
        t = float(measurement.t)
        dt = t - mem["t"]
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("navigation timestamps must increase")
        if measurement.accel_body is None or measurement.gyro_body is None:
            raise ValueError("INS propagation requires accelerometer and gyro samples")
        accel = _vector(measurement.accel_body, "accel_body") - mem["ba"]
        gyro = _vector(measurement.gyro_body, "gyro_body") - mem["bg"]
        c_bn = rot(mem["q"])
        altitude = max(-mem["r"][2], 0.0)
        gravity = 9.80665 * (6_371_000.0 / (6_371_000.0 + altitude)) ** 2
        acceleration = c_bn @ accel + np.array([0.0, 0.0, gravity])
        mem["r"] += mem["v"] * dt + 0.5 * acceleration * dt**2
        mem["v"] += acceleration * dt
        mem["q"] = qnorm(qmul(mem["q"], dq_body(gyro * dt)))
        mem["t"] = t
        return mem

    @staticmethod
    def _estimate_record(memory: dict, measurement: SensorMeasurement) -> Measurement:
        gyro = _vector(measurement.gyro_body, "gyro_body") - memory["bg"]
        return Measurement(
            r_ned=memory["r"].copy(),
            v_ned=memory["v"].copy(),
            q_bn=memory["q"].copy(),
            w_body=gyro,
            m=memory["m"],
            t=float(measurement.t),
        )

    def estimate(self, measurement: SensorMeasurement, act, memory):
        mem = (
            self._initial_filter_state(measurement)
            if "t" not in memory
            else self._propagate(memory, measurement)
        )
        if "P" not in mem:
            mem["P"] = np.diag(self.initial_sigmas**2)
            mem["previous_t"] = mem["t"]
            mem["previous_q"] = mem["q"].copy()
        else:
            self._predict_covariance(mem, measurement)

        mem["gnss_nis"] = float("nan")
        mem["gnss_normalized_innovation"] = np.full(6, np.nan)
        mem["gnss_update_accepted"] = False
        mem["radar_nis"] = float("nan")
        mem["radar_normalized_innovation"] = np.full(1, np.nan)
        mem["radar_update_accepted"] = False
        if measurement.gnss_valid:
            h = np.zeros((6, 15))
            h[:6, :6] = np.eye(6)
            observed = np.r_[measurement.r_ned, measurement.v_ned]
            predicted = np.r_[mem["r"], mem["v"]]
            covariance = np.diag([self.sp**2] * 3 + [self.sv**2] * 3)
            (
                mem["gnss_nis"],
                mem["gnss_normalized_innovation"],
                mem["gnss_update_accepted"],
            ) = self._update(mem, observed, predicted, h, covariance, self.gnss_gate)
        if measurement.radar_valid:
            h = np.zeros((1, 15))
            h[0, 2] = -1.0
            (
                mem["radar_nis"],
                mem["radar_normalized_innovation"],
                mem["radar_update_accepted"],
            ) = self._update(
                mem,
                np.array([measurement.radar_alt_m]),
                np.array([-mem["r"][2]]),
                h,
                np.array([[self.srad**2]]),
                self.radar_gate,
            )
        return self._estimate_record(mem, measurement), mem

    def _predict_covariance(self, memory: dict, measurement: SensorMeasurement) -> None:
        dt = float(measurement.t) - float(memory["previous_t"])
        c_bn = rot(memory["previous_q"])
        specific_force = _vector(measurement.accel_body, "accel_body") - memory["ba"]
        f_ned = c_bn @ specific_force
        f = np.zeros((15, 15))
        f[:3, 3:6] = np.eye(3)
        f[3:6, 6:9] = -skew(f_ned)
        f[3:6, 12:15] = -c_bn
        f[6:9, 9:12] = -c_bn
        transition = np.eye(15) + f * dt + 0.5 * (f @ f) * dt**2

        q = np.zeros((15, 15))
        accel_cov = np.eye(3) * self.sa**2
        q[:3, :3] = accel_cov * (dt**4 / 4.0)
        q[:3, 3:6] = accel_cov * (dt**3 / 2.0)
        q[3:6, :3] = q[:3, 3:6].T
        q[3:6, 3:6] = accel_cov * dt**2
        q[6:9, 6:9] = np.eye(3) * self.sg**2 * dt**2
        q[9:12, 9:12] = np.eye(3) * self.bg_rw**2 * dt
        q[12:15, 12:15] = np.eye(3) * self.ba_rw**2 * dt

        previous_p = memory["P"]
        memory["P"] = transition @ previous_p @ transition.T + q
        memory["P"] = 0.5 * (memory["P"] + memory["P"].T)
        memory["previous_t"] = float(measurement.t)
        memory["previous_q"] = memory["q"].copy()

    @staticmethod
    def _update(memory: dict, observed, predicted, h, covariance, gate_threshold):
        p = memory["P"]
        innovation_covariance = h @ p @ h.T + covariance
        innovation = observed - predicted
        normalized_innovation = innovation / np.sqrt(np.diag(innovation_covariance))
        nis = float(innovation @ np.linalg.solve(innovation_covariance, innovation))
        if nis > gate_threshold:
            return nis, normalized_innovation, False
        gain = np.linalg.solve(innovation_covariance, (p @ h.T).T).T
        correction = gain @ innovation
        memory["r"] += correction[:3]
        memory["v"] += correction[3:6]
        memory["q"] = qnorm(qmul(dq_body(correction[6:9]), memory["q"]))
        memory["bg"] += correction[9:12]
        memory["ba"] += correction[12:15]
        residual = np.eye(15) - gain @ h
        updated = residual @ p @ residual.T + gain @ covariance @ gain.T
        memory["P"] = 0.5 * (updated + updated.T)
        return nis, normalized_innovation, True


class UnfilteredINS:
    """IMU-only strapdown baseline using the same initialization and sensor samples."""

    def __init__(self, initial_state=None):
        self.initial_position = (
            _vector(initial_state.r_ned, "initial_state.r_ned")
            if initial_state is not None
            else None
        )
        self.initial_velocity = (
            _vector(initial_state.v_ned, "initial_state.v_ned")
            if initial_state is not None
            else None
        )
        self.initial_attitude = (
            qnorm(initial_state.q_bn)
            if initial_state is not None
            else np.array([1.0, 0.0, 0.0, 0.0])
        )
        self.initial_mass = float(initial_state.m) if initial_state is not None else None
        self._mechanizer = SyntheticESKF(
            initial_position=self.initial_position,
            initial_velocity=self.initial_velocity,
            initial_attitude=self.initial_attitude,
            initial_mass=self.initial_mass,
        )

    def initial_memory(self):
        return {}

    def estimate(self, measurement: SensorMeasurement, act, memory):
        if "t" not in memory:
            memory = self._mechanizer._initial_filter_state(measurement)
            memory.update(bg=np.zeros(3), ba=np.zeros(3))
        else:
            memory = self._mechanizer._propagate(memory, measurement)
        return self._mechanizer._estimate_record(memory, measurement), memory


def build(ctx):
    """Factory named by configs/gnc/navigation/synthetic_eskf.yaml."""
    cfg = ctx.cfg
    kwargs = {
        key: cfg[key]
        for key in (
            "imu_rate_hz",
            "imu_accel_sigma",
            "imu_gyro_sigma_deg",
            "accel_bias",
            "gyro_bias_deg",
            "accel_scale_factor",
            "gyro_scale_factor",
            "accel_bias_random_walk",
            "gyro_bias_random_walk_deg",
            "gnss_pos_sigma",
            "gnss_vel_sigma",
            "gnss_dropout",
            "gnss_period",
            "gnss_outlier_times",
            "gnss_outlier_position_m",
            "gnss_outlier_velocity_mps",
            "gnss_gate_threshold",
            "radar_sigma",
            "radar_period",
            "radar_range",
            "radar_gate_threshold",
            "gnss_outage_start",
            "gnss_outage_end",
            "seed_offset",
            "initial_position_sigma_m",
            "initial_velocity_sigma_mps",
            "initial_attitude_sigma_deg",
            "initial_gyro_bias_sigma_deg",
            "initial_accel_bias_sigma_mps2",
        )
        if key in cfg
    }
    nav_dt = getattr(ctx, "nav_dt", None)
    if nav_dt is not None and not np.isclose(
        float(nav_dt), 1.0 / float(kwargs["imu_rate_hz"]), rtol=0.0, atol=1e-8
    ):
        raise ValueError(
            "synthetic_eskf requires scenario.horizon.nav_dt to match "
            "1 / gnc.navigation.imu_rate_hz"
        )
    return SyntheticESKF(initial_state=getattr(ctx, "initial_state", None), **kwargs)
