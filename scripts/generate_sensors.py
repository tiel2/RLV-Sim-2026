"""Generate reproducible synthetic sensor observations from RLV-Sim trajectory.csv.

OFFLINE prototype: measurements are not fed back to the simulator. IMU samples
are evaluated on the logged trajectory grid (no fabricated high-rate dynamics).
Frame: NED for GNSS; body axes for IMU; altitude positive above ground.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

G0 = 9.80665


def rotate_ned_to_body(q, vectors):
    """q=[w,x,y,z] represents body-to-NED attitude; return R(q)^T v_NED."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    v = np.asarray(vectors)
    # R(q)^T * v
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    return np.column_stack(
        (
            r00 * v[:, 0] + r10 * v[:, 1] + r20 * v[:, 2],
            r01 * v[:, 0] + r11 * v[:, 1] + r21 * v[:, 2],
            r02 * v[:, 0] + r12 * v[:, 1] + r22 * v[:, 2],
        )
    )


def subsample(frame, period):
    """Nearest logged sample per measurement period; avoid interpolation across impact."""
    t = frame["t"].to_numpy(dtype=float)
    bins = np.floor((t - t[0] + 1e-8) / period).astype(int)
    return frame.loc[~pd.Series(bins).duplicated().to_numpy()].copy()


def gauss_markov_rw(rng, n, dt, sigma_per_sqrt_s):
    increments = (
        rng.standard_normal((n, 3)) * sigma_per_sqrt_s * np.sqrt(np.maximum(dt, 0))[:, None]
    )
    return np.cumsum(increments, axis=0)


def generate(path, out, seed=42, gps_dropout=0.03):
    rng = np.random.default_rng(seed)
    truth = pd.read_csv(path).sort_values("t").drop_duplicates("t").reset_index(drop=True)
    needed = [
        "t",
        "r_N",
        "r_E",
        "r_D",
        "v_N",
        "v_E",
        "v_D",
        "q_w",
        "q_x",
        "q_y",
        "q_z",
        "w_x",
        "w_y",
        "w_z",
    ]
    missing = set(needed) - set(truth.columns)
    if missing:
        raise ValueError(f"Missing columns: {sorted(missing)}")
    if len(truth) < 3:
        raise ValueError("Need at least three samples to estimate acceleration")
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    t = truth.t.to_numpy()
    dt = np.diff(t, prepend=t[0])
    dt[0] = np.median(np.diff(t))
    v = truth[["v_N", "v_E", "v_D"]].to_numpy()
    q = truth[["q_w", "q_x", "q_y", "q_z"]].to_numpy()
    # Offline approximation: numerical derivative of logged velocity minus gravity.
    # The native plant's inverse-square field is used as a close approximation.
    a_ned = np.column_stack([np.gradient(v[:, i], t) for i in range(3)])
    h = np.maximum(-truth.r_D.to_numpy(), 0)
    mu = 3.986004418e14
    earth_radius = 6371000.0
    g = mu / (earth_radius + h) ** 2
    f_body = rotate_ned_to_body(q, a_ned - np.column_stack([np.zeros(len(t)), np.zeros(len(t)), g]))
    omega = truth[["w_x", "w_y", "w_z"]].to_numpy()
    # Illustrative noise/bias settings, not calibrated hardware specifications.
    accel_bias = np.array([0.04, -0.03, 0.08]) + gauss_markov_rw(rng, len(t), dt, 0.001)
    gyro_bias = np.deg2rad([0.02, -0.01, 0.025]) + gauss_markov_rw(
        rng, len(t), dt, np.deg2rad(0.0004)
    )
    imu = truth[["t"]].copy()
    for j, axis in enumerate("xyz"):
        imu[f"accel_{axis}_mps2"] = f_body[:, j] + accel_bias[:, j] + rng.normal(0, 0.08, len(t))
        imu[f"gyro_{axis}_radps"] = (
            omega[:, j] + gyro_bias[:, j] + rng.normal(0, np.deg2rad(0.025), len(t))
        )
        imu[f"truth_accel_{axis}_mps2"] = f_body[:, j]
        imu[f"truth_gyro_{axis}_radps"] = omega[:, j]
    imu.to_csv(out / "imu.csv", index=False)
    gps = subsample(truth, 1.0)
    for ax in "NED":
        gps[f"pos_{ax}_m"] = gps[f"r_{ax}"] + rng.normal(0, 2.5, len(gps))
        gps[f"vel_{ax}_mps"] = gps[f"v_{ax}"] + rng.normal(0, 0.15, len(gps))
    gps["valid"] = rng.random(len(gps)) >= gps_dropout
    gps.loc[~gps.valid, [f"pos_{a}_m" for a in "NED"] + [f"vel_{a}_mps" for a in "NED"]] = np.nan
    gps[["t", "valid"] + [f"pos_{a}_m" for a in "NED"] + [f"vel_{a}_mps" for a in "NED"]].to_csv(
        out / "gnss.csv", index=False
    )
    # Pressure model provides ambient pressure rather than direct altitude.
    # This simple isothermal model is deliberately separate from the USSA plant.
    baro = subsample(truth, 0.2)
    alt = -baro.r_D.to_numpy()
    p0 = 101325.0
    H = 8500.0
    p = p0 * np.exp(-np.maximum(alt, 0) / H)
    baro_out = pd.DataFrame(
        {"t": baro.t, "pressure_pa": p + rng.normal(0, 3.0, len(p)), "truth_altitude_m": alt}
    )
    baro_out.to_csv(out / "barometer.csv", index=False)
    radar = subsample(truth, 0.2)
    alt = -radar.r_D.to_numpy()
    valid = (alt >= 0) & (alt <= 1500)
    radar_out = pd.DataFrame(
        {
            "t": radar.t,
            "valid": valid,
            "range_m": np.where(valid, alt + rng.normal(0, 0.5, len(alt)), np.nan),
            "truth_altitude_m": alt,
        }
    )
    radar_out.to_csv(out / "radar_altimeter.csv", index=False)
    print(
        f"Wrote {len(imu)} IMU, {len(gps)} GNSS, {len(baro_out)} barometer "
        f"and {len(radar_out)} radar samples to {out}"
    )
    print(
        "Note: IMU output rate equals trajectory logging rate; "
        "do not use this for high-rate INS integration."
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--out", default="results/synthetic_sensors")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--gps-dropout", type=float, default=0.03)
    a = parser.parse_args()
    generate(a.trajectory, a.out, a.seed, a.gps_dropout)
