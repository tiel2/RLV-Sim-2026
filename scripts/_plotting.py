"""Shared plotting for episode/batch diagnostics."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from core.frames import euler_from_quat


def _aux_series(result, key):
    out = []
    for a in result.aux.get("per_tick", []):
        out.append(float(np.asarray(a[key]))) if key in a else out.append(np.nan)
    return np.array(out)


def plot_episode(result, out_dir: Path, title: str = "episode"):
    """6-panel summary: trajectory, speed, attitude, rates, mass, thrust/aero."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    t = result.t
    r, v, w, m = result.r_ned, result.v_ned, result.w_body, result.m
    alt = -r[:, 2]

    eul = np.array([euler_from_quat(q) for q in result.q_bn])  # (T,3) roll,pitch,yaw
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))

    ax[0, 0].plot(t, alt, label="altitude")
    ax[0, 0].plot(t, r[:, 0], label="north")
    ax[0, 0].plot(t, r[:, 1], label="east")
    ax[0, 0].set(xlabel="t [s]", ylabel="position [m]", title="trajectory")
    ax[0, 0].legend()
    ax[0, 0].grid(True)

    ax[0, 1].plot(t, v[:, 0], label="vN")
    ax[0, 1].plot(t, v[:, 1], label="vE")
    ax[0, 1].plot(t, v[:, 2], label="vD")
    ax[0, 1].plot(t, np.linalg.norm(v, axis=1), "k--", label="|v|")
    ax[0, 1].set(xlabel="t [s]", ylabel="velocity [m/s]", title="velocity NED")
    ax[0, 1].legend()
    ax[0, 1].grid(True)

    ax[0, 2].plot(t, np.rad2deg(eul[:, 0]), label="roll")
    ax[0, 2].plot(t, np.rad2deg(eul[:, 1]), label="pitch")
    ax[0, 2].plot(t, np.rad2deg(eul[:, 2]), label="yaw")
    ax[0, 2].set(xlabel="t [s]", ylabel="euler [deg]", title="attitude")
    ax[0, 2].legend()
    ax[0, 2].grid(True)

    ax[1, 0].plot(t, np.rad2deg(w[:, 0]), label="p")
    ax[1, 0].plot(t, np.rad2deg(w[:, 1]), label="q")
    ax[1, 0].plot(t, np.rad2deg(w[:, 2]), label="r")
    ax[1, 0].set(xlabel="t [s]", ylabel="body rate [deg/s]", title="angular rates")
    ax[1, 0].legend()
    ax[1, 0].grid(True)

    ax[1, 1].plot(t, m)
    ax[1, 1].set(xlabel="t [s]", ylabel="mass [kg]", title="mass")
    ax[1, 1].grid(True)

    thrust = _aux_series(result, "thrust_mag")
    mach = _aux_series(result, "mach")
    alpha = np.rad2deg(_aux_series(result, "aero_alpha"))
    ax2 = ax[1, 2].twinx()
    ax[1, 2].plot(t, thrust / 1e3, "b-", label="thrust [kN]")
    ax2.plot(t, mach, "r-", label="Mach")
    ax2.plot(t, alpha, "g-", label="alpha [deg]")
    ax[1, 2].set(xlabel="t [s]", ylabel="thrust [kN]", title="thrust / aero")
    ax[1, 2].legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax[1, 2].grid(True)

    fig.suptitle(title)
    fig.tight_layout()
    path = out_dir / "episode_summary.png"
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def plot_plan_tracking(result, out_dir: Path, title: str = "plan tracking"):
    """3x2 plan-vs-flown: rows = NED components, left col position, right col velocity.

    Red dashed = the ignition-committed guidance plan (nodes at t_ignition + tau);
    blue solid = the flown trajectory. Saved after every run for tracking diagnosis."""
    plan = result.aux.get("plan_ignition")
    t_ig = result.aux.get("t_ignition")
    if plan is None or t_ig is None:
        return None
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.asarray(result.t)
    r = np.asarray(result.r_ned)
    v = np.asarray(result.v_ned)
    t_plan = float(t_ig) + np.linspace(0.0, float(plan["tf"]), int(plan["N"]) + 1)
    r_plan = np.asarray(plan["r"])
    v_plan = np.asarray(plan["v"])
    labels = ["N [m]", "E [m]", "D [m]"]
    fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    for i in range(3):
        axes[i, 0].plot(t_plan, r_plan[:, i], "--", color=(1.0, 0.0, 0.0), label="plan")
        axes[i, 0].plot(t, r[:, i], "-", color=(0.0, 0.0, 1.0), label="flown")
        axes[i, 0].set_ylabel(f"r {labels[i]}")
        axes[i, 1].plot(t_plan, v_plan[:, i], "--", color=(1.0, 0.0, 0.0), label="plan")
        axes[i, 1].plot(t, v[:, i], "-", color=(0.0, 0.0, 1.0), label="flown")
        axes[i, 1].set_ylabel(f"v {labels[i]} /s")
        for ax in (axes[i, 0], axes[i, 1]):
            ax.grid(True, alpha=0.3)
    axes[0, 0].legend(loc="best")
    axes[2, 0].set_xlabel("t [s]")
    axes[2, 1].set_xlabel("t [s]")
    fig.suptitle(title)
    fig.tight_layout()
    path = out_dir / "plan_tracking.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_physics(data, out_dir, reference=None):
    """Physical quantities and optional analytical reference on the truth timeline."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    t = data["t"]
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    axes[0, 0].plot(t, data["altitude_m"], label="altitude")
    axes[0, 0].plot(t, data["downrange_m"], label="horizontal distance from start")
    axes[0, 0].set_ylabel("m")
    axes[0, 1].plot(t, np.rad2deg(data["flight_path_angle_rad"]), label="flight-path angle")
    axes[0, 1].plot(t, np.rad2deg(data["aero_alpha"]), label="angle of attack")
    axes[0, 1].set_ylabel("deg")
    axes[1, 0].plot(t, data["aero_q_dyn"] / 1000, label="dynamic pressure")
    axes[1, 0].set_ylabel("kPa")
    axes[1, 1].plot(t, data["main_propellant"], label="main propellant")
    axes[1, 1].plot(t, data["rcs_fuel"], label="RCS propellant")
    axes[1, 1].set_ylabel("kg")
    axes[2, 0].plot(t, data["x_cg"], label="axial CG")
    axes[2, 0].set_ylabel("body x [m]")
    for axis, label in enumerate(("Ixx", "Iyy", "Izz")):
        axes[2, 1].plot(t, data["inertia_diag"][:, axis], label=label)
    axes[2, 1].set_ylabel("kg m²")
    for ax in axes.flat:
        ax.set_xlabel("t [s]")
        ax.grid(True)
        ax.legend()
    fig.tight_layout()
    fig.savefig(out / "physics.png", dpi=120)
    plt.close(fig)
    reference = reference or {}
    pairs = [
        (key, data[key], reference[key]) for key in ("r_ned", "v_ned", "mass") if key in reference
    ]
    for key in ("specific_energy", "angular_momentum", "rotational_energy"):
        if key in reference:
            pairs.append((key, reference[key], reference["expected_" + key]))
    units = {
        "r_ned": "m",
        "v_ned": "m/s",
        "mass": "kg",
        "specific_energy": "J/kg",
        "angular_momentum": "kg m²/s",
        "rotational_energy": "J",
    }
    if pairs:
        fig, axes = plt.subplots(len(pairs), 2, figsize=(11, 3 * len(pairs)), squeeze=False)
        for row, (key, actual, expected) in enumerate(pairs):
            actual, expected = np.asarray(actual), np.asarray(expected)
            if actual.ndim == 1:
                actual, expected = actual[:, None], expected[:, None]
            for column in range(actual.shape[1]):
                label = ("N", "E", "D")[column] if actual.shape[1] == 3 else key
                color = f"C{column}"
                axes[row, 0].plot(t, actual[:, column], color=color, label=label)
                axes[row, 0].plot(t, expected[:, column], "--", color=color)
                axes[row, 1].plot(t, actual[:, column] - expected[:, column], color=color)
            axes[row, 0].set_title(f"{key}: simulated (solid), reference (dashed)")
            axes[row, 1].set_title(f"{key}: signed error")
            axes[row, 0].legend()
            for ax in axes[row]:
                ax.set_ylabel(units[key])
        for ax in axes.flat:
            ax.set_xlabel("t [s]")
            ax.grid(True)
        fig.tight_layout()
        fig.savefig(out / "analytical_comparison.png", dpi=120)
        plt.close(fig)
