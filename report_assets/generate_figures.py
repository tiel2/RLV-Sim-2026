"""Regenerate report figures and metrics from archived runs; never rerun the plant.

Run from the repository root: .venv/bin/python report_assets/generate_figures.py
"""

import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from omegaconf import OmegaConf

from validation.flight_reference import error_statistics
from validation.segment_spec import load_snapshot, observation

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "report_assets"
BASE = ROOT / "results/verification/project2_segments/physics"
CASES = ["ascent", "entry_burn", "aero_descent", "powered_descent", "landing"]
NAMES = ["Powered ascent", "Entry burn", "Aerodynamic descent", "Powered descent", "Landing"]
plt.rcParams.update(
    {
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "savefig.dpi": 170,
        "axes.prop_cycle": plt.cycler(color=["#245780", "#c36526", "#45836a"]),
    }
)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / (name + ".pdf"), metadata={"CreationDate": None})
    fig.savefig(OUT / (name + ".png"))
    plt.close(fig)


def format_axes(axes, xlabel="Elapsed time [s]"):
    for ax in np.asarray(axes).flat:
        ax.grid(alpha=0.2)
        ax.set_xlabel(xlabel)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


manifest, rt, rh, rv, events = load_snapshot(ROOT / "data/reference/crs11")
metrics = {
    "basis": "2026-11-03 archived campaign; figures reprocessed without plant execution",
    "reference_commit": manifest["commit"],
    "sources": {},
    "cases": {},
}
source_paths = [ROOT / "data/reference/crs11" / f["file"] for f in manifest["files"]]
source_paths += [
    BASE / "summary.json",
    Path(__file__),
    ROOT / "src/validation/flight_reference.py",
    ROOT / "src/validation/segment_spec.py",
]
overlays, oa = plt.subplots(4, 2, figsize=(9.4, 10.5))
residuals, ra = plt.subplots(4, 2, figsize=(9.4, 10.5))
attitudes, aa = plt.subplots(5, 2, figsize=(9.4, 11.5))
massfig, ma = plt.subplots(2, 2, figsize=(9.4, 6))

for idx, (case, title) in enumerate(zip(CASES, NAMES, strict=True)):
    folder = BASE / case
    d = np.load(folder / "physics.npz")
    cfg = OmegaConf.load(folder / "resolved_config.yaml")
    verification = json.loads((folder / "verification.json").read_text())
    source_paths += [
        folder / f for f in ["physics.npz", "resolved_config.yaml", "verification.json"]
    ]
    if case != "landing":
        source_paths.append(folder / "convergence.json")
    t = d["t"]
    fig, axes = plt.subplots(3, 2, figsize=(9.4, 7.0))
    axes[0, 0].plot(t, d["altitude_m"] / 1000, label="Altitude")
    axes[0, 0].plot(t, d["downrange_m"] / 1000, label="Distance from origin")
    axes[0, 0].set_ylabel("Distance [km]")
    axes[0, 0].legend(fontsize=8)
    for ax, key, scale, label in [
        (axes[0, 1], "speed_mps", 1, "Ground speed [m/s]"),
        (axes[1, 0], "thrust_mag", 1000, "Main-engine thrust [kN]"),
        (axes[1, 1], "aero_q_dyn", 1000, "Dynamic pressure [kPa]"),
        (axes[2, 0], "main_propellant", 1, "Main propellant [kg]"),
        (axes[2, 1], "aero_alpha", np.pi / 180, "Angle of attack [deg]"),
    ]:
        ax.plot(t, d[key] / scale)
        ax.set_ylabel(label)
    format_axes(axes)
    save(fig, case)

    # Body +x inclination to radial upward direction avoids Euler singularities.
    q = d["q_bn"]
    w, x, y, z = q.T
    body_x = np.column_stack([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
    tilt = np.rad2deg(np.arccos(np.clip(np.sum(body_x * -d["local_down_ned"], axis=1), -1, 1)))
    aa[idx, 0].plot(t, tilt, label="Body-axis tilt")
    aa[idx, 0].plot(t, np.rad2deg(d["flight_path_angle_rad"]), label="Flight-path angle")
    aa[idx, 0].set_ylabel(title + "\nAngle [deg]")
    aa[idx, 1].plot(t, np.rad2deg(d["w_body"]))
    aa[idx, 1].set_ylabel("Body rate [deg/s]")
    if idx == 0:
        aa[idx, 0].legend(fontsize=8)
        aa[idx, 1].legend(["p", "q", "r"], fontsize=8)

    if case in ["ascent", "entry_burn"]:
        row = 0 if case == "ascent" else 1
        ma[row, 0].plot(t, d["x_cg"])
        ma[row, 0].set_ylabel(title + "\nAxial CG [m]")
        ma[row, 1].plot(t, d["inertia_diag"] / 1e6)
        ma[row, 1].set_ylabel("Inertia [10$^6$ kg m$^2$]")
        ma[row, 1].legend(["Ixx", "Iyy", "Izz"], fontsize=8)

    qi = int(np.argmax(d["aero_q_dyn"]))
    m = {
        "run_id": cfg.run.run_id,
        "verification_status": verification["status"],
        "final_altitude_m": float(d["altitude_m"][-1]),
        "final_speed_mps": float(d["speed_mps"][-1]),
        "final_main_propellant_kg": float(d["main_propellant"][-1]),
        "max_q_kpa": float(d["aero_q_dyn"][qi] / 1000),
        "max_q_time_s": float(t[qi]),
        "max_alpha_deg": float(np.rad2deg(d["aero_alpha"]).max()),
        "initial_mass_kg": float(d["mass"][0]),
        "final_mass_kg": float(d["mass"][-1]),
    }
    if case == "landing":
        kpi_path = ROOT / "results/project2_physics/landing/kpi.json"
        kpi = json.loads(kpi_path.read_text())
        # Cross-check the separately stored contact KPIs against this campaign.
        for key in ["landing_speed_mps", "landing_position_error_m", "landing_tilt_deg"]:
            assert np.isclose(kpi[key], verification["diagnostics"][key], atol=1e-10, rtol=0)
        m["contact_kpi"] = kpi
        h0, h1 = d["altitude_m"][-2:]
        m["contact_time_s"] = float(t[-2] + h0 / (h0 - h1) * (t[-1] - t[-2]))
        source_paths.append(kpi_path)
    else:
        ref = cfg.scenario.reference
        offset = float(ref.time_offset_s)
        mask = (rt >= ref.start_s) & (rt <= ref.end_s)
        mask &= (rt + offset > t[0] + 1e-9) & (rt + offset <= t[-1])
        obs_t = rt[mask]
        comp = {}
        for col, (key, values, truth, scale, label) in enumerate(
            [
                (
                    "altitude_m",
                    d["altitude_m"] + float(ref.altitude_offset_m),
                    rh,
                    1000,
                    "Altitude [km]",
                ),
                ("speed_mps", d["speed_mps"], rv, 1, "Speed [m/s]"),
            ]
        ):
            sim = np.interp(obs_t + offset, t, values)
            err = sim - truth[mask]
            comp[key] = error_statistics(obs_t, err)
            oa[idx, col].plot(obs_t, truth[mask] / scale, ".", ms=1.5, label="Webcast")
            oa[idx, col].plot(obs_t, sim / scale, ".", ms=1.5, label="Simulation")
            oa[idx, col].set_ylabel(title + "\n" + label)
            ra[idx, col].plot(obs_t, err / scale, ".", ms=1.5)
            ra[idx, col].axhline(0, color="0.4", lw=0.7)
            ra[idx, col].set_ylabel(title + "\n" + label.replace(" [", " residual ["))
            if idx == 0:
                oa[idx, col].legend(fontsize=8, markerscale=3)
        comp["time_offset_s"] = offset
        comp["observed_range_s"] = [float(obs_t[0]), float(obs_t[-1])]
        m["flight_comparison"] = comp
        if case == "ascent":
            m["meco_145s"] = {
                "altitude_m": float(np.interp(145, t, d["altitude_m"])),
                "speed_mps": float(np.interp(145, t, d["speed_mps"])),
                "observed_altitude_m": observation(rt, rh, 145)["value"],
                "observed_speed_mps": observation(rt, rv, 145)["value"],
            }
    metrics["cases"][case] = m

format_axes(oa, "Mission time T+ [s]")
format_axes(ra, "Mission time T+ [s]")
format_axes(aa)
format_axes(ma)
save(overlays, "flight_comparison")
save(residuals, "flight_residuals")
save(attitudes, "attitude_rates")
save(massfig, "mass_properties")
metrics["sources"] = {str(p.relative_to(ROOT)): sha(p) for p in source_paths}
(OUT / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
print(json.dumps(metrics["cases"], indent=2))
