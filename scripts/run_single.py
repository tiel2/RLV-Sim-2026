"""Single prescribed-input or closed-loop rollout.

Usage (from repo root, venv active):
    python scripts/run_single.py
    python scripts/run_single.py environment/wind=dryden fault=tvc_bias
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import hydra
import numpy as np
from _plotting import plot_episode, plot_physics, plot_plan_tracking
from omegaconf import OmegaConf

from core.seeding import write_manifest
from sim.replay import build_replay_doc, write_replay
from sim.run import run_configured
from sim.telemetry import write_physics, write_telemetry


@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg):
    out_dir = Path.cwd()  # hydra.run.dir = results/<script>/<date>/<time>
    resolved = OmegaConf.to_yaml(cfg, resolve=True)
    (out_dir / "resolved_config.yaml").write_text(resolved)
    write_manifest(out_dir, cfg.seed, resolved, extra={"mode": cfg.run.mode})

    bundle, result = run_configured(cfg, progress=True)

    for kind, path in write_telemetry(result, out_dir).items():
        print(f"{kind}: {path}")

    physics = write_physics(result, out_dir)

    # Trajectory as a flat CSV (one row per logged tick). Vectors split into named columns; the
    # per-engine/gimbal/fin actuator channels widen to scalar columns (count follows the vehicle).
    throttle = np.asarray(result.throttle_act)  # (T, n_engines)
    tvc = np.asarray(result.tvc)  # (T, n_gimbal, 2) -> pitch/yaw
    fin = np.asarray(result.fin)  # (T, n_fins)
    act = {f"throttle{e}": throttle[:, e] for e in range(throttle.shape[1])}
    for g in range(tvc.shape[1]):
        act[f"tvc{g}_pitch"], act[f"tvc{g}_yaw"] = tvc[:, g, 0], tvc[:, g, 1]
    act.update({f"fin{i}": fin[:, i] for i in range(fin.shape[1])})
    cols = {
        "t": result.t,
        "r_N": result.r_ned[:, 0],
        "r_E": result.r_ned[:, 1],
        "r_D": result.r_ned[:, 2],
        "v_N": result.v_ned[:, 0],
        "v_E": result.v_ned[:, 1],
        "v_D": result.v_ned[:, 2],
        "q_w": result.q_bn[:, 0],
        "q_x": result.q_bn[:, 1],
        "q_y": result.q_bn[:, 2],
        "q_z": result.q_bn[:, 3],
        "w_x": result.w_body[:, 0],
        "w_y": result.w_body[:, 1],
        "w_z": result.w_body[:, 2],
        "m": result.m,
        "phase": result.phase,
        **{
            key: physics[key]
            for key in (
                "altitude_m",
                "speed_mps",
                "downrange_m",
                "flight_path_angle_rad",
                "aero_q_dyn",
                "mach",
                "aero_alpha",
                "main_propellant",
                "rcs_fuel",
                "x_cg",
                "fin_deployment",
            )
        },
        **act,
    }
    with (out_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(cols)
        writer.writerows(zip(*cols.values(), strict=True))
    contact = result.landing_class is not None
    kpi = {
        "termination_reason": result.termination_reason,
        "landing_class": result.landing_class,  # soft / hard / crash (3-tier contact quality)
        "touchdown_index": result.touchdown_index,
        "landing_position_error_m": result.landing_position_error if contact else None,
        "landing_speed_mps": result.landing_speed if contact else None,
        "landing_speed_vertical_mps": result.landing_speed_vertical if contact else None,
        "landing_tilt_deg": float(np.rad2deg(result.landing_tilt)) if contact else None,
        "fuel_margin_kg": result.fuel_margin,
        "success": bool(result.success),
        "violations": result.violation_flags,
    }
    (out_dir / "kpi.json").write_text(json.dumps(kpi, indent=2))
    print(json.dumps(kpi, indent=2))

    # Only emit solver/controller statistics when those components actually ran.
    for kind in ("solver", "control"):
        if kind in result.aux:
            (out_dir / f"{kind}_stats.json").write_text(json.dumps(result.aux[kind], indent=2))

    from validation.analytical import validate_episode, write_validation

    report, reference = validate_episode(result, bundle, cfg)
    write_validation(report, reference, out_dir, plot=bool(cfg.run.save_plots))
    print("validation:", report["status"])

    # Per-run 3D replay for the web viewer.
    if cfg.run.save_replay:
        doc = build_replay_doc(result, bundle, cfg)
        path = write_replay(doc, out_dir / "replay.json")
        print(f"replay: {path} ({doc['meta']['n_frames']} frames)")

    if cfg.run.save_plots:
        plot_physics(physics, out_dir / "plots", reference=reference)
        path = plot_episode(result, out_dir / "plots", title=f"single | {cfg.run.run_id}")
        print(f"plot: {path}")
        path = plot_plan_tracking(
            result, out_dir / "plots", title=f"plan vs flown | {cfg.run.run_id}"
        )
        if path is not None:
            print(f"plot: {path}")
    print(f"output: {out_dir}")
    if report["status"] == "failed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
