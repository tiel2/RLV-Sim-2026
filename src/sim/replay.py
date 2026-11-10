"""Serialise a closed-loop episode (`SimResult`) to a self-contained replay.json for the web viewer.

Used by ``scripts/run_single.py`` — auto-exported per run into the Hydra run dir (results/...).

The plant works in NED (z down); the three.js viewer is Y-up, so the frame change is done ONCE here
and the viewer stays dumb:

    viz = [North, Up, East] = [r_ned[0], -r_ned[2], r_ned[1]]      (proper rotation, det=+1)
    q_bv = q_vn ⊗ q_bn,  q_vn = [√2/2, √2/2, 0, 0]  (NED->viz, +90° about X)

Quaternions are scalar-first Hamilton in the plant; the JSON emits three.js order [x,y,z,w].
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from core.frames import body_to_ned

# NED -> viz (Y-up, [North, Up, East]) is a +90° rotation about X: q_vn (scalar-first Hamilton).
_SQRT_HALF = float(np.sqrt(0.5))
Q_VN = np.array([_SQRT_HALF, _SQRT_HALF, 0.0, 0.0])

PHASE_NAMES = {0: "REENTRY_BURN", 1: "AERO_DESCENT", 2: "POWERED_BRAKE", 3: "POWERED_DESCENT"}


def _qmul(q, p):
    """Hamilton product of scalar-first quaternions q ⊗ p."""
    qw, qx, qy, qz = q
    pw, px, py, pz = p
    return np.array(
        [
            qw * pw - qx * px - qy * py - qz * pz,
            qw * px + qx * pw + qy * pz - qz * py,
            qw * py - qx * pz + qy * pw + qz * px,
            qw * pz + qx * py - qy * px + qz * pw,
        ]
    )


def _ned_to_viz_pos(r_ned):
    return [float(r_ned[0]), float(-r_ned[2]), float(r_ned[1])]


def _q_bn_to_three(q_bn):
    """body->NED (scalar-first) -> body->viz in three.js order [x,y,z,w]."""
    qbv = _qmul(Q_VN, np.asarray(q_bn))
    qbv = qbv / (np.linalg.norm(qbv) + 1e-12)
    return [float(qbv[1]), float(qbv[2]), float(qbv[3]), float(qbv[0])]


def _aux_series(result, key):
    """Pull a per-tick scalar/vector series out of result.aux['per_tick']."""
    return [np.asarray(tick[key]) for tick in result.aux["per_tick"]]


def rcs_metadata(bundle):
    """Body-frame mounts and force directions; exhaust travels opposite to force."""
    r = bundle.params.rcs
    return {
        "positions": np.asarray(r.positions).tolist(),
        "directions": np.asarray(r.directions).tolist(),
        "channels": np.asarray(r.channels).tolist(),
        "thrust_scale": np.asarray(r.thrust_scale).tolist(),
        "pulse_ticks": int(r.pulse_ticks) if bool(r.pulsed) else None,
    }


def rcs_activity(t_start, t_end, held_rcs, metadata, fuel_t, fuel):
    """Compress held nozzle duties independently of downsampled pose frames.

    Display period-averaged PWM activity: 20 ms pulses alias at display/logging rates.
    Commands are held, never interpolated ahead of their epoch. Fuel exhaustion and
    failed nozzles suppress the display. This is visual post-processing only.
    """
    rows = []
    last = None
    for start, _end, command in zip(t_start, t_end, held_rcs, strict=True):
        duty = np.clip(command, -1, 1)
        ticks = metadata["pulse_ticks"]
        if ticks is not None:
            duty = np.sign(duty) * np.floor(np.abs(duty) * ticks + 0.5) / ticks
        nozzle = np.clip(np.asarray(metadata["channels"]) @ duty, 0, 1)
        nozzle *= metadata["thrust_scale"]
        if np.interp(start, fuel_t, fuel) <= 1e-9:
            nozzle[:] = 0
        if last is None or not np.array_equal(last, nozzle):
            rows.append({"t": float(start), "duty": nozzle.tolist()})
            last = nozzle
    if rows:
        # An episode timeout does not itself switch actuators off. Freeze the
        # actual held activity at the endpoint, unless the tank is exhausted.
        terminal = last if np.interp(t_end[-1], fuel_t, fuel) > 1e-9 else np.zeros_like(last)
        rows.append({"t": float(t_end[-1]), "duty": terminal.tolist()})
    return rows


def vehicle_metadata(bundle, cfg):
    """Geometry and hardware for one discrete vehicle configuration."""
    p = bundle.params
    meta = bundle.meta
    gimbal_idx = [int(x) for x in np.asarray(meta["gimbal_idx"]).ravel()]
    vehicle = {
        "length": float(p.geom.length),
        "diameter": float(p.geom.d_ref),
        "x_eng": float(p.geom.x_eng),
        "n_engines": int(meta["n_engines"]),
        "n_gimbal": int(meta["n_gimbal"]),
        "n_fins": int(meta["n_fins"]),
        "gimbal_idx": gimbal_idx,
        # body-frame geometry (x toward nose, negative aft) so the viewer can place plumes/fins
        "engine_pos": [[float(c) for c in row] for row in np.asarray(p.engines.positions)],
        "fin_arm": [[float(c) for c in row] for row in np.asarray(p.aero.fin_arm)],
        "fin_normal": [[float(c) for c in row] for row in np.asarray(p.aero.fin_normal)],
        "x_cg_dry": float(p.mass.x_cg_dry),
        "x_cg_wet": float(p.mass.x_cg_wet),
        "tvc_deflection_max_rad": float(p.actuator.tvc_deflection_max_rad),
        "fin_deflection_max_rad": float(p.actuator.fin_deflection_max_rad),
        "thrust_vac": [float(x) for x in np.atleast_1d(p.engines.thrust_vac)],
        "rcs": rcs_metadata(bundle),
    }
    attached = cfg.plant.vehicle.get("attached_component")
    vehicle["attached_component"] = (
        {"aft_x": float(attached.aft_x), "mass": float(attached.mass)}
        if attached is not None
        else None
    )
    return vehicle


def build_replay_doc(result, bundle, cfg, max_frames: int = 1500) -> dict:
    """Serialise an already-computed closed-loop ``result`` to the viewer's replay document.

    Pure post-processing — it does NOT run the simulation (the caller does that and passes the
    ``result`` from ``sim.loop.run_closed_loop`` plus the ``bundle`` from ``plant.build_plant``).
    """
    n = len(result.t)
    stride = max(1, n // max_frames)
    idx = list(range(0, n, stride))
    # Keep discontinuous configuration/command events even when downsampling.
    for event in result.aux.get("events", []):
        nearest = int(np.argmin(np.abs(result.t - float(event["t"]))))
        idx.extend([max(0, nearest - 1), nearest])
    idx = sorted(set(idx))
    if idx[-1] != n - 1:
        idx.append(n - 1)  # always keep touchdown

    xcg = _aux_series(result, "x_cg")
    thrust_mag = _aux_series(result, "thrust_mag")
    # Public actuator records are the effective endpoint inputs for every actuator backend.
    throttle_eff = np.asarray(result.throttle_act)
    mach = _aux_series(result, "mach")
    altitude = _aux_series(result, "altitude")

    missing_engine_forces = sum(
        "engine_force_body" not in result.aux["per_tick"][i] for i in idx
    )
    if missing_engine_forces:
        import warnings

        warnings.warn(
            f"Replay: engine_force_body missing in {missing_engine_forces} frames; "
            "using zero force vectors for those frames (visualization only).",
            RuntimeWarning,
            stacklevel=2,
        )
    frames = []
    for i in idx:
        frames.append(
            {
                "t": float(result.t[i]),
                "p": _ned_to_viz_pos(result.r_ned[i]),
                "q": _q_bn_to_three(result.q_bn[i]),
                "xcg": float(xcg[i]),
                "thr": [float(x) for x in np.atleast_1d(throttle_eff[i])],
                "gim": [[float(a) for a in row] for row in np.atleast_2d(result.tvc[i])],
                "fin": [float(x) for x in np.atleast_1d(result.fin[i])],
                "thrust": float(thrust_mag[i]),
                "mach": float(mach[i]),
                "alt": float(altitude[i]),
                "mass": float(result.m[i]),
                "v": _ned_to_viz_pos(result.v_ned[i]),
                "speed": float(np.linalg.norm(result.v_ned[i])),
                "downrange": float(np.linalg.norm(result.r_ned[i, :2] - result.r_ned[0, :2])),
                "phase": int(result.phase[i]),
                "configuration": int(result.aux["per_tick"][i].get("configuration", 0)),
                "fin_deployment": float(result.aux["per_tick"][i].get("fin_deployment", 1.0)),
                # Older / scenario-specific plant snapshots may omit engine forces.
                # Keep replay export functional without inventing physical thrust.
                "engine_force_body": np.asarray(
                    result.aux["per_tick"][i].get(
                        "engine_force_body", np.zeros((int(bundle.meta["n_engines"]), 3))
                    )
                ).tolist(),
                "launch_pending": bool(result.aux["per_tick"][i].get("launch_pending", False)),
                "rcs_on": np.asarray(result.aux["per_tick"][i].get("rcs_thruster_on", [])).tolist(),
                "rcs_fuel": float(result.aux["per_tick"][i].get("rcs_fuel", 0)),
            }
        )

    # viz leg-deploy time: the first instant the booster descends below the configured altitude
    # (else null means no deployment in this episode). cfg.run.leg_deploy_alt_m is
    # CLI-overridable (e.g. run.leg_deploy_alt_m=40); a higher altitude deploys the legs earlier.
    alt_full = np.array([float(a) for a in altitude])
    deploy_alt = float(cfg.run.leg_deploy_alt_m)
    below = np.flatnonzero(alt_full < deploy_alt)
    leg_deploy_t = float(result.t[int(below[0])]) if below.size else None

    vehicle = vehicle_metadata(bundle, cfg)
    vehicles = [vehicle]
    if cfg.scenario.get("separation") is not None:
        from plant.separation import Separation

        separation = Separation(cfg)
        vehicles.append(vehicle_metadata(separation.bundle, separation.cfg))
    p = bundle.params
    contact = result.landing_class is not None
    # Same-time left/right limits prevent interpolation through a separation jump.
    for event in result.aux.get("events", []):
        if event["kind"] != "stage_separation":
            continue
        before, aux = event["before"], event["before_aux"]
        effective = event["before_effective"]
        post = next(f for f in frames if abs(f["t"] - event["t"]) < 1e-8)
        pre = dict(post)
        for key, field in (("p", "r_ned"), ("v", "v_ned")):
            pre[key] = _ned_to_viz_pos(before[field])
        pre.update(
            q=_q_bn_to_three(before["q_bn"]),
            xcg=before["x_cg"],
            mass=before["m"],
            alt=aux["altitude"],
            mach=aux["mach"],
            speed=float(np.linalg.norm(before["v_ned"])),
            downrange=float(np.linalg.norm(np.array(before["r_ned"][:2]) - result.r_ned[0, :2])),
            configuration=event["configuration_before"],
            phase=frames[max(0, frames.index(post) - 1)]["phase"],
            thr=effective["throttle_act"],
            gim=effective["tvc"],
            fin=effective["fin"],
            thrust=aux["thrust_mag"],
            engine_force_body=aux.get(
                "engine_force_body", np.zeros((int(bundle.meta["n_engines"]), 3)).tolist()
            ),
            fin_deployment=aux["fin_deployment"],
            rcs_on=aux.get("rcs_thruster_on", pre["rcs_on"]),
            rcs_fuel=before["rcs_fuel"],
            sample_side="before_event",
        )
        frames.insert(frames.index(post), pre)
    is_launch = bool(cfg.scenario.ic.get("launch_pad", False))
    anchor = 0 if is_launch else -1
    anchor_offset = body_to_ned(
        np.array([float(p.geom.x_eng) - float(xcg[anchor]), 0, 0]), result.q_bn[anchor]
    )
    ground_y = (
        float(-anchor_offset[2])
        if (is_launch or contact)
        else -abs(float(p.geom.x_eng) - float(p.mass.x_cg_dry))
    )
    scene = {
        "kind": "land" if is_launch else "ocean",
        "launch_position": frames[0]["p"] if is_launch else None,
        "ground_y": ground_y,
        "alignment": "visual engine-plane offset; physical contact uses point altitude zero",
    }
    kpi = {
        "termination_reason": int(result.termination_reason),
        "landing_position_error_m": float(result.landing_position_error) if contact else None,
        "landing_speed_mps": float(result.landing_speed) if contact else None,
        "landing_tilt_deg": float(np.rad2deg(result.landing_tilt)) if contact else None,
        "fuel_margin_kg": float(result.fuel_margin),
        "success": bool(result.success),
    }
    return {
        "meta": {
            "earth": result.aux.get("earth"),
            "frame": "viz=[North,Up,East]; q=[x,y,z,w] body->viz",
            "scenario": str(cfg.scenario.name),
            "guidance": (
                result.aux.get("input_mode")
                if result.aux.get("input_mode") in ("prescribed", "mission")
                else str(cfg.gnc.guidance.powered_descent_method)
            ),
            "input_mode": result.aux.get("input_mode", "closed_loop"),
            "configuration_events": len(vehicles) > 1
            or any(row["fin_deployment"] == 0 for row in frames),
            "events": result.aux.get("events", []),
            "mission": result.aux.get("mission"),
            "rcs_visualization": "period-averaged nozzle duty; schematic cold-gas exhaust",
            "rcs_activity": (
                rcs_activity(
                    result.actuators.t_start,
                    result.actuators.t_end,
                    result.actuators.held.rcs,
                    vehicle["rcs"],
                    result.t,
                    _aux_series(result, "rcs_fuel"),
                )
                if result.actuators is not None
                else []
            ),
            "run_id": str(cfg.run.run_id),
            "dt_nominal": result.aux.get("schedule", {})
            .get("period_s", {})
            .get("logging", float(np.median(np.diff(result.t))) if n > 1 else 0.0)
            * stride,
            "n_frames": len(frames),
            "missing_engine_force_samples": missing_engine_forces,
            "time_origin_s": float(result.t[0]),
            "flight_reference": (
                {
                    key: cfg.scenario.reference.get(key)
                    for key in (
                        "enabled",
                        "dataset",
                        "segment",
                        "start_s",
                        "end_s",
                        "time_offset_s",
                        "role",
                    )
                }
                if cfg.scenario.get("reference") is not None
                else None
            ),
            "duration_s": float(result.t[idx[-1]] - result.t[0]),
            "leg_deploy": {
                "t": leg_deploy_t,
                "visual_only": True,
                "enabled": result.aux.get("input_mode") not in ("prescribed", "mission"),
            },
            "phase_names": result.aux.get("phase_names", PHASE_NAMES),
            "vehicle": vehicle,
            "vehicles": vehicles,
            "scene": scene,
            "kpi": kpi,
            "credits": "Procedural viewer (no external asset); glTF hook in viz/README.md.",
        },
        "frames": frames,
    }


def write_replay(doc: dict, path) -> Path:
    """Write the replay document as compact JSON, creating parent dirs. Returns the path."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, separators=(",", ":")))
    return path
