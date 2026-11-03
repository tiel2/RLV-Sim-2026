"""Verify release-to-boostback dynamics, guards and numerical sensitivity; no energy audit."""

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf, open_dict
from scipy.spatial.transform import Rotation, Slerp

from plant.separation import Separation
from sim.run import run_configured
from validation.ascent import compare
from validation.boostback import replay_window
from validation.separation import event_balances

ROOT = Path(__file__).resolve().parents[1]


def sensitivity(a, b):
    time = np.arange(150.2, min(a.t[-1], b.t[-1]) - 0.2, 0.2)
    errors = {}
    for key in ("r_ned", "v_ned", "w_body", "m"):

        def sample(result, key=key):
            values = getattr(result, key)
            if values.ndim == 1:
                return np.interp(time, result.t, values)[:, None]
            return np.column_stack([np.interp(time, result.t, values[:, i]) for i in range(3)])

        errors[key] = float(np.max(np.linalg.norm(sample(a) - sample(b), axis=1)))
    qa = Slerp(a.t, Rotation.from_quat(a.q_bn, scalar_first=True))(time)
    qb = Slerp(b.t, Rotation.from_quat(b.q_bn, scalar_first=True))(time)
    errors["attitude_rad"] = float(np.max((qa.inv() * qb).magnitude()))
    ta = {e["phase_after"]: e["t"] for e in a.aux["events"] if e["kind"] == "mission_transition"}
    tb = {e["phase_after"]: e["t"] for e in b.aux["events"] if e["kind"] == "mission_transition"}
    errors["transition_time_s"] = (
        max(abs(ta[k] - tb[k]) for k in ta) if ta.keys() == tb.keys() else None
    )
    errors["final_rcs_kg"] = abs(
        float(a.aux["per_tick"][-1]["rcs_fuel"]) - float(b.aux["per_tick"][-1]["rcs_fuel"])
    )
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/boostback_physics")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        cfg = compose(config_name="config", overrides=["scenario=ascent_boostback"])
    with open_dict(cfg):
        cfg.scenario.mission.capture_reference_windows = True
    (args.output / "resolved_config.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    _, actual = run_configured(cfg, record_telemetry=True)
    sep = next(e for e in actual.aux["events"] if e["kind"] == "stage_separation")
    config = Separation(cfg).cfg
    windows = {}
    checks = []
    limits = dict(
        r_ned=1e-4,
        v_ned=1e-5,
        w_body=1e-7,
        q_bn=1e-7,
        m=1e-3,
        rcs_fuel=1e-6,
        throttle=1e-7,
        tvc=1e-7,
    )
    for phase, rows in actual.aux["reference_windows"].items():
        print("Independent DOP853 window, phase", phase, flush=True)
        expected = replay_window(config, rows)
        refined = replay_window(config, rows, rtol=1e-13)
        measured = SimpleNamespace(
            **{
                k: np.array([r["state"][k] for r in rows])
                for k in ("r_ned", "v_ned", "q_bn", "w_body", "m")
            }
        )
        errors = compare(measured, refined)
        refinement = compare(SimpleNamespace(**expected), refined)
        for key, state_key in (
            ("rcs_fuel", "rcs_fuel"),
            ("throttle", "throttle_act"),
            ("tvc", "tvc"),
        ):
            recorded = np.array([np.asarray(r["state"][state_key]).ravel() for r in rows])
            ref = refined[key].reshape(len(rows), -1)
            errors[key] = float(np.max(np.abs(recorded - ref)))
            refinement[key] = float(np.max(np.abs(expected[key] - refined[key])))
        windows[phase] = dict(
            start_s=rows[0]["state"]["t"],
            end_s=rows[-1]["state"]["t"],
            errors=errors,
            refinement_errors=refinement,
        )
        checks.append(all(errors[k] <= v and refinement[k] <= v / 10 for k, v in limits.items()))
    cfg.scenario.mission.capture_reference_windows = False
    cfg.plant_dt = 0.0005
    _, fine = run_configured(cfg, record_telemetry=False)
    pwm_sensitivity = sensitivity(actual, fine)
    cfg.plant.rcs.mode = "averaged"
    cfg.plant_dt = 0.001
    _, averaged = run_configured(cfg, record_telemetry=False)
    cfg.plant_dt = 0.0005
    _, averaged_fine = run_configured(cfg, record_telemetry=False)
    averaged_sensitivity = sensitivity(averaged, averaged_fine)

    def within(values, tolerances):
        return all(values[k] is not None and values[k] <= v for k, v in tolerances.items())

    # Fixed 5% command quantization preserves physical PWM duration on both grids.
    # These are sensitivity limits for a hybrid feedback loop, not an order-of-convergence claim.
    pwm_limits = dict(
        r_ned=0.1,
        v_ned=0.01,
        w_body=1e-5,
        m=0.01,
        attitude_rad=1e-4,
        transition_time_s=0.021,
        final_rcs_kg=0.01,
    )
    averaged_limits = dict(
        r_ned=0.1,
        v_ned=0.01,
        w_body=1e-5,
        m=0.01,
        attitude_rad=1e-4,
        transition_time_s=0.021,
        final_rcs_kg=0.01,
    )
    checks += [within(pwm_sensitivity, pwm_limits), within(averaged_sensitivity, averaged_limits)]
    histories = [actual, fine, averaged, averaged_fine]
    for run in histories:
        records = run.aux["mission_records"]
        checks += [
            run.aux["mission"]["completed"],
            run.aux["mission"]["failure"] is None,
            min(r["main_propellant_kg"] for r in records) >= cfg.scenario.mission.main_reserve_kg,
            min(r["rcs_fuel_kg"] for r in records) >= cfg.scenario.mission.rcs_reserve_kg,
        ]
    # Each native block spans a whole 20 ms PWM period. Count gas from commanded
    # nozzle on-time, independently of the native mass/flow diagnostics.
    act = actual.actuators
    period = float(cfg.plant.rcs.pulse_period)
    np.testing.assert_allclose((act.t_end - act.t_start) / period, 1, atol=1e-8, rtol=0)
    flow = (
        2
        * float(cfg.plant.vehicle.actuator.rcs_thrust)
        / (float(cfg.plant.vehicle.actuator.rcs_isp) * 9.80665)
    )
    expected_gas = float(
        np.sum((act.t_end - act.t_start) * np.abs(act.held.rcs).sum(axis=1) * flow)
    )
    used_gas = float(cfg.scenario.ic.rcs_fuel - actual.aux["per_tick"][-1]["rcs_fuel"])
    gas_residual = abs(used_gas - expected_gas)
    checks.append(gas_residual < 1e-6)
    balances = event_balances(sep)
    checks.append(max(balances.values()) < 1e-4)
    report = dict(
        status="passed" if all(checks) else "failed",
        scope=(
            "release balances; four independent 1 s input-replay windows; closed-loop sensitivity"
        ),
        independent_windows=windows,
        independent_limits=limits,
        release_balances=balances,
        full_run_rcs_budget=dict(
            used_kg=used_gas,
            independent_on_time_kg=expected_gas,
            residual_kg=gas_residual,
            limit_kg=1e-6,
        ),
        pwm_sensitivity=dict(errors=pwm_sensitivity, limits=pwm_limits),
        averaged_sensitivity=dict(errors=averaged_sensitivity, limits=averaged_limits),
        averaged_vs_pwm=sensitivity(actual, averaged),
        mission=actual.aux["mission"],
        transitions=[e for e in actual.aux["events"] if e["kind"] == "mission_transition"],
        limitations=(
            "target not attained; approximate ballistic targeting; "
            "high-altitude atmosphere extrapolation; no energy or landing validation"
        ),
        source_sha256={
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in (
                "src/gnc/guidance/boostback.py",
                "src/sim/loop.py",
                "src/validation/boostback.py",
                "src/validation/vehicle_references.py",
                "scripts/check_boostback_physics.py",
                "src/native/plant.cpp",
                "configs/scenario/ascent_boostback.yaml",
            )
        },
    )
    (args.output / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not all(checks):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
