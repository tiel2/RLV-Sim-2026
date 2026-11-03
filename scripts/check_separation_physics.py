"""Verify release balances, independent booster coast and full-run timestep agreement."""

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from plant.separation import Separation
from sim.run import run_configured
from validation.ascent import compare
from validation.separation import coast_reference, event_balances

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/separation_physics")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        cfg = compose(config_name="config", overrides=["scenario=ascent_separation"])
    (args.output / "resolved_config.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    _, result = run_configured(cfg, record_telemetry=False)
    event = next(e for e in result.aux["events"] if e["kind"] == "stage_separation")
    mask = result.t >= event["t"]
    keys = ("r_ned", "v_ned", "q_bn", "w_body", "m")
    actual = SimpleNamespace(**{k: getattr(result, k)[mask] for k in keys})
    retained = Separation(cfg).cfg
    expected = coast_reference(retained, event["after"], result.t[mask])
    refined = coast_reference(retained, event["after"], result.t[mask], rtol=1e-13)
    errors = compare(actual, refined)
    refinement = compare(SimpleNamespace(**expected), refined)
    limits = {"r_ned": 1e-3, "v_ned": 1e-4, "q_bn": 1e-7, "w_body": 1e-8, "m": 1e-6}
    balances = event_balances(event)
    balance_limits = {
        "mass_kg": 1e-7,
        "centroid_m": 1e-8,
        "linear_momentum_kg_m_s": 1e-5,
        "angular_momentum_kg_m2_s": 1e-4,
        "main_propellant_kg": 1e-7,
        "rcs_propellant_kg": 1e-7,
    }
    # The zero-command coast reference excludes residual below-ignition throttle states.
    aux = result.aux["per_tick"]
    coast_thrust = max(
        float(a["thrust_mag"]) for a, selected in zip(aux, mask, strict=True) if selected
    )
    cfg.plant_dt = float(cfg.plant_dt) / 2
    _, finer = run_configured(cfg, record_telemetry=False)
    np.testing.assert_allclose(result.t, finer.t, atol=1e-7, rtol=0)
    timestep_errors = compare(result, {k: getattr(finer, k) for k in keys})
    # Reuse the established ascent mass tolerance for the full powered trajectory;
    # the unpowered reference above has a separate constant-mass tolerance.
    timestep_limits = {**limits, "m": 1e-3}
    passed = (
        all(errors[k] <= v and refinement[k] <= v / 10 for k, v in limits.items())
        and all(balances[k] <= v for k, v in balance_limits.items())
        and all(timestep_errors[k] <= v for k, v in timestep_limits.items())
        and coast_thrust < 1e-9
    )
    report = {
        "status": "passed" if passed else "failed",
        "scope": "0–160 s ascent; impulse-free release at 150 s; retained booster coast",
        "event_balances": {"residuals": balances, "limits": balance_limits},
        "coast_reference": {
            "method": "independent mass quadrature, matrix DOP853",
            "errors": errors,
            "refinement_errors": refinement,
            "limits": limits,
        },
        "timestep": {
            "steps_s": [0.001, 0.0005],
            "errors": timestep_errors,
            "limits": timestep_limits,
        },
        "max_coast_thrust_n": coast_thrust,
        "mass_before_kg": event["before"]["m"],
        "mass_after_kg": event["after"]["m"],
        "remaining_main_propellant_kg": float(result.fuel_margin),
        "limits_of_evidence": (
            "assumed timing/aerodynamics; removed body not propagated; no landing"
        ),
        "source_sha256": {
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in (
                "src/native/plant.cpp",
                "src/plant/separation.py",
                "src/sim/loop.py",
                "src/validation/separation.py",
                "configs/scenario/ascent_separation.yaml",
            )
        },
    }
    (args.output / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
