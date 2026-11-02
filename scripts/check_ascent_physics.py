"""Verify the attached ascent using independent equations and recorded-load balances."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf

from sim.run import run_configured
from validation.ascent import balances, compare, reference

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/ascent_physics")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with initialize_config_dir(version_base=None, config_dir=str(ROOT / "configs")):
        cfg = compose(config_name="config", overrides=["scenario=ascent"])
    cfg.scenario.horizon.log_dt = 0.01
    (args.output / "resolved_config.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    _, actual = run_configured(cfg, record_telemetry=False)
    constants = OmegaConf.to_container(cfg, resolve=True)
    print("Independent DOP853 references (two tolerances)...", flush=True)
    expected = reference(constants, actual.t, rtol=1e-12)
    refined = reference(constants, actual.t, rtol=1e-13)
    limits = {"r_ned": 0.01, "v_ned": 1e-3, "w_body": 1e-6, "q_bn": 1e-6, "m": 1e-3}
    errors = compare(actual, refined)
    from types import SimpleNamespace

    refinement = compare(SimpleNamespace(**expected), refined)
    coarse_balance = balances(actual)
    cfg.scenario.horizon.log_dt = 0.005
    _, finer = run_configured(cfg, record_telemetry=False)
    fine_balance = balances(finer)
    # These tolerances account for load quadrature; they are not state-solver tolerances.
    balance_limits = {"mass_kg": 0.1, "linear_momentum_kg_m_s": 100, "angular_momentum_kg_m2_s": 50}
    checks = [errors[k] <= v and refinement[k] <= v / 10 for k, v in limits.items()]
    checks += [
        fine_balance[k] <= v and fine_balance[k] <= coarse_balance[k] * 0.4 + 1e-6
        for k, v in balance_limits.items()
    ]
    report = {
        "status": "passed" if all(checks) else "failed",
        "scope": "0–150 s attached-stack realistic-actuator ascent; no separation or flight data",
        "independent_reference": {
            "method": "DOP853 with rotation matrices and mass quadrature",
            "rtol": [1e-12, 1e-13],
            "errors": errors,
            "refinement_errors": refinement,
            "limits": limits,
        },
        "balance_audit": {
            "log_steps_s": [0.01, 0.005],
            "coarse": coarse_balance,
            "fine": fine_balance,
            "limits": balance_limits,
            "criterion": "fine residual within limit and <= 0.4*coarse + 1e-6",
            "note": "recorded loads, not independent force verification",
        },
        "source_sha256": {
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in [
                "src/native/plant.cpp",
                "src/validation/ascent.py",
                "src/validation/references.py",
                "src/validation/vehicle_references.py",
            ]
        },
    }
    (args.output / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(args.output / "reference.npz", t=actual.t, **refined)
    print(json.dumps(report, indent=2))
    if report["status"] != "passed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
