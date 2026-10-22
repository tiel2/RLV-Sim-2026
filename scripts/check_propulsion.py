"""Fixed-state engine/TVC checks and unloaded servo verification.

python scripts/check_propulsion.py scenario=vacuum_burn plant/thrust=vacuum
Exit 2 means a model-agreement or enforced-constraint check failed; artifacts are still saved.
"""

import csv
import hashlib
import json
from pathlib import Path

import hydra
import numpy as np
from omegaconf import OmegaConf

from validation.propulsion import check_propulsion


def write_results(report, rows, series, configs, out):
    (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (out / "static_cases.json").write_text(json.dumps(rows, indent=2, allow_nan=False) + "\n")
    fields = [
        "variant",
        "group",
        "name",
        "mass_kg",
        "altitude_m",
        "cg_x_m",
        "pressure_pa",
        "passed",
    ]
    with (out / "static_checks.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields + list(report["static_limits"]))
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    **{k: row[k] for k in fields},
                    **{k: row["checks"][k]["error"] for k in report["static_limits"]},
                }
            )
    (out / "settings").mkdir(exist_ok=True)
    for name, cfg in configs.items():
        (out / "settings" / f"{name}.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    (out / "dynamic").mkdir(exist_ok=True)
    for name, data in series.items():
        np.savez_compressed(out / "dynamic" / f"{name}.npz", **data)
    root = Path(__file__).resolve().parents[1]
    paths = [
        "src/validation/propulsion.py",
        "src/validation/references.py",
        "src/plant/dynamics.py",
        "src/native/plant.cpp",
        "scripts/check_propulsion.py",
        "tests/plant/test_propulsion_validation.py",
        "src/core/interfaces.py",
    ]
    hashes = {p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in paths}
    for p in paths:
        target = out / "source" / p
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((root / p).read_bytes())
    (out / "source_sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")


@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg):
    out = Path.cwd()
    (out / "resolved_config.yaml").write_text(OmegaConf.to_yaml(cfg, resolve=True))
    report, rows, series, configs = check_propulsion(cfg)
    write_results(report, rows, series, configs, out)
    print(json.dumps({k: v for k, v in report.items() if k != "dynamic_cases"}, indent=2))
    print(f"output: {out}")
    if report["status"] == "failed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
