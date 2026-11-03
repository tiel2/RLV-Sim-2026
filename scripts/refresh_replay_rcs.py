"""Add recorded RCS channels to existing replays without rerunning dynamics.

Usage: python scripts/refresh_replay_rcs.py results/ascent_boostback [other_run_dirs ...]
Requires resolved_config.yaml, physics.npz and actuators.npz beside replay.json.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from omegaconf import OmegaConf

from plant.dynamics import build_plant
from sim.replay import rcs_activity, rcs_metadata, write_replay


def refresh(directory):
    directory = Path(directory)
    cfg = OmegaConf.load(directory / "resolved_config.yaml")
    bundle = build_plant(cfg)
    path = directory / "replay.json"
    doc = json.loads(path.read_text())
    metadata = rcs_metadata(bundle)
    doc["meta"]["vehicle"]["rcs"] = metadata
    vehicles = doc["meta"].get("vehicles", [doc["meta"]["vehicle"]])
    vehicles[0]["rcs"] = metadata
    if len(vehicles) > 1:
        from plant.separation import Separation

        vehicles[1]["rcs"] = rcs_metadata(Separation(cfg).bundle)
    with np.load(directory / "physics.npz") as physics:
        with np.load(directory / "actuators.npz") as actuators:
            doc["meta"]["rcs_activity"] = rcs_activity(
                actuators["t_start"],
                actuators["t_end"],
                actuators["held_rcs"],
                metadata,
                physics["t"],
                physics["rcs_fuel"],
            )
        for frame in doc["frames"]:
            i = int(np.argmin(abs(physics["t"] - frame["t"])))
            frame["rcs_on"] = physics["rcs_thruster_on"][i].tolist()
            frame["rcs_fuel"] = float(physics["rcs_fuel"][i])
    doc["meta"]["rcs_visualization"] = "period-averaged nozzle duty; schematic cold-gas exhaust"
    write_replay(doc, path)
    print(f"Updated RCS display channels: {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    for run in parser.parse_args().runs:
        refresh(run)
