"""Check broader display-tolerance assumptions without changing the chosen landing."""

import json
from pathlib import Path

import numpy as np
from omegaconf import OmegaConf

from core.interfaces import Phase
from gnc.build import build_gnc
from plant.dynamics import build_plant
from sim.run import run_configured
from validation.flight_reconstruction import landing_state

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/verification/public_flight_comparison/reconstruction_20261107"


def main():
    cfg = OmegaConf.load(OUT / "final/landing/resolved_config.yaml")
    rows = []
    for envelope in json.loads((OUT / "direction_envelope.json").read_text()):
        if envelope["fraction_interval"] is None:
            rows.append(
                dict(tolerance_m=envelope["tolerance_m"], status="no_direction_under_assumptions")
            )
            continue
        for fraction in [envelope["fraction_interval"][0], envelope["nominal_fraction"], 1.0]:
            state = landing_state(
                fraction, envelope["speed0_mps"], 300, float(cfg.environment.gravity.radius)
            )
            cfg.scenario.ic.r_ned, cfg.scenario.ic.v_ned = state["r_ned"], state["v_ned"]
            bundle = build_plant(cfg)
            gnc = build_gnc(cfg, bundle.params)
            law = gnc.guidance.phase_guidance[int(Phase.POWERED_DESCENT)]._law
            plan = law._plan(
                np.array(state["r_ned"]), np.array(state["v_ned"]), float(cfg.scenario.ic.m)
            )
            row = dict(
                tolerance_m=envelope["tolerance_m"], **state, initial_plan_feasible=plan is not None
            )
            if plan is not None:
                _, result = run_configured(cfg, record_telemetry=False)
                row.update(
                    landing_class=result.landing_class,
                    success=bool(result.success),
                    speed_mps=float(result.landing_speed),
                    position_error_m=float(result.landing_position_error),
                    duration_s=float(result.t[-1]),
                    fuel_kg=float(result.fuel_margin),
                )
            rows.append(row)
            print(row, flush=True)
    (OUT / "display_tolerance_sensitivity.json").write_text(json.dumps(rows, indent=2) + "\n")


if __name__ == "__main__":
    main()
