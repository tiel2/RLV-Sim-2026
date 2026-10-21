"""Build and run a configured episode without prescribing a particular GNC method."""

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from sim.loop import run_simulation
from sim.open_loop import PrescribedInputs


def run_configured(cfg, *, progress=False, record_telemetry=True):
    bundle = build_plant(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, cfg.seed), bundle.params)
    h = cfg.scenario.horizon
    mode = str(cfg.scenario.get("input_mode", "closed_loop"))
    common = dict(
        t_max=float(h.t_max),
        plant_dt=float(cfg.plant_dt),
        log_dt=float(h.log_dt),
        progress=progress,
        record_telemetry=record_telemetry,
    )
    if mode == "prescribed":
        inputs = PrescribedInputs(cfg.scenario.inputs, bundle.params, float(h.t_max))
        result = run_simulation(
            bundle, state, None, None, ctrl_dt=float(h.input_dt), prescribed=inputs, **common
        )
    elif mode == "closed_loop":
        # Prescribed runs have no dependency on constructing GNC or a navigation module.
        from gnc.build import build_gnc, build_nav

        result = run_simulation(
            bundle,
            state,
            build_gnc(cfg, bundle.params),
            build_nav(cfg),
            ctrl_dt=float(h.ctrl_dt),
            nav_dt=h.nav_dt,
            guidance_dt=h.guidance_dt,
            delay_ticks=cfg.gnc.delay_ticks,
            success_pos_tol=cfg.scenario.success.pos_tol,
            success_speed_tol=cfg.scenario.success.speed_tol,
            success_tilt_tol_deg=cfg.scenario.success.tilt_tol_deg,
            **common,
        )
    else:
        raise ValueError(f"unknown input_mode {mode!r}; use prescribed or closed_loop")
    return bundle, result
