"""Coaxial, impulse-free component release with an explicit booster body origin."""

import numpy as np
from omegaconf import OmegaConf, open_dict

from core.frames import body_to_ned
from plant.dynamics import build_plant


class Separation:
    """Build a retained vehicle from the same hardware, then release in the live session.

    The retained body origin is the attachment interface. All hardware coordinates
    translate together; fuel, engine layout, actuator limits and environment are retained.
    Only the declared body aerodynamic configuration replaces its attached-stack counterpart.
    """

    def __init__(self, cfg):
        spec = cfg.scenario.separation
        if set(spec) != {"impulse_model", "aero"} or spec.impulse_model != "none":
            raise ValueError("separation requires explicit impulse_model=none and aero")
        component = cfg.plant.vehicle.get("attached_component")
        if component is None or cfg.plant.vehicle.massprops_model != "dual_tank":
            raise ValueError("separation requires an attached dual_tank vehicle")
        if cfg.plant.slosh.model != "none":
            raise ValueError("separation with slosh is not yet supported")
        self.component = OmegaConf.to_container(component, resolve=True)
        self.origin_x = float(component.aft_x)
        if not -float(cfg.plant.vehicle.geom.length) < self.origin_x < 0:
            raise ValueError("attachment interface must lie inside the stack")
        self.cfg = OmegaConf.create(OmegaConf.to_container(cfg, resolve=True))
        with open_dict(self.cfg):
            v = self.cfg.plant.vehicle
            del v.attached_component
            v.geom.length += self.origin_x
            v.geom.x_eng -= self.origin_x
            v.engines.x_eng -= self.origin_x
            v.mass.m_wet -= float(component.mass)
            v.mass.x_cg_dry -= self.origin_x
            v.mass.x_cg_wet -= self.origin_x
            v.tanks.bottom = [float(x) - self.origin_x for x in v.tanks.bottom]
            v.tanks.rcs_gas_x -= self.origin_x
            self.cfg.plant.rcs.center_x -= self.origin_x
            self.cfg.plant.slosh.x_attach -= self.origin_x
            # Fin hardware is retained; reject an aero override that relocates it.
            if "fins" in spec.aero or "linear" in spec.aero:
                raise ValueError("separation aero overrides must describe body aerodynamics only")
            self.cfg.plant.aero = OmegaConf.merge(self.cfg.plant.aero, spec.aero)
            self.cfg.plant.aero.fins.fin_x -= self.origin_x
        self.bundle = build_plant(self.cfg)

    def apply(self, session):
        before = session.snapshot(full=True)
        before_public, before_aux = session.observe()
        c = self.component
        removed_offset = np.array([c["x_cg"] - before_aux["x_cg"], 0, 0])
        removed = {
            "m": c["mass"],
            "r_ned": before["r_ned"] + body_to_ned(removed_offset, before["q_bn"]),
            "v_ned": before["v_ned"]
            + body_to_ned(np.cross(before["w_body"], removed_offset), before["q_bn"]),
            "q_bn": before["q_bn"],
            "w_body": before["w_body"],
            "inertia_diag": c["inertia"],
        }
        session.separate_axial_component(
            self.bundle.native, c["mass"], c["x_cg"], c["inertia"], self.origin_x
        )
        after = session.snapshot(full=True)
        _, after_aux = session.observe()
        for state, aux in ((before, before_aux), (after, after_aux)):
            state.update({k: aux[k] for k in ("x_cg", "inertia_diag", "main_propellant")})
        serial = lambda row: {k: np.asarray(v).tolist() for k, v in row.items()}
        return {
            "t": float(before["t"]),
            "kind": "stage_separation",
            "trigger": "prescribed_time",
            "impulse_model": "none",
            "configuration_before": 0,
            "configuration_after": 1,
            "body_origin_shift_m": self.origin_x,
            "before": serial(before),
            "before_aux": serial(before_aux),
            "before_effective": serial(
                {k: before_public[k] for k in ("throttle_act", "tvc", "fin")}
            ),
            "after": serial(after),
            "removed": serial(removed),
            "removed_body_propagated": False,
        }
