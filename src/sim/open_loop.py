"""Open-loop actuator schedule for plant smoke tests."""

from __future__ import annotations

import numpy as np

from core.interfaces import ActuatorCommand


class PrescribedInputs:
    """Piecewise-constant commands on the plant grid, relative to the initial epoch.

    Rows are ordered, start at zero, and hold omitted channels from the preceding row.
    Angles are in radians. A deployment event changes aerodynamic fin exposure only;
    it does not represent hinge dynamics, an impulse, or a change in inertia.
    """

    def __init__(self, rows, params, duration):
        self.dt = float(params.plant_dt)
        self.rows = {}
        shapes = {
            "throttle": (len(params.engines.positions),),
            "gimbal": (len(params.engines.tvc_bias), 2),
            "fin": (len(params.aero.fin_arm),),
            "rcs": (3,),
        }
        limits = {
            "throttle": (0.0, 1.0),
            "gimbal": (
                -float(params.actuator.tvc_deflection_max_rad),
                float(params.actuator.tvc_deflection_max_rad),
            ),
            "fin": (
                -float(params.actuator.fin_deflection_max_rad),
                float(params.actuator.fin_deflection_max_rad),
            ),
            "rcs": (-1.0, 1.0),
        }
        current = {key: np.zeros(shape) for key, shape in shapes.items()}
        deployment = 1.0
        previous = -1
        for phase, row in enumerate(rows):
            unknown = set(row) - {*shapes, "t", "label", "fin_deployment"}
            if unknown:
                raise ValueError(f"unknown prescribed input fields: {sorted(unknown)}")
            t = float(row["t"])
            if not np.isfinite(t) or not 0 <= t < duration:
                raise ValueError("input times must be finite and within [0, t_max)")
            tick = round(t / self.dt)
            if not np.isclose(t / self.dt, tick, atol=1e-8, rtol=0):
                raise ValueError("input times must be integer multiples of plant_dt")
            if tick <= previous:
                raise ValueError("input times must be strictly increasing")
            previous = tick
            for key, shape in shapes.items():
                if key in row:
                    value = np.asarray(row[key], dtype=float)
                    if value.ndim == 0:
                        value = np.full(shape, value)
                    lo, hi = limits[key]
                    if (
                        value.shape != shape
                        or not np.all(np.isfinite(value))
                        or np.any(value < lo)
                        or np.any(value > hi)
                    ):
                        raise ValueError(
                            f"{key} must have shape {shape} and values in [{lo}, {hi}]"
                        )
                    current[key] = value.copy()
            deployment = float(row.get("fin_deployment", deployment))
            if not np.isfinite(deployment) or deployment not in (0.0, 1.0):
                raise ValueError("fin_deployment must be 0 (stowed) or 1 (deployed)")
            if deployment == 0 and float(params.aero.fin_incremental) > 0.5:
                raise ValueError("fin stow requires a body aero table excluding neutral fins")
            self.rows[tick] = {
                "command": ActuatorCommand(**{k: v.copy() for k, v in current.items()}),
                "phase": phase,
                "label": str(row.get("label", f"segment_{phase}")),
                "fin_deployment": deployment,
            }
        if not self.rows or next(iter(self.rows)) != 0:
            raise ValueError("prescribed inputs require a first row at t=0")

    @property
    def phase_names(self):
        return {row["phase"]: row["label"] for row in self.rows.values()}


def open_loop_schedule(
    n_steps: int, plant_dt: float, n_fins: int = 4, n_engines: int = 1, n_gimbal: int = 1
) -> ActuatorCommand:
    """Stacked ActuatorCommand (leading axis n_steps) exercising all actuator channels.

    Per-engine shapes: throttle (n_steps, N), gimbal (n_steps, Ng, 2).
    """
    t = np.arange(n_steps) * plant_dt
    throttle = np.clip(0.6 + 0.2 * np.sin(0.5 * t), 0.0, 1.0)[:, None] * np.ones((1, n_engines))
    gimbal_one = np.stack([0.05 * np.sin(2.0 * t), 0.03 * np.cos(2.0 * t)], axis=1)
    gimbal = gimbal_one[:, None, :] * np.ones((1, n_gimbal, 1))
    fin = 0.1 * np.sin(1.0 * t)[:, None] * np.ones((1, n_fins))
    rcs = np.stack([0.2 * np.sign(np.sin(0.3 * t)), np.zeros(n_steps), np.zeros(n_steps)], axis=1)
    return ActuatorCommand(throttle=throttle, gimbal=gimbal, fin=fin, rcs=rcs)
