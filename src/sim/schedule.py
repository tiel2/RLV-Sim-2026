"""Exact multi-rate schedules on the native plant's fixed integration grid."""

from __future__ import annotations

from dataclasses import dataclass
from math import gcd, isclose, isfinite


@dataclass(frozen=True)
class TickSchedule:
    plant_dt: float
    horizon: int
    control: int
    navigation: int
    guidance: int
    logging: int

    @property
    def block(self) -> int:
        return gcd(self.control, self.navigation, self.guidance, self.logging)

    def metadata(self) -> dict:
        periods = {
            name: getattr(self, name) for name in ("control", "navigation", "guidance", "logging")
        }
        return {
            "plant_dt_s": self.plant_dt,
            "horizon_ticks": self.horizon,
            "period_ticks": periods,
            "period_s": {key: value * self.plant_dt for key, value in periods.items()},
            "block_ticks": self.block,
            "block_dt_s": self.block * self.plant_dt,
        }


def build_schedule(plant_dt, t_max, ctrl_dt, nav_dt=None, guidance_dt=None, log_dt=None):
    """Reject off-grid periods instead of silently changing the requested frequencies."""
    dt = float(plant_dt)
    if not isfinite(dt) or dt <= 0:
        raise ValueError("plant_dt must be finite and positive")

    def ticks(value, name):
        value = float(value)
        if not isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        ratio = value / dt
        count = round(ratio)
        if count < 1 or not isclose(ratio, count, rel_tol=0.0, abs_tol=1e-8):
            raise ValueError(f"{name}={value} must be an integer multiple of plant_dt={dt}")
        return count

    return TickSchedule(
        dt,
        ticks(t_max, "t_max"),
        ticks(ctrl_dt, "ctrl_dt"),
        ticks(ctrl_dt if nav_dt is None else nav_dt, "nav_dt"),
        ticks(ctrl_dt if guidance_dt is None else guidance_dt, "guidance_dt"),
        ticks(ctrl_dt if log_dt is None else log_dt, "log_dt"),
    )
