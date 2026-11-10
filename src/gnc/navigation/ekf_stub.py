"""Legacy EKF config placeholder; use synthetic_eskf for the research ESKF implementation."""

from __future__ import annotations

from core.interfaces import Measurement


class EKFNav:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            "The generic EKF selection is unavailable; select synthetic_eskf or identity"
        )

    def measure(self, state, key) -> Measurement:  # pragma: no cover
        raise NotImplementedError

    def estimate(self, meas, act, memory):  # pragma: no cover
        raise NotImplementedError


def build(ctx):
    """Reject the unavailable ``gnc/navigation=ekf`` selection explicitly."""
    return EKFNav()
