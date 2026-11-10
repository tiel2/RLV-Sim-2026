"""Navigation (N) — estimates the state x_hat the rest of GNC consumes.

Contract: measure(state, key) -> Measurement; estimate(meas, act, memory) -> (x_hat, memory).
Current baseline: identity (truth pass-through); template is a runnable scaffold.
The explicit synthetic_eskf option provides assumed IMU/GNSS/radar models and an aided ESKF.
ekf_stub remains unavailable and raises NotImplementedError when selected.
Select via config (gnc/navigation/<name>.yaml sets law._target_ to a build(ctx) factory); see
docs/2_adding_a_module.md and the navigation section in README.md.
"""
