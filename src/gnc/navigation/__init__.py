"""Navigation (N) — estimates the state x_hat the rest of GNC consumes.

Contract: measure(state, key) -> Measurement; estimate(meas, act, memory) -> (x_hat, memory).
Current baseline: identity (truth pass-through); template is a runnable scaffold.
TODO: physical sensor models, INS propagation, and aided state estimation.
ekf_stub is unavailable and raises NotImplementedError when selected.
Select via config (gnc/navigation/<name>.yaml sets law._target_ to a build(ctx) factory); see
docs/2_adding_a_module.md and the navigation TODO list in README.md.
"""
