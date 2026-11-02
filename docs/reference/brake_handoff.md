# Convex brake-to-landing handoff

With `gnc.scheduler.powered_brake=true`, LCvx and SCvx receive a brake target
whose NED down coordinate is `-gnc.scheduler.brake_end_altitude`. The scheduler
uses that same altitude to transition from the brake phase to final powered descent.
The horizontal target coordinates are inherited from `gnc.guidance.convex_target_ned`.

`gnc.scheduler.brake_terminal_vz_max_mps` sets the positive-down terminal speed
bound for the brake plan: `0 <= v_D <= brake_terminal_vz_max_mps`. It must be finite
and positive; the default remains 180 m/s. This is a planning constraint, not a
speed guard on the phase transition, and does not guarantee a feasible final landing.

The builder copies the guidance configuration for the brake phase. The final
landing retains the original target and terminal-speed settings. Both `altitude`
and `guard` schedulers use this contract. The guidance classes no longer silently
replace constructor arguments with a 2 km / 180 m/s target.

Example using the staged-descent experiment's 2 km handoff:

```bash
python scripts/run_single.py plant/vehicle=falcon9_octaweb scenario=reentry_to_landing \
  gnc.scheduler.powered_brake=true gnc.scheduler.switch_altitude=5000 \
  gnc.scheduler.brake_end_altitude=2000 gnc.scheduler.brake_terminal_vz_max_mps=180 \
  gnc.guidance.convex_replan=true plant/actuator=ideal
```

The default handoff altitude is 800 m. Changing either experiment value is a
configuration change; neither value represents a flight-validated operating limit.

`tests/test_brake_handoff_config.py` checks both convex methods and both schedulers
at 800 m and 2 km, verifies the actual scheduler transition, preserves the landing
configuration, and rejects invalid speed limits. `tests/test_phase_plan_reset.py`
checks that cached brake plans are reset on the transition. These tests verify
configuration and state handling, not full-flight landing success.
