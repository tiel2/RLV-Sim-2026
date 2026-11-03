"""Release balances, material-point continuity and preservation of live native state."""

import numpy as np
import pytest
from conftest import compose_cfg
from scipy.spatial.transform import Rotation

from plant.dynamics import build_plant, ic_from_scenario, initial_state
from plant.separation import Separation
from validation.references import mass_properties
from validation.separation import event_balances
from validation.vehicle_references import _constants


@pytest.mark.parametrize(
    "mass,rates", [(545200, [0, 0, 0]), (165000, [0.1, -0.2, 0.3]), (145200, [-0.3, 0.2, -0.1])]
)
def test_release_conserves_two_body_momentum_and_hardware_position(mass, rates):
    cfg = compose_cfg(["scenario=ascent_separation"])
    cfg.scenario.ic.launch_pad = False
    cfg.scenario.ic.r_ned = [300, -120, -40000]
    cfg.scenario.ic.v_ned = [400, 200, -600]
    cfg.scenario.ic.pitch_deg = 33
    cfg.scenario.ic.roll_deg = 21
    cfg.scenario.ic.m = mass
    cfg.scenario.ic.w_body = rates
    bundle, separation = build_plant(cfg), Separation(cfg)
    state = initial_state(ic_from_scenario(cfg.scenario, 13), bundle.params)
    session = bundle.native.create_session(state)
    session.set_fin_deployment(0)
    event = separation.apply(session)
    before, after = event["before"], event["after"]
    residuals = event_balances(event)
    assert max(residuals.values()) < 1e-6
    for key in set(before) - {"m", "r_ned", "v_ned", "x_cg", "inertia_diag"}:
        np.testing.assert_array_equal(before[key], after[key], err_msg=key)
    assert session.observe()[1]["fin_deployment"] == 0
    # Independent quadrature reconstructs mass properties on both sides.
    for config, row in ((cfg, before), (separation.cfg, after)):
        cg, inertia = mass_properties(_constants(config).plant.vehicle, row["m"], row["rcs_fuel"])
        assert row["x_cg"] == pytest.approx(cg, abs=1e-12)
        np.testing.assert_allclose(row["inertia_diag"], np.diag(inertia), rtol=1e-12)
    rotation = Rotation.from_quat(before["q_bn"], scalar_first=True).as_matrix()
    for point in bundle.params.engines.positions:
        old_arm = point - [before["x_cg"], 0, 0]
        new_arm = point - [separation.origin_x + after["x_cg"], 0, 0]
        np.testing.assert_allclose(
            np.array(before["r_ned"]) + rotation @ old_arm,
            np.array(after["r_ned"]) + rotation @ new_arm,
            atol=1e-10,
        )
        np.testing.assert_allclose(
            np.array(before["v_ned"]) + rotation @ np.cross(rates, old_arm),
            np.array(after["v_ned"]) + rotation @ np.cross(rates, new_arm),
            atol=1e-10,
        )
    # Repeating release fails atomically instead of removing another 120 tonnes.
    with pytest.raises(ValueError, match="inconsistent separation"):
        separation.apply(session)
    np.testing.assert_array_equal(session.snapshot()["r_ned"], after["r_ned"])


def test_separation_rejects_pad_and_unsupported_impulse():
    cfg = compose_cfg(["scenario=ascent_separation"])
    bundle = build_plant(cfg)
    session = bundle.native.create_session(
        initial_state(ic_from_scenario(cfg.scenario, 0), bundle.params)
    )
    with pytest.raises(ValueError, match="airborne"):
        Separation(cfg).apply(session)
    cfg.scenario.separation.impulse_model = "spring"
    with pytest.raises(ValueError, match="impulse_model"):
        Separation(cfg)
