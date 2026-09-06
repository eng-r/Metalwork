import math

from sim.controller.load_governor import (
    PumpPressureFeedforward,
    TorqueGovernorConfig,
    TorqueToPressureGovernor,
)


def test_tob_setpoint_step_has_immediate_physics_feedforward() -> None:
    gov = TorqueToPressureGovernor(
        TorqueGovernorConfig(
            target_torque_nm=4.0,
            pressure_ceiling_bar=40.0,
        )
    )
    gov.reset(initial_pressure_bar=24.0)
    old_ff = gov.feedforward_pressure_bar

    gov.set_target_torque(6.0)
    gov.update(0.01, measured_torque_nm=4.0)

    assert gov.feedforward_pressure_bar > old_ff + 8.0
    assert gov.reference_bar > 24.0


def test_governor_reports_unachievable_torque() -> None:
    gov = TorqueToPressureGovernor(
        TorqueGovernorConfig(
            target_torque_nm=8.0,
            pressure_ceiling_bar=35.0,
        )
    )
    assert gov.control_limited
    assert gov.achievable_torque_nm < gov.target_torque_nm


def test_pump_head_feedforward_is_monotonic() -> None:
    ff = PumpPressureFeedforward()
    rpms = [ff.rpm_for_pressure(p) for p in (10.0, 20.0, 30.0, 35.0)]
    assert rpms == sorted(rpms)
    assert 2000.0 < ff.rpm_for_pressure(20.0) < 2300.0
    assert 2600.0 < ff.rpm_for_pressure(30.0) < 2700.0
