from sim.controller.load_governor import (
    TorqueGovernorConfig,
    TorqueToPressureGovernor,
)


def test_torque_governor_raises_pressure_when_torque_is_low() -> None:
    gov = TorqueToPressureGovernor(
        TorqueGovernorConfig(target_torque_nm=4.0, pressure_ceiling_bar=40.0)
    )
    gov.reset(initial_pressure_bar=20.0)
    p0 = gov.reference_bar
    for _ in range(50):
        gov.update(0.01, measured_torque_nm=1.5)
    assert gov.reference_bar > p0


def test_torque_governor_unloads_fast_on_torque_spike() -> None:
    gov = TorqueToPressureGovernor(
        TorqueGovernorConfig(target_torque_nm=4.0, pressure_ceiling_bar=40.0)
    )
    gov.reset(initial_pressure_bar=32.0)
    p0 = gov.reference_bar
    for _ in range(20):
        gov.update(0.01, measured_torque_nm=7.0)
    assert gov.reference_bar < p0
