import math

from sim.controller.load_governor import (
    TorqueGovernorConfig,
    TorqueToPressureGovernor,
)
from sim.plant.motors import PMSMSpindleDrive, PMSMPumpDrive


def test_live_tob_setpoint_changes_governor_without_reset() -> None:
    gov = TorqueToPressureGovernor(
        TorqueGovernorConfig(target_torque_nm=4.0, pressure_ceiling_bar=40.0)
    )
    gov.reset(initial_pressure_bar=20.0)

    # Sit near the old target.
    for _ in range(100):
        gov.update(0.01, measured_torque_nm=4.0)
    before = gov.reference_bar

    # A larger desired ToB must make pressure reference rise from its CURRENT
    # value rather than reset to an approach pressure.
    gov.set_target_torque(5.5)
    for _ in range(50):
        gov.update(0.01, measured_torque_nm=4.0)

    assert gov.reference_bar > before


def test_spindle_iq_observer_does_not_call_spinup_torque_tob() -> None:
    drive = PMSMSpindleDrive()
    cmd = 3500.0 * math.pi / 30.0

    peak_after_initial_transient = 0.0
    for i in range(1200):
        _, _, tob_est = drive.step(0.001, cmd, load_torque=0.0)
        if i > 250:
            peak_after_initial_transient = max(
                peak_after_initial_transient,
                tob_est,
            )

    # No cutting load: estimated ToB should remain close to zero after spin-up.
    assert peak_after_initial_transient < 0.8


def test_pump_speed_response_is_monotonic_and_smooth() -> None:
    drive = PMSMPumpDrive()
    cmd = 3000.0 * math.pi / 30.0
    speeds = [drive.step(0.001, cmd) for _ in range(500)]

    assert all(b >= a for a, b in zip(speeds, speeds[1:]))
    # It should have made substantial progress toward command without a
    # controller-generated triangular reset.
    assert speeds[-1] > 0.75 * cmd
