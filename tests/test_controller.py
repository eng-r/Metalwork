"""Controller unit tests aligned with the physics-informed rebaseline."""

from sim.common.contracts import ProcessVariables
from sim.controller.adrc import CascadeLADRCController, FirstOrderPressureESO
from sim.controller.baselines import BaselinePIDController
from sim.controller.state_machine import OperatingMode


def test_baseline_pid_clamping() -> None:
    ctrl = BaselinePIDController(target_pressure_bar=40.0, max_pump_rpm=4500.0)
    ctrl.reset()
    ctrl.state_machine.current_mode = OperatingMode.NORMAL_MILLING
    ctrl.current_mode = OperatingMode.NORMAL_MILLING

    sensors = ProcessVariables(
        pressure_hyd=1.0e5,
        spindle_speed_res=366.5,
        spindle_torque_est=2.0,
    )
    cmd = ctrl.update(0.010, sensors)
    assert 0.0 <= cmd.pump_speed_cmd_rpm <= 4500.0


def test_first_order_pressure_eso_converges_on_constant_signal() -> None:
    eso = FirstOrderPressureESO(omega_o=14.0, b0=75.0)
    eso.reset(initial_y=1.0)

    # Zero local correction input, steady 25 bar measurement.
    for _ in range(100):
        eso.update(0.010, y_meas=25.0, v_applied=0.0)

    assert abs(eso.z1 - 25.0) < 0.2


def test_ladrc_normal_operation_does_not_relay_pump_off_at_small_pressure_crossing() -> None:
    ctrl = CascadeLADRCController(target_pressure_bar=40.0, target_torque_nm=4.0)
    ctrl.reset()
    ctrl.state_machine.current_mode = OperatingMode.NORMAL_MILLING
    ctrl.current_mode = OperatingMode.NORMAL_MILLING

    sensors = ProcessVariables(
        pressure_hyd=25.0e5,
        spindle_speed_res=366.5,
        pump_speed_res=250.0,
        spindle_torque_est=4.0,
    )
    cmd = ctrl.update(0.010, sensors)

    # Normal control is continuous around pump-map feedforward; zero pump is reserved for recovery.
    assert cmd.enable_pump
    assert cmd.pump_speed_cmd_rpm >= 0.0


def test_setpoint_update_does_not_rewrite_safety_thresholds() -> None:
    ctrl = CascadeLADRCController(target_torque_nm=4.0)
    overload_before = ctrl.state_machine.overload_torque_thresh
    ctrl.apply_runtime_config(
        target_torque_nm=6.0,
        pressure_ceiling_bar=40.0,
        spindle_rpm_nominal=3500.0,
    )
    assert ctrl.state_machine.overload_torque_thresh == overload_before
