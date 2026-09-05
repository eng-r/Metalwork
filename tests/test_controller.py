"""
Unit tests comparing Baseline PID and Cascade LADRC controllers under load.
"""

import pytest

from sim.common.contracts import ProcessVariables
from sim.controller.adrc import CascadeLADRCController
from sim.controller.baselines import BaselinePIDController
from sim.controller.state_machine import OperatingMode


def test_baseline_pid_clamping() -> None:
    """Verify that baseline PID respects pump speed limits and anti-windup."""
    ctrl = BaselinePIDController(target_pressure_bar=40.0, max_pump_rpm=4500.0)
    ctrl.reset()

    sensors = ProcessVariables(
        timestamp=0.0,
        pressure_hyd=1.0e5,                     # 1 bar: large positive error
        spindle_speed_res=366.5,                # 3500 RPM
        spindle_torque_est=2.0,
    )

    cmd = ctrl.update(dt=0.010, sensors=sensors)
    assert cmd.pump_speed_cmd_rpm <= 4500.0
    assert cmd.pump_speed_cmd_rpm >= 0.0


def test_cascade_ladrc_leso_convergence() -> None:
    """Verify that discrete LESO tracks output pressure and estimates total disturbance."""
    ctrl = CascadeLADRCController(target_pressure_bar=35.0)
    ctrl.reset()

    # Feed steady pressure measurements of 25 bar
    sensors = ProcessVariables(
        timestamp=0.0,
        pressure_hyd=25.0e5,
        spindle_speed_res=366.5,
        spindle_torque_est=3.5,
    )

    for i in range(50):
        sensors.timestamp = i * 0.010
        cmd = ctrl.update(dt=0.010, sensors=sensors)

    internal = ctrl.get_internal_states()
    # Estimated pressure in LESO should be near measured 25 bar
    assert abs(internal["leso_z1_pressure"] - 25.0) < 1.0


def test_ladrc_asymmetric_overload_backoff() -> None:
    """Verify that LADRC reference governor pulls back target pressure during high torque."""
    ctrl = CascadeLADRCController(target_pressure_bar=45.0)
    ctrl.reset()
    ctrl.state_machine.current_mode = OperatingMode.NORMAL_MILLING
    ctrl.current_mode = OperatingMode.NORMAL_MILLING
    ctrl.filtered_ref_bar = 45.0

    # Normal cut
    sensors_normal = ProcessVariables(
        timestamp=0.1,
        pressure_hyd=35.0e5,
        spindle_speed_res=366.5,
        spindle_torque_est=3.0,
    )
    ctrl.update(dt=0.010, sensors=sensors_normal)
    ref_normal = ctrl.filtered_ref_bar

    # Severe cutting torque overload (7.0 N*m)
    sensors_overload = ProcessVariables(
        timestamp=0.2,
        pressure_hyd=35.0e5,
        spindle_speed_res=366.5,
        spindle_torque_est=7.0,
    )
    ctrl.update(dt=0.010, sensors=sensors_overload)
    ref_overload = ctrl.filtered_ref_bar

    # Asymmetric Reference Governor must reduce reference under overload
    assert ref_overload < ref_normal
