"""
Baseline cascaded controller.

The operator sets desired Torque-on-Bit (ToB). A slow outer ToB governor creates
the pressure reference; the baseline inner loop is conventional PID. This gives
a fairer PID-vs-LADRC comparison because both controllers pursue the same load
objective through the same supervisory architecture.
"""

from typing import Any, Dict

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController
from sim.controller.load_governor import TorqueGovernorConfig, TorqueToPressureGovernor
from sim.controller.soft_sensor import SoftWOBObserver
from sim.controller.state_machine import OperatingMode, SupervisoryStateMachine


class BaselinePIDController(IController):
    """Torque-governed hydraulic milling controller with PID pressure inner loop."""

    def __init__(
        self,
        target_pressure_bar: float = 35.0,
        target_torque_nm: float = 4.0,
        spindle_rpm_nominal: float = 3500.0,
        kp: float = 85.0,
        ki: float = 40.0,
        kd: float = 12.0,
        max_pump_rpm: float = 4500.0,
        positive_slew_rpm_per_s: float = 1200.0,
        negative_slew_rpm_per_s: float = 4000.0,
    ) -> None:
        self.target_pressure_bar = target_pressure_bar
        self.target_torque_nm = target_torque_nm
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_pump_rpm = max_pump_rpm
        self.pos_slew = positive_slew_rpm_per_s
        self.neg_slew = negative_slew_rpm_per_s

        overload = max(target_torque_nm * 1.55, target_torque_nm + 1.8)
        safe = max(target_torque_nm * 1.12, target_torque_nm + 0.5)
        self.state_machine = SupervisoryStateMachine(
            overload_torque_threshold=overload,
            safe_torque_threshold=safe,
        )
        self.soft_wob = SoftWOBObserver()
        self.load_governor = TorqueToPressureGovernor(
            TorqueGovernorConfig(
                target_torque_nm=target_torque_nm,
                pressure_ceiling_bar=target_pressure_bar,
            )
        )

        self.integrator = 0.0
        self.prev_error = 0.0
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def reset(self) -> None:
        self.state_machine.reset(nominal_spindle_rpm=self.spindle_rpm_nominal)
        self.soft_wob.reset()
        self.load_governor.reset(initial_pressure_bar=18.0)
        self.integrator = 0.0
        self.prev_error = 0.0
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def update(self, dt: float, sensors: ProcessVariables) -> ControlCommands:
        wob_est = self.soft_wob.update(dt, sensors)
        sensors.wob_soft_sensor = wob_est

        self.current_mode = self.state_machine.update(
            dt=dt,
            pressure_bar=sensors.pressure_bar,
            spindle_rpm=sensors.spindle_rpm,
            torque_est_nm=sensors.spindle_torque_est,
            wob_est_n=wob_est,
        )

        if self.current_mode == OperatingMode.APPROACH:
            pump_target = 1800.0
            spindle_target = self.spindle_rpm_nominal
            self.integrator = 0.0

        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            pump_target = 800.0
            spindle_target = self.spindle_rpm_nominal
            self.integrator = 0.0

        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            pressure_ref = self.load_governor.update(
                dt=dt,
                measured_torque_nm=sensors.spindle_torque_est,
            )
            error = pressure_ref - sensors.pressure_bar
            p_term = self.kp * error

            d_err = (error - self.prev_error) / dt
            d_term = self.kd * d_err
            self.prev_error = error

            if 0.0 < self.last_pump_cmd_rpm < self.max_pump_rpm:
                self.integrator += error * dt
            self.integrator = max(-80.0, min(80.0, self.integrator))
            i_term = self.ki * self.integrator

            raw_cmd = p_term + i_term + d_term
            pump_target = max(0.0, min(self.max_pump_rpm, raw_cmd))
            spindle_target = self.spindle_rpm_nominal

        elif self.current_mode in (
            OperatingMode.PRESSURE_RELAXATION,
            OperatingMode.OVERLOAD_RECOVERY,
            OperatingMode.STALL_RECOVERY,
        ):
            pump_target = 0.0
            # Keep the cutter rotating during hydraulic unload. Otherwise the
            # baseline is unfairly prevented from clearing material.
            spindle_target = self.spindle_rpm_nominal
            self.integrator = max(-50.0, min(50.0, self.integrator))

        else:
            pump_target = 0.0
            spindle_target = self.spindle_rpm_nominal

        delta_max_up = self.pos_slew * dt
        delta_max_down = self.neg_slew * dt
        delta = pump_target - self.last_pump_cmd_rpm
        if delta > delta_max_up:
            pump_cmd = self.last_pump_cmd_rpm + delta_max_up
        elif delta < -delta_max_down:
            pump_cmd = self.last_pump_cmd_rpm - delta_max_down
        else:
            pump_cmd = pump_target

        pump_cmd = max(0.0, min(self.max_pump_rpm, pump_cmd))
        self.last_pump_cmd_rpm = pump_cmd

        return ControlCommands(
            timestamp=sensors.timestamp,
            pump_speed_cmd_rpm=pump_cmd,
            spindle_speed_cmd_rpm=spindle_target,
            enable_pump=self.current_mode
            not in (
                OperatingMode.STALL_RECOVERY,
                OperatingMode.PRESSURE_RELAXATION,
                OperatingMode.OVERLOAD_RECOVERY,
            ),
            enable_spindle=True,
        )

    def get_operating_mode(self) -> str:
        return self.current_mode.value

    def get_internal_states(self) -> Dict[str, Any]:
        return {
            "controller_type": "BaselinePID",
            "operating_mode": self.current_mode.value,
            "integrator": self.integrator,
            "target_torque_nm": self.target_torque_nm,
            "torque_error_nm": self.load_governor.last_error_nm,
            "torque_pressure_reference_bar": self.load_governor.reference_bar,
            "pressure_ceiling_bar": self.target_pressure_bar,
        }
