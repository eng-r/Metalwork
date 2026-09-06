"""
Fair baseline controller for comparison with LADRC.

Both controller packages share:
- the same ToB -> pressure governor;
- the same centrifugal-pump pressure feedforward;
- the same safety supervisor.

The only intended difference in NORMAL_MILLING is the pressure feedback law: conventional PID
here versus ESO/LADRC in sim/controller/adrc.py.
"""

from typing import Any, Dict

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController
from sim.controller.load_governor import (
    PumpFeedforwardConfig,
    PumpPressureFeedforward,
    TorqueGovernorConfig,
    TorqueToPressureGovernor,
)
from sim.controller.soft_sensor import SoftWOBObserver
from sim.controller.state_machine import OperatingMode, SupervisoryStateMachine


class BaselinePIDController(IController):
    def __init__(
        self,
        target_pressure_bar: float = 35.0,
        target_torque_nm: float = 4.0,
        spindle_rpm_nominal: float = 3500.0,
        kp_rpm_per_bar: float = 65.0,
        ki_rpm_per_bar_s: float = 35.0,
        kd_rpm_per_bar_s: float = 2.0,
        max_pump_rpm: float = 4500.0,
        # Legacy arguments accepted so older callers/tests do not break.
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        positive_slew_rpm_per_s: float | None = None,
        negative_slew_rpm_per_s: float | None = None,
    ) -> None:
        self.target_pressure_bar = target_pressure_bar
        self.target_torque_nm = target_torque_nm
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.max_pump_rpm = max_pump_rpm

        # If legacy gains are explicitly provided, treat them as rpm/bar family gains.
        self.kp = kp_rpm_per_bar if kp is None else kp
        self.ki = ki_rpm_per_bar_s if ki is None else ki
        self.kd = kd_rpm_per_bar_s if kd is None else kd

        self.state_machine = SupervisoryStateMachine()
        self.soft_wob = SoftWOBObserver()
        self.load_governor = TorqueToPressureGovernor(
            TorqueGovernorConfig(
                target_torque_nm=target_torque_nm,
                pressure_ceiling_bar=target_pressure_bar,
            )
        )
        self.pump_ff = PumpPressureFeedforward(
            PumpFeedforwardConfig(max_pump_rpm=max_pump_rpm)
        )

        self.integral_rpm = 0.0
        self.prev_pressure_bar = 1.01325
        self.pressure_rate_bar_s = 0.0
        self.pressure_reference_bar = 12.0
        self.pump_feedforward_rpm = 0.0
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def apply_runtime_config(
        self,
        target_torque_nm: float,
        pressure_ceiling_bar: float,
        spindle_rpm_nominal: float,
    ) -> None:
        self.target_torque_nm = max(0.1, float(target_torque_nm))
        self.target_pressure_bar = max(8.5, float(pressure_ceiling_bar))
        self.spindle_rpm_nominal = max(100.0, float(spindle_rpm_nominal))
        self.load_governor.set_target_torque(self.target_torque_nm)
        self.load_governor.set_pressure_ceiling(self.target_pressure_bar)
        self.state_machine.nominal_spindle_rpm = self.spindle_rpm_nominal

    def reset(self) -> None:
        self.state_machine.reset(nominal_spindle_rpm=self.spindle_rpm_nominal)
        self.soft_wob.reset()
        self.load_governor.reset(initial_pressure_bar=12.0)
        self.integral_rpm = 0.0
        self.prev_pressure_bar = 1.01325
        self.pressure_rate_bar_s = 0.0
        self.pressure_reference_bar = 12.0
        self.pump_feedforward_rpm = self.pump_ff.rpm_for_pressure(12.0)
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def _pressure_reference(
        self,
        dt: float,
        sensors: ProcessVariables,
    ) -> float:
        if self.current_mode == OperatingMode.APPROACH:
            return 12.0
        if self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            return 18.0
        if self.current_mode == OperatingMode.NORMAL_MILLING:
            return self.load_governor.update(
                dt,
                sensors.spindle_torque_est,
            )
        return 0.0

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
        recovery_mode = self.current_mode in (
            OperatingMode.PRESSURE_RELAXATION,
            OperatingMode.OVERLOAD_RECOVERY,
            OperatingMode.STALL_RECOVERY,
        )

        self.pressure_reference_bar = self._pressure_reference(dt, sensors)

        if recovery_mode:
            pump_cmd = 0.0
            self.pump_feedforward_rpm = 0.0
            # Track out of saturation instead of accumulating hidden integral debt.
            self.integral_rpm *= max(0.0, 1.0 - 3.0 * dt)
        else:
            self.pump_feedforward_rpm = self.pump_ff.rpm_for_pressure(
                self.pressure_reference_bar
            )
            error = self.pressure_reference_bar - sensors.pressure_bar

            raw_rate = (sensors.pressure_bar - self.prev_pressure_bar) / max(dt, 1.0e-9)
            alpha_rate = dt / (0.05 + dt)
            self.pressure_rate_bar_s += alpha_rate * (
                raw_rate - self.pressure_rate_bar_s
            )
            self.prev_pressure_bar = sensors.pressure_bar

            p_term = self.kp * error
            d_term = -self.kd * self.pressure_rate_bar_s
            proposed_integral = self.integral_rpm + self.ki * error * dt
            proposed_integral = max(-1200.0, min(1200.0, proposed_integral))

            raw_cmd = self.pump_feedforward_rpm + p_term + proposed_integral + d_term
            pump_cmd = max(0.0, min(self.max_pump_rpm, raw_cmd))

            # Conditional integration anti-windup.
            drives_high = raw_cmd > self.max_pump_rpm and error > 0.0
            drives_low = raw_cmd < 0.0 and error < 0.0
            if not (drives_high or drives_low):
                self.integral_rpm = proposed_integral

        self.last_pump_cmd_rpm = pump_cmd
        return ControlCommands(
            timestamp=sensors.timestamp,
            pump_speed_cmd_rpm=pump_cmd,
            spindle_speed_cmd_rpm=self.spindle_rpm_nominal,
            enable_pump=not recovery_mode,
            enable_spindle=True,
        )

    def get_operating_mode(self) -> str:
        return self.current_mode.value

    def get_internal_states(self) -> Dict[str, Any]:
        return {
            "controller_type": "BaselinePID",
            "operating_mode": self.current_mode.value,
            "target_torque_nm": self.target_torque_nm,
            "target_wob_n": self.load_governor.target_wob_n,
            "torque_error_nm": self.load_governor.last_error_nm,
            "filtered_torque_nm": self.load_governor.filtered_torque_nm,
            "torque_pressure_reference_bar": self.load_governor.reference_bar,
            "pressure_feedforward_bar": self.load_governor.feedforward_pressure_bar,
            "filtered_reference_bar": self.pressure_reference_bar,
            "pressure_ceiling_bar": self.target_pressure_bar,
            "pump_feedforward_rpm": self.pump_feedforward_rpm,
            "integrator_rpm": self.integral_rpm,
            "pressure_rate_bar_s": self.pressure_rate_bar_s,
            "control_limited": self.load_governor.control_limited,
            "limit_reason": self.load_governor.limit_reason,
            "achievable_torque_nm": self.load_governor.achievable_torque_nm,
        }
