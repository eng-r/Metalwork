"""
Physics-informed cascade LADRC controller.

Architecture:
    desired ToB
      -> ToB/WOB pressure governor (physics feedforward + PI trim)
      -> centrifugal pump head feedforward
      -> first-order pressure LADRC correction
      -> pump RPM

The supervisory state machine is reserved for genuine safety/recovery events. Normal pressure
crossings do not force the pump to zero.
"""

from typing import Any, Dict
import math

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


class FirstOrderPressureESO:
    """
    Linear ESO for the local pressure-error model:

        P_dot = f + b0 * v

    where v is deviation of normalized pump-speed-squared from the known pump-map feedforward.
    """

    def __init__(
        self,
        omega_o: float = 14.0,
        b0: float = 75.0,
        disturbance_limit_bar_s: float = 220.0,
    ) -> None:
        self.omega_o = omega_o
        self.b0 = b0
        self.disturbance_limit_bar_s = disturbance_limit_bar_s
        self.z1 = 0.0
        self.z2 = 0.0

    def reset(self, initial_y: float = 0.0) -> None:
        self.z1 = initial_y
        self.z2 = 0.0

    def update(
        self,
        dt: float,
        y_meas: float,
        v_applied: float,
    ) -> None:
        innovation = y_meas - self.z1
        beta1 = 2.0 * self.omega_o
        beta2 = self.omega_o**2

        self.z1 += dt * (
            self.z2 + self.b0 * v_applied + beta1 * innovation
        )
        self.z2 += dt * beta2 * innovation
        self.z2 = max(
            -self.disturbance_limit_bar_s,
            min(self.disturbance_limit_bar_s, self.z2),
        )


# Backward-compatible import name.
DiscreteLESO = FirstOrderPressureESO


class CascadeLADRCController(IController):
    def __init__(
        self,
        target_pressure_bar: float = 35.0,
        target_torque_nm: float = 4.0,
        spindle_rpm_nominal: float = 3500.0,
        omega_c: float = 5.0,
        omega_o: float = 14.0,
        b0: float = 75.0,
        max_pump_rpm: float = 4500.0,
    ) -> None:
        self.target_pressure_bar = target_pressure_bar
        self.target_torque_nm = target_torque_nm
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.omega_c = omega_c
        self.omega_o = omega_o
        self.b0 = b0
        self.max_pump_rpm = max_pump_rpm

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
        self.leso = FirstOrderPressureESO(
            omega_o=omega_o,
            b0=b0,
        )

        self.current_mode = OperatingMode.APPROACH
        self.pressure_reference_bar = 12.0
        self.pump_feedforward_rpm = 0.0
        self.last_pump_cmd_rpm = 0.0
        self.last_u_delta = 0.0


    @property
    def filtered_ref_bar(self) -> float:
        """Backward-compatible alias for older diagnostics/tests."""
        return self.pressure_reference_bar

    @filtered_ref_bar.setter
    def filtered_ref_bar(self, value: float) -> None:
        self.pressure_reference_bar = float(value)

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
        self.leso.reset(initial_y=1.01325)
        self.current_mode = OperatingMode.APPROACH
        self.pressure_reference_bar = 12.0
        self.pump_feedforward_rpm = self.pump_ff.rpm_for_pressure(12.0)
        self.last_pump_cmd_rpm = 0.0
        self.last_u_delta = 0.0

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
            self.pump_feedforward_rpm = 0.0
            u_ff = 0.0
        else:
            self.pump_feedforward_rpm = self.pump_ff.rpm_for_pressure(
                self.pressure_reference_bar
            )
            u_ff = (self.pump_feedforward_rpm / 1000.0) ** 2

        # Observer input is actual pump-speed deviation from the known quasi-static feedforward.
        u_actual = (max(0.0, sensors.pump_rpm) / 1000.0) ** 2
        v_actual = u_actual - u_ff
        self.leso.update(
            dt,
            y_meas=sensors.pressure_bar,
            v_applied=v_actual,
        )

        if recovery_mode:
            pump_rpm_cmd = 0.0
            self.last_u_delta = -u_ff
        else:
            pressure_error = self.pressure_reference_bar - self.leso.z1
            desired_rate = self.omega_c * pressure_error
            u_delta = (desired_rate - self.leso.z2) / max(1.0e-6, self.b0)

            # Smooth normal operation: ADRC trims around the feedforward operating point.
            # No pump on/off gate is used for ordinary pressure crossings.
            u_total = max(0.0, u_ff + u_delta)
            pump_rpm_cmd = math.sqrt(u_total) * 1000.0
            pump_rpm_cmd = max(0.0, min(self.max_pump_rpm, pump_rpm_cmd))
            self.last_u_delta = u_delta

        self.last_pump_cmd_rpm = pump_rpm_cmd
        return ControlCommands(
            timestamp=sensors.timestamp,
            pump_speed_cmd_rpm=pump_rpm_cmd,
            spindle_speed_cmd_rpm=self.spindle_rpm_nominal,
            enable_pump=not recovery_mode,
            enable_spindle=True,
        )

    def get_operating_mode(self) -> str:
        return self.current_mode.value

    def get_internal_states(self) -> Dict[str, Any]:
        return {
            "controller_type": "CascadeLADRC",
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
            "pump_feedback_delta_u": self.last_u_delta,
            "leso_z1_pressure": self.leso.z1,
            "leso_z2_disturbance_bar_s": self.leso.z2,
            # Legacy key retained for the existing UI card.
            "leso_z3_disturbance": self.leso.z2,
            "control_limited": self.load_governor.control_limited,
            "limit_reason": self.load_governor.limit_reason,
            "achievable_torque_nm": self.load_governor.achievable_torque_nm,
        }
