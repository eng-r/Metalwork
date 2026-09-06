"""
Asymmetric first-order LADRC pressure controller with outer ToB governor.

The hydraulic continuity equation is fundamentally first-order in pressure:
    P_dot = beta/V * (Qpump - Qloss - A*x_dot)
so the inner ADRC model is written as
    P_dot = f(t) + b0*u
where f(t) lumps pump nonlinearity, back-pressure, rod displacement flow,
leakage and cutting/mechanical coupling.

This is a better control-oriented model than treating pressure as a generic
second-order plant.
"""

from typing import Any, Dict
import math

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController
from sim.controller.load_governor import (
    TorqueGovernorConfig,
    TorqueToPressureGovernor,
)
from sim.controller.soft_sensor import SoftWOBObserver
from sim.controller.state_machine import (
    OperatingMode,
    SupervisoryStateMachine,
)


class FirstOrderPressureESO:
    """
    2-state ESO for:
        y_dot = f + b0*u

    z1 estimates pressure [bar]
    z2 estimates total pressure-rate disturbance [bar/s]
    """

    def __init__(
        self,
        omega_o: float = 12.0,
        b0: float = 18.0,
        disturbance_limit_bar_s: float = 250.0,
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
        u_applied: float,
    ) -> None:
        innovation = y_meas - self.z1

        beta1 = 2.0 * self.omega_o
        beta2 = self.omega_o ** 2

        z1_dot = (
            self.z2
            + self.b0 * u_applied
            + beta1 * innovation
        )
        z2_dot = beta2 * innovation

        self.z1 += dt * z1_dot
        self.z2 += dt * z2_dot
        self.z2 = max(
            -self.disturbance_limit_bar_s,
            min(
                self.disturbance_limit_bar_s,
                self.z2,
            ),
        )


# Backward-compatible import name if any external script used it.
DiscreteLESO = FirstOrderPressureESO


class CascadeLADRCController(IController):
    """Outer ToB governor + asymmetric first-order LADRC pressure loop."""

    def __init__(
        self,
        target_pressure_bar: float = 35.0,
        target_torque_nm: float = 4.0,
        spindle_rpm_nominal: float = 3500.0,
        omega_c: float = 3.0,
        omega_o: float = 12.0,
        b0: float = 18.0,
        max_pump_rpm: float = 4500.0,
        max_pressure_rate_bar_per_s: float = 15.0,
    ) -> None:
        self.target_pressure_bar = target_pressure_bar
        self.target_torque_nm = target_torque_nm
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.omega_c = omega_c
        self.omega_o = omega_o
        self.b0 = b0
        self.max_pump_rpm = max_pump_rpm
        self.max_rate_bar_s = max_pressure_rate_bar_per_s

        overload = max(
            target_torque_nm * 1.55,
            target_torque_nm + 1.8,
        )
        safe = max(
            target_torque_nm * 1.12,
            target_torque_nm + 0.5,
        )

        self.state_machine = SupervisoryStateMachine(
            overload_torque_threshold=overload,
            safe_torque_threshold=safe,
        )
        self.soft_wob = SoftWOBObserver()

        self.leso = FirstOrderPressureESO(
            omega_o=self.omega_o,
            b0=self.b0,
        )
        self.load_governor = TorqueToPressureGovernor(
            TorqueGovernorConfig(
                target_torque_nm=target_torque_nm,
                pressure_ceiling_bar=target_pressure_bar,
            )
        )

        self.filtered_ref_bar = 1.01325
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def apply_runtime_config(
        self,
        target_torque_nm: float,
        pressure_ceiling_bar: float,
        spindle_rpm_nominal: float,
    ) -> None:
        self.target_torque_nm = max(
            0.1,
            float(target_torque_nm),
        )
        self.target_pressure_bar = max(
            10.5,
            float(pressure_ceiling_bar),
        )
        self.spindle_rpm_nominal = max(
            100.0,
            float(spindle_rpm_nominal),
        )

        self.load_governor.set_target_torque(
            self.target_torque_nm,
        )
        self.load_governor.set_pressure_ceiling(
            self.target_pressure_bar,
        )

        self.state_machine.nominal_spindle_rpm = (
            self.spindle_rpm_nominal
        )
        self.state_machine.overload_torque_thresh = max(
            self.target_torque_nm * 1.55,
            self.target_torque_nm + 1.8,
        )
        self.state_machine.safe_torque_thresh = max(
            self.target_torque_nm * 1.12,
            self.target_torque_nm + 0.5,
        )

    def reset(self) -> None:
        self.state_machine.reset(
            nominal_spindle_rpm=self.spindle_rpm_nominal
        )
        self.soft_wob.reset()
        self.leso.reset(initial_y=1.01325)
        self.load_governor.reset(
            initial_pressure_bar=18.0
        )
        self.filtered_ref_bar = 1.01325
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def update(
        self,
        dt: float,
        sensors: ProcessVariables,
    ) -> ControlCommands:
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
            target_p = 12.0
        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            target_p = 18.0
        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            target_p = self.load_governor.update(
                dt=dt,
                measured_torque_nm=sensors.spindle_torque_est,
            )
        else:
            # All recovery modes are passive hydraulic unload modes.
            target_p = 0.0

        # Limit pressure-reference rise. Downward changes are immediate because
        # the actuator is asymmetric and cannot command negative pump flow.
        max_up = self.max_rate_bar_s * dt
        if target_p > self.filtered_ref_bar + max_up:
            self.filtered_ref_bar += max_up
        else:
            self.filtered_ref_bar = target_p

        # Use ACTUAL pump speed in the observer, not the previous command.
        # The pump drive has non-negligible acceleration dynamics.
        u_applied = (
            max(0.0, sensors.pump_rpm) / 1000.0
        ) ** 2
        self.leso.update(
            dt=dt,
            y_meas=sensors.pressure_bar,
            u_applied=u_applied,
        )

        pressure_error = (
            self.filtered_ref_bar - self.leso.z1
        )
        desired_pressure_rate = (
            self.omega_c * pressure_error
        )

        u_linear = (
            desired_pressure_rate - self.leso.z2
        ) / max(0.01, self.b0)

        torque_is_high = (
            self.load_governor.filtered_torque_nm
            > self.target_torque_nm
            + self.load_governor.config.torque_deadband_nm
        )
        pressure_is_above_ref = (
            sensors.pressure_bar
            > self.filtered_ref_bar + 0.25
        )

        recovery_mode = self.current_mode in (
            OperatingMode.PRESSURE_RELAXATION,
            OperatingMode.OVERLOAD_RECOVERY,
            OperatingMode.STALL_RECOVERY,
        )

        # Critical asymmetric-control rule:
        # when load/pressure is above the requested level, DO NOT let ADRC
        # "reject" the desired pressure decay by adding pump flow.
        if (
            recovery_mode
            or pressure_is_above_ref
            or torque_is_high
            or u_linear <= 0.0
        ):
            pump_rpm_cmd = 0.0
        else:
            pump_rpm_cmd = (
                math.sqrt(u_linear) * 1000.0
            )

        pump_rpm_cmd = max(
            0.0,
            min(self.max_pump_rpm, pump_rpm_cmd),
        )
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
            "leso_z1_pressure": self.leso.z1,
            "leso_z2_disturbance_bar_s": self.leso.z2,

            # Backward-compatible telemetry key.
            "leso_z3_disturbance": self.leso.z2,

            "filtered_reference_bar": self.filtered_ref_bar,
            "target_torque_nm": self.target_torque_nm,
            "torque_error_nm": self.load_governor.last_error_nm,
            "filtered_torque_nm": (
                self.load_governor.filtered_torque_nm
            ),
            "torque_pressure_reference_bar": (
                self.load_governor.reference_bar
            ),
            "pressure_ceiling_bar": self.target_pressure_bar,
        }
