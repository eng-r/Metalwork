"""
Cascade Linear Active Disturbance Rejection Control (LADRC).

Architecture:
- operator sets desired Torque-on-Bit (ToB);
- a slow asymmetric outer governor converts ToB error to pressure reference;
- LADRC regulates hydraulic pressure;
- the supervisory state machine handles approach, relaxation and recovery.
"""

from typing import Any, Dict
import math

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController
from sim.controller.load_governor import TorqueGovernorConfig, TorqueToPressureGovernor
from sim.controller.soft_sensor import SoftWOBObserver
from sim.controller.state_machine import OperatingMode, SupervisoryStateMachine


class DiscreteLESO:
    """Third-order discrete LESO for a second-order pressure-channel model."""

    def __init__(self, omega_o: float = 35.0, b0: float = 8.5e3) -> None:
        self.omega_o = omega_o
        self.b0 = b0
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0

    def reset(self, initial_y: float = 0.0) -> None:
        self.z1 = initial_y
        self.z2 = 0.0
        self.z3 = 0.0

    def update(
        self,
        dt: float,
        y_meas: float,
        u_applied: float,
        freeze_disturbance: bool = False,
    ) -> None:
        beta = math.exp(-self.omega_o * dt)
        l1 = 1.0 - beta**3
        l2 = (3.0 / (2.0 * dt)) * ((1.0 - beta) ** 2) * (1.0 + beta)
        l3 = (1.0 / (dt * dt)) * ((1.0 - beta) ** 3)

        z1_pred = self.z1 + self.z2 * dt + 0.5 * (
            self.z3 + self.b0 * u_applied
        ) * dt**2
        z2_pred = self.z2 + (self.z3 + self.b0 * u_applied) * dt
        z3_pred = self.z3

        err = y_meas - z1_pred
        self.z1 = z1_pred + l1 * err
        self.z2 = z2_pred + l2 * err
        if not freeze_disturbance:
            self.z3 = z3_pred + l3 * err


class CascadeLADRCController(IController):
    """Torque-governed hydraulic milling controller with LADRC inner loop."""

    def __init__(
        self,
        target_pressure_bar: float = 35.0,
        target_torque_nm: float = 4.0,
        spindle_rpm_nominal: float = 3500.0,
        omega_c: float = 10.0,
        omega_o: float = 40.0,
        b0: float = 12.0,
        max_pump_rpm: float = 4500.0,
        max_pressure_rate_bar_per_s: float = 15.0,
    ) -> None:
        # target_pressure_bar is now a hydraulic safety/authority ceiling.
        self.target_pressure_bar = target_pressure_bar
        self.target_torque_nm = target_torque_nm
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.omega_c = omega_c
        self.omega_o = omega_o
        self.b0 = b0
        self.max_pump_rpm = max_pump_rpm
        self.max_rate_bar_s = max_pressure_rate_bar_per_s

        overload = max(target_torque_nm * 1.55, target_torque_nm + 1.8)
        safe = max(target_torque_nm * 1.12, target_torque_nm + 0.5)
        self.state_machine = SupervisoryStateMachine(
            overload_torque_threshold=overload,
            safe_torque_threshold=safe,
        )
        self.soft_wob = SoftWOBObserver()
        self.leso = DiscreteLESO(omega_o=self.omega_o, b0=self.b0)
        self.load_governor = TorqueToPressureGovernor(
            TorqueGovernorConfig(
                target_torque_nm=target_torque_nm,
                pressure_ceiling_bar=target_pressure_bar,
            )
        )

        self.kp = self.omega_c**2
        self.kd = 2.0 * self.omega_c

        self.filtered_ref_bar = 1.01325
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def apply_runtime_config(
        self,
        target_torque_nm: float,
        pressure_ceiling_bar: float,
        spindle_rpm_nominal: float,
    ) -> None:
        """Bumpless live update: do not reset LESO, FSM, or pump command."""
        self.target_torque_nm = max(0.1, float(target_torque_nm))
        self.target_pressure_bar = max(10.5, float(pressure_ceiling_bar))
        self.spindle_rpm_nominal = max(100.0, float(spindle_rpm_nominal))

        self.load_governor.set_target_torque(self.target_torque_nm)
        self.load_governor.set_pressure_ceiling(self.target_pressure_bar)

        self.state_machine.nominal_spindle_rpm = self.spindle_rpm_nominal
        self.state_machine.overload_torque_thresh = max(
            self.target_torque_nm * 1.55,
            self.target_torque_nm + 1.8,
        )
        self.state_machine.safe_torque_thresh = max(
            self.target_torque_nm * 1.12,
            self.target_torque_nm + 0.5,
        )

    def reset(self) -> None:
        self.state_machine.reset(nominal_spindle_rpm=self.spindle_rpm_nominal)
        self.soft_wob.reset()
        self.leso.reset(initial_y=1.01325)
        self.load_governor.reset(initial_pressure_bar=18.0)
        self.filtered_ref_bar = 1.01325
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
            target_p = 12.0
        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            target_p = 18.0
        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            target_p = self.load_governor.update(
                dt=dt,
                measured_torque_nm=sensors.spindle_torque_est,
            )
        elif self.current_mode in (
            OperatingMode.PRESSURE_RELAXATION,
            OperatingMode.OVERLOAD_RECOVERY,
            OperatingMode.STALL_RECOVERY,
        ):
            target_p = 0.0
        else:
            target_p = 0.0

        # Additional command-reference shaping. The outer ToB governor is already
        # asymmetric; this protects the pressure loop from sudden positive steps.
        delta_p_max = self.max_rate_bar_s * dt
        if target_p > self.filtered_ref_bar + delta_p_max:
            self.filtered_ref_bar += delta_p_max
        else:
            self.filtered_ref_bar = target_p

        u_prev = (self.last_pump_cmd_rpm / 1000.0) ** 2
        freeze_dist = (
            self.last_pump_cmd_rpm <= 10.0
            and self.current_mode == OperatingMode.PRESSURE_RELAXATION
        )

        self.leso.update(
            dt=dt,
            y_meas=sensors.pressure_bar,
            u_applied=u_prev,
            freeze_disturbance=freeze_dist,
        )

        e = self.filtered_ref_bar - self.leso.z1
        u0 = self.kp * e - self.kd * self.leso.z2
        u_linear = (u0 - self.leso.z3) / max(0.01, self.b0)

        if u_linear <= 0.0 or self.current_mode in (
            OperatingMode.PRESSURE_RELAXATION,
            OperatingMode.OVERLOAD_RECOVERY,
            OperatingMode.STALL_RECOVERY,
        ):
            pump_rpm_cmd = 0.0
        else:
            pump_rpm_cmd = math.sqrt(u_linear) * 1000.0

        pump_rpm_cmd = max(0.0, min(self.max_pump_rpm, pump_rpm_cmd))

        # Do not add a second artificial pump-speed slew limiter here.
        # The physical PMSMPumpDrive already owns actuator acceleration dynamics.
        # Serial slew limiters were a major source of the geometric triangular
        # pump-RPM waveform.
        self.last_pump_cmd_rpm = pump_rpm_cmd

        return ControlCommands(
            timestamp=sensors.timestamp,
            pump_speed_cmd_rpm=pump_rpm_cmd,
            spindle_speed_cmd_rpm=self.spindle_rpm_nominal,
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
            "controller_type": "CascadeLADRC",
            "operating_mode": self.current_mode.value,
            "leso_z1_pressure": self.leso.z1,
            "leso_z2_dp_dt": self.leso.z2,
            "leso_z3_disturbance": self.leso.z3,
            "filtered_reference_bar": self.filtered_ref_bar,
            "target_torque_nm": self.target_torque_nm,
            "torque_error_nm": self.load_governor.last_error_nm,
            "torque_pressure_reference_bar": self.load_governor.reference_bar,
            "pressure_ceiling_bar": self.target_pressure_bar,
        }
