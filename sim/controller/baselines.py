"""
Gain-Scheduled Baseline PID Controller with Tracking Anti-Windup and Asymmetric Slew Limits.
Serves as the benchmark comparator for advanced adaptive architectures.
"""

from typing import Any, Dict
import math

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController
from sim.controller.handbook import compute_recommended_spindle_rpm
from sim.controller.soft_sensor import SoftWOBObserver
from sim.controller.state_machine import OperatingMode, SupervisoryStateMachine


class BaselinePIDController(IController):
    """
    Classical baseline controller regulating line pressure or soft WOB via pump RPM.
    Incorporates anti-windup, asymmetric slew limits, and supervisory state machine.
    """

    def __init__(
        self,
        target_pressure_bar: float = 35.0,       # Desired steady cutting pressure (bar)
        spindle_rpm_nominal: float = 3500.0,     # Desired cutting speed
        kp: float = 85.0,                        # Proportional gain (RPM / bar)
        ki: float = 40.0,                        # Integral gain (RPM / (bar * s))
        kd: float = 12.0,                        # Derivative gain (RPM * s / bar)
        max_pump_rpm: float = 4500.0,
        positive_slew_rpm_per_s: float = 1200.0, # Slower positive ramp to avoid slamming
        negative_slew_rpm_per_s: float = 4000.0, # Faster negative ramp to drop command
    ) -> None:
        self.target_pressure_bar = target_pressure_bar
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_pump_rpm = max_pump_rpm
        self.pos_slew = positive_slew_rpm_per_s
        self.neg_slew = negative_slew_rpm_per_s

        self.state_machine = SupervisoryStateMachine()
        self.soft_wob = SoftWOBObserver()

        # Controller state variables
        self.integrator = 0.0
        self.prev_error = 0.0
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def reset(self) -> None:
        """Reset internal integrator and state machine."""
        self.state_machine.reset(nominal_spindle_rpm=self.spindle_rpm_nominal)
        self.soft_wob.reset()
        self.integrator = 0.0
        self.prev_error = 0.0
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def update(self, dt: float, sensors: ProcessVariables) -> ControlCommands:
        """Compute next actuator commands."""
        # 1. Update Soft WOB sensor
        wob_est = self.soft_wob.update(dt, sensors)
        sensors.wob_soft_sensor = wob_est

        # 2. Update supervisory state machine
        self.current_mode = self.state_machine.update(
            dt=dt,
            pressure_bar=sensors.pressure_bar,
            spindle_rpm=sensors.spindle_rpm,
            torque_est_nm=sensors.spindle_torque_est,
            wob_est_n=wob_est,
        )

        # 3. Mode-specific behavior
        if self.current_mode == OperatingMode.APPROACH:
            # Constant moderate feed in free space
            pump_target = 1800.0
            spindle_target = self.spindle_rpm_nominal
            self.integrator = 0.0

        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            # Creep feed to seat cutter
            pump_target = 800.0
            spindle_target = self.spindle_rpm_nominal
            self.integrator = 0.0

        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            # Closed-loop PID regulation on pressure
            error = self.target_pressure_bar - sensors.pressure_bar
            p_term = self.kp * error

            # Conditional integration (anti-windup)
            d_err = (error - self.prev_error) / dt
            d_term = self.kd * d_err
            self.prev_error = error

            # Integrate only if not deeply saturated
            if 0.0 < self.last_pump_cmd_rpm < self.max_pump_rpm:
                self.integrator += error * dt
            i_term = self.ki * self.integrator

            raw_cmd = p_term + i_term + d_term
            pump_target = max(0.0, min(self.max_pump_rpm, raw_cmd))
            spindle_target = self.spindle_rpm_nominal

        elif self.current_mode == OperatingMode.PRESSURE_RELAXATION:
            # Command zero pump speed to allow material removal to relieve pressure
            pump_target = 0.0
            spindle_target = self.spindle_rpm_nominal
            # Clamped integrator reset
            self.integrator = max(-50.0, min(50.0, self.integrator))

        elif self.current_mode == OperatingMode.OVERLOAD_RECOVERY:
            pump_target = 0.0
            spindle_target = self.spindle_rpm_nominal

        else:  # STALL_RECOVERY
            pump_target = 0.0
            spindle_target = 0.0

        # 4. Asymmetric slew rate limiter
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
            enable_pump=(self.current_mode != OperatingMode.STALL_RECOVERY),
            enable_spindle=True,
        )

    def get_operating_mode(self) -> str:
        return self.current_mode.value

    def get_internal_states(self) -> Dict[str, Any]:
        return {
            "controller_type": "BaselinePID",
            "operating_mode": self.current_mode.value,
            "integrator": self.integrator,
            "target_pressure_bar": self.target_pressure_bar,
        }
