"""
Cascade Linear Active Disturbance Rejection Control (LADRC) with Asymmetric Reference Governor.
Estimates total plant disturbances in real time via a 3rd-order discrete LESO.
"""

from typing import Any, Dict
import math

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController
from sim.controller.handbook import compute_recommended_spindle_rpm
from sim.controller.soft_sensor import SoftWOBObserver
from sim.controller.state_machine import OperatingMode, SupervisoryStateMachine


class DiscreteLESO:
    """
    3rd-Order Discrete Linear Extended State Observer (LESO) for second-order plant:
    d^2 y / dt^2 = f(y, dy/dt, w, t) + b0 * u
    States: z1 = y, z2 = dy/dt, z3 = f (total lumped disturbance)
    """

    def __init__(self, omega_o: float = 35.0, b0: float = 8.5e3) -> None:
        self.omega_o = omega_o                  # Observer bandwidth (rad/s)
        self.b0 = b0                            # High-frequency control gain (Pa / (RPM^2 * s^2))

        self.z1 = 0.0                           # Estimated output y
        self.z2 = 0.0                           # Estimated velocity dy/dt
        self.z3 = 0.0                           # Estimated total disturbance f(t)

    def reset(self, initial_y: float = 0.0) -> None:
        """Reset observer states."""
        self.z1 = initial_y
        self.z2 = 0.0
        self.z3 = 0.0

    def update(self, dt: float, y_meas: float, u_applied: float, freeze_disturbance: bool = False) -> None:
        """
        Advance LESO states using exact discrete pole placement at z = exp(-omega_o * dt).
        """
        beta = math.exp(-self.omega_o * dt)

        # Exact discrete LESO gains (Herbst formulation)
        l1 = 1.0 - (beta ** 3)
        l2 = (3.0 / (2.0 * dt)) * ((1.0 - beta) ** 2) * (1.0 + beta)
        l3 = (1.0 / (dt * dt)) * ((1.0 - beta) ** 3)

        # 1. Prediction step (Euler/ZOH propagation)
        z1_pred = self.z1 + self.z2 * dt + 0.5 * (self.z3 + self.b0 * u_applied) * (dt ** 2)
        z2_pred = self.z2 + (self.z3 + self.b0 * u_applied) * dt
        z3_pred = self.z3

        # 2. Correction step from measurement innovation
        err = y_meas - z1_pred

        self.z1 = z1_pred + l1 * err
        self.z2 = z2_pred + l2 * err
        if not freeze_disturbance:
            self.z3 = z3_pred + l3 * err


class CascadeLADRCController(IController):
    """
    Primary milling controller coupling:
    - Master Load Governor: CNC handbook spindle RPM & target WOB
    - Slave Pressure Regulator: Discrete LADRC with 3rd-order LESO
    - Asymmetric Reference Governor: active back-off & rate-limiting to handle pressure trapping
    """

    def __init__(
        self,
        target_pressure_bar: float = 35.0,
        spindle_rpm_nominal: float = 3500.0,
        omega_c: float = 10.0,                   # Controller bandwidth (rad/s)
        omega_o: float = 40.0,                   # Observer bandwidth (rad/s, ~4*omega_c)
        b0: float = 12.0,                        # Gain scaling: Pa / (RPM^2 * s^2)
        max_pump_rpm: float = 4500.0,
        max_pressure_rate_bar_per_s: float = 15.0,# Slew rate limit on reference ramp
    ) -> None:
        self.target_pressure_bar = target_pressure_bar
        self.spindle_rpm_nominal = spindle_rpm_nominal
        self.omega_c = omega_c
        self.omega_o = omega_o
        self.b0 = b0
        self.max_pump_rpm = max_pump_rpm
        self.max_rate_bar_s = max_pressure_rate_bar_per_s

        self.state_machine = SupervisoryStateMachine()
        self.soft_wob = SoftWOBObserver()
        self.leso = DiscreteLESO(omega_o=self.omega_o, b0=self.b0)

        # Controller gains (critically damped: kp = omega_c^2, kd = 2*omega_c)
        self.kp = self.omega_c ** 2
        self.kd = 2.0 * self.omega_c

        # State memory
        self.filtered_ref_bar = 1.01325
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def reset(self) -> None:
        """Reset internal controller, LESO, and supervisory state."""
        self.state_machine.reset(nominal_spindle_rpm=self.spindle_rpm_nominal)
        self.soft_wob.reset()
        self.leso.reset(initial_y=1.01325)
        self.filtered_ref_bar = 1.01325
        self.last_pump_cmd_rpm = 0.0
        self.current_mode = OperatingMode.APPROACH

    def update(self, dt: float, sensors: ProcessVariables) -> ControlCommands:
        """Execute discrete control step."""
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

        # 3. Asymmetric Reference Governor: determine target pressure
        if self.current_mode == OperatingMode.APPROACH:
            target_p = 12.0                     # Low pressure limit in approach
        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            target_p = 18.0                     # Moderate contact seat pressure
        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            target_p = self.target_pressure_bar
            # Overload torque backoff: if torque is high, proactively pull back reference
            if sensors.spindle_torque_est > 5.5:
                backoff = 4.0 * (sensors.spindle_torque_est - 5.5)
                target_p = max(15.0, target_p - backoff)
        elif self.current_mode == OperatingMode.PRESSURE_RELAXATION:
            target_p = 0.0                      # Idle pump completely
        elif self.current_mode == OperatingMode.OVERLOAD_RECOVERY:
            target_p = 0.0
        else:
            target_p = 0.0

        # Rate limiter on reference increase (prevents pumping faster than cutting clearance)
        delta_p_max = self.max_rate_bar_s * dt
        if target_p > self.filtered_ref_bar + delta_p_max:
            self.filtered_ref_bar += delta_p_max
        else:
            # Immediate downward response allowed
            self.filtered_ref_bar = target_p

        # 4. Discrete LESO Update
        # u is normalized pump effort: u = (omega_pump / 1000)^2
        u_prev = (self.last_pump_cmd_rpm / 1000.0) ** 2
        # Anti-windup flag: freeze disturbance if pump was commanded to 0
        freeze_dist = (self.last_pump_cmd_rpm <= 10.0 and self.current_mode == OperatingMode.PRESSURE_RELAXATION)

        self.leso.update(
            dt=dt,
            y_meas=sensors.pressure_bar,
            u_applied=u_prev,
            freeze_disturbance=freeze_dist,
        )

        # 5. LADRC Control Law
        # Feedback error
        e = self.filtered_ref_bar - self.leso.z1
        u0 = self.kp * e - self.kd * self.leso.z2

        # Rejection of total disturbance z3
        u_linear = (u0 - self.leso.z3) / max(0.01, self.b0)

        # Convert back to pump RPM setpoint
        if u_linear <= 0.0 or self.current_mode in (OperatingMode.PRESSURE_RELAXATION, OperatingMode.OVERLOAD_RECOVERY, OperatingMode.STALL_RECOVERY):
            pump_rpm_cmd = 0.0
        else:
            pump_rpm_cmd = math.sqrt(u_linear) * 1000.0

        # Clamp to hardware limits
        pump_rpm_cmd = max(0.0, min(self.max_pump_rpm, pump_rpm_cmd))

        # Asymmetric ramp limiter on actuator command (positive: 1500 RPM/s, negative: 5000 RPM/s)
        delta_cmd = pump_rpm_cmd - self.last_pump_cmd_rpm
        max_up = 1500.0 * dt
        max_down = 5000.0 * dt
        if delta_cmd > max_up:
            pump_rpm_cmd = self.last_pump_cmd_rpm + max_up
        elif delta_cmd < -max_down:
            pump_rpm_cmd = self.last_pump_cmd_rpm - max_down

        self.last_pump_cmd_rpm = pump_rpm_cmd
        spindle_rpm_cmd = self.spindle_rpm_nominal if self.current_mode != OperatingMode.STALL_RECOVERY else 0.0

        return ControlCommands(
            timestamp=sensors.timestamp,
            pump_speed_cmd_rpm=pump_rpm_cmd,
            spindle_speed_cmd_rpm=spindle_rpm_cmd,
            enable_pump=(self.current_mode != OperatingMode.STALL_RECOVERY),
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
        }
