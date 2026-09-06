"""
Safety supervisor for the asymmetric hydraulic milling process.

The supervisor is intentionally not a second load controller. ToB setpoint changes do not retune
its limits. It intervenes only for contact acquisition and genuine hard safety/recovery events.
"""

from enum import Enum


class OperatingMode(str, Enum):
    APPROACH = "APPROACH"
    CONTACT_ACQUISITION = "CONTACT_ACQUISITION"
    NORMAL_MILLING = "NORMAL_MILLING"
    PRESSURE_RELAXATION = "PRESSURE_RELAXATION"
    OVERLOAD_RECOVERY = "OVERLOAD_RECOVERY"
    STALL_RECOVERY = "STALL_RECOVERY"


class SupervisoryStateMachine:
    def __init__(
        self,
        contact_torque_threshold: float = 0.45,
        contact_wob_threshold_n: float = 140.0,
        overload_torque_threshold: float = 8.2,
        safe_torque_threshold: float = 6.2,
        max_pressure_bar: float = 55.0,
        safe_pressure_bar: float = 45.0,
        stall_speed_fraction: float = 0.55,
        deep_stall_speed_fraction: float = 0.30,
    ) -> None:
        self.contact_torque_thresh = contact_torque_threshold
        self.contact_wob_thresh_n = contact_wob_threshold_n
        self.overload_torque_thresh = overload_torque_threshold
        self.safe_torque_thresh = safe_torque_threshold
        self.max_pressure_bar = max_pressure_bar
        self.safe_pressure_bar = safe_pressure_bar
        self.stall_speed_frac = stall_speed_fraction
        self.deep_stall_speed_frac = deep_stall_speed_fraction

        self.current_mode = OperatingMode.APPROACH
        self.nominal_spindle_rpm = 3500.0
        self.mode_timer = 0.0

    def reset(self, nominal_spindle_rpm: float = 3500.0) -> None:
        self.current_mode = OperatingMode.APPROACH
        self.nominal_spindle_rpm = nominal_spindle_rpm
        self.mode_timer = 0.0

    def update(
        self,
        dt: float,
        pressure_bar: float,
        spindle_rpm: float,
        torque_est_nm: float,
        wob_est_n: float,
    ) -> OperatingMode:
        self.mode_timer += dt

        contact_pressure_present = pressure_bar > 6.0
        torque_contact = torque_est_nm > self.contact_torque_thresh
        wob_contact = wob_est_n > self.contact_wob_thresh_n
        in_cut = contact_pressure_present and (torque_contact or wob_contact)

        # Deep stall has highest priority once contact is established.
        if (
            self.current_mode != OperatingMode.APPROACH
            and in_cut
            and spindle_rpm < self.nominal_spindle_rpm * self.deep_stall_speed_frac
        ):
            self.current_mode = OperatingMode.STALL_RECOVERY
            self.mode_timer = 0.0
            return self.current_mode

        if self.current_mode == OperatingMode.APPROACH:
            if in_cut:
                self.current_mode = OperatingMode.CONTACT_ACQUISITION
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            if self.mode_timer > 0.15:
                if in_cut:
                    self.current_mode = OperatingMode.NORMAL_MILLING
                    self.mode_timer = 0.0
                elif self.mode_timer > 0.60:
                    self.current_mode = OperatingMode.APPROACH
                    self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            if (
                torque_est_nm > self.overload_torque_thresh
                or pressure_bar > self.max_pressure_bar
            ):
                self.current_mode = OperatingMode.PRESSURE_RELAXATION
                self.mode_timer = 0.0
            elif (
                in_cut
                and spindle_rpm < self.nominal_spindle_rpm * self.stall_speed_frac
            ):
                self.current_mode = OperatingMode.OVERLOAD_RECOVERY
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.PRESSURE_RELAXATION:
            if (
                torque_est_nm < self.safe_torque_thresh
                and pressure_bar < self.safe_pressure_bar
                and spindle_rpm > self.nominal_spindle_rpm * 0.75
            ):
                self.current_mode = OperatingMode.NORMAL_MILLING
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.OVERLOAD_RECOVERY:
            if (
                spindle_rpm > self.nominal_spindle_rpm * 0.90
                and torque_est_nm < self.safe_torque_thresh
            ):
                self.current_mode = OperatingMode.NORMAL_MILLING
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.STALL_RECOVERY:
            if (
                spindle_rpm > self.nominal_spindle_rpm * 0.75
                or self.mode_timer > 1.5
            ):
                self.current_mode = OperatingMode.PRESSURE_RELAXATION
                self.mode_timer = 0.0

        return self.current_mode
