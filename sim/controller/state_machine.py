"""
Supervisory Finite State Machine for Asymmetric Hydraulic Milling Control.
Manages mode transitions across approach, contact, steady milling, relaxation, and stall recovery.
"""

from enum import Enum
from typing import Tuple


class OperatingMode(str, Enum):
    APPROACH = "APPROACH"
    CONTACT_ACQUISITION = "CONTACT_ACQUISITION"
    NORMAL_MILLING = "NORMAL_MILLING"
    PRESSURE_RELAXATION = "PRESSURE_RELAXATION"
    OVERLOAD_RECOVERY = "OVERLOAD_RECOVERY"
    STALL_RECOVERY = "STALL_RECOVERY"


class SupervisoryStateMachine:
    """
    Evaluates physical sensor cues to safely govern milling process mode.
    """

    def __init__(
        self,
        contact_torque_threshold: float = 0.6,   # N*m to detect contact
        overload_torque_threshold: float = 6.8,  # N*m to trigger relaxation
        safe_torque_threshold: float = 4.5,      # N*m to return from relaxation
        stall_speed_fraction: float = 0.65,      # Speed drops below 65% of nominal -> overload
    ) -> None:
        self.contact_torque_thresh = contact_torque_threshold
        self.overload_torque_thresh = overload_torque_threshold
        self.safe_torque_thresh = safe_torque_threshold
        self.stall_speed_frac = stall_speed_fraction

        self.current_mode = OperatingMode.APPROACH
        self.nominal_spindle_rpm = 3000.0
        self.mode_timer = 0.0

    def reset(self, nominal_spindle_rpm: float = 3000.0) -> None:
        """Reset state machine to initial approach mode."""
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
        """
        Evaluate sensor cues and update supervisory state.
        """
        self.mode_timer += dt

        # Impending stall check: only valid if in contact with load
        is_in_cut = (torque_est_nm > 1.2 or wob_est_n > 50.0)
        if is_in_cut and spindle_rpm < self.nominal_spindle_rpm * 0.35 and self.current_mode != OperatingMode.APPROACH:
            self.current_mode = OperatingMode.STALL_RECOVERY
            self.mode_timer = 0.0
            return self.current_mode

        if self.current_mode == OperatingMode.APPROACH:
            # Transition to contact acquisition if torque rises or pressure surges
            if torque_est_nm > self.contact_torque_thresh or wob_est_n > 80.0:
                self.current_mode = OperatingMode.CONTACT_ACQUISITION
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.CONTACT_ACQUISITION:
            # Confirm stable contact before starting aggressive feed
            if self.mode_timer > 0.2:
                if torque_est_nm > self.contact_torque_thresh * 1.5:
                    self.current_mode = OperatingMode.NORMAL_MILLING
                    self.mode_timer = 0.0
                elif torque_est_nm < self.contact_torque_thresh * 0.5:
                    # False contact / bounced
                    self.current_mode = OperatingMode.APPROACH
                    self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.NORMAL_MILLING:
            # Overload check
            if torque_est_nm > self.overload_torque_thresh or pressure_bar > 60.0:
                self.current_mode = OperatingMode.PRESSURE_RELAXATION
                self.mode_timer = 0.0
            elif spindle_rpm < self.nominal_spindle_rpm * self.stall_speed_frac:
                self.current_mode = OperatingMode.OVERLOAD_RECOVERY
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.PRESSURE_RELAXATION:
            # Pressure relaxation mode: pump is idling; waiting for cutting to clear interference
            if torque_est_nm < self.safe_torque_thresh and pressure_bar < 50.0:
                self.current_mode = OperatingMode.NORMAL_MILLING
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.OVERLOAD_RECOVERY:
            # Spindle bogged down, recovering speed
            if spindle_rpm > self.nominal_spindle_rpm * 0.90:
                self.current_mode = OperatingMode.NORMAL_MILLING
                self.mode_timer = 0.0

        elif self.current_mode == OperatingMode.STALL_RECOVERY:
            # Automated recovery: spindle spins up while pump is zeroed
            if spindle_rpm > self.nominal_spindle_rpm * 0.70 or self.mode_timer > 1.5:
                self.current_mode = OperatingMode.PRESSURE_RELAXATION
                self.mode_timer = 0.0

        return self.current_mode
