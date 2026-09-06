"""
Outer Torque-on-Bit (ToB) governor and pump-map feedforward utilities.

The governor uses a physics-informed nominal inverse rather than slowly integrating pressure
until torque happens to change. A PI trim handles model error. This makes upward ToB steps
responsive while preserving the plant's genuinely slower downward unloading dynamics.
"""

from dataclasses import dataclass
import math


@dataclass
class TorqueGovernorConfig:
    target_torque_nm: float = 4.0

    pressure_ceiling_bar: float = 35.0
    pressure_floor_bar: float = 8.0
    atmospheric_pressure_bar: float = 1.01325

    piston_area_m2: float = 2.0e-3

    # Nominal ToB/WOB relation: T ~= c_TF * F_wob.
    torque_force_coeff_m: float = 1.05e-3

    # Nominal forward seal load used only in feedforward inversion. Feedback corrects the error.
    seal_force_feedforward_n: float = 650.0

    kp_bar_per_nm: float = 1.8
    ki_bar_per_nm_s: float = 0.55
    integral_limit_bar: float = 6.0
    torque_deadband_nm: float = 0.06
    torque_filter_tau_s: float = 0.08

    # Reference motion is allowed to be much faster than the previous 5 bar/s limit. The physical
    # pump/contact plant, not an arbitrary outer ramp, determines the actual step-up speed.
    max_reference_rise_bar_s: float = 30.0
    max_reference_fall_bar_s: float = 45.0


class TorqueToPressureGovernor:
    """Physics-informed ToB -> pressure reference governor with bumpless live setpoint changes."""

    def __init__(self, config: TorqueGovernorConfig) -> None:
        self.config = config
        self.filtered_torque_nm = 0.0
        self.last_error_nm = 0.0
        self.integral_bar = 0.0

        self.feedforward_pressure_bar = self._pressure_feedforward(
            config.target_torque_nm
        )
        self.reference_bar = self._clamp_pressure(self.feedforward_pressure_bar)
        self.control_limited = False
        self.limit_reason = ""
        self.achievable_torque_nm = self._max_nominal_torque()
        self.target_wob_n = self._target_wob(config.target_torque_nm)
        self._update_limit_state()

    @property
    def target_torque_nm(self) -> float:
        return self.config.target_torque_nm

    def _target_wob(self, torque_nm: float) -> float:
        return max(
            0.0,
            torque_nm / max(1.0e-8, self.config.torque_force_coeff_m),
        )

    def _pressure_feedforward(self, torque_nm: float) -> float:
        wob = self._target_wob(torque_nm)
        gauge_pa = (
            wob + self.config.seal_force_feedforward_n
        ) / max(1.0e-9, self.config.piston_area_m2)
        return self.config.atmospheric_pressure_bar + gauge_pa / 1.0e5

    def _max_nominal_torque(self) -> float:
        gauge_pa = max(
            0.0,
            (self.config.pressure_ceiling_bar - self.config.atmospheric_pressure_bar)
            * 1.0e5,
        )
        available_wob = max(
            0.0,
            gauge_pa * self.config.piston_area_m2
            - self.config.seal_force_feedforward_n,
        )
        return available_wob * self.config.torque_force_coeff_m

    def _clamp_pressure(self, pressure_bar: float) -> float:
        return max(
            self.config.pressure_floor_bar,
            min(self.config.pressure_ceiling_bar, pressure_bar),
        )

    def _update_limit_state(self) -> None:
        self.achievable_torque_nm = self._max_nominal_torque()
        self.target_wob_n = self._target_wob(self.config.target_torque_nm)
        required = self._pressure_feedforward(self.config.target_torque_nm)
        if required > self.config.pressure_ceiling_bar + 1.0e-9:
            self.control_limited = True
            self.limit_reason = "ToB setpoint exceeds nominal hydraulic authority"
        else:
            self.control_limited = False
            self.limit_reason = ""

    def reset(self, initial_pressure_bar: float | None = None) -> None:
        self.filtered_torque_nm = 0.0
        self.last_error_nm = 0.0
        self.integral_bar = 0.0
        self.feedforward_pressure_bar = self._pressure_feedforward(
            self.config.target_torque_nm
        )
        initial = (
            self.feedforward_pressure_bar
            if initial_pressure_bar is None
            else initial_pressure_bar
        )
        self.reference_bar = self._clamp_pressure(initial)
        self._update_limit_state()

    def set_target_torque(self, target_torque_nm: float) -> None:
        """Bumpless setpoint update; observer/controller states are not reset."""
        self.config.target_torque_nm = max(0.1, float(target_torque_nm))
        self._update_limit_state()

    def set_pressure_ceiling(self, ceiling_bar: float) -> None:
        self.config.pressure_ceiling_bar = max(
            self.config.pressure_floor_bar + 0.5,
            float(ceiling_bar),
        )
        self.reference_bar = min(
            self.reference_bar,
            self.config.pressure_ceiling_bar,
        )
        self._update_limit_state()

    def update(self, dt: float, measured_torque_nm: float) -> float:
        measured = max(0.0, float(measured_torque_nm))
        alpha = dt / max(dt, self.config.torque_filter_tau_s + dt)
        self.filtered_torque_nm += alpha * (measured - self.filtered_torque_nm)

        error = self.config.target_torque_nm - self.filtered_torque_nm
        self.last_error_nm = error
        error_ctrl = 0.0 if abs(error) < self.config.torque_deadband_nm else error

        self.feedforward_pressure_bar = self._pressure_feedforward(
            self.config.target_torque_nm
        )

        # Candidate integral update with conditional anti-windup.
        proposed_integral = self.integral_bar + (
            self.config.ki_bar_per_nm_s * error_ctrl * dt
        )
        proposed_integral = max(
            -self.config.integral_limit_bar,
            min(self.config.integral_limit_bar, proposed_integral),
        )

        desired_unclamped = (
            self.feedforward_pressure_bar
            + self.config.kp_bar_per_nm * error_ctrl
            + proposed_integral
        )
        desired = self._clamp_pressure(desired_unclamped)

        drives_upper_saturation = (
            desired_unclamped > self.config.pressure_ceiling_bar
            and error_ctrl > 0.0
        )
        drives_lower_saturation = (
            desired_unclamped < self.config.pressure_floor_bar
            and error_ctrl < 0.0
        )
        if not (drives_upper_saturation or drives_lower_saturation):
            self.integral_bar = proposed_integral

        delta = desired - self.reference_bar
        max_up = self.config.max_reference_rise_bar_s * dt
        max_down = self.config.max_reference_fall_bar_s * dt
        if delta > max_up:
            self.reference_bar += max_up
        elif delta < -max_down:
            self.reference_bar -= max_down
        else:
            self.reference_bar = desired

        self.reference_bar = self._clamp_pressure(self.reference_bar)
        self._update_limit_state()
        return self.reference_bar


@dataclass
class PumpFeedforwardConfig:
    atmospheric_pressure_bar: float = 1.01325
    pump_a0: float = 38.0
    max_pump_rpm: float = 4500.0


class PumpPressureFeedforward:
    """Q≈0 inverse of the centrifugal-pump head curve."""

    def __init__(self, config: PumpFeedforwardConfig = PumpFeedforwardConfig()) -> None:
        self.config = config

    def rpm_for_pressure(self, pressure_bar_abs: float) -> float:
        delta_pa = max(
            0.0,
            (pressure_bar_abs - self.config.atmospheric_pressure_bar) * 1.0e5,
        )
        if delta_pa <= 0.0:
            return 0.0
        omega = math.sqrt(delta_pa / max(1.0e-12, self.config.pump_a0))
        rpm = omega * 30.0 / math.pi
        return max(0.0, min(self.config.max_pump_rpm, rpm))
