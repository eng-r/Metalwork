"""
Outer Torque-on-Bit (ToB) governor.

Converts the operator's desired cutting torque into a slowly varying hydraulic
pressure reference. The inner pressure controller remains either PID or LADRC.
All internal quantities are SI except the pressure reference, which follows the
existing controller convention of bar.
"""

from dataclasses import dataclass


@dataclass
class TorqueGovernorConfig:
    target_torque_nm: float = 4.0
    pressure_ceiling_bar: float = 35.0
    pressure_floor_bar: float = 14.0
    pressure_bias_bar: float = 24.0
    kp_bar_per_nm: float = 4.0
    ki_bar_per_nm_s: float = 1.2
    rise_rate_bar_s: float = 8.0
    fall_rate_bar_s: float = 30.0
    integral_limit_bar: float = 14.0


class TorqueToPressureGovernor:
    """
    Slow outer loop for load control.

    The process cannot unload pressure symmetrically: increasing pump speed can
    raise load quickly, while reducing pressure may require continued material
    removal. Therefore the reference is deliberately slow upward and much faster
    downward.
    """

    def __init__(self, config: TorqueGovernorConfig) -> None:
        self.config = config
        self.integral_bar = 0.0
        self.reference_bar = min(18.0, config.pressure_ceiling_bar)
        self.last_error_nm = 0.0

    @property
    def target_torque_nm(self) -> float:
        return self.config.target_torque_nm

    def reset(self, initial_pressure_bar: float = 18.0) -> None:
        self.integral_bar = 0.0
        self.reference_bar = max(
            self.config.pressure_floor_bar,
            min(self.config.pressure_ceiling_bar, initial_pressure_bar),
        )
        self.last_error_nm = 0.0

    def update(self, dt: float, measured_torque_nm: float) -> float:
        measured = max(0.0, measured_torque_nm)
        error = self.config.target_torque_nm - measured
        self.last_error_nm = error

        self.integral_bar += self.config.ki_bar_per_nm_s * error * dt
        self.integral_bar = max(
            -self.config.integral_limit_bar,
            min(self.config.integral_limit_bar, self.integral_bar),
        )

        desired = (
            self.config.pressure_bias_bar
            + self.config.kp_bar_per_nm * error
            + self.integral_bar
        )

        # Strong proactive unloading after a hardness/chip-induced torque excursion.
        overshoot = max(0.0, measured - self.config.target_torque_nm)
        desired -= 5.0 * overshoot

        desired = max(
            self.config.pressure_floor_bar,
            min(self.config.pressure_ceiling_bar, desired),
        )

        delta = desired - self.reference_bar
        max_up = self.config.rise_rate_bar_s * dt
        max_down = self.config.fall_rate_bar_s * dt
        if delta > max_up:
            self.reference_bar += max_up
        elif delta < -max_down:
            self.reference_bar -= max_down
        else:
            self.reference_bar = desired

        return self.reference_bar
