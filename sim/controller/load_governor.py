"""
Bumpless outer Torque-on-Bit (ToB) governor.

The operator sets desired cutting torque. This slow outer loop trims the
hydraulic pressure reference; PID or LADRC remains the inner pressure loop.

The governor is intentionally asymmetric because the plant can increase
hydraulic load much faster than it can unload trapped pressure.
"""

from dataclasses import dataclass


@dataclass
class TorqueGovernorConfig:
    target_torque_nm: float = 4.0
    pressure_ceiling_bar: float = 35.0
    pressure_floor_bar: float = 10.0
    initial_reference_bar: float = 18.0

    # Incremental outer-loop gains. Error is N*m and result is bar/s.
    raise_gain_bar_per_nm_s: float = 2.4
    unload_gain_bar_per_nm_s: float = 6.5
    max_raise_rate_bar_s: float = 5.0
    max_unload_rate_bar_s: float = 18.0

    # Do not chase Iq-observer noise around the setpoint.
    torque_deadband_nm: float = 0.08
    torque_filter_tau_s: float = 0.10


class TorqueToPressureGovernor:
    """
    Integrating load governor with bumpless live setpoint changes.

    Unlike the previous absolute formula (bias + Kp*error + integral), this
    governor moves pressure only while ToB error exists. That makes a change of
    desired ToB directly observable and avoids hiding the setpoint behind a
    pressure bias/floor/ceiling saturation.
    """

    def __init__(self, config: TorqueGovernorConfig) -> None:
        self.config = config
        self.reference_bar = max(
            config.pressure_floor_bar,
            min(config.pressure_ceiling_bar, config.initial_reference_bar),
        )
        self.filtered_torque_nm = 0.0
        self.last_error_nm = 0.0

    @property
    def target_torque_nm(self) -> float:
        return self.config.target_torque_nm

    def reset(self, initial_pressure_bar: float | None = None) -> None:
        initial = (
            self.config.initial_reference_bar
            if initial_pressure_bar is None
            else initial_pressure_bar
        )
        self.reference_bar = max(
            self.config.pressure_floor_bar,
            min(self.config.pressure_ceiling_bar, initial),
        )
        self.filtered_torque_nm = 0.0
        self.last_error_nm = 0.0

    def set_target_torque(self, target_torque_nm: float) -> None:
        """Change desired ToB without resetting the governor/reference state."""
        self.config.target_torque_nm = max(0.1, float(target_torque_nm))

    def set_pressure_ceiling(self, ceiling_bar: float) -> None:
        """Change hydraulic authority limit without resetting the controller."""
        self.config.pressure_ceiling_bar = max(
            self.config.pressure_floor_bar + 0.5,
            float(ceiling_bar),
        )
        self.reference_bar = min(
            self.reference_bar,
            self.config.pressure_ceiling_bar,
        )

    def update(self, dt: float, measured_torque_nm: float) -> float:
        measured = max(0.0, float(measured_torque_nm))

        alpha = dt / max(dt, self.config.torque_filter_tau_s + dt)
        self.filtered_torque_nm += alpha * (
            measured - self.filtered_torque_nm
        )

        error = self.config.target_torque_nm - self.filtered_torque_nm
        if abs(error) < self.config.torque_deadband_nm:
            error_for_control = 0.0
        else:
            error_for_control = error
        self.last_error_nm = error

        if error_for_control >= 0.0:
            rate = self.config.raise_gain_bar_per_nm_s * error_for_control
            rate = min(self.config.max_raise_rate_bar_s, rate)
        else:
            # Faster unload path for hard spots / chip jams.
            rate = self.config.unload_gain_bar_per_nm_s * error_for_control
            rate = max(-self.config.max_unload_rate_bar_s, rate)

        self.reference_bar += rate * dt
        self.reference_bar = max(
            self.config.pressure_floor_bar,
            min(self.config.pressure_ceiling_bar, self.reference_bar),
        )
        return self.reference_bar
