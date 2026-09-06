"""
Axial pusher mechanics with pressure-dependent seal friction and explicit stiction.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class MechanicsParameters:
    effective_mass: float = 45.0
    viscous_damping: float = 120.0
    stroke_max: float = 0.25
    endstop_stiffness: float = 5.0e7
    endstop_damping: float = 1.0e5

    coulomb_friction_base: float = 220.0
    coulomb_pressure_alpha: float = 1.2e-4
    static_friction_base: float = 480.0
    static_pressure_alpha: float = 2.4e-4
    stribeck_velocity: float = 0.008
    stribeck_exponent: float = 1.5
    viscous_seal_base: float = 350.0
    viscous_seal_alpha: float = 8.0e-5

    velocity_transition_tol: float = 1.0e-4

    # Soft-sensor prior: static seal force is not uniquely observable from pressure alone.
    # The observer uses this fraction of the breakaway limit while the rod is stuck.
    static_friction_observer_fraction: float = 0.65


class MechanicsSubsystem:
    def __init__(
        self,
        params: MechanicsParameters = MechanicsParameters(),
    ) -> None:
        self.params = params
        self.position = 0.0
        self.velocity = 0.0

    def reset(
        self,
        initial_position: float = 0.0,
        initial_velocity: float = 0.0,
    ) -> None:
        self.position = initial_position
        self.velocity = initial_velocity

    def friction_levels(
        self,
        gauge_pressure: float,
    ) -> Tuple[float, float, float]:
        """Return Coulomb, static-breakaway, and viscous seal coefficients."""
        p_eff = max(0.0, gauge_pressure)
        f_c = (
            self.params.coulomb_friction_base
            + self.params.coulomb_pressure_alpha * p_eff
        )
        f_s = (
            self.params.static_friction_base
            + self.params.static_pressure_alpha * p_eff
        )
        sigma_v = (
            self.params.viscous_seal_base
            + self.params.viscous_seal_alpha * p_eff
        )
        return f_c, f_s, sigma_v

    # Backward-compatible alias for older code/tests.
    _friction_levels = friction_levels

    def compute_seal_friction(
        self,
        gauge_pressure: float,
        velocity: float,
        impending_force: float = 0.0,
    ) -> float:
        """
        Signed seal friction opposing actual or incipient rod motion.

        In the static regime friction exactly balances the net sub-breakaway force. This is the
        plant truth model; a soft sensor cannot know that exact value without another independent
        load measurement.
        """
        f_c, f_s, sigma_v = self.friction_levels(gauge_pressure)
        tol = self.params.velocity_transition_tol

        if abs(velocity) < tol:
            if abs(impending_force) <= f_s:
                return impending_force
            return math.copysign(f_s, impending_force)

        abs_v = abs(velocity)
        stribeck = math.exp(
            -(
                abs_v
                / max(1.0e-6, self.params.stribeck_velocity)
            ) ** self.params.stribeck_exponent
        )
        dry = f_c + (f_s - f_c) * stribeck
        magnitude = dry + sigma_v * abs_v
        return math.copysign(magnitude, velocity)

    def estimate_forward_seal_friction(
        self,
        gauge_pressure: float,
        velocity: float,
    ) -> float:
        """
        Observer-side prior for forward pusher friction.

        Static friction is set-valued. When the rod is nearly stationary we therefore return a
        calibrated fraction of the breakaway bound rather than the impossible-to-know exact plant
        stiction force. This makes the WOB estimate explicit about the information limitation.
        """
        f_c, f_s, sigma_v = self.friction_levels(gauge_pressure)
        tol = self.params.velocity_transition_tol
        if abs(velocity) < tol:
            return self.params.static_friction_observer_fraction * f_s

        abs_v = abs(velocity)
        stribeck = math.exp(
            -(
                abs_v
                / max(1.0e-6, self.params.stribeck_velocity)
            ) ** self.params.stribeck_exponent
        )
        dry = f_c + (f_s - f_c) * stribeck
        return math.copysign(dry + sigma_v * abs_v, velocity)

    def compute_endstop_force(
        self,
        position: float,
        velocity: float,
    ) -> float:
        if position < 0.0:
            penetration = -position
            return (
                self.params.endstop_stiffness * penetration
                - self.params.endstop_damping * velocity
            )
        if position > self.params.stroke_max:
            penetration = position - self.params.stroke_max
            return -(
                self.params.endstop_stiffness * penetration
                + self.params.endstop_damping * velocity
            )
        return 0.0

    def compute_acceleration(
        self,
        gauge_pressure: float,
        piston_area: float,
        axial_cutting_force: float,
        position: float,
        velocity: float,
    ) -> Tuple[float, float]:
        f_hyd = max(0.0, gauge_pressure) * piston_area
        f_structural = self.params.viscous_damping * velocity
        f_stop = self.compute_endstop_force(position, velocity)

        impending = (
            f_hyd
            - f_structural
            - axial_cutting_force
            + f_stop
        )
        f_friction = self.compute_seal_friction(
            gauge_pressure,
            velocity,
            impending_force=impending,
        )

        _, f_static_limit, _ = self.friction_levels(gauge_pressure)
        if (
            abs(velocity) < self.params.velocity_transition_tol
            and abs(impending) <= f_static_limit
        ):
            return 0.0, f_friction

        acceleration = (impending - f_friction) / self.params.effective_mass
        return acceleration, f_friction
