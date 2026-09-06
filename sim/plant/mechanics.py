"""
Pusher mechanics with pressure-dependent seal friction and true stiction.

The previous regularized tanh law produced exactly zero seal friction at zero
velocity. That is not static friction and can create artificial rod/pressure
limit cycles. This implementation explicitly balances sub-breakaway force.
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

    # Velocity below which a static-force balance is allowed.
    velocity_transition_tol: float = 1.0e-4


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

    def _friction_levels(
        self,
        gauge_pressure: float,
    ) -> Tuple[float, float, float]:
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

    def compute_seal_friction(
        self,
        gauge_pressure: float,
        velocity: float,
        impending_force: float = 0.0,
    ) -> float:
        """
        Signed friction force in the direction opposing motion/incipient motion.
        """
        f_c, f_s, sigma_v = self._friction_levels(
            gauge_pressure,
        )
        tol = self.params.velocity_transition_tol

        if abs(velocity) < tol:
            if abs(impending_force) <= f_s:
                # Exact stiction balance.
                return impending_force
            direction = 1.0 if impending_force >= 0.0 else -1.0
            return direction * f_s

        abs_v = abs(velocity)
        stribeck_factor = math.exp(
            -(
                abs_v
                / max(
                    1.0e-6,
                    self.params.stribeck_velocity,
                )
            )
            ** self.params.stribeck_exponent
        )
        dry_magnitude = (
            f_c
            + (f_s - f_c) * stribeck_factor
        )
        total_magnitude = dry_magnitude + sigma_v * abs_v
        return math.copysign(total_magnitude, velocity)

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
        f_hyd = gauge_pressure * piston_area
        f_structural = self.params.viscous_damping * velocity
        f_stop = self.compute_endstop_force(
            position,
            velocity,
        )

        # Force that the seal friction must oppose.
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

        _, f_static_limit, _ = self._friction_levels(
            gauge_pressure,
        )
        if (
            abs(velocity) < self.params.velocity_transition_tol
            and abs(impending) <= f_static_limit
        ):
            # Stuck seal: friction exactly balances the remaining load.
            return 0.0, f_friction

        f_net = impending - f_friction
        acceleration = f_net / self.params.effective_mass
        return acceleration, f_friction
