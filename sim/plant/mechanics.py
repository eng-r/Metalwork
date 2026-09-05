"""
Pusher Mechanics Model: Rod Kinematics, Mass Balance, and Pressure-Dependent Seal Friction.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class MechanicsParameters:
    """Mechanical and friction parameters for the pusher assembly."""
    effective_mass: float = 45.0                # Total moving mass M_eff (kg)
    viscous_damping: float = 120.0              # Rod structural viscous damping C_visc (N*s/m)
    stroke_max: float = 0.25                    # Maximum rod travel stroke (m)
    endstop_stiffness: float = 5.0e7            # Hard stop stiffness K_stop (N/m)
    endstop_damping: float = 1.0e5              # Hard stop damping C_stop (N*s/m)
    # Pressure-dependent Stribeck friction parameters:
    coulomb_friction_base: float = 220.0        # F_c0 base Coulomb friction at 0 bar gauge (N)
    coulomb_pressure_alpha: float = 1.2e-4      # alpha_cp: Coulomb increase per Pa (~12 N / bar)
    static_friction_base: float = 480.0         # F_s0 base breakaway friction at 0 bar gauge (N)
    static_pressure_alpha: float = 2.4e-4       # alpha_sp: Breakaway increase per Pa (~24 N / bar)
    stribeck_velocity: float = 0.008            # v_s characteristic Stribeck velocity (m/s)
    stribeck_exponent: float = 1.5              # delta shape factor
    viscous_seal_base: float = 350.0            # sigma_v0 seal viscous friction (N*s/m)
    viscous_seal_alpha: float = 8.0e-5          # alpha_vp viscous increase per Pa
    velocity_transition_tol: float = 1.0e-4     # v_trans for continuous tanh regularization (m/s)


class MechanicsSubsystem:
    """
    Computes axial acceleration, velocity, displacement, and pressure-dependent seal friction.
    """

    def __init__(self, params: MechanicsParameters = MechanicsParameters()) -> None:
        self.params = params
        self.position = 0.0
        self.velocity = 0.0

    def reset(self, initial_position: float = 0.0, initial_velocity: float = 0.0) -> None:
        """Reset kinematic state variables."""
        self.position = initial_position
        self.velocity = initial_velocity

    def compute_seal_friction(self, gauge_pressure: float, velocity: float) -> float:
        """
        Compute nonlinear pressure-dependent Stribeck friction.
        """
        p_eff = max(0.0, gauge_pressure)

        f_c = self.params.coulomb_friction_base + self.params.coulomb_pressure_alpha * p_eff
        f_s = self.params.static_friction_base + self.params.static_pressure_alpha * p_eff
        sigma_v = self.params.viscous_seal_base + self.params.viscous_seal_alpha * p_eff

        # Stribeck decay term
        abs_v = abs(velocity)
        stribeck_factor = math.exp(-((abs_v / max(1.0e-6, self.params.stribeck_velocity)) ** self.params.stribeck_exponent))
        dry_friction = f_c + (f_s - f_c) * stribeck_factor

        # Regularized directional scaling via tanh
        directional_term = math.tanh(velocity / self.params.velocity_transition_tol)

        return dry_friction * directional_term + sigma_v * velocity

    def compute_endstop_force(self, position: float, velocity: float) -> float:
        """Compute structural restoring force when contacting travel limits."""
        f_stop = 0.0
        if position < 0.0:
            penetration = -position
            f_stop = self.params.endstop_stiffness * penetration - self.params.endstop_damping * velocity
        elif position > self.params.stroke_max:
            penetration = position - self.params.stroke_max
            f_stop = -(self.params.endstop_stiffness * penetration + self.params.endstop_damping * velocity)
        return f_stop

    def compute_acceleration(
        self,
        gauge_pressure: float,
        piston_area: float,
        axial_cutting_force: float,
        position: float,
        velocity: float,
    ) -> Tuple[float, float]:
        """
        Compute rod net acceleration and seal friction force.
        Returns: (acceleration, seal_friction_force)
        """
        f_hyd = gauge_pressure * piston_area
        f_fric = self.compute_seal_friction(gauge_pressure, velocity)
        f_visc = self.params.viscous_damping * velocity
        f_stop = self.compute_endstop_force(position, velocity)

        # Net axial force
        f_net = f_hyd - f_fric - f_visc - axial_cutting_force + f_stop

        acceleration = f_net / self.params.effective_mass
        return acceleration, f_fric
