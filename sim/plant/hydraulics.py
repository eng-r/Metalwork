"""
Hydraulic pusher model: centrifugal pump H-Q curve, compressibility, trapped
pressure, passive leakage, and displacement-flow coupling.

The chamber deliberately has no active suction/unload actuator. Reducing pump
speed can stop adding flow, but pressure falls only through passive leakage/
bypass and, importantly, by rod advance as cutting removes material.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class HydraulicParameters:
    """Hydraulic circuit physical parameters."""

    piston_area: float = 2.0e-3
    dead_volume: float = 1.5e-4
    atmospheric_pressure: float = 1.01325e5
    fluid_density: float = 870.0

    pure_bulk_modulus: float = 1.4e9
    entrained_air_fraction: float = 0.008
    adiabatic_index: float = 1.4
    casing_bulk_modulus: float = 2.0e10

    # Intentionally small passive unload paths. The plant is asymmetric:
    # pump-off does not actively pull pressure down.
    bypass_orifice_cd_a: float = 1.5e-10
    seal_leakage_coeff: float = 4.0e-15

    # Centrifugal pump head-flow curve:
    #   delta_P = a0*w^2 - a1*w*Q - a2*Q^2
    #
    # a0 preserves ~66 bar shutoff head at 4000 rpm. a1/a2 are calibrated for
    # a small high-head hydraulic pusher: a few mL/s, not tens of L/min.
    pump_a0: float = 38.0
    pump_a1: float = 8.5e8
    pump_a2: float = 4.5e16


class HydraulicSubsystem:
    """
    Continuous hydraulic chamber model.

    Positive rod velocity increases chamber volume and therefore unloads
    pressure. This is the mechanism by which continued cutting can relax a
    trapped hydraulic load even when the pump cannot reverse.
    """

    def __init__(self, params: HydraulicParameters = HydraulicParameters()) -> None:
        self.params = params
        self.pressure = params.atmospheric_pressure

    def reset(self, initial_pressure: float = 1.01325e5) -> None:
        self.pressure = max(self.params.atmospheric_pressure, initial_pressure)

    def effective_bulk_modulus(self, pressure: float) -> float:
        p_clamped = max(1.0e4, pressure)
        p0 = self.params.atmospheric_pressure
        alpha0 = self.params.entrained_air_fraction
        gamma = self.params.adiabatic_index

        air_comp = (
            alpha0 * (p0 / p_clamped) ** (1.0 / gamma)
        ) / p_clamped
        inv_beta = (
            (1.0 / self.params.pure_bulk_modulus)
            + air_comp
            + (1.0 / self.params.casing_bulk_modulus)
        )
        return 1.0 / inv_beta

    def compute_pump_flow(
        self,
        omega_pump: float,
        chamber_pressure: float,
    ) -> float:
        """
        Solve the positive-flow operating point on the pump H-Q curve.

        A check valve / non-reversible pump path is implicit: Q_pump >= 0.
        """
        if omega_pump <= 0.0:
            return 0.0

        p_shutoff = self.params.pump_a0 * (omega_pump ** 2)
        delta_p = chamber_pressure - self.params.atmospheric_pressure

        if p_shutoff <= delta_p:
            return 0.0

        a = self.params.pump_a2
        b = self.params.pump_a1 * omega_pump
        c = delta_p - p_shutoff

        discriminant = b * b - 4.0 * a * c
        if discriminant <= 0.0:
            return 0.0

        q = (-b + math.sqrt(discriminant)) / (2.0 * a)
        return max(0.0, q)

    def compute_bypass_flow(self, chamber_pressure: float) -> float:
        delta_p = chamber_pressure - self.params.atmospheric_pressure
        if abs(delta_p) < 1.0:
            return 0.0

        sign = 1.0 if delta_p > 0.0 else -1.0
        return (
            sign
            * self.params.bypass_orifice_cd_a
            * math.sqrt(
                (2.0 / self.params.fluid_density) * abs(delta_p)
            )
        )

    def compute_seal_leakage(self, chamber_pressure: float) -> float:
        delta_p = chamber_pressure - self.params.atmospheric_pressure
        return max(
            0.0,
            self.params.seal_leakage_coeff * delta_p,
        )

    def compute_pressure_derivative(
        self,
        pressure: float,
        rod_position: float,
        rod_velocity: float,
        omega_pump: float,
    ) -> Tuple[float, float, float, float]:
        """
        Continuity:
          dP/dt = beta_eff/V * (Qpump - Qbypass - Qleak - A_p*x_dot)
        """
        p_clamped = max(
            self.params.atmospheric_pressure,
            pressure,
        )
        chamber_volume = (
            self.params.dead_volume
            + self.params.piston_area * max(0.0, rod_position)
        )

        q_pump = self.compute_pump_flow(
            omega_pump,
            p_clamped,
        )
        q_bypass = self.compute_bypass_flow(p_clamped)
        q_leak = self.compute_seal_leakage(p_clamped)
        q_displacement = self.params.piston_area * rod_velocity

        beta_eff = self.effective_bulk_modulus(p_clamped)
        net_flow = (
            q_pump
            - q_bypass
            - q_leak
            - q_displacement
        )

        dp_dt = (beta_eff / chamber_volume) * net_flow
        return dp_dt, q_pump, q_bypass, q_leak
