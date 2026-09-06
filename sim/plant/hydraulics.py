"""
Hydraulic pusher model: centrifugal pump H-Q curve, compressibility, trapped pressure,
passive leakage, and displacement-flow coupling.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class HydraulicParameters:
    piston_area: float = 2.0e-3
    dead_volume: float = 1.5e-4
    atmospheric_pressure: float = 1.01325e5
    fluid_density: float = 870.0

    pure_bulk_modulus: float = 1.4e9
    entrained_air_fraction: float = 0.008
    adiabatic_index: float = 1.4
    casing_bulk_modulus: float = 2.0e10

    # Small passive unload paths. The pump cannot actively pull chamber pressure down.
    bypass_orifice_cd_a: float = 1.5e-10
    seal_leakage_coeff: float = 4.0e-15

    # Calibrated compact high-head centrifugal pump map:
    # delta_P = a0*w^2 - a1*w*Q - a2*Q^2
    pump_a0: float = 38.0
    pump_a1: float = 8.5e8
    pump_a2: float = 4.5e16


class HydraulicSubsystem:
    def __init__(
        self,
        params: HydraulicParameters = HydraulicParameters(),
    ) -> None:
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
            1.0 / self.params.pure_bulk_modulus
            + air_comp
            + 1.0 / self.params.casing_bulk_modulus
        )
        return 1.0 / inv_beta

    def pressure_to_shutoff_speed_rpm(self, pressure_bar_abs: float) -> float:
        """Quasi-static inverse pump map at Q≈0, useful as controller feedforward."""
        pressure_pa = pressure_bar_abs * 1.0e5
        delta_p = max(0.0, pressure_pa - self.params.atmospheric_pressure)
        if delta_p <= 0.0:
            return 0.0
        omega = math.sqrt(delta_p / max(1.0e-12, self.params.pump_a0))
        return omega * 30.0 / math.pi

    def compute_pump_flow(
        self,
        omega_pump: float,
        chamber_pressure: float,
    ) -> float:
        if omega_pump <= 0.0:
            return 0.0

        p_shutoff = self.params.pump_a0 * omega_pump**2
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
        return math.copysign(
            self.params.bypass_orifice_cd_a
            * math.sqrt(
                2.0 * abs(delta_p) / self.params.fluid_density
            ),
            delta_p,
        )

    def compute_seal_leakage(self, chamber_pressure: float) -> float:
        delta_p = chamber_pressure - self.params.atmospheric_pressure
        return max(0.0, self.params.seal_leakage_coeff * delta_p)

    def compute_pressure_derivative(
        self,
        pressure: float,
        rod_position: float,
        rod_velocity: float,
        omega_pump: float,
    ) -> Tuple[float, float, float, float]:
        """
        Fluid continuity:
            P_dot = beta_eff / V * (Qpump - Qbypass - Qleak - A*x_dot)
        """
        p_clamped = max(self.params.atmospheric_pressure, pressure)
        chamber_volume = (
            self.params.dead_volume
            + self.params.piston_area * max(0.0, rod_position)
        )

        q_pump = self.compute_pump_flow(omega_pump, p_clamped)
        q_bypass = self.compute_bypass_flow(p_clamped)
        q_leak = self.compute_seal_leakage(p_clamped)
        q_displacement = self.params.piston_area * rod_velocity

        beta_eff = self.effective_bulk_modulus(p_clamped)
        dp_dt = (
            beta_eff
            / chamber_volume
            * (q_pump - q_bypass - q_leak - q_displacement)
        )
        return dp_dt, q_pump, q_bypass, q_leak
