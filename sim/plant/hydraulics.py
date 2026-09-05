"""
Hydraulic Pusher Model: Centrifugal Pump H-Q Curves, Bulk Modulus, and Pressure Dynamics.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class HydraulicParameters:
    """Hydraulic circuit physical parameters."""
    piston_area: float = 2.0e-3                 # Piston effective area A_p (m^2, approx 50 mm bore)
    dead_volume: float = 1.5e-4                 # Dead chamber volume V_0 (m^3, approx 150 ml)
    atmospheric_pressure: float = 1.01325e5     # P_0 (Pa)
    fluid_density: float = 870.0                # Mineral hydraulic oil density rho (kg/m^3)
    pure_bulk_modulus: float = 1.4e9            # beta_oil (Pa)
    entrained_air_fraction: float = 0.008       # alpha_air,0 at atmospheric pressure
    adiabatic_index: float = 1.4                # gamma
    casing_bulk_modulus: float = 2.0e10         # K_wall structural elastance (Pa)
    bypass_orifice_cd_a: float = 6.0e-10         # C_bypass stiff calibrated orifice area (m^2)
    seal_leakage_coeff: float = 2.0e-14         # C_leak parasitic seal conductance (m^3/(s*Pa))
    # Centrifugal pump head-flow coefficients: delta_P = a0*w^2 - a1*w*Q - a2*Q^2
    pump_a0: float = 0.42                       # Pa / (rad/s)^2 (gives ~67 bar at 4000 RPM)
    pump_a1: float = 8.5e4                      # Pa / ((rad/s) * (m^3/s))
    pump_a2: float = 4.5e9                      # Pa / (m^3/s)^2


class HydraulicSubsystem:
    """
    Simulates continuous hydraulic chamber pressure and flow rates.
    Enforces check-valve non-reversibility: centrifugal pump cannot suck fluid back.
    """

    def __init__(self, params: HydraulicParameters = HydraulicParameters()) -> None:
        self.params = params
        self.pressure = params.atmospheric_pressure

    def reset(self, initial_pressure: float = 1.01325e5) -> None:
        """Reset chamber pressure to initial state."""
        self.pressure = max(self.params.atmospheric_pressure, initial_pressure)

    def effective_bulk_modulus(self, pressure: float) -> float:
        """
        Calculate effective bulk modulus beta_eff(P) accounting for entrained air and wall compliance.
        """
        p_clamped = max(1.0e4, pressure)
        p0 = self.params.atmospheric_pressure
        alpha0 = self.params.entrained_air_fraction
        gamma = self.params.adiabatic_index

        # Air compliance term
        air_comp = (alpha0 * (p0 / p_clamped) ** (1.0 / gamma)) / p_clamped
        # Total fluid compliance
        inv_beta = (1.0 / self.params.pure_bulk_modulus) + air_comp + (1.0 / self.params.casing_bulk_modulus)
        return 1.0 / inv_beta

    def compute_pump_flow(self, omega_pump: float, chamber_pressure: float) -> float:
        """
        Solve operating point flow on pump H-Q curve given pump speed and back-pressure.
        Enforces non-reversibility: Q_pump >= 0.
        """
        if omega_pump <= 0.0:
            return 0.0

        p_shutoff = self.params.pump_a0 * (omega_pump ** 2)
        delta_p = chamber_pressure - self.params.atmospheric_pressure

        if p_shutoff <= delta_p:
            # Back-pressure exceeds centrifugal shutoff head -> zero flow
            return 0.0

        # Solve a2*Q^2 + a1*w*Q + (delta_p - a0*w^2) = 0
        a = self.params.pump_a2
        b = self.params.pump_a1 * omega_pump
        c = delta_p - p_shutoff

        discriminant = b * b - 4.0 * a * c
        if discriminant <= 0.0:
            return 0.0

        q = (-b + math.sqrt(discriminant)) / (2.0 * a)
        return max(0.0, q)

    def compute_bypass_flow(self, chamber_pressure: float) -> float:
        """Flow through stiff calibrated bypass orifice."""
        delta_p = chamber_pressure - self.params.atmospheric_pressure
        if abs(delta_p) < 1.0:
            return 0.0
        sign = 1.0 if delta_p > 0 else -1.0
        return sign * self.params.bypass_orifice_cd_a * math.sqrt(
            (2.0 / self.params.fluid_density) * abs(delta_p)
        )

    def compute_seal_leakage(self, chamber_pressure: float) -> float:
        """Laminar parasitic seal leakage."""
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
        Compute dP/dt and instantaneous component flows.
        Returns: (dP_dt, Q_pump, Q_bypass, Q_leak)
        """
        p_clamped = max(self.params.atmospheric_pressure, pressure)
        chamber_volume = self.params.dead_volume + self.params.piston_area * max(0.0, rod_position)

        q_pump = self.compute_pump_flow(omega_pump, p_clamped)
        q_bypass = self.compute_bypass_flow(p_clamped)
        q_leak = self.compute_seal_leakage(p_clamped)
        q_disp = self.params.piston_area * rod_velocity

        beta_eff = self.effective_bulk_modulus(p_clamped)
        net_flow = q_pump - q_bypass - q_leak - q_disp

        dp_dt = (beta_eff / chamber_volume) * net_flow
        return dp_dt, q_pump, q_bypass, q_leak
