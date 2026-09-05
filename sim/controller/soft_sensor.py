"""
Dynamic Soft Weight-On-Bit (WOB) Observer.
Estimates axial cutting reaction force from hydraulic line pressure, seal friction, and rod kinematics.
"""

from dataclasses import dataclass
import math

from sim.common.contracts import ProcessVariables
from sim.common.interfaces import ISoftSensor
from sim.plant.mechanics import MechanicsParameters, MechanicsSubsystem


class SoftWOBObserver(ISoftSensor):
    """
    Fuses hydraulic pressure with estimated friction and rod inertia:
    F_wob_est = (P_hyd - P_0) * A_p - F_fric(P, v_est) - M_eff * a_est
    """

    def __init__(
        self,
        piston_area: float = 2.0e-3,
        effective_mass: float = 45.0,
        atmospheric_pressure: float = 1.01325e5,
        mech_params: MechanicsParameters = MechanicsParameters(),
    ) -> None:
        self.piston_area = piston_area
        self.effective_mass = effective_mass
        self.atmospheric_pressure = atmospheric_pressure
        self.friction_model = MechanicsSubsystem(mech_params)

        # Alpha-Beta-Gamma tracking filter states
        self.pos_est = 0.0
        self.vel_est = 0.0
        self.accel_est = 0.0
        self.wob_filtered = 0.0

        # Filter gains (critically damped tracking)
        self.alpha = 0.45
        self.beta = 0.08
        self.gamma = 0.005

    def reset(self) -> None:
        """Reset observer internal states."""
        self.pos_est = 0.0
        self.vel_est = 0.0
        self.accel_est = 0.0
        self.wob_filtered = 0.0

    def update(self, dt: float, sensors: ProcessVariables) -> float:
        """
        Update kinematic filter and compute instantaneous estimated WOB force.
        """
        # 1. Kinematic Alpha-Beta-Gamma state prediction
        pos_pred = self.pos_est + self.vel_est * dt + 0.5 * self.accel_est * (dt ** 2)
        vel_pred = self.vel_est + self.accel_est * dt

        # Innovation
        residual = sensors.rod_displacement - pos_pred

        # Correction
        self.pos_est = pos_pred + self.alpha * residual
        self.vel_est = vel_pred + (self.beta / dt) * residual
        self.accel_est = self.accel_est + (self.gamma / (0.5 * dt * dt)) * residual

        # 2. Estimate pressure-dependent seal friction
        gauge_p = max(0.0, sensors.pressure_hyd - self.atmospheric_pressure)
        f_fric_est = self.friction_model.compute_seal_friction(gauge_p, self.vel_est)

        # 3. Dynamic force balance
        f_hyd = gauge_p * self.piston_area
        f_inertia = self.effective_mass * self.accel_est
        f_wob_raw = f_hyd - f_fric_est - f_inertia

        # 4. Low-pass filter to reject measurement jitter
        alpha_lpf = dt / (0.015 + dt)
        self.wob_filtered += alpha_lpf * (max(0.0, f_wob_raw) - self.wob_filtered)

        return self.wob_filtered
