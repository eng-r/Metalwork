"""
PMSM Drives Model: FOC Iq Torque Derivation, Rotor Dynamics, and Resolver Soft-Sensing.
"""

from dataclasses import dataclass
import math
import random
from typing import Tuple


@dataclass
class PMSMParameters:
    """Electrical and mechanical parameters for PMSM drives."""
    pole_pairs: int = 4                         # Number of magnetic pole pairs p
    flux_linkage: float = 0.055                 # Permanent magnet flux linkage lambda_m (Wb)
    stator_resistance: float = 0.45             # Phase resistance R_s (Ohm)
    q_inductance: float = 1.2e-3                # Q-axis inductance L_q (H)
    d_inductance: float = 1.2e-3                # D-axis inductance L_d (H, SPMSM)
    rotor_inertia: float = 0.0035               # Rotor & tool holder inertia J_m (kg*m^2)
    friction_damping: float = 0.0015            # Mechanical viscous damping B_m (N*m*s/rad)
    current_loop_tau: float = 0.0005            # FOC current loop equivalent time constant (s)
    speed_loop_kp: float = 0.25                 # Embedded speed loop proportional gain
    speed_loop_ki: float = 2.5                  # Embedded speed loop integral gain
    max_current_iq: float = 35.0                # Current limit I_q,max (A)
    # Resolver parameters:
    resolver_bits: int = 14                     # 14-bit quantization (16384 counts/rev)
    resolver_noise_sigma: float = 0.0005        # Angle noise standard deviation (rad)
    iq_filter_tau: float = 0.005                # 5 ms low-pass filter on Iq for torque observer


class PMSMSpindleDrive:
    """
    Simulates the cutting spindle PMSM motor, FOC torque generation, and resolver feedback.
    """

    def __init__(self, params: PMSMParameters = PMSMParameters(), seed: int = 42) -> None:
        self.params = params
        self.seed = seed
        self.rng = random.Random(seed)
        self.mechanical_speed = 0.0             # True rotor speed (rad/s)
        self.rotor_angle = 0.0                  # True electrical angle (rad)
        self.iq_current = 0.0                   # Quadrature current (A)
        self.filtered_iq = 0.0                  # Filtered Iq for torque estimation (A)
        self.resolver_speed = 0.0               # Estimated speed from resolver (rad/s)
        self.speed_integrator = 0.0             # Speed loop integrator state

        # Dynamic load-torque observer states. Without subtracting J*dω/dt the
        # previous Iq observer interpreted motor acceleration torque as ToB,
        # causing false contact/overload detection during spindle speed changes.
        self.prev_resolver_speed = 0.0
        self.accel_est_filtered = 0.0
        self.load_torque_est_filtered = 0.0

    def reset(self, initial_speed_rads: float = 0.0) -> None:
        """Reset drive states."""
        self.rng = random.Random(self.seed)
        self.mechanical_speed = initial_speed_rads
        self.rotor_angle = 0.0
        self.iq_current = 0.0
        self.filtered_iq = 0.0
        self.resolver_speed = initial_speed_rads
        self.speed_integrator = 0.0
        self.prev_resolver_speed = initial_speed_rads
        self.accel_est_filtered = 0.0
        self.load_torque_est_filtered = 0.0

    def compute_electromagnetic_torque(self, iq: float) -> float:
        """Standard SPMSM torque: T_e = 1.5 * p * lambda_m * i_q."""
        return 1.5 * self.params.pole_pairs * self.params.flux_linkage * iq

    def step(
        self,
        dt: float,
        speed_command_rads: float,
        load_torque: float,
    ) -> Tuple[float, float, float]:
        """
        Step drive dynamics by dt.
        Returns: (mechanical_speed, resolver_speed, estimated_load_torque)
        """
        # 1. Resolver angle emulation with quantization & noise
        counts = 2 ** self.params.resolver_bits
        q_angle = (2.0 * math.pi) / counts
        quantized_angle = math.floor(self.rotor_angle / q_angle) * q_angle
        noisy_angle = quantized_angle + self.rng.gauss(0.0, self.params.resolver_noise_sigma)

        # 2. Embedded speed controller generating Iq_ref
        speed_err = speed_command_rads - self.resolver_speed
        self.speed_integrator += speed_err * dt
        # Anti-windup clamping on speed integrator
        self.speed_integrator = max(-10.0, min(10.0, self.speed_integrator))

        iq_ref = self.params.speed_loop_kp * speed_err + self.params.speed_loop_ki * self.speed_integrator
        iq_ref = max(-self.params.max_current_iq, min(self.params.max_current_iq, iq_ref))

        # 3. FOC closed-loop current dynamics (first-order lag)
        alpha_i = dt / (self.params.current_loop_tau + dt)
        self.iq_current += alpha_i * (iq_ref - self.iq_current)

        # 4. Electromagnetic torque and rotor acceleration
        t_em = self.compute_electromagnetic_torque(self.iq_current)
        acceleration = (t_em - load_torque - self.params.friction_damping * self.mechanical_speed) / self.params.rotor_inertia

        self.mechanical_speed = max(0.0, self.mechanical_speed + acceleration * dt)
        self.rotor_angle = (self.rotor_angle + self.params.pole_pairs * self.mechanical_speed * dt) % (2.0 * math.pi)

        # 5. Filtered resolver speed (RDC tracking observer emulation)
        alpha_res = dt / (0.002 + dt)
        self.resolver_speed += alpha_res * (self.mechanical_speed - self.resolver_speed)

        # 6. Filtered Iq and dynamic load-torque observer.
        alpha_iq = dt / (self.params.iq_filter_tau + dt)
        self.filtered_iq += alpha_iq * (self.iq_current - self.filtered_iq)

        raw_accel = (
            self.resolver_speed - self.prev_resolver_speed
        ) / max(dt, 1.0e-9)
        self.prev_resolver_speed = self.resolver_speed

        alpha_accel = dt / (0.025 + dt)
        self.accel_est_filtered += alpha_accel * (
            raw_accel - self.accel_est_filtered
        )

        # Rotor balance:
        #   J*dω/dt = T_em - T_load - B*ω
        # therefore:
        #   T_load = T_em - B*ω - J*dω/dt
        t_em_est = self.compute_electromagnetic_torque(self.filtered_iq)
        torque_load_raw = (
            t_em_est
            - self.params.friction_damping * self.resolver_speed
            - self.params.rotor_inertia * self.accel_est_filtered
        )
        torque_load_raw = max(0.0, torque_load_raw)

        alpha_load = dt / (0.018 + dt)
        self.load_torque_est_filtered += alpha_load * (
            torque_load_raw - self.load_torque_est_filtered
        )

        return (
            self.mechanical_speed,
            self.resolver_speed,
            max(0.0, self.load_torque_est_filtered),
        )


class PMSMPumpDrive:
    """
    Simulates the centrifugal pump PMSM motor drive.
    """

    def __init__(
        self,
        inertia: float = 0.002,
        tau_speed: float = 0.075,
        max_accel_rpm_s: float = 12000.0,
    ) -> None:
        self.inertia = inertia
        self.tau_speed = tau_speed
        self.max_accel_rads_s2 = max_accel_rpm_s * math.pi / 30.0
        self.mechanical_speed = 0.0
        self.resolver_speed = 0.0

    def reset(self, initial_speed_rads: float = 0.0) -> None:
        self.mechanical_speed = initial_speed_rads
        self.resolver_speed = initial_speed_rads

    def step(self, dt: float, speed_command_rads: float) -> float:
        """Smooth speed-servo response with physical acceleration saturation."""
        max_rads = 6000.0 * math.pi / 30.0
        cmd_clamped = max(0.0, min(max_rads, speed_command_rads))

        # Previous implementation first clipped a position-like delta and then
        # multiplied it by a first-order alpha. That double attenuation produces
        # visually perfect triangular ramps. Here the first-order law produces an
        # acceleration request, which is smoothly saturated once.
        accel_request = (
            cmd_clamped - self.mechanical_speed
        ) / max(1.0e-4, self.tau_speed)
        accel = self.max_accel_rads_s2 * math.tanh(
            accel_request / max(1.0e-6, self.max_accel_rads_s2)
        )
        self.mechanical_speed += accel * dt
        self.mechanical_speed = max(0.0, min(max_rads, self.mechanical_speed))

        alpha_res = dt / (0.002 + dt)
        self.resolver_speed += alpha_res * (
            self.mechanical_speed - self.resolver_speed
        )
        return self.resolver_speed
