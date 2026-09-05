"""
Mechanistic Milling Mechanics: Cylindrical Cutter Engaging Inconel 718 Spherical Target.
Provides Level A (revolution-averaged) and Level B (tooth-resolved) fidelity models.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class CuttingParameters:
    """Geometrical and mechanistic cutting parameters for Inconel 718 milling."""
    cutter_radius: float = 0.010                # Tool radius R_bit (m, 20 mm diameter)
    flute_count: int = 4                        # Number of flutes Z
    helix_angle_deg: float = 30.0               # Flute helix angle beta (deg)
    target_sphere_radius: float = 0.050         # Workpiece ball radius R_sphere (m, 100 mm diameter)
    contact_start_pos: float = 0.080            # Rod position where initial contact occurs x_0 (m)
    # Mechanistic cutting force coefficients for Inconel 718 (identified / calibrated):
    k_tc: float = 3100.0e6                      # Tangential shear coefficient K_tc (N/m^2 = Pa)
    k_te: float = 65.0e3                        # Tangential edge coefficient K_te (N/m)
    k_rc: float = 1350.0e6                      # Radial shear coefficient K_rc (N/m^2)
    k_re: float = 45.0e3                        # Radial edge coefficient K_re (N/m)
    k_ac: float = 820.0e6                       # Axial shear coefficient K_ac (N/m^2)
    k_ae: float = 32.0e3                        # Axial edge coefficient K_ae (N/m)
    # Level A averaged model scaling factors:
    averaged_specific_energy: float = 3.2e9     # Specific cutting energy (J/m^3 = Pa)
    axial_thrust_coeff: float = 1.8e7           # Axial thrust force per contact area (N/m^2)
    rubbing_torque_coeff: float = 1200.0        # Edge rubbing torque per contact area (N*m/m^2)
    axial_damping: float = 800.0                # Feed velocity damping (N*s/m)
    # Level B runout eccentricity:
    runout_amplitude: float = 1.5e-5            # 15 microns tool runout


class MechanisticCuttingSubsystem:
    """
    Simulates cutting force, spindle torque, and material removal during milling.
    """

    def __init__(self, params: CuttingParameters = CuttingParameters()) -> None:
        self.params = params
        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.spindle_angle = 0.0

    def reset(self) -> None:
        """Reset cutting and workpiece crater states."""
        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.spindle_angle = 0.0

    def compute_engagement_geometry(self, rod_position: float) -> Tuple[float, float]:
        """
        Compute penetration depth d and projected frontal contact area A_contact
        between cylindrical cutter and target sphere.
        """
        if rod_position <= self.params.contact_start_pos:
            return 0.0, 0.0

        d = rod_position - self.params.contact_start_pos
        r_s = self.params.target_sphere_radius
        r_b = self.params.cutter_radius

        # Projected contact area: spherical cap intersection bounded by cutter radius
        # Cap surface area projection = pi * (2*R_s*d - d^2)
        if d < r_s:
            a_geom = math.pi * (2.0 * r_s * d - d * d)
        else:
            a_geom = math.pi * (r_s * r_s)

        # Bounded by flat cylindrical cutter face area
        a_max = math.pi * (r_b * r_b)
        a_contact = min(a_geom, a_max)

        return d, a_contact

    def step_level_a_averaged(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        """
        Level A (Control-Oriented): Revolution-averaged cutting forces.
        Returns: (axial_force, cutting_torque, mrr, chip_thickness)
        """
        d, a_contact = self.compute_engagement_geometry(rod_position)
        self.penetration_depth = d

        if d <= 0.0 or a_contact <= 0.0:
            return 0.0, 0.0, 0.0, 0.0

        # Positive feed advances cut into material
        v_feed = max(0.0, rod_velocity)
        omega = max(1.0, abs(spindle_speed))

        # Material Removal Rate: MRR = A_contact * v_feed
        mrr = a_contact * v_feed
        d_vol = mrr * dt
        self.cumulative_volume_removed += d_vol

        # Average chip thickness h_avg = v_feed / (Z * n_rev)
        n_rev = omega / (2.0 * math.pi)
        h_avg = v_feed / (self.params.flute_count * n_rev)

        # Spindle Cutting Torque: Power = Specific_Energy * MRR -> Torque = Power / omega
        t_cutting = (self.params.averaged_specific_energy * mrr) / omega + self.params.rubbing_torque_coeff * a_contact

        # Axial Reaction Force (WOB): Area thrust + dynamic feed resistance
        f_axial = self.params.axial_thrust_coeff * a_contact + self.params.axial_damping * v_feed

        return f_axial, t_cutting, mrr, h_avg

    def step_level_b_tooth_resolved(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        """
        Level B (Disturbance-Rich): Tooth-resolved forces with flute passing harmonics and runout.
        Returns: (axial_force, cutting_torque, mrr, chip_thickness)
        """
        d, a_contact = self.compute_engagement_geometry(rod_position)
        self.penetration_depth = d

        if d <= 0.0 or a_contact <= 0.0:
            return 0.0, 0.0, 0.0, 0.0

        v_feed = max(0.0, rod_velocity)
        omega = max(1.0, abs(spindle_speed))
        self.spindle_angle = (self.spindle_angle + omega * dt) % (2.0 * math.pi)

        z_flutes = self.params.flute_count
        r_b = self.params.cutter_radius
        n_rev = omega / (2.0 * math.pi)
        feed_per_tooth = v_feed / (z_flutes * n_rev)

        # Axial depth of cut slice
        axial_depth = min(d, r_b)
        dz = max(1.0e-4, axial_depth / 5.0)

        total_torque = 0.0
        total_axial_force = 0.0
        max_h = 0.0

        # Sum contributions over all flutes and axial slices
        for j in range(z_flutes):
            flute_angle = self.spindle_angle + j * (2.0 * math.pi / z_flutes)
            # Runout eccentricity
            r_eff = r_b + self.params.runout_amplitude * math.cos(j * (2.0 * math.pi / z_flutes))

            for z_idx in range(5):
                z_curr = (z_idx + 0.5) * dz
                helix_lag = (z_curr * math.tan(math.radians(self.params.helix_angle_deg))) / r_b
                phi = (flute_angle - helix_lag) % (2.0 * math.pi)

                # Active immersion window (approximate plunge entry: 0 to pi)
                if 0.0 < phi < math.pi:
                    h_inst = feed_per_tooth * math.sin(phi)
                    if h_inst > max_h:
                        max_h = h_inst

                    df_t = (self.params.k_tc * h_inst + self.params.k_te) * dz
                    df_a = (self.params.k_ac * h_inst + self.params.k_ae) * dz

                    total_torque += df_t * r_eff
                    total_axial_force += df_a

        mrr = a_contact * v_feed
        self.cumulative_volume_removed += mrr * dt

        # Add static rubbing and contact resistance
        total_axial_force += self.params.axial_thrust_coeff * a_contact * 0.4
        total_torque += self.params.rubbing_torque_coeff * a_contact * 0.4

        return total_axial_force, total_torque, mrr, max_h
