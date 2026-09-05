"""
Mechanistic Milling Mechanics: Cylindrical Cutter Engaging Inconel 718 Spherical Target.
Incorporates localized material hardness inclusions, non-ideal chip packing & clearing bursts,
and high-frequency engagement vibration/micro-chatter.
"""

from dataclasses import dataclass
import math
import random
from typing import Tuple


@dataclass
class CuttingParameters:
    """Geometrical and mechanistic cutting parameters for Inconel 718 milling."""
    cutter_radius: float = 0.010                # Tool radius R_bit (m, 20 mm diameter)
    flute_count: int = 4                        # Number of flutes Z
    helix_angle_deg: float = 30.0               # Flute helix angle beta (deg)
    target_sphere_radius: float = 0.050         # Workpiece ball radius R_sphere (m, 100 mm diameter)
    contact_start_pos: float = 0.015            # Rod position where initial contact occurs x_0 (15 mm)
    # Mechanistic cutting force coefficients for Inconel 718:
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
    Features realistic non-uniform material hardness spots, chip buildup surges, and vibration.
    """

    def __init__(self, params: CuttingParameters = CuttingParameters(), seed: int = 42) -> None:
        self.params = params
        self.seed = seed
        self.rng = random.Random(seed)

        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.spindle_angle = 0.0
        self.sim_time = 0.0

        # Physical disturbance generators:
        self.chip_packing_level = 0.0           # Chip packing accumulation (0.0 to 1.5)
        self.hard_spot_active = False
        self.hard_spot_timer = 0.0
        self.next_hard_spot_time = 1.5          # First hard inclusion occurs at ~1.5s
        self.vibration_phase = 0.0

    def reset(self) -> None:
        """Reset cutting and workpiece crater states."""
        self.rng = random.Random(self.seed)
        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.spindle_angle = 0.0
        self.sim_time = 0.0
        self.chip_packing_level = 0.0
        self.hard_spot_active = False
        self.hard_spot_timer = 0.0
        self.next_hard_spot_time = 1.5
        self.vibration_phase = 0.0

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
        if d < r_s:
            a_geom = math.pi * (2.0 * r_s * d - d * d)
        else:
            a_geom = math.pi * (r_s * r_s)

        a_max = math.pi * (r_b * r_b)
        a_contact = min(a_geom, a_max)

        return d, a_contact

    def _update_material_disturbances(self, dt: float, in_contact: bool, v_feed: float) -> Tuple[float, float]:
        """
        Simulate Inconel 718 hard precipitate inclusions and non-ideal chip clogging/bursts.
        Returns: (hardness_multiplier, chip_clog_torque_surge)
        """
        self.sim_time += dt

        if not in_contact:
            self.chip_packing_level = 0.0
            return 1.0, 0.0

        # 1. Hard Spot (NbC/TiC Carbide Precipitate Cluster)
        hard_mult = 1.0 + 0.08 * math.sin(19.2 * self.sim_time) + 0.05 * math.sin(7.3 * self.penetration_depth * 1000.0)

        if not self.hard_spot_active and self.sim_time >= self.next_hard_spot_time:
            self.hard_spot_active = True
            self.hard_spot_timer = 0.0

        if self.hard_spot_active:
            self.hard_spot_timer += dt
            # Intense localized hard spot surge (+65% hardness)
            hard_mult += 0.65
            if self.hard_spot_timer >= 0.20:    # Lasts 200 ms
                self.hard_spot_active = False
                self.hard_spot_timer = 0.0
                # Schedule next inclusion in 1.8 to 3.2 seconds
                self.next_hard_spot_time = self.sim_time + 1.8 + self.rng.uniform(0.0, 1.4)

        # 2. Non-Ideal Chip Removal / Flute Jamming & Burst Release
        if v_feed > 0:
            self.chip_packing_level += 2.2 * dt * (v_feed / 0.001)

        chip_clog_surge = 0.0
        if self.chip_packing_level > 0.8:
            # Flute gullet fills with sticky Inconel chips -> exponential torque surge
            chip_clog_surge = 1.4 * (self.chip_packing_level - 0.8)
            if self.chip_packing_level >= 1.5:
                # Sudden chip evacuation / burst release!
                self.chip_packing_level = 0.0

        return hard_mult, chip_clog_surge

    def step_level_a_averaged(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        """
        Level A (Control-Oriented): Revolution-averaged cutting forces with realistic disturbances.
        Returns: (axial_force, cutting_torque, mrr, chip_thickness)
        """
        d, a_contact = self.compute_engagement_geometry(rod_position)
        self.penetration_depth = d

        in_contact = (d > 0.0 and a_contact > 0.0)
        v_feed = max(0.0, rod_velocity)
        omega = max(1.0, abs(spindle_speed))

        hard_mult, clog_surge = self._update_material_disturbances(dt, in_contact, v_feed)

        if not in_contact:
            return 0.0, 0.0, 0.0, 0.0

        # Material Removal Rate: MRR = A_contact * v_feed
        mrr = a_contact * v_feed
        d_vol = mrr * dt
        self.cumulative_volume_removed += d_vol

        # Average chip thickness h_avg = v_feed / (Z * n_rev)
        n_rev = omega / (2.0 * math.pi)
        h_avg = v_feed / (self.params.flute_count * n_rev)

        # 3. Mechanical vibration & tooth-passing noise
        self.vibration_phase = (self.vibration_phase + self.params.flute_count * omega * dt) % (2.0 * math.pi)
        t_vib = 0.35 * math.sin(self.vibration_phase) + self.rng.gauss(0.0, 0.08)
        f_vib = 40.0 * math.cos(self.vibration_phase) + self.rng.gauss(0.0, 15.0)

        # Base cutting torque with hardness and chip clog scaling
        base_torque = (self.params.averaged_specific_energy * hard_mult * (1.0 + clog_surge) * mrr) / omega
        t_cutting = base_torque + self.params.rubbing_torque_coeff * a_contact * hard_mult + t_vib

        # Axial Reaction Force (WOB)
        base_axial = self.params.axial_thrust_coeff * hard_mult * (1.0 + 0.35 * clog_surge) * a_contact
        f_axial = base_axial + self.params.axial_damping * v_feed + f_vib

        return max(0.0, f_axial), max(0.0, t_cutting), mrr, h_avg

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

        in_contact = (d > 0.0 and a_contact > 0.0)
        v_feed = max(0.0, rod_velocity)
        omega = max(1.0, abs(spindle_speed))

        hard_mult, clog_surge = self._update_material_disturbances(dt, in_contact, v_feed)

        if not in_contact:
            return 0.0, 0.0, 0.0, 0.0

        self.spindle_angle = (self.spindle_angle + omega * dt) % (2.0 * math.pi)

        z_flutes = self.params.flute_count
        r_b = self.params.cutter_radius
        n_rev = omega / (2.0 * math.pi)
        feed_per_tooth = v_feed / (z_flutes * n_rev)

        axial_depth = min(d, r_b)
        dz = max(1.0e-4, axial_depth / 5.0)

        total_torque = 0.0
        total_axial_force = 0.0
        max_h = 0.0

        for j in range(z_flutes):
            flute_angle = self.spindle_angle + j * (2.0 * math.pi / z_flutes)
            r_eff = r_b + self.params.runout_amplitude * math.cos(j * (2.0 * math.pi / z_flutes))

            for z_idx in range(5):
                z_curr = (z_idx + 0.5) * dz
                helix_lag = (z_curr * math.tan(math.radians(self.params.helix_angle_deg))) / r_b
                phi = (flute_angle - helix_lag) % (2.0 * math.pi)

                if 0.0 < phi < math.pi:
                    h_inst = feed_per_tooth * math.sin(phi)
                    if h_inst > max_h:
                        max_h = h_inst

                    df_t = (self.params.k_tc * hard_mult * (1.0 + clog_surge) * h_inst + self.params.k_te) * dz
                    df_a = (self.params.k_ac * hard_mult * h_inst + self.params.k_ae) * dz

                    total_torque += df_t * r_eff
                    total_axial_force += df_a

        mrr = a_contact * v_feed
        self.cumulative_volume_removed += mrr * dt

        total_axial_force += self.params.axial_thrust_coeff * hard_mult * a_contact * 0.4
        total_torque += self.params.rubbing_torque_coeff * a_contact * 0.4

        return max(0.0, total_axial_force), max(0.0, total_torque), mrr, max_h
