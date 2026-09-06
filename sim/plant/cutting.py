"""
Mechanistic milling mechanics for a cylindrical cutter engaging a nickel-alloy spherical target.

The control-oriented model deliberately keeps the geometry compact, but the disturbance
model is stateful and physical: a deterministic spatial hardness map, chip packing/jam
cycles, a damped structural vibration mode, and cutter dwell clearing all feed the same
cutting-force equations. No display-only noise is injected.
"""

from dataclasses import dataclass
import math
import random
from typing import List, Tuple


@dataclass
class CuttingParameters:
    """Geometrical, cutting, and disturbance parameters."""
    cutter_radius: float = 0.010
    flute_count: int = 4
    helix_angle_deg: float = 30.0
    target_sphere_radius: float = 0.050
    contact_start_pos: float = 0.015

    # Mechanistic cutting coefficients. Treat as calibration placeholders.
    k_tc: float = 3100.0e6
    k_te: float = 65.0e3
    k_rc: float = 1350.0e6
    k_re: float = 45.0e3
    k_ac: float = 820.0e6
    k_ae: float = 32.0e3

    # Level-A control-oriented coefficients.
    averaged_specific_energy: float = 3.2e9
    axial_thrust_coeff: float = 1.8e7
    rubbing_torque_coeff: float = 13000.0    # calibrated edge/engagement torque term
    axial_damping: float = 800.0

    # Unilateral contact compliance. Once the cylindrical face is fully
    # engaged, additional virtual penetration represents elastic compression /
    # impossible overlap, not more machinable volume. A penalty force prevents
    # the rod from accumulating centimetres of uncut "engagement backlog".
    contact_overtravel_stiffness: float = 8.0e6
    contact_overtravel_damping: float = 1200.0
    contact_full_face_margin: float = 5.0e-5

    # Level-B runout.
    runout_amplitude: float = 1.5e-5

    # Slow physical material-removal model.
    # ~0.08 mm/min gives about 16 h for 3 in and 21 h for 4 in at nominal conditions.
    # Only material geometry is time-compressed for the interactive demo.
    nominal_spindle_rpm: float = 3500.0
    nominal_physical_rop_mm_min: float = 0.08
    min_physical_rop_mm_min: float = 0.01
    max_physical_rop_mm_min: float = 0.16
    demo_acceleration: float = 120.0
    engagement_scale: float = 0.00025

    # Spatial material non-uniformity.
    material_texture_amplitude: float = 0.07
    hard_spot_count: int = 7
    hard_spot_depth_span: float = 0.014
    hard_spot_width_min: float = 0.00012
    hard_spot_width_max: float = 0.00038
    hard_spot_gain_min: float = 0.25
    hard_spot_gain_max: float = 0.70

    # Chip transport / intermittent flute loading.
    chip_generation_gain: float = 2.20
    chip_evacuation_rate: float = 0.26
    chip_jam_threshold_min: float = 0.58
    chip_jam_threshold_max: float = 0.82
    chip_jam_duration_min: float = 0.10
    chip_jam_duration_max: float = 0.28
    chip_jam_torque_gain: float = 0.95

    # Structural vibration / micro-chatter mode.
    vibration_frequency_hz: float = 58.0
    vibration_damping_ratio: float = 0.16
    vibration_torque_fraction: float = 0.16
    vibration_force_fraction: float = 0.08


class MechanisticCuttingSubsystem:
    """
    Cutting subsystem with deterministic disturbance realization.

    `surface_recession_depth` is the local crater advance. The instantaneous
    interference/engagement is the rod's geometric penetration minus that removed
    depth. This lets load relax while the spindle keeps cutting after pump reduction.
    """

    def __init__(self, params: CuttingParameters = CuttingParameters(), seed: int = 42) -> None:
        self.params = params
        self.seed = seed
        self.rng = random.Random(seed)

        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.surface_recession_depth = 0.0
        self.engagement_depth = 0.0
        self.spindle_angle = 0.0
        self.sim_time = 0.0
        self.physical_rop_m_s = 0.0
        self.equivalent_process_time = 0.0

        self.hardness_multiplier = 1.0
        self.chip_packing_level = 0.0
        self.chip_jam_active = False
        self.chip_jam_timer = 0.0
        self.chip_release_timer = 0.0
        self.next_chip_jam_threshold = 0.70

        self.vibration_state = 0.0
        self.vibration_velocity = 0.0
        self.vibration_torque = 0.0
        self.vibration_force = 0.0
        self.disturbance_event = "FREE"

        self._hard_spots: List[Tuple[float, float, float]] = []
        self._reset_disturbance_map()

    def _reset_disturbance_map(self) -> None:
        self._hard_spots = []
        count = max(1, self.params.hard_spot_count)
        span = max(1.0e-4, self.params.hard_spot_depth_span)
        for i in range(count):
            nominal = ((i + 0.65) / count) * span
            spacing = span / count
            center = nominal + self.rng.uniform(-0.18, 0.18) * spacing
            width = self.rng.uniform(self.params.hard_spot_width_min, self.params.hard_spot_width_max)
            gain = self.rng.uniform(self.params.hard_spot_gain_min, self.params.hard_spot_gain_max)
            self._hard_spots.append((max(0.0, center), width, gain))
        self._hard_spots.sort(key=lambda item: item[0])
        self.next_chip_jam_threshold = self.rng.uniform(
            self.params.chip_jam_threshold_min,
            self.params.chip_jam_threshold_max,
        )

    def reset(self) -> None:
        self.rng = random.Random(self.seed)
        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.surface_recession_depth = 0.0
        self.engagement_depth = 0.0
        self.spindle_angle = 0.0
        self.sim_time = 0.0
        self.physical_rop_m_s = 0.0
        self.equivalent_process_time = 0.0
        self.hardness_multiplier = 1.0
        self.chip_packing_level = 0.0
        self.chip_jam_active = False
        self.chip_jam_timer = 0.0
        self.chip_release_timer = 0.0
        self.vibration_state = 0.0
        self.vibration_velocity = 0.0
        self.vibration_torque = 0.0
        self.vibration_force = 0.0
        self.disturbance_event = "FREE"
        self._reset_disturbance_map()

    def _contact_area_from_depth(self, depth: float) -> float:
        if depth <= 0.0:
            return 0.0
        r_s = self.params.target_sphere_radius
        r_b = self.params.cutter_radius
        d = min(depth, 2.0 * r_s)
        if d < r_s:
            a_geom = math.pi * max(0.0, 2.0 * r_s * d - d * d)
        else:
            a_geom = math.pi * r_s * r_s
        return min(a_geom, math.pi * r_b * r_b)

    def compute_engagement_geometry(self, rod_position: float) -> Tuple[float, float]:
        gross_depth = max(0.0, rod_position - self.params.contact_start_pos)
        engagement = max(0.0, gross_depth - self.surface_recession_depth)
        return engagement, self._contact_area_from_depth(engagement)

    def _full_face_engagement_depth(self) -> float:
        r_s = self.params.target_sphere_radius
        r_b = min(self.params.cutter_radius, r_s * 0.999999)
        return r_s - math.sqrt(max(0.0, r_s * r_s - r_b * r_b))

    def _contact_overtravel_force(
        self,
        engagement_depth: float,
        rod_velocity: float,
    ) -> float:
        onset = (
            self._full_face_engagement_depth()
            + self.params.contact_full_face_margin
        )
        excess = max(0.0, engagement_depth - onset)
        if excess <= 0.0:
            return 0.0

        return (
            self.params.contact_overtravel_stiffness * excess
            + self.params.contact_overtravel_damping
            * max(0.0, rod_velocity)
        )

    def _material_hardness(self, cut_front_depth: float) -> float:
        depth_mm = cut_front_depth * 1000.0
        texture = self.params.material_texture_amplitude * (
            0.62 * math.sin(2.0 * math.pi * depth_mm / 1.55 + 0.35)
            + 0.38 * math.sin(2.0 * math.pi * depth_mm / 0.63 + 1.2)
        )
        inclusions = 0.0
        for center, width, gain in self._hard_spots:
            z = (cut_front_depth - center) / max(width, 1.0e-6)
            inclusions += gain * math.exp(-0.5 * z * z)
        return max(0.82, 1.0 + texture + inclusions)

    def _update_surface_removal(
        self,
        dt: float,
        gross_depth: float,
        engagement_depth: float,
        contact_area: float,
        rod_velocity: float,
        spindle_speed: float,
        hardness: float,
    ) -> Tuple[float, float]:
        """
        Advance crater geometry at accelerated demo time while preserving a
        physically slow ROP for cutting power, telemetry and reporting.

        Returns:
            physical_mrr [m^3/s], physical_rop [m/s]
        """
        if engagement_depth <= 0.0 or contact_area <= 0.0 or abs(spindle_speed) < 5.0:
            self.physical_rop_m_s = 0.0
            return 0.0, 0.0

        p = self.params
        nominal_rop = p.nominal_physical_rop_mm_min / (1000.0 * 60.0)
        min_rop = p.min_physical_rop_mm_min / (1000.0 * 60.0)
        max_rop = p.max_physical_rop_mm_min / (1000.0 * 60.0)

        speed_ref = p.nominal_spindle_rpm * math.pi / 30.0
        speed_factor = max(
            0.15,
            min(1.25, abs(spindle_speed) / max(speed_ref, 1.0)),
        )

        # More interference raises chip engagement, but saturates quickly; feed
        # velocity only trims the achievable ROP rather than dictating it.
        engagement_factor = 0.35 + 0.75 * math.tanh(
            engagement_depth / max(1.0e-6, p.engagement_scale)
        )
        feed_trim = 1.0 + min(
            0.20,
            0.10 * max(0.0, rod_velocity) / 0.001,
        )

        chip_efficiency = 1.0 / (1.0 + 0.95 * self.chip_packing_level)
        if self.chip_jam_active:
            chip_efficiency *= 0.38
        hardness_efficiency = 1.0 / max(0.80, hardness ** 1.20)

        physical_rop = (
            nominal_rop
            * speed_factor
            * engagement_factor
            * feed_trim
            * chip_efficiency
            * hardness_efficiency
        )
        physical_rop = max(min_rop, min(max_rop, physical_rop))
        self.physical_rop_m_s = physical_rop

        # Time compression is intentionally restricted to the slow geometry state.
        demo_surface_rate = physical_rop * max(1.0, p.demo_acceleration)
        clearable = max(0.0, gross_depth - self.surface_recession_depth)
        delta_clear = min(clearable, demo_surface_rate * dt)
        self.surface_recession_depth += delta_clear
        self.equivalent_process_time += dt * max(1.0, p.demo_acceleration)

        physical_mrr = contact_area * physical_rop
        equivalent_removed_volume = contact_area * delta_clear
        self.cumulative_volume_removed += equivalent_removed_volume
        return physical_mrr, physical_rop

    def _update_chip_transport(
        self,
        dt: float,
        mrr: float,
        spindle_speed: float,
        engagement_depth: float,
        hardness: float,
    ) -> Tuple[float, bool, bool]:
        area_max = math.pi * self.params.cutter_radius ** 2
        nominal_surface_rate = (
            self.params.nominal_physical_rop_mm_min / (1000.0 * 60.0)
        )
        reference_mrr = max(1.0e-15, area_max * nominal_surface_rate)
        mrr_norm = min(3.0, mrr / reference_mrr)
        speed_ref = self.params.nominal_spindle_rpm * math.pi / 30.0
        speed_factor = min(1.5, max(0.0, abs(spindle_speed) / max(speed_ref, 1.0)))
        engagement_ratio = min(1.0, engagement_depth / max(1.0e-6, self.params.cutter_radius))
        generation = self.params.chip_generation_gain * mrr_norm * max(0.9, hardness)
        evacuation = self.params.chip_evacuation_rate * speed_factor * (1.0 - 0.42 * engagement_ratio)
        jam_started = False
        release_started = False
        if self.chip_release_timer > 0.0:
            self.chip_release_timer = max(0.0, self.chip_release_timer - dt)
        if self.chip_jam_active:
            self.chip_jam_timer -= dt
            self.chip_packing_level = min(
                1.35,
                self.chip_packing_level + dt * (0.35 * generation - 0.12 * evacuation),
            )
            if self.chip_jam_timer <= 0.0:
                self.chip_jam_active = False
                self.chip_release_timer = 0.12
                self.chip_packing_level *= self.rng.uniform(0.16, 0.32)
                self.next_chip_jam_threshold = self.rng.uniform(
                    self.params.chip_jam_threshold_min,
                    self.params.chip_jam_threshold_max,
                )
                release_started = True
        else:
            self.chip_packing_level += dt * (generation - evacuation)
            self.chip_packing_level = min(1.25, max(0.0, self.chip_packing_level))
            if mrr_norm > 0.12 and self.chip_packing_level >= self.next_chip_jam_threshold:
                self.chip_jam_active = True
                self.chip_jam_timer = self.rng.uniform(
                    self.params.chip_jam_duration_min,
                    self.params.chip_jam_duration_max,
                )
                jam_started = True
        chip_multiplier = 1.0 + 0.42 * (self.chip_packing_level ** 1.6)
        if self.chip_jam_active:
            chip_multiplier += self.params.chip_jam_torque_gain * (
                0.55 + 0.45 * min(1.0, self.chip_packing_level)
            )
        elif self.chip_release_timer > 0.0:
            chip_multiplier *= 0.78
        return max(0.65, chip_multiplier), jam_started, release_started

    def _update_vibration(
        self,
        dt: float,
        spindle_speed: float,
        base_torque: float,
        base_force: float,
        hardness: float,
        chip_multiplier: float,
        jam_started: bool,
        release_started: bool,
    ) -> Tuple[float, float]:
        self.spindle_angle = (self.spindle_angle + spindle_speed * dt) % (2.0 * math.pi)
        tooth_phase = self.params.flute_count * self.spindle_angle
        excitation = 0.70 * math.sin(tooth_phase)
        excitation += 0.18 * self.rng.gauss(0.0, 1.0)
        excitation += 1.2 * max(0.0, hardness - 1.0)
        excitation += 0.85 * max(0.0, chip_multiplier - 1.0)
        if jam_started:
            excitation += 2.2
        if release_started:
            excitation -= 1.8
        wn = 2.0 * math.pi * self.params.vibration_frequency_hz
        zeta = self.params.vibration_damping_ratio
        accel = (wn * wn) * (excitation - self.vibration_state) - 2.0 * zeta * wn * self.vibration_velocity
        self.vibration_velocity += accel * dt
        self.vibration_state += self.vibration_velocity * dt
        self.vibration_state = max(-3.0, min(3.0, self.vibration_state))
        self.vibration_velocity = max(-2500.0, min(2500.0, self.vibration_velocity))
        self.vibration_torque = self.params.vibration_torque_fraction * max(0.15, base_torque) * self.vibration_state
        self.vibration_force = self.params.vibration_force_fraction * max(20.0, base_force) * self.vibration_state
        return self.vibration_torque, self.vibration_force

    def _set_disturbance_event(self, in_contact: bool) -> None:
        if not in_contact:
            self.disturbance_event = "FREE"
        elif self.chip_jam_active:
            self.disturbance_event = "CHIP_JAM"
        elif self.chip_release_timer > 0.0:
            self.disturbance_event = "CHIP_RELEASE"
        elif self.hardness_multiplier >= 1.24:
            self.disturbance_event = "HARD_SPOT"
        elif abs(self.vibration_state) >= 1.05:
            self.disturbance_event = "CHATTER"
        else:
            self.disturbance_event = "CUTTING"

    def _prepare_cut_state(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float, float, bool, bool]:
        self.sim_time += dt
        gross_depth = max(0.0, rod_position - self.params.contact_start_pos)
        self.penetration_depth = gross_depth
        engagement, area = self.compute_engagement_geometry(rod_position)
        in_contact = engagement > 0.0 and area > 0.0 and abs(spindle_speed) > 5.0
        if not in_contact:
            self.physical_rop_m_s = 0.0
            self.chip_packing_level = max(0.0, self.chip_packing_level - 0.7 * dt)
            self.hardness_multiplier = self._material_hardness(self.surface_recession_depth)
            self.engagement_depth = engagement
            self.vibration_torque = 0.0
            self.vibration_force = 0.0
            self._set_disturbance_event(False)
            return engagement, area, 0.0, 0.0, 1.0, False, False
        self.hardness_multiplier = self._material_hardness(self.surface_recession_depth)
        mrr, _ = self._update_surface_removal(
            dt, gross_depth, engagement, area, rod_velocity, spindle_speed, self.hardness_multiplier
        )
        chip_multiplier, jam_started, release_started = self._update_chip_transport(
            dt, mrr, spindle_speed, engagement, self.hardness_multiplier
        )
        # Material removal changed the cutting surface during this same step.
        # Return the POST-removal engagement, otherwise force/torque can remain
        # artificially saturated one step (or much longer if a large backlog
        # was allowed to accumulate).
        self.engagement_depth = max(
            0.0,
            gross_depth - self.surface_recession_depth,
        )
        area_after = self._contact_area_from_depth(
            self.engagement_depth
        )

        omega = max(1.0, abs(spindle_speed))
        n_rev = omega / (2.0 * math.pi)
        surface_rate = mrr / max(area, 1.0e-12)
        h_avg = surface_rate / max(
            1.0e-9,
            self.params.flute_count * n_rev,
        )

        return (
            self.engagement_depth,
            area_after,
            mrr,
            h_avg,
            chip_multiplier,
            jam_started,
            release_started,
        )

    def step_level_a_averaged(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        engagement, area, mrr, h_avg, chip_mult, jam_started, release_started = self._prepare_cut_state(
            dt, rod_position, rod_velocity, spindle_speed
        )
        if engagement <= 0.0 or area <= 0.0 or abs(spindle_speed) <= 5.0:
            return 0.0, 0.0, 0.0, 0.0
        omega = max(1.0, abs(spindle_speed))
        hard = self.hardness_multiplier
        base_torque = (self.params.averaged_specific_energy * hard * chip_mult * mrr) / omega
        rubbing = self.params.rubbing_torque_coeff * area * hard * (1.0 + 0.35 * self.chip_packing_level)
        torque_nominal = base_torque + rubbing
        force_nominal = (
            self.params.axial_thrust_coeff
            * hard
            * (1.0 + 0.30 * max(0.0, chip_mult - 1.0))
            * area
            + self.params.axial_damping * max(0.0, rod_velocity)
            + self._contact_overtravel_force(
                engagement,
                rod_velocity,
            )
        )
        t_vib, f_vib = self._update_vibration(
            dt, spindle_speed, torque_nominal, force_nominal, hard, chip_mult, jam_started, release_started
        )
        self._set_disturbance_event(True)
        return max(0.0, force_nominal + f_vib), max(0.0, torque_nominal + t_vib), mrr, h_avg

    def step_level_b_tooth_resolved(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        engagement, area, mrr, h_avg, chip_mult, jam_started, release_started = self._prepare_cut_state(
            dt, rod_position, rod_velocity, spindle_speed
        )
        if engagement <= 0.0 or area <= 0.0 or abs(spindle_speed) <= 5.0:
            return 0.0, 0.0, 0.0, 0.0
        omega = max(1.0, abs(spindle_speed))
        hard = self.hardness_multiplier
        z_flutes = self.params.flute_count
        r_b = self.params.cutter_radius
        n_rev = omega / (2.0 * math.pi)
        surface_rate = mrr / max(area, 1.0e-12)
        feed_per_tooth = surface_rate / max(1.0e-9, z_flutes * n_rev)
        axial_depth = min(engagement, r_b)
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
                    h_inst = max(0.0, feed_per_tooth * math.sin(phi))
                    max_h = max(max_h, h_inst)
                    df_t = (self.params.k_tc * hard * chip_mult * h_inst + self.params.k_te * hard) * dz
                    df_a = (self.params.k_ac * hard * h_inst + self.params.k_ae) * dz
                    total_torque += df_t * r_eff
                    total_axial_force += df_a
        total_axial_force += (
            self.params.axial_thrust_coeff
            * hard
            * area
            * 0.35
        )
        total_axial_force += self._contact_overtravel_force(
            engagement,
            rod_velocity,
        )
        total_torque += (
            self.params.rubbing_torque_coeff
            * area
            * 0.35
        )
        t_vib, f_vib = self._update_vibration(
            dt, spindle_speed, max(0.15, total_torque), max(20.0, total_axial_force),
            hard, chip_mult, jam_started, release_started
        )
        self._set_disturbance_event(True)
        return (
            max(0.0, total_axial_force + f_vib),
            max(0.0, total_torque + t_vib),
            mrr,
            max(max_h, h_avg),
        )
