"""
Control-oriented milling mechanics for a cylindrical cutter engaging a nickel-alloy target.

The model is deliberately causal for control studies:

    hydraulic pressure -> axial force / WOB -> ToB and ROP -> material removal

Material removal then changes the contact geometry, which closes the slow mechanical loop.
Random hardness/chip/vibration effects are disturbances around this nominal plant, not the
source of its mean behavior.
"""

from dataclasses import dataclass
import math
import random
from typing import List, Tuple


@dataclass
class CuttingParameters:
    """Geometrical, cutting, removal, and disturbance parameters."""

    # Geometry.
    cutter_radius: float = 0.010
    flute_count: int = 4
    helix_angle_deg: float = 30.0
    target_sphere_radius: float = 0.050
    contact_start_pos: float = 0.015

    # Retained mechanistic coefficients for Level-B / future calibration.
    k_tc: float = 3100.0e6
    k_te: float = 65.0e3
    k_rc: float = 1350.0e6
    k_re: float = 45.0e3
    k_ac: float = 820.0e6
    k_ae: float = 32.0e3

    # Contact/WOB model. Engagement is the amount of unremoved target interference.
    # At ~0.8 mm engagement this gives ~4.4 kN nominal WOB before hardness scaling.
    contact_stiffness_n_m: float = 5.5e6
    contact_damping_n_s_m: float = 900.0
    max_contact_force_n: float = 12000.0

    # ToB = c_TF * WOB + specific-energy term + edge term + disturbances.
    # c_TF has units of metres and is physically interpretable as mu_eff * r_eff.
    torque_force_coeff_m: float = 1.05e-3
    averaged_specific_energy: float = 3.2e9
    edge_torque_nm: float = 0.10

    # Slow Inconel removal model.
    nominal_spindle_rpm: float = 3500.0
    nominal_physical_rop_mm_min: float = 0.080
    max_physical_rop_mm_min: float = 0.20
    reference_wob_n: float = 4500.0
    minimum_cut_wob_n: float = 250.0
    wob_rop_exponent: float = 0.82
    speed_rop_exponent: float = 0.65
    hardness_rop_exponent: float = 1.25

    # Demo acceleration applies only to the slow material-surface state. The displayed ROP
    # remains the physical value.
    demo_acceleration: float = 120.0

    # Spatial material non-uniformity.
    base_material_hardness_scale: float = 1.0
    disturbances_enabled: bool = True
    material_texture_amplitude: float = 0.035
    hard_spot_count: int = 5
    hard_spot_depth_span: float = 0.014
    hard_spot_width_min: float = 0.00015
    hard_spot_width_max: float = 0.00045
    hard_spot_gain_min: float = 0.15
    hard_spot_gain_max: float = 0.40

    # Chip transport. This is intentionally not a deterministic fill/jam/release oscillator.
    chip_load_gain: float = 0.18
    chip_clear_rate: float = 0.28
    chip_jam_load_threshold: float = 0.30
    chip_jam_hazard_per_s: float = 0.80
    chip_jam_duration_min: float = 0.12
    chip_jam_duration_max: float = 0.35
    chip_jam_refractory_min: float = 1.5
    chip_jam_refractory_max: float = 3.5
    chip_jam_torque_gain: float = 0.55

    # Structural vibration. Raw plant truth may contain this mode; controller telemetry is
    # filtered/anti-aliased elsewhere.
    vibration_frequency_hz: float = 58.0
    vibration_damping_ratio: float = 0.20
    vibration_torque_fraction: float = 0.045
    vibration_force_fraction: float = 0.025

    # Tooth-resolved modulation used by Level B around the same mean causal model.
    runout_amplitude: float = 1.5e-5
    tooth_torque_modulation_fraction: float = 0.12
    tooth_force_modulation_fraction: float = 0.06


class MechanisticCuttingSubsystem:
    """Stateful material/contact model shared by both fidelity levels."""

    def __init__(
        self,
        params: CuttingParameters = CuttingParameters(),
        seed: int = 42,
    ) -> None:
        self.params = params
        self.seed = seed
        self.rng = random.Random(seed)

        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.surface_recession_depth = 0.0
        self.engagement_depth = 0.0
        self.physical_rop_m_s = 0.0
        self.equivalent_process_time = 0.0
        self.sim_time = 0.0
        self.spindle_angle = 0.0

        self.hardness_multiplier = 1.0
        self.chip_load = 0.0
        self.chip_jam_active = False
        self.chip_jam_timer = 0.0
        self.chip_jam_refractory = 0.0
        self.chip_release_timer = 0.0

        self.vibration_state = 0.0
        self.vibration_velocity = 0.0
        self.vibration_torque = 0.0
        self.vibration_force = 0.0
        self.disturbance_event = "FREE"

        self._hard_spots: List[Tuple[float, float, float]] = []
        self._reset_disturbance_map()

    def _reset_disturbance_map(self) -> None:
        self._hard_spots.clear()
        if not self.params.disturbances_enabled:
            return

        count = max(0, self.params.hard_spot_count)
        span = max(1.0e-6, self.params.hard_spot_depth_span)
        for i in range(count):
            spacing = span / max(1, count)
            center = (i + 0.65) * spacing
            center += self.rng.uniform(-0.20, 0.20) * spacing
            width = self.rng.uniform(
                self.params.hard_spot_width_min,
                self.params.hard_spot_width_max,
            )
            gain = self.rng.uniform(
                self.params.hard_spot_gain_min,
                self.params.hard_spot_gain_max,
            )
            self._hard_spots.append((max(0.0, center), width, gain))
        self._hard_spots.sort(key=lambda item: item[0])

    def reset(self) -> None:
        self.rng = random.Random(self.seed)
        self.cumulative_volume_removed = 0.0
        self.penetration_depth = 0.0
        self.surface_recession_depth = 0.0
        self.engagement_depth = 0.0
        self.physical_rop_m_s = 0.0
        self.equivalent_process_time = 0.0
        self.sim_time = 0.0
        self.spindle_angle = 0.0

        self.hardness_multiplier = 1.0
        self.chip_load = 0.0
        self.chip_jam_active = False
        self.chip_jam_timer = 0.0
        self.chip_jam_refractory = 0.0
        self.chip_release_timer = 0.0

        self.vibration_state = 0.0
        self.vibration_velocity = 0.0
        self.vibration_torque = 0.0
        self.vibration_force = 0.0
        self.disturbance_event = "FREE"
        self._reset_disturbance_map()

    @property
    def full_face_area(self) -> float:
        return math.pi * self.params.cutter_radius**2

    def _contact_area_from_depth(self, depth: float) -> float:
        """Projected cylinder/sphere contact area for the current unremoved engagement."""
        if depth <= 0.0:
            return 0.0

        r_s = self.params.target_sphere_radius
        r_b = self.params.cutter_radius
        d = min(depth, 2.0 * r_s)
        if d < r_s:
            area_geom = math.pi * max(0.0, 2.0 * r_s * d - d * d)
        else:
            area_geom = math.pi * r_s * r_s
        return min(area_geom, self.full_face_area)

    def compute_engagement_geometry(self, rod_position: float) -> Tuple[float, float]:
        gross_depth = max(0.0, rod_position - self.params.contact_start_pos)
        engagement = max(0.0, gross_depth - self.surface_recession_depth)
        return engagement, self._contact_area_from_depth(engagement)

    def _material_hardness(self, cut_front_depth: float) -> float:
        base = max(0.70, self.params.base_material_hardness_scale)
        if not self.params.disturbances_enabled:
            return base

        depth_mm = cut_front_depth * 1000.0
        texture = self.params.material_texture_amplitude * (
            0.60 * math.sin(2.0 * math.pi * depth_mm / 1.7 + 0.4)
            + 0.40 * math.sin(2.0 * math.pi * depth_mm / 0.71 + 1.3)
        )

        inclusions = 0.0
        for center, width, gain in self._hard_spots:
            z = (cut_front_depth - center) / max(width, 1.0e-8)
            inclusions += gain * math.exp(-0.5 * z * z)

        return max(0.70, base * (1.0 + texture + inclusions))

    def evaluate_axial_force(
        self,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> float:
        """
        Pure (non-state-advancing) contact/WOB evaluation for ODE integration stages.

        This function is intentionally callable at every RK stage. It uses the currently frozen
        material surface, chip state and hardness state but the stage-specific position/velocity.
        """
        engagement, area = self.compute_engagement_geometry(rod_position)
        if engagement <= 0.0 or area <= 0.0:
            return 0.0

        area_ratio = min(1.0, area / max(self.full_face_area, 1.0e-12))
        hardness = max(0.88, self.hardness_multiplier)

        # Contact stiffness is mildly hardness- and area-dependent. The area dependence softens
        # first touch while preserving a monotonic pressure->WOB path after full engagement.
        k_eff = self.params.contact_stiffness_n_m * (
            0.50 + 0.50 * math.sqrt(max(0.0, area_ratio))
        ) * math.sqrt(hardness)

        force = k_eff * engagement
        if rod_velocity > 0.0:
            force += self.params.contact_damping_n_s_m * rod_velocity

        # Jammed chips raise axial ploughing resistance without inventing a separate force source.
        if self.chip_jam_active:
            force *= 1.0 + 0.15 * self.params.chip_jam_torque_gain
        elif self.params.disturbances_enabled:
            force *= 1.0 + 0.035 * self.chip_load

        return max(0.0, min(self.params.max_contact_force_n, force))

    def _compute_physical_rop(
        self,
        wob_n: float,
        spindle_speed: float,
        hardness: float,
        contact_area: float,
    ) -> float:
        """Physical material surface advance rate [m/s], before demo acceleration."""
        p = self.params
        if (
            wob_n <= p.minimum_cut_wob_n
            or contact_area <= 0.0
            or abs(spindle_speed) < 5.0
        ):
            return 0.0

        nominal = p.nominal_physical_rop_mm_min / (1000.0 * 60.0)
        max_rop = p.max_physical_rop_mm_min / (1000.0 * 60.0)

        effective_wob = max(0.0, wob_n - p.minimum_cut_wob_n)
        reference_effective = max(1.0, p.reference_wob_n - p.minimum_cut_wob_n)
        wob_factor = (effective_wob / reference_effective) ** p.wob_rop_exponent
        wob_factor = min(1.80, max(0.0, wob_factor))

        speed_ref = p.nominal_spindle_rpm * math.pi / 30.0
        speed_ratio = max(0.0, abs(spindle_speed) / max(1.0, speed_ref))
        speed_factor = min(1.35, speed_ratio ** p.speed_rop_exponent)

        area_ratio = min(1.0, contact_area / max(self.full_face_area, 1.0e-12))
        geometry_factor = area_ratio ** 0.15
        hardness_factor = 1.0 / max(0.70, hardness ** p.hardness_rop_exponent)

        chip_efficiency = 1.0 / (1.0 + 0.45 * self.chip_load)
        if self.chip_jam_active:
            chip_efficiency *= 0.48

        rop = (
            nominal
            * wob_factor
            * speed_factor
            * geometry_factor
            * hardness_factor
            * chip_efficiency
        )
        return max(0.0, min(max_rop, rop))

    def _update_chip_transport(
        self,
        dt: float,
        physical_mrr: float,
        spindle_speed: float,
    ) -> Tuple[bool, bool]:
        """Update a bounded chip-load state with stochastic, refractory jam events."""
        if not self.params.disturbances_enabled:
            self.chip_load = 0.0
            self.chip_jam_active = False
            self.chip_jam_timer = 0.0
            self.chip_jam_refractory = 0.0
            self.chip_release_timer = 0.0
            return False, False

        p = self.params
        reference_mrr = max(
            1.0e-15,
            self.full_face_area
            * p.nominal_physical_rop_mm_min
            / (1000.0 * 60.0),
        )
        mrr_norm = min(2.5, max(0.0, physical_mrr / reference_mrr))
        speed_ref = p.nominal_spindle_rpm * math.pi / 30.0
        speed_factor = min(1.5, max(0.0, abs(spindle_speed) / max(1.0, speed_ref)))

        self.chip_jam_refractory = max(0.0, self.chip_jam_refractory - dt)
        self.chip_release_timer = max(0.0, self.chip_release_timer - dt)
        jam_started = False
        jam_released = False

        if self.chip_jam_active:
            self.chip_jam_timer -= dt
            self.chip_load = min(
                1.2,
                self.chip_load + 0.08 * mrr_norm * dt,
            )
            if self.chip_jam_timer <= 0.0:
                self.chip_jam_active = False
                self.chip_load *= self.rng.uniform(0.30, 0.50)
                self.chip_jam_refractory = self.rng.uniform(
                    p.chip_jam_refractory_min,
                    p.chip_jam_refractory_max,
                )
                self.chip_release_timer = 0.15
                jam_released = True
            return jam_started, jam_released

        accumulation = p.chip_load_gain * mrr_norm
        clearing = p.chip_clear_rate * speed_factor * self.chip_load
        self.chip_load += (accumulation - clearing) * dt
        self.chip_load = min(1.0, max(0.0, self.chip_load))

        if (
            self.chip_jam_refractory <= 0.0
            and self.chip_load > p.chip_jam_load_threshold
        ):
            severity = (
                self.chip_load - p.chip_jam_load_threshold
            ) / max(1.0e-6, 1.0 - p.chip_jam_load_threshold)
            hazard = p.chip_jam_hazard_per_s * severity
            if self.rng.random() < hazard * dt:
                self.chip_jam_active = True
                self.chip_jam_timer = self.rng.uniform(
                    p.chip_jam_duration_min,
                    p.chip_jam_duration_max,
                )
                jam_started = True

        return jam_started, jam_released

    def _update_vibration(
        self,
        dt: float,
        spindle_speed: float,
        base_torque: float,
        base_force: float,
        hardness: float,
        jam_started: bool,
        jam_released: bool,
    ) -> Tuple[float, float]:
        if not self.params.disturbances_enabled:
            self.vibration_state = 0.0
            self.vibration_velocity = 0.0
            self.vibration_torque = 0.0
            self.vibration_force = 0.0
            return 0.0, 0.0

        self.spindle_angle = (
            self.spindle_angle + spindle_speed * dt
        ) % (2.0 * math.pi)

        tooth_phase = self.params.flute_count * self.spindle_angle
        excitation = 0.40 * math.sin(tooth_phase)
        excitation += 0.06 * self.rng.gauss(0.0, 1.0)
        excitation += 0.45 * max(0.0, hardness - 1.0)
        excitation += 0.20 * self.chip_load
        if jam_started:
            excitation += 1.0
        if jam_released:
            excitation -= 0.8

        wn = 2.0 * math.pi * self.params.vibration_frequency_hz
        zeta = self.params.vibration_damping_ratio
        accel = (
            wn * wn * (excitation - self.vibration_state)
            - 2.0 * zeta * wn * self.vibration_velocity
        )

        self.vibration_velocity += accel * dt
        self.vibration_state += self.vibration_velocity * dt
        self.vibration_state = max(-2.0, min(2.0, self.vibration_state))
        self.vibration_velocity = max(-1800.0, min(1800.0, self.vibration_velocity))

        self.vibration_torque = (
            self.params.vibration_torque_fraction
            * max(0.2, base_torque)
            * self.vibration_state
        )
        self.vibration_force = (
            self.params.vibration_force_fraction
            * max(50.0, base_force)
            * self.vibration_state
        )
        return self.vibration_torque, self.vibration_force

    def _set_disturbance_event(self, in_contact: bool) -> None:
        if not in_contact:
            self.disturbance_event = "FREE"
        elif self.chip_jam_active:
            self.disturbance_event = "CHIP_JAM"
        elif self.chip_release_timer > 0.0:
            self.disturbance_event = "CHIP_RELEASE"
        elif self.hardness_multiplier >= 1.18:
            self.disturbance_event = "HARD_SPOT"
        elif abs(self.vibration_state) >= 1.15:
            self.disturbance_event = "CHATTER"
        else:
            self.disturbance_event = "CUTTING"

    def _step_mean_model(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        self.sim_time += dt

        gross_depth = max(0.0, rod_position - self.params.contact_start_pos)
        self.penetration_depth = gross_depth
        engagement_before, area_before = self.compute_engagement_geometry(rod_position)

        if engagement_before <= 0.0 or area_before <= 0.0 or abs(spindle_speed) < 5.0:
            self.engagement_depth = engagement_before
            self.physical_rop_m_s = 0.0
            self.chip_load = max(0.0, self.chip_load - 0.15 * dt)
            self.hardness_multiplier = self._material_hardness(
                self.surface_recession_depth
            )
            self._set_disturbance_event(False)
            return 0.0, 0.0, 0.0, 0.0

        self.hardness_multiplier = self._material_hardness(
            self.surface_recession_depth
        )

        wob_before = self.evaluate_axial_force(
            rod_position,
            rod_velocity,
            spindle_speed,
        )
        physical_rop = self._compute_physical_rop(
            wob_before,
            spindle_speed,
            self.hardness_multiplier,
            area_before,
        )
        self.physical_rop_m_s = physical_rop

        # Physical MRR is used for cutting power/telemetry. Only geometry evolution is accelerated.
        physical_mrr = area_before * physical_rop
        demo_surface_rate = physical_rop * max(1.0, self.params.demo_acceleration)
        clearable = max(0.0, gross_depth - self.surface_recession_depth)
        delta_clear = min(clearable, demo_surface_rate * dt)
        self.surface_recession_depth += delta_clear
        self.equivalent_process_time += dt * max(1.0, self.params.demo_acceleration)
        self.cumulative_volume_removed += area_before * delta_clear

        jam_started, jam_released = self._update_chip_transport(
            dt,
            physical_mrr,
            spindle_speed,
        )

        # Re-evaluate force after material was removed during this tick.
        self.engagement_depth, area_after = self.compute_engagement_geometry(rod_position)
        wob_after = self.evaluate_axial_force(
            rod_position,
            rod_velocity - demo_surface_rate,
            spindle_speed,
        )

        omega = max(1.0, abs(spindle_speed))
        hardness = self.hardness_multiplier
        area_ratio = min(1.0, area_after / max(self.full_face_area, 1.0e-12))

        # Main causal torque path: WOB -> tangential/ploughing torque.
        load_torque = self.params.torque_force_coeff_m * wob_after
        load_torque *= 1.0 + 0.35 * max(0.0, hardness - 1.0)
        load_torque *= 1.0 + 0.18 * self.chip_load
        if self.chip_jam_active:
            load_torque *= 1.0 + self.params.chip_jam_torque_gain

        # Cutting-energy term is physically retained but is small at the intentionally slow ROP.
        shear_torque = (
            self.params.averaged_specific_energy
            * hardness
            * physical_mrr
        ) / omega
        edge_torque = self.params.edge_torque_nm * math.sqrt(max(0.0, area_ratio))
        torque_nominal = load_torque + shear_torque + edge_torque

        vib_torque, vib_force = self._update_vibration(
            dt,
            spindle_speed,
            torque_nominal,
            wob_after,
            hardness,
            jam_started,
            jam_released,
        )
        self._set_disturbance_event(True)

        force_out = max(0.0, wob_after + vib_force)
        torque_out = max(0.0, torque_nominal + vib_torque)

        n_rev = omega / (2.0 * math.pi)
        feed_per_tooth = physical_rop / max(
            1.0e-9,
            self.params.flute_count * n_rev,
        )
        return force_out, torque_out, physical_mrr, feed_per_tooth

    def step_level_a_averaged(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        """Control-oriented revolution-averaged model."""
        return self._step_mean_model(
            dt,
            rod_position,
            rod_velocity,
            spindle_speed,
        )

    def step_level_b_tooth_resolved(
        self,
        dt: float,
        rod_position: float,
        rod_velocity: float,
        spindle_speed: float,
    ) -> Tuple[float, float, float, float]:
        """
        Tooth-resolved visualization model built around the same mean causal load model.

        It deliberately does not define a second, contradictory steady-state plant. Tooth passage
        and runout modulate the Level-A mean while preserving pressure/WOB/ToB causality.
        """
        force_mean, torque_mean, mrr, h_avg = self._step_mean_model(
            dt,
            rod_position,
            rod_velocity,
            spindle_speed,
        )
        if force_mean <= 0.0 and torque_mean <= 0.0:
            return force_mean, torque_mean, mrr, h_avg

        tooth_phase = self.params.flute_count * self.spindle_angle
        runout_phase = self.spindle_angle
        torque_mod = 1.0 + (
            self.params.tooth_torque_modulation_fraction * math.sin(tooth_phase)
            + 0.025 * math.cos(runout_phase)
        )
        force_mod = 1.0 + (
            self.params.tooth_force_modulation_fraction * math.sin(tooth_phase + 0.6)
        )

        return (
            max(0.0, force_mean * force_mod),
            max(0.0, torque_mean * torque_mod),
            mrr,
            h_avg,
        )
