"""
Machinist CNC Handbook Guidelines and Operational Limits for Inconel 718.
Provides safe surface cutting speed envelopes, chip load limits, and torque ceilings.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class InconelMachiningEnvelope:
    """CNC Machining recommendations for Inconel 718 nickel-based superalloy."""
    material_name: str = "Inconel 718 (Nickel-Chromium Austenitic)"
    hardness_hrc: float = 42.0                  # Typical aged hardness (38-44 HRC)
    density_kg_m3: float = 8190.0
    specific_cutting_resistance_mpa: float = 3100.0  # k_c1.1 (MPa)
    # Surface speed limits (m/min) with TiAlN-coated carbide / ceramic tool
    surface_speed_min_mpm: float = 25.0
    surface_speed_nominal_mpm: float = 35.0
    surface_speed_max_mpm: float = 55.0
    # Chip load per tooth limits (mm/tooth)
    chip_load_min_mm: float = 0.015             # Minimum chip thickness to prevent rubbing
    chip_load_nominal_mm: float = 0.035
    chip_load_max_mm: float = 0.070
    # Safety cutoffs:
    max_spindle_torque_nm: float = 7.5          # Torque threshold before spindle overload
    critical_stall_torque_nm: float = 9.0       # Impending stall torque
    max_hydraulic_pressure_bar: float = 65.0    # Pusher maximum operating pressure


def compute_recommended_spindle_rpm(
    cutter_diameter_mm: float = 20.0,
    surface_speed_mpm: float = 35.0,
) -> float:
    """
    Calculate spindle RPM from surface cutting speed v_c (m/min) and cutter diameter (mm):
    n = (1000 * v_c) / (pi * D)
    """
    d_eff = max(1.0, cutter_diameter_mm)
    rpm = (1000.0 * surface_speed_mpm) / (math.pi * d_eff)
    return max(100.0, min(8000.0, rpm))


def compute_safe_axial_feed_rate(
    spindle_rpm: float,
    flute_count: int = 4,
    target_chip_load_mm: float = 0.035,
) -> float:
    """
    Calculate target axial plunge feed velocity v_f (m/s):
    v_f = (f_z * Z * n) / 60000
    """
    vf_mm_min = target_chip_load_mm * flute_count * spindle_rpm
    return (vf_mm_min / 60.0) / 1000.0          # m/s
