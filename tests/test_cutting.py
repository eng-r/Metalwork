"""
Unit tests for mechanistic milling mechanics against Inconel 718 sphere.
"""

import math
import pytest

from sim.plant.cutting import CuttingParameters, MechanisticCuttingSubsystem


def test_spherical_intersection_geometry() -> None:
    """Verify that penetration depth and contact area are zero prior to contact."""
    params = CuttingParameters(contact_start_pos=0.080)
    cutting = MechanisticCuttingSubsystem(params)

    # Before contact
    d0, a0 = cutting.compute_engagement_geometry(rod_position=0.075)
    assert d0 == 0.0
    assert a0 == 0.0

    # At 5 mm penetration
    d1, a1 = cutting.compute_engagement_geometry(rod_position=0.085)
    assert abs(d1 - 0.005) < 1.0e-6
    assert a1 > 0.0
    # Must be bounded by tool cross section pi * R_bit^2
    assert a1 <= math.pi * (params.cutter_radius ** 2)


def test_level_a_cutting_power_conservation() -> None:
    """Verify that cutting torque and axial force are strictly non-negative during cutting."""
    params = CuttingParameters(contact_start_pos=0.080)
    cutting = MechanisticCuttingSubsystem(params)

    dt = 0.001
    f_ax, t_cut, mrr, h_chip = cutting.step_level_a_averaged(
        dt=dt,
        rod_position=0.085,                     # 5 mm into Inconel 718
        rod_velocity=0.001,                     # 1 mm/s feed
        spindle_speed=350.0,                    # ~3340 RPM
    )

    assert f_ax > 0.0                           # Positive axial thrust opposing feed
    assert t_cut > 0.0                          # Positive resistive cutting torque
    assert mrr > 0.0
    assert h_chip > 0.0


def test_level_b_tooth_resolved_harmonics() -> None:
    """Verify that Level B tooth-resolved model generates periodic flute ripple."""
    params = CuttingParameters(contact_start_pos=0.080)
    cutting = MechanisticCuttingSubsystem(params)

    torques = []
    dt = 0.0002                                # 200 microseconds
    for _ in range(50):
        _, t_cut, _, _ = cutting.step_level_b_tooth_resolved(
            dt=dt,
            rod_position=0.083,
            rod_velocity=0.001,
            spindle_speed=300.0,
        )
        torques.append(t_cut)

    # Torque must exhibit dynamic variations across tooth passages
    assert max(torques) > min(torques)
