"""
Unit tests for pusher mechanics: pressure-dependent Stribeck friction and mass acceleration.
"""

import pytest

from sim.plant.mechanics import MechanicsParameters, MechanicsSubsystem


def test_pressure_dependent_friction_scaling() -> None:
    """Verify that Coulomb and breakaway friction scale linearly with chamber pressure."""
    params = MechanicsParameters()
    mech = MechanicsSubsystem(params)

    # Low pressure friction at 0 bar gauge
    f_fric_0bar = mech.compute_seal_friction(gauge_pressure=0.0, velocity=0.05)

    # High pressure friction at 50 bar gauge (5.0 MPa)
    f_fric_50bar = mech.compute_seal_friction(gauge_pressure=5.0e6, velocity=0.05)

    assert f_fric_50bar > f_fric_0bar
    # Breakaway and Coulomb scaling factor should produce at least 300 N additional friction
    assert (f_fric_50bar - f_fric_0bar) > 300.0


def test_stribeck_zero_velocity_regularization() -> None:
    """Verify that friction vanishes cleanly at exact zero velocity via tanh regularization."""
    params = MechanicsParameters()
    mech = MechanicsSubsystem(params)

    f_fric_zero = mech.compute_seal_friction(gauge_pressure=30.0e5, velocity=0.0)
    assert abs(f_fric_zero) < 1.0e-3


def test_rod_acceleration_force_balance() -> None:
    """Verify that axial acceleration equals net force divided by effective moving mass."""
    params = MechanicsParameters(effective_mass=50.0)
    mech = MechanicsSubsystem(params)

    # Apply 10 bar gauge with piston area 2e-3 m^2 -> 2000 N hydraulic force
    gauge_p = 10.0e5
    piston_area = 2.0e-3
    accel, f_fric = mech.compute_acceleration(
        gauge_pressure=gauge_p,
        piston_area=piston_area,
        axial_cutting_force=0.0,
        position=0.05,
        velocity=0.01,
    )

    expected_f_net = (gauge_p * piston_area) - f_fric - (params.viscous_damping * 0.01)
    expected_accel = expected_f_net / params.effective_mass
    assert abs(accel - expected_accel) < 1.0e-4
