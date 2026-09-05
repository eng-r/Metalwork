"""
Unit tests for hydraulic subsystem: bulk modulus, centrifugal pump H-Q, and pressure continuity.
"""

import math
import pytest

from sim.plant.hydraulics import HydraulicParameters, HydraulicSubsystem


def test_bulk_modulus_with_entrained_air() -> None:
    """Validate that bulk modulus is softer at low pressure due to entrained air."""
    params = HydraulicParameters()
    hyd = HydraulicSubsystem(params)

    beta_1bar = hyd.effective_bulk_modulus(1.0e5)
    beta_50bar = hyd.effective_bulk_modulus(5.0e6)

    # At 50 bar, air is compressed, so fluid is much stiffer than at 1 bar
    assert beta_50bar > beta_1bar
    assert beta_50bar > 1.0e9                   # Approaching pure oil bulk modulus
    assert beta_1bar < 5.0e8                    # Significant cushioning from entrained air


def test_pump_flow_non_reversibility() -> None:
    """Assert that centrifugal pump flow is strictly non-negative (no reverse suction)."""
    params = HydraulicParameters()
    hyd = HydraulicSubsystem(params)

    # Zero speed -> zero flow
    assert hyd.compute_pump_flow(0.0, 1.0e5) == 0.0

    # High pressure exceeding shutoff head -> zero flow (does not pull negative flow)
    shutoff_p = params.pump_a0 * ((2000.0 * math.pi / 30.0) ** 2)
    p_high = params.atmospheric_pressure + shutoff_p + 1.0e6
    q_high = hyd.compute_pump_flow(2000.0 * math.pi / 30.0, p_high)
    assert q_high == 0.0


def test_pressure_relaxation_asymmetry() -> None:
    """
    Assert that when pump RPM is 0 and rod is stationary (dx/dt=0),
    pressure can only relax through leakage/bypass, and cannot drop instantly.
    """
    params = HydraulicParameters()
    hyd = HydraulicSubsystem(params)

    p_initial = 40.0e5                          # 40 bar
    dp_dt, q_pump, q_bypass, q_leak = hyd.compute_pressure_derivative(
        pressure=p_initial,
        rod_position=0.05,
        rod_velocity=0.0,                       # Stationary rod
        omega_pump=0.0,                         # Pump idling
    )

    assert q_pump == 0.0
    assert q_bypass > 0.0
    assert dp_dt < 0.0                          # Pressure must decay

    # The decay rate should be modest (stiff orifice): not instantaneous drop
    assert abs(dp_dt) < 5.0e6                   # Less than 50 bar/s decay rate
