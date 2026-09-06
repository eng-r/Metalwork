import math

from sim.plant.cutting import CuttingParameters, MechanisticCuttingSubsystem


def _sample(depth_m: float, hardness_scale: float = 1.0):
    params = CuttingParameters(
        disturbances_enabled=False,
        demo_acceleration=1.0,
        base_material_hardness_scale=hardness_scale,
    )
    cut = MechanisticCuttingSubsystem(params, seed=42)
    spindle = 3500.0 * math.pi / 30.0
    return cut.step_level_a_averaged(
        0.001,
        params.contact_start_pos + depth_m,
        0.0,
        spindle,
    ), cut


def test_tob_and_wob_increase_monotonically_with_engagement() -> None:
    samples = [_sample(d)[0] for d in (0.00025, 0.00050, 0.00075, 0.00100)]
    forces = [s[0] for s in samples]
    torques = [s[1] for s in samples]

    assert forces == sorted(forces)
    assert torques == sorted(torques)
    assert forces[-1] > 4.0 * forces[0]
    assert torques[-1] > 4.0 * torques[0]


def test_rop_increases_with_wob_and_is_zero_without_contact() -> None:
    low, cut_low = _sample(0.00035)
    high, cut_high = _sample(0.00085)

    assert cut_low.physical_rop_m_s > 0.0
    assert cut_high.physical_rop_m_s > cut_low.physical_rop_m_s

    params = CuttingParameters(disturbances_enabled=False, demo_acceleration=1.0)
    free = MechanisticCuttingSubsystem(params)
    spindle = 3500.0 * math.pi / 30.0
    free.step_level_a_averaged(
        0.001,
        params.contact_start_pos - 0.001,
        0.0,
        spindle,
    )
    assert free.physical_rop_m_s == 0.0


def test_harder_material_raises_torque_and_reduces_rop() -> None:
    nominal, cut_nom = _sample(0.00075, hardness_scale=1.0)
    hard, cut_hard = _sample(0.00075, hardness_scale=1.20)

    assert hard[1] > nominal[1]
    assert cut_hard.physical_rop_m_s < cut_nom.physical_rop_m_s
