import math

from sim.plant.cutting import CuttingParameters, MechanisticCuttingSubsystem


def test_rotating_tool_clears_engagement_while_feed_is_held() -> None:
    params = CuttingParameters(contact_start_pos=0.015)
    cutting = MechanisticCuttingSubsystem(params, seed=42)
    dt = 0.001
    rod_position = 0.018
    omega = 3500.0 * math.pi / 30.0

    engagement0, _ = cutting.compute_engagement_geometry(rod_position)
    for _ in range(1000):
        cutting.step_level_a_averaged(
            dt=dt,
            rod_position=rod_position,
            rod_velocity=0.0,
            spindle_speed=omega,
        )

    engagement1, _ = cutting.compute_engagement_geometry(rod_position)
    assert cutting.surface_recession_depth > 0.0
    assert engagement1 < engagement0
    assert cutting.cumulative_volume_removed > 0.0


def test_disturbance_realization_is_repeatable_for_same_seed() -> None:
    params = CuttingParameters(contact_start_pos=0.015)
    a = MechanisticCuttingSubsystem(params, seed=17)
    b = MechanisticCuttingSubsystem(params, seed=17)
    dt = 0.001
    omega = 3500.0 * math.pi / 30.0

    trace_a = []
    trace_b = []
    for i in range(4000):
        x = 0.014 + i * dt * 0.0008
        trace_a.append(a.step_level_a_averaged(dt, x, 0.0008, omega)[1])
        trace_b.append(b.step_level_a_averaged(dt, x, 0.0008, omega)[1])

    assert trace_a == trace_b


def test_realistic_cut_produces_intermittent_disturbance_events() -> None:
    cutting = MechanisticCuttingSubsystem(CuttingParameters(), seed=42)
    dt = 0.001
    omega = 3500.0 * math.pi / 30.0
    seen = set()

    for i in range(12000):
        x = 0.014 + i * dt * 0.0008
        cutting.step_level_a_averaged(dt, x, 0.0008, omega)
        seen.add(cutting.disturbance_event)

    assert "HARD_SPOT" in seen
    assert "CHIP_JAM" in seen
    assert "CHIP_RELEASE" in seen
