import math

from sim.plant.cutting import CuttingParameters, MechanisticCuttingSubsystem


def test_rotating_loaded_tool_clears_engagement_while_feed_is_held() -> None:
    params = CuttingParameters(contact_start_pos=0.015, disturbances_enabled=False)
    cutting = MechanisticCuttingSubsystem(params, seed=42)
    dt = 0.001
    rod_position = 0.0160
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
    for i in range(8000):
        x = 0.014 + i * dt * 0.0008
        trace_a.append(a.step_level_a_averaged(dt, x, 0.0008, omega)[1])
        trace_b.append(b.step_level_a_averaged(dt, x, 0.0008, omega)[1])

    assert trace_a == trace_b


def test_disturbances_perturb_nominal_cut_without_becoming_periodic_plant() -> None:
    """
    Hard spots and chip events should occur for the seeded demo, but they must not create the
    previous deterministic fill/jam/release sawtooth on every short cycle.
    """
    cutting = MechanisticCuttingSubsystem(CuttingParameters(), seed=42)
    dt = 0.001
    omega = 3500.0 * math.pi / 30.0
    seen = set()
    jam_entries = 0
    previous = "FREE"

    # Drive a representative loaded cut for 30 s of demo time.
    for i in range(30000):
        x = 0.0154 + i * dt * 0.00030
        cutting.step_level_a_averaged(dt, x, 0.00005, omega)
        event = cutting.disturbance_event
        seen.add(event)
        if event == "CHIP_JAM" and previous != "CHIP_JAM":
            jam_entries += 1
        previous = event

    assert "HARD_SPOT" in seen or "CHIP_JAM" in seen
    assert jam_entries <= 4
