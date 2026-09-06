import math

from sim.common.contracts import ControlCommands
from sim.plant.cutting import (
    CuttingParameters,
    MechanisticCuttingSubsystem,
)
from sim.plant.hydraulics import (
    HydraulicParameters,
    HydraulicSubsystem,
)
from sim.plant.mechanics import (
    MechanicsParameters,
    MechanicsSubsystem,
)
from sim.plant.system import (
    CentrifugalHydraulicMillingPlant,
)


def test_pump_curve_is_small_high_head_not_tens_of_lpm() -> None:
    hyd = HydraulicSubsystem(HydraulicParameters())
    q = hyd.compute_pump_flow(
        omega_pump=3000.0 * math.pi / 30.0,
        chamber_pressure=20.0e5,
    )

    # Around a few mL/s for this compact hydraulic pusher.
    assert 1.0e-6 < q < 1.0e-5


def test_seal_has_true_static_friction() -> None:
    mech = MechanicsSubsystem(MechanicsParameters())

    # 20 bar gauge gives several hundred newtons of static capacity.
    accel, friction = mech.compute_acceleration(
        gauge_pressure=20.0e5,
        piston_area=2.0e-3,
        axial_cutting_force=3800.0,
        position=0.010,
        velocity=0.0,
    )

    assert abs(accel) < 1.0e-12
    assert abs(friction) > 1.0


def test_contact_force_penalizes_unphysical_overtravel() -> None:
    cut = MechanisticCuttingSubsystem(
        CuttingParameters(),
        seed=42,
    )

    spindle = 3500.0 * math.pi / 30.0

    # Compare near-full-face contact to excessive virtual penetration.
    f1, _, _, _ = cut.step_level_a_averaged(
        0.001,
        cut.params.contact_start_pos + 0.0010,
        0.0,
        spindle,
    )

    cut.reset()
    f2, _, _, _ = cut.step_level_a_averaged(
        0.001,
        cut.params.contact_start_pos + 0.0020,
        0.0,
        spindle,
    )

    assert f2 > f1 + 3000.0


def test_pump_off_cutting_relaxes_engagement_and_torque() -> None:
    plant = CentrifugalHydraulicMillingPlant()

    full_face = (
        plant.cut_params.target_sphere_radius
        - math.sqrt(
            plant.cut_params.target_sphere_radius ** 2
            - plant.cut_params.cutter_radius ** 2
        )
    )

    initial_engagement = full_face + 0.00020
    initial_position = (
        plant.cut_params.contact_start_pos
        + initial_engagement
    )

    plant.reset(
        initial_state={
            "pressure": 35.0e5,
            "position": initial_position,
        }
    )

    spindle = 3500.0 * math.pi / 30.0
    plant.spindle_drive.reset(
        initial_speed_rads=spindle
    )

    cmd = ControlCommands(
        pump_speed_cmd_rpm=0.0,
        spindle_speed_cmd_rpm=3500.0,
        enable_pump=False,
        enable_spindle=True,
    )

    plant.step(0.001, cmd)
    initial = plant.get_truth_diagnostics()

    # Several demo seconds correspond to minutes of physical Inconel cutting.
    for _ in range(6000):
        plant.step(0.001, cmd)

    final = plant.get_truth_diagnostics()

    assert final.engagement_depth < initial.engagement_depth
    assert final.spindle_cutting_torque_true < (
        0.75 * initial.spindle_cutting_torque_true
    )
    assert final.chamber_pressure_true < initial.chamber_pressure_true
