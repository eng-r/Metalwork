"""End-to-end acceptance tests missing from the previous incremental development cycle."""

from sim.controller.adrc import CascadeLADRCController
from sim.controller.baselines import BaselinePIDController
from sim.kernel.scheduler import VirtualTimeScheduler
from sim.plant.cutting import CuttingParameters
from sim.plant.system import CentrifugalHydraulicMillingPlant


def _make_loop(controller_cls, target_torque_nm: float = 4.07):
    plant = CentrifugalHydraulicMillingPlant(
        cut_params=CuttingParameters(disturbances_enabled=False)
    )
    ctrl = controller_cls(
        target_torque_nm=target_torque_nm,
        target_pressure_bar=35.0,
    )
    sched = VirtualTimeScheduler(
        plant=plant,
        controller=ctrl,
        dt_plant=0.001,
        dt_ctrl=0.010,
    )
    sched.reset()
    return plant, ctrl, sched


def test_ladrc_closed_loop_step_up_has_clear_fast_response() -> None:
    plant, ctrl, sched = _make_loop(CascadeLADRCController)
    sched.run_until(12.0)

    before = plant.get_truth_diagnostics().spindle_cutting_torque_true
    assert 3.3 < before < 4.7

    ctrl.apply_runtime_config(
        target_torque_nm=6.10,
        pressure_ceiling_bar=35.0,
        spindle_rpm_nominal=3500.0,
    )
    sched.run_until(12.5)
    after_half_second = plant.get_truth_diagnostics().spindle_cutting_torque_true

    assert after_half_second > before + 1.0
    assert after_half_second > 5.2


def test_pid_and_ladrc_share_same_nominal_authority_path() -> None:
    results = []
    for controller_cls in (BaselinePIDController, CascadeLADRCController):
        plant, ctrl, sched = _make_loop(controller_cls)
        sched.run_until(12.0)
        ctrl.apply_runtime_config(
            target_torque_nm=6.10,
            pressure_ceiling_bar=35.0,
            spindle_rpm_nominal=3500.0,
        )
        sched.run_until(13.0)
        results.append(plant.get_truth_diagnostics().spindle_cutting_torque_true)

    # Both controller packages must have enough authority to reach the same load neighborhood.
    assert min(results) > 5.2
    assert abs(results[0] - results[1]) < 1.2
