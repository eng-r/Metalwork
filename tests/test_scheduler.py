"""
Unit tests for deterministic virtual-time scheduler and ZOH command latching.
"""

import pytest

from sim.controller.adrc import CascadeLADRCController
from sim.kernel.backplane import ProcessBackplane
from sim.kernel.scheduler import VirtualTimeScheduler
from sim.plant.system import CentrifugalHydraulicMillingPlant


def test_bit_exact_simulation_determinism() -> None:
    """
    Verify that repeated simulation runs on the virtual-time scheduler
    produce bit-exact identical floating point results.
    """
    def run_trial() -> list[float]:
        plant = CentrifugalHydraulicMillingPlant()
        controller = CascadeLADRCController()
        backplane = ProcessBackplane(history_len=500)
        sched = VirtualTimeScheduler(plant, controller, backplane, dt_plant=0.001, dt_ctrl=0.010)

        sched.reset()
        sched.run_until(1.0)                    # Run 1 second (1000 plant steps, 100 ctrl steps)

        history = backplane.get_recent_history()
        return [f.sensors.pressure_hyd for f in history]

    trial_1 = run_trial()
    trial_2 = run_trial()

    assert len(trial_1) == len(trial_2)
    assert trial_1 == trial_2                   # Bit-exact equality across floating point array


def test_zero_order_hold_latching() -> None:
    """Verify that actuator commands are held constant over dt_ctrl interval."""
    plant = CentrifugalHydraulicMillingPlant()
    controller = CascadeLADRCController()
    backplane = ProcessBackplane(history_len=50)
    sched = VirtualTimeScheduler(plant, controller, backplane, dt_plant=0.001, dt_ctrl=0.010)

    sched.reset()
    # Step 5 plant ticks (5 ms, half of a controller period)
    for _ in range(5):
        sched.step_tick()

    cmd_first = sched.current_commands.pump_speed_cmd_rpm
    sched.step_tick()
    cmd_sixth = sched.current_commands.pump_speed_cmd_rpm

    # Must be identical because controller has not triggered a new cycle yet
    assert cmd_first == cmd_sixth
