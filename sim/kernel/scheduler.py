"""
Deterministic Virtual-Time Multirate Event Scheduler.
Ensures zero-jitter, reproducible simulation with explicit sample periods and Zero-Order Holds.
"""

from typing import Callable, Optional
import time

from sim.common.contracts import ControlCommands, ProcessVariables, TruthDiagnostics
from sim.common.interfaces import IController, IPlant
from sim.kernel.backplane import ProcessBackplane


class VirtualTimeScheduler:
    """
    Coordinates plant integration and discrete controller evaluation on a deterministic virtual clock.
    """

    def __init__(
        self,
        plant: IPlant,
        controller: IController,
        backplane: Optional[ProcessBackplane] = None,
        dt_plant: float = 0.001,    # 1.0 ms plant physics step (1000 Hz)
        dt_ctrl: float = 0.010,     # 10.0 ms controller step (100 Hz)
    ) -> None:
        self.plant = plant
        self.controller = controller
        self.backplane = backplane or ProcessBackplane()
        self.dt_plant = dt_plant
        self.dt_ctrl = dt_ctrl

        self.ctrl_ratio = max(1, round(dt_ctrl / dt_plant))
        self.t_sim = 0.0
        self.step_count = 0

        # Current latched commands (Zero-Order Hold)
        self.current_commands = ControlCommands()
        self.current_sensors = ProcessVariables()

    def reset(self) -> None:
        """Reset virtual clock, plant, controller, and latches to t=0."""
        self.t_sim = 0.0
        self.step_count = 0
        self.plant.reset()
        self.controller.reset()
        self.current_sensors = self.plant.get_sensor_readings()
        self.current_commands = self.controller.update(self.dt_ctrl, self.current_sensors)
        self.current_commands.timestamp = 0.0
        self.backplane.clear()
        self.backplane.record_step(
            sensors=self.current_sensors,
            commands=self.current_commands,
            truth=self.plant.get_truth_diagnostics(),
        )

    def step_tick(self) -> None:
        """
        Advance virtual time by exactly one plant timestep dt_plant.
        Evaluates discrete controller when step_count aligns with dt_ctrl.
        """
        # 1. Check if controller update is due (Zero-Order Hold update)
        if self.step_count % self.ctrl_ratio == 0:
            self.current_sensors = self.plant.get_sensor_readings()
            self.current_sensors.timestamp = self.t_sim
            self.current_commands = self.controller.update(self.dt_ctrl, self.current_sensors)
            self.current_commands.timestamp = self.t_sim
            self.backplane.publish_commands(self.current_commands)

        # 2. Integrate continuous plant physics by dt_plant
        self.plant.step(self.dt_plant, self.current_commands)
        self.t_sim += self.dt_plant
        self.step_count += 1

        # 3. Capture truth diagnostics and record step to backplane
        truth = self.plant.get_truth_diagnostics()
        truth.timestamp = self.t_sim
        sensors = self.plant.get_sensor_readings()
        sensors.timestamp = self.t_sim

        self.backplane.record_step(
            sensors=sensors,
            commands=self.current_commands,
            truth=truth,
        )

    def run_until(self, t_target: float) -> None:
        """Execute simulation in deterministic batch mode until t_sim reaches t_target."""
        while self.t_sim < t_target:
            self.step_tick()

    def run_realtime_paced(
        self,
        duration: float,
        speed_factor: float = 1.0,
        stop_check: Optional[Callable[[], bool]] = None,
    ) -> None:
        """
        Run simulation with wall-clock pacing for live interactive UI streaming.
        Virtual time increments deterministically regardless of timing jitter.
        """
        t_end = self.t_sim + duration
        t_wall_start = time.perf_counter()
        t_sim_start = self.t_sim

        while self.t_sim < t_end:
            if stop_check and stop_check():
                break

            self.step_tick()

            # Pacing calculation
            elapsed_sim = self.t_sim - t_sim_start
            target_wall_elapsed = elapsed_sim / max(0.01, speed_factor)
            actual_wall_elapsed = time.perf_counter() - t_wall_start
            sleep_needed = target_wall_elapsed - actual_wall_elapsed

            if sleep_needed > 0.002:
                time.sleep(sleep_needed)
