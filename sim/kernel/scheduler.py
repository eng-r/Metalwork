"""
Deterministic virtual-time multirate event scheduler.
"""

from typing import Callable, Optional
import time

from sim.common.contracts import ControlCommands, ProcessVariables
from sim.common.interfaces import IController, IPlant
from sim.kernel.backplane import ProcessBackplane


class VirtualTimeScheduler:
    """Coordinate plant integration and discrete controller evaluation."""

    def __init__(
        self,
        plant: IPlant,
        controller: IController,
        backplane: Optional[ProcessBackplane] = None,
        dt_plant: float = 0.001,
        dt_ctrl: float = 0.010,
    ) -> None:
        self.plant = plant
        self.controller = controller
        self.backplane = backplane or ProcessBackplane()
        self.dt_plant = dt_plant
        self.dt_ctrl = dt_ctrl

        self.ctrl_ratio = max(1, round(dt_ctrl / dt_plant))
        self.t_sim = 0.0
        self.step_count = 0

        self.current_commands = ControlCommands()
        self.current_sensors = ProcessVariables()

    def reset(self) -> None:
        self.t_sim = 0.0
        self.step_count = 0
        self.plant.reset()
        self.controller.reset()
        self.current_sensors = self.plant.get_sensor_readings()
        self.current_commands = self.controller.update(
            self.dt_ctrl, self.current_sensors
        )
        self.current_commands.timestamp = 0.0
        self.backplane.clear()
        self.backplane.record_step(
            sensors=self.current_sensors,
            commands=self.current_commands,
            truth=self.plant.get_truth_diagnostics(),
        )

    def step_tick(self) -> None:
        if self.step_count % self.ctrl_ratio == 0:
            self.current_sensors = self.plant.get_sensor_readings()
            self.current_sensors.timestamp = self.t_sim
            self.current_commands = self.controller.update(
                self.dt_ctrl, self.current_sensors
            )
            self.current_commands.timestamp = self.t_sim
            self.backplane.publish_commands(self.current_commands)

        self.plant.step(self.dt_plant, self.current_commands)
        self.t_sim += self.dt_plant
        self.step_count += 1

        truth = self.plant.get_truth_diagnostics()
        truth.timestamp = self.t_sim
        sensors = self.plant.get_sensor_readings()
        sensors.timestamp = self.t_sim

        # Critical: the WOB estimator lives in the controller and mutates the
        # sampled ProcessVariables object. plant.get_sensor_readings() returns a
        # fresh object with wob_soft_sensor=0. Preserve the latest controller
        # estimate in every backplane/UI telemetry frame.
        sensors.wob_soft_sensor = self.current_sensors.wob_soft_sensor

        self.backplane.record_step(
            sensors=sensors,
            commands=self.current_commands,
            truth=truth,
        )

    def run_until(self, t_target: float) -> None:
        while self.t_sim < t_target:
            self.step_tick()

    def run_realtime_paced(
        self,
        duration: float,
        speed_factor: float = 1.0,
        stop_check: Optional[Callable[[], bool]] = None,
    ) -> None:
        t_end = self.t_sim + duration
        t_wall_start = time.perf_counter()
        t_sim_start = self.t_sim

        while self.t_sim < t_end:
            if stop_check and stop_check():
                break

            self.step_tick()

            elapsed_sim = self.t_sim - t_sim_start
            target_wall_elapsed = elapsed_sim / max(0.01, speed_factor)
            actual_wall_elapsed = time.perf_counter() - t_wall_start
            sleep_needed = target_wall_elapsed - actual_wall_elapsed

            if sleep_needed > 0.002:
                time.sleep(sleep_needed)
