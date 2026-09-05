"""
Abstract Base Classes and Interfaces for Swappable Plants, Controllers, and Observers.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from sim.common.contracts import ControlCommands, ProcessVariables, TruthDiagnostics


class IPlant(ABC):
    """
    Contract for physical truth simulation models.
    Can be stepped using explicit numerical integrators (e.g. RK4).
    """

    @abstractmethod
    def reset(self, initial_state: Optional[Dict[str, Any]] = None) -> None:
        """Reset internal physical states to initial conditions."""
        pass

    @abstractmethod
    def step(self, dt: float, commands: ControlCommands) -> None:
        """Advance physical states by continuous timestep dt under active actuator commands."""
        pass

    @abstractmethod
    def get_sensor_readings(self) -> ProcessVariables:
        """Return noisy, filtered, sensor-equivalent telemetry for controller consumption."""
        pass

    @abstractmethod
    def get_truth_diagnostics(self) -> TruthDiagnostics:
        """Return full internal physics state for validation, logging, and 3D visualization."""
        pass


class IController(ABC):
    """
    Contract for discrete-time milling controllers.
    Executed at a slower rate (e.g. 50 Hz - 100 Hz) using sampled sensor feedback.
    """

    @abstractmethod
    def reset(self) -> None:
        """Reset controller integrator states, observer poles, and state machine."""
        pass

    @abstractmethod
    def update(self, dt: float, sensors: ProcessVariables) -> ControlCommands:
        """Compute next actuator setpoints given latest sampled sensor measurements."""
        pass

    @abstractmethod
    def get_operating_mode(self) -> str:
        """Return the current supervisory operational mode."""
        pass

    @abstractmethod
    def get_internal_states(self) -> Dict[str, Any]:
        """Return diagnostic state metrics (e.g. LESO disturbance, integrator terms)."""
        pass


class ISoftSensor(ABC):
    """
    Contract for soft sensors and dynamic state estimators (e.g. Soft WOB observer).
    """

    @abstractmethod
    def reset(self) -> None:
        """Reset internal observer states."""
        pass

    @abstractmethod
    def update(self, dt: float, sensors: ProcessVariables) -> float:
        """Update observer state and return primary estimated scalar."""
        pass
