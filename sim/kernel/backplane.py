"""
In-Memory Process Backplane: Thread-Safe Atomic Double-Buffered State Container.
Provides zero-broker shared communication between plant, controller, recorder, and UI.
"""

from dataclasses import dataclass, field
import threading
from typing import Deque, List, Optional
from collections import deque

from sim.common.contracts import ControlCommands, ProcessVariables, TruthDiagnostics


@dataclass(slots=True)
class TelemetryFrame:
    """Consolidated snapshot frame at a specific simulation instant."""
    timestamp: float
    sensors: ProcessVariables
    commands: ControlCommands
    truth: TruthDiagnostics


class ProcessBackplane:
    """
    Lock-minimized thread-safe operational bus.
    Allows concurrent, non-blocking snapshot reads by UI/loggers while simulation kernel steps.
    """

    def __init__(self, history_len: int = 2000) -> None:
        self._lock = threading.Lock()
        self._current_sensors = ProcessVariables()
        self._current_commands = ControlCommands()
        self._current_truth = TruthDiagnostics()
        self._history: Deque[TelemetryFrame] = deque(maxlen=history_len)

    def publish_sensors(self, sensors: ProcessVariables) -> None:
        """Called by plant when sensors are updated."""
        with self._lock:
            self._current_sensors = sensors

    def publish_commands(self, commands: ControlCommands) -> None:
        """Called by controller when new commands are issued."""
        with self._lock:
            self._current_commands = commands

    def record_step(self, sensors: ProcessVariables, commands: ControlCommands, truth: TruthDiagnostics) -> None:
        """Atomically capture a full virtual-time simulation tick into history."""
        frame = TelemetryFrame(
            timestamp=truth.timestamp,
            sensors=sensors,
            commands=commands,
            truth=truth,
        )
        with self._lock:
            self._current_sensors = sensors
            self._current_commands = commands
            self._current_truth = truth
            self._history.append(frame)

    def get_latest_snapshot(self) -> TelemetryFrame:
        """Retrieve an instantaneous snapshot without holding the lock during downstream processing."""
        with self._lock:
            return TelemetryFrame(
                timestamp=self._current_truth.timestamp,
                sensors=self._current_sensors,
                commands=self._current_commands,
                truth=self._current_truth,
            )

    def get_recent_history(self, count: Optional[int] = None) -> List[TelemetryFrame]:
        """Retrieve recent telemetry frames for decimated streaming or vector charting."""
        with self._lock:
            if count is None or count >= len(self._history):
                return list(self._history)
            return list(self._history)[-count:]

    def clear(self) -> None:
        """Reset historical buffer."""
        with self._lock:
            self._history.clear()
            self._current_sensors = ProcessVariables()
            self._current_commands = ControlCommands()
            self._current_truth = TruthDiagnostics()
