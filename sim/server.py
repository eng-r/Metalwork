"""
FastAPI REST and High-Speed WebSocket Telemetry Server for Milling Workstation UI.
"""

import asyncio
from dataclasses import asdict
import threading
import time
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel
import uvicorn

from sim.common.contracts import ControlCommands, ProcessVariables, TruthDiagnostics
from sim.controller.adrc import CascadeLADRCController
from sim.controller.baselines import BaselinePIDController
from sim.kernel.backplane import ProcessBackplane, TelemetryFrame
from sim.kernel.scheduler import VirtualTimeScheduler
from sim.plant.system import CentrifugalHydraulicMillingPlant


class ConfigRequest(BaseModel):
    controller_type: str = "adrc"               # "adrc" or "pid"
    target_pressure_bar: float = 35.0
    spindle_rpm_nominal: float = 3500.0
    material_hardness_hrc: float = 42.0
    bypass_orifice_area_scale: float = 1.0


class CommandRequest(BaseModel):
    action: str                                 # "start", "pause", "reset", "step"


app = FastAPI(title="Milling Workstation Telemetry Server", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global simulation state container
class SimulationRuntime:
    def __init__(self) -> None:
        self.plant = CentrifugalHydraulicMillingPlant()
        self.controller = CascadeLADRCController()
        self.backplane = ProcessBackplane(history_len=3000)
        self.scheduler = VirtualTimeScheduler(
            plant=self.plant,
            controller=self.controller,
            backplane=self.backplane,
            dt_plant=0.001,
            dt_ctrl=0.010,
        )
        self.is_running = False
        self.stop_requested = False
        self.worker_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.active_websockets: List[WebSocket] = []

        self.scheduler.reset()

    def start(self) -> None:
        with self.lock:
            if self.is_running:
                return
            self.is_running = True
            self.stop_requested = False
            self.worker_thread = threading.Thread(target=self._run_loop, daemon=True)
            self.worker_thread.start()

    def pause(self) -> None:
        with self.lock:
            self.stop_requested = True
            self.is_running = False

    def reset(self) -> None:
        self.pause()
        with self.lock:
            self.scheduler.reset()

    def step(self) -> None:
        self.pause()
        with self.lock:
            # Advance 10 ms (one controller cycle)
            for _ in range(10):
                self.scheduler.step_tick()

    def _run_loop(self) -> None:
        while self.is_running and not self.stop_requested:
            # Advance in blocks of 10 ms paced to wall-clock
            t_start = time.perf_counter()
            with self.lock:
                for _ in range(10):
                    self.scheduler.step_tick()

            # Pacing sleep (approx 10 ms wall time)
            elapsed = time.perf_counter() - t_start
            sleep_time = 0.010 - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    def reconfigure(self, cfg: ConfigRequest) -> None:
        was_running = self.is_running
        self.pause()

        with self.lock:
            # Update plant parameters
            self.plant.cut_params.averaged_specific_energy = (cfg.material_hardness_hrc / 42.0) * 3.2e9
            self.plant.hyd_params.bypass_orifice_cd_a = 1.2e-8 * cfg.bypass_orifice_area_scale

            # Recreate controller
            if cfg.controller_type.lower() == "pid":
                self.controller = BaselinePIDController(
                    target_pressure_bar=cfg.target_pressure_bar,
                    spindle_rpm_nominal=cfg.spindle_rpm_nominal,
                )
            else:
                self.controller = CascadeLADRCController(
                    target_pressure_bar=cfg.target_pressure_bar,
                    spindle_rpm_nominal=cfg.spindle_rpm_nominal,
                )
            self.scheduler.controller = self.controller

        if was_running:
            self.start()


runtime = SimulationRuntime()


@app.get("/api/status")
def get_status() -> Dict[str, Any]:
    snap = runtime.backplane.get_latest_snapshot()
    return {
        "is_running": runtime.is_running,
        "sim_time": snap.timestamp,
        "operating_mode": runtime.controller.get_operating_mode(),
        "controller_type": runtime.controller.__class__.__name__,
        "pressure_bar": snap.sensors.pressure_bar,
        "spindle_rpm": snap.sensors.spindle_rpm,
        "pump_rpm": snap.sensors.pump_rpm,
        "spindle_torque_nm": snap.sensors.spindle_torque_est,
        "wob_n": snap.sensors.wob_soft_sensor,
        "crater_depth_mm": snap.truth.penetration_depth * 1000.0,
    }


@app.post("/api/command")
def post_command(cmd: CommandRequest) -> Dict[str, str]:
    if cmd.action == "start":
        runtime.start()
    elif cmd.action == "pause":
        runtime.pause()
    elif cmd.action == "reset":
        runtime.reset()
    elif cmd.action == "step":
        runtime.step()
    return {"status": "ok", "action": cmd.action}


@app.post("/api/configure")
def post_configure(cfg: ConfigRequest) -> Dict[str, str]:
    runtime.reconfigure(cfg)
    return {"status": "ok"}


@app.get("/api/export/csv", response_class=PlainTextResponse)
def export_csv() -> str:
    history = runtime.backplane.get_recent_history()
    lines = [
        "timestamp,pressure_bar,pressure_true_bar,spindle_rpm,pump_rpm,spindle_torque_nm,wob_n,depth_mm,mrr_mm3_s"
    ]
    for f in history:
        lines.append(
            f"{f.timestamp:.4f},{f.sensors.pressure_bar:.3f},{f.truth.chamber_pressure_true/1e5:.3f},"
            f"{f.sensors.spindle_rpm:.1f},{f.sensors.pump_rpm:.1f},{f.sensors.spindle_torque_est:.3f},"
            f"{f.sensors.wob_soft_sensor:.1f},{f.truth.penetration_depth*1000:.3f},{f.truth.material_removal_rate*1e9:.2f}"
        )
    return "\n".join(lines)


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            snap = runtime.backplane.get_latest_snapshot()
            data = {
                "timestamp": snap.timestamp,
                "is_running": runtime.is_running,
                "operating_mode": runtime.controller.get_operating_mode(),
                "pressure_bar": round(snap.sensors.pressure_bar, 2),
                "pressure_true_bar": round(snap.truth.chamber_pressure_true / 1.0e5, 2),
                "spindle_rpm": round(snap.sensors.spindle_rpm, 1),
                "pump_rpm": round(snap.sensors.pump_rpm, 1),
                "pump_cmd_rpm": round(snap.commands.pump_speed_cmd_rpm, 1),
                "spindle_torque_est": round(snap.sensors.spindle_torque_est, 3),
                "spindle_torque_true": round(snap.truth.spindle_cutting_torque_true, 3),
                "wob_soft_sensor": round(snap.sensors.wob_soft_sensor, 1),
                "axial_cutting_force_true": round(snap.truth.axial_cutting_force_true, 1),
                "rod_position_mm": round(snap.truth.rod_position_true * 1000.0, 3),
                "penetration_depth_mm": round(snap.truth.penetration_depth * 1000.0, 3),
                "mrr_mm3_s": round(snap.truth.material_removal_rate * 1.0e9, 2),
                "cumulative_volume_mm3": round(snap.truth.cumulative_volume_removed * 1.0e9, 2),
                "seal_friction_n": round(snap.truth.seal_friction_force, 1),
                "leso_z3_disturbance": round(
                    runtime.controller.get_internal_states().get("leso_z3_disturbance", 0.0), 3
                ),
            }
            await websocket.send_json(data)
            # Decimate to 30 Hz stream (approx 33 ms period)
            await asyncio.sleep(0.033)
    except WebSocketDisconnect:
        pass


def start_server(host: str = "127.0.0.1", port: int = 8000) -> None:
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    start_server()
