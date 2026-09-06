"""
FastAPI REST and WebSocket telemetry server for the milling workstation.
"""

import asyncio
import threading
import time
from typing import Any, Dict, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel
import uvicorn

from sim.controller.adrc import CascadeLADRCController
from sim.controller.baselines import BaselinePIDController
from sim.kernel.backplane import ProcessBackplane
from sim.kernel.scheduler import VirtualTimeScheduler
from sim.plant.system import CentrifugalHydraulicMillingPlant


class ConfigRequest(BaseModel):
    controller_type: str = "adrc"
    target_pressure_bar: float = 35.0       # hydraulic ceiling, not primary load SP
    target_torque_nm: float = 4.0           # desired Torque-on-Bit
    spindle_rpm_nominal: float = 3500.0
    material_hardness_hrc: float = 42.0
    bypass_orifice_area_scale: float = 1.0


class CommandRequest(BaseModel):
    action: str


app = FastAPI(title="Milling Workstation Telemetry Server", version="0.2.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class SimulationRuntime:
    def __init__(self) -> None:
        self.plant = CentrifugalHydraulicMillingPlant()
        self.controller = CascadeLADRCController(target_torque_nm=4.0)

        # 4x previous backend history (3000 -> 12000 physics frames).
        self.backplane = ProcessBackplane(history_len=12000)
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

        self.scheduler.reset()

    def start(self) -> None:
        with self.lock:
            if self.is_running:
                return
            self.is_running = True
            self.stop_requested = False
            self.worker_thread = threading.Thread(
                target=self._run_loop, daemon=True
            )
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
            for _ in range(10):
                self.scheduler.step_tick()

    def _run_loop(self) -> None:
        while self.is_running and not self.stop_requested:
            t_start = time.perf_counter()
            with self.lock:
                for _ in range(10):
                    self.scheduler.step_tick()

            elapsed = time.perf_counter() - t_start
            sleep_time = 0.010 - elapsed
            if sleep_time > 0.001:
                time.sleep(sleep_time)

    def reconfigure(self, cfg: ConfigRequest) -> None:
        was_running = self.is_running
        self.pause()

        with self.lock:
            self.plant.cut_params.averaged_specific_energy = (
                cfg.material_hardness_hrc / 42.0
            ) * 3.2e9
            self.plant.hyd_params.bypass_orifice_cd_a = (
                1.2e-8 * cfg.bypass_orifice_area_scale
            )

            controller_cls = (
                BaselinePIDController
                if cfg.controller_type.lower() == "pid"
                else CascadeLADRCController
            )
            self.controller = controller_cls(
                target_pressure_bar=cfg.target_pressure_bar,
                target_torque_nm=cfg.target_torque_nm,
                spindle_rpm_nominal=cfg.spindle_rpm_nominal,
            )
            self.controller.reset()
            self.scheduler.controller = self.controller

        if was_running:
            self.start()


runtime = SimulationRuntime()


@app.get("/api/status")
def get_status() -> Dict[str, Any]:
    snap = runtime.backplane.get_latest_snapshot()
    internal = runtime.controller.get_internal_states()
    return {
        "is_running": runtime.is_running,
        "sim_time": snap.timestamp,
        "operating_mode": runtime.controller.get_operating_mode(),
        "controller_type": internal.get("controller_type", runtime.controller.__class__.__name__),
        "pressure_pa": snap.sensors.pressure_hyd,
        "spindle_rpm": snap.sensors.spindle_rpm,
        "pump_rpm": snap.sensors.pump_rpm,
        "spindle_torque_nm": snap.sensors.spindle_torque_est,
        "target_torque_nm": internal.get("target_torque_nm", 4.0),
        "wob_n": snap.sensors.wob_soft_sensor,
        "physical_rop_m_s": snap.truth.physical_rop,
        "surface_recession_m": snap.truth.surface_recession_depth,
        "equivalent_process_time_s": snap.truth.equivalent_process_time,
        "demo_acceleration": snap.truth.demo_acceleration,
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
        "timestamp,pressure_pa,spindle_rpm,pump_rpm,spindle_torque_nm,"
        "wob_n,physical_rop_m_s,surface_recession_m,disturbance_event"
    ]
    for f in history:
        lines.append(
            f"{f.timestamp:.4f},{f.sensors.pressure_hyd:.1f},"
            f"{f.sensors.spindle_rpm:.1f},{f.sensors.pump_rpm:.1f},"
            f"{f.sensors.spindle_torque_est:.4f},{f.sensors.wob_soft_sensor:.2f},"
            f"{f.truth.physical_rop:.9g},{f.truth.surface_recession_depth:.9g},"
            f"{f.truth.disturbance_event}"
        )
    return "\n".join(lines)


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            snap = runtime.backplane.get_latest_snapshot()
            internal = runtime.controller.get_internal_states()
            data = {
                "timestamp": snap.timestamp,
                "is_running": runtime.is_running,
                "operating_mode": runtime.controller.get_operating_mode(),
                "controller_type": internal.get(
                    "controller_type", runtime.controller.__class__.__name__
                ),

                # Existing engineering-unit API fields. The browser performs
                # all requested US-customary display conversions.
                "pressure_bar": round(snap.sensors.pressure_bar, 3),
                "pressure_true_bar": round(
                    snap.truth.chamber_pressure_true / 1.0e5, 3
                ),
                "spindle_rpm": round(snap.sensors.spindle_rpm, 1),
                "pump_rpm": round(snap.sensors.pump_rpm, 1),
                "pump_cmd_rpm": round(snap.commands.pump_speed_cmd_rpm, 1),
                "spindle_torque_est": round(
                    snap.sensors.spindle_torque_est, 4
                ),
                "spindle_torque_true": round(
                    snap.truth.spindle_cutting_torque_true, 4
                ),
                "target_torque_nm": round(
                    float(internal.get("target_torque_nm", 4.0)), 4
                ),
                "wob_soft_sensor": round(snap.sensors.wob_soft_sensor, 2),
                "axial_cutting_force_true": round(
                    snap.truth.axial_cutting_force_true, 2
                ),
                "rod_position_mm": round(
                    snap.truth.rod_position_true * 1000.0, 3
                ),
                "penetration_depth_mm": round(
                    snap.truth.penetration_depth * 1000.0, 3
                ),
                "mrr_mm3_s": round(
                    snap.truth.material_removal_rate * 1.0e9, 6
                ),
                "cumulative_volume_mm3": round(
                    snap.truth.cumulative_volume_removed * 1.0e9, 3
                ),
                "seal_friction_n": round(snap.truth.seal_friction_force, 1),
                "leso_z3_disturbance": round(
                    float(internal.get("leso_z3_disturbance", 0.0)), 3
                ),

                # Raw SI slow-process quantities.
                "physical_rop_m_s": snap.truth.physical_rop,
                "surface_recession_m": snap.truth.surface_recession_depth,
                "demo_acceleration": snap.truth.demo_acceleration,
                "equivalent_process_time_s": snap.truth.equivalent_process_time,
                "disturbance_event": snap.truth.disturbance_event,
            }
            await websocket.send_json(data)
            await asyncio.sleep(0.033)
    except WebSocketDisconnect:
        pass


def start_server(host: str = "127.0.0.1", port: int = 8000) -> None:
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    start_server()
