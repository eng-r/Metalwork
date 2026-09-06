"""FastAPI REST and anti-aliased WebSocket telemetry server for the milling workstation."""

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
from sim.kernel.backplane import ProcessBackplane, TelemetryFrame
from sim.kernel.scheduler import VirtualTimeScheduler
from sim.plant.system import CentrifugalHydraulicMillingPlant


class ConfigRequest(BaseModel):
    controller_type: str = "adrc"
    target_pressure_bar: float = 35.0
    target_torque_nm: float = 4.0
    spindle_rpm_nominal: float = 3500.0
    material_hardness_hrc: float = 42.0
    bypass_orifice_area_scale: float = 1.0


class CommandRequest(BaseModel):
    action: str


app = FastAPI(title="Milling Workstation Telemetry Server", version="0.3.0")
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
        self.nominal_specific_energy = self.plant.cut_params.averaged_specific_energy
        self.nominal_bypass_orifice_cd_a = self.plant.hyd_params.bypass_orifice_cd_a

        self.controller = CascadeLADRCController(target_torque_nm=4.0)
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

    def reconfigure(self, cfg: ConfigRequest) -> Dict[str, Any]:
        """Live/bumpless setpoint and plant-parameter update."""
        with self.lock:
            hardness_scale = max(0.75, min(1.35, (cfg.material_hardness_hrc / 42.0) ** 1.2))
            self.plant.cut_params.base_material_hardness_scale = hardness_scale
            self.plant.cut_params.averaged_specific_energy = (
                self.nominal_specific_energy * hardness_scale
            )
            self.plant.hyd_params.bypass_orifice_cd_a = (
                self.nominal_bypass_orifice_cd_a * cfg.bypass_orifice_area_scale
            )

            wants_pid = cfg.controller_type.lower() == "pid"
            same_architecture = (
                wants_pid and isinstance(self.controller, BaselinePIDController)
            ) or (
                (not wants_pid) and isinstance(self.controller, CascadeLADRCController)
            )

            if same_architecture:
                self.controller.apply_runtime_config(
                    target_torque_nm=cfg.target_torque_nm,
                    pressure_ceiling_bar=cfg.target_pressure_bar,
                    spindle_rpm_nominal=cfg.spindle_rpm_nominal,
                )
            else:
                controller_cls = BaselinePIDController if wants_pid else CascadeLADRCController
                self.controller = controller_cls(
                    target_pressure_bar=cfg.target_pressure_bar,
                    target_torque_nm=cfg.target_torque_nm,
                    spindle_rpm_nominal=cfg.spindle_rpm_nominal,
                )
                self.controller.reset()
                self.scheduler.controller = self.controller

            internal = self.controller.get_internal_states()
            return {
                "controller_type": internal.get(
                    "controller_type", self.controller.__class__.__name__
                ),
                "target_torque_nm": float(
                    internal.get("target_torque_nm", cfg.target_torque_nm)
                ),
                "pressure_ceiling_bar": float(
                    internal.get("pressure_ceiling_bar", cfg.target_pressure_bar)
                ),
                "spindle_rpm_nominal": float(self.controller.spindle_rpm_nominal),
                "control_limited": bool(internal.get("control_limited", False)),
                "limit_reason": str(internal.get("limit_reason", "")),
                "achievable_torque_nm": float(
                    internal.get("achievable_torque_nm", cfg.target_torque_nm)
                ),
            }


runtime = SimulationRuntime()


def _mean(frames: list[TelemetryFrame], getter) -> float:
    if not frames:
        return 0.0
    return sum(float(getter(f)) for f in frames) / len(frames)


def _anti_aliased_snapshot() -> Dict[str, float]:
    """
    Return ~33 ms rectangular-window averages for actual waveforms before 30 Hz UI streaming.

    This prevents raw 58 Hz tooth/vibration content from aliasing into fake low-frequency motion.
    References/setpoints are intentionally taken from the latest frame so their steps stay sharp.
    """
    frames = runtime.backplane.get_recent_history(33)
    latest = runtime.backplane.get_latest_snapshot()
    if not frames:
        frames = [latest]

    return {
        "pressure_bar": _mean(frames, lambda f: f.sensors.pressure_bar),
        "pressure_true_bar": _mean(
            frames, lambda f: f.truth.chamber_pressure_true / 1.0e5
        ),
        "spindle_rpm": _mean(frames, lambda f: f.sensors.spindle_rpm),
        "pump_rpm": _mean(frames, lambda f: f.sensors.pump_rpm),
        "pump_cmd_rpm": _mean(frames, lambda f: f.commands.pump_speed_cmd_rpm),
        "spindle_torque_est": _mean(
            frames, lambda f: f.sensors.spindle_torque_est
        ),
        "spindle_torque_true": _mean(
            frames, lambda f: f.truth.spindle_cutting_torque_true
        ),
        "wob_soft_sensor": _mean(frames, lambda f: f.sensors.wob_soft_sensor),
        "axial_cutting_force_true": _mean(
            frames, lambda f: f.truth.axial_cutting_force_true
        ),
        "physical_rop_m_s": _mean(frames, lambda f: f.truth.physical_rop),
    }


@app.get("/api/status")
def get_status() -> Dict[str, Any]:
    snap = runtime.backplane.get_latest_snapshot()
    internal = runtime.controller.get_internal_states()
    return {
        "is_running": runtime.is_running,
        "sim_time": snap.timestamp,
        "operating_mode": runtime.controller.get_operating_mode(),
        "controller_type": internal.get(
            "controller_type", runtime.controller.__class__.__name__
        ),
        "pressure_pa": snap.sensors.pressure_hyd,
        "pressure_reference_bar": internal.get(
            "filtered_reference_bar", snap.sensors.pressure_bar
        ),
        "spindle_rpm": snap.sensors.spindle_rpm,
        "pump_rpm": snap.sensors.pump_rpm,
        "spindle_torque_nm": snap.sensors.spindle_torque_est,
        "target_torque_nm": internal.get("target_torque_nm", 4.0),
        "wob_n": snap.sensors.wob_soft_sensor,
        "target_wob_n": internal.get("target_wob_n", 0.0),
        "engagement_depth_m": snap.truth.engagement_depth,
        "physical_rop_m_s": snap.truth.physical_rop,
        "surface_recession_m": snap.truth.surface_recession_depth,
        "equivalent_process_time_s": snap.truth.equivalent_process_time,
        "demo_acceleration": snap.truth.demo_acceleration,
        "control_limited": internal.get("control_limited", False),
        "limit_reason": internal.get("limit_reason", ""),
        "achievable_torque_nm": internal.get("achievable_torque_nm", 0.0),
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
def post_configure(cfg: ConfigRequest) -> Dict[str, Any]:
    return {"status": "ok", **runtime.reconfigure(cfg)}


@app.get("/api/export/csv", response_class=PlainTextResponse)
def export_csv() -> str:
    history = runtime.backplane.get_recent_history()
    lines = [
        "timestamp,pressure_pa,spindle_rpm,pump_rpm,pump_cmd_rpm,spindle_torque_nm,"
        "wob_n,physical_rop_m_s,engagement_m,surface_recession_m,disturbance_event"
    ]
    for f in history:
        lines.append(
            f"{f.timestamp:.4f},{f.sensors.pressure_hyd:.1f},"
            f"{f.sensors.spindle_rpm:.1f},{f.sensors.pump_rpm:.1f},"
            f"{f.commands.pump_speed_cmd_rpm:.1f},{f.sensors.spindle_torque_est:.4f},"
            f"{f.sensors.wob_soft_sensor:.2f},{f.truth.physical_rop:.9g},"
            f"{f.truth.engagement_depth:.9g},{f.truth.surface_recession_depth:.9g},"
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
            aa = _anti_aliased_snapshot()

            data = {
                "timestamp": snap.timestamp,
                "is_running": runtime.is_running,
                "operating_mode": runtime.controller.get_operating_mode(),
                "controller_type": internal.get(
                    "controller_type", runtime.controller.__class__.__name__
                ),

                "pressure_bar": round(aa["pressure_bar"], 3),
                "pressure_true_bar": round(aa["pressure_true_bar"], 3),
                "pressure_reference_bar": round(
                    float(internal.get("filtered_reference_bar", snap.sensors.pressure_bar)),
                    3,
                ),
                "pressure_feedforward_bar": round(
                    float(internal.get("pressure_feedforward_bar", 0.0)), 3
                ),
                "pressure_ceiling_bar": round(
                    float(internal.get("pressure_ceiling_bar", 35.0)), 3
                ),

                "spindle_rpm": round(aa["spindle_rpm"], 1),
                "spindle_cmd_rpm": round(snap.commands.spindle_speed_cmd_rpm, 1),
                "pump_rpm": round(aa["pump_rpm"], 1),
                "pump_cmd_rpm": round(aa["pump_cmd_rpm"], 1),
                "pump_feedforward_rpm": round(
                    float(internal.get("pump_feedforward_rpm", 0.0)), 1
                ),

                "spindle_torque_est": round(aa["spindle_torque_est"], 4),
                "spindle_torque_true": round(aa["spindle_torque_true"], 4),
                "target_torque_nm": round(
                    float(internal.get("target_torque_nm", 4.0)), 4
                ),
                "achievable_torque_nm": round(
                    float(internal.get("achievable_torque_nm", 0.0)), 4
                ),
                "control_limited": bool(internal.get("control_limited", False)),
                "limit_reason": str(internal.get("limit_reason", "")),

                "wob_soft_sensor": round(aa["wob_soft_sensor"], 2),
                "axial_cutting_force_true": round(
                    aa["axial_cutting_force_true"], 2
                ),
                "target_wob_n": round(float(internal.get("target_wob_n", 0.0)), 2),

                "rod_position_mm": round(snap.truth.rod_position_true * 1000.0, 3),
                "penetration_depth_mm": round(snap.truth.penetration_depth * 1000.0, 3),
                "engagement_depth_mm": round(snap.truth.engagement_depth * 1000.0, 4),
                "mrr_mm3_s": round(snap.truth.material_removal_rate * 1.0e9, 6),
                "cumulative_volume_mm3": round(
                    snap.truth.cumulative_volume_removed * 1.0e9, 3
                ),
                "seal_friction_n": round(snap.truth.seal_friction_force, 1),

                "leso_z3_disturbance": round(
                    float(internal.get("leso_z3_disturbance", 0.0)), 3
                ),
                "physical_rop_m_s": aa["physical_rop_m_s"],
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
