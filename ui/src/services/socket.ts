/** WebSocket client service connecting to FastAPI backend telemetry stream. */

export interface TelemetryData {
  timestamp: number;
  is_running: boolean;
  operating_mode: string;
  controller_type: string;

  pressure_bar: number;
  pressure_true_bar: number;
  pressure_reference_bar: number;
  pressure_feedforward_bar: number;
  pressure_ceiling_bar: number;

  spindle_rpm: number;
  spindle_cmd_rpm: number;
  pump_rpm: number;
  pump_cmd_rpm: number;
  pump_feedforward_rpm: number;

  spindle_torque_est: number;
  spindle_torque_true: number;
  target_torque_nm: number;
  achievable_torque_nm: number;
  control_limited: boolean;
  limit_reason: string;

  wob_soft_sensor: number;
  axial_cutting_force_true: number;
  target_wob_n: number;

  rod_position_mm: number;
  penetration_depth_mm: number;
  engagement_depth_mm: number;
  mrr_mm3_s: number;
  cumulative_volume_mm3: number;
  seal_friction_n: number;
  leso_z3_disturbance: number;

  physical_rop_m_s: number;
  surface_recession_m: number;
  demo_acceleration: number;
  equivalent_process_time_s: number;
  disturbance_event: string;
}

export interface AppliedConfig {
  status: string;
  controller_type: string;
  target_torque_nm: number;
  pressure_ceiling_bar: number;
  spindle_rpm_nominal: number;
  control_limited: boolean;
  limit_reason: string;
  achievable_torque_nm: number;
}

type TelemetryCallback = (data: TelemetryData) => void;

class TelemetrySocketClient {
  private socket: WebSocket | null = null;
  private listeners: Set<TelemetryCallback> = new Set();
  private reconnectInterval = 2000;
  private url = 'ws://127.0.0.1:8000/ws/telemetry';

  constructor() {
    this.connect();
  }

  public connect(): void {
    try {
      this.socket = new WebSocket(this.url);
      this.socket.onmessage = (event: MessageEvent) => {
        try {
          const data: TelemetryData = JSON.parse(event.data);
          this.listeners.forEach((callback) => callback(data));
        } catch (err) {
          console.error('[WS] Parse error:', err);
        }
      };
      this.socket.onclose = () => {
        setTimeout(() => this.connect(), this.reconnectInterval);
      };
      this.socket.onerror = () => {
        this.socket?.close();
      };
    } catch {
      setTimeout(() => this.connect(), this.reconnectInterval);
    }
  }

  public subscribe(callback: TelemetryCallback): () => void {
    this.listeners.add(callback);
    return () => this.listeners.delete(callback);
  }

  public async sendCommand(
    action: 'start' | 'pause' | 'reset' | 'step',
  ): Promise<void> {
    try {
      await fetch('http://127.0.0.1:8000/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action }),
      });
    } catch (err) {
      console.error('[API] Failed to dispatch command:', err);
    }
  }

  public async updateConfig(config: {
    controller_type: string;
    target_pressure_bar: number;
    target_torque_nm: number;
    spindle_rpm_nominal: number;
    material_hardness_hrc: number;
    bypass_orifice_area_scale: number;
  }): Promise<AppliedConfig | null> {
    try {
      const response = await fetch('http://127.0.0.1:8000/api/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config),
      });
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      return (await response.json()) as AppliedConfig;
    } catch (err) {
      console.error('[API] Failed to update config:', err);
      return null;
    }
  }
}

export const telemetryClient = new TelemetrySocketClient();
