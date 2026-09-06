/**
 * WebSocket client service connecting to FastAPI backend telemetry stream.
 */

export interface TelemetryData {
  timestamp: number;
  is_running: boolean;
  operating_mode: string;
  controller_type: string;

  pressure_bar: number;
  pressure_true_bar: number;
  spindle_rpm: number;
  pump_rpm: number;
  pump_cmd_rpm: number;
  spindle_torque_est: number;
  spindle_torque_true: number;
  target_torque_nm: number;
  wob_soft_sensor: number;
  axial_cutting_force_true: number;

  rod_position_mm: number;
  penetration_depth_mm: number;
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

type TelemetryCallback = (data: TelemetryData) => void;

class TelemetrySocketClient {
  private socket: WebSocket | null = null;
  private listeners: Set<TelemetryCallback> = new Set();
  private reconnectInterval: number = 2000;
  private url: string = 'ws://127.0.0.1:8000/ws/telemetry';

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
    return () => {
      this.listeners.delete(callback);
    };
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
  }): Promise<void> {
    try {
      await fetch('http://127.0.0.1:8000/api/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config),
      });
    } catch (err) {
      console.error('[API] Failed to update config:', err);
    }
  }
}

export const telemetryClient = new TelemetrySocketClient();
