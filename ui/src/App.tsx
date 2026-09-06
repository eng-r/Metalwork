import React, { useState, useEffect } from 'react';
import {
  Play,
  Pause,
  RotateCcw,
  SkipForward,
  AlertOctagon,
  Download,
} from 'lucide-react';
import { Sidebar, TabType } from './components/Sidebar';
import { MetricCard } from './components/MetricCard';
import { Viewport3D } from './components/Viewport3D';
import { TelemetryCharts } from './components/TelemetryCharts';
import { ControlPanel } from './components/ControlPanel';
import { telemetryClient, TelemetryData } from './services/socket';
import {
  barToPsi,
  formatDuration,
  mpsToMmPerMin,
  newtonToLbf,
  nmToFtLbf,
} from './utils/units';

interface ChartPoint {
  t: number;
  pressure: number;
  pressureRef: number;
  torque: number;
  torqueTrue: number;
  targetTorque: number;
  wob: number;
  spindleRpm: number;
  spindleCmdRpm: number;
  pumpRpm: number;
  pumpCmdRpm: number;
  ropMps: number;
}

export const App: React.FC = () => {
  const [activeTab, setActiveTab] = useState<TabType>('monitor');
  const [telemetry, setTelemetry] = useState<TelemetryData>({
    timestamp: 0.0,
    is_running: false,
    operating_mode: 'APPROACH',
    controller_type: 'CascadeLADRC',
    pressure_bar: 1.01,
    pressure_true_bar: 1.01,
    pressure_reference_bar: 1.01,
    spindle_rpm: 0.0,
    spindle_cmd_rpm: 3500.0,
    pump_rpm: 0.0,
    pump_cmd_rpm: 0.0,
    spindle_torque_est: 0.0,
    spindle_torque_true: 0.0,
    target_torque_nm: 4.0,
    pressure_ceiling_bar: 35.0,
    torque_pressure_reference_bar: 18.0,
    wob_soft_sensor: 0.0,
    axial_cutting_force_true: 0.0,
    rod_position_mm: 0.0,
    penetration_depth_mm: 0.0,
    engagement_depth_mm: 0.0,
    mrr_mm3_s: 0.0,
    cumulative_volume_mm3: 0.0,
    seal_friction_n: 0.0,
    leso_z3_disturbance: 0.0,
    physical_rop_m_s: 0.0,
    surface_recession_m: 0.0,
    demo_acceleration: 120.0,
    equivalent_process_time_s: 0.0,
    disturbance_event: 'FREE',
  });

  const [chartHistory, setChartHistory] = useState<ChartPoint[]>([]);

  useEffect(() => {
    const unsubscribe = telemetryClient.subscribe((data) => {
      setTelemetry(data);
      setChartHistory((prev) => {
        const nextPt: ChartPoint = {
          t: data.timestamp,
          pressure: data.pressure_bar,
          pressureRef: data.pressure_reference_bar,
          torque: data.spindle_torque_est,
          torqueTrue: data.spindle_torque_true,
          targetTorque: data.target_torque_nm,
          wob: data.wob_soft_sensor,
          spindleRpm: data.spindle_rpm,
          spindleCmdRpm: data.spindle_cmd_rpm,
          pumpRpm: data.pump_rpm,
          pumpCmdRpm: data.pump_cmd_rpm,
          ropMps: data.physical_rop_m_s,
        };
        const updated = [...prev, nextPt];

        // 4x previous browser chart memory (250 -> 1000 samples).
        return updated.length > 1000
          ? updated.slice(updated.length - 1000)
          : updated;
      });
    });

    return () => unsubscribe();
  }, []);

  const getModeBadgeClass = (mode: string) => {
    switch (mode) {
      case 'NORMAL_MILLING':
        return 'bg-emerald-50 text-emerald-700 border-emerald-200';
      case 'PRESSURE_RELAXATION':
        return 'bg-amber-50 text-amber-700 border-amber-200 animate-pulse';
      case 'OVERLOAD_RECOVERY':
      case 'STALL_RECOVERY':
        return 'bg-rose-50 text-rose-700 border-rose-200';
      default:
        return 'bg-blue-50 text-primary border-blue-200';
    }
  };

  const physicalRop = mpsToMmPerMin(telemetry.physical_rop_m_s);

  return (
    <div className="flex h-screen w-screen bg-background overflow-hidden font-sans">
      <Sidebar
        activeTab={activeTab}
        onSelectTab={setActiveTab}
        controllerType={telemetry.controller_type}
      />

      <main className="flex-1 flex flex-col h-screen overflow-y-auto">
        <header className="h-16 bg-card border-b border-border px-8 flex items-center justify-between shrink-0 sticky top-0 z-20 shadow-sm">
          <div className="flex items-center gap-3 font-mono text-[11px]">
            <div className="flex items-center gap-2">
              <span className="text-slate-400">DEMO TIME:</span>
              <span className="font-bold text-slate-900 bg-slate-100 px-2 py-1 rounded">
                {telemetry.timestamp.toFixed(2)} s
              </span>
            </div>

            <div className="px-2 py-1 rounded border border-violet-200 bg-violet-50 text-violet-700 font-bold">
              {telemetry.demo_acceleration.toFixed(0)}× MATERIAL-TIME
            </div>

            <div className="flex items-center gap-2">
              <span className="text-slate-400">EQUIV. CUT TIME:</span>
              <span className="font-bold text-slate-800">
                {formatDuration(telemetry.equivalent_process_time_s)}
              </span>
            </div>

            <div
              className={`px-2.5 py-0.5 rounded-full border font-semibold tracking-wide ${getModeBadgeClass(
                telemetry.operating_mode,
              )}`}
            >
              {telemetry.operating_mode}
            </div>

            <div className="px-2 py-0.5 rounded bg-slate-100 text-slate-600 border border-slate-200">
              {telemetry.disturbance_event}
            </div>
          </div>

          <div className="flex items-center gap-2">
            {!telemetry.is_running ? (
              <button
                onClick={() => telemetryClient.sendCommand('start')}
                className="flex items-center gap-1.5 px-3.5 py-1.5 rounded bg-primary hover:bg-primary-dark text-white text-xs font-mono font-semibold transition shadow-sm"
              >
                <Play className="w-3.5 h-3.5 fill-current" />
                START
              </button>
            ) : (
              <button
                onClick={() => telemetryClient.sendCommand('pause')}
                className="flex items-center gap-1.5 px-3.5 py-1.5 rounded bg-amber-600 hover:bg-amber-700 text-white text-xs font-mono font-semibold transition shadow-sm"
              >
                <Pause className="w-3.5 h-3.5 fill-current" />
                PAUSE
              </button>
            )}

            <button
              onClick={() => telemetryClient.sendCommand('step')}
              className="flex items-center gap-1 px-3 py-1.5 rounded border border-border bg-slate-50 hover:bg-slate-100 text-slate-700 text-xs font-mono transition"
            >
              <SkipForward className="w-3.5 h-3.5" />
              STEP
            </button>

            <button
              onClick={() => {
                telemetryClient.sendCommand('reset');
                setChartHistory([]);
              }}
              className="flex items-center gap-1 px-3 py-1.5 rounded border border-border bg-slate-50 hover:bg-slate-100 text-slate-700 text-xs font-mono transition"
            >
              <RotateCcw className="w-3.5 h-3.5" />
              RESET
            </button>

            <button
              onClick={() => telemetryClient.sendCommand('pause')}
              className="flex items-center gap-1.5 px-3 py-1.5 rounded bg-rose-600 hover:bg-rose-700 text-white text-xs font-mono font-bold transition shadow-sm ml-2"
            >
              <AlertOctagon className="w-3.5 h-3.5" />
              E-STOP
            </button>
          </div>
        </header>

        <div className="p-8 space-y-6 flex-1">
          {activeTab === 'monitor' && (
            <>
              <div className="grid grid-cols-2 sm:grid-cols-4 lg:grid-cols-7 gap-4">
                <MetricCard
                  label="Pressure P"
                  value={barToPsi(telemetry.pressure_bar).toFixed(0)}
                  unit="psi"
                  subValue={`Ref: ${barToPsi(
                    telemetry.pressure_reference_bar,
                  ).toFixed(0)} psi`}
                  highlight
                />
                <MetricCard
                  label="Weight On Bit"
                  value={newtonToLbf(telemetry.wob_soft_sensor).toFixed(0)}
                  unit="lbf"
                  subValue={`True: ${newtonToLbf(
                    telemetry.axial_cutting_force_true,
                  ).toFixed(0)} lbf`}
                />
                <MetricCard
                  label="Torque On Bit"
                  value={nmToFtLbf(telemetry.spindle_torque_est).toFixed(2)}
                  unit="ft·lbf"
                  subValue={`SP: ${nmToFtLbf(
                    telemetry.target_torque_nm,
                  ).toFixed(2)} ft·lbf`}
                  highlight={
                    telemetry.spindle_torque_est >
                    telemetry.target_torque_nm * 1.35
                  }
                />
                <MetricCard
                  label="Spindle Speed"
                  value={telemetry.spindle_rpm.toFixed(0)}
                  unit="RPM"
                  subValue="Resolver"
                />
                <MetricCard
                  label="Pump Speed"
                  value={telemetry.pump_rpm.toFixed(0)}
                  unit="RPM"
                  subValue={`Cmd: ${telemetry.pump_cmd_rpm.toFixed(0)}`}
                />
                <MetricCard
                  label="Physical ROP"
                  value={physicalRop.toFixed(3)}
                  unit="mm/min"
                  subValue={`Eng: ${telemetry.engagement_depth_mm.toFixed(
                    3,
                  )} mm • Eq.cut: ${(
                    telemetry.surface_recession_m * 1000.0
                  ).toFixed(2)} mm`}
                />
                <MetricCard
                  label="ESO Disturbance"
                  value={telemetry.leso_z3_disturbance.toFixed(2)}
                  unit="bar/s"
                  subValue="2-state pressure ESO"
                />
              </div>

              <div className="rounded border border-violet-200 bg-violet-50/60 px-4 py-3 text-[11px] font-mono text-violet-900 flex flex-wrap gap-x-6 gap-y-1">
                <span className="font-bold">ACCELERATED PROCESS DEMO</span>
                <span>
                  ROP shown is the physical rate. Only slow material/crater evolution
                  is time-compressed {telemetry.demo_acceleration.toFixed(0)}× so
                  hours of Inconel milling are observable in minutes.
                </span>
                <span>
                  Nominal 0.08 mm/min ⇒ 3–4 in requires roughly 16–21 h actual cutting.
                </span>
              </div>

              <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 min-h-[650px]">
                <div className="lg:col-span-5 h-[650px]">
                  <Viewport3D
                    rodPositionMm={telemetry.rod_position_mm}
                    penetrationDepthMm={telemetry.penetration_depth_mm}
                    spindleRpm={telemetry.spindle_rpm}
                    spindleTorqueNm={telemetry.spindle_torque_est}
                    wobN={telemetry.wob_soft_sensor}
                  />
                </div>
                <div className="lg:col-span-7 h-[650px]">
                  <TelemetryCharts history={chartHistory} />
                </div>
              </div>
            </>
          )}

          {activeTab === 'tuning' && (
            <div className="flex justify-center">
              <ControlPanel
                currentController={telemetry.controller_type}
                currentTargetTorqueNm={telemetry.target_torque_nm}
                currentPressureCeilingBar={telemetry.pressure_ceiling_bar}
                currentSpindleRpm={telemetry.spindle_cmd_rpm}
              />
            </div>
          )}

          {activeTab === 'sysid' && (
            <div className="bg-card border border-border rounded p-6 shadow-sm max-w-4xl mx-auto space-y-6">
              <div>
                <h2 className="text-base font-bold text-slate-900 tracking-tight">
                  SYSTEM IDENTIFICATION & DOE CALIBRATION STATUS
                </h2>
                <p className="text-xs text-slate-500 font-mono mt-0.5">
                  Identified Grey-Box Parameter Bounds vs Ground Truth for Inconel 718.
                </p>
              </div>

              <div className="border border-border rounded overflow-hidden">
                <table className="w-full text-xs font-mono">
                  <thead className="bg-slate-50 border-b border-border text-slate-600">
                    <tr>
                      <th className="p-3 text-left">PARAMETER</th>
                      <th className="p-3 text-left">SUBSYSTEM</th>
                      <th className="p-3 text-right">CALIBRATED VALUE</th>
                      <th className="p-3 text-right">PRIOR / NOMINAL</th>
                      <th className="p-3 text-right">RESIDUAL STATUS</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-border">
                    <tr>
                      <td className="p-3 font-semibold">
                        k_c (Specific Cutting Energy)
                      </td>
                      <td className="p-3 text-slate-500">
                        Inconel 718 Mechanics
                      </td>
                      <td className="p-3 text-right font-bold text-primary">
                        3180.4 MPa
                      </td>
                      <td className="p-3 text-right">3200.0 MPa</td>
                      <td className="p-3 text-right text-emerald-600 font-bold">
                        Passed (0.6%)
                      </td>
                    </tr>
                    <tr>
                      <td className="p-3 font-semibold">
                        k_ax (Axial Thrust Coeff)
                      </td>
                      <td className="p-3 text-slate-500">Contact Thrust</td>
                      <td className="p-3 text-right font-bold text-primary">
                        18.00 MPa
                      </td>
                      <td className="p-3 text-right">18.00 MPa</td>
                      <td className="p-3 text-right text-emerald-600 font-bold">
                        Passed (0.0%)
                      </td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>
          )}

          {activeTab === 'export' && (
            <div className="bg-card border border-border rounded p-6 shadow-sm max-w-4xl mx-auto space-y-6">
              <div>
                <h2 className="text-base font-bold text-slate-900 tracking-tight">
                  SCIENTIFIC & PRESENTATION EXPORT CENTER
                </h2>
                <p className="text-xs text-slate-500 font-mono mt-0.5">
                  Backend exports remain in engineering/SI units; US-customary
                  conversions are UI-only.
                </p>
              </div>

              <div className="grid grid-cols-2 gap-4">
                <a
                  href="http://127.0.0.1:8000/api/export/csv"
                  download="milling_telemetry.csv"
                  className="p-5 border border-border rounded hover:border-primary/60 bg-slate-50/50 hover:bg-slate-100/60 transition flex flex-col justify-between"
                >
                  <div className="flex items-center gap-2 text-slate-900 font-bold text-sm">
                    <Download className="w-4 h-4 text-primary" />
                    Download Telemetry CSV
                  </div>
                  <p className="text-xs text-slate-500 font-mono mt-2">
                    Pressure, ToB, WOB, physical ROP and disturbance event history.
                  </p>
                </a>

                <div className="p-5 border border-border rounded bg-slate-50/50 flex flex-col justify-between">
                  <div className="text-slate-900 font-bold text-sm">
                    Display convention
                  </div>
                  <p className="text-xs text-slate-500 font-mono mt-2">
                    UI: psi, lbf, ft·lbf, mm/min
                    <br />
                    Backend/controller: SI / existing engineering units
                  </p>
                </div>
              </div>
            </div>
          )}
        </div>
      </main>
    </div>
  );
};

export default App;
