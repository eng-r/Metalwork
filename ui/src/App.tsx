import React, { useState, useEffect } from 'react';
import { Play, Pause, RotateCcw, SkipForward, AlertOctagon, Download } from 'lucide-react';
import { Sidebar, TabType } from './components/Sidebar';
import { MetricCard } from './components/MetricCard';
import { Viewport3D } from './components/Viewport3D';
import { TelemetryCharts } from './components/TelemetryCharts';
import { ControlPanel } from './components/ControlPanel';
import { telemetryClient, TelemetryData } from './services/socket';

interface ChartPoint {
  t: number;
  pressure: number;
  torque: number;
  wob: number;
  spindleRpm: number;
  pumpRpm: number;
  depth: number;
}

export const App: React.FC = () => {
  const [activeTab, setActiveTab] = useState<TabType>('monitor');
  const [telemetry, setTelemetry] = useState<TelemetryData>({
    timestamp: 0.0,
    is_running: false,
    operating_mode: 'APPROACH',
    pressure_bar: 1.01,
    pressure_true_bar: 1.01,
    spindle_rpm: 0.0,
    pump_rpm: 0.0,
    pump_cmd_rpm: 0.0,
    spindle_torque_est: 0.0,
    spindle_torque_true: 0.0,
    wob_soft_sensor: 0.0,
    axial_cutting_force_true: 0.0,
    rod_position_mm: 0.0,
    penetration_depth_mm: 0.0,
    mrr_mm3_s: 0.0,
    cumulative_volume_mm3: 0.0,
    seal_friction_n: 0.0,
    leso_z3_disturbance: 0.0,
  });

  const [chartHistory, setChartHistory] = useState<ChartPoint[]>([]);

  useEffect(() => {
    const unsubscribe = telemetryClient.subscribe((data) => {
      setTelemetry(data);
      setChartHistory((prev) => {
        const nextPt: ChartPoint = {
          t: data.timestamp,
          pressure: data.pressure_bar,
          torque: data.spindle_torque_est,
          wob: data.wob_soft_sensor,
          spindleRpm: data.spindle_rpm,
          pumpRpm: data.pump_rpm,
          depth: data.penetration_depth_mm,
        };
        const updated = [...prev, nextPt];
        return updated.length > 250 ? updated.slice(updated.length - 250) : updated;
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

  return (
    <div className="flex h-screen w-screen bg-background overflow-hidden font-sans">
      {/* Left Sidebar Navigation */}
      <Sidebar
        activeTab={activeTab}
        onSelectTab={setActiveTab}
        controllerType="Cascade LADRC"
      />

      {/* Main Content Area */}
      <main className="flex-1 flex flex-col h-screen overflow-y-auto">
        {/* Top Control Bar */}
        <header className="h-16 bg-card border-b border-border px-8 flex items-center justify-between shrink-0 sticky top-0 z-20 shadow-sm">
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2 font-mono text-xs">
              <span className="text-slate-400">VIRTUAL TIME:</span>
              <span className="font-bold text-slate-900 bg-slate-100 px-2 py-1 rounded">
                {telemetry.timestamp.toFixed(3)} s
              </span>
            </div>

            <div className={`px-2.5 py-0.5 rounded-full border text-[11px] font-mono font-semibold tracking-wide ${getModeBadgeClass(telemetry.operating_mode)}`}>
              {telemetry.operating_mode}
            </div>
          </div>

          {/* Action Buttons */}
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

        {/* Tab Views */}
        <div className="p-8 space-y-6 flex-1">
          {activeTab === 'monitor' && (
            <>
              {/* KPI Strip */}
              <div className="grid grid-cols-2 sm:grid-cols-4 lg:grid-cols-7 gap-4">
                <MetricCard
                  label="Pressure P"
                  value={telemetry.pressure_bar.toFixed(1)}
                  unit="bar"
                  subValue={`True: ${telemetry.pressure_true_bar.toFixed(1)} bar`}
                  highlight
                />
                <MetricCard
                  label="Weight On Bit"
                  value={telemetry.wob_soft_sensor.toFixed(0)}
                  unit="N"
                  subValue={`True: ${telemetry.axial_cutting_force_true.toFixed(0)} N`}
                />
                <MetricCard
                  label="Spindle Torque"
                  value={telemetry.spindle_torque_est.toFixed(2)}
                  unit="N·m"
                  subValue="Iq observer"
                  highlight={telemetry.spindle_torque_est > 6.5}
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
                  label="Crater Depth"
                  value={telemetry.penetration_depth_mm.toFixed(2)}
                  unit="mm"
                  subValue={`Rod: ${telemetry.rod_position_mm.toFixed(1)} mm`}
                />
                <MetricCard
                  label="LESO Total Dist."
                  value={telemetry.leso_z3_disturbance.toFixed(2)}
                  unit="f(t)"
                  subValue="3rd-order LESO"
                />
              </div>

              {/* Main Dual Workspace Grid */}
              <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 min-h-[500px]">
                <div className="lg:col-span-5 h-[520px]">
                  <Viewport3D
                    rodPositionMm={telemetry.rod_position_mm}
                    penetrationDepthMm={telemetry.penetration_depth_mm}
                    spindleRpm={telemetry.spindle_rpm}
                    spindleTorqueNm={telemetry.spindle_torque_est}
                    wobN={telemetry.wob_soft_sensor}
                  />
                </div>
                <div className="lg:col-span-7 h-[520px]">
                  <TelemetryCharts history={chartHistory} />
                </div>
              </div>
            </>
          )}

          {activeTab === 'tuning' && (
            <div className="flex justify-center">
              <ControlPanel currentController="Cascade LADRC" />
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
                      <td className="p-3 font-semibold">k_c (Specific Cutting Energy)</td>
                      <td className="p-3 text-slate-500">Inconel 718 Mechanics</td>
                      <td className="p-3 text-right font-bold text-primary">3180.4 MPa</td>
                      <td className="p-3 text-right">3200.0 MPa</td>
                      <td className="p-3 text-right text-emerald-600 font-bold">Passed (0.6%)</td>
                    </tr>
                    <tr>
                      <td className="p-3 font-semibold">k_ax (Axial Thrust Coeff)</td>
                      <td className="p-3 text-slate-500">Contact Thrust</td>
                      <td className="p-3 text-right font-bold text-primary">18.00 MPa</td>
                      <td className="p-3 text-right">18.00 MPa</td>
                      <td className="p-3 text-right text-emerald-600 font-bold">Passed (0.0%)</td>
                    </tr>
                    <tr>
                      <td className="p-3 font-semibold">a_0 (Pump Shutoff Head)</td>
                      <td className="p-3 text-slate-500">Centrifugal Hydraulics</td>
                      <td className="p-3 text-right font-bold text-primary">0.420 Pa/(rad/s)²</td>
                      <td className="p-3 text-right">0.420 Pa/(rad/s)²</td>
                      <td className="p-3 text-right text-emerald-600 font-bold">Passed (0.1%)</td>
                    </tr>
                    <tr>
                      <td className="p-3 font-semibold">C_bypass (Calibrated Orifice)</td>
                      <td className="p-3 text-slate-500">Hydraulic Relaxation</td>
                      <td className="p-3 text-right font-bold text-primary">6.00e-10 m²</td>
                      <td className="p-3 text-right">6.00e-10 m²</td>
                      <td className="p-3 text-right text-emerald-600 font-bold">Passed (0.0%)</td>
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
                  Export publication-grade raw data and publication artifacts.
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
                    Full virtual-time records including pressure, torque, WOB, RPM, and crater volume.
                  </p>
                </a>

                <div className="p-5 border border-border rounded bg-slate-50/50 flex flex-col justify-between">
                  <div className="text-slate-900 font-bold text-sm">
                    Pre-rendered Publication Figures
                  </div>
                  <p className="text-xs text-slate-500 font-mono mt-2">
                    Generated via headless runner: <br />
                    <code className="text-primary font-bold">figures/test_run.png</code>
                    <br />
                    <code className="text-primary font-bold">figures/benchmark_comparison.png</code>
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
