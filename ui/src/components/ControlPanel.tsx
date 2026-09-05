import React, { useState } from 'react';
import { Sliders, RefreshCw, Check } from 'lucide-react';
import { telemetryClient } from '../services/socket';

interface ControlPanelProps {
  currentController: string;
  onConfigApplied?: () => void;
}

export const ControlPanel: React.FC<ControlPanelProps> = ({ currentController, onConfigApplied }) => {
  const [controllerType, setControllerType] = useState<string>(
    currentController.toLowerCase().includes('pid') ? 'pid' : 'adrc'
  );
  const [targetPressure, setTargetPressure] = useState<number>(35.0);
  const [spindleRpm, setSpindleRpm] = useState<number>(3500.0);
  const [hardnessHrc, setHardnessHrc] = useState<number>(42.0);
  const [bypassScale, setBypassScale] = useState<number>(1.0);
  const [saved, setSaved] = useState<boolean>(false);

  const handleApply = async () => {
    await telemetryClient.updateConfig({
      controller_type: controllerType,
      target_pressure_bar: targetPressure,
      spindle_rpm_nominal: spindleRpm,
      material_hardness_hrc: hardnessHrc,
      bypass_orifice_area_scale: bypassScale,
    });
    setSaved(true);
    setTimeout(() => setSaved(false), 2000);
    onConfigApplied?.();
  };

  return (
    <div className="bg-card border border-border rounded p-6 shadow-sm max-w-4xl">
      <div className="flex items-center justify-between border-b border-border pb-4 mb-6">
        <div>
          <h2 className="text-base font-bold text-slate-900 tracking-tight flex items-center gap-2">
            <Sliders className="w-4 h-4 text-primary" />
            CONTROL LAW & PHYSICAL PLANT TUNING
          </h2>
          <p className="text-xs text-slate-500 font-mono mt-0.5">
            Configure discrete controller parameters, material properties, and hydraulic stiffness.
          </p>
        </div>
        <button
          onClick={handleApply}
          className={`flex items-center gap-2 px-4 py-2 rounded text-xs font-mono font-semibold transition shadow-sm ${
            saved
              ? 'bg-emerald-600 text-white'
              : 'bg-primary hover:bg-primary-dark text-white'
          }`}
        >
          {saved ? <Check className="w-3.5 h-3.5" /> : <RefreshCw className="w-3.5 h-3.5" />}
          {saved ? 'APPLIED' : 'APPLY CONFIGURATION'}
        </button>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        {/* Section 1: Controller Architecture */}
        <div className="space-y-4">
          <label className="block text-xs font-mono font-bold uppercase text-slate-700">
            Control Algorithm Architecture
          </label>
          <div className="grid grid-cols-2 gap-3">
            <button
              onClick={() => setControllerType('adrc')}
              className={`p-3 rounded border text-left transition ${
                controllerType === 'adrc'
                  ? 'border-primary bg-blue-50/50 ring-1 ring-primary'
                  : 'border-border bg-slate-50/50 hover:bg-slate-100/60'
              }`}
            >
              <div className="font-semibold text-xs text-slate-900">Cascade LADRC</div>
              <div className="text-[10px] text-slate-500 mt-1">
                3rd-order LESO with Asymmetric Reference Governor. Total disturbance rejection.
              </div>
            </button>
            <button
              onClick={() => setControllerType('pid')}
              className={`p-3 rounded border text-left transition ${
                controllerType === 'pid'
                  ? 'border-primary bg-blue-50/50 ring-1 ring-primary'
                  : 'border-border bg-slate-50/50 hover:bg-slate-100/60'
              }`}
            >
              <div className="font-semibold text-xs text-slate-900">Baseline PID</div>
              <div className="text-[10px] text-slate-500 mt-1">
                Gain-scheduled PI/PID with conditional tracking anti-windup.
              </div>
            </button>
          </div>

          <div className="pt-2">
            <div className="flex justify-between text-xs font-mono text-slate-700 mb-1">
              <span>TARGET CUTTING PRESSURE</span>
              <span className="font-bold text-primary">{targetPressure.toFixed(1)} bar</span>
            </div>
            <input
              type="range"
              min="15.0"
              max="60.0"
              step="1.0"
              value={targetPressure}
              onChange={(e) => setTargetPressure(parseFloat(e.target.value))}
              className="w-full accent-primary h-1.5 bg-slate-200 rounded"
            />
          </div>

          <div>
            <div className="flex justify-between text-xs font-mono text-slate-700 mb-1">
              <span>NOMINAL SPINDLE SPEED</span>
              <span className="font-bold text-primary">{spindleRpm.toFixed(0)} RPM</span>
            </div>
            <input
              type="range"
              min="2000"
              max="5000"
              step="100"
              value={spindleRpm}
              onChange={(e) => setSpindleRpm(parseFloat(e.target.value))}
              className="w-full accent-primary h-1.5 bg-slate-200 rounded"
            />
          </div>
        </div>

        {/* Section 2: Physical Workpiece & Hydraulic Parameters */}
        <div className="space-y-4">
          <label className="block text-xs font-mono font-bold uppercase text-slate-700">
            Workpiece & Plant Physics
          </label>

          <div>
            <div className="flex justify-between text-xs font-mono text-slate-700 mb-1">
              <span>INCONEL 718 HARDNESS</span>
              <span className="font-bold text-slate-900">{hardnessHrc.toFixed(1)} HRC</span>
            </div>
            <input
              type="range"
              min="36.0"
              max="48.0"
              step="0.5"
              value={hardnessHrc}
              onChange={(e) => setHardnessHrc(parseFloat(e.target.value))}
              className="w-full accent-slate-700 h-1.5 bg-slate-200 rounded"
            />
            <div className="text-[10px] text-slate-400 font-mono mt-1">
              Scales specific cutting energy k_c from 2700 MPa to 3700 MPa.
            </div>
          </div>

          <div>
            <div className="flex justify-between text-xs font-mono text-slate-700 mb-1">
              <span>CALIBRATED BYPASS ORIFICE SCALE</span>
              <span className="font-bold text-slate-900">{bypassScale.toFixed(2)}x</span>
            </div>
            <input
              type="range"
              min="0.2"
              max="3.0"
              step="0.1"
              value={bypassScale}
              onChange={(e) => setBypassScale(parseFloat(e.target.value))}
              className="w-full accent-slate-700 h-1.5 bg-slate-200 rounded"
            />
            <div className="text-[10px] text-slate-400 font-mono mt-1">
              Modulates passive pressure relaxation rate through calibrated leak.
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
