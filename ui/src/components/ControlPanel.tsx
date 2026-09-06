import React, { useEffect, useState } from 'react';
import { Sliders, RefreshCw, Check, AlertTriangle } from 'lucide-react';
import { telemetryClient } from '../services/socket';
import {
  barToPsi,
  ftLbfToNm,
  nmToFtLbf,
  psiToBar,
} from '../utils/units';

interface ControlPanelProps {
  currentController: string;
  currentTargetTorqueNm: number;
  currentPressureCeilingBar: number;
  currentSpindleRpm: number;
  currentAchievableTorqueNm: number;
  controlLimited: boolean;
  onConfigApplied?: () => void;
}

export const ControlPanel: React.FC<ControlPanelProps> = ({
  currentController,
  currentTargetTorqueNm,
  currentPressureCeilingBar,
  currentSpindleRpm,
  currentAchievableTorqueNm,
  controlLimited,
  onConfigApplied,
}) => {
  const [controllerType, setControllerType] = useState<string>(
    currentController.toLowerCase().includes('pid') ? 'pid' : 'adrc',
  );
  const [targetToBFtLbf, setTargetToBFtLbf] = useState<number>(
    nmToFtLbf(currentTargetTorqueNm),
  );
  const [pressureCeilingPsi, setPressureCeilingPsi] = useState<number>(
    barToPsi(currentPressureCeilingBar),
  );
  const [spindleRpm, setSpindleRpm] = useState<number>(currentSpindleRpm);
  const [hardnessHrc, setHardnessHrc] = useState<number>(42.0);
  const [bypassScale, setBypassScale] = useState<number>(1.0);
  const [saved, setSaved] = useState<boolean>(false);
  const [appliedToB, setAppliedToB] = useState<number>(
    nmToFtLbf(currentTargetTorqueNm),
  );
  const [appliedMaxToB, setAppliedMaxToB] = useState<number>(
    nmToFtLbf(currentAchievableTorqueNm),
  );
  const [appliedLimited, setAppliedLimited] = useState<boolean>(controlLimited);

  useEffect(() => {
    setAppliedToB(nmToFtLbf(currentTargetTorqueNm));
    setAppliedMaxToB(nmToFtLbf(currentAchievableTorqueNm));
    setAppliedLimited(controlLimited);
  }, [currentTargetTorqueNm, currentAchievableTorqueNm, controlLimited]);

  const handleApply = async () => {
    const applied = await telemetryClient.updateConfig({
      controller_type: controllerType,
      target_pressure_bar: psiToBar(pressureCeilingPsi),
      target_torque_nm: ftLbfToNm(targetToBFtLbf),
      spindle_rpm_nominal: spindleRpm,
      material_hardness_hrc: hardnessHrc,
      bypass_orifice_area_scale: bypassScale,
    });
    if (applied) {
      setAppliedToB(nmToFtLbf(applied.target_torque_nm));
      setAppliedMaxToB(nmToFtLbf(applied.achievable_torque_nm));
      setAppliedLimited(applied.control_limited);
      setTargetToBFtLbf(nmToFtLbf(applied.target_torque_nm));
      setPressureCeilingPsi(barToPsi(applied.pressure_ceiling_bar));
      setSpindleRpm(applied.spindle_rpm_nominal);
      setSaved(true);
      setTimeout(() => setSaved(false), 2000);
      onConfigApplied?.();
    }
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
            ToB setpoint → WOB/pressure feedforward + trim → pump-map feedforward + PID/LADRC.
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
          {saved ? (
            <Check className="w-3.5 h-3.5" />
          ) : (
            <RefreshCw className="w-3.5 h-3.5" />
          )}
          {saved ? 'APPLIED' : 'APPLY CONFIGURATION'}
        </button>
      </div>

      {appliedLimited && (
        <div className="mb-5 rounded border border-rose-300 bg-rose-50 px-3 py-2 text-[10px] font-mono text-rose-900 flex items-center gap-2">
          <AlertTriangle className="w-4 h-4 shrink-0" />
          <span>
            ACTIVE ToB SP {appliedToB.toFixed(2)} ft·lbf is above the nominal
            hydraulic capability ≈ {appliedMaxToB.toFixed(2)} ft·lbf at the selected
            pressure ceiling. The controller will saturate and the monitor will flag LIMITED.
          </span>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <div className="space-y-4">
          <label className="block text-xs font-mono font-bold uppercase text-slate-700">
            Control Architecture
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
                Shared ToB + pump feedforward; ESO rejects pressure-rate disturbances.
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
                Same ToB and pump feedforward; conventional pressure PID trim.
              </div>
            </button>
          </div>

          <div className="pt-2">
            <div className="flex justify-between text-xs font-mono text-slate-700 mb-1">
              <span>DESIRED TORQUE ON BIT (ToB)</span>
              <span className="font-bold text-primary">{targetToBFtLbf.toFixed(1)} ft·lbf</span>
            </div>
            <input
              type="range"
              min="1.0"
              max="5.0"
              step="0.1"
              value={targetToBFtLbf}
              onChange={(e) => setTargetToBFtLbf(parseFloat(e.target.value))}
              className="w-full accent-primary h-1.5 bg-slate-200 rounded"
            />
            <div className="text-[10px] text-slate-400 font-mono mt-1 flex justify-between">
              <span>Primary milling-load setpoint; live and bumpless within one controller type.</span>
              <span className="font-bold text-emerald-700">
                ACTIVE: {appliedToB.toFixed(1)} ft·lbf
              </span>
            </div>
          </div>

          <div>
            <div className="flex justify-between text-xs font-mono text-slate-700 mb-1">
              <span>HYDRAULIC PRESSURE CEILING</span>
              <span className="font-bold text-primary">{pressureCeilingPsi.toFixed(0)} psi</span>
            </div>
            <input
              type="range"
              min="250"
              max="850"
              step="25"
              value={pressureCeilingPsi}
              onChange={(e) => setPressureCeilingPsi(parseFloat(e.target.value))}
              className="w-full accent-primary h-1.5 bg-slate-200 rounded"
            />
            <div className="text-[10px] text-slate-400 font-mono mt-1">
              Nominal ToB capability at current ceiling ≈ {appliedMaxToB.toFixed(2)} ft·lbf.
            </div>
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
              Hardness now affects both cutting torque sensitivity and physical ROP.
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
              Scales the calibrated nominal bypass; Apply no longer swaps in another plant.
            </div>
          </div>

          <div className="rounded border border-amber-200 bg-amber-50/60 p-3 text-[10px] text-amber-900 font-mono leading-relaxed">
            Slow material evolution is time-compressed for the workstation demo. Pressure,
            mechanics, motor and controller equations stay on the normal simulation clock.
            Disturbances are perturbations around a causal nominal WOB→ToB/ROP plant.
          </div>
        </div>
      </div>
    </div>
  );
};
