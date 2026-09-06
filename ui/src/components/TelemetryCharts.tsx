import React, { useRef, useEffect } from 'react';
import { Download } from 'lucide-react';
import {
  barToPsi,
  mpsToMmPerMin,
  nmToFtLbf,
} from '../utils/units';

interface ChartPoint {
  t: number;
  pressure: number;       // backend bar
  torque: number;         // backend N*m
  targetTorque: number;   // backend N*m
  wob: number;            // backend N
  spindleRpm: number;
  pumpRpm: number;
  ropMps: number;         // backend m/s
}

interface TelemetryChartsProps {
  history: ChartPoint[];
}

export const TelemetryCharts: React.FC<TelemetryChartsProps> = ({ history }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || history.length < 2) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const width = canvas.parentElement?.clientWidth || 600;
    const height = 570;
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, width, height);

    const laneHeight = 118;
    const gap = 18;
    const padLeft = 62;
    const padRight = 45;
    const plotWidth = width - padLeft - padRight;
    const tMin = history[0].t;
    const tMax = Math.max(tMin + 0.5, history[history.length - 1].t);
    const getX = (t: number) =>
      padLeft + ((t - tMin) / (tMax - tMin)) * plotWidth;

    const drawLaneGrid = (
      top: number,
      yLabel: string,
      yMin: number,
      yMax: number,
      unit: string,
      decimals = 0,
    ) => {
      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1;
      ctx.strokeRect(padLeft, top, plotWidth, laneHeight);

      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(padLeft, top + laneHeight / 2);
      ctx.lineTo(padLeft + plotWidth, top + laneHeight / 2);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillStyle = '#64748b';
      ctx.font = '10px "JetBrains Mono", monospace';
      ctx.textAlign = 'right';
      ctx.fillText(yMax.toFixed(decimals), padLeft - 6, top + 10);
      ctx.fillText(
        ((yMin + yMax) / 2).toFixed(decimals),
        padLeft - 6,
        top + laneHeight / 2 + 3,
      );
      ctx.fillText(yMin.toFixed(decimals), padLeft - 6, top + laneHeight - 2);

      ctx.textAlign = 'left';
      ctx.fillStyle = '#0f172a';
      ctx.font = '600 11px Inter, sans-serif';
      ctx.fillText(`${yLabel} [${unit}]`, padLeft + 8, top + 16);
    };

    const drawSeries = (
      top: number,
      yMin: number,
      yMax: number,
      value: (p: ChartPoint) => number,
      stroke: string,
      lineWidth = 2,
      dashed = false,
    ) => {
      ctx.strokeStyle = stroke;
      ctx.lineWidth = lineWidth;
      if (dashed) ctx.setLineDash([5, 4]);
      ctx.beginPath();
      history.forEach((pt, idx) => {
        const x = getX(pt.t);
        const v = Math.min(yMax, Math.max(yMin, value(pt)));
        const y =
          top +
          laneHeight -
          ((v - yMin) / Math.max(1e-9, yMax - yMin)) * laneHeight;
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    };

    const top1 = 8;
    const top2 = top1 + laneHeight + gap;
    const top3 = top2 + laneHeight + gap;
    const top4 = top3 + laneHeight + gap;

    const pressureMaxPsi = barToPsi(70);
    drawLaneGrid(top1, 'HYDRAULIC PRESSURE', 0, pressureMaxPsi, 'psi');
    drawSeries(
      top1,
      0,
      pressureMaxPsi,
      (p) => barToPsi(p.pressure),
      '#2563eb',
    );

    const torqueMaxFtLbf = nmToFtLbf(10);
    drawLaneGrid(
      top2,
      'TORQUE ON BIT (ToB)',
      0,
      torqueMaxFtLbf,
      'ft·lbf',
      1,
    );
    drawSeries(
      top2,
      0,
      torqueMaxFtLbf,
      (p) => nmToFtLbf(p.targetTorque),
      '#94a3b8',
      1.3,
      true,
    );
    drawSeries(
      top2,
      0,
      torqueMaxFtLbf,
      (p) => nmToFtLbf(p.torque),
      '#dc2626',
    );

    drawLaneGrid(top3, 'DRIVE SPEEDS (SPINDLE & PUMP)', 0, 5000, 'RPM');
    drawSeries(top3, 0, 5000, (p) => p.spindleRpm, '#334155', 1.8);
    drawSeries(top3, 0, 5000, (p) => p.pumpRpm, '#d97706', 1.8);

    const maxRopSeen = Math.max(
      ...history.map((p) => mpsToMmPerMin(p.ropMps)),
      0.08,
    );
    const ropMax = Math.max(0.12, Math.ceil(maxRopSeen / 0.05) * 0.05);
    drawLaneGrid(top4, 'PHYSICAL RATE OF PENETRATION', 0, ropMax, 'mm/min', 2);
    drawSeries(
      top4,
      0,
      ropMax,
      (p) => mpsToMmPerMin(p.ropMps),
      '#7c3aed',
      2.2,
    );
  }, [history]);

  const handleExportPNG = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const link = document.createElement('a');
    link.download = `milling_telemetry_${Date.now()}.png`;
    link.href = canvas.toDataURL('image/png');
    link.click();
  };

  return (
    <div className="bg-card border border-border rounded p-4 shadow-sm flex flex-col justify-between">
      <div className="flex items-center justify-between mb-3 border-b border-border pb-2.5">
        <div>
          <div className="text-xs font-bold uppercase font-mono tracking-wider text-slate-800">
            MULTICHANNEL TELEMETRY RECORDER
          </div>
          <div className="text-[11px] text-slate-400 font-mono">
            4× waveform history • ROP is physical, material geometry is demo-time compressed
          </div>
        </div>
        <button
          onClick={handleExportPNG}
          className="flex items-center gap-1.5 px-3 py-1.5 rounded border border-border bg-slate-50 hover:bg-slate-100 text-slate-700 text-xs font-mono font-medium transition"
        >
          <Download className="w-3.5 h-3.5" />
          Export PNG
        </button>
      </div>

      <div className="w-full overflow-hidden flex justify-center">
        <canvas ref={canvasRef} className="block rounded" />
      </div>

      <div className="flex flex-wrap items-center justify-center gap-5 mt-3 text-[11px] font-mono text-slate-500 border-t border-border pt-2">
        <span>Pressure [psi]</span>
        <span>ToB [ft·lbf] — dashed = SP</span>
        <span>Spindle / Pump [RPM]</span>
        <span>ROP [mm/min]</span>
      </div>
    </div>
  );
};
