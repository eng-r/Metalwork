import React, { useRef, useEffect } from 'react';
import { Download } from 'lucide-react';
import {
  barToPsi,
  mpsToMmPerMin,
  nmToFtLbf,
} from '../utils/units';

interface ChartPoint {
  t: number;
  pressure: number;
  pressureRef: number;
  torque: number;
  torqueTrue: number;
  targetTorque: number;
  achievableTorque: number;
  limited: boolean;
  wob: number;
  spindleRpm: number;
  spindleCmdRpm: number;
  pumpRpm: number;
  pumpCmdRpm: number;
  pumpFeedforwardRpm: number;
  ropMps: number;
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
      dash: number[] = [],
    ) => {
      ctx.strokeStyle = stroke;
      ctx.lineWidth = lineWidth;
      ctx.setLineDash(dash);
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

    const drawLegend = (
      top: number,
      entries: Array<{ label: string; color: string; dash?: number[] }>,
    ) => {
      ctx.font = '10px "JetBrains Mono", monospace';
      let x = padLeft + plotWidth - 8;
      ctx.textAlign = 'right';
      [...entries].reverse().forEach((entry) => {
        const labelWidth = ctx.measureText(entry.label).width;
        ctx.fillStyle = '#475569';
        ctx.fillText(entry.label, x, top + 16);
        x -= labelWidth + 6;
        ctx.strokeStyle = entry.color;
        ctx.lineWidth = 2;
        ctx.setLineDash(entry.dash ?? []);
        ctx.beginPath();
        ctx.moveTo(x - 22, top + 12);
        ctx.lineTo(x - 4, top + 12);
        ctx.stroke();
        ctx.setLineDash([]);
        x -= 30;
      });
      ctx.textAlign = 'left';
    };

    const top1 = 8;
    const top2 = top1 + laneHeight + gap;
    const top3 = top2 + laneHeight + gap;
    const top4 = top3 + laneHeight + gap;

    const maxPressurePsi = Math.max(
      600,
      ...history.map((p) => barToPsi(p.pressure)),
      ...history.map((p) => barToPsi(p.pressureRef)),
    );
    const pressureMaxPsi = Math.ceil((maxPressurePsi * 1.15) / 100) * 100;

    drawLaneGrid(top1, 'HYDRAULIC PRESSURE', 0, pressureMaxPsi, 'psi');
    drawSeries(
      top1,
      0,
      pressureMaxPsi,
      (p) => barToPsi(p.pressureRef),
      '#64748b',
      1.6,
      [7, 4],
    );
    drawSeries(
      top1,
      0,
      pressureMaxPsi,
      (p) => barToPsi(p.pressure),
      '#2563eb',
      2.1,
    );
    drawLegend(top1, [
      { label: 'controller ref', color: '#64748b', dash: [7, 4] },
      { label: 'actual', color: '#2563eb' },
    ]);

    const maxTorqueFtLbf = Math.max(
      4.0,
      ...history.map((p) => nmToFtLbf(p.torque)),
      ...history.map((p) => nmToFtLbf(p.torqueTrue)),
      ...history.map((p) => nmToFtLbf(p.targetTorque)),
      ...history.map((p) => nmToFtLbf(p.achievableTorque)),
    );
    const torqueMaxFtLbf = Math.ceil(maxTorqueFtLbf * 1.25 * 2) / 2;

    drawLaneGrid(top2, 'TORQUE ON BIT (ToB)', 0, torqueMaxFtLbf, 'ft·lbf', 1);
    drawSeries(
      top2,
      0,
      torqueMaxFtLbf,
      (p) => nmToFtLbf(p.targetTorque),
      '#64748b',
      1.6,
      [7, 4],
    );
    if (history.some((p) => p.limited)) {
      drawSeries(
        top2,
        0,
        torqueMaxFtLbf,
        (p) => nmToFtLbf(p.achievableTorque),
        '#7c3aed',
        1.2,
        [3, 3],
      );
    }
    drawSeries(
      top2,
      0,
      torqueMaxFtLbf,
      (p) => nmToFtLbf(p.torqueTrue),
      '#f59e0b',
      1.2,
      [2, 3],
    );
    drawSeries(
      top2,
      0,
      torqueMaxFtLbf,
      (p) => nmToFtLbf(p.torque),
      '#dc2626',
      2.2,
    );
    drawLegend(top2, [
      { label: 'SP', color: '#64748b', dash: [7, 4] },
      { label: 'Iq estimate', color: '#dc2626' },
      { label: 'truth', color: '#f59e0b', dash: [2, 3] },
      ...(history.some((p) => p.limited)
        ? [{ label: 'nominal max', color: '#7c3aed', dash: [3, 3] }]
        : []),
    ]);

    drawLaneGrid(top3, 'DRIVE SPEEDS', 0, 5000, 'RPM');
    drawSeries(top3, 0, 5000, (p) => p.spindleRpm, '#334155', 2.0);
    drawSeries(top3, 0, 5000, (p) => p.pumpRpm, '#d97706', 2.0);
    drawSeries(
      top3,
      0,
      5000,
      (p) => p.pumpFeedforwardRpm,
      '#94a3b8',
      1.1,
      [2, 3],
    );
    drawSeries(
      top3,
      0,
      5000,
      (p) => p.pumpCmdRpm,
      '#f59e0b',
      1.5,
      [6, 4],
    );
    drawLegend(top3, [
      { label: 'SPINDLE actual', color: '#334155' },
      { label: 'PUMP actual', color: '#d97706' },
      { label: 'PUMP FF', color: '#94a3b8', dash: [2, 3] },
      { label: 'PUMP command', color: '#f59e0b', dash: [6, 4] },
    ]);

    const maxRopSeen = Math.max(
      ...history.map((p) => mpsToMmPerMin(p.ropMps)),
      0.08,
    );
    const ropMax = Math.max(0.12, Math.ceil(maxRopSeen / 0.05) * 0.05);
    drawLaneGrid(
      top4,
      'PHYSICAL RATE OF PENETRATION',
      0,
      ropMax,
      'mm/min',
      2,
    );
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
            33 ms anti-aliased actuals • sharp SP/ref traces • pump feedforward + feedback command
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
        <span>Pressure: ref / actual [psi]</span>
        <span>ToB: SP / Iq estimate / truth [ft·lbf]</span>
        <span>RPM: spindle / pump actual / pump FF / command</span>
        <span>ROP [mm/min]</span>
      </div>
    </div>
  );
};
