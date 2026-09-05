import React, { useRef, useEffect } from 'react';
import { Download } from 'lucide-react';

interface ChartPoint {
  t: number;
  pressure: number;
  torque: number;
  wob: number;
  spindleRpm: number;
  pumpRpm: number;
  depth: number;
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
    const height = 480;

    canvas.width = width * dpr;
    canvas.height = height * dpr;
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;

    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, width, height);

    // Split canvas into 3 horizontal chart lanes:
    // Lane 1: Hydraulic Pressure & WOB
    // Lane 2: Spindle Torque (Iq derived)
    // Lane 3: Speeds (Spindle & Pump RPM)
    const laneHeight = 140;
    const padLeft = 55;
    const padRight = 45;
    const plotWidth = width - padLeft - padRight;

    const tMin = history[0].t;
    const tMax = Math.max(tMin + 0.5, history[history.length - 1].t);

    const getX = (t: number) => padLeft + ((t - tMin) / (tMax - tMin)) * plotWidth;

    // Helper to draw a lane grid
    const drawLaneGrid = (top: number, yLabel: string, yMin: number, yMax: number, unit: string) => {
      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1;
      ctx.strokeRect(padLeft, top, plotWidth, laneHeight);

      // Horizontal mid-grid lines
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(padLeft, top + laneHeight / 2);
      ctx.lineTo(padLeft + plotWidth, top + laneHeight / 2);
      ctx.stroke();
      ctx.setLineDash([]);

      // Labels
      ctx.fillStyle = '#64748b';
      ctx.font = '10px "JetBrains Mono", monospace';
      ctx.textAlign = 'right';
      ctx.fillText(`${yMax.toFixed(0)}`, padLeft - 6, top + 10);
      ctx.fillText(`${((yMin + yMax) / 2).toFixed(0)}`, padLeft - 6, top + laneHeight / 2 + 3);
      ctx.fillText(`${yMin.toFixed(0)}`, padLeft - 6, top + laneHeight - 2);

      // Lane Title Badge
      ctx.textAlign = 'left';
      ctx.fillStyle = '#0f172a';
      ctx.font = '600 11px Inter, sans-serif';
      ctx.fillText(`${yLabel} [${unit}]`, padLeft + 8, top + 16);
    };

    // --- LANE 1: Pressure (0 - 70 bar) ---
    drawLaneGrid(10, 'HYDRAULIC PRESSURE P', 0, 70, 'bar');
    ctx.strokeStyle = '#2563eb';
    ctx.lineWidth = 2;
    ctx.beginPath();
    history.forEach((pt, idx) => {
      const x = getX(pt.t);
      const y = 10 + laneHeight - (Math.min(70, Math.max(0, pt.pressure)) / 70) * laneHeight;
      if (idx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // --- LANE 2: Spindle Torque (0 - 10 N*m) ---
    drawLaneGrid(165, 'SPINDLE TORQUE (Iq DERIVED)', 0, 10, 'N·m');
    // Overload ceiling line (7.5 N*m)
    ctx.strokeStyle = '#fca5a5';
    ctx.setLineDash([4, 4]);
    const yOverload = 165 + laneHeight - (7.5 / 10.0) * laneHeight;
    ctx.beginPath();
    ctx.moveTo(padLeft, yOverload);
    ctx.lineTo(padLeft + plotWidth, yOverload);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 2;
    ctx.beginPath();
    history.forEach((pt, idx) => {
      const x = getX(pt.t);
      const y = 165 + laneHeight - (Math.min(10, Math.max(0, pt.torque)) / 10) * laneHeight;
      if (idx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // --- LANE 3: Speeds (0 - 5000 RPM) ---
    drawLaneGrid(320, 'DRIVE SPEEDS (SPINDLE & PUMP)', 0, 5000, 'RPM');
    // Spindle RPM (slate-700)
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    history.forEach((pt, idx) => {
      const x = getX(pt.t);
      const y = 320 + laneHeight - (Math.min(5000, Math.max(0, pt.spindleRpm)) / 5000) * laneHeight;
      if (idx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Pump RPM (amber-500)
    ctx.strokeStyle = '#d97706';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    history.forEach((pt, idx) => {
      const x = getX(pt.t);
      const y = 320 + laneHeight - (Math.min(5000, Math.max(0, pt.pumpRpm)) / 5000) * laneHeight;
      if (idx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
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
            High-Resolution Dynamic Vector Strips
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

      <div className="flex items-center justify-center gap-6 mt-3 text-[11px] font-mono text-slate-500 border-t border-border pt-2">
        <div className="flex items-center gap-1.5">
          <span className="w-3 h-0.5 bg-blue-600 rounded" />
          <span>Pressure [bar]</span>
        </div>
        <div className="flex items-center gap-1.5">
          <span className="w-3 h-0.5 bg-red-600 rounded" />
          <span>Torque [N·m]</span>
        </div>
        <div className="flex items-center gap-1.5">
          <span className="w-3 h-0.5 bg-slate-700 rounded" />
          <span>Spindle [RPM]</span>
        </div>
        <div className="flex items-center gap-1.5">
          <span className="w-3 h-0.5 bg-amber-600 rounded" />
          <span>Pump [RPM]</span>
        </div>
      </div>
    </div>
  );
};
