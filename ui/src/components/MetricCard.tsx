import React from 'react';

interface MetricCardProps {
  label: string;
  value: string | number;
  unit: string;
  subValue?: string;
  highlight?: boolean;
}

export const MetricCard: React.FC<MetricCardProps> = ({
  label,
  value,
  unit,
  subValue,
  highlight = false,
}) => {
  return (
    <div className={`bg-card border border-border rounded p-4 shadow-sm transition-all hover:border-slate-300 ${
      highlight ? 'ring-1 ring-primary/20' : ''
    }`}>
      <div className="flex items-center justify-between text-[11px] font-mono uppercase tracking-wider text-slate-500 mb-1.5">
        <span>{label}</span>
        <span className="text-slate-400 font-normal">{unit}</span>
      </div>
      <div className="flex items-baseline gap-1.5">
        <span className="text-2xl font-bold font-mono tracking-tight text-slate-900 tabular-nums">
          {value}
        </span>
      </div>
      {subValue && (
        <div className="mt-1 text-[11px] font-mono text-slate-400">
          {subValue}
        </div>
      )}
    </div>
  );
};
