import React from 'react';
import { Activity, Sliders, Database, Download, Cpu, HardDrive } from 'lucide-react';

export type TabType = 'monitor' | 'tuning' | 'sysid' | 'export';

interface SidebarProps {
  activeTab: TabType;
  onSelectTab: (tab: TabType) => void;
  controllerType: string;
}

export const Sidebar: React.FC<SidebarProps> = ({ activeTab, onSelectTab, controllerType }) => {
  const navItems = [
    { id: 'monitor' as TabType, label: 'Live Workstation', icon: Activity },
    { id: 'tuning' as TabType, label: 'Control & Overrides', icon: Sliders },
    { id: 'sysid' as TabType, label: 'SysID Calibration', icon: Database },
    { id: 'export' as TabType, label: 'Scientific Export', icon: Download },
  ];

  return (
    <aside className="w-64 bg-card border-r border-border flex flex-col justify-between select-none shrink-0 h-screen sticky top-0">
      <div>
        {/* Brand Header */}
        <div className="h-16 border-b border-border flex items-center px-6 gap-3">
          <div className="w-8 h-8 rounded bg-primary flex items-center justify-center text-white shadow-sm font-mono font-bold text-sm">
            MW
          </div>
          <div>
            <div className="font-semibold text-slate-900 tracking-tight text-sm">METALWORK</div>
            <div className="text-[10px] uppercase font-mono tracking-wider text-slate-400">CO-SIMULATOR v0.1</div>
          </div>
        </div>

        {/* Navigation List */}
        <nav className="p-3 space-y-1">
          {navItems.map((item) => {
            const Icon = item.icon;
            const isActive = activeTab === item.id;
            return (
              <button
                key={item.id}
                onClick={() => onSelectTab(item.id)}
                className={`w-full flex items-center gap-3 px-3.5 py-2.5 rounded text-xs font-medium transition-colors ${
                  isActive
                    ? 'bg-blue-50/80 text-primary font-semibold border-l-2 border-primary'
                    : 'text-slate-600 hover:text-slate-900 hover:bg-slate-50'
                }`}
              >
                <Icon className={`w-4 h-4 ${isActive ? 'text-primary' : 'text-slate-400'}`} />
                {item.label}
              </button>
            );
          })}
        </nav>
      </div>

      {/* Footer Hardware & Model Info */}
      <div className="p-4 border-t border-border bg-slate-50/50 space-y-3">
        <div className="flex items-center gap-2 text-[11px] text-slate-500 font-mono">
          <Cpu className="w-3.5 h-3.5 text-slate-400" />
          <span>Active: {controllerType.toUpperCase()}</span>
        </div>
        <div className="flex items-center gap-2 text-[11px] text-slate-500 font-mono">
          <HardDrive className="w-3.5 h-3.5 text-slate-400" />
          <span>Target: INCONEL 718</span>
        </div>
        <div className="text-[10px] text-slate-400 border-t border-slate-200/60 pt-2 font-mono">
          Virtual Clock 1000 Hz RK4
        </div>
      </div>
    </aside>
  );
};
