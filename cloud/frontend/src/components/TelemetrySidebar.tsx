import React from 'react';
import {
  Activity,
  AlertTriangle,
  BatteryCharging,
  Cpu,
  Home,
  Lock,
  Moon,
  Radio,
  Shield,
  Signal,
  Unlock,
  Wifi,
} from 'lucide-react';
import { CommandAuditItem, TelemetryFrame } from '../types/telemetry';

interface TelemetrySidebarProps {
  latest: TelemetryFrame | null;
  wsConnected: boolean;
  sitlActive: boolean;
  auditLog: CommandAuditItem[];
  onCommand: (cmd: string, extra?: Record<string, unknown>) => void;
  onToggleSitl: () => void;
}

export const TelemetrySidebar: React.FC<TelemetrySidebarProps> = ({
  latest,
  wsConnected,
  sitlActive,
  auditLog,
  onCommand,
  onToggleSitl,
}) => {
  const vbat = latest?.vbatV ?? 0;
  const curr = latest?.currA ?? 0;
  const battPct = Math.max(0, Math.min(100, Math.round(((vbat - 10.2) / (12.6 - 10.2)) * 100)));
  const linkId = latest?.activeLink ?? 0;

  const isCriticalLed =
    latest?.flightMode === 10 || latest?.flags.propAnomaly || latest?.flags.geofenceHit;
  const isRthLed = latest?.flightMode === 6 || latest?.flags.failsafeRth;
  const isArmed = latest?.flags.armed ?? false;

  return (
    <div className="flex flex-col gap-3">
      {/* Power & Multi-Link Telemetry */}
      <div className="rounded-xl border border-slate-800 bg-slate-900/90 p-3 shadow-lg">
        <div className="mb-2 flex items-center justify-between">
          <span className="text-xs font-semibold tracking-wider text-cyan-400">
            MULTI-LINK ROUTER &amp; POWER
          </span>
          <button
            type="button"
            onClick={onToggleSitl}
            className={`rounded px-2 py-0.5 text-[11px] font-semibold transition ${
              sitlActive
                ? 'bg-emerald-500/20 text-emerald-300 ring-1 ring-emerald-500/40'
                : 'bg-slate-800 text-slate-400 hover:text-slate-200'
            }`}
          >
            {sitlActive ? 'SITL Sim: ON' : 'SITL Sim: OFF'}
          </button>
        </div>

        <div className="grid grid-cols-2 gap-2 text-xs">
          <div className="rounded-lg bg-slate-950/80 p-2">
            <div className="flex items-center justify-between text-slate-400">
              <span>Battery (3S)</span>
              <BatteryCharging className="h-3.5 w-3.5 text-emerald-400" />
            </div>
            <div className="mt-1 font-mono text-base font-bold text-slate-100">
              {vbat.toFixed(2)}V <span className="text-xs text-slate-400">/ {curr.toFixed(1)}A</span>
            </div>
            <div className="mt-1.5 h-1.5 w-full overflow-hidden rounded-full bg-slate-800">
              <div
                className={`h-full transition-all ${
                  battPct > 35 ? 'bg-emerald-400' : battPct > 18 ? 'bg-amber-400' : 'bg-rose-500'
                }`}
                style={{ width: `${battPct}%` }}
              />
            </div>
          </div>

          <div className="rounded-lg bg-slate-950/80 p-2">
            <div className="flex items-center justify-between text-slate-400">
              <span>Active Link</span>
              {linkId === 2 ? (
                <Wifi className="h-3.5 w-3.5 text-cyan-400" />
              ) : linkId === 1 ? (
                <Radio className="h-3.5 w-3.5 text-violet-400" />
              ) : (
                <Signal className="h-3.5 w-3.5 text-emerald-400" />
              )}
            </div>
            <div className="mt-1 truncate font-mono text-xs font-bold text-slate-100">
              {latest?.activeLinkName ?? (wsConnected ? 'Awaiting Frame' : 'Disconnected')}
            </div>
            <div className="mt-1 font-mono text-[11px] text-slate-400">
              GPS: {latest?.sats ?? 0} sats · CSQ {latest?.rssiScaled ?? 0}/15
            </div>
          </div>
        </div>

        {/* LilyGO T-SIM7000G GPIO 12 LED Indicator State */}
        <div className="mt-2 flex items-center justify-between rounded-lg border border-slate-800 bg-slate-950/80 px-2.5 py-1.5 text-xs">
          <div className="flex items-center gap-2">
            <span
              className={`inline-block h-2.5 w-2.5 rounded-full ${
                isCriticalLed
                  ? 'led-fast bg-rose-500 shadow-[0_0_8px_#f43f5e]'
                  : isRthLed
                  ? 'led-fast bg-amber-400 shadow-[0_0_8px_#f59e0b]'
                  : isArmed
                  ? 'bg-emerald-400 shadow-[0_0_8px_#34d399]'
                  : 'led-heartbeat bg-cyan-400 shadow-[0_0_6px_#22d3ee]'
              }`}
            />
            <span className="text-slate-400">GPIO 12 Board LED</span>
          </div>
          <span className="font-mono text-[11px] text-slate-200">
            {latest?.ledPattern ?? 'BOOT_INIT'}
          </span>
        </div>
      </div>

      {/* Sensor & Nav Subsystems */}
      <div className="rounded-xl border border-slate-800 bg-slate-900/90 p-3 shadow-lg">
        <div className="mb-2 flex items-center justify-between text-xs font-semibold tracking-wider text-cyan-400">
          <span>EKF-15 &amp; SENSOR HEALTH</span>
          <Cpu className="h-3.5 w-3.5 text-cyan-400" />
        </div>
        <div className="grid grid-cols-2 gap-1.5 font-mono text-[11px]">
          <StatusPill label="ARMED" ok={Boolean(latest?.flags.armed)} />
          <StatusPill label="GPS LOCK" ok={Boolean(latest?.flags.gpsLock)} />
          <StatusPill label="OPTFLOW PMW3901" ok={Boolean(latest?.flags.optflowOk)} />
          <StatusPill
            label={latest?.flags.vehicleMode ? 'ROLE: VEHICLE' : 'ROLE: DRONE'}
            ok={!latest?.flags.geofenceHit}
          />
          <StatusPill
            label="PROP/FFT HEALTH"
            ok={!latest?.flags.propAnomaly}
            warn={Boolean(latest?.flags.propAnomaly)}
          />
          <StatusPill
            label="WOM / TAMPER"
            ok={!latest?.flags.womAlert}
            warn={Boolean(latest?.flags.womAlert)}
          />
        </div>
      </div>

      {/* HMAC-SHA256 Signed Command Dispatch */}
      <div className="rounded-xl border border-slate-800 bg-slate-900/90 p-3 shadow-lg">
        <div className="mb-2 flex items-center justify-between text-xs font-semibold tracking-wider text-amber-400">
          <span>HMAC-SHA256 COMMAND CONSOLE</span>
          <Shield className="h-3.5 w-3.5 text-amber-400" />
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => onCommand('ARM')}
            className="flex items-center justify-center gap-1.5 rounded-lg bg-emerald-600/25 px-3 py-2 text-xs font-bold text-emerald-300 ring-1 ring-emerald-500/40 hover:bg-emerald-600/35"
          >
            <Unlock className="h-3.5 w-3.5" />
            ARM
          </button>
          <button
            type="button"
            onClick={() => onCommand('DISARM')}
            className="flex items-center justify-center gap-1.5 rounded-lg bg-rose-600/25 px-3 py-2 text-xs font-bold text-rose-300 ring-1 ring-rose-500/40 hover:bg-rose-600/35"
          >
            <Lock className="h-3.5 w-3.5" />
            DISARM
          </button>
          <button
            type="button"
            onClick={() => onCommand('RTH')}
            className="flex items-center justify-center gap-1.5 rounded-lg bg-amber-600/25 px-3 py-2 text-xs font-bold text-amber-300 ring-1 ring-amber-500/40 hover:bg-amber-600/35"
          >
            <Home className="h-3.5 w-3.5" />
            SMART RTH
          </button>
          <button
            type="button"
            onClick={() => onCommand('SET_MODE', { mode: latest?.flags.vehicleMode ? 0 : 1 })}
            className="flex items-center justify-center gap-1.5 rounded-lg bg-cyan-600/25 px-3 py-2 text-xs font-bold text-cyan-300 ring-1 ring-cyan-500/40 hover:bg-cyan-600/35"
          >
            <Activity className="h-3.5 w-3.5" />
            {latest?.flags.vehicleMode ? 'MODE: DRONE' : 'MODE: VEHICLE'}
          </button>
          <button
            type="button"
            onClick={() => onCommand('VEHICLE_SLEEP')}
            className="col-span-2 flex items-center justify-center gap-1.5 rounded-lg bg-indigo-600/25 px-3 py-1.5 text-xs font-bold text-indigo-300 ring-1 ring-indigo-500/40 hover:bg-indigo-600/35"
          >
            <Moon className="h-3.5 w-3.5" />
            DEEP SLEEP + IMU WOM GUARD
          </button>
        </div>

        {/* Audit Trail */}
        <div className="mt-3 max-h-32 overflow-y-auto rounded-lg bg-slate-950/90 p-2 font-mono text-[10px]">
          <div className="mb-1 text-slate-400">SIGNED COMMAND AUDIT LOG</div>
          {auditLog.length === 0 ? (
            <div className="text-slate-600">No commands dispatched yet.</div>
          ) : (
            auditLog.slice(0, 6).map((item, idx) => (
              <div
                key={`${item.nonce}-${idx}`}
                className="flex items-center justify-between border-t border-slate-900 py-1 text-slate-300"
              >
                <span className="font-bold text-amber-300">{item.cmd}</span>
                <span className="text-slate-500">
                  n={item.nonce} · {item.hmacPrefix}…
                </span>
              </div>
            ))
          )}
        </div>
      </div>

      {latest?.flags.propAnomaly && (
        <div className="flex items-center gap-2 rounded-lg border border-rose-500/50 bg-rose-950/60 px-3 py-2 text-xs text-rose-200">
          <AlertTriangle className="h-4 w-4 shrink-0 text-rose-400" />
          <span>FFT Gyro Resonance / Prop-Strike Anomaly Flagged</span>
        </div>
      )}
    </div>
  );
};

const StatusPill: React.FC<{ label: string; ok: boolean; warn?: boolean }> = ({
  label,
  ok,
  warn = false,
}) => (
  <div
    className={`flex items-center justify-between rounded px-2 py-1 ${
      warn
        ? 'bg-rose-500/20 text-rose-300 ring-1 ring-rose-500/40'
        : ok
        ? 'bg-emerald-500/15 text-emerald-300'
        : 'bg-slate-950 text-slate-500'
    }`}
  >
    <span className="truncate">{label}</span>
    <span className={`h-1.5 w-1.5 rounded-full ${warn ? 'bg-rose-400' : ok ? 'bg-emerald-400' : 'bg-slate-600'}`} />
  </div>
);

