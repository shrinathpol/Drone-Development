import React, { useEffect, useState } from 'react';
import { Compass, RadioTower } from 'lucide-react';
import { ArtificialHorizon } from './components/ArtificialHorizon';
import { TacticalMap } from './components/TacticalMap';
import { TelemetrySidebar } from './components/TelemetrySidebar';
import { CommandAuditItem, GeofencePoly, TelemetryFrame } from './types/telemetry';

export const App: React.FC = () => {
  const [latest, setLatest] = useState<TelemetryFrame | null>(null);
  const [history, setHistory] = useState<TelemetryFrame[]>([]);
  const [geofences, setGeofences] = useState<GeofencePoly[]>([]);
  const [auditLog, setAuditLog] = useState<CommandAuditItem[]>([]);
  const [wsConnected, setWsConnected] = useState(false);
  const [sitlActive, setSitlActive] = useState(false);

  useEffect(() => {
    fetch('/api/health')
      .then((r) => r.json())
      .then((d) => setSitlActive(Boolean(d.sitlSimActive)))
      .catch(() => {});

    fetch('/api/geofences')
      .then((r) => r.json())
      .then((d) => {
        if (Array.isArray(d.geofences)) setGeofences(d.geofences);
      })
      .catch(() => {});

    fetch('/api/cmd/audit')
      .then((r) => r.json())
      .then((d) => {
        if (Array.isArray(d.items)) setAuditLog(d.items);
      })
      .catch(() => {});

    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${proto}//${window.location.host}/ws/telemetry`;
    let ws: WebSocket | null = null;
    let reconnectTimer: number | null = null;

    const connect = () => {
      ws = new WebSocket(wsUrl);
      ws.onopen = () => setWsConnected(true);
      ws.onclose = () => {
        setWsConnected(false);
        reconnectTimer = window.setTimeout(connect, 2000);
      };
      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data);
          if (msg.type === 'telemetry' && msg.data) {
            const frame = msg.data as TelemetryFrame;
            setLatest(frame);
            setHistory((prev) => [...prev.slice(-299), frame]);
          } else if (msg.type === 'snapshot') {
            if (Array.isArray(msg.history)) {
              setHistory(msg.history);
              if (msg.history.length > 0) {
                setLatest(msg.history[msg.history.length - 1]);
              }
            }
            if (Array.isArray(msg.geofences)) setGeofences(msg.geofences);
            setSitlActive(Boolean(msg.sitlSimActive));
          } else if (msg.type === 'cmd_audit' && msg.data) {
            setAuditLog((prev) => [msg.data as CommandAuditItem, ...prev.slice(0, 49)]);
          }
        } catch {
          // ignore malformed messages
        }
      };
    };

    connect();
    return () => {
      if (reconnectTimer) clearTimeout(reconnectTimer);
      ws?.close();
    };
  }, []);

  const handleCommand = async (cmd: string, extra: Record<string, unknown> = {}) => {
    try {
      await fetch('/api/cmd', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ cmd, ...extra }),
      });
    } catch (err) {
      console.error('Failed to dispatch signed command:', err);
    }
  };

  const handleToggleSitl = async () => {
    try {
      const res = await fetch('/api/sim/toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enable: !sitlActive }),
      });
      const data = await res.json();
      setSitlActive(Boolean(data.sitlSimActive));
    } catch {
      // ignore
    }
  };

  return (
    <div className="flex min-h-screen flex-col bg-slate-950 text-slate-100">
      {/* Top Status Bar */}
      <header className="flex flex-wrap items-center justify-between gap-3 border-b border-slate-800 bg-slate-900/95 px-5 py-3">
        <div className="flex items-center gap-3">
          <div className="flex h-9 w-9 items-center justify-center rounded-lg bg-cyan-500/20 text-cyan-400 ring-1 ring-cyan-500/40">
            <Compass className="h-5 w-5" />
          </div>
          <div>
            <h1 className="text-sm font-bold tracking-wider text-slate-100">
              PAN-BRAIN AUTOPILOT &amp; BVLOS TELEMETRY GCS
            </h1>
            <p className="font-mono text-[11px] text-slate-400">
              MCU-Agnostic C11 Core · LilyGO T-SIM7000G BSP · 36B Binary + HMAC-SHA256
            </p>
          </div>
        </div>

        <div className="flex flex-wrap items-center gap-3 font-mono text-xs">
          <span className="rounded-lg bg-slate-800 px-3 py-1 text-slate-200">
            MODE:{' '}
            <strong className="text-cyan-300">{latest?.flightModeName ?? 'OFFLINE'}</strong>
          </span>
          <span className="rounded-lg bg-slate-800 px-3 py-1 text-slate-300">
            SEQ #{latest?.seqId ?? 0} ({latest?.sourceProto ?? '--'})
          </span>
          <span
            className={`flex items-center gap-1.5 rounded-lg px-3 py-1 ${
              wsConnected
                ? 'bg-emerald-500/15 text-emerald-300 ring-1 ring-emerald-500/30'
                : 'bg-rose-500/15 text-rose-300 ring-1 ring-rose-500/30'
            }`}
          >
            <RadioTower className="h-3.5 w-3.5" />
            {wsConnected ? 'WS STREAM LIVE' : 'WS RECONNECTING'}
          </span>
        </div>
      </header>

      {/* Main Split Workspace */}
      <main className="grid flex-1 grid-cols-1 gap-4 p-4 lg:grid-cols-12">
        <section className="lg:col-span-8 xl:col-span-9">
          <TacticalMap
            latest={latest}
            history={history}
            geofences={geofences}
            onSendWaypoint={(lat, lon, altM) =>
              void handleCommand('GOTO_WAYPOINT', { lat, lon, alt: altM })
            }
          />
        </section>

        <aside className="flex flex-col gap-3 lg:col-span-4 xl:col-span-3">
          <ArtificialHorizon
            rollDeg={latest?.rollDeg ?? 0}
            pitchDeg={latest?.pitchDeg ?? 0}
            yawDeg={latest?.yawDeg ?? 0}
            altM={latest?.altM ?? 0}
            speedMs={latest?.speedMs ?? 0}
          />
          <TelemetrySidebar
            latest={latest}
            wsConnected={wsConnected}
            sitlActive={sitlActive}
            auditLog={auditLog}
            onCommand={(cmd, extra) => void handleCommand(cmd, extra)}
            onToggleSitl={() => void handleToggleSitl()}
          />
        </aside>
      </main>
    </div>
  );
};

export default App;

