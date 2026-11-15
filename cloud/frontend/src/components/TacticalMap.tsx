import React, { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import { Crosshair, Navigation, ShieldAlert } from 'lucide-react';
import { GeofencePoly, TelemetryFrame } from '../types/telemetry';

interface TacticalMapProps {
  latest: TelemetryFrame | null;
  history: TelemetryFrame[];
  geofences: GeofencePoly[];
  onSendWaypoint: (lat: number, lon: number, altM: number) => void;
}

export const TacticalMap: React.FC<TacticalMapProps> = ({
  latest,
  history,
  geofences,
  onSendWaypoint,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const droneMarkerRef = useRef<L.Marker | null>(null);
  const trailRef = useRef<L.Polyline | null>(null);
  const wpMarkerRef = useRef<L.Marker | null>(null);
  const fenceLayerRef = useRef<L.LayerGroup | null>(null);

  const [followDrone, setFollowDrone] = useState(true);
  const [selectedWp, setSelectedWp] = useState<{ lat: number; lon: number } | null>(null);
  const [wpAltM, setWpAltM] = useState<number>(25);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const map = L.map(containerRef.current, {
      center: [15.3647, 75.124],
      zoom: 17,
      zoomControl: true,
    });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; OpenStreetMap',
      maxZoom: 20,
    }).addTo(map);

    const trail = L.polyline([], {
      color: '#22d3ee',
      weight: 3,
      opacity: 0.85,
    }).addTo(map);

    const fenceGroup = L.layerGroup().addTo(map);

    map.on('click', (ev: L.LeafletMouseEvent) => {
      const { lat, lng } = ev.latlng;
      setSelectedWp({ lat, lon: lng });
      if (wpMarkerRef.current) {
        wpMarkerRef.current.setLatLng([lat, lng]);
      } else {
        wpMarkerRef.current = L.marker([lat, lng], {
          icon: L.divIcon({
            className: '',
            html: '<div style="width:16px;height:16px;border-radius:9999px;background:#f59e0b;border:2px solid #fff;box-shadow:0 0 12px #f59e0b;"></div>',
            iconSize: [16, 16],
            iconAnchor: [8, 8],
          }),
        }).addTo(map);
      }
    });

    mapRef.current = map;
    trailRef.current = trail;
    fenceLayerRef.current = fenceGroup;

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  useEffect(() => {
    const map = mapRef.current;
    const fenceLayer = fenceLayerRef.current;
    if (!map || !fenceLayer) return;

    fenceLayer.clearLayers();
    geofences.forEach((f) => {
      if (f.polygon.length >= 3) {
        L.polygon(f.polygon, {
          color: f.fenceType === 'KEEP_OUT' ? '#ef4444' : '#10b981',
          weight: 2,
          dashArray: f.fenceType === 'KEEP_OUT' ? '6 4' : undefined,
          fillOpacity: 0.08,
        })
          .bindTooltip(`${f.name} (${f.fenceType})`)
          .addTo(fenceLayer);
      }
    });
  }, [geofences]);

  useEffect(() => {
    const map = mapRef.current;
    const trail = trailRef.current;
    if (!map || !trail) return;

    const coords = history
      .filter((p) => Math.abs(p.lat) > 0.01 && Math.abs(p.lon) > 0.01)
      .map((p): [number, number] => [p.lat, p.lon]);
    trail.setLatLngs(coords);

    if (latest && Math.abs(latest.lat) > 0.01 && Math.abs(latest.lon) > 0.01) {
      const iconHtml = `<div style="transform: rotate(${latest.yawDeg.toFixed(
        1
      )}deg); width: 28px; height: 28px; display:flex; align-items:center; justify-content:center;">
        <svg width="26" height="26" viewBox="0 0 24 24" fill="${
          latest.flags.armed ? '#22d3ee' : '#94a3b8'
        }" stroke="#020617" stroke-width="1.5">
          <polygon points="12,2 21,21 12,16 3,21"/>
        </svg>
      </div>`;

      const icon = L.divIcon({
        className: '',
        html: iconHtml,
        iconSize: [28, 28],
        iconAnchor: [14, 14],
      });

      if (droneMarkerRef.current) {
        droneMarkerRef.current.setLatLng([latest.lat, latest.lon]);
        droneMarkerRef.current.setIcon(icon);
      } else {
        droneMarkerRef.current = L.marker([latest.lat, latest.lon], { icon }).addTo(map);
      }

      if (followDrone) {
        map.panTo([latest.lat, latest.lon], { animate: true, duration: 0.2 });
      }
    }
  }, [latest, history, followDrone]);

  return (
    <div className="relative flex h-full min-h-[480px] flex-col overflow-hidden rounded-xl border border-slate-800 bg-slate-900">
      <div className="flex flex-wrap items-center justify-between gap-2 border-b border-slate-800 bg-slate-900/95 px-4 py-2.5">
        <div className="flex items-center gap-2 text-xs font-semibold tracking-wider text-slate-200">
          <Navigation className="h-4 w-4 text-cyan-400" />
          <span>TACTICAL MAP &amp; 3D GEOFENCE CONSOLE</span>
          <span className="rounded bg-slate-800 px-2 py-0.5 font-mono text-[11px] text-slate-400">
            Click map to drop GOTO waypoint
          </span>
        </div>

        <div className="flex items-center gap-2">
          <button
            type="button"
            onClick={() => setFollowDrone((v) => !v)}
            className={`flex items-center gap-1.5 rounded-lg px-2.5 py-1 text-xs font-medium transition ${
              followDrone
                ? 'bg-cyan-500/20 text-cyan-300 ring-1 ring-cyan-500/50'
                : 'bg-slate-800 text-slate-400 hover:text-slate-200'
            }`}
          >
            <Crosshair className="h-3.5 w-3.5" />
            {followDrone ? 'Tracking Drone' : 'Free Pan'}
          </button>

          {selectedWp && (
            <div className="flex items-center gap-2 rounded-lg border border-amber-500/40 bg-amber-500/10 px-2.5 py-1">
              <span className="font-mono text-[11px] text-amber-300">
                WP {selectedWp.lat.toFixed(5)}, {selectedWp.lon.toFixed(5)}
              </span>
              <input
                type="number"
                value={wpAltM}
                min={2}
                max={120}
                onChange={(e) => setWpAltM(Number(e.target.value))}
                className="w-14 rounded bg-slate-950 px-1.5 py-0.5 font-mono text-xs text-slate-100 ring-1 ring-slate-700"
                title="Target Altitude (m)"
              />
              <button
                type="button"
                onClick={() => onSendWaypoint(selectedWp.lat, selectedWp.lon, wpAltM)}
                className="rounded bg-amber-500 px-2.5 py-0.5 text-xs font-bold text-slate-950 hover:bg-amber-400"
              >
                FLY TO WP (HMAC)
              </button>
            </div>
          )}
        </div>
      </div>

      <div ref={containerRef} className="dark-tiles relative flex-1" />

      {latest?.flags.geofenceHit && (
        <div className="absolute bottom-4 left-4 z-[1000] flex items-center gap-2 rounded-lg border border-rose-500 bg-rose-950/90 px-3 py-2 text-xs font-bold text-rose-200 shadow-xl">
          <ShieldAlert className="h-4 w-4 text-rose-400" />
          GEOFENCE BOUNDARY ENGAGED - VIRTUAL BUMPER / RTH ACTIVE
        </div>
      )}
    </div>
  );
};

