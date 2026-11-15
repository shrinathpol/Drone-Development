import React from 'react';

interface ArtificialHorizonProps {
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  altM: number;
  speedMs: number;
}

export const ArtificialHorizon: React.FC<ArtificialHorizonProps> = ({
  rollDeg,
  pitchDeg,
  yawDeg,
  altM,
  speedMs,
}) => {
  const pitchOffset = Math.max(-45, Math.min(45, pitchDeg)) * 2.2;
  const headingInt = Math.round(((yawDeg % 360) + 360) % 360);

  return (
    <div className="relative rounded-xl border border-slate-800 bg-slate-900/90 p-3 shadow-lg">
      <div className="mb-2 flex items-center justify-between text-xs text-slate-400">
        <span className="font-semibold tracking-wider text-cyan-400">ATTITUDE HUD</span>
        <span className="font-mono text-slate-300">HDG {String(headingInt).padStart(3, '0')}°</span>
      </div>

      <div className="relative mx-auto h-48 w-full overflow-hidden rounded-lg border border-slate-700 bg-slate-950">
        <svg viewBox="-120 -90 240 180" className="h-full w-full">
          <defs>
            <clipPath id="hud-clip">
              <rect x="-120" y="-90" width="240" height="180" rx="6" />
            </clipPath>
          </defs>

          <g clipPath="url(#hud-clip)">
            <g transform={`rotate(${-rollDeg}) translate(0, ${pitchOffset})`}>
              {/* Sky */}
              <rect x="-240" y="-260" width="480" height="260" fill="#0c4a6e" />
              {/* Ground */}
              <rect x="-240" y="0" width="480" height="260" fill="#3f2718" />
              {/* Horizon line */}
              <line x1="-240" y1="0" x2="240" y2="0" stroke="#38bdf8" strokeWidth="1.8" />
              {/* Pitch ladder */}
              {[-30, -20, -10, 10, 20, 30].map((deg) => {
                const y = -deg * 2.2;
                const w = deg % 20 === 0 ? 34 : 20;
                return (
                  <g key={deg}>
                    <line
                      x1={-w}
                      y1={y}
                      x2={w}
                      y2={y}
                      stroke="#e2e8f0"
                      strokeWidth="1.1"
                      strokeDasharray={deg < 0 ? '4 3' : undefined}
                    />
                    <text
                      x={w + 6}
                      y={y + 3}
                      fill="#e2e8f0"
                      fontSize="8"
                      fontFamily="monospace"
                    >
                      {deg}
                    </text>
                  </g>
                );
              })}
            </g>
          </g>

          {/* Fixed Aircraft Bore Reference */}
          <path
            d="M -36 0 L -12 0 L -5 6 L 0 0 L 5 6 L 12 0 L 36 0"
            fill="none"
            stroke="#facc15"
            strokeWidth="2.4"
            strokeLinecap="round"
          />
          <circle cx="0" cy="0" r="2.2" fill="#facc15" />

          {/* Roll Bank Scale Arc */}
          <path
            d="M -52 -52 A 74 74 0 0 1 52 -52"
            fill="none"
            stroke="#94a3b8"
            strokeWidth="1.2"
          />
          <polygon points="0,-68 -4,-61 4,-61" fill="#facc15" />
        </svg>

        {/* Corner Speed / Alt Tapes */}
        <div className="pointer-events-none absolute left-2 top-2 rounded bg-slate-950/80 px-2 py-1 font-mono text-xs text-emerald-400">
          SPD {speedMs.toFixed(1)} m/s
        </div>
        <div className="pointer-events-none absolute right-2 top-2 rounded bg-slate-950/80 px-2 py-1 font-mono text-xs text-amber-300">
          ALT {altM.toFixed(1)} m
        </div>
        <div className="pointer-events-none absolute bottom-1.5 left-2 rounded bg-slate-950/75 px-1.5 py-0.5 font-mono text-[11px] text-slate-300">
          R {rollDeg.toFixed(1)}°
        </div>
        <div className="pointer-events-none absolute bottom-1.5 right-2 rounded bg-slate-950/75 px-1.5 py-0.5 font-mono text-[11px] text-slate-300">
          P {pitchDeg.toFixed(1)}°
        </div>
      </div>
    </div>
  );
};

