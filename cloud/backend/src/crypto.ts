import crypto from 'crypto';

export type PanCloudCommandType =
  | 'ARM'
  | 'DISARM'
  | 'RTH'
  | 'GOTO_WAYPOINT'
  | 'SET_MODE'
  | 'VEHICLE_SLEEP'
  | 'OTA_MANIFEST';

export interface CommandRequestInput {
  cmd: PanCloudCommandType;
  lat?: number;
  lon?: number;
  alt?: number;
  mode?: number;
  sha256?: string;
  size?: number;
}

export interface SignedCommandEnvelope {
  cmd: PanCloudCommandType;
  ts: number;
  nonce: number;
  lat: number;
  lon: number;
  alt: number;
  mode: number;
  sha256?: string;
  size?: number;
  hmac: string;
}

let nonceCounter = Math.floor(Date.now() % 1000000) + 1;

/**
 * Matches firmware core/src/crypto_auth.c:
 * Canonical signing string: "<cmd>|<ts>|<nonce>|<lat_e7>|<lon_e7>|<alt_cm>"
 */
export function buildAndSignCommand(
  input: CommandRequestInput,
  secretKey: string
): SignedCommandEnvelope {
  const ts = Math.floor(Date.now() / 1000);
  nonceCounter = (nonceCounter + 1) >>> 0;
  if (nonceCounter === 0) nonceCounter = 1;
  const nonce = nonceCounter;

  const lat = Number.isFinite(input.lat) ? Number(input.lat) : 0;
  const lon = Number.isFinite(input.lon) ? Number(input.lon) : 0;
  const alt = Number.isFinite(input.alt) ? Number(input.alt) : 0;
  const mode = Number.isFinite(input.mode) ? Number(input.mode) : 0;

  const latE7 = Math.round(lat * 1e7);
  const lonE7 = Math.round(lon * 1e7);
  const altCm = Math.round(alt * 100);

  const canonical = `${input.cmd}|${ts}|${nonce}|${latE7}|${lonE7}|${altCm}`;
  const hmac = crypto
    .createHmac('sha256', secretKey)
    .update(canonical, 'utf8')
    .digest('hex');

  const envelope: SignedCommandEnvelope = {
    cmd: input.cmd,
    ts,
    nonce,
    lat: Number(lat.toFixed(7)),
    lon: Number(lon.toFixed(7)),
    alt: Number(alt.toFixed(2)),
    mode,
    hmac,
  };

  if (input.cmd === 'OTA_MANIFEST') {
    if (input.sha256) envelope.sha256 = input.sha256;
    if (input.size !== undefined) envelope.size = input.size;
  }

  return envelope;
}

