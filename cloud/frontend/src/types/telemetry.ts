export interface TelemetryFlags {
  armed: boolean;
  gpsLock: boolean;
  optflowOk: boolean;
  geofenceHit: boolean;
  propAnomaly: boolean;
  failsafeRth: boolean;
  vehicleMode: boolean;
  womAlert: boolean;
}

export interface TelemetryFrame {
  deviceId: string;
  msgType: number;
  seqId: number;
  utcEpoch: number;
  timestampIso: string;
  lat: number;
  lon: number;
  altM: number;
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  speedMs: number;
  vbatV: number;
  currA: number;
  flightMode: number;
  flightModeName: string;
  sats: number;
  rssiScaled: number;
  activeLink: number;
  activeLinkName: string;
  healthFlags: number;
  flags: TelemetryFlags;
  ledPattern: string;
  sourceProto: 'bin36' | 'json' | 'sitl';
}

export interface GeofencePoly {
  id: number;
  deviceId: string;
  name: string;
  fenceType: 'KEEP_IN' | 'KEEP_OUT';
  minAltM: number;
  maxAltM: number;
  polygon: Array<[number, number]>;
}

export interface CommandAuditItem {
  ts: string;
  deviceId: string;
  cmd: string;
  nonce: number;
  hmacPrefix: string;
  transports: string[];
}

