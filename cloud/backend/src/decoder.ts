export const PAN_TELEM_MAGIC = 0xa5;
export const PAN_TELEM_PKT_SIZE = 36;

export const FLIGHT_MODE_NAMES: Record<number, string> = {
  0: 'DISARMED',
  1: 'MANUAL_ACRO',
  2: 'STABILIZE',
  3: 'ALT_HOLD',
  4: 'LOITER',
  5: 'MISSION',
  6: 'SMART_RTH',
  7: 'AUTOLAND',
  8: 'VEHICLE_ACTIVE',
  9: 'VEHICLE_SLEEP',
  10: 'EMERGENCY_KILL',
};

export const LINK_NAMES: Record<number, string> = {
  0: 'NONE',
  1: 'CRSF (Sub-GHz)',
  2: 'Wi-Fi UDP',
  3: 'LTE/NB-IoT MQTT',
  4: 'GSM SMS',
};

export interface DecodedFlags {
  armed: boolean;
  gpsLock: boolean;
  optflowOk: boolean;
  geofenceHit: boolean;
  propAnomaly: boolean;
  failsafeRth: boolean;
  vehicleMode: boolean;
  womAlert: boolean;
}

export interface DecodedTelemetry {
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
  flags: DecodedFlags;
  ledPattern: string;
  sourceProto: 'bin36' | 'json' | 'sitl';
}

export function crc16Ccitt(buf: Buffer, len = buf.length): number {
  let crc = 0xffff;
  for (let i = 0; i < len; i++) {
    crc ^= (buf[i] << 8) & 0xffff;
    for (let b = 0; b < 8; b++) {
      if ((crc & 0x8000) !== 0) {
        crc = ((crc << 1) ^ 0x1021) & 0xffff;
      } else {
        crc = (crc << 1) & 0xffff;
      }
    }
  }
  return crc;
}

export function decodeFlags(bitmask: number): DecodedFlags {
  return {
    armed: (bitmask & (1 << 0)) !== 0,
    gpsLock: (bitmask & (1 << 1)) !== 0,
    optflowOk: (bitmask & (1 << 2)) !== 0,
    geofenceHit: (bitmask & (1 << 3)) !== 0,
    propAnomaly: (bitmask & (1 << 4)) !== 0,
    failsafeRth: (bitmask & (1 << 5)) !== 0,
    vehicleMode: (bitmask & (1 << 6)) !== 0,
    womAlert: (bitmask & (1 << 7)) !== 0,
  };
}

export function inferLedPattern(flightMode: number, flags: DecodedFlags, activeLink: number): string {
  if (flightMode === 10 || flags.propAnomaly || flags.geofenceHit) {
    return 'CRITICAL_ERROR (10Hz Strobe)';
  }
  if (flightMode === 6 || flightMode === 7 || flags.failsafeRth) {
    return 'FAILSAFE_RTH (5Hz Fast)';
  }
  if (flightMode === 9) {
    return 'OFF_SLEEP';
  }
  if (flags.armed) {
    return 'ARMED_FLIGHT (Long Pulse)';
  }
  if (flags.gpsLock && activeLink !== 0) {
    return 'DISARMED_ONLINE (2s Heartbeat)';
  }
  return 'SEARCHING_NET_GPS (1Hz Blink)';
}

export function decodeBinaryTelemetry(
  buf: Buffer,
  deviceId = 'pan-brain-01',
  sourceProto: 'bin36' | 'sitl' = 'bin36'
): DecodedTelemetry | null {
  if (buf.length !== PAN_TELEM_PKT_SIZE) {
    return null;
  }
  if (buf.readUInt8(0) !== PAN_TELEM_MAGIC) {
    return null;
  }
  const rxCrc = buf.readUInt16LE(34);
  const calcCrc = crc16Ccitt(buf, 34);
  if (rxCrc !== calcCrc) {
    return null;
  }

  const msgType = buf.readUInt8(1);
  const seqId = buf.readUInt16LE(2);
  const utcEpoch = buf.readUInt32LE(4);
  const lat = buf.readInt32LE(8) / 1e7;
  const lon = buf.readInt32LE(12) / 1e7;
  const altM = buf.readInt16LE(16) / 100.0;
  const rollDeg = buf.readInt16LE(18) / 100.0;
  const pitchDeg = buf.readInt16LE(20) / 100.0;
  const yawDeg = buf.readUInt16LE(22) / 100.0;
  const speedMs = buf.readUInt16LE(24) / 100.0;
  const vbatV = buf.readUInt16LE(26) / 1000.0;
  const currA = buf.readUInt16LE(28) / 100.0;
  const flightMode = buf.readUInt8(30);
  const satsRssi = buf.readUInt8(31);
  const sats = (satsRssi >> 4) & 0x0f;
  const rssiScaled = satsRssi & 0x0f;
  const activeLink = buf.readUInt8(32);
  const healthFlags = buf.readUInt8(33);
  const flags = decodeFlags(healthFlags);

  return {
    deviceId,
    msgType,
    seqId,
    utcEpoch: utcEpoch > 0 ? utcEpoch : Math.floor(Date.now() / 1000),
    timestampIso: new Date((utcEpoch > 1700000000 ? utcEpoch : Math.floor(Date.now() / 1000)) * 1000).toISOString(),
    lat,
    lon,
    altM,
    rollDeg,
    pitchDeg,
    yawDeg,
    speedMs,
    vbatV,
    currA,
    flightMode,
    flightModeName: FLIGHT_MODE_NAMES[flightMode] ?? `MODE_${flightMode}`,
    sats,
    rssiScaled,
    activeLink,
    activeLinkName: LINK_NAMES[activeLink] ?? 'UNKNOWN',
    healthFlags,
    flags,
    ledPattern: inferLedPattern(flightMode, flags, activeLink),
    sourceProto,
  };
}

export function encodeBinaryTelemetry(pkt: DecodedTelemetry): Buffer {
  const buf = Buffer.alloc(PAN_TELEM_PKT_SIZE);
  buf.writeUInt8(PAN_TELEM_MAGIC, 0);
  buf.writeUInt8(pkt.msgType & 0xff, 1);
  buf.writeUInt16LE(pkt.seqId & 0xffff, 2);
  buf.writeUInt32LE(pkt.utcEpoch >>> 0, 4);
  buf.writeInt32LE(Math.round(pkt.lat * 1e7), 8);
  buf.writeInt32LE(Math.round(pkt.lon * 1e7), 12);
  buf.writeInt16LE(Math.max(-32768, Math.min(32767, Math.round(pkt.altM * 100))), 16);
  buf.writeInt16LE(Math.max(-18000, Math.min(18000, Math.round(pkt.rollDeg * 100))), 18);
  buf.writeInt16LE(Math.max(-9000, Math.min(9000, Math.round(pkt.pitchDeg * 100))), 20);
  buf.writeUInt16LE(Math.round((((pkt.yawDeg % 360) + 360) % 360) * 100) & 0xffff, 22);
  buf.writeUInt16LE(Math.max(0, Math.min(65535, Math.round(pkt.speedMs * 100))), 24);
  buf.writeUInt16LE(Math.max(0, Math.min(65535, Math.round(pkt.vbatV * 1000))), 26);
  buf.writeUInt16LE(Math.max(0, Math.min(65535, Math.round(pkt.currA * 100))), 28);
  buf.writeUInt8(pkt.flightMode & 0xff, 30);
  const satsRssi = ((Math.min(15, Math.max(0, pkt.sats)) & 0x0f) << 4) | (Math.min(15, Math.max(0, pkt.rssiScaled)) & 0x0f);
  buf.writeUInt8(satsRssi, 31);
  buf.writeUInt8(pkt.activeLink & 0xff, 32);
  buf.writeUInt8(pkt.healthFlags & 0xff, 33);
  const crc = crc16Ccitt(buf, 34);
  buf.writeUInt16LE(crc, 34);
  return buf;
}

export function decodeJsonTelemetry(rawText: string, fallbackDeviceId = 'pan-brain-01'): DecodedTelemetry | null {
  try {
    const obj = JSON.parse(rawText) as Record<string, unknown>;
    if (typeof obj.lat !== 'number' || typeof obj.lon !== 'number') {
      return null;
    }
    const flightMode = Number(obj.mode ?? 0);
    const activeLink = Number(obj.link ?? 3);
    const healthFlags = Number(obj.flags ?? 0);
    const flags = decodeFlags(healthFlags);
    const utcEpoch = Number(obj.ts ?? Math.floor(Date.now() / 1000));
    return {
      deviceId: typeof obj.id === 'string' ? obj.id : fallbackDeviceId,
      msgType: 1,
      seqId: Number(obj.seq ?? 0),
      utcEpoch,
      timestampIso: new Date((utcEpoch > 1700000000 ? utcEpoch : Math.floor(Date.now() / 1000)) * 1000).toISOString(),
      lat: Number(obj.lat),
      lon: Number(obj.lon),
      altM: Number(obj.alt ?? 0),
      rollDeg: Number(obj.roll ?? 0),
      pitchDeg: Number(obj.pitch ?? 0),
      yawDeg: Number(obj.yaw ?? 0),
      speedMs: Number(obj.spd ?? 0),
      vbatV: Number(obj.vbat ?? 0),
      currA: Number(obj.curr ?? 0),
      flightMode,
      flightModeName: FLIGHT_MODE_NAMES[flightMode] ?? `MODE_${flightMode}`,
      sats: Number(obj.sats ?? 0),
      rssiScaled: Number(obj.rssi ?? 0),
      activeLink,
      activeLinkName: LINK_NAMES[activeLink] ?? 'UNKNOWN',
      healthFlags,
      flags,
      ledPattern: inferLedPattern(flightMode, flags, activeLink),
      sourceProto: 'json',
    };
  } catch {
    return null;
  }
}

