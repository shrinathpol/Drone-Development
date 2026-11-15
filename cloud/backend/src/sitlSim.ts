import { DecodedTelemetry, decodeBinaryTelemetry, encodeBinaryTelemetry } from './decoder';
import { SignedCommandEnvelope } from './crypto';

interface SimWaypoint {
  lat: number;
  lon: number;
  altM: number;
}

export class BuiltinSitlSimulator {
  private running = false;
  private timer: NodeJS.Timeout | null = null;
  private seq = 0;
  private readonly homeLat = 15.3647;
  private readonly homeLon = 75.1240;

  private lat = this.homeLat;
  private lon = this.homeLon;
  private altM = 0.0;
  private rollDeg = 0;
  private pitchDeg = 0;
  private yawDeg = 35;
  private speedMs = 0;
  private vbatV = 12.58;
  private currA = 0.9;
  private flightMode = 2; // STABILIZE initially armed or disarmed
  private armed = false;
  private vehicleMode = false;
  private sats = 13;
  private rssiScaled = 12;
  private activeLink = 3; // LTE MQTT
  private targetWp: SimWaypoint | null = null;
  private tSec = 0;

  constructor(private readonly deviceId: string) {}

  isRunning(): boolean {
    return this.running;
  }

  start(onFrame: (pkt: DecodedTelemetry) => void): void {
    if (this.running) return;
    this.running = true;
    this.armed = true;
    this.flightMode = 5; // MISSION

    this.timer = setInterval(() => {
      this.step();
      const raw = encodeBinaryTelemetry(this.snapshot());
      const decoded = decodeBinaryTelemetry(raw, this.deviceId, 'sitl');
      if (decoded) {
        onFrame(decoded);
      }
    }, 200); // 5Hz telemetry cadence in UI simulator
  }

  stop(): void {
    this.running = false;
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  applyCommand(env: SignedCommandEnvelope): void {
    switch (env.cmd) {
      case 'ARM':
        this.armed = true;
        this.flightMode = this.vehicleMode ? 8 : 4;
        break;
      case 'DISARM':
        this.armed = false;
        this.flightMode = 0;
        this.targetWp = null;
        break;
      case 'RTH':
        this.armed = true;
        this.flightMode = 6; // SMART_RTH
        this.targetWp = { lat: this.homeLat, lon: this.homeLon, altM: 22.0 };
        break;
      case 'GOTO_WAYPOINT':
        this.armed = true;
        this.flightMode = 5; // MISSION
        this.targetWp = {
          lat: env.lat || this.homeLat + 0.001,
          lon: env.lon || this.homeLon + 0.001,
          altM: env.alt > 1 ? env.alt : 25.0,
        };
        break;
      case 'SET_MODE':
        this.vehicleMode = env.mode === 1;
        this.flightMode = this.vehicleMode ? 8 : (this.armed ? 4 : 0);
        break;
      case 'VEHICLE_SLEEP':
        this.vehicleMode = true;
        this.armed = false;
        this.flightMode = 9;
        break;
      default:
        break;
    }
  }

  private step(): void {
    const dt = 0.2;
    this.tSec += dt;
    this.seq = (this.seq + 1) & 0xffff;

    if (!this.armed || this.flightMode === 0 || this.flightMode === 9) {
      this.altM = Math.max(0, this.altM - 2.0 * dt);
      this.speedMs = 0;
      this.currA = 0.45;
      this.rollDeg *= 0.8;
      this.pitchDeg *= 0.8;
      return;
    }

    if (this.targetWp) {
      const dLat = (this.targetWp.lat - this.lat) * 111320.0;
      const dLon = (this.targetWp.lon - this.lon) * (111320.0 * Math.cos((this.lat * Math.PI) / 180));
      const dist = Math.hypot(dLat, dLon);
      if (dist > 1.5) {
        const vx = (dLat / dist) * Math.min(8.5, dist * 0.6);
        const vy = (dLon / dist) * Math.min(8.5, dist * 0.6);
        this.lat += (vx * dt) / 111320.0;
        this.lon += (vy * dt) / (111320.0 * Math.cos((this.lat * Math.PI) / 180));
        this.yawDeg = ((Math.atan2(vy, vx) * 180) / Math.PI + 360) % 360;
        this.speedMs = Math.hypot(vx, vy);
        this.pitchDeg = -6.0;
        this.rollDeg = Math.sin(this.tSec * 1.4) * 4.5;
      } else {
        this.speedMs = 0.2;
        this.pitchDeg = 0;
        this.rollDeg = Math.sin(this.tSec) * 1.2;
        if (this.flightMode === 6) {
          this.altM = Math.max(0, this.altM - 1.5 * dt);
          if (this.altM < 0.2) {
            this.armed = false;
            this.flightMode = 0;
            this.targetWp = null;
          }
        } else {
          this.flightMode = 4; // LOITER
        }
      }
      if (this.targetWp && (this.flightMode !== 6 || dist > 1.5)) {
        const targetAlt = this.vehicleMode ? 0 : this.targetWp.altM;
        this.altM += (targetAlt - this.altM) * 0.4 * dt;
      }
    } else {
      // Autonomous figure-8 / orbit pattern around Hubballi test range
      const rMeters = 65.0;
      const omega = 0.14;
      const targetLat = this.homeLat + (rMeters * Math.sin(omega * this.tSec)) / 111320.0;
      const targetLon =
        this.homeLon +
        (rMeters * Math.sin(omega * this.tSec * 2.0)) /
          (111320.0 * Math.cos((this.homeLat * Math.PI) / 180));
      const vn = (targetLat - this.lat) * (111320.0 / dt);
      const ve = (targetLon - this.lon) * ((111320.0 * Math.cos((this.homeLat * Math.PI) / 180)) / dt);
      this.lat = targetLat;
      this.lon = targetLon;
      this.speedMs = Math.min(14.0, Math.hypot(vn, ve));
      this.yawDeg = ((Math.atan2(ve, vn) * 180) / Math.PI + 360) % 360;
      this.altM = this.vehicleMode ? 0 : 28.0 + Math.sin(this.tSec * 0.5) * 3.5;
      this.rollDeg = Math.sin(this.tSec * 0.9) * 11.5;
      this.pitchDeg = -5.5 + Math.cos(this.tSec * 0.7) * 3.0;
    }

    this.currA = this.vehicleMode ? 3.2 : 12.5 + this.speedMs * 0.85;
    this.vbatV = Math.max(10.3, this.vbatV - 0.0005 * this.currA * dt);
    this.activeLink = Math.floor(this.tSec / 12) % 3 === 0 ? 2 : 3; // alternates Wi-Fi UDP & LTE MQTT
  }

  private snapshot(): DecodedTelemetry {
    let flags = 0;
    if (this.armed) flags |= 1 << 0;
    if (this.sats >= 6) flags |= 1 << 1;
    flags |= 1 << 2; // OPTFLOW_OK
    if (this.flightMode === 6) flags |= 1 << 5; // FAILSAFE_RTH
    if (this.vehicleMode) flags |= 1 << 6;

    return {
      deviceId: this.deviceId,
      msgType: 1,
      seqId: this.seq,
      utcEpoch: Math.floor(Date.now() / 1000),
      timestampIso: new Date().toISOString(),
      lat: this.lat,
      lon: this.lon,
      altM: this.altM,
      rollDeg: this.rollDeg,
      pitchDeg: this.pitchDeg,
      yawDeg: this.yawDeg,
      speedMs: this.speedMs,
      vbatV: this.vbatV,
      currA: this.currA,
      flightMode: this.flightMode,
      flightModeName: 'MISSION',
      sats: this.sats,
      rssiScaled: this.rssiScaled,
      activeLink: this.activeLink,
      activeLinkName: 'LTE/NB-IoT MQTT',
      healthFlags: flags,
      flags: {
        armed: this.armed,
        gpsLock: true,
        optflowOk: true,
        geofenceHit: false,
        propAnomaly: false,
        failsafeRth: this.flightMode === 6,
        vehicleMode: this.vehicleMode,
        womAlert: false,
      },
      ledPattern: 'ARMED_FLIGHT',
      sourceProto: 'sitl',
    };
  }
}

