import 'dotenv/config';
import dgram from 'dgram';
import http from 'http';
import cors from 'cors';
import express from 'express';
import mqtt, { MqttClient } from 'mqtt';
import { Pool } from 'pg';
import { WebSocket, WebSocketServer } from 'ws';
import { buildAndSignCommand, CommandRequestInput } from './crypto';
import {
  DecodedTelemetry,
  decodeBinaryTelemetry,
  decodeJsonTelemetry,
  PAN_TELEM_MAGIC,
  PAN_TELEM_PKT_SIZE,
} from './decoder';
import { BuiltinSitlSimulator } from './sitlSim';

const PORT = Number(process.env.PORT ?? 8080);
const UDP_PORT = Number(process.env.UDP_PORT ?? 14550);
const UDP_CMD_PORT = Number(process.env.UDP_CMD_PORT ?? 14551);
const MQTT_URL = process.env.MQTT_URL ?? 'mqtt://localhost:1883';
const PG_URL = process.env.PG_URL ?? '';
const HMAC_SECRET = process.env.HMAC_SECRET ?? 'PAN_BRAIN_DEV_KEY_2026';
const DEFAULT_DEVICE_ID = process.env.DEVICE_ID ?? 'pan-brain-01';
const ENABLE_SITL_SIM = (process.env.ENABLE_SITL_SIM ?? 'true').toLowerCase() === 'true';

const app = express();
app.use(cors());
app.use(express.json());

const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: '/ws/telemetry' });

// In-memory ring buffers so the GCS remains fully responsive even without DB
const telemetryHistory: DecodedTelemetry[] = [];
const MAX_HISTORY = 1200;
let latestTelemetry: DecodedTelemetry | null = null;
const commandAuditMem: Array<Record<string, unknown>> = [];
const geofencesMem: Array<Record<string, unknown>> = [
  {
    id: 1,
    deviceId: DEFAULT_DEVICE_ID,
    name: 'Test Range Keep-In',
    fenceType: 'KEEP_IN',
    minAltM: 0,
    maxAltM: 120,
    polygon: [
      [15.3627, 75.1215],
      [15.3667, 75.1215],
      [15.3667, 75.1265],
      [15.3627, 75.1265],
    ],
  },
];

let lastUdpPeerIp: string | null = null;
let dbPool: Pool | null = null;
let dbConnected = false;
let mqttClient: MqttClient | null = null;
let mqttConnected = false;

const sitl = new BuiltinSitlSimulator(DEFAULT_DEVICE_ID);

function broadcastWs(payload: unknown): void {
  const raw = JSON.stringify(payload);
  wss.clients.forEach((client: WebSocket) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(raw);
    }
  });
}

async function ingestTelemetry(pkt: DecodedTelemetry): Promise<void> {
  latestTelemetry = pkt;
  telemetryHistory.push(pkt);
  if (telemetryHistory.length > MAX_HISTORY) {
    telemetryHistory.shift();
  }

  broadcastWs({ type: 'telemetry', data: pkt });

  if (dbPool && dbConnected) {
    try {
      await dbPool.query(
        `INSERT INTO telemetry_frames
          (ts, device_id, seq_id, msg_type, lat, lon, alt_m, roll_deg, pitch_deg, yaw_deg,
           speed_ms, vbat_v, curr_a, flight_mode, sats, rssi_csq, active_link, health_flags, source_proto)
         VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16, $17, $18, $19)`,
        [
          new Date(pkt.utcEpoch * 1000),
          pkt.deviceId,
          pkt.seqId,
          pkt.msgType,
          pkt.lat,
          pkt.lon,
          pkt.altM,
          pkt.rollDeg,
          pkt.pitchDeg,
          pkt.yawDeg,
          pkt.speedMs,
          pkt.vbatV,
          pkt.currA,
          pkt.flightMode,
          pkt.sats,
          pkt.rssiScaled,
          pkt.activeLink,
          pkt.healthFlags,
          pkt.sourceProto,
        ]
      );
    } catch (err) {
      console.error('[DB] insert error:', (err as Error).message);
    }
  }
}

// UDP 14550 Telemetry Listener (Wi-Fi AP / PC SITL binary or JSON frames)
const udpSocket = dgram.createSocket('udp4');
udpSocket.on('message', (msg, rinfo) => {
  lastUdpPeerIp = rinfo.address;
  let pkt: DecodedTelemetry | null = null;
  if (msg.length === PAN_TELEM_PKT_SIZE && msg[0] === PAN_TELEM_MAGIC) {
    pkt = decodeBinaryTelemetry(msg, DEFAULT_DEVICE_ID, 'bin36');
  } else {
    pkt = decodeJsonTelemetry(msg.toString('utf8'), DEFAULT_DEVICE_ID);
  }
  if (pkt) {
    if (sitl.isRunning()) {
      sitl.stop(); // Automatically yield to live hardware or C SITL packets
    }
    void ingestTelemetry(pkt);
  }
});
udpSocket.on('error', (err) => {
  console.warn('[UDP] socket warning:', err.message);
});
udpSocket.bind(UDP_PORT, () => {
  console.log(`[UDP] Listening for 36-byte/JSON telemetry on 0.0.0.0:${UDP_PORT}`);
});

// Optional MQTT Broker Listener
function initMqtt(): void {
  mqttClient = mqtt.connect(MQTT_URL, { reconnectPeriod: 5000 });
  mqttClient.on('connect', () => {
    mqttConnected = true;
    console.log(`[MQTT] Connected to ${MQTT_URL}`);
    mqttClient?.subscribe(['drone/+/telem', 'drone/+/alert']);
  });
  mqttClient.on('close', () => {
    mqttConnected = false;
  });
  mqttClient.on('error', () => {
    mqttConnected = false;
  });
  mqttClient.on('message', (topic, payload) => {
    const parts = topic.split('/');
    const devId = parts[1] || DEFAULT_DEVICE_ID;
    let pkt: DecodedTelemetry | null = null;
    if (payload.length === PAN_TELEM_PKT_SIZE && payload[0] === PAN_TELEM_MAGIC) {
      pkt = decodeBinaryTelemetry(payload, devId, 'bin36');
    } else {
      pkt = decodeJsonTelemetry(payload.toString('utf8'), devId);
    }
    if (pkt) {
      if (sitl.isRunning()) sitl.stop();
      void ingestTelemetry(pkt);
    }
  });
}

async function initDb(): Promise<void> {
  if (!PG_URL) return;
  dbPool = new Pool({ connectionString: PG_URL, connectionTimeoutMillis: 2500 });
  try {
    await dbPool.query('SELECT 1');
    dbConnected = true;
    console.log('[DB] Connected to TimescaleDB');
  } catch {
    dbConnected = false;
    console.warn('[DB] TimescaleDB unavailable; using in-memory ring buffer fallback.');
  }
}

// REST Endpoints
app.get('/api/health', (_req, res) => {
  res.json({
    status: 'ok',
    mqttConnected,
    dbConnected,
    sitlSimActive: sitl.isRunning(),
    lastUdpPeerIp,
    packetsBuffered: telemetryHistory.length,
  });
});

app.get('/api/telemetry/latest', (_req, res) => {
  res.json({
    latest: latestTelemetry,
    historyCount: telemetryHistory.length,
  });
});

app.get('/api/telemetry/history', (req, res) => {
  const limit = Math.min(MAX_HISTORY, Math.max(1, Number(req.query.limit ?? 200)));
  res.json({
    frames: telemetryHistory.slice(-limit),
  });
});

app.get('/api/geofences', async (_req, res) => {
  if (dbPool && dbConnected) {
    try {
      const q = await dbPool.query(
        'SELECT id, device_id AS "deviceId", name, fence_type AS "fenceType", min_alt_m AS "minAltM", max_alt_m AS "maxAltM", polygon_json AS polygon FROM geofences ORDER BY id'
      );
      return res.json({ geofences: q.rows });
    } catch {
      // fall through
    }
  }
  return res.json({ geofences: geofencesMem });
});

app.post('/api/geofences', async (req, res) => {
  const { name, fenceType, minAltM = 0, maxAltM = 120, polygon, deviceId = DEFAULT_DEVICE_ID } = req.body ?? {};
  if (!name || !Array.isArray(polygon) || polygon.length < 3) {
    return res.status(400).json({ error: 'Invalid geofence polygon' });
  }
  const entry = {
    id: geofencesMem.length + 1,
    deviceId,
    name,
    fenceType: fenceType === 'KEEP_OUT' ? 'KEEP_OUT' : 'KEEP_IN',
    minAltM: Number(minAltM),
    maxAltM: Number(maxAltM),
    polygon,
  };
  geofencesMem.push(entry);
  if (dbPool && dbConnected) {
    try {
      await dbPool.query(
        'INSERT INTO geofences (device_id, name, fence_type, min_alt_m, max_alt_m, polygon_json) VALUES ($1,$2,$3,$4,$5,$6)',
        [deviceId, entry.name, entry.fenceType, entry.minAltM, entry.maxAltM, JSON.stringify(polygon)]
      );
    } catch {
      // ignore if DB not initialized
    }
  }
  return res.json({ geofence: entry });
});

app.post('/api/cmd', async (req, res) => {
  const body = req.body as CommandRequestInput & { deviceId?: string };
  if (!body || !body.cmd) {
    return res.status(400).json({ error: 'Missing cmd field' });
  }

  const deviceId = body.deviceId ?? DEFAULT_DEVICE_ID;
  const envelope = buildAndSignCommand(body, HMAC_SECRET);
  const rawJson = JSON.stringify(envelope);

  const transports: string[] = [];

  if (mqttClient && mqttConnected) {
    mqttClient.publish(`drone/${deviceId}/cmd`, rawJson, { qos: 1 });
    transports.push('mqtt');
  }

  if (lastUdpPeerIp) {
    udpSocket.send(Buffer.from(rawJson, 'utf8'), UDP_CMD_PORT, lastUdpPeerIp);
    transports.push(`udp:${lastUdpPeerIp}:${UDP_CMD_PORT}`);
  }

  if (sitl.isRunning()) {
    sitl.applyCommand(envelope);
    transports.push('builtin-sitl');
  }

  const auditEntry = {
    ts: new Date().toISOString(),
    deviceId,
    cmd: envelope.cmd,
    nonce: envelope.nonce,
    hmacPrefix: envelope.hmac.slice(0, 16),
    transports,
    envelope,
  };
  commandAuditMem.unshift(auditEntry);
  if (commandAuditMem.length > 50) commandAuditMem.pop();

  if (dbPool && dbConnected) {
    try {
      await dbPool.query(
        'INSERT INTO command_audit (device_id, cmd, nonce, payload_json, hmac_hex, transport) VALUES ($1,$2,$3,$4,$5,$6)',
        [deviceId, envelope.cmd, envelope.nonce, rawJson, envelope.hmac, transports.join(',') || 'none']
      );
    } catch {
      // keep non-fatal
    }
  }

  broadcastWs({ type: 'cmd_audit', data: auditEntry });
  return res.json({ ok: true, audit: auditEntry });
});

app.get('/api/cmd/audit', (_req, res) => {
  res.json({ items: commandAuditMem });
});

app.post('/api/sim/toggle', (req, res) => {
  const enable = req.body?.enable ?? !sitl.isRunning();
  if (enable) {
    sitl.start((pkt) => void ingestTelemetry(pkt));
  } else {
    sitl.stop();
  }
  res.json({ sitlSimActive: sitl.isRunning() });
});

wss.on('connection', (ws) => {
  if (latestTelemetry) {
    ws.send(JSON.stringify({ type: 'telemetry', data: latestTelemetry }));
  }
  ws.send(
    JSON.stringify({
      type: 'snapshot',
      history: telemetryHistory.slice(-150),
      geofences: geofencesMem,
      sitlSimActive: sitl.isRunning(),
    })
  );
});

server.listen(PORT, () => {
  console.log(`[HTTP/WS] PAN-BRAIN Cloud GCS Backend running on http://localhost:${PORT}`);
  initMqtt();
  void initDb();
  if (ENABLE_SITL_SIM) {
    sitl.start((pkt) => void ingestTelemetry(pkt));
    console.log('[SITL-SIM] Built-in telemetry simulator started (yields automatically on live UDP/MQTT traffic)');
  }
});

