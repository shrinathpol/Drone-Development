-- PAN-BRAIN Cloud Telemetry & Command Audit Schema (TimescaleDB / PostgreSQL 15)

CREATE EXTENSION IF NOT EXISTS timescaledb;

CREATE TABLE IF NOT EXISTS telemetry_frames (
    ts            TIMESTAMPTZ      NOT NULL,
    device_id     TEXT             NOT NULL,
    seq_id        INTEGER          NOT NULL,
    msg_type      SMALLINT         NOT NULL,
    lat           DOUBLE PRECISION NOT NULL,
    lon           DOUBLE PRECISION NOT NULL,
    alt_m         REAL             NOT NULL,
    roll_deg      REAL             NOT NULL,
    pitch_deg     REAL             NOT NULL,
    yaw_deg       REAL             NOT NULL,
    speed_ms      REAL             NOT NULL,
    vbat_v        REAL             NOT NULL,
    curr_a        REAL             NOT NULL,
    flight_mode   SMALLINT         NOT NULL,
    sats          SMALLINT         NOT NULL,
    rssi_csq      SMALLINT         NOT NULL,
    active_link   SMALLINT         NOT NULL,
    health_flags  INTEGER          NOT NULL,
    source_proto  TEXT             NOT NULL DEFAULT 'bin36'
);

SELECT create_hypertable('telemetry_frames', 'ts', if_not_exists => TRUE);

CREATE INDEX IF NOT EXISTS idx_telemetry_device_ts
    ON telemetry_frames (device_id, ts DESC);

CREATE TABLE IF NOT EXISTS command_audit (
    id            BIGSERIAL        PRIMARY KEY,
    ts            TIMESTAMPTZ      NOT NULL DEFAULT NOW(),
    device_id     TEXT             NOT NULL,
    cmd           TEXT             NOT NULL,
    nonce         BIGINT           NOT NULL,
    payload_json  JSONB            NOT NULL,
    hmac_hex      TEXT             NOT NULL,
    transport     TEXT             NOT NULL
);

CREATE TABLE IF NOT EXISTS geofences (
    id            SERIAL           PRIMARY KEY,
    device_id     TEXT             NOT NULL,
    name          TEXT             NOT NULL,
    fence_type    TEXT             NOT NULL CHECK (fence_type IN ('KEEP_IN', 'KEEP_OUT')),
    min_alt_m     REAL             NOT NULL DEFAULT -10.0,
    max_alt_m     REAL             NOT NULL DEFAULT 120.0,
    polygon_json  JSONB            NOT NULL,
    updated_at    TIMESTAMPTZ      NOT NULL DEFAULT NOW()
);

CREATE TABLE IF NOT EXISTS mission_waypoints (
    id            SERIAL           PRIMARY KEY,
    device_id     TEXT             NOT NULL,
    seq           INTEGER          NOT NULL,
    lat           DOUBLE PRECISION NOT NULL,
    lon           DOUBLE PRECISION NOT NULL,
    alt_m         REAL             NOT NULL DEFAULT 15.0,
    created_at    TIMESTAMPTZ      NOT NULL DEFAULT NOW()
);

