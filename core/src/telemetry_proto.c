#include "telemetry_proto.h"

#include <stdio.h>
#include <string.h>

uint16_t telemetry_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL) {
        return crc;
    }
    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t b = 0U; b < 8U; b++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

void telemetry_finalize_pkt(pan_telem_pkt_t *pkt)
{
    if (pkt == NULL) {
        return;
    }
    pkt->magic = PAN_TELEM_MAGIC;
    pkt->crc16 = telemetry_crc16_ccitt((const uint8_t *)pkt, sizeof(pan_telem_pkt_t) - sizeof(uint16_t));
}

bool telemetry_verify_pkt(const uint8_t *raw, size_t len, pan_telem_pkt_t *out_pkt)
{
    if (raw == NULL || len != sizeof(pan_telem_pkt_t)) {
        return false;
    }
    pan_telem_pkt_t tmp;
    memcpy(&tmp, raw, sizeof(tmp));
    if (tmp.magic != PAN_TELEM_MAGIC) {
        return false;
    }
    uint16_t expected = telemetry_crc16_ccitt(raw, sizeof(pan_telem_pkt_t) - sizeof(uint16_t));
    if (tmp.crc16 != expected) {
        return false;
    }
    if (out_pkt != NULL) {
        *out_pkt = tmp;
    }
    return true;
}

size_t telemetry_format_json(const pan_telem_pkt_t *pkt,
                             const char *device_id,
                             char *out_buf,
                             size_t out_buf_len)
{
    if (pkt == NULL || out_buf == NULL || out_buf_len == 0U) {
        return 0U;
    }
    const char *id = (device_id != NULL) ? device_id : "drone_brain_01";
    int n = snprintf(
        out_buf,
        out_buf_len,
        "{\"id\":\"%s\",\"seq\":%u,\"ts\":%lu,\"lat\":%.7f,\"lon\":%.7f,"
        "\"alt\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
        "\"spd\":%.2f,\"vbat\":%.3f,\"curr\":%.2f,\"mode\":%u,"
        "\"sats\":%u,\"rssi\":%u,\"link\":%u,\"flags\":%u}",
        id,
        (unsigned)pkt->seq_id,
        (unsigned long)pkt->utc_epoch,
        (double)pkt->lat_e7 / 1e7,
        (double)pkt->lon_e7 / 1e7,
        (double)pkt->alt_cm / 100.0,
        (double)pkt->roll_cdeg / 100.0,
        (double)pkt->pitch_cdeg / 100.0,
        (double)pkt->yaw_cdeg / 100.0,
        (double)pkt->speed_cms / 100.0,
        (double)pkt->vbat_mv / 1000.0,
        (double)pkt->curr_ca / 100.0,
        (unsigned)pkt->flight_mode,
        (unsigned)((pkt->sats_rssi >> 4U) & 0x0FU),
        (unsigned)(pkt->sats_rssi & 0x0FU),
        (unsigned)pkt->active_link,
        (unsigned)pkt->health_flags);
    return (n > 0) ? (size_t)n : 0U;
}
