#include "gps_ublox.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define UBX_SYNC1 0xB5U
#define UBX_SYNC2 0x62U
#define EXT_GPS_TIMEOUT_MS 2000U

static int32_t read_le32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
                     ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U));
}

static uint16_t read_le16u(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

static void handle_ubx_nav_pvt(gps_ublox_t *gps)
{
    if (gps->ubx_len < 92U) {
        return;
    }
    const uint8_t *p = gps->ubx_payload;
    uint8_t fix_type = p[20];
    uint8_t flags = p[21];
    uint8_t num_sv = p[23];
    int32_t lon_1e7 = read_le32(&p[24]);
    int32_t lat_1e7 = read_le32(&p[28]);
    int32_t hmsl_mm = read_le32(&p[36]);
    int32_t vel_n_mms = read_le32(&p[48]);
    int32_t vel_e_mms = read_le32(&p[52]);
    int32_t vel_d_mms = read_le32(&p[56]);
    int32_t gspeed_mms = read_le32(&p[60]);
    int32_t head_1e5 = read_le32(&p[64]);
    uint16_t pdop_01 = read_le16u(&p[76]);

    gps->ext_data.fix_type = fix_type;
    gps->ext_data.fix_valid = (fix_type >= 3U) && ((flags & 0x01U) != 0U);
    gps->ext_data.satellites = num_sv;
    gps->ext_data.longitude = (double)lon_1e7 * 1e-7;
    gps->ext_data.latitude = (double)lat_1e7 * 1e-7;
    gps->ext_data.altitude_msl_m = (float)hmsl_mm * 0.001f;
    gps->ext_data.vel_n_ms = (float)vel_n_mms * 0.001f;
    gps->ext_data.vel_e_ms = (float)vel_e_mms * 0.001f;
    gps->ext_data.vel_d_ms = (float)vel_d_mms * 0.001f;
    gps->ext_data.ground_speed_ms = (float)gspeed_mms * 0.001f;
    gps->ext_data.heading_deg = (float)head_1e5 * 1e-5f;
    gps->ext_data.hdop = (float)pdop_01 * 0.01f;
    gps->ext_data.last_fix_ms = gps->sys->millis();
    gps->last_ext_rx_ms = gps->ext_data.last_fix_ms;
}

static double nmea_deg_to_decimal(const char *s, char dir)
{
    if (s == NULL || s[0] == '\0') {
        return 0.0;
    }
    double raw = strtod(s, NULL);
    int deg = (int)(raw / 100.0);
    double mins = raw - (double)deg * 100.0;
    double val = (double)deg + mins / 60.0;
    if (dir == 'S' || dir == 'W') {
        val = -val;
    }
    return val;
}

static void parse_nmea_sentence(gps_ublox_t *gps, char *line)
{
    if (line[0] != '$') {
        return;
    }
    char *fields[18];
    size_t count = 0U;
    fields[count++] = line;
    for (char *p = line; *p != '\0' && count < 18U; p++) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    if (strncmp(fields[0] + 3, "GGA", 3) == 0 && count >= 10U) {
        int q = atoi(fields[6]);
        gps->ext_data.fix_valid = (q > 0);
        if (gps->ext_data.fix_valid) {
            gps->ext_data.latitude = nmea_deg_to_decimal(fields[2], fields[3][0]);
            gps->ext_data.longitude = nmea_deg_to_decimal(fields[4], fields[5][0]);
            gps->ext_data.satellites = (uint8_t)atoi(fields[7]);
            gps->ext_data.hdop = (float)strtod(fields[8], NULL);
            gps->ext_data.altitude_msl_m = (float)strtod(fields[9], NULL);
            gps->ext_data.fix_type = 3U;
            gps->ext_data.last_fix_ms = gps->sys->millis();
            gps->last_ext_rx_ms = gps->ext_data.last_fix_ms;
        }
    } else if (strncmp(fields[0] + 3, "RMC", 3) == 0 && count >= 9U) {
        if (fields[2][0] == 'A') {
            gps->ext_data.ground_speed_ms = (float)strtod(fields[7], NULL) * 0.514444f;
            gps->ext_data.heading_deg = (float)strtod(fields[8], NULL);
            float rad = gps->ext_data.heading_deg * 0.0174532925f;
            gps->ext_data.vel_n_ms = gps->ext_data.ground_speed_ms * cosf(rad);
            gps->ext_data.vel_e_ms = gps->ext_data.ground_speed_ms * sinf(rad);
            gps->last_ext_rx_ms = gps->sys->millis();
        }
    }
}

static void feed_byte(gps_ublox_t *gps, uint8_t b)
{
    /* UBX Binary Parser */
    switch (gps->ubx_state) {
    case 0U:
        if (b == UBX_SYNC1) {
            gps->ubx_state = 1U;
        }
        break;
    case 1U:
        gps->ubx_state = (b == UBX_SYNC2) ? 2U : 0U;
        gps->ubx_ck_a = 0U;
        gps->ubx_ck_b = 0U;
        break;
    case 2U:
        gps->ubx_class = b;
        gps->ubx_ck_a += b;
        gps->ubx_ck_b += gps->ubx_ck_a;
        gps->ubx_state = 3U;
        break;
    case 3U:
        gps->ubx_id = b;
        gps->ubx_ck_a += b;
        gps->ubx_ck_b += gps->ubx_ck_a;
        gps->ubx_state = 4U;
        break;
    case 4U:
        gps->ubx_len = b;
        gps->ubx_ck_a += b;
        gps->ubx_ck_b += gps->ubx_ck_a;
        gps->ubx_state = 5U;
        break;
    case 5U:
        gps->ubx_len |= (uint16_t)(b << 8U);
        gps->ubx_ck_a += b;
        gps->ubx_ck_b += gps->ubx_ck_a;
        gps->ubx_idx = 0U;
        if (gps->ubx_len == 0U || gps->ubx_len > sizeof(gps->ubx_payload)) {
            gps->ubx_state = 0U;
        } else {
            gps->ubx_state = 6U;
        }
        break;
    case 6U:
        gps->ubx_payload[gps->ubx_idx++] = b;
        gps->ubx_ck_a += b;
        gps->ubx_ck_b += gps->ubx_ck_a;
        if (gps->ubx_idx >= gps->ubx_len) {
            gps->ubx_state = 7U;
        }
        break;
    case 7U:
        gps->ubx_state = (b == gps->ubx_ck_a) ? 8U : 0U;
        break;
    case 8U:
        if (b == gps->ubx_ck_b && gps->ubx_class == 0x01U && gps->ubx_id == 0x07U) {
            handle_ubx_nav_pvt(gps);
        }
        gps->ubx_state = 0U;
        break;
    default:
        gps->ubx_state = 0U;
        break;
    }

    /* Parallel NMEA Line Parser */
    if (b == '$') {
        gps->nmea_idx = 0U;
        gps->nmea_buf[gps->nmea_idx++] = '$';
    } else if (gps->nmea_idx > 0U) {
        if (b == '\r' || b == '\n') {
            gps->nmea_buf[gps->nmea_idx] = '\0';
            parse_nmea_sentence(gps, gps->nmea_buf);
            gps->nmea_idx = 0U;
        } else if (gps->nmea_idx < sizeof(gps->nmea_buf) - 1U) {
            gps->nmea_buf[gps->nmea_idx++] = (char)b;
        } else {
            gps->nmea_idx = 0U;
        }
    }
}

int gps_ublox_init(gps_ublox_t *gps, hal_uart_t *uart, hal_sys_t *sys, sim7000g_dev_t *fallback_modem)
{
    if (gps == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(gps, 0, sizeof(*gps));
    gps->uart = uart;
    gps->sys = sys;
    gps->fallback_modem = fallback_modem;
    gps->ext_data.hdop = 99.9f;
    gps->active.hdop = 99.9f;

    if (uart != NULL && uart->init != NULL) {
        uart->init(uart->ctx, 115200U);
    }
    return HAL_OK;
}

void gps_ublox_poll(gps_ublox_t *gps)
{
    if (gps == NULL || gps->sys == NULL) {
        return;
    }

    if (gps->uart != NULL && gps->uart->read != NULL) {
        uint8_t buf[128];
        int n = gps->uart->read(gps->uart->ctx, buf, sizeof(buf));
        while (n > 0) {
            for (int i = 0; i < n; i++) {
                feed_byte(gps, buf[i]);
            }
            n = gps->uart->read(gps->uart->ctx, buf, sizeof(buf));
        }
    }

    uint32_t now = gps->sys->millis();
    bool ext_alive = (gps->last_ext_rx_ms > 0U) && ((now - gps->last_ext_rx_ms) <= EXT_GPS_TIMEOUT_MS);

    if (ext_alive && gps->ext_data.fix_valid) {
        gps->active = gps->ext_data;
        gps->active.active_source = GPS_SRC_EXT_UBLOX;
    } else if (gps->fallback_modem != NULL && gps->fallback_modem->gnss.fix_valid) {
        const modem_gnss_t *mg = &gps->fallback_modem->gnss;
        gps->active.fix_valid = true;
        gps->active.fix_type = 3U;
        gps->active.latitude = mg->latitude;
        gps->active.longitude = mg->longitude;
        gps->active.altitude_msl_m = mg->altitude_m;
        gps->active.ground_speed_ms = mg->speed_kmh / 3.6f;
        gps->active.heading_deg = mg->heading_deg;
        float rad = mg->heading_deg * 0.0174532925f;
        gps->active.vel_n_ms = gps->active.ground_speed_ms * cosf(rad);
        gps->active.vel_e_ms = gps->active.ground_speed_ms * sinf(rad);
        gps->active.vel_d_ms = 0.0f;
        gps->active.satellites = mg->satellites;
        gps->active.hdop = 2.0f;
        gps->active.utc_epoch = mg->utc_epoch;
        gps->active.active_source = GPS_SRC_SIM7000G_FALLBACK;
    } else {
        gps->active.fix_valid = false;
        gps->active.active_source = GPS_SRC_NONE;
    }
}

const gps_nav_data_t *gps_ublox_get_active(const gps_ublox_t *gps)
{
    return (gps != NULL) ? &gps->active : NULL;
}
