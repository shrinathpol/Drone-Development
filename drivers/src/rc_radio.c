#include "rc_radio.h"

#include <string.h>

#define CRSF_ADDR_FC        0xC8U
#define CRSF_ADDR_SYNC      0xEEU
#define CRSF_TYPE_RC_CHANS  0x16U
#define CRSF_TYPE_LINK_STAT 0x14U
#define SBUS_HEADER         0x0FU
#define SBUS_FRAME_LEN      25U
#define RC_TIMEOUT_MS       500U

static uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0xD5U);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

static float norm_stick(uint16_t us)
{
    float v = ((float)us - 1500.0f) / 500.0f;
    if (v > -0.02f && v < 0.02f) v = 0.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

static void update_mapped_controls(rc_radio_t *rc)
{
    rc->state.roll = norm_stick(rc->state.raw_ch[0]);
    rc->state.pitch = norm_stick(rc->state.raw_ch[1]);
    float thr = ((float)rc->state.raw_ch[2] - 1000.0f) / 1000.0f;
    if (thr < 0.0f) thr = 0.0f;
    if (thr > 1.0f) thr = 1.0f;
    rc->state.throttle = thr;
    rc->state.yaw = norm_stick(rc->state.raw_ch[3]);

    rc->state.arm_switch = (rc->state.raw_ch[4] > 1600U);
    if (rc->state.raw_ch[5] < 1300U) {
        rc->state.mode_switch = 0U;
    } else if (rc->state.raw_ch[5] < 1700U) {
        rc->state.mode_switch = 1U;
    } else {
        rc->state.mode_switch = 2U;
    }
    rc->state.rth_switch = (rc->state.raw_ch[6] > 1600U);
}

static void decode_crsf_frame(rc_radio_t *rc, const uint8_t *frame, size_t total_len)
{
    uint8_t len = frame[1];
    if (total_len < (size_t)(len + 2U) || len < 3U) {
        return;
    }
    uint8_t crc = crsf_crc8(&frame[2], (uint8_t)(len - 1U));
    if (crc != frame[len + 1U]) {
        return;
    }

    uint8_t type = frame[2];
    const uint8_t *p = &frame[3];
    if (type == CRSF_TYPE_RC_CHANS && len >= 24U) {
        uint32_t bitbuf = 0U;
        uint8_t bits = 0U;
        size_t p_idx = 0U;
        for (uint8_t ch = 0U; ch < RC_MAX_CHANNELS; ch++) {
            while (bits < 11U && p_idx < 22U) {
                bitbuf |= ((uint32_t)p[p_idx++]) << bits;
                bits += 8U;
            }
            uint16_t raw11 = (uint16_t)(bitbuf & 0x07FFU);
            bitbuf >>= 11U;
            bits -= 11U;
            /* CRSF 172..1811 -> 988..2012 us */
            uint16_t us = (uint16_t)(1500 + (((int32_t)raw11 - 992) * 5) / 8);
            if (us < 900U) us = 900U;
            if (us > 2100U) us = 2100U;
            rc->state.raw_ch[ch] = us;
        }
        rc->state.protocol = RC_PROTO_CRSF;
        rc->state.failsafe = false;
        rc->state.signal_valid = true;
        rc->state.last_frame_ms = rc->sys->millis();
        update_mapped_controls(rc);
    } else if (type == CRSF_TYPE_LINK_STAT && len >= 12U) {
        rc->state.rssi_dbm = -(int8_t)p[0];
        rc->state.link_quality_pct = p[2];
    }
}

static void decode_sbus_frame(rc_radio_t *rc, const uint8_t *f)
{
    if (f[0] != SBUS_HEADER || (f[24] != 0x00U && (f[24] & 0x0FU) != 0x04U)) {
        return;
    }
    uint32_t bitbuf = 0U;
    uint8_t bits = 0U;
    size_t idx = 1U;
    for (uint8_t ch = 0U; ch < RC_MAX_CHANNELS; ch++) {
        while (bits < 11U && idx <= 22U) {
            bitbuf |= ((uint32_t)f[idx++]) << bits;
            bits += 8U;
        }
        uint16_t raw = (uint16_t)(bitbuf & 0x07FFU);
        bitbuf >>= 11U;
        bits -= 11U;
        uint16_t us = (uint16_t)(1500 + (((int32_t)raw - 992) * 5) / 8);
        rc->state.raw_ch[ch] = us;
    }
    bool fs = (f[23] & (1U << 3U)) != 0U;
    rc->state.protocol = RC_PROTO_SBUS;
    rc->state.failsafe = fs;
    rc->state.signal_valid = !fs;
    rc->state.link_quality_pct = fs ? 0U : 100U;
    rc->state.last_frame_ms = rc->sys->millis();
    update_mapped_controls(rc);
}

int rc_radio_init(rc_radio_t *rc, hal_uart_t *uart, hal_sys_t *sys, bool use_sbus)
{
    if (rc == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(rc, 0, sizeof(*rc));
    rc->uart = uart;
    rc->sys = sys;
    rc->use_sbus = use_sbus;

    for (uint8_t i = 0U; i < RC_MAX_CHANNELS; i++) {
        rc->state.raw_ch[i] = (i == 2U) ? 1000U : 1500U;
    }
    rc->state.raw_ch[4] = 1000U;
    rc->state.raw_ch[5] = 1000U;
    rc->state.raw_ch[6] = 1000U;
    update_mapped_controls(rc);

    if (uart != NULL && uart->init != NULL) {
        uart->init(uart->ctx, use_sbus ? 100000U : 420000U);
    }
    return HAL_OK;
}

void rc_radio_poll(rc_radio_t *rc)
{
    if (rc == NULL || rc->sys == NULL) {
        return;
    }
    if (rc->sim_override) {
        update_mapped_controls(rc);
        return;
    }

    if (rc->uart != NULL && rc->uart->read != NULL) {
        uint8_t buf[64];
        int n = rc->uart->read(rc->uart->ctx, buf, sizeof(buf));
        while (n > 0) {
            for (int i = 0; i < n; i++) {
                uint8_t b = buf[i];
                if (rc->use_sbus) {
                    if (rc->rx_idx == 0U && b != SBUS_HEADER) {
                        continue;
                    }
                    rc->rx_buf[rc->rx_idx++] = b;
                    if (rc->rx_idx >= SBUS_FRAME_LEN) {
                        decode_sbus_frame(rc, rc->rx_buf);
                        rc->rx_idx = 0U;
                    }
                } else {
                    if (rc->rx_idx == 0U) {
                        if (b == CRSF_ADDR_FC || b == CRSF_ADDR_SYNC) {
                            rc->rx_buf[rc->rx_idx++] = b;
                        }
                    } else if (rc->rx_idx == 1U) {
                        if (b >= 2U && b <= 62U) {
                            rc->rx_buf[rc->rx_idx++] = b;
                            rc->expected_len = (uint8_t)(b + 2U);
                        } else {
                            rc->rx_idx = 0U;
                        }
                    } else {
                        rc->rx_buf[rc->rx_idx++] = b;
                        if (rc->rx_idx >= rc->expected_len) {
                            decode_crsf_frame(rc, rc->rx_buf, rc->rx_idx);
                            rc->rx_idx = 0U;
                        }
                    }
                }
            }
            n = rc->uart->read(rc->uart->ctx, buf, sizeof(buf));
        }
    }

    uint32_t now = rc->sys->millis();
    if (rc->state.last_frame_ms == 0U || (now - rc->state.last_frame_ms) > RC_TIMEOUT_MS) {
        rc->state.failsafe = true;
        rc->state.signal_valid = false;
    }
}

const rc_state_t *rc_radio_get_state(const rc_radio_t *rc)
{
    return (rc != NULL) ? &rc->state : NULL;
}
