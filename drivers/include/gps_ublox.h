#ifndef GPS_UBLOX_H
#define GPS_UBLOX_H

#include "hal_interfaces.h"
#include "modem_sim7000g.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPS_SRC_NONE = 0,
    GPS_SRC_EXT_UBLOX,
    GPS_SRC_SIM7000G_FALLBACK
} gps_source_t;

typedef struct {
    bool fix_valid;
    uint8_t fix_type;
    uint8_t satellites;
    double latitude;
    double longitude;
    float altitude_msl_m;
    float vel_n_ms;
    float vel_e_ms;
    float vel_d_ms;
    float ground_speed_ms;
    float heading_deg;
    float hdop;
    uint32_t utc_epoch;
    uint32_t last_fix_ms;
    gps_source_t active_source;
} gps_nav_data_t;

typedef struct {
    hal_uart_t *uart;
    hal_sys_t *sys;
    sim7000g_dev_t *fallback_modem;

    /* UBX binary parser state */
    uint8_t ubx_state;
    uint8_t ubx_class;
    uint8_t ubx_id;
    uint16_t ubx_len;
    uint16_t ubx_idx;
    uint8_t ubx_ck_a;
    uint8_t ubx_ck_b;
    uint8_t ubx_payload[128];

    /* NMEA line parser buffer */
    char nmea_buf[128];
    size_t nmea_idx;

    gps_nav_data_t ext_data;
    gps_nav_data_t active;
    uint32_t last_ext_rx_ms;
} gps_ublox_t;

int gps_ublox_init(gps_ublox_t *gps, hal_uart_t *uart, hal_sys_t *sys, sim7000g_dev_t *fallback_modem);
void gps_ublox_poll(gps_ublox_t *gps);
const gps_nav_data_t *gps_ublox_get_active(const gps_ublox_t *gps);

#ifdef __cplusplus
}
#endif

#endif /* GPS_UBLOX_H */
