#ifndef TELEMETRY_PROTO_H
#define TELEMETRY_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PAN_TELEM_MAGIC        0xA5U
#define PAN_TELEM_MSG_FAST_HUD 0x01U
#define PAN_TELEM_MSG_GPS_NAV  0x02U
#define PAN_TELEM_MSG_HEALTH   0x03U

/* Health & Alert bitflags in pan_telem_pkt_t.health_flags */
#define PAN_HEALTH_ARMED        (1U << 0)
#define PAN_HEALTH_GPS_LOCK     (1U << 1)
#define PAN_HEALTH_OPTFLOW_OK   (1U << 2)
#define PAN_HEALTH_GEOFENCE_HIT (1U << 3)
#define PAN_HEALTH_PROP_ANOMALY (1U << 4)
#define PAN_HEALTH_FAILSAFE_RTH (1U << 5)
#define PAN_HEALTH_VEHICLE_MODE (1U << 6)
#define PAN_HEALTH_WOM_ALERT    (1U << 7)

/**
 * @brief Flight modes encoded in pan_telem_pkt_t.flight_mode
 */
typedef enum {
    PAN_MODE_DISARMED = 0,
    PAN_MODE_MANUAL_ACRO = 1,
    PAN_MODE_STABILIZE = 2,
    PAN_MODE_ALT_HOLD = 3,
    PAN_MODE_LOITER = 4,
    PAN_MODE_MISSION = 5,
    PAN_MODE_SMART_RTH = 6,
    PAN_MODE_AUTOLAND = 7,
    PAN_MODE_VEHICLE_ACTIVE = 8,
    PAN_MODE_VEHICLE_SLEEP = 9,
    PAN_MODE_EMERGENCY_KILL = 10
} pan_flight_mode_t;

/**
 * @brief Compact 36-byte packed binary telemetry packet (Little-Endian, CRC-16-CCITT).
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;        /* 0xA5 */
    uint8_t  msg_type;     /* 0x01 = Fast HUD, 0x02 = GPS/Nav, 0x03 = Health */
    uint16_t seq_id;
    uint32_t utc_epoch;    /* GSM/GPS synced UTC epoch (seconds) */
    int32_t  lat_e7;       /* Latitude * 1e7 */
    int32_t  lon_e7;       /* Longitude * 1e7 */
    int16_t  alt_cm;       /* Altitude AGL in cm */
    int16_t  roll_cdeg;    /* Roll * 100 (-18000..18000) */
    int16_t  pitch_cdeg;   /* Pitch * 100 (-9000..9000) */
    uint16_t yaw_cdeg;     /* Yaw * 100 (0..36000) */
    uint16_t speed_cms;    /* Ground speed in cm/s */
    uint16_t vbat_mv;      /* Battery voltage in mV */
    uint16_t curr_ca;      /* Battery current in centi-amps (10mA units) */
    uint8_t  flight_mode;  /* pan_flight_mode_t */
    uint8_t  sats_rssi;    /* Upper 4 bits: Sats (0..15), Lower 4 bits: Scaled CSQ (0..15) */
    uint8_t  active_link;  /* Active link ID (1=CRSF, 2=WiFi, 3=LTE, 4=SMS) */
    uint8_t  health_flags; /* PAN_HEALTH_* bitmask */
    uint16_t crc16;        /* CRC-16-CCITT (poly 0x1021, init 0xFFFF) over bytes [0..33] */
} pan_telem_pkt_t;

_Static_assert(sizeof(pan_telem_pkt_t) == 36, "pan_telem_pkt_t must remain exactly 36 bytes");

uint16_t telemetry_crc16_ccitt(const uint8_t *data, size_t len);

void telemetry_finalize_pkt(pan_telem_pkt_t *pkt);

bool telemetry_verify_pkt(const uint8_t *raw, size_t len, pan_telem_pkt_t *out_pkt);

size_t telemetry_format_json(const pan_telem_pkt_t *pkt,
                             const char *device_id,
                             char *out_buf,
                             size_t out_buf_len);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PROTO_H */
