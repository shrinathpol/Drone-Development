#ifndef RC_RADIO_H
#define RC_RADIO_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_MAX_CHANNELS 16U

typedef enum {
    RC_PROTO_NONE = 0,
    RC_PROTO_CRSF,
    RC_PROTO_SBUS
} rc_protocol_t;

typedef struct {
    uint16_t raw_ch[RC_MAX_CHANNELS]; /* 1000..2000 us equivalent */
    float roll;       /* [-1.0, +1.0] */
    float pitch;      /* [-1.0, +1.0] */
    float yaw;        /* [-1.0, +1.0] */
    float throttle;   /* [0.0, 1.0] */
    bool arm_switch;  /* CH5 > 1500 */
    uint8_t mode_switch; /* 0=Stabilize, 1=Loiter/AltHold, 2=Mission */
    bool rth_switch;  /* CH7 > 1500 */
    uint8_t link_quality_pct;
    int8_t rssi_dbm;
    bool failsafe;
    bool signal_valid;
    uint32_t last_frame_ms;
    rc_protocol_t protocol;
} rc_state_t;

typedef struct {
    hal_uart_t *uart;
    hal_sys_t *sys;

    uint8_t rx_buf[64];
    size_t rx_idx;
    uint8_t expected_len;
    bool use_sbus;

    rc_state_t state;
    bool sim_override;
} rc_radio_t;

int rc_radio_init(rc_radio_t *rc, hal_uart_t *uart, hal_sys_t *sys, bool use_sbus);
void rc_radio_poll(rc_radio_t *rc);
const rc_state_t *rc_radio_get_state(const rc_radio_t *rc);

#ifdef __cplusplus
}
#endif

#endif /* RC_RADIO_H */
