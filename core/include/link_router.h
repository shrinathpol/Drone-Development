#ifndef LINK_ROUTER_H
#define LINK_ROUTER_H

#include "crypto_auth.h"
#include "hal_interfaces.h"
#include "modem_sim7000g.h"
#include "offline_queue.h"
#include "rc_radio.h"
#include "telemetry_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINK_ID_NONE = 0,
    LINK_ID_CRSF = 1,     /* 5ms sub-GHz RC + basic telemetry */
    LINK_ID_WIFI_UDP = 2, /* 15ms short-range high-rate UDP */
    LINK_ID_LTE_MQTT = 3, /* 150ms BVLOS cellular cloud MQTT */
    LINK_ID_GSM_SMS = 4   /* Last-resort emergency SMS */
} link_type_t;

typedef struct {
    hal_sys_t *sys;
    sim7000g_dev_t *modem;
    hal_wireless_t *wifi;
    rc_radio_t *rc;
    offline_queue_t *offline_q;
    crypto_auth_ctx_t *auth;

    link_type_t active_link;
    const char *device_id;
    const char *emergency_phone;
    uint32_t last_lte_tx_ms;
    uint32_t last_wifi_tx_ms;
    uint32_t last_backlog_drip_ms;
    bool use_binary_mqtt;
} link_router_t;

void link_router_init(link_router_t *router,
                      hal_sys_t *sys,
                      sim7000g_dev_t *modem,
                      hal_wireless_t *wifi,
                      rc_radio_t *rc,
                      offline_queue_t *offline_q,
                      crypto_auth_ctx_t *auth,
                      const char *device_id,
                      const char *emergency_phone);

void link_router_send_telemetry(link_router_t *router, const pan_telem_pkt_t *pkt);

bool link_router_poll_wifi_command(link_router_t *router, uint32_t utc_epoch, pan_cloud_cmd_t *out_cmd);

void link_router_send_emergency_sms(link_router_t *router, const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* LINK_ROUTER_H */

