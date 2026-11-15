#include "link_router.h"

#include <stdio.h>
#include <string.h>

void link_router_init(link_router_t *router,
                      hal_sys_t *sys,
                      sim7000g_dev_t *modem,
                      hal_wireless_t *wifi,
                      rc_radio_t *rc,
                      offline_queue_t *offline_q,
                      crypto_auth_ctx_t *auth,
                      const char *device_id,
                      const char *emergency_phone)
{
    if (router == NULL) {
        return;
    }
    memset(router, 0, sizeof(*router));
    router->sys = sys;
    router->modem = modem;
    router->wifi = wifi;
    router->rc = rc;
    router->offline_q = offline_q;
    router->auth = auth;
    router->device_id = (device_id != NULL) ? device_id : "drone_brain_01";
    router->emergency_phone = emergency_phone;
    router->use_binary_mqtt = true;
    router->active_link = LINK_ID_LTE_MQTT;
}

void link_router_send_telemetry(link_router_t *router, const pan_telem_pkt_t *pkt)
{
    if (router == NULL || router->sys == NULL || pkt == NULL) {
        return;
    }

    uint32_t now = router->sys->millis();
    bool rc_ok = (router->rc != NULL && router->rc->state.signal_valid);
    bool lte_ok = (router->modem != NULL && router->modem->is_mqtt_connected);

    if (rc_ok) {
        router->active_link = LINK_ID_CRSF;
    } else if (lte_ok) {
        router->active_link = LINK_ID_LTE_MQTT;
    } else if (router->wifi != NULL) {
        router->active_link = LINK_ID_WIFI_UDP;
    } else {
        router->active_link = LINK_ID_GSM_SMS;
    }

    /* Link 2: Local Wi-Fi UDP broadcast at up to 20Hz when available */
    if (router->wifi != NULL && router->wifi->send_udp_telem != NULL &&
        (now - router->last_wifi_tx_ms) >= 50U) {
        router->last_wifi_tx_ms = now;
        router->wifi->send_udp_telem(router->wifi->ctx, (const uint8_t *)pkt, sizeof(*pkt));
    }

    /* Link 3: Cellular LTE MQTT (1Hz-5Hz) + Offline PSRAM/SD Buffer */
    if ((now - router->last_lte_tx_ms) >= 500U) {
        router->last_lte_tx_ms = now;
        char topic[64];
        snprintf(topic, sizeof(topic), "drone/%s/telemetry", router->device_id);

        if (lte_ok) {
            if (sim7000g_mqtt_publish(router->modem, topic, pkt, sizeof(*pkt)) != HAL_OK &&
                router->offline_q != NULL) {
                offline_queue_push(router->offline_q, pkt, true);
            }
        } else if (router->offline_q != NULL) {
            offline_queue_push(router->offline_q, pkt, false);
        }
    }

    /* Drip stored offline PSRAM/SD packets back to cloud when LTE is online */
    if (lte_ok && router->offline_q != NULL &&
        (now - router->last_backlog_drip_ms) >= 750U &&
        !router->modem->pub_pending) {
        pan_telem_pkt_t old_pkt;
        if (offline_queue_pop(router->offline_q, &old_pkt)) {
            router->last_backlog_drip_ms = now;
            char topic[64];
            snprintf(topic, sizeof(topic), "drone/%s/telemetry", router->device_id);
            sim7000g_mqtt_publish(router->modem, topic, &old_pkt, sizeof(old_pkt));
        }
    }
}

bool link_router_poll_wifi_command(link_router_t *router, uint32_t utc_epoch, pan_cloud_cmd_t *out_cmd)
{
    if (router == NULL || router->wifi == NULL || router->wifi->poll_rx_cmd == NULL ||
        router->auth == NULL || out_cmd == NULL) {
        return false;
    }
    uint8_t buf[512];
    int n = router->wifi->poll_rx_cmd(router->wifi->ctx, buf, sizeof(buf) - 1U);
    if (n <= 0) {
        return false;
    }
    buf[n] = '\0';
    return crypto_verify_and_parse_cmd(router->auth, (const char *)buf, (size_t)n, utc_epoch, out_cmd);
}

void link_router_send_emergency_sms(link_router_t *router, const char *msg)
{
    if (router == NULL || router->modem == NULL || router->emergency_phone == NULL || msg == NULL) {
        return;
    }
    router->active_link = LINK_ID_GSM_SMS;
    sim7000g_send_emergency_sms(router->modem, router->emergency_phone, msg);
}

