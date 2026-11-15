#ifndef MODEM_SIM7000G_H
#define MODEM_SIM7000G_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODEM_STATE_OFF = 0,
    MODEM_STATE_PWRKEY_PULSE,
    MODEM_STATE_WAIT_BOOT,
    MODEM_STATE_AT_SYNC,
    MODEM_STATE_INIT_CFG,
    MODEM_STATE_ENABLE_GNSS,
    MODEM_STATE_WAIT_NET_REG,
    MODEM_STATE_ACTIVATE_PDP,
    MODEM_STATE_MQTT_CONFIG,
    MODEM_STATE_MQTT_CONNECT,
    MODEM_STATE_MQTT_SUB,
    MODEM_STATE_ONLINE_IDLE,
    MODEM_STATE_MQTT_PUB_WAIT_PROMPT,
    MODEM_STATE_MQTT_PUB_SEND_PAYLOAD,
    MODEM_STATE_SMS_SET_TEXT,
    MODEM_STATE_SMS_WAIT_PROMPT,
    MODEM_STATE_SMS_SEND_BODY,
    MODEM_STATE_ERROR_RECOVERY
} modem_fsm_state_t;

typedef struct {
    bool fix_valid;
    double latitude;
    double longitude;
    float altitude_m;
    float speed_kmh;
    float heading_deg;
    uint8_t satellites;
    uint32_t utc_epoch;
} modem_gnss_t;

typedef void (*mqtt_cmd_callback_t)(const char *topic, const uint8_t *payload, size_t len, void *user_ctx);

typedef struct {
    /* Hardware Abstraction Pointers (Injected from BSP) */
    hal_uart_t *uart;
    hal_gpio_t *gpio;
    hal_sys_t  *sys;
    uint8_t pin_pwrkey;
    uint8_t pin_dtr;

    /* Configuration */
    const char *apn;
    const char *mqtt_host;
    uint16_t mqtt_port;
    const char *mqtt_user;
    const char *mqtt_pass;
    const char *device_id;

    /* Runtime State Machine */
    modem_fsm_state_t state;
    uint8_t sub_step;
    uint32_t state_entry_ms;
    uint32_t last_cmd_tx_ms;
    uint32_t last_gnss_poll_ms;
    uint32_t last_csq_poll_ms;
    uint32_t last_cclk_poll_ms;
    uint8_t retry_count;
    bool cmd_in_flight;
    int8_t rssi_dbm;
    uint8_t csq_raw;
    bool is_net_connected;
    bool is_mqtt_connected;
    uint32_t utc_epoch;

    /* Parsed Internal GNSS Data */
    modem_gnss_t gnss;

    /* TX/RX Buffers */
    char rx_line_buf[512];
    size_t rx_idx;
    uint8_t pub_buf[512];
    size_t pub_len;
    char pub_topic[64];
    bool pub_pending;

    /* Emergency SMS Queue */
    char sms_phone[24];
    char sms_msg[160];
    bool sms_pending;

    /* Cloud Command URC Hook */
    mqtt_cmd_callback_t on_cloud_command;
    void *cmd_user_ctx;
} sim7000g_dev_t;

void sim7000g_init(sim7000g_dev_t *dev);
void sim7000g_step_poll(sim7000g_dev_t *dev);
void sim7000g_Step_poll(sim7000g_dev_t *dev);
int  sim7000g_mqtt_publish(sim7000g_dev_t *dev, const char *topic, const void *data, size_t len);
int  sim7000g_send_emergency_sms(sim7000g_dev_t *dev, const char *phone, const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* MODEM_SIM7000G_H */
