#include "modem_sim7000g.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void uart_send_str(sim7000g_dev_t *dev, const char *cmd)
{
    if (dev != NULL && dev->uart != NULL && dev->uart->write != NULL && cmd != NULL) {
        dev->uart->write(dev->uart->ctx, (const uint8_t *)cmd, strlen(cmd));
        dev->last_cmd_tx_ms = dev->sys->millis();
        dev->cmd_in_flight = true;
    }
}

static const char *modem_state_name(modem_fsm_state_t st)
{
    switch (st) {
    case MODEM_STATE_OFF: return "OFF";
    case MODEM_STATE_PWRKEY_PULSE: return "PWRKEY_PULSE";
    case MODEM_STATE_WAIT_BOOT: return "WAIT_BOOT";
    case MODEM_STATE_AT_SYNC: return "AT_SYNC";
    case MODEM_STATE_INIT_CFG: return "INIT_CFG";
    case MODEM_STATE_ENABLE_GNSS: return "ENABLE_GNSS";
    case MODEM_STATE_WAIT_NET_REG: return "WAIT_NET_REG";
    case MODEM_STATE_ACTIVATE_PDP: return "ACTIVATE_PDP";
    case MODEM_STATE_MQTT_CONFIG: return "MQTT_CONFIG";
    case MODEM_STATE_MQTT_CONNECT: return "MQTT_CONNECT";
    case MODEM_STATE_MQTT_SUB: return "MQTT_SUB";
    case MODEM_STATE_ONLINE_IDLE: return "ONLINE_IDLE";
    case MODEM_STATE_MQTT_PUB_WAIT_PROMPT: return "MQTT_PUB_PROMPT";
    case MODEM_STATE_MQTT_PUB_SEND_PAYLOAD: return "MQTT_PUB_SEND";
    case MODEM_STATE_SMS_SET_TEXT: return "SMS_SET_TEXT";
    case MODEM_STATE_SMS_WAIT_PROMPT: return "SMS_PROMPT";
    case MODEM_STATE_SMS_SEND_BODY: return "SMS_SEND";
    case MODEM_STATE_ERROR_RECOVERY: return "ERROR_RECOVERY";
    default: return "UNKNOWN";
    }
}

static void transition_state(sim7000g_dev_t *dev, modem_fsm_state_t next, uint8_t sub_step)
{
    if (dev->state != next && dev->sys != NULL && dev->sys->log_msg != NULL) {
        char log_buf[128];
        snprintf(log_buf, sizeof(log_buf), "FSM %s -> %s (sub=%u retries=%u csq=%u)",
                 modem_state_name(dev->state), modem_state_name(next),
                 (unsigned)sub_step, (unsigned)dev->retry_count, (unsigned)dev->csq_raw);
        dev->sys->log_msg(2, "sim7000g", log_buf);
    }
    dev->state = next;
    dev->sub_step = sub_step;
    dev->state_entry_ms = dev->sys->millis();
    dev->cmd_in_flight = false;
}

static uint32_t parse_cclk_to_epoch(const char *cclk_line)
{
    /* Format: +CCLK: "26/11/15,13:20:00+22" */
    const char *q = strchr(cclk_line, '\"');
    if (q == NULL) {
        return 0U;
    }
    int yy = 0, mm = 0, dd = 0, hh = 0, mi = 0, ss = 0;
    if (sscanf(q + 1, "%d/%d/%d,%d:%d:%d", &yy, &mm, &dd, &hh, &mi, &ss) < 6) {
        return 0U;
    }
    int year = 2000 + yy;
    if (mm < 1 || mm > 12 || dd < 1 || dd > 31) {
        return 0U;
    }

    static const int mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint32_t days = 0U;
    for (int y = 1970; y < year; y++) {
        bool leap = ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
        days += leap ? 366U : 365U;
    }
    for (int m = 1; m < mm; m++) {
        days += (uint32_t)mdays[m - 1];
        if (m == 2 && (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0))) {
            days += 1U;
        }
    }
    days += (uint32_t)(dd - 1);
    return days * 86400U + (uint32_t)hh * 3600U + (uint32_t)mi * 60U + (uint32_t)ss;
}

static void parse_cgnsinf_line(sim7000g_dev_t *dev, const char *line)
{
    const char *p = strchr(line, ':');
    if (p == NULL) {
        return;
    }
    p++;
    while (*p == ' ') {
        p++;
    }

    char copy[256];
    snprintf(copy, sizeof(copy), "%s", p);

    char *fields[22];
    size_t count = 0U;
    char *tok = copy;
    fields[count++] = tok;
    while (*tok != '\0' && count < 22U) {
        if (*tok == ',') {
            *tok = '\0';
            fields[count++] = tok + 1;
        }
        tok++;
    }

    if (count < 8U) {
        return;
    }
    int fix = atoi(fields[1]);
    dev->gnss.fix_valid = (fix == 1);
    if (dev->gnss.fix_valid) {
        dev->gnss.latitude = strtod(fields[3], NULL);
        dev->gnss.longitude = strtod(fields[4], NULL);
        dev->gnss.altitude_m = (float)strtod(fields[5], NULL);
        dev->gnss.speed_kmh = (float)strtod(fields[6], NULL);
        dev->gnss.heading_deg = (float)strtod(fields[7], NULL);
        if (count > 14U && fields[14][0] != '\0') {
            dev->gnss.satellites = (uint8_t)atoi(fields[14]);
        } else if (count > 13U && fields[13][0] != '\0') {
            dev->gnss.satellites = (uint8_t)atoi(fields[13]);
        }
        dev->gnss.utc_epoch = dev->utc_epoch;
    }
}

static void handle_smsub_urc(sim7000g_dev_t *dev, const char *line)
{
    /* Format: +SMSUB: "drone/drone_brain_01/cmd","{\"cmd\":\"RTH\"}" */
    const char *q1 = strchr(line, '\"');
    if (q1 == NULL) {
        return;
    }
    const char *q2 = strchr(q1 + 1, '\"');
    if (q2 == NULL) {
        return;
    }
    char topic[64];
    size_t tlen = (size_t)(q2 - (q1 + 1));
    if (tlen >= sizeof(topic)) {
        tlen = sizeof(topic) - 1U;
    }
    memcpy(topic, q1 + 1, tlen);
    topic[tlen] = '\0';

    const char *p_start = strchr(q2 + 1, '\"');
    if (p_start == NULL) {
        return;
    }
    p_start++;
    size_t plen = strlen(p_start);
    if (plen > 0U && p_start[plen - 1U] == '\"') {
        plen--;
    }

    if (dev->on_cloud_command != NULL && plen > 0U) {
        dev->on_cloud_command(topic, (const uint8_t *)p_start, plen, dev->cmd_user_ctx);
    }
}

static void process_rx_line(sim7000g_dev_t *dev, const char *line)
{
    if (line[0] == '\0') {
        return;
    }

    /* Asynchronous URCs handled across all states */
    if (strncmp(line, "+SMSUB:", 7) == 0) {
        handle_smsub_urc(dev, line);
        return;
    }
    if (strncmp(line, "+SMSTATE: 0", 11) == 0) {
        dev->is_mqtt_connected = false;
        if (dev->state >= MODEM_STATE_ONLINE_IDLE) {
            transition_state(dev, MODEM_STATE_MQTT_CONNECT, 0U);
        }
        return;
    }
    if (strncmp(line, "+CGNSINF:", 9) == 0) {
        parse_cgnsinf_line(dev, line);
        return;
    }
    if (strncmp(line, "+CSQ:", 5) == 0) {
        int rssi = 99, ber = 99;
        if (sscanf(line + 5, " %d,%d", &rssi, &ber) >= 1) {
            dev->csq_raw = (uint8_t)rssi;
            dev->rssi_dbm = (rssi >= 0 && rssi <= 31) ? (int8_t)(-113 + rssi * 2) : -127;
        }
        return;
    }
    if (strncmp(line, "+CCLK:", 6) == 0) {
        uint32_t ep = parse_cclk_to_epoch(line);
        if (ep > 1700000000U) {
            dev->utc_epoch = ep;
        }
        return;
    }

    /* State-specific command responses */
    bool is_ok = (strcmp(line, "OK") == 0);
    bool is_err = (strcmp(line, "ERROR") == 0 || strncmp(line, "+CME ERROR:", 11) == 0 ||
                   strncmp(line, "+CMS ERROR:", 11) == 0);

    switch (dev->state) {
    case MODEM_STATE_AT_SYNC:
        if (is_ok) {
            dev->retry_count = 0U;
            transition_state(dev, MODEM_STATE_INIT_CFG, 0U);
        }
        break;

    case MODEM_STATE_INIT_CFG:
        if (dev->sub_step == 0U && is_ok) {
            dev->sub_step = 1U;
            dev->cmd_in_flight = false;
        } else if (dev->sub_step == 1U && is_ok) {
            dev->sub_step = 2U;
            dev->cmd_in_flight = false;
        } else if (dev->sub_step == 2U && strstr(line, "+CPIN: READY") != NULL) {
            dev->sub_step = 3U;
        } else if (dev->sub_step == 3U && is_ok) {
            dev->sub_step = 4U;
            dev->cmd_in_flight = false;
        } else if (dev->sub_step == 4U && is_ok) {
            dev->sub_step = 5U;
            dev->cmd_in_flight = false;
        } else if (dev->sub_step == 5U && is_ok) {
            dev->retry_count = 0U;
            transition_state(dev, MODEM_STATE_ENABLE_GNSS, 0U);
        } else if (is_err) {
            if (dev->sub_step == 2U) {
                dev->retry_count++;
                if (dev->retry_count >= 2U) {
                    if (dev->sys != NULL && dev->sys->log_msg != NULL) {
                        dev->sys->log_msg(1, "sim7000g", "SIM card not ready (AT+CPIN? error) - advancing to GNSS enable");
                    }
                    dev->retry_count = 0U;
                    dev->sub_step = 4U;
                }
            }
            dev->cmd_in_flight = false;
        }
        break;

    case MODEM_STATE_ENABLE_GNSS:
        if (is_ok || is_err) {
            dev->sub_step++;
            dev->cmd_in_flight = false;
            if (dev->sub_step >= 3U) {
                transition_state(dev, MODEM_STATE_WAIT_NET_REG, 0U);
            }
        }
        break;

    case MODEM_STATE_WAIT_NET_REG:
        if (dev->sub_step < 2U && is_ok) {
            dev->sub_step++;
            dev->cmd_in_flight = false;
        } else if (strncmp(line, "+CGREG:", 7) == 0 || strncmp(line, "+CEREG:", 7) == 0) {
            int n = 0, stat = 0;
            if (sscanf(line + 7, " %d,%d", &n, &stat) == 2 && (stat == 1 || stat == 5)) {
                dev->is_net_connected = true;
                transition_state(dev, MODEM_STATE_ACTIVATE_PDP, 0U);
            }
        } else if (is_ok && dev->sub_step >= 2U) {
            dev->cmd_in_flight = false;
        }
        break;

    case MODEM_STATE_ACTIVATE_PDP:
        if (dev->sub_step < 2U && (is_ok || is_err)) {
            dev->sub_step++;
            dev->cmd_in_flight = false;
        } else if (dev->sub_step == 2U && (is_ok || strstr(line, "+APP PDP: 0,ACTIVE") != NULL)) {
            dev->sub_step = 3U;
            dev->cmd_in_flight = false;
        } else if (strncmp(line, "+CNACT:", 7) == 0) {
            if (strstr(line, ",1,") != NULL) {
                transition_state(dev, MODEM_STATE_MQTT_CONFIG, 0U);
            }
        } else if (dev->sub_step == 3U && is_ok) {
            dev->cmd_in_flight = false;
        } else if (is_err) {
            transition_state(dev, MODEM_STATE_WAIT_NET_REG, 2U);
        }
        break;

    case MODEM_STATE_MQTT_CONFIG:
        if (is_ok) {
            dev->sub_step++;
            dev->cmd_in_flight = false;
            if (dev->sub_step >= 6U) {
                transition_state(dev, MODEM_STATE_MQTT_CONNECT, 0U);
            }
        } else if (is_err) {
            dev->sub_step++;
            dev->cmd_in_flight = false;
        }
        break;

    case MODEM_STATE_MQTT_CONNECT:
        if (is_ok) {
            dev->is_mqtt_connected = true;
            transition_state(dev, MODEM_STATE_MQTT_SUB, 0U);
        } else if (is_err) {
            dev->is_mqtt_connected = false;
            dev->retry_count++;
            dev->cmd_in_flight = false;
            if (dev->retry_count > 5U) {
                transition_state(dev, MODEM_STATE_ACTIVATE_PDP, 2U);
            }
        }
        break;

    case MODEM_STATE_MQTT_SUB:
        if (is_ok || is_err) {
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_ONLINE_IDLE:
        if (is_ok || is_err) {
            dev->cmd_in_flight = false;
        }
        break;

    case MODEM_STATE_MQTT_PUB_SEND_PAYLOAD:
        if (is_ok) {
            dev->pub_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        } else if (is_err) {
            dev->pub_pending = false;
            dev->is_mqtt_connected = false;
            transition_state(dev, MODEM_STATE_MQTT_CONNECT, 0U);
        }
        break;

    case MODEM_STATE_SMS_SET_TEXT:
        if (is_ok) {
            transition_state(dev, MODEM_STATE_SMS_WAIT_PROMPT, 0U);
        } else if (is_err) {
            dev->sms_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_SMS_SEND_BODY:
        if (is_ok || strncmp(line, "+CMGS:", 6) == 0) {
            dev->sms_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        } else if (is_err) {
            dev->sms_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    default:
        break;
    }
}

static void drain_uart_rx(sim7000g_dev_t *dev)
{
    uint8_t chunk[128];
    int got = dev->uart->read(dev->uart->ctx, chunk, sizeof(chunk));
    while (got > 0) {
        for (int i = 0; i < got; i++) {
            char c = (char)chunk[i];

            /* Check prompt '>' immediately during AT+SMPUB or AT+CMGS */
            if (c == '>' && (dev->state == MODEM_STATE_MQTT_PUB_WAIT_PROMPT ||
                             dev->state == MODEM_STATE_SMS_WAIT_PROMPT)) {
                if (dev->state == MODEM_STATE_MQTT_PUB_WAIT_PROMPT) {
                    dev->uart->write(dev->uart->ctx, dev->pub_buf, dev->pub_len);
                    transition_state(dev, MODEM_STATE_MQTT_PUB_SEND_PAYLOAD, 0U);
                } else {
                    dev->uart->write(dev->uart->ctx, (const uint8_t *)dev->sms_msg, strlen(dev->sms_msg));
                    uint8_t ctrl_z = 0x1AU;
                    dev->uart->write(dev->uart->ctx, &ctrl_z, 1U);
                    transition_state(dev, MODEM_STATE_SMS_SEND_BODY, 0U);
                }
                dev->rx_idx = 0U;
                continue;
            }

            if (c == '\r' || c == '\n') {
                if (dev->rx_idx > 0U) {
                    dev->rx_line_buf[dev->rx_idx] = '\0';
                    process_rx_line(dev, dev->rx_line_buf);
                    dev->rx_idx = 0U;
                }
            } else if (dev->rx_idx < sizeof(dev->rx_line_buf) - 1U) {
                dev->rx_line_buf[dev->rx_idx++] = c;
            } else {
                dev->rx_idx = 0U;
            }
        }
        got = dev->uart->read(dev->uart->ctx, chunk, sizeof(chunk));
    }
}

void sim7000g_init(sim7000g_dev_t *dev)
{
    if (dev == NULL || dev->uart == NULL || dev->gpio == NULL || dev->sys == NULL) {
        return;
    }

    dev->uart->init(dev->uart->ctx, 115200U);
    dev->gpio->mode_output(dev->pin_pwrkey);
    dev->gpio->mode_output(dev->pin_dtr);
    dev->gpio->write(dev->pin_dtr, false); /* Keep UART awake */
    dev->gpio->write(dev->pin_pwrkey, true);

    dev->state = MODEM_STATE_PWRKEY_PULSE;
    dev->sub_step = 0U;
    dev->state_entry_ms = dev->sys->millis();
    dev->last_cmd_tx_ms = dev->state_entry_ms;
    dev->last_gnss_poll_ms = 0U;
    dev->last_csq_poll_ms = 0U;
    dev->last_cclk_poll_ms = 0U;
    dev->retry_count = 0U;
    dev->cmd_in_flight = false;
    dev->is_net_connected = false;
    dev->is_mqtt_connected = false;
    dev->rx_idx = 0U;
    dev->pub_pending = false;
    dev->sms_pending = false;
    dev->rssi_dbm = -127;
}

void sim7000g_step_poll(sim7000g_dev_t *dev)
{
    if (dev == NULL || dev->uart == NULL || dev->sys == NULL) {
        return;
    }

    uint32_t now = dev->sys->millis();
    drain_uart_rx(dev);

    switch (dev->state) {
    case MODEM_STATE_OFF:
        break;

    case MODEM_STATE_PWRKEY_PULSE:
        if ((now - dev->state_entry_ms) >= 1100U) {
            dev->gpio->write(dev->pin_pwrkey, false);
            transition_state(dev, MODEM_STATE_WAIT_BOOT, 0U);
        }
        break;

    case MODEM_STATE_WAIT_BOOT:
        if ((now - dev->state_entry_ms) >= 3000U) {
            dev->retry_count = 0U;
            transition_state(dev, MODEM_STATE_AT_SYNC, 0U);
        }
        break;

    case MODEM_STATE_AT_SYNC:
        if (!dev->cmd_in_flight || (now - dev->last_cmd_tx_ms) >= 500U) {
            if (dev->retry_count >= 10U) {
                transition_state(dev, MODEM_STATE_ERROR_RECOVERY, 0U);
            } else {
                dev->retry_count++;
                uart_send_str(dev, "AT\r\n");
            }
        }
        break;

    case MODEM_STATE_INIT_CFG:
        if (!dev->cmd_in_flight) {
            switch (dev->sub_step) {
            case 0U: uart_send_str(dev, "ATE0\r\n"); break;
            case 1U: uart_send_str(dev, "AT+CMEE=2\r\n"); break;
            case 2U: uart_send_str(dev, "AT+CPIN?\r\n"); break;
            case 4U: uart_send_str(dev, "AT+CSCLK=0\r\n"); break;
            case 5U: uart_send_str(dev, "AT+CLTS=1\r\n"); break;
            default: dev->sub_step = 0U; break;
            }
        } else if ((now - dev->last_cmd_tx_ms) > 2000U) {
            dev->cmd_in_flight = false;
            if (++dev->retry_count > 6U) {
                transition_state(dev, MODEM_STATE_ERROR_RECOVERY, 0U);
            }
        }
        break;

    case MODEM_STATE_ENABLE_GNSS:
        if (!dev->cmd_in_flight) {
            switch (dev->sub_step) {
            case 0U: uart_send_str(dev, "AT+CGNSPWR=1\r\n"); break;
            case 1U: uart_send_str(dev, "AT+CGNSMOD=1,1,0,0\r\n"); break;
            default: uart_send_str(dev, "AT+CGNSURC=0\r\n"); break;
            }
        } else if ((now - dev->last_cmd_tx_ms) > 2000U) {
            dev->cmd_in_flight = false;
            dev->sub_step++;
            if (dev->sub_step >= 3U) {
                transition_state(dev, MODEM_STATE_WAIT_NET_REG, 0U);
            }
        }
        break;

    case MODEM_STATE_WAIT_NET_REG:
        if (!dev->cmd_in_flight) {
            if (dev->sub_step == 0U) {
                uart_send_str(dev, "AT+CNMP=2\r\n");
            } else if (dev->sub_step == 1U) {
                uart_send_str(dev, "AT+CMNB=3\r\n");
            } else if ((now - dev->last_cmd_tx_ms) >= 1500U || dev->last_cmd_tx_ms == dev->state_entry_ms) {
                uart_send_str(dev, "AT+CGREG?\r\n");
            }
        } else if ((now - dev->last_cmd_tx_ms) > 2500U) {
            dev->cmd_in_flight = false;
        }
        if ((now - dev->state_entry_ms) > 60000U) {
            transition_state(dev, MODEM_STATE_ERROR_RECOVERY, 0U);
        }
        break;

    case MODEM_STATE_ACTIVATE_PDP:
        if (!dev->cmd_in_flight) {
            char cmd[128];
            const char *apn = (dev->apn != NULL) ? dev->apn : "iot.com";
            switch (dev->sub_step) {
            case 0U:
                snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", apn);
                uart_send_str(dev, cmd);
                break;
            case 1U:
                snprintf(cmd, sizeof(cmd), "AT+CNCFG=0,1,\"%s\"\r\n", apn);
                uart_send_str(dev, cmd);
                break;
            case 2U:
                uart_send_str(dev, "AT+CNACT=0,1\r\n");
                break;
            default:
                uart_send_str(dev, "AT+CNACT?\r\n");
                break;
            }
        } else if ((now - dev->last_cmd_tx_ms) > 8000U) {
            dev->cmd_in_flight = false;
            transition_state(dev, MODEM_STATE_ERROR_RECOVERY, 0U);
        }
        break;

    case MODEM_STATE_MQTT_CONFIG:
        if (!dev->cmd_in_flight) {
            char cmd[192];
            const char *id = (dev->device_id != NULL) ? dev->device_id : "drone_brain_01";
            const char *host = (dev->mqtt_host != NULL) ? dev->mqtt_host : "broker.hivemq.com";
            uint16_t port = (dev->mqtt_port != 0U) ? dev->mqtt_port : 1883U;
            switch (dev->sub_step) {
            case 0U:
                snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"CLIENTID\",\"%s\"\r\n", id);
                uart_send_str(dev, cmd);
                break;
            case 1U:
                snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"URL\",\"%s\",%u\r\n", host, (unsigned)port);
                uart_send_str(dev, cmd);
                break;
            case 2U:
                uart_send_str(dev, "AT+SMCONF=\"KEEPTIME\",60\r\n");
                break;
            case 3U:
                uart_send_str(dev, "AT+SMCONF=\"CLEANSS\",1\r\n");
                break;
            case 4U:
                if (dev->mqtt_user != NULL && dev->mqtt_user[0] != '\0') {
                    snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"USERNAME\",\"%s\"\r\n", dev->mqtt_user);
                    uart_send_str(dev, cmd);
                } else {
                    dev->sub_step++;
                }
                break;
            default:
                if (dev->mqtt_pass != NULL && dev->mqtt_pass[0] != '\0') {
                    snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PASSWORD\",\"%s\"\r\n", dev->mqtt_pass);
                    uart_send_str(dev, cmd);
                } else {
                    transition_state(dev, MODEM_STATE_MQTT_CONNECT, 0U);
                }
                break;
            }
        } else if ((now - dev->last_cmd_tx_ms) > 3000U) {
            dev->cmd_in_flight = false;
            dev->sub_step++;
        }
        break;

    case MODEM_STATE_MQTT_CONNECT:
        if (!dev->cmd_in_flight) {
            uart_send_str(dev, "AT+SMCONN\r\n");
        } else if ((now - dev->last_cmd_tx_ms) > 15000U) {
            dev->cmd_in_flight = false;
            dev->is_mqtt_connected = false;
            transition_state(dev, MODEM_STATE_ERROR_RECOVERY, 0U);
        }
        break;

    case MODEM_STATE_MQTT_SUB:
        if (!dev->cmd_in_flight) {
            char cmd[128];
            const char *id = (dev->device_id != NULL) ? dev->device_id : "drone_brain_01";
            snprintf(cmd, sizeof(cmd), "AT+SMSUB=\"drone/%s/cmd\",1\r\n", id);
            uart_send_str(dev, cmd);
        } else if ((now - dev->last_cmd_tx_ms) > 4000U) {
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_ONLINE_IDLE:
        if (dev->sms_pending && !dev->cmd_in_flight) {
            transition_state(dev, MODEM_STATE_SMS_SET_TEXT, 0U);
            uart_send_str(dev, "AT+CMGF=1\r\n");
            break;
        }
        if (dev->pub_pending && dev->is_mqtt_connected && !dev->cmd_in_flight) {
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "AT+SMPUB=\"%s\",%u,0,0\r\n",
                     dev->pub_topic, (unsigned)dev->pub_len);
            transition_state(dev, MODEM_STATE_MQTT_PUB_WAIT_PROMPT, 0U);
            uart_send_str(dev, cmd);
            break;
        }
        if (!dev->cmd_in_flight) {
            if ((now - dev->last_gnss_poll_ms) >= 1000U) {
                dev->last_gnss_poll_ms = now;
                uart_send_str(dev, "AT+CGNSINF\r\n");
            } else if ((now - dev->last_csq_poll_ms) >= 10000U) {
                dev->last_csq_poll_ms = now;
                uart_send_str(dev, "AT+CSQ\r\n");
            } else if ((now - dev->last_cclk_poll_ms) >= 30000U) {
                dev->last_cclk_poll_ms = now;
                uart_send_str(dev, "AT+CCLK?\r\n");
            }
        } else if ((now - dev->last_cmd_tx_ms) > 2000U) {
            dev->cmd_in_flight = false;
        }
        break;

    case MODEM_STATE_MQTT_PUB_WAIT_PROMPT:
    case MODEM_STATE_MQTT_PUB_SEND_PAYLOAD:
        if ((now - dev->state_entry_ms) > 5000U) {
            dev->pub_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_SMS_SET_TEXT:
        if ((now - dev->state_entry_ms) > 3000U) {
            dev->sms_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_SMS_WAIT_PROMPT:
        if (!dev->cmd_in_flight) {
            char cmd[48];
            snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r\n", dev->sms_phone);
            uart_send_str(dev, cmd);
        } else if ((now - dev->state_entry_ms) > 5000U) {
            dev->sms_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_SMS_SEND_BODY:
        if ((now - dev->state_entry_ms) > 10000U) {
            dev->sms_pending = false;
            transition_state(dev, MODEM_STATE_ONLINE_IDLE, 0U);
        }
        break;

    case MODEM_STATE_ERROR_RECOVERY:
        dev->is_net_connected = false;
        dev->is_mqtt_connected = false;
        dev->retry_count = 0U;
        if ((now - dev->state_entry_ms) > 2000U) {
            dev->uart->flush_rx(dev->uart->ctx);
            dev->gpio->write(dev->pin_pwrkey, true);
            transition_state(dev, MODEM_STATE_PWRKEY_PULSE, 0U);
        }
        break;
    }
}

void sim7000g_Step_poll(sim7000g_dev_t *dev)
{
    sim7000g_step_poll(dev);
}

int sim7000g_mqtt_publish(sim7000g_dev_t *dev, const char *topic, const void *data, size_t len)
{
    if (dev == NULL || topic == NULL || data == NULL || len == 0U || len > sizeof(dev->pub_buf)) {
        return HAL_ERR_INVALID_ARG;
    }
    if (!dev->is_mqtt_connected || dev->pub_pending) {
        return HAL_ERR_NOT_READY;
    }
    snprintf(dev->pub_topic, sizeof(dev->pub_topic), "%s", topic);
    memcpy(dev->pub_buf, data, len);
    dev->pub_len = len;
    dev->pub_pending = true;
    return HAL_OK;
}

int sim7000g_send_emergency_sms(sim7000g_dev_t *dev, const char *phone, const char *msg)
{
    if (dev == NULL || phone == NULL || msg == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    snprintf(dev->sms_phone, sizeof(dev->sms_phone), "%s", phone);
    snprintf(dev->sms_msg, sizeof(dev->sms_msg), "%s", msg);
    dev->sms_pending = true;
    return HAL_OK;
}
