#include "config_manager.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void config_manager_set_defaults(pan_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->role = ROLE_DRONE;
    snprintf(cfg->device_id, sizeof(cfg->device_id), "drone_brain_01");
    snprintf(cfg->apn, sizeof(cfg->apn), "bsnlnet");
    snprintf(cfg->mqtt_host, sizeof(cfg->mqtt_host), "broker.hivemq.com");
    cfg->mqtt_port = 1883U;
    snprintf(cfg->hmac_secret, sizeof(cfg->hmac_secret), "PAN_BRAIN_DEV_KEY_2026");
    snprintf(cfg->emergency_phone, sizeof(cfg->emergency_phone), "+910000000000");
    snprintf(cfg->wifi_ap_ssid, sizeof(cfg->wifi_ap_ssid), "PAN_BRAIN_CFG");
    snprintf(cfg->wifi_ap_pass, sizeof(cfg->wifi_ap_pass), "panbrain123");
    cfg->battery_cap_wh = 24.4f; /* 3S 2200mAh LiPo */
    cfg->rth_alt_m = 25.0f;
    cfg->max_fence_alt_m = 120.0f;

    cfg->roll_kp = 0.0022f; cfg->roll_ki = 0.0015f; cfg->roll_kd = 0.00008f;
    cfg->pitch_kp = 0.0022f; cfg->pitch_ki = 0.0015f; cfg->pitch_kd = 0.00008f;
    cfg->yaw_kp = 0.0040f; cfg->yaw_ki = 0.0020f; cfg->yaw_kd = 0.00002f;
}

static bool extract_str(const char *json, const char *key, char *out, size_t max_len)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') p++;
    if (*p != '\"') return false;
    p++;
    size_t i = 0U;
    while (*p != '\0' && *p != '\"' && (i + 1U) < max_len) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return true;
}

static bool extract_num(const char *json, const char *key, double *out)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) return false;
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') p++;
    if (*p != '-' && !isdigit((unsigned char)*p)) return false;
    *out = strtod(p, NULL);
    return true;
}

bool config_manager_apply_json(pan_config_t *cfg, const char *json)
{
    if (cfg == NULL || json == NULL) {
        return false;
    }

    char tmp[64];
    double v = 0.0;
    if (extract_str(json, "role", tmp, sizeof(tmp))) {
        cfg->role = (strcmp(tmp, "VEHICLE") == 0) ? ROLE_VEHICLE : ROLE_DRONE;
    }
    extract_str(json, "device_id", cfg->device_id, sizeof(cfg->device_id));
    extract_str(json, "apn", cfg->apn, sizeof(cfg->apn));
    extract_str(json, "mqtt_host", cfg->mqtt_host, sizeof(cfg->mqtt_host));
    if (extract_num(json, "mqtt_port", &v) && v > 0.0) cfg->mqtt_port = (uint16_t)v;
    extract_str(json, "mqtt_user", cfg->mqtt_user, sizeof(cfg->mqtt_user));
    extract_str(json, "mqtt_pass", cfg->mqtt_pass, sizeof(cfg->mqtt_pass));
    extract_str(json, "hmac_secret", cfg->hmac_secret, sizeof(cfg->hmac_secret));
    extract_str(json, "emergency_phone", cfg->emergency_phone, sizeof(cfg->emergency_phone));
    if (extract_num(json, "battery_cap_wh", &v) && v > 1.0) cfg->battery_cap_wh = (float)v;
    if (extract_num(json, "rth_alt_m", &v) && v > 2.0) cfg->rth_alt_m = (float)v;
    if (extract_num(json, "max_fence_alt_m", &v) && v > 5.0) cfg->max_fence_alt_m = (float)v;
    if (extract_num(json, "roll_kp", &v)) cfg->roll_kp = (float)v;
    if (extract_num(json, "roll_ki", &v)) cfg->roll_ki = (float)v;
    if (extract_num(json, "roll_kd", &v)) cfg->roll_kd = (float)v;
    if (extract_num(json, "pitch_kp", &v)) cfg->pitch_kp = (float)v;
    if (extract_num(json, "pitch_ki", &v)) cfg->pitch_ki = (float)v;
    if (extract_num(json, "pitch_kd", &v)) cfg->pitch_kd = (float)v;
    if (extract_num(json, "yaw_kp", &v)) cfg->yaw_kp = (float)v;
    if (extract_num(json, "yaw_ki", &v)) cfg->yaw_ki = (float)v;
    if (extract_num(json, "yaw_kd", &v)) cfg->yaw_kd = (float)v;
    return true;
}

static char s_cfg_buf[1024];

int config_manager_load(pan_config_t *cfg, hal_storage_t *storage)
{
    if (cfg == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    config_manager_set_defaults(cfg);
    if (storage == NULL || storage->read_file == NULL) {
        return HAL_ERR_NOT_READY;
    }

    int n = storage->read_file(CONFIG_SD_PATH, s_cfg_buf, sizeof(s_cfg_buf) - 1U, 0U);
    if (n <= 0) {
        config_manager_save(cfg, storage);
        return HAL_ERR_NOT_READY;
    }
    s_cfg_buf[n] = '\0';
    config_manager_apply_json(cfg, s_cfg_buf);
    return HAL_OK;
}

int config_manager_save(const pan_config_t *cfg, hal_storage_t *storage)
{
    if (cfg == NULL || storage == NULL || storage->write_file == NULL) {
        return HAL_ERR_NOT_READY;
    }

    int n = snprintf(
        s_cfg_buf,
        sizeof(s_cfg_buf),
        "{\n"
        "  \"role\": \"%s\",\n"
        "  \"device_id\": \"%s\",\n"
        "  \"apn\": \"%s\",\n"
        "  \"mqtt_host\": \"%s\",\n"
        "  \"mqtt_port\": %u,\n"
        "  \"emergency_phone\": \"%s\",\n"
        "  \"battery_cap_wh\": %.2f,\n"
        "  \"rth_alt_m\": %.1f,\n"
        "  \"max_fence_alt_m\": %.1f,\n"
        "  \"roll_kp\": %.5f, \"roll_ki\": %.5f, \"roll_kd\": %.6f,\n"
        "  \"pitch_kp\": %.5f, \"pitch_ki\": %.5f, \"pitch_kd\": %.6f,\n"
        "  \"yaw_kp\": %.5f, \"yaw_ki\": %.5f, \"yaw_kd\": %.6f\n"
        "}\n",
        (cfg->role == ROLE_VEHICLE) ? "VEHICLE" : "DRONE",
        cfg->device_id,
        cfg->apn,
        cfg->mqtt_host,
        (unsigned)cfg->mqtt_port,
        cfg->emergency_phone,
        (double)cfg->battery_cap_wh,
        (double)cfg->rth_alt_m,
        (double)cfg->max_fence_alt_m,
        (double)cfg->roll_kp, (double)cfg->roll_ki, (double)cfg->roll_kd,
        (double)cfg->pitch_kp, (double)cfg->pitch_ki, (double)cfg->pitch_kd,
        (double)cfg->yaw_kp, (double)cfg->yaw_ki, (double)cfg->yaw_kd);
    if (n <= 0) {
        return HAL_ERR_IO;
    }
    return storage->write_file(CONFIG_SD_PATH, s_cfg_buf, (size_t)n);
}

void config_manager_apply_to_pids(const pan_config_t *cfg, flight_pid_ctx_t *pids)
{
    if (cfg == NULL || pids == NULL) {
        return;
    }
    pids->rate_roll.kp = cfg->roll_kp;
    pids->rate_roll.ki = cfg->roll_ki;
    pids->rate_roll.kd = cfg->roll_kd;
    pids->rate_pitch.kp = cfg->pitch_kp;
    pids->rate_pitch.ki = cfg->pitch_ki;
    pids->rate_pitch.kd = cfg->pitch_kd;
    pids->rate_yaw.kp = cfg->yaw_kp;
    pids->rate_yaw.ki = cfg->yaw_ki;
    pids->rate_yaw.kd = cfg->yaw_kd;
}

