#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "failsafe_engine.h"
#include "flight_pid.h"
#include "hal_interfaces.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_SD_PATH "/sdcard/config.json"

typedef struct {
    pan_system_role_t role;
    char device_id[32];
    char apn[48];
    char mqtt_host[64];
    uint16_t mqtt_port;
    char mqtt_user[32];
    char mqtt_pass[32];
    char hmac_secret[64];
    char emergency_phone[24];
    char wifi_ap_ssid[32];
    char wifi_ap_pass[32];
    float battery_cap_wh;
    float rth_alt_m;
    float max_fence_alt_m;

    /* Tunable Roll/Pitch/Yaw Rate PIDs */
    float roll_kp, roll_ki, roll_kd;
    float pitch_kp, pitch_ki, pitch_kd;
    float yaw_kp, yaw_ki, yaw_kd;
} pan_config_t;

void config_manager_set_defaults(pan_config_t *cfg);
int config_manager_load(pan_config_t *cfg, hal_storage_t *storage);
int config_manager_save(const pan_config_t *cfg, hal_storage_t *storage);
bool config_manager_apply_json(pan_config_t *cfg, const char *json);
void config_manager_apply_to_pids(const pan_config_t *cfg, flight_pid_ctx_t *pids);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MANAGER_H */

