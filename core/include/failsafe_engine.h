#ifndef FAILSAFE_ENGINE_H
#define FAILSAFE_ENGINE_H

#include "hal_interfaces.h"
#include "mission_engine.h"
#include "telemetry_proto.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROLE_DRONE = 0,
    ROLE_VEHICLE = 1
} pan_system_role_t;

typedef enum {
    FS_STATE_NONE = 0,
    FS_STATE_RC_LOSS_LOITER,
    FS_STATE_DUAL_LOSS_RTH_CLIMB,
    FS_STATE_DUAL_LOSS_RTH_CRUISE,
    FS_STATE_DUAL_LOSS_AUTOLAND,
    FS_STATE_CRASH_DISARM,
    FS_STATE_VEHICLE_STATIONARY_SLEEP,
    FS_STATE_VEHICLE_WOM_ALARM
} failsafe_state_t;

typedef struct {
    hal_sys_t *sys;
    pan_system_role_t role;
    failsafe_state_t state;
    pan_flight_mode_t recommended_mode;

    uint32_t rc_lost_since_ms;
    uint32_t lte_lost_since_ms;
    uint32_t stationary_since_ms;

    float rth_safe_alt_m;
    float target_alt_m;
    bool request_disarm;
    bool request_emergency_sms;
    bool request_deep_sleep;
    char alert_msg[140];
} failsafe_engine_t;

void failsafe_init(failsafe_engine_t *fs, hal_sys_t *sys, pan_system_role_t role);

void failsafe_step_100hz(failsafe_engine_t *fs,
                         bool armed,
                         bool rc_valid,
                         bool lte_valid,
                         bool rth_switch_active,
                         float roll_deg,
                         float pitch_deg,
                         float cur_alt_agl_m,
                         float tof_range_m,
                         bool tof_valid,
                         double cur_lat,
                         double cur_lon,
                         float vbat_mv,
                         float speed_ms,
                         mission_engine_t *mission);

#ifdef __cplusplus
}
#endif

#endif /* FAILSAFE_ENGINE_H */

