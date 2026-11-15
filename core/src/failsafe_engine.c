#include "failsafe_engine.h"
#include "flight_pid.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void failsafe_init(failsafe_engine_t *fs, hal_sys_t *sys, pan_system_role_t role)
{
    if (fs == NULL) {
        return;
    }
    memset(fs, 0, sizeof(*fs));
    fs->sys = sys;
    fs->role = role;
    fs->rth_safe_alt_m = 25.0f;
    fs->recommended_mode = (role == ROLE_VEHICLE) ? PAN_MODE_VEHICLE_ACTIVE : PAN_MODE_DISARMED;
}

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
                         mission_engine_t *mission)
{
    if (fs == NULL || fs->sys == NULL) {
        return;
    }

    uint32_t now = fs->sys->millis();
    fs->request_disarm = false;
    fs->request_deep_sleep = false;

    if (fs->role == ROLE_VEHICLE) {
        fs->recommended_mode = PAN_MODE_VEHICLE_ACTIVE;
        if (speed_ms < 0.3f) {
            if (fs->stationary_since_ms == 0U) {
                fs->stationary_since_ms = now;
            }
            /* Vehicle parked for > 300s enters ultra-low-power deep sleep with IMU WOM active */
            if ((now - fs->stationary_since_ms) > 300000U) {
                fs->state = FS_STATE_VEHICLE_STATIONARY_SLEEP;
                fs->recommended_mode = PAN_MODE_VEHICLE_SLEEP;
                fs->request_deep_sleep = true;
            }
        } else {
            fs->stationary_since_ms = 0U;
            if (fs->state == FS_STATE_VEHICLE_STATIONARY_SLEEP) {
                fs->state = FS_STATE_VEHICLE_WOM_ALARM;
                fs->request_emergency_sms = true;
                snprintf(fs->alert_msg, sizeof(fs->alert_msg),
                         "THEFT ALERT: Vehicle moved! Lat:%.5f,Lon:%.5f,Bat:%.1fV",
                         cur_lat, cur_lon, (double)vbat_mv * 0.001);
            }
        }
        return;
    }

    /* ---------------- ROLE_DRONE Failsafe State Machine ---------------- */
    if (!armed) {
        if (fs->state != FS_STATE_CRASH_DISARM) {
            fs->state = FS_STATE_NONE;
            fs->recommended_mode = PAN_MODE_DISARMED;
        }
        fs->rc_lost_since_ms = 0U;
        fs->lte_lost_since_ms = 0U;
        return;
    }

    /* 1. Crash / Flip Protection (>75 deg tilt while armed) */
    if (fabsf(roll_deg) > 75.0f || fabsf(pitch_deg) > 75.0f) {
        fs->state = FS_STATE_CRASH_DISARM;
        fs->recommended_mode = PAN_MODE_EMERGENCY_KILL;
        fs->request_disarm = true;
        fs->request_emergency_sms = true;
        snprintf(fs->alert_msg, sizeof(fs->alert_msg),
                 "ALERT: Drone Crash/Flip! Lat:%.5f,Lon:%.5f,Bat:%.1fV",
                 cur_lat, cur_lon, (double)vbat_mv * 0.001);
        return;
    }

    if (!rc_valid) {
        if (fs->rc_lost_since_ms == 0U) fs->rc_lost_since_ms = now;
    } else {
        fs->rc_lost_since_ms = 0U;
    }

    if (!lte_valid) {
        if (fs->lte_lost_since_ms == 0U) fs->lte_lost_since_ms = now;
    } else {
        fs->lte_lost_since_ms = 0U;
    }

    bool rc_lost_long = (fs->rc_lost_since_ms > 0U) && ((now - fs->rc_lost_since_ms) > 500U);
    bool dual_lost_3s = rc_lost_long && (fs->lte_lost_since_ms > 0U) && ((now - fs->rc_lost_since_ms) > 3000U);
    bool smart_rth = (mission != NULL) && mission->smart_rth_triggered;

    /* Trigger Autonomous Return-to-Home */
    if (rth_switch_active || dual_lost_3s || smart_rth) {
        if (fs->state < FS_STATE_DUAL_LOSS_RTH_CLIMB) {
            fs->state = FS_STATE_DUAL_LOSS_RTH_CLIMB;
            if (mission != NULL) {
                mission_begin_rth_backtrack(mission);
            }
            if (smart_rth && !fs->request_emergency_sms) {
                fs->request_emergency_sms = true;
                snprintf(fs->alert_msg, sizeof(fs->alert_msg),
                         "ALERT: Smart RTH/LowBat at Lat:%.5f,Lon:%.5f,Bat:%.1fV",
                         cur_lat, cur_lon, (double)vbat_mv * 0.001);
            }
        }
    } else if (rc_lost_long && fs->state == FS_STATE_NONE) {
        fs->state = FS_STATE_RC_LOSS_LOITER;
        fs->recommended_mode = PAN_MODE_LOITER;
    } else if (!rc_lost_long && !rth_switch_active && !smart_rth &&
               fs->state == FS_STATE_RC_LOSS_LOITER) {
        fs->state = FS_STATE_NONE;
    }

    switch (fs->state) {
    case FS_STATE_DUAL_LOSS_RTH_CLIMB:
        fs->recommended_mode = PAN_MODE_SMART_RTH;
        fs->target_alt_m = (cur_alt_agl_m > fs->rth_safe_alt_m) ? cur_alt_agl_m : fs->rth_safe_alt_m;
        if (cur_alt_agl_m >= (fs->rth_safe_alt_m - 1.5f)) {
            fs->state = FS_STATE_DUAL_LOSS_RTH_CRUISE;
        }
        break;

    case FS_STATE_DUAL_LOSS_RTH_CRUISE:
        fs->recommended_mode = PAN_MODE_SMART_RTH;
        fs->target_alt_m = fs->rth_safe_alt_m;
        if (mission != NULL && mission->home_set) {
            double d_home = nav_haversine_distance_m(cur_lat, cur_lon, mission->home.lat, mission->home.lon);
            if (d_home < 2.5) {
                fs->state = FS_STATE_DUAL_LOSS_AUTOLAND;
            }
        } else {
            fs->state = FS_STATE_DUAL_LOSS_AUTOLAND;
        }
        break;

    case FS_STATE_DUAL_LOSS_AUTOLAND:
        fs->recommended_mode = PAN_MODE_AUTOLAND;
        if (tof_valid && tof_range_m < 0.25f) {
            fs->request_disarm = true;
            fs->state = FS_STATE_NONE;
            fs->recommended_mode = PAN_MODE_DISARMED;
        } else if (!tof_valid && cur_alt_agl_m < 0.35f) {
            fs->request_disarm = true;
            fs->state = FS_STATE_NONE;
            fs->recommended_mode = PAN_MODE_DISARMED;
        } else {
            fs->target_alt_m = 0.0f;
        }
        break;

    case FS_STATE_RC_LOSS_LOITER:
        fs->recommended_mode = PAN_MODE_LOITER;
        break;

    default:
        break;
    }
}

