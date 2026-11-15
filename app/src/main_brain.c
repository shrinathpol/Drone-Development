#include "main_brain.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void brain_log(const pan_brain_system_t *s, uint8_t level, const char *fmt, ...)
{
    if (s == NULL || s->bsp.sys == NULL || s->bsp.sys->log_msg == NULL || fmt == NULL) {
        return;
    }
    char msg[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    s->bsp.sys->log_msg(level, "pan_brain", msg);
}

static void apply_verified_cloud_cmd(pan_brain_system_t *s, const pan_cloud_cmd_t *cmd)
{
    switch (cmd->type) {
    case CMD_ARM:
        s->cloud_arm_req = true;
        s->cloud_rth_req = false;
        break;
    case CMD_DISARM:
        s->cloud_arm_req = false;
        motor_esc_disarm(&s->esc);
        s->flight_mode = PAN_MODE_DISARMED;
        break;
    case CMD_RTH:
        s->cloud_rth_req = true;
        s->flight_mode = PAN_MODE_SMART_RTH;
        mission_begin_rth_backtrack(&s->mission);
        break;
    case CMD_GOTO_WAYPOINT:
        s->cloud_rth_req = false;
        mission_add_waypoint(&s->mission, cmd->target_lat, cmd->target_lon, cmd->target_alt_m, true);
        s->flight_mode = PAN_MODE_MISSION;
        break;
    case CMD_SET_MODE:
        if (cmd->mode_arg == 1U) {
            s->cfg.role = ROLE_VEHICLE;
            s->failsafe.role = ROLE_VEHICLE;
            s->flight_mode = PAN_MODE_VEHICLE_ACTIVE;
            motor_esc_disarm(&s->esc);
        } else {
            s->cfg.role = ROLE_DRONE;
            s->failsafe.role = ROLE_DRONE;
            s->flight_mode = s->esc.armed ? PAN_MODE_LOITER : PAN_MODE_DISARMED;
        }
        break;
    case CMD_VEHICLE_SLEEP:
        if (s->cfg.role == ROLE_VEHICLE && !s->esc.armed) {
            imu_universal_enable_wom(&s->imu, 120U);
            led_status_set_pattern(&s->led, LED_PAT_OFF_SLEEP);
            led_status_step_poll(&s->led);
            if (s->bsp.sys->enter_deep_sleep != NULL) {
                s->bsp.sys->enter_deep_sleep(s->bsp.pin_imu_int);
            }
        }
        break;
    case CMD_OTA_MANIFEST:
        ota_manager_begin_manifest(&s->ota, cmd->ota_size_bytes, cmd->ota_sha256_hex);
        break;
    default:
        break;
    }
}

static void on_modem_mqtt_cmd(const char *topic, const uint8_t *payload, size_t len, void *user_ctx)
{
    (void)topic;
    pan_brain_system_t *s = (pan_brain_system_t *)user_ctx;
    if (s == NULL || payload == NULL) {
        return;
    }
    pan_brain_inject_cloud_cmd_json(s, (const char *)payload, len);
}

void pan_brain_inject_cloud_cmd_json(pan_brain_system_t *s, const char *json, size_t len)
{
    if (s == NULL || json == NULL || len == 0U) {
        return;
    }
    pan_cloud_cmd_t cmd;
    uint32_t utc = s->modem.utc_epoch;
    if (crypto_verify_and_parse_cmd(&s->auth, json, len, utc, &cmd)) {
        apply_verified_cloud_cmd(s, &cmd);
    }
}

int pan_brain_init(pan_brain_system_t *s, const pan_brain_bsp_t *bsp)
{
    if (s == NULL || bsp == NULL || bsp->sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    /* Preserve virtual IMU backend if SITL pre-registered it before pan_brain_init */
    const imu_vtable_t *saved_imu_vt = s->imu.vtable;
    void *saved_imu_ctx = s->imu.backend_ctx;

    memset(s, 0, sizeof(*s));
    s->bsp = *bsp;
    s->boot_ms = bsp->sys->millis();
    s->last_loop_us = bsp->sys->micros();
    s->imu.vtable = saved_imu_vt;
    s->imu.backend_ctx = saved_imu_ctx;

    brain_log(s, 2, "[1/12] Mounting storage & initializing LED (GPIO %u active_low=%d)...",
              (unsigned)bsp->pin_led, bsp->led_active_low ? 1 : 0);
    led_status_init(&s->led, bsp->gpio, bsp->sys, bsp->pin_led, bsp->led_active_low);
    led_status_set_pattern(&s->led, LED_PAT_BOOT_INIT);
    led_status_step_poll(&s->led);

    bool sd_ok = false;
    if (bsp->storage != NULL && bsp->storage->mount != NULL) {
        sd_ok = (bsp->storage->mount() == HAL_OK);
    }

    config_manager_load(&s->cfg, bsp->storage);
    brain_log(s, 2, "[1/12] Storage mounted=%d | id=%s role=%u apn=%s mqtt=%s:%u",
              sd_ok ? 1 : 0, s->cfg.device_id, (unsigned)s->cfg.role,
              s->cfg.apn, s->cfg.mqtt_host, (unsigned)s->cfg.mqtt_port);

    brain_log(s, 2, "[2/12] Initializing DSP, 15-State EKF, Mahony AHRS & Dual-Loop PIDs...");
    dsp_filter_bank_init(&s->dsp, 500.0f, 90.0f, 30.0f);
    ahrs_init(&s->ahrs, 1.5f, 0.05f);
    ekf_nav_init(&s->ekf);
    flight_pid_init_defaults(&s->pids);
    config_manager_apply_to_pids(&s->cfg, &s->pids);

    mission_engine_init(&s->mission, bsp->sys);
    s->mission.fence.max_alt_agl_m = s->cfg.max_fence_alt_m;
    failsafe_init(&s->failsafe, bsp->sys, s->cfg.role);
    s->failsafe.rth_safe_alt_m = s->cfg.rth_alt_m;

    crypto_auth_init(&s->auth, s->cfg.hmac_secret);
    offline_queue_init(&s->offline_q, bsp->sys, bsp->storage);
    ota_manager_init(&s->ota, bsp->sys, bsp->storage, bsp->flash);
    brain_log(s, 2, "[3/12] Offline queue ready (cap=%u pkts) & OTA manager ready",
              (unsigned)s->offline_q.capacity);

    /* Drivers */
    brain_log(s, 2, "[4/12] Probing Universal IMU (MPU6050 / ICM-42688-P / BMI270)...");
    int imu_rc = imu_universal_init(&s->imu, bsp->i2c, bsp->spi_imu, bsp->gpio, bsp->sys, bsp->pin_imu_int);
    brain_log(s, 2, "[4/12] IMU init rc=%d healthy=%d chip_type=%d",
              imu_rc, s->imu.healthy ? 1 : 0, (int)s->imu.chip_type);

    brain_log(s, 2, "[5/12] Probing Env Sensors (BMP390, QMC5883L, VL53L1X, INA226, ADC)...");
    env_sensors_init(&s->env, bsp->i2c, bsp->adc, bsp->sys);
    brain_log(s, 2, "[5/12] Env: bmp390=%d mag=%d tof=%d ina226=%d",
              s->env.bmp390_ok ? 1 : 0, s->env.qmc5883l_ok ? 1 : 0,
              s->env.vl53l1x_ok ? 1 : 0, s->env.ina226_ok ? 1 : 0);

    brain_log(s, 2, "[6/12] Probing Optical Flow (PMW3901)...");
    optflow_pmw3901_init(&s->optflow, bsp->spi_optflow, bsp->sys);
    brain_log(s, 2, "[6/12] OptFlow initialized=%d", s->optflow.initialized ? 1 : 0);

    brain_log(s, 2, "[7/12] Initializing Quad-X Motor ESC MCPWM (50Hz)...");
    int esc_rc = motor_esc_init(&s->esc, bsp->pwm_esc, bsp->sys);
    brain_log(s, 2, "[7/12] ESC init rc=%d armed=%d", esc_rc, s->esc.armed ? 1 : 0);

    brain_log(s, 2, "[8/12] Initializing RC Receiver & Camera/Gimbal Controller...");
    rc_radio_init(&s->rc, bsp->uart_ext, bsp->sys, false);
    camera_ctrl_init(&s->cam, bsp->pwm_gimbal, NULL);

    brain_log(s, 2, "[9/12] Initializing SIM7000G Cellular Modem FSM (PWRKEY=%u DTR=%u)...",
              (unsigned)bsp->pin_sim_pwrkey, (unsigned)bsp->pin_sim_dtr);
    s->modem.uart = bsp->uart_sim;
    s->modem.gpio = bsp->gpio;
    s->modem.sys = bsp->sys;
    s->modem.pin_pwrkey = bsp->pin_sim_pwrkey;
    s->modem.pin_dtr = bsp->pin_sim_dtr;
    s->modem.apn = s->cfg.apn;
    s->modem.mqtt_host = s->cfg.mqtt_host;
    s->modem.mqtt_port = s->cfg.mqtt_port;
    s->modem.mqtt_user = s->cfg.mqtt_user;
    s->modem.mqtt_pass = s->cfg.mqtt_pass;
    s->modem.device_id = s->cfg.device_id;
    s->modem.on_cloud_command = on_modem_mqtt_cmd;
    s->modem.cmd_user_ctx = s;
    sim7000g_init(&s->modem);

    brain_log(s, 2, "[10/12] Initializing u-blox / Fallback GNSS & Multi-Link Router...");
    gps_ublox_init(&s->gps, bsp->uart_ext, bsp->sys, &s->modem);
    link_router_init(&s->link_router, bsp->sys, &s->modem, bsp->wireless, &s->rc,
                     &s->offline_q, &s->auth, s->cfg.device_id, s->cfg.emergency_phone);

    s->flight_mode = (s->cfg.role == ROLE_VEHICLE) ? PAN_MODE_VEHICLE_ACTIVE : PAN_MODE_DISARMED;
    brain_log(s, 2, "[11/12] System flight_mode=%u (1=DISARMED, 6=VEHICLE)", (unsigned)s->flight_mode);
    brain_log(s, 2, "[12/12] PAN-BRAIN subsystem init complete");
    return HAL_OK;
}

void pan_brain_step_500hz(pan_brain_system_t *s)
{
    if (s == NULL || s->bsp.sys == NULL) {
        return;
    }

    uint64_t now_us = s->bsp.sys->micros();
    float dt_s = (now_us > s->last_loop_us) ? ((float)(now_us - s->last_loop_us) * 1e-6f) : 0.002f;
    if (dt_s < 0.0005f || dt_s > 0.05f) {
        dt_s = 0.002f;
    }
    s->last_loop_us = now_us;

    imu_sample_t imu_raw;
    if (imu_universal_read(&s->imu, &imu_raw) != HAL_OK || !imu_raw.valid) {
        return;
    }

    dsp_filter_bank_update_rpm_notch(&s->dsp, s->esc.dominant_freq_hz);
    float gyro_filt[3], accel_filt[3];
    dsp_filter_bank_process(&s->dsp, imu_raw.gyro_rad_s, imu_raw.accel_m_s2, gyro_filt, accel_filt);

    ahrs_update(&s->ahrs,
                gyro_filt,
                accel_filt,
                s->env.mag.mag_ut,
                s->env.mag.valid,
                s->env.baro.altitude_m,
                s->env.baro.valid,
                s->env.tof.range_m,
                s->env.tof.valid,
                dt_s);

    ekf_nav_predict(&s->ekf, gyro_filt, accel_filt, &s->ahrs, dt_s);

    if (s->cfg.role == ROLE_VEHICLE) {
        motor_esc_disarm(&s->esc);
        return;
    }

    /* Arm / Disarm Safety Interlock */
    const rc_state_t *rc = &s->rc.state;
    bool want_arm = (rc->signal_valid && rc->arm_switch) || s->cloud_arm_req;
    if (want_arm && !s->esc.armed && s->failsafe.state != FS_STATE_CRASH_DISARM) {
        float arm_thr = rc->signal_valid ? rc->throttle : 0.0f;
        if (motor_esc_arm(&s->esc, arm_thr, s->imu.healthy) == HAL_OK) {
            flight_pid_reset_all(&s->pids);
            s->target_alt_m = s->ekf.est_alt_agl_m;
            if (s->gps.active.fix_valid && !s->mission.home_set) {
                mission_engine_set_home(&s->mission, s->gps.active.latitude, s->gps.active.longitude, s->ekf.est_alt_agl_m);
                ekf_nav_set_origin(&s->ekf, s->gps.active.latitude, s->gps.active.longitude);
            }
            s->flight_mode = PAN_MODE_STABILIZE;
        }
    } else if (!want_arm && s->esc.armed &&
               s->failsafe.state < FS_STATE_DUAL_LOSS_RTH_CLIMB) {
        motor_esc_disarm(&s->esc);
        s->flight_mode = PAN_MODE_DISARMED;
    }

    if (!s->esc.armed) {
        motor_esc_disarm(&s->esc);
        camera_ctrl_update_gimbal(&s->cam, s->ahrs.roll_deg, s->ahrs.pitch_deg, -15.0f);
        return;
    }

    /* Determine Flight Mode & Control Targets */
    float target_roll = rc->roll * s->pids.max_tilt_deg;
    float target_pitch = rc->pitch * s->pids.max_tilt_deg;
    float target_yaw_rate = rc->yaw * s->pids.max_rate_yaw_dps;
    float throttle_out = rc->throttle;

    if (s->failsafe.state == FS_STATE_DUAL_LOSS_RTH_CLIMB ||
        s->failsafe.state == FS_STATE_DUAL_LOSS_RTH_CRUISE ||
        s->failsafe.state == FS_STATE_DUAL_LOSS_AUTOLAND ||
        s->flight_mode == PAN_MODE_SMART_RTH ||
        s->flight_mode == PAN_MODE_MISSION ||
        (rc->signal_valid && rc->mode_switch >= 1U) ||
        s->failsafe.state == FS_STATE_RC_LOSS_LOITER) {

        float nav_pn = s->hold_pn_m;
        float nav_pe = s->hold_pe_m;
        float nav_alt = s->target_alt_m;

        geo_point_t wp;
        if (mission_get_active_target(&s->mission, s->ekf.est_lat, s->ekf.est_lon, s->ekf.est_alt_agl_m, &wp)) {
            if (s->ekf.origin_set) {
                nav_latlon_to_ned_offset(s->ekf.origin_lat, s->ekf.origin_lon, wp.lat, wp.lon, &nav_pn, &nav_pe);
            }
            nav_alt = wp.alt_m;
            if (s->failsafe.state >= FS_STATE_DUAL_LOSS_RTH_CLIMB) {
                nav_alt = s->failsafe.target_alt_m;
            }
        } else if (!s->pos_hold_locked) {
            s->hold_pn_m = s->ekf.x[6];
            s->hold_pe_m = s->ekf.x[7];
            s->pos_hold_locked = true;
            nav_pn = s->hold_pn_m;
            nav_pe = s->hold_pe_m;
        }

        if (s->failsafe.state == FS_STATE_DUAL_LOSS_AUTOLAND) {
            nav_alt = (s->ekf.est_alt_agl_m > 0.5f) ? (s->ekf.est_alt_agl_m - 0.6f * dt_s) : 0.0f;
            target_roll = 0.0f;
            target_pitch = 0.0f;
        } else {
            flight_pid_nav_to_ned(&s->pids, nav_pn, nav_pe,
                                  s->ekf.x[6], s->ekf.x[7], s->ekf.x[3], s->ekf.x[4],
                                  s->ahrs.yaw_deg, dt_s, &target_roll, &target_pitch);
        }

        /* Apply Polygonal Geofence Rubber Bumper braking if breached */
        if (s->mission.geofence_breach) {
            target_roll += s->mission.bumper_ve_ms * 3.0f;
            target_pitch -= s->mission.bumper_vn_ms * 3.0f;
            if (nav_alt > s->mission.clamped_alt_target_m) {
                nav_alt = s->mission.clamped_alt_target_m;
            }
        }

        if (rc->signal_valid && s->failsafe.state == FS_STATE_NONE && s->flight_mode != PAN_MODE_MISSION) {
            /* Pilot climb stick adjustment around hold altitude */
            if (fabsf(rc->throttle - 0.5f) > 0.1f) {
                s->target_alt_m += (rc->throttle - 0.5f) * 3.0f * dt_s;
                if (s->target_alt_m < 0.5f) s->target_alt_m = 0.5f;
            }
            nav_alt = s->target_alt_m;
            s->flight_mode = PAN_MODE_LOITER;
        }

        throttle_out = flight_pid_altitude_step(&s->pids, nav_alt, s->ekf.est_alt_agl_m, s->ahrs.climb_rate_ms, dt_s);
        target_yaw_rate = rc->signal_valid ? (rc->yaw * s->pids.max_rate_yaw_dps) : 0.0f;
    } else {
        s->pos_hold_locked = false;
        s->flight_mode = PAN_MODE_STABILIZE;
    }

    float r_cmd = 0.0f, p_cmd = 0.0f, y_cmd = 0.0f;
    flight_pid_attitude_step(&s->pids, target_roll, target_pitch, target_yaw_rate,
                             s->ahrs.roll_deg, s->ahrs.pitch_deg, gyro_filt, dt_s,
                             &r_cmd, &p_cmd, &y_cmd);

    motor_esc_mix_and_write(&s->esc, throttle_out, r_cmd, p_cmd, y_cmd,
                            (float)s->env.power.vbat_mv, s->dsp.gyro_rms_rad_s);
    camera_ctrl_update_gimbal(&s->cam, s->ahrs.roll_deg, s->ahrs.pitch_deg, -15.0f);
}

void pan_brain_step_100hz_nav(pan_brain_system_t *s)
{
    if (s == NULL) {
        return;
    }

    rc_radio_poll(&s->rc);
    gps_ublox_poll(&s->gps);
    env_sensors_poll_100hz(&s->env);

    s->nav_div++;
    if ((s->nav_div % 5U) == 0U) {
        env_sensors_poll_20hz(&s->env);
    }

    optflow_sample_t flow;
    optflow_pmw3901_poll(&s->optflow,
                         s->env.tof.valid ? s->env.tof.range_m : s->ekf.est_alt_agl_m,
                         s->imu.last_sample.gyro_rad_s[0],
                         s->imu.last_sample.gyro_rad_s[1],
                         &flow);

    ekf_nav_update_measurements(&s->ekf,
                                &s->gps.active,
                                &flow,
                                s->env.baro.altitude_m,
                                s->env.baro.valid,
                                s->env.tof.range_m,
                                s->env.tof.valid,
                                &s->ahrs);

    if (s->gps.active.fix_valid && !s->mission.home_set) {
        mission_engine_set_home(&s->mission, s->gps.active.latitude, s->gps.active.longitude, s->ekf.est_alt_agl_m);
        ekf_nav_set_origin(&s->ekf, s->gps.active.latitude, s->gps.active.longitude);
    }

    if ((s->nav_div % 10U) == 0U) {
        if (s->esc.armed && s->ekf.origin_set) {
            mission_record_breadcrumb(&s->mission, s->ekf.est_lat, s->ekf.est_lon, s->ekf.est_alt_agl_m);
        }
        mission_evaluate_geofence_10hz(&s->mission,
                                       s->ekf.est_lat,
                                       s->ekf.est_lon,
                                       s->ekf.est_alt_agl_m,
                                       s->ekf.x[3],
                                       s->ekf.x[4]);
        mission_update_wind_and_smart_rth(&s->mission,
                                          s->ekf.est_lat,
                                          s->ekf.est_lon,
                                          s->ahrs.roll_deg,
                                          s->ahrs.pitch_deg,
                                          s->ahrs.yaw_deg,
                                          s->ekf.x[3],
                                          s->ekf.x[4],
                                          s->env.power.power_w,
                                          s->cfg.battery_cap_wh,
                                          s->env.power.wh_consumed,
                                          (float)s->env.power.vbat_mv);
    }

    failsafe_step_100hz(&s->failsafe,
                        s->esc.armed,
                        s->rc.state.signal_valid,
                        s->modem.is_mqtt_connected,
                        s->rc.state.rth_switch || s->cloud_rth_req,
                        s->ahrs.roll_deg,
                        s->ahrs.pitch_deg,
                        s->ekf.est_alt_agl_m,
                        s->env.tof.range_m,
                        s->env.tof.valid,
                        s->ekf.est_lat,
                        s->ekf.est_lon,
                        (float)s->env.power.vbat_mv,
                        s->ekf.ground_speed_ms,
                        &s->mission);

    if (s->failsafe.request_disarm) {
        motor_esc_disarm(&s->esc);
        s->cloud_arm_req = false;
        s->flight_mode = s->failsafe.recommended_mode;
    } else if (s->failsafe.state != FS_STATE_NONE) {
        s->flight_mode = s->failsafe.recommended_mode;
    }

    if (s->failsafe.request_emergency_sms) {
        s->failsafe.request_emergency_sms = false;
        link_router_send_emergency_sms(&s->link_router, s->failsafe.alert_msg);
    }

    if (s->failsafe.request_deep_sleep && s->cfg.role == ROLE_VEHICLE) {
        imu_universal_enable_wom(&s->imu, 120U);
        led_status_set_pattern(&s->led, LED_PAT_OFF_SLEEP);
        led_status_step_poll(&s->led);
        if (s->bsp.sys->enter_deep_sleep != NULL) {
            s->bsp.sys->enter_deep_sleep(s->bsp.pin_imu_int);
        }
    }
}

void pan_brain_build_telemetry_pkt(pan_brain_system_t *s, pan_telem_pkt_t *pkt)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->magic = PAN_TELEM_MAGIC;
    pkt->msg_type = PAN_TELEM_MSG_FAST_HUD;
    pkt->seq_id = s->telem_seq++;
    pkt->utc_epoch = (s->modem.utc_epoch > 0U) ? s->modem.utc_epoch : s->gps.active.utc_epoch;
    pkt->lat_e7 = (int32_t)llround(s->ekf.est_lat * 1e7);
    pkt->lon_e7 = (int32_t)llround(s->ekf.est_lon * 1e7);
    pkt->alt_cm = (int16_t)(s->ekf.est_alt_agl_m * 100.0f);
    pkt->roll_cdeg = (int16_t)(s->ahrs.roll_deg * 100.0f);
    pkt->pitch_cdeg = (int16_t)(s->ahrs.pitch_deg * 100.0f);
    pkt->yaw_cdeg = (uint16_t)(s->ahrs.yaw_deg * 100.0f);
    pkt->speed_cms = (uint16_t)(s->ekf.ground_speed_ms * 100.0f);
    pkt->vbat_mv = s->env.power.vbat_mv;
    pkt->curr_ca = (uint16_t)(s->env.power.current_ma / 10);
    pkt->flight_mode = (uint8_t)s->flight_mode;

    uint8_t sats = (s->gps.active.satellites > 15U) ? 15U : s->gps.active.satellites;
    uint8_t csq = (s->modem.csq_raw <= 31U) ? (uint8_t)(s->modem.csq_raw / 2U) : 0U;
    pkt->sats_rssi = (uint8_t)((sats << 4U) | (csq & 0x0FU));
    pkt->active_link = (uint8_t)s->link_router.active_link;

    uint8_t flags = 0U;
    if (s->esc.armed) flags |= PAN_HEALTH_ARMED;
    if (s->gps.active.fix_valid) flags |= PAN_HEALTH_GPS_LOCK;
    if (s->ekf.using_optflow_dead_reckon) flags |= PAN_HEALTH_OPTFLOW_OK;
    if (s->mission.geofence_breach) flags |= PAN_HEALTH_GEOFENCE_HIT;
    if (s->esc.prop_strike_alert) flags |= PAN_HEALTH_PROP_ANOMALY;
    if (s->failsafe.state >= FS_STATE_DUAL_LOSS_RTH_CLIMB) flags |= PAN_HEALTH_FAILSAFE_RTH;
    if (s->cfg.role == ROLE_VEHICLE) flags |= PAN_HEALTH_VEHICLE_MODE;
    if (s->failsafe.state == FS_STATE_VEHICLE_WOM_ALARM) flags |= PAN_HEALTH_WOM_ALERT;
    pkt->health_flags = flags;

    telemetry_finalize_pkt(pkt);
}

void pan_brain_step_100hz_comms(pan_brain_system_t *s)
{
    if (s == NULL || s->bsp.sys == NULL) {
        return;
    }

    sim7000g_step_poll(&s->modem);

    /* Update Onboard LilyGO LED Status Pattern */
    uint32_t up = s->bsp.sys->millis() - s->boot_ms;
    if (up < 2500U) {
        led_status_set_pattern(&s->led, LED_PAT_BOOT_INIT);
    } else if (s->ota.state == OTA_STATE_STAGING_CHUNKS || s->ota.state == OTA_STATE_FLASHING_BANK) {
        led_status_set_pattern(&s->led, LED_PAT_OTA_UPDATING);
    } else if (s->failsafe.state == FS_STATE_CRASH_DISARM || s->esc.prop_strike_alert) {
        led_status_set_pattern(&s->led, LED_PAT_CRITICAL_ERROR);
    } else if (s->failsafe.state >= FS_STATE_DUAL_LOSS_RTH_CLIMB &&
               s->failsafe.state <= FS_STATE_DUAL_LOSS_AUTOLAND) {
        led_status_set_pattern(&s->led, LED_PAT_FAILSAFE_RTH);
    } else if (s->esc.armed) {
        led_status_set_pattern(&s->led, LED_PAT_ARMED_FLIGHT);
    } else if (s->modem.is_mqtt_connected && s->gps.active.fix_valid) {
        led_status_set_pattern(&s->led, LED_PAT_DISARMED_ONLINE);
    } else {
        led_status_set_pattern(&s->led, LED_PAT_SEARCHING_NET_GPS);
    }
    led_status_step_poll(&s->led);

    /* Manage Wi-Fi SoftAP Configuration Server (active only when disarmed) */
    if (s->bsp.wireless != NULL) {
        if (!s->esc.armed && !s->wifi_ap_active && s->bsp.wireless->start_ap_cfg != NULL) {
            if (s->bsp.wireless->start_ap_cfg(s->bsp.wireless->ctx, s->cfg.wifi_ap_ssid, s->cfg.wifi_ap_pass) == HAL_OK) {
                s->wifi_ap_active = true;
                brain_log(s, 2, "Wi-Fi SoftAP Config Server active: SSID=%s", s->cfg.wifi_ap_ssid);
            }
        } else if (s->esc.armed && s->wifi_ap_active && s->bsp.wireless->stop_ap_cfg != NULL) {
            s->bsp.wireless->stop_ap_cfg(s->bsp.wireless->ctx);
            s->wifi_ap_active = false;
            brain_log(s, 2, "Wi-Fi SoftAP Config Server stopped (armed)");
        }

        if (s->wifi_ap_active) {
            pan_cloud_cmd_t wcmd;
            if (link_router_poll_wifi_command(&s->link_router, s->modem.utc_epoch, &wcmd)) {
                apply_verified_cloud_cmd(s, &wcmd);
            }
            if (s->bsp.wireless->has_pending_cfg_json != NULL &&
                s->bsp.wireless->has_pending_cfg_json(s->bsp.wireless->ctx)) {
                char cfg_json[1024];
                if (s->bsp.wireless->take_pending_cfg_json(s->bsp.wireless->ctx, cfg_json, sizeof(cfg_json)) == HAL_OK) {
                    if (config_manager_apply_json(&s->cfg, cfg_json)) {
                        config_manager_apply_to_pids(&s->cfg, &s->pids);
                        config_manager_save(&s->cfg, s->bsp.storage);
                        brain_log(s, 2, "Updated config saved from Wi-Fi portal");
                    }
                }
            }
        }
    }

    /* Package and route binary telemetry */
    pan_telem_pkt_t pkt;
    pan_brain_build_telemetry_pkt(s, &pkt);
    link_router_send_telemetry(&s->link_router, &pkt);

    if (s->wifi_ap_active && s->bsp.wireless != NULL && s->bsp.wireless->update_status_json != NULL) {
        char js[256];
        telemetry_format_json(&pkt, s->cfg.device_id, js, sizeof(js));
        s->bsp.wireless->update_status_json(s->bsp.wireless->ctx, js);
    }

    ota_poll_boot_verification(&s->ota, s->imu.healthy, s->modem.state >= MODEM_STATE_INIT_CFG);

    /* Periodic 1 Hz serial debug summary */
    if ((s->comms_tick_count % 100U) == 0U) {
        brain_log(s, 2,
                  "STATUS | mode=%u armed=%d led_pat=%d led_on=%d link=%u modem_st=%d csq=%u gps_fix=%u sats=%u vbat=%umV r=%.1f p=%.1f y=%.1f alt=%.2fm q_cnt=%u",
                  (unsigned)s->flight_mode,
                  s->esc.armed ? 1 : 0,
                  (int)s->led.pattern,
                  s->led.current_on ? 1 : 0,
                  (unsigned)s->link_router.active_link,
                  (int)s->modem.state,
                  (unsigned)s->modem.csq_raw,
                  s->gps.active.fix_valid ? 1U : 0U,
                  (unsigned)s->gps.active.satellites,
                  (unsigned)s->env.power.vbat_mv,
                  s->ahrs.roll_deg,
                  s->ahrs.pitch_deg,
                  s->ahrs.yaw_deg,
                  s->ekf.est_alt_agl_m,
                  (unsigned)offline_queue_count(&s->offline_q));
    }
    s->comms_tick_count++;
}

