#ifndef MAIN_BRAIN_H
#define MAIN_BRAIN_H

#include "ahrs_fusion.h"
#include "camera_ctrl.h"
#include "config_manager.h"
#include "crypto_auth.h"
#include "dsp_filters.h"
#include "ekf_nav.h"
#include "env_sensors.h"
#include "failsafe_engine.h"
#include "flight_pid.h"
#include "gps_ublox.h"
#include "hal_interfaces.h"
#include "imu_universal.h"
#include "led_status.h"
#include "link_router.h"
#include "mission_engine.h"
#include "modem_sim7000g.h"
#include "motor_esc.h"
#include "offline_queue.h"
#include "optflow_pmw3901.h"
#include "ota_manager.h"
#include "rc_radio.h"
#include "telemetry_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    hal_sys_t *sys;
    hal_gpio_t *gpio;
    hal_uart_t *uart_sim;
    hal_uart_t *uart_ext;
    hal_i2c_t *i2c;
    hal_spi_t *spi_imu;
    hal_spi_t *spi_optflow;
    hal_pwm_t *pwm_esc;
    hal_pwm_t *pwm_gimbal;
    hal_adc_t *adc;
    hal_storage_t *storage;
    hal_flash_t *flash;
    hal_wireless_t *wireless;

    uint8_t pin_sim_pwrkey;
    uint8_t pin_sim_dtr;
    uint8_t pin_imu_int;
    uint8_t pin_led;
    bool led_active_low;
} pan_brain_bsp_t;

typedef struct {
    pan_brain_bsp_t bsp;
    pan_config_t cfg;

    /* Peripheral Drivers */
    sim7000g_dev_t modem;
    imu_driver_t imu;
    env_sensors_t env;
    gps_ublox_t gps;
    optflow_pmw3901_t optflow;
    motor_esc_t esc;
    rc_radio_t rc;
    camera_ctrl_t cam;
    led_status_t led;

    /* Pure C11 Core Engines */
    dsp_filter_bank_t dsp;
    ahrs_state_t ahrs;
    ekf_nav_t ekf;
    flight_pid_ctx_t pids;
    mission_engine_t mission;
    failsafe_engine_t failsafe;
    offline_queue_t offline_q;
    crypto_auth_ctx_t auth;
    link_router_t link_router;
    ota_manager_t ota;

    /* Live System State */
    pan_flight_mode_t flight_mode;
    float target_alt_m;
    float hold_pn_m;
    float hold_pe_m;
    bool pos_hold_locked;
    uint16_t telem_seq;
    uint64_t last_loop_us;
    uint8_t nav_div;
    bool wifi_ap_active;
    bool cloud_arm_req;
    bool cloud_rth_req;
    uint32_t boot_ms;
    uint32_t comms_tick_count;
} pan_brain_system_t;

int pan_brain_init(pan_brain_system_t *sys_obj, const pan_brain_bsp_t *bsp);
void pan_brain_step_500hz(pan_brain_system_t *sys_obj);
void pan_brain_step_100hz_nav(pan_brain_system_t *sys_obj);
void pan_brain_step_100hz_comms(pan_brain_system_t *sys_obj);
void pan_brain_build_telemetry_pkt(pan_brain_system_t *sys_obj, pan_telem_pkt_t *out_pkt);
void pan_brain_inject_cloud_cmd_json(pan_brain_system_t *sys_obj, const char *json, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_BRAIN_H */

