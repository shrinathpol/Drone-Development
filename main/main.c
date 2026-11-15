#include "board_lilygo_sim7000g.h"
#include "main_brain.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pan_main";
static pan_brain_system_t g_brain;

static void task_flight_500hz(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2); /* 500 Hz */
    for (;;) {
        pan_brain_step_500hz(&g_brain);
        vTaskDelayUntil(&last_wake, period > 0 ? period : 1);
    }
}

static void task_nav_100hz(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); /* 100 Hz */
    for (;;) {
        pan_brain_step_100hz_nav(&g_brain);
        vTaskDelayUntil(&last_wake, period);
    }
}

static void task_comms_100hz(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); /* 100 Hz */
    for (;;) {
        pan_brain_step_100hz_comms(&g_brain);
        vTaskDelayUntil(&last_wake, period);
    }
}

void app_main(void)
{
    bsp_esp32_board_early_init();

    char locked_id[32] = {0};
    bsp_get_or_lock_device_id(locked_id, sizeof(locked_id));

    pan_brain_bsp_t bsp = {
        .sys = &bsp_sys,
        .gpio = &bsp_gpio,
        .uart_sim = &bsp_uart_sim,
        .uart_ext = &bsp_uart_ext,
        .i2c = &bsp_i2c0,
        .spi_imu = (PIN_IMU_SPI_CS >= 0) ? &bsp_spi_imu : NULL,
        .spi_optflow = (PIN_OPTFLOW_SPI_CS >= 0) ? &bsp_spi_optflow : NULL,
        .pwm_esc = &bsp_pwm_esc,
        .pwm_gimbal = &bsp_pwm_gimbal,
        .adc = &bsp_adc,
        .storage = &bsp_storage,
        .flash = &bsp_flash,
        .wireless = &bsp_wireless,
        .pin_sim_pwrkey = PIN_SIM_PWRKEY,
        .pin_sim_dtr = PIN_SIM_DTR_SLEEP,
        .pin_imu_int = PIN_IMU_INT_WOM,
        .pin_led = PIN_ONBOARD_LED,
        .led_active_low = PIN_ONBOARD_LED_ACTIVE_LOW
    };

    pan_brain_init(&g_brain, &bsp);
    if (locked_id[0] != '\0') {
        strncpy(g_brain.cfg.device_id, locked_id, sizeof(g_brain.cfg.device_id) - 1);
        g_brain.cfg.device_id[sizeof(g_brain.cfg.device_id) - 1] = '\0';
        g_brain.modem.device_id = g_brain.cfg.device_id;
        g_brain.link_router.device_id = g_brain.cfg.device_id;
    }
    ESP_LOGI(TAG, "PAN-BRAIN active: Device ID='%s' APN='%s' MQTT='%s:%u'",
             g_brain.cfg.device_id, g_brain.cfg.apn, g_brain.cfg.mqtt_host, (unsigned)g_brain.cfg.mqtt_port);
    ESP_LOGI(TAG, "PAN-BRAIN initialized on LilyGO T-SIM7000G (Onboard LED GPIO %d)", PIN_ONBOARD_LED);

    /* Core 1: Hard real-time Flight Control (500Hz) & EKF/Mission/Failsafe (100Hz) */
    xTaskCreatePinnedToCore(task_flight_500hz, "flight_500hz", 6144, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task_nav_100hz, "nav_100hz", 6144, NULL, 7, NULL, 1);

    /* Core 0: SIM7000G Cellular FSM, LED Indicator, Wi-Fi Config, Offline SD Queue & OTA */
    xTaskCreatePinnedToCore(task_comms_100hz, "comms_100hz", 8192, NULL, 5, NULL, 0);
}

