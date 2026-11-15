#include "board_lilygo_sim7000g.h"

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <stdlib.h>

static const char *TAG = "bsp_sys";

static uint32_t sys_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint64_t sys_micros(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void sys_delay_ms(uint32_t ms)
{
    if (ms == 0) {
        return;
    }
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

static void sys_enter_deep_sleep(uint32_t wake_gpio_mask)
{
    /* GPIO 34 (PIN_IMU_INT_WOM) is an RTC_IO pin (> 31), so support both
     * direct pin number / 0 default (EXT0 on GPIO 34) and bitmask < 32 (EXT1). */
    if (wake_gpio_mask == 0U || wake_gpio_mask == (uint32_t)PIN_IMU_INT_WOM) {
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_IMU_INT_WOM, 1);
    } else {
        esp_sleep_enable_ext1_wakeup((uint64_t)wake_gpio_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    ESP_LOGI(TAG, "Entering deep sleep (wake_arg=0x%08lx)", (unsigned long)wake_gpio_mask);
    esp_deep_sleep_start();
}

static void *sys_alloc_psram(size_t bytes)
{
    if (bytes == 0U) {
        return NULL;
    }
    void *ptr = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr == NULL) {
        ptr = heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT);
    }
    return ptr;
}

static void sys_free_psram(void *ptr)
{
    if (ptr != NULL) {
        heap_caps_free(ptr);
    }
}

static void sys_log_msg(uint8_t level, const char *tag, const char *msg)
{
    const char *t = (tag != NULL) ? tag : "pan";
    const char *m = (msg != NULL) ? msg : "";
    switch (level) {
    case 0:
        ESP_LOGE(t, "%s", m);
        break;
    case 1:
        ESP_LOGW(t, "%s", m);
        break;
    case 2:
        ESP_LOGI(t, "%s", m);
        break;
    default:
        ESP_LOGD(t, "%s", m);
        break;
    }
}

static void sys_reboot(void)
{
    esp_restart();
}

static bool is_reserved_pin(uint8_t pin)
{
    if (pin == PIN_PSRAM_RESERVED_0 || pin == PIN_PSRAM_RESERVED_1) {
        return true;
    }
    if (pin >= 6U && pin <= 11U) {
        return true; /* Internal SPI Flash */
    }
    return false;
}

static bool bsp_is_jtag_pin_guarded(uint8_t pin)
{
    if (pin < 12U || pin > 15U) {
        return false;
    }
    return esp_cpu_dbgr_is_attached() || (BSP_JTAG_FORCE_ENABLE != 0);
}

static int gpio_mode_output(uint8_t pin)
{
    if (is_reserved_pin(pin) || pin >= 34U) {
        /* GPIOs 34-39 on ESP32 are input-only */
        return HAL_ERR_INVALID_ARG;
    }
    if (bsp_is_jtag_pin_guarded(pin)) {
        ESP_LOGW(TAG, "Preserving JTAG MUX on GPIO %u (debugger attached)", (unsigned)pin);
        return HAL_OK;
    }
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return (gpio_config(&cfg) == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

static int gpio_mode_input(uint8_t pin, bool pullup)
{
    if (is_reserved_pin(pin)) {
        return HAL_ERR_INVALID_ARG;
    }
    if (bsp_is_jtag_pin_guarded(pin)) {
        ESP_LOGW(TAG, "Preserving JTAG MUX on GPIO %u (debugger attached)", (unsigned)pin);
        return HAL_OK;
    }
    /* GPIOs 34-39 lack internal pull-up/pull-down resistors on ESP32 */
    bool allow_pullup = pullup && (pin < 34U);
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = allow_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return (gpio_config(&cfg) == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

static void gpio_write_pin(uint8_t pin, bool level)
{
    if (is_reserved_pin(pin) || pin >= 34U || bsp_is_jtag_pin_guarded(pin)) {
        return;
    }
    gpio_set_level((gpio_num_t)pin, level ? 1U : 0U);
}

static bool gpio_read_pin(uint8_t pin)
{
    if (is_reserved_pin(pin)) {
        return false;
    }
    return gpio_get_level((gpio_num_t)pin) != 0;
}

hal_sys_t bsp_sys = {
    .millis = sys_millis,
    .micros = sys_micros,
    .delay_ms = sys_delay_ms,
    .enter_deep_sleep = sys_enter_deep_sleep,
    .alloc_psram = sys_alloc_psram,
    .free_psram = sys_free_psram,
    .log_msg = sys_log_msg,
    .reboot = sys_reboot
};

hal_gpio_t bsp_gpio = {
    .mode_output = gpio_mode_output,
    .mode_input = gpio_mode_input,
    .write = gpio_write_pin,
    .read = gpio_read_pin
};

int bsp_esp32_board_early_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM ready: %u bytes", (unsigned)esp_psram_get_size());
    } else {
        ESP_LOGW(TAG, "PSRAM not detected; falling back to internal SRAM");
    }

    return (ret == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

int bsp_get_or_lock_device_id(char *out_id, size_t max_len)
{
    if (out_id == NULL || max_len < 16U) {
        return HAL_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("pan_device", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        size_t required = max_len;
        err = nvs_get_str(nvs, "device_id", out_id, &required);
        if (err == ESP_OK && out_id[0] != '\0') {
            ESP_LOGI(TAG, "Loaded locked device_id from flash: '%s'", out_id);
            nvs_close(nvs);
            return HAL_OK;
        }
    }

    /* First initialization: derive permanent unique ID from factory eFuse MAC */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out_id, max_len, "drone_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (err == ESP_OK) {
        nvs_set_str(nvs, "device_id", out_id);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "First initialization: LOCKED unique device_id '%s' into NVS flash", out_id);
    } else {
        ESP_LOGW(TAG, "Generated hardware ID '%s' (NVS write err 0x%x)", out_id, (unsigned)err);
    }
    return HAL_OK;
}

int bsp_lock_device_id(const char *new_id)
{
    if (new_id == NULL || new_id[0] == '\0') {
        return HAL_ERR_INVALID_ARG;
    }
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("pan_device", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return HAL_ERR_IO;
    }
    nvs_set_str(nvs, "device_id", new_id);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Custom device ID permanently locked in flash: '%s'", new_id);
    return HAL_OK;
}
