#include "board_lilygo_sim7000g.h"

#include "driver/sdspi_host.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include <stdio.h>
#include <sys/stat.h>

static const char *TAG = "bsp_sdcard_adc_ota";

/* ---------------- SD Card Storage (FATFS over HSPI) ---------------- */

static sdmmc_card_t *s_sd_card = NULL;
static bool s_sd_mounted = false;

static int bsp_storage_mount(void)
{
    if (s_sd_mounted) {
        return HAL_OK;
    }

    if (bsp_spi_ensure_bus_init() != HAL_OK) {
        return HAL_ERR_BUS;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 10000; /* 10 MHz reliable across drone PCB traces */

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = (gpio_num_t)PIN_SD_CS;
    slot_cfg.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = true,
        .max_files = 6,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_cfg, &mount_cfg, &s_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card mount failed (%s) - running without SD storage", esp_err_to_name(ret));
        return HAL_ERR_IO;
    }

    s_sd_mounted = true;
    ESP_LOGI(TAG, "SD card mounted at /sdcard");
    return HAL_OK;
}

static int bsp_storage_append_file(const char *path, const void *data, size_t len)
{
    if (!s_sd_mounted || path == NULL || data == NULL || len == 0U) {
        return HAL_ERR_NOT_READY;
    }
    FILE *f = fopen(path, "ab");
    if (f == NULL) {
        return HAL_ERR_IO;
    }
    size_t written = fwrite(data, 1U, len, f);
    fclose(f);
    return (written == len) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_storage_write_file(const char *path, const void *data, size_t len)
{
    if (!s_sd_mounted || path == NULL || data == NULL) {
        return HAL_ERR_NOT_READY;
    }
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        return HAL_ERR_IO;
    }
    size_t written = (len > 0U) ? fwrite(data, 1U, len, f) : 0U;
    fclose(f);
    return (written == len) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_storage_read_file(const char *path, void *buf, size_t max_len, size_t offset)
{
    if (!s_sd_mounted || path == NULL || buf == NULL || max_len == 0U) {
        return HAL_ERR_NOT_READY;
    }
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return HAL_ERR_IO;
    }
    if (offset > 0U && fseek(f, (long)offset, SEEK_SET) != 0) {
        fclose(f);
        return HAL_ERR_IO;
    }
    size_t n = fread(buf, 1U, max_len, f);
    fclose(f);
    return (int)n;
}

static int bsp_storage_delete_file(const char *path)
{
    if (!s_sd_mounted || path == NULL) {
        return HAL_ERR_NOT_READY;
    }
    return (remove(path) == 0) ? HAL_OK : HAL_ERR_IO;
}

static int64_t bsp_storage_file_size(const char *path)
{
    if (!s_sd_mounted || path == NULL) {
        return -1;
    }
    struct stat st;
    if (stat(path, &st) != 0) {
        return -1;
    }
    return (int64_t)st.st_size;
}

hal_storage_t bsp_storage = {
    .mount = bsp_storage_mount,
    .append_file = bsp_storage_append_file,
    .read_file = bsp_storage_read_file,
    .delete_file = bsp_storage_delete_file,
    .write_file = bsp_storage_write_file,
    .file_size = bsp_storage_file_size
};

/* ---------------- Battery & Solar ADC (ADC1 Oneshot + Line Calibration) ---------------- */

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t s_adc1_cali = NULL;
static bool s_adc_ch_init[2] = { false, false };

static adc_channel_t map_adc_channel(uint8_t ch)
{
    if (ch == BSP_ADC_CH_BAT) {
        return ADC_CHANNEL_7; /* GPIO 35 on ESP32 */
    }
    return ADC_CHANNEL_0;     /* GPIO 36 on ESP32 */
}

static int bsp_adc_init(uint8_t channel)
{
    if (channel > BSP_ADC_CH_SOLAR) {
        return HAL_ERR_INVALID_ARG;
    }

    if (s_adc1_handle == NULL) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {
            .unit_id = ADC_UNIT_1,
            .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE
        };
        if (adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle) != ESP_OK) {
            return HAL_ERR_BUS;
        }

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .default_vref = 1100
        };
        adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc1_cali);
#endif
    }

    if (!s_adc_ch_init[channel]) {
        adc_oneshot_chan_cfg_t ch_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if (adc_oneshot_config_channel(s_adc1_handle, map_adc_channel(channel), &ch_cfg) != ESP_OK) {
            return HAL_ERR_BUS;
        }
        s_adc_ch_init[channel] = true;
    }

    return HAL_OK;
}

static uint32_t bsp_adc_read_mv(uint8_t channel)
{
    if (channel > BSP_ADC_CH_SOLAR || s_adc1_handle == NULL || !s_adc_ch_init[channel]) {
        return 0U;
    }

    int raw = 0;
    if (adc_oneshot_read(s_adc1_handle, map_adc_channel(channel), &raw) != ESP_OK) {
        return 0U;
    }

    int mv = 0;
    if (s_adc1_cali != NULL && adc_cali_raw_to_voltage(s_adc1_cali, raw, &mv) == ESP_OK) {
        /* LilyGO T-SIM7000G BAT_ADC uses a 1:2 divider (100k/100k) */
        return (channel == BSP_ADC_CH_BAT) ? (uint32_t)(mv * 2) : (uint32_t)mv;
    }

    /* Fallback linear approximation for 12-bit 3.3V full scale */
    uint32_t approx_mv = (uint32_t)((raw * 3300) / 4095);
    return (channel == BSP_ADC_CH_BAT) ? (approx_mv * 2U) : approx_mv;
}

hal_adc_t bsp_adc = {
    .init = bsp_adc_init,
    .read_mv = bsp_adc_read_mv
};

/* ---------------- Dual-Bank Atomic OTA Flash Interface ---------------- */

static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_ota_part = NULL;
static bool s_ota_active = false;

static int bsp_ota_begin(size_t image_size)
{
    if (s_ota_active) {
        esp_ota_abort(s_ota_handle);
        s_ota_active = false;
    }
    s_ota_part = esp_ota_get_next_update_partition(NULL);
    if (s_ota_part == NULL) {
        return HAL_ERR_NOT_READY;
    }
    esp_err_t err = esp_ota_begin(s_ota_part, (image_size > 0U) ? image_size : OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return HAL_ERR_IO;
    }
    s_ota_active = true;
    return HAL_OK;
}

static int bsp_ota_write(const void *data, size_t len)
{
    if (!s_ota_active || data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (len == 0U) {
        return HAL_OK;
    }
    return (esp_ota_write(s_ota_handle, data, len) == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_ota_end(void)
{
    if (!s_ota_active) {
        return HAL_ERR_NOT_READY;
    }
    s_ota_active = false;
    return (esp_ota_end(s_ota_handle) == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_ota_abort(void)
{
    if (!s_ota_active) {
        return HAL_OK;
    }
    s_ota_active = false;
    return (esp_ota_abort(s_ota_handle) == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_ota_set_boot_next(void)
{
    if (s_ota_part == NULL) {
        return HAL_ERR_NOT_READY;
    }
    return (esp_ota_set_boot_partition(s_ota_part) == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_ota_mark_app_valid(void)
{
    return (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_ota_trigger_rollback(void)
{
    return (esp_ota_mark_app_invalid_rollback_and_reboot() == ESP_OK) ? HAL_OK : HAL_ERR_IO;
}

static bool bsp_ota_is_pending_verify(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (running != NULL && esp_ota_get_state_partition(running, &state) == ESP_OK) {
        return (state == ESP_OTA_IMG_PENDING_VERIFY);
    }
    return false;
}

hal_flash_t bsp_flash = {
    .ota_begin = bsp_ota_begin,
    .ota_write = bsp_ota_write,
    .ota_end = bsp_ota_end,
    .ota_abort = bsp_ota_abort,
    .ota_set_boot_next = bsp_ota_set_boot_next,
    .ota_mark_app_valid = bsp_ota_mark_app_valid,
    .ota_trigger_rollback = bsp_ota_trigger_rollback,
    .ota_is_pending_verify = bsp_ota_is_pending_verify
};
