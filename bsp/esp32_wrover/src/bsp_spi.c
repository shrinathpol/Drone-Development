#include "board_lilygo_sim7000g.h"

#include "driver/spi_master.h"
#include "esp_cpu.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "bsp_spi";

typedef struct {
    int cs_pin;
    spi_device_handle_t dev;
    bool initialized;
} esp32_spi_ctx_t;

static bool s_hspi_bus_ready = false;

static esp32_spi_ctx_t s_spi_imu_ctx = {
    .cs_pin = PIN_IMU_SPI_CS,
    .dev = NULL,
    .initialized = false
};

static esp32_spi_ctx_t s_spi_optflow_ctx = {
    .cs_pin = PIN_OPTFLOW_SPI_CS,
    .dev = NULL,
    .initialized = false
};

int bsp_spi_ensure_bus_init(void)
{
    if (esp_cpu_dbgr_is_attached() || (BSP_JTAG_FORCE_ENABLE != 0)) {
        ESP_LOGW(TAG, "HSPI (GPIO 13/14/15) skipped to preserve JTAG debugging");
        return HAL_ERR_NOT_READY;
    }
    if (s_hspi_bus_ready) {
        return HAL_OK;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SD_MOSI,
        .miso_io_num = PIN_SD_MISO,
        .sclk_io_num = PIN_SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return HAL_ERR_BUS;
    }

    s_hspi_bus_ready = true;
    return HAL_OK;
}

static int bsp_spi_init_dev(void *ctx, uint32_t clk_hz)
{
    esp32_spi_ctx_t *s = (esp32_spi_ctx_t *)ctx;
    if (s == NULL || clk_hz == 0U) {
        return HAL_ERR_INVALID_ARG;
    }
    if (s->cs_pin < 0) {
        return HAL_ERR_NOT_READY;
    }
    if (s->initialized) {
        return HAL_OK;
    }
    if (bsp_spi_ensure_bus_init() != HAL_OK) {
        return HAL_ERR_BUS;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = (int)clk_hz,
        .mode = 0,
        .spics_io_num = s->cs_pin,
        .queue_size = 4
    };

    if (spi_bus_add_device(SPI2_HOST, &dev_cfg, &s->dev) != ESP_OK) {
        return HAL_ERR_BUS;
    }

    s->initialized = true;
    return HAL_OK;
}

static int bsp_spi_transfer_raw(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    esp32_spi_ctx_t *s = (esp32_spi_ctx_t *)ctx;
    if (s == NULL || !s->initialized || s->dev == NULL) {
        return HAL_ERR_NOT_READY;
    }
    if (len == 0U) {
        return HAL_OK;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8U;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    return (spi_device_polling_transmit(s->dev, &t) == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

static int bsp_spi_transfer_reg(void *ctx, uint8_t reg, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len > 64U) {
        return HAL_ERR_INVALID_ARG;
    }

    uint8_t tx_buf[65];
    uint8_t rx_buf[65];
    tx_buf[0] = reg;
    if (tx != NULL && len > 0U) {
        memcpy(&tx_buf[1], tx, len);
    } else if (len > 0U) {
        memset(&tx_buf[1], 0xFF, len);
    }

    int rc = bsp_spi_transfer_raw(ctx, tx_buf, rx_buf, len + 1U);
    if (rc == HAL_OK && rx != NULL && len > 0U) {
        memcpy(rx, &rx_buf[1], len);
    }
    return rc;
}

hal_spi_t bsp_spi_imu = {
    .ctx = &s_spi_imu_ctx,
    .init = bsp_spi_init_dev,
    .transfer_reg = bsp_spi_transfer_reg,
    .transfer_raw = bsp_spi_transfer_raw
};

hal_spi_t bsp_spi_optflow = {
    .ctx = &s_spi_optflow_ctx,
    .init = bsp_spi_init_dev,
    .transfer_reg = bsp_spi_transfer_reg,
    .transfer_raw = bsp_spi_transfer_raw
};
