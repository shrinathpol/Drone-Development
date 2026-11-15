#include "board_lilygo_sim7000g.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "bsp_i2c";

#define MAX_I2C_CACHED_DEVS 16
#define I2C_XFER_TIMEOUT_MS 20
#define I2C_PROBE_TIMEOUT_MS 15

typedef struct {
    uint8_t addr;
    i2c_master_dev_handle_t handle;
} i2c_dev_entry_t;

typedef struct {
    i2c_port_num_t port;
    int sda_pin;
    int scl_pin;
    uint32_t clk_hz;
    bool initialized;
    i2c_master_bus_handle_t bus_handle;
    i2c_dev_entry_t devs[MAX_I2C_CACHED_DEVS];
    size_t dev_count;
    uint8_t nack_addrs[MAX_I2C_CACHED_DEVS];
    size_t nack_count;
} esp32_i2c_ctx_t;

static esp32_i2c_ctx_t s_i2c0_ctx = {
    .port = I2C_NUM_0,
    .sda_pin = PIN_I2C_SDA,
    .scl_pin = PIN_I2C_SCL,
    .clk_hz = 400000U,
    .initialized = false,
    .bus_handle = NULL,
    .dev_count = 0,
    .nack_count = 0
};

static i2c_master_dev_handle_t get_or_add_device(esp32_i2c_ctx_t *bus, uint8_t dev_addr)
{
    for (size_t i = 0; i < bus->dev_count; i++) {
        if (bus->devs[i].addr == dev_addr) {
            return bus->devs[i].handle;
        }
    }

    for (size_t i = 0; i < bus->nack_count; i++) {
        if (bus->nack_addrs[i] == dev_addr) {
            return NULL;
        }
    }

    if (bus->dev_count >= MAX_I2C_CACHED_DEVS) {
        return NULL;
    }

    /* Probe before creating a device handle so floating/unpopulated I2C addresses
     * do not stall i2c_master_transmit_receive() on repeated polls. */
    esp_err_t probe_err = i2c_master_probe(bus->bus_handle, dev_addr, I2C_PROBE_TIMEOUT_MS);
    if (probe_err != ESP_OK) {
        if (bus->nack_count < MAX_I2C_CACHED_DEVS) {
            bus->nack_addrs[bus->nack_count++] = dev_addr;
        }
        ESP_LOGW(TAG, "I2C device 0x%02X not detected (NACK: %s)", dev_addr, esp_err_to_name(probe_err));
        return NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = bus->clk_hz,
        .scl_wait_us = 0,
        .flags.disable_ack_check = 0
    };

    i2c_master_dev_handle_t handle = NULL;
    if (i2c_master_bus_add_device(bus->bus_handle, &dev_cfg, &handle) != ESP_OK) {
        return NULL;
    }

    ESP_LOGI(TAG, "I2C device 0x%02X detected (ACK)", dev_addr);
    bus->devs[bus->dev_count].addr = dev_addr;
    bus->devs[bus->dev_count].handle = handle;
    bus->dev_count++;
    return handle;
}

static int bsp_i2c_init(void *ctx, uint32_t clk_hz)
{
    esp32_i2c_ctx_t *bus = (esp32_i2c_ctx_t *)ctx;
    if (bus == NULL || clk_hz == 0U) {
        return HAL_ERR_INVALID_ARG;
    }

    if (bus->initialized) {
        return HAL_OK;
    }

    bus->clk_hz = clk_hz;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = bus->port,
        .sda_io_num = (gpio_num_t)bus->sda_pin,
        .scl_io_num = (gpio_num_t)bus->scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = 1,
        .flags.allow_pd = 0
    };

    if (i2c_new_master_bus(&bus_cfg, &bus->bus_handle) != ESP_OK) {
        return HAL_ERR_BUS;
    }

    bus->initialized = true;
    return HAL_OK;
}

static int bsp_i2c_write_reg(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t len)
{
    esp32_i2c_ctx_t *bus = (esp32_i2c_ctx_t *)ctx;
    if (bus == NULL || !bus->initialized) {
        return HAL_ERR_NOT_READY;
    }
    if (len > 64U || (len > 0U && data == NULL)) {
        return HAL_ERR_INVALID_ARG;
    }

    i2c_master_dev_handle_t dev = get_or_add_device(bus, dev_addr);
    if (dev == NULL) {
        return HAL_ERR_BUS;
    }

    uint8_t tx_buf[65];
    tx_buf[0] = reg;
    if (len > 0U) {
        memcpy(&tx_buf[1], data, len);
    }

    esp_err_t err = i2c_master_transmit(dev, tx_buf, len + 1U, I2C_XFER_TIMEOUT_MS);
    if (err == ESP_ERR_TIMEOUT) {
        return HAL_ERR_TIMEOUT;
    }
    return (err == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

static int bsp_i2c_read_reg(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    esp32_i2c_ctx_t *bus = (esp32_i2c_ctx_t *)ctx;
    if (bus == NULL || !bus->initialized) {
        return HAL_ERR_NOT_READY;
    }
    if (data == NULL || len == 0U) {
        return HAL_ERR_INVALID_ARG;
    }

    i2c_master_dev_handle_t dev = get_or_add_device(bus, dev_addr);
    if (dev == NULL) {
        return HAL_ERR_BUS;
    }

    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1U, data, len, I2C_XFER_TIMEOUT_MS);
    if (err == ESP_ERR_TIMEOUT) {
        return HAL_ERR_TIMEOUT;
    }
    return (err == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

hal_i2c_t bsp_i2c0 = {
    .ctx = &s_i2c0_ctx,
    .init = bsp_i2c_init,
    .write_reg = bsp_i2c_write_reg,
    .read_reg = bsp_i2c_read_reg
};
