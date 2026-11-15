#include "board_lilygo_sim7000g.h"

#include "driver/uart.h"
#include "esp_log.h"

#define UART_RX_RING_BYTES 2048
#define UART_TX_RING_BYTES 1024

typedef struct {
    uart_port_t port;
    int tx_pin;
    int rx_pin;
    bool initialized;
} esp32_uart_ctx_t;

static esp32_uart_ctx_t s_uart1_ctx = {
    .port = UART_NUM_1,
    .tx_pin = PIN_SIM_TX,
    .rx_pin = PIN_SIM_RX,
    .initialized = false
};

static esp32_uart_ctx_t s_uart2_ctx = {
    .port = UART_NUM_2,
    .tx_pin = PIN_EXT_UART_TX,
    .rx_pin = PIN_EXT_UART_RX,
    .initialized = false
};

static int bsp_uart_init(void *ctx, uint32_t baudrate)
{
    esp32_uart_ctx_t *u = (esp32_uart_ctx_t *)ctx;
    if (u == NULL || baudrate == 0U) {
        return HAL_ERR_INVALID_ARG;
    }

    if (u->initialized) {
        return (uart_set_baudrate(u->port, baudrate) == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
    }

    uart_config_t cfg = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    if (uart_driver_install(u->port, UART_RX_RING_BYTES, UART_TX_RING_BYTES, 0, NULL, 0) != ESP_OK) {
        return HAL_ERR_BUS;
    }
    if (uart_param_config(u->port, &cfg) != ESP_OK) {
        return HAL_ERR_BUS;
    }
    if (uart_set_pin(u->port, u->tx_pin, u->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return HAL_ERR_BUS;
    }

    u->initialized = true;
    return HAL_OK;
}

static int bsp_uart_set_baudrate(void *ctx, uint32_t baudrate)
{
    esp32_uart_ctx_t *u = (esp32_uart_ctx_t *)ctx;
    if (u == NULL || !u->initialized || baudrate == 0U) {
        return HAL_ERR_INVALID_ARG;
    }
    return (uart_set_baudrate(u->port, baudrate) == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

static int bsp_uart_write(void *ctx, const uint8_t *data, size_t len)
{
    esp32_uart_ctx_t *u = (esp32_uart_ctx_t *)ctx;
    if (u == NULL || !u->initialized || data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (len == 0U) {
        return 0;
    }
    int written = uart_write_bytes(u->port, (const char *)data, len);
    return (written >= 0) ? written : HAL_ERR_BUS;
}

static int bsp_uart_read(void *ctx, uint8_t *buf, size_t max_len)
{
    esp32_uart_ctx_t *u = (esp32_uart_ctx_t *)ctx;
    if (u == NULL || !u->initialized || buf == NULL || max_len == 0U) {
        return 0;
    }
    size_t buffered = 0;
    if (uart_get_buffered_data_len(u->port, &buffered) != ESP_OK || buffered == 0U) {
        return 0;
    }
    size_t to_read = (buffered < max_len) ? buffered : max_len;
    int got = uart_read_bytes(u->port, buf, to_read, 0);
    return (got > 0) ? got : 0;
}

static void bsp_uart_flush_rx(void *ctx)
{
    esp32_uart_ctx_t *u = (esp32_uart_ctx_t *)ctx;
    if (u != NULL && u->initialized) {
        uart_flush_input(u->port);
    }
}

hal_uart_t bsp_uart_sim = {
    .ctx = &s_uart1_ctx,
    .init = bsp_uart_init,
    .set_baudrate = bsp_uart_set_baudrate,
    .write = bsp_uart_write,
    .read = bsp_uart_read,
    .flush_rx = bsp_uart_flush_rx
};

hal_uart_t bsp_uart_ext = {
    .ctx = &s_uart2_ctx,
    .init = bsp_uart_init,
    .set_baudrate = bsp_uart_set_baudrate,
    .write = bsp_uart_write,
    .read = bsp_uart_read,
    .flush_rx = bsp_uart_flush_rx
};
