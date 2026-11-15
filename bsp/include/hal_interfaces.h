#ifndef HAL_INTERFACES_H
#define HAL_INTERFACES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Portable HAL status codes shared across all MCU BSPs, drivers, and core logic.
 */
typedef enum {
    HAL_OK = 0,
    HAL_ERR_TIMEOUT = -1,
    HAL_ERR_BUS = -2,
    HAL_ERR_INVALID_ARG = -3,
    HAL_ERR_NOT_READY = -4,
    HAL_ERR_IO = -5,
    HAL_ERR_NO_MEM = -6
} hal_status_t;

/**
 * @brief 1. Generic System, Timing, Memory & Sleep Interface
 */
typedef struct {
    uint32_t (*millis)(void);
    uint64_t (*micros)(void);
    void (*delay_ms)(uint32_t ms); /**< Boot-time init only; forbidden inside runtime control/driver loops */
    void (*enter_deep_sleep)(uint32_t wake_gpio_mask);
    void *(*alloc_psram)(size_t bytes); /**< Optional PSRAM/large-heap allocator; falls back to malloc if NULL */
    void (*free_psram)(void *ptr);
    void (*log_msg)(uint8_t level, const char *tag, const char *msg);
    void (*reboot)(void);
} hal_sys_t;

/**
 * @brief 2. Generic GPIO Interface
 */
typedef struct {
    int (*mode_output)(uint8_t pin);
    int (*mode_input)(uint8_t pin, bool pullup);
    void (*write)(uint8_t pin, bool level);
    bool (*read)(uint8_t pin);
} hal_gpio_t;

/**
 * @brief 3. Generic Non-Blocking UART Interface (Ring-buffer backed in BSP)
 */
typedef struct {
    void *ctx; /**< Opaque pointer to MCU UART port handle */
    int (*init)(void *ctx, uint32_t baudrate);
    int (*set_baudrate)(void *ctx, uint32_t baudrate);
    int (*write)(void *ctx, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint8_t *buf, size_t max_len); /**< Non-blocking: returns bytes available */
    void (*flush_rx)(void *ctx);
} hal_uart_t;

/**
 * @brief 4. Generic I2C Master Interface
 */
typedef struct {
    void *ctx;
    int (*init)(void *ctx, uint32_t clk_hz);
    int (*write_reg)(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t len);
    int (*read_reg)(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len);
} hal_i2c_t;

/**
 * @brief 5. Generic SPI Master Interface
 */
typedef struct {
    void *ctx;
    int (*init)(void *ctx, uint32_t clk_hz);
    int (*transfer_reg)(void *ctx, uint8_t reg, const uint8_t *tx, uint8_t *rx, size_t len);
    int (*transfer_raw)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
} hal_spi_t;

/**
 * @brief 6. Generic PWM / ESC / DShot Output & Telemetry Interface
 */
typedef struct {
    void *ctx;
    int (*init)(void *ctx, uint32_t freq_hz);
    int (*set_pulse_us)(void *ctx, uint8_t channel, uint16_t pulse_us); /**< 1000 - 2000 us */
    int (*read_erpm)(void *ctx, uint8_t channel, uint32_t *erpm_out); /**< Optional Bidirectional DShot/ESC feedback */
} hal_pwm_t;

/**
 * @brief 7. Generic ADC Interface
 */
typedef struct {
    int (*init)(uint8_t channel);
    uint32_t (*read_mv)(uint8_t channel);
} hal_adc_t;

/**
 * @brief 8. Generic Storage (SD Card / Flash File System) Interface
 */
typedef struct {
    int (*mount)(void);
    int (*append_file)(const char *path, const void *data, size_t len);
    int (*read_file)(const char *path, void *buf, size_t max_len, size_t offset);
    int (*delete_file)(const char *path);
    int (*write_file)(const char *path, const void *data, size_t len); /**< Atomic/truncate write */
    int64_t (*file_size)(const char *path);
} hal_storage_t;

/**
 * @brief 9. Generic Dual-Bank OTA Flash Interface
 */
typedef struct {
    int (*ota_begin)(size_t image_size);
    int (*ota_write)(const void *data, size_t len);
    int (*ota_end)(void);
    int (*ota_abort)(void);
    int (*ota_set_boot_next)(void);
    int (*ota_mark_app_valid)(void);
    int (*ota_trigger_rollback)(void);
    bool (*ota_is_pending_verify)(void);
} hal_flash_t;

/**
 * @brief 10. Optional Local Wireless Config & UDP/WebSocket Link Interface
 */
typedef struct {
    void *ctx;
    int (*start_ap_cfg)(void *ctx, const char *ssid, const char *pass);
    int (*stop_ap_cfg)(void *ctx);
    int (*send_udp_telem)(void *ctx, const uint8_t *data, size_t len);
    int (*poll_rx_cmd)(void *ctx, uint8_t *buf, size_t max_len);
    bool (*has_pending_cfg_json)(void *ctx);
    int (*take_pending_cfg_json)(void *ctx, char *out_json, size_t max_len);
    void (*update_status_json)(void *ctx, const char *status_json);
} hal_wireless_t;

#ifdef __cplusplus
}
#endif

#endif /* HAL_INTERFACES_H */
