#ifndef BOARD_LILYGO_SIM7000G_H
#define BOARD_LILYGO_SIM7000G_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LilyGO T-SIM7000G (ESP32-WROVER-E 8MB PSRAM) Hardware Pinout
 *
 * CRITICAL HARDWARE RULE:
 *   GPIO 16 and GPIO 17 are wired internally to the 8MB SPI PSRAM on ESP32-WROVER-E.
 *   GPIO 6..11 are wired internally to SPI Flash. NEVER configure these pins.
 */
#define PIN_PSRAM_RESERVED_0    16
#define PIN_PSRAM_RESERVED_1    17

/* SIM7000G Cellular + GNSS Modem (UART1) */
#define PIN_SIM_TX              27
#define PIN_SIM_RX              26
#define PIN_SIM_PWRKEY          4
#define PIN_SIM_DTR_SLEEP       25
/* Note: On LilyGO T-SIM7000G boards GPIO 36 is shared/routed to ADC/Solar sensing in some revs.
 * Guard ADC vs digital usage cleanly in BSP. */
#define PIN_SIM_STATUS          36

/* MicroSD Card (HSPI / SPI2_HOST) */
#define PIN_SD_MOSI             15
#define PIN_SD_MISO             2
#define PIN_SD_SCLK             14
#define PIN_SD_CS               13

/* Battery & Solar Voltage ADC (ADC1 input-only pins) */
#define PIN_BAT_ADC             35  /* 1:2 resistive divider */
#define PIN_SOLAR_ADC           36

/* I2C0 Sensor Bus (400kHz: IMU, BMP390, QMC5883L, VL53L1X, INA226) */
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22

/* IMU Hardware Wake-on-Motion / DataReady Interrupt (RTC_IO Wake capable, input-only) */
#define PIN_IMU_INT_WOM         34

/* External GPS / CRSF Radio (UART2) */
#define PIN_EXT_UART_RX         32
#define PIN_EXT_UART_TX         33

/* 4x Brushless ESC MCPWM Outputs (Group 0) */
#define PIN_ESC_M1              18  /* Front-Right CCW */
#define PIN_ESC_M2              19  /* Rear-Left   CCW */
#define PIN_ESC_M3              23  /* Front-Left  CW  */
#define PIN_ESC_M4              5   /* Rear-Right  CW  */

/* LilyGO T-SIM7000G Onboard Blue Status LED (GPIO 12, active-low on hardware)
 * NOTE: GPIO 12..15 also serve as ESP32 hardware JTAG pins (MTDI/MTCK/MTMS/MTDO).
 * When an OpenOCD/ESP-Prog JTAG debugger is attached (or BSP_JTAG_FORCE_ENABLE == 1),
 * the BSP automatically preserves GPIO 12..15 JTAG pin muxing over LED & SD HSPI. */
#define PIN_ONBOARD_LED             12
#define PIN_ONBOARD_LED_ACTIVE_LOW  true
#define BSP_JTAG_FORCE_ENABLE       0

/* Optional 2-Axis Gimbal PWM & Optional SPI Sensor Chip-Selects */
#define PIN_GIMBAL_PITCH        (-1) /* Reserved off GPIO 12 (dedicated to PIN_ONBOARD_LED) */
#define PIN_GIMBAL_ROLL         (-1) /* NC by default to avoid pin conflict; set GPIO if wired */
#define PIN_IMU_SPI_CS          (-1) /* Set GPIO if ICM-42688-P is wired on HSPI instead of I2C */
#define PIN_OPTFLOW_SPI_CS      (-1) /* Set GPIO if PMW3901 is wired on HSPI */

/* ADC Channel IDs passed to hal_adc_t */
#define BSP_ADC_CH_BAT          0
#define BSP_ADC_CH_SOLAR        1

/* Board-level HAL singletons exported by bsp/esp32_wrover/ */
extern hal_sys_t      bsp_sys;
extern hal_gpio_t     bsp_gpio;
extern hal_uart_t     bsp_uart_sim;
extern hal_uart_t     bsp_uart_ext;
extern hal_i2c_t      bsp_i2c0;
extern hal_spi_t      bsp_spi_imu;
extern hal_spi_t      bsp_spi_optflow;
extern hal_pwm_t      bsp_pwm_esc;
extern hal_pwm_t      bsp_pwm_gimbal;
extern hal_adc_t      bsp_adc;
extern hal_storage_t  bsp_storage;
extern hal_flash_t    bsp_flash;
extern hal_wireless_t bsp_wireless;

/**
 * @brief Initialize ESP32 NVS, PSRAM checks, and shared buses before app orchestration.
 */
int bsp_esp32_board_early_init(void);

/**
 * @brief Ensure HSPI (SPI2_HOST) bus is initialized exactly once (shared by SD card & SPI sensors).
 */
int bsp_spi_ensure_bus_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LILYGO_SIM7000G_H */
