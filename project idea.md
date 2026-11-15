This master engineering specification and execution plan is designed so you can feed it directly into **Antigravity** (or any AI coding agent / IDE workspace) to generate the entire production-grade repository phase by phase, while keeping 100% of the application, flight control, and peripheral drivers strictly decoupled from the ESP32 silicon.

---

## Part 1: Master Prompt & Architecture Spec for Antigravity

Copy and paste the following block into **Antigravity** as your system/project context prompt. It locks down the coding rules so the agent never leaks ESP-IDF or Arduino dependencies into your core drivers or flight brain.

```markdown
# PROJECT SPECIFICATION: "PAN-BRAIN" MCU-AGNOSTIC DRONE & VEHICLE AUTOPILOT + TELEMETRY ENGINE

## 1. ARCHITECTURAL MANDATE (STRICT ZERO-DEPENDENCY RULE)
You are generating firmware in C11 for a dual-role Autonomous Drone Flight Controller and Real-Time LTE/GPS Vehicle Tracker.
Currently, the target hardware is the **LilyGO T-SIM7000G (ESP32-WROVER-E with 8MB PSRAM)**. However, the codebase MUST be 100% portable to STM32 (STM32H7/F4/WB) or NXP MCUs in the future by changing ONLY the `bsp/` folder.

### Strict Layering Rules:
1. **`bsp/` (Board Support Package & Silicon HAL):**
   - The ONLY directory allowed to `#include` MCU-specific headers (`driver/uart.h`, `driver/i2c.h`, `driver/spi_master.h`, `driver/mcpwm.h`, `esp_timer.h`, `freertos/FreeRTOS.h`).
   - Implements the function pointers defined in `bsp/hal_interfaces.h`.
2. **`drivers/` (Universal Hardware Peripheral Drivers):**
   - MUST NOT include any ESP-IDF, STM32 HAL, or Arduino headers. Only standard C headers (`<stdint.h>`, `<stdbool.h>`, `<string.h>`, `<stdio.h>`) and `bsp/hal_interfaces.h`.
   - Every driver takes a pointer to a HAL interface struct (`hal_uart_t*`, `hal_i2c_t*`, `hal_spi_t*`, `hal_pwm_t*`, `hal_gpio_t*`) during `init()`.
   - Zero blocking `delay()` loops inside drivers; use non-blocking state machines driven by `hal_sys_t->millis()` or `micros()`.
3. **`core/` (Middleware, DSP, AHRS, PID, Protocols, Failsafe):**
   - Pure ANSI C11 math and state machines.
   - Contains Biquad/Notch filters, Mahony/Madgwick Quaternion AHRS, Altitude Complementary/Kalman Filter, Cascaded PID Controller, CRSF/UBX/NMEA parsers, Offline Ring-Buffer Manager, and Config Manager.
4. **`app/` (System Orchestrator):**
   - Instantiates the `bsp` objects, injects them into `drivers`, connects `drivers` to `core`, and schedules tasks.
5. **`cloud/` (Google Cloud Backend & Web Dashboard):**
   - MQTT Broker config, Node.js/Go/Python telemetry ingestion & command dispatcher, TimescaleDB schema, and React/Leaflet real-time HUD + Waypoint control UI.

## 2. HARDWARE PINOUT (LILYGO T-SIM7000G BSP TARGET)
- **SIM7000G UART1:** TX = GPIO 27, RX = GPIO 26 (115200 baud, 8N1)
- **SIM7000G Control:** PWRKEY = GPIO 4, DTR/SLEEP = GPIO 25, STATUS = GPIO 36
- **SD Card (HSPI):** MOSI = GPIO 15, MISO = GPIO 2, SCLK = GPIO 14, CS = GPIO 13
- **Battery & Solar ADC:** BAT_ADC = GPIO 35 (Divider 1:2), SOLAR_ADC = GPIO 36
- **I2C0 Sensor Bus (400kHz):** SDA = GPIO 21, SCL = GPIO 22 (IMU, BMP390 Baro, QMC5883L Mag, VL53L1X ToF, INA226 Power)
- **IMU Wake-on-Motion / DataReady Interrupt:** GPIO 34 (RTC Wake capable)
- **External GPS / CRSF Radio UART2:** RX = GPIO 32, TX = GPIO 33
- **4x Brushless ESC Outputs (MCPWM):** M1 = GPIO 18, M2 = GPIO 19, M3 = GPIO 23, M4 = GPIO 5
- **RESERVED (DO NOT USE):** GPIO 16, GPIO 17 (Used internally by ESP32-WROVER-E 8MB PSRAM)

```

---

## Part 2: Complete Portable HAL Contract (`bsp/hal_interfaces.h`)

To guarantee that moving to another microcontroller only requires updating GPIO mappings and low-level bus drivers, use this exact header as the foundation of your project:

```c
#ifndef HAL_INTERFACES_H
#define HAL_INTERFACES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERR_TIMEOUT = -1,
    HAL_ERR_BUS = -2,
    HAL_ERR_INVALID_ARG = -3,
    HAL_ERR_NOT_READY = -4
} hal_status_t;

// 1. Generic System & Time Interface
typedef struct {
    uint32_t (*millis)(void);
    uint64_t (*micros)(void);
    void (*delay_ms)(uint32_t ms); // Only for boot-time init, forbidden in runtime loops
    void (*enter_deep_sleep)(uint32_t wake_gpio_mask);
} hal_sys_t;

// 2. Generic GPIO Interface
typedef struct {
    int (*mode_output)(uint8_t pin);
    int (*mode_input)(uint8_t pin, bool pullup);
    void (*write)(uint8_t pin, bool level);
    bool (*read)(uint8_t pin);
} hal_gpio_t;

// 3. Generic Non-Blocking UART Interface (Ring-buffer backed in BSP)
typedef struct {
    void *ctx; // Opaque pointer to MCU UART port handle
    int (*init)(void *ctx, uint32_t baudrate);
    int (*set_baudrate)(void *ctx, uint32_t baudrate);
    int (*write)(void *ctx, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint8_t *buf, size_t max_len); // Non-blocking: returns bytes available
    void (*flush_rx)(void *ctx);
} hal_uart_t;

// 4. Generic I2C Master Interface
typedef struct {
    void *ctx;
    int (*init)(void *ctx, uint32_t clk_hz);
    int (*write_reg)(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t len);
    int (*read_reg)(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len);
} hal_i2c_t;

// 5. Generic SPI Master Interface
typedef struct {
    void *ctx;
    int (*init)(void *ctx, uint32_t clk_hz);
    int (*transfer_reg)(void *ctx, uint8_t reg, const uint8_t *tx, uint8_t *rx, size_t len);
} hal_spi_t;

// 6. Generic PWM / ESC Output Interface
typedef struct {
    void *ctx;
    int (*init)(void *ctx, uint32_t freq_hz);
    int (*set_pulse_us)(void *ctx, uint8_t channel, uint16_t pulse_us); // 1000 - 2000us
} hal_pwm_t;

// 7. Generic ADC Interface
typedef struct {
    int (*init)(uint8_t channel);
    uint32_t (*read_mv)(uint8_t channel);
} hal_adc_t;

// 8. Generic Storage (SD Card / Flash File System) Interface
typedef struct {
    int (*mount)(void);
    int (*append_file)(const char *path, const void *data, size_t len);
    int (*read_file)(const char *path, void *buf, size_t max_len, size_t offset);
    int (*delete_file)(const char *path);
} hal_storage_t;

#endif // HAL_INTERFACES_H

```

---

## Part 3: Full Custom SIM7000G AT Command Reference & Non-Blocking State Machine

To avoid using blocking third-party libraries like TinyGSM, your custom driver (`drivers/modem_sim7000g.c`) must execute a **non-blocking deterministic FSM (Finite State Machine)** driven by `hal_sys->millis()`.

### A. Complete SIM7000G AT Command Sequence by Subsystem

#### 1. Hardware Power-On & Initialization Sequence

* **Hardware Boot Pulse:** Pull `PWRKEY` (GPIO 4) `HIGH` for `1100 ms`, then pull `LOW`. Wait `3000 ms` for UART readiness.
* `AT\r\n` $\rightarrow$ Expect `OK` (Sync baudrate; retry up to 10 times every 500ms).
* `ATE0\r\n` $\rightarrow$ Expect `OK` (Disable command echo to save UART parsing overhead).
* `AT+CMEE=2\r\n` $\rightarrow$ Expect `OK` (Enable verbose error reporting for diagnostics).
* `AT+CPIN?\r\n` $\rightarrow$ Expect `+CPIN: READY` (Verify SIM card is inserted and unlocked).
* `AT+CSCLK=0\r\n` $\rightarrow$ Expect `OK` (Disable UART sleep mode while active; pull `DTR` GPIO 25 LOW).

#### 2. Internal GNSS (GPS/GLONASS) Engine Control

*(Note: On SIM7000G, internal GNSS and LTE RF share power constraints on weak batteries; ensure LiPo/18650 cell is connected alongside USB).*

* `AT+CGNSPWR=1\r\n` $\rightarrow$ Expect `OK` (Power on internal GNSS engine).
* `AT+CGNSMOD=1,1,0,0\r\n` $\rightarrow$ Expect `OK` (Configure GPS + GLONASS mode for faster lock).
* `AT+CGNSINF\r\n` $\rightarrow$ Returns:
`+CGNSINF: <run>,<fix>,<utc_time>,<lat>,<lon>,<alt>,<speed_kmh>,<course>,<fix_mode>,...,<hdop>,<pdop>,<vdop>,...,<sats_in_view>,<sats_used>`
*Parse fields 1 (`fix == 1`), 2 (`UTC`), 3 (`Lat`), 4 (`Lon`), 5 (`Alt`), 6 (`Speed`), 7 (`Course`), and 14 (`Sats`).*
* `AT+CGNSURC=0\r\n` $\rightarrow$ Expect `OK` (Disable unsolicited GNSS output so it doesn't corrupt MQTT rx packets; poll `AT+CGNSINF` every 1000ms instead).

#### 3. Network Selection (LTE Cat-M1 / NB-IoT / GSM Fallback)

* `AT+CNMP=2\r\n` $\rightarrow$ Expect `OK` (`2` = Automatic LTE/GSM, `13` = GSM Only, `38` = LTE Only. In India, Jio/Airtel NB-IoT or Vi/BSNL 2G GSM fallback works best with `2`).
* `AT+CMNB=3\r\n` $\rightarrow$ Expect `OK` (`1` = Cat-M1, `2` = NB-IoT, `3` = Both Cat-M1 & NB-IoT).
* `AT+CSQ\r\n` $\rightarrow$ Expect `+CSQ: <rssi>,<ber>` (Verify `<rssi>` is between `2` and `31`; `99` means no signal).
* `AT+CGREG?\r\n` or `AT+CEREG?\r\n` $\rightarrow$ Expect `+CGREG: 0,1` (Home Registered) or `+CGREG: 0,5` (Roaming Registered).

#### 4. PDP Context & Data Bearer Activation (SIM7000G Specific `CNACT`)

Unlike older SIM800 modules that use `AT+CSTT`/`AT+CIICR`, the SIM7000G uses `AT+CNACT` for its internal IP stack (used by native MQTT/HTTP):

* `AT+CGDCONT=1,"IP","<YOUR_APN>"\r\n` $\rightarrow$ Expect `OK` (Set PDP context APN, e.g., `"iot.com"` or `"airtelgprs.com"`).
* `AT+CNCFG=0,1,"<YOUR_APN>"\r\n` $\rightarrow$ Expect `OK` (Bind PDP context 0 to IPv4 and APN).
* `AT+CNACT=0,1\r\n` $\rightarrow$ Expect `OK` followed by URC `+APP PDP: 0,ACTIVE` (Opens IP bearer).
* `AT+CNACT?\r\n` $\rightarrow$ Expect `+CNACT: 0,1,"10.x.x.x"` (Confirms valid IP address assigned).

#### 5. Native Hardware-Accelerated MQTT Client (`AT+SM...`)

Using the SIM7000G's built-in MQTT stack offloads TCP/IP framing from your MCU:

* `AT+SMCONF="CLIENTID","drone_brain_01"\r\n` $\rightarrow$ Expect `OK`
* `AT+SMCONF="URL","<GCP_SERVER_IP>",1883\r\n` $\rightarrow$ Expect `OK`
* `AT+SMCONF="KEEPTIME",60\r\n` $\rightarrow$ Expect `OK` (60s MQTT Keep-Alive ping).
* `AT+SMCONF="CLEANSS",1\r\n` $\rightarrow$ Expect `OK`
* `AT+SMCONF="USERNAME","<MQTT_USER>"\r\n` $\rightarrow$ Expect `OK`
* `AT+SMCONF="PASSWORD","<MQTT_PASS>"\r\n` $\rightarrow$ Expect `OK`
* `AT+SMCONN\r\n` $\rightarrow$ Expect `OK` (Connects to Cloud Broker; timeout up to 15s).
* `AT+SMSUB="drone/drone_brain_01/cmd",1\r\n` $\rightarrow$ Expect `OK` (Subscribe to Cloud-to-Drone command topic with QoS 1).
* **Publishing Live Telemetry:**

1. Send `AT+SMPUB="drone/drone_brain_01/telemetry",<payload_length>,0,0\r\n`
2. Wait for prompt character `>`
3. Write raw JSON or Binary struct bytes (`<payload_length>` bytes).
4. Expect `OK`.

* **Receiving Cloud Control Commands (Asynchronous URC):**
When the Google Cloud server sends a command (e.g., `RTH`, `WAYPOINT`, `DISARM`), the SIM7000G immediately outputs:
`+SMSUB: "drone/drone_brain_01/cmd","{\"cmd\":\"RTH\",\"alt\":35}"`
Your UART RX line parser intercepts any line starting with `+SMSUB:` and fires a callback directly to `core/failsafe_engine.c`.

#### 6. Emergency SMS Fallback & Network Time Sync

* **Network UTC Epoch Time Sync:**
* `AT+CLTS=1\r\n` $\rightarrow$ Enable local timestamp from network tower.
* `AT+CCLK?\r\n` $\rightarrow$ Returns `+CCLK: "26/11/15,13:20:00+22"`. Convert to UTC Epoch timestamp.

* **Emergency Crash / Low-Battery SMS Broadcast (Works even when data bearer fails):**
* `AT+CMGF=1\r\n` $\rightarrow$ Expect `OK` (Text mode).
* `AT+CMGS="+91XXXXXXXXXX"\r\n` $\rightarrow$ Wait for `>` $\rightarrow$ Send `"ALERT: Drone RTH/LowBat at Lat:15.3647,Lon:75.1240,Bat:10.2V"` + `0x1A` (`Ctrl+Z`).

---

### B. Portable Non-Blocking SIM7000G Driver Header (`drivers/modem_sim7000g.h`)

```c
#ifndef MODEM_SIM7000G_H
#define MODEM_SIM7000G_H

#include "hal_interfaces.h"

typedef enum {
    MODEM_STATE_OFF = 0,
    MODEM_STATE_PWRKEY_PULSE,
    MODEM_STATE_WAIT_BOOT,
    MODEM_STATE_AT_SYNC,
    MODEM_STATE_INIT_CFG,
    MODEM_STATE_ENABLE_GNSS,
    MODEM_STATE_WAIT_NET_REG,
    MODEM_STATE_ACTIVATE_PDP,
    MODEM_STATE_MQTT_CONFIG,
    MODEM_STATE_MQTT_CONNECT,
    MODEM_STATE_MQTT_SUB,
    MODEM_STATE_ONLINE_IDLE,
    MODEM_STATE_MQTT_PUB_WAIT_PROMPT,
    MODEM_STATE_MQTT_PUB_SEND_PAYLOAD,
    MODEM_STATE_ERROR_RECOVERY
} modem_fsm_state_t;

typedef struct {
    bool fix_valid;
    double latitude;
    double longitude;
    float altitude_m;
    float speed_kmh;
    float heading_deg;
    uint8_t satellites;
    uint32_t utc_epoch;
} modem_gnss_t;

typedef void (*mqtt_cmd_callback_t)(const char *topic, const uint8_t *payload, size_t len);

typedef struct {
    // Hardware Abstraction Pointers (Injected from BSP)
    hal_uart_t *uart;
    hal_gpio_t *gpio;
    hal_sys_t  *sys;
    uint8_t pin_pwrkey;
    uint8_t pin_dtr;

    // Configuration
    const char *apn;
    const char *mqtt_host;
    uint16_t mqtt_port;
    const char *device_id;

    // Runtime State Machine
    modem_fsm_state_t state;
    uint32_t state_entry_ms;
    uint32_t last_gnss_poll_ms;
    uint32_t last_csq_poll_ms;
    uint8_t retry_count;
    int8_t rssi_dbm;
    bool is_net_connected;
    bool is_mqtt_connected;

    // Parsed Internal GNSS Data
    modem_gnss_t gnss;

    // TX/RX Buffers
    char rx_line_buf[512];
    size_t rx_idx;
    uint8_t pub_buf[512];
    size_t pub_len;
    char pub_topic[64];
    bool pub_pending;

    // Cloud Command URC Hook
    mqtt_cmd_callback_t on_cloud_command;
} sim7000g_dev_t;

void sim7000g_init(sim7000g_dev_t *dev);
void sim7000g_Step_poll(sim7000g_dev_t *dev); // Call every 10ms in FreeRTOS task or superloop
int  sim7000g_mqtt_publish(sim7000g_dev_t *dev, const char *topic, const void *data, size_t len);
int  sim7000g_send_emergency_sms(sim7000g_dev_t *dev, const char *phone, const char *msg);

#endif

```

---

## Part 4: Step-by-Step Code Generation Roadmap for Antigravity

Do **not** ask Antigravity to generate all 35 files in a single prompt—LLMs truncate code when asked for too much at once. Feed Antigravity these **7 sequential prompts** one at a time. Wait for each phase to compile cleanly before moving to the next.

### Phase 1: Project Skeleton, CMake & Portable HAL Contract

> **Prompt to give Antigravity:**
> *"Create the ESP-IDF CMake project structure (`CMakeLists.txt`, `main/CMakeLists.txt`, `sdkconfig.defaults` with 8MB PSRAM enabled). Generate `bsp/hal_interfaces.h` containing pure C function-pointer structs for `hal_sys_t`, `hal_gpio_t`, `hal_uart_t`, `hal_i2c_t`, `hal_spi_t`, `hal_pwm_t`, `hal_adc_t`, and `hal_storage_t`. Then implement the ESP32-WROVER-E BSP inside `bsp/esp32_wrover/` (`bsp_uart.c`, `bsp_i2c.c`, `bsp_spi.c`, `bsp_mcpwm.c`, `bsp_sdcard.c`, `bsp_gpio_sys.c`, and `board_lilygo_sim7000g.h`) with the exact LilyGO T-SIM7000G pinout. Ensure zero ESP-IDF headers exist outside `bsp/esp32_wrover/`."*

### Phase 2: Non-Blocking SIM7000G Driver & Offline SD Queue

> **Prompt to give Antigravity:**
> *"Generate `drivers/modem_sim7000g.h` and `drivers/modem_sim7000g.c` using ONLY `hal_interfaces.h`. Implement the complete non-blocking AT state machine: hardware PWRKEY pulse, `ATE0`, `AT+CGNSPWR=1`, `AT+CGNSINF` parser, `AT+CNMP=2`, `AT+CNACT=0,1` PDP activation, and `AT+SMCONF`/`AT+SMCONN`/`AT+SMPUB`/`+SMSUB:` MQTT telemetry & command URC handler, plus `AT+CCLK?` UTC epoch sync and `AT+CMGS` emergency SMS. Also generate `core/offline_queue.c` which buffers telemetry packets into PSRAM and flushes them to `/sdcard/offline_log.bin` whenever `is_mqtt_connected == false`, automatically re-streaming them when LTE reconnects."*

### Phase 3: IMU, Environmental Sensors & External Dual-GPS Drivers

> **Prompt to give Antigravity:**
> *"Generate hardware-independent drivers in `drivers/` using ONLY `hal_i2c_t`, `hal_spi_t`, and `hal_uart_t`:
>
> 1. `imu_universal.c`: Vtable-based 6-axis IMU driver supporting both ICM-42688-P (SPI/I2C) and MPU6050/BMI270 (I2C), including hardware Wake-on-Motion (`hw_enable_wom`) interrupt configuration on GPIO 34 for vehicle anti-theft mode.
> 2. `env_sensors.c`: Drivers for BMP390 Barometer, QMC5883L 3-axis Magnetometer, VL53L1X ToF laser rangefinder, and INA226 current/voltage sensor.
> 3. `gps_ublox.c`: Non-blocking UART NMEA + UBX binary parser for an external 10Hz u-blox M9N/M10 GPS module, with automatic fallback to `sim7000g_dev_t.gnss` if the external UART GPS times out for >2 seconds."*
>
>

### Phase 4: Drone Actuators, RC Receiver & Camera Driver

> **Prompt to give Antigravity:**
> *"Generate the remaining portable drivers in `drivers/`:
>
> 1. `motor_esc.c`: Universal 4-motor brushless ESC driver using `hal_pwm_t` with safety arming interlocks, motor idle spin speed, throttle slew-rate limiting, and motor-mixing matrix (Quad-X layout).
> 2. `rc_radio.c`: Non-blocking CRSF (ELRS) and SBUS single-wire UART receiver parser extracting 16 channels (`roll, pitch, yaw, throttle, arm_switch, mode_switch, rth_switch`) and link-quality/failsafe flags.
> 3. `camera_ctrl.c`: PWM 2-axis gimbal pitch/roll stabilizer output and RunCam/MAVLink UART shutter trigger."*
>
>

### Phase 5: AHRS Sensor Fusion, Cascaded Flight PID & Autonomous RTH Brain

> **Prompt to give Antigravity:**
> *"Generate the pure C11 flight & intelligence engine inside `core/`:
>
> 1. `ahrs_fusion.c`: Biquad Low-Pass Filter for gyro/accel noise, Mahony 9-DOF Quaternion filter (fusing Gyro + Accel + Tilt-compensated Mag), and a Complementary/Kalman vertical altitude estimator fusing Earth-frame Accel-Z, Barometer, and ToF rangefinder.
> 2. `flight_pid.c`: Cascaded Attitude (Angle) and Rate PID controllers for Roll, Pitch, and Yaw, plus GPS Position Hold (Loiter) and Waypoint Navigation controller (Haversine distance & bearing calculator).
> 3. `failsafe_engine.c`: Dual-role state machine (`ROLE_DRONE` vs `ROLE_VEHICLE`). Implements RC-Loss Loiter, Dual-Loss (RC + LTE lost > 3s) Autonomous Return-to-Home (climb to safe altitude -> fly to Home Lat/Lon -> auto-land with ToF laser), Crash/Flip >75-deg emergency disarm, and Vehicle Deep-Sleep Wake-on-Motion alert."*
>
>

### Phase 6: BLE + Wi-Fi Provisioning & Dual-Core FreeRTOS App

> **Prompt to give Antigravity:**
> *"Generate `core/config_manager.c` and `bsp/esp32_wrover/bsp_wireless_cfg.c` to provide a BLE GATT Server and Wi-Fi SoftAP WebSocket JSON configuration interface when disarmed, saving PID gains, APN, MQTT Broker URL, and `ROLE_DRONE`/`ROLE_VEHICLE` mode to `/sdcard/config.json`. Then generate `app/main_brain.c` orchestrating Core 1 (500Hz IMU/AHRS/PID/ESC loop, 100Hz RC/GPS loop) and Core 0 (SIM7000G FSM, SD card logger, BLE/Wi-Fi config)."*

### Phase 7: Google Cloud Server & Real-Time Web Control Dashboard

> **Prompt to give Antigravity:**
> *"Generate the `cloud/` directory for deployment on Google Cloud Platform:
>
> 1. `docker-compose.yml`: Mosquitto/EMQX MQTT broker, PostgreSQL + TimescaleDB, and a Node.js (TypeScript) or Python FastAPI backend.
> 2. Backend service that subscribes to `drone/+/telemetry`, stores time-series coordinates/attitude, exposes REST APIs for historical trip playback, and bridges real-time telemetry and uplink commands (`ARM`, `DISARM`, `RTH`, `GOTO_WAYPOINT`, `SET_MODE`) to the browser via WebSockets.
> 3. Frontend dashboard (React + Tailwind + Leaflet/Mapbox GL) displaying a live dark-mode map, 3D artificial horizon HUD, battery/current gauges, signal strength, and click-to-fly waypoint mission planner."*
>
>

---

## Part 5: How to Port to STM32 (or Any Other MCU) Later in 1 Hour

Because every file in `drivers/`, `core/`, and `app/` only talks to `hal_interfaces.h`, when you transition from the ESP32-WROVER-E to an **STM32WB55**, **STM32F405**, or **STM32H743** in your VS Code + CMake workflow:

1. Copy `drivers/`, `core/`, and `bsp/hal_interfaces.h` directly into your STM32Cube CMake project—**zero lines of code change** in those folders.
2. Create `bsp/stm32/` and wrap `HAL_UART_Transmit_DMA`, `HAL_I2C_Mem_Read`, `HAL_SPI_TransmitReceive`, and `TIM1->CCR1` into the `hal_uart_t`, `hal_i2c_t`, `hal_spi_t`, and `hal_pwm_t` structs.
3. Pass your new `&stm32_uart1` and `&stm32_i2c1` pointers into `sim7000g_init()` and `imu_init()`.

If you want, we can execute **Phase 1 & Phase 2 right here right now**—I can write the complete code for `hal_interfaces.h`, the LilyGO ESP32 BSP, and the full `modem_sim7000g.c` implementation so you can drop them straight into your workspace.
