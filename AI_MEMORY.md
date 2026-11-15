# PAN-BRAIN — Persistent AI & Engineering Development Memory

This document captures the complete architectural decisions, hardware mappings, protocol contracts, build quirks, verification results, and open bring-up notes for the **PAN-BRAIN** MCU-Agnostic Autopilot & BVLOS Telemetry Engine. Every future AI agent or engineer working on this codebase should read this file before modifying code.

---

## 1. Architectural Golden Rule (Zero-MCU-Dependency Mandate)

The codebase is strictly layered so the entire autopilot, sensor fusion, cryptography, mission planning, and device driver stack compiles cleanly on **any 32/64-bit C11 target** (`ESP32`, `STM32`, `RP2040`, Linux/Windows PC SITL) by swapping only the Board Support Package (`bsp/`).

### Layer Boundaries & Header Isolation
Every C layer strictly separates public headers (`include/`) from implementations (`src/`):

1. `core/include/` & `core/src/` — **Pure C11 Core Flight & Telemetry Engine**
   - Allowed headers: `<stdint.h>`, `<stdbool.h>`, `<stddef.h>`, `<math.h>`, `<string.h>`, `<stdio.h>`, `<stdlib.h>`, `<ctype.h>`, `hal_interfaces.h`, and peer `drivers/include/*.h` / `core/include/*.h`.
   - **FORBIDDEN**: `#include "esp_*"`, `#include "driver/*"`, `#include "freertos/*"`, `#include "sdkconfig.h"`, `#include "Arduino.h"`.
2. `drivers/include/` & `drivers/src/` — **MCU-Agnostic Sensor & Radio Drivers**
   - Talks to hardware strictly through `hal_interfaces.h` function-pointer structs (`hal_i2c_t`, `hal_spi_t`, `hal_uart_t`, `hal_gpio_t`, `hal_pwm_t`, `hal_adc_t`, `hal_sys_t`).
   - **FORBIDDEN**: Any ESP-IDF, FreeRTOS, or Arduino header.
3. `app/include/` & `app/src/` — **Platform-Agnostic Application Orchestrator**
   - `main_brain.c` schedules 500 Hz inner attitude loop, 100 Hz EKF/navigation/mission/failsafe loop, and 100 Hz comms/modem/LED/OTA loop.
   - **FORBIDDEN**: Any ESP-IDF or FreeRTOS header (`main/main.c` wraps `app/` inside pinned FreeRTOS tasks; `bsp/pc_sitl/src/sitl_main.c` calls the exact same 3 step functions in a host simulation loop).
4. `bsp/include/hal_interfaces.h` — **Abstract Hardware Contracts**
   - Defines `hal_sys_t`, `hal_gpio_t`, `hal_uart_t`, `hal_i2c_t`, `hal_spi_t`, `hal_pwm_t`, `hal_adc_t`, `hal_storage_t`, `hal_flash_t`, `hal_wireless_t`, and `HAL_OK` / `HAL_ERR_*` return codes.
5. `bsp/esp32_wrover/` — **ESP-IDF v6.1 + LilyGO T-SIM7000G Concrete BSP**
   - ONLY `bsp/esp32_wrover/` and `main/` may include ESP-IDF and FreeRTOS headers.
6. `bsp/pc_sitl/` — **PC Software-In-The-Loop Virtual BSP & 6-DOF Physics Engine**
   - Simulates quadrotor dynamics, virtual IMU/Baro/Mag/GPS/Battery, virtual SIM7000G AT state machine, and WinSock2/POSIX UDP streaming to `127.0.0.1:14550` (`telemetry`) and `:14551` (`commands`).

> **Verification rule after any C edit:** Re-run:
> `Select-String -Path drivers\**\*.c,drivers\**\*.h,core\**\*.c,core\**\*.h,app\**\*.c,app\**\*.h -Pattern '#include\s*[<"](esp_|driver/|freertos/|sdkconfig\.h|Arduino\.h)'`
> This must return **zero** matches.

---

## 2. Exact Directory & File Map

```text
drone/
├── CMakeLists.txt                       # Root ESP-IDF project ("pan_brain_firmware")
├── partitions.csv                       # 4MB flash layout (nvs, otadata, phy_init, ota_0 0x1A0000, ota_1 0x1A0000, spiffs 0xA0000)
├── sdkconfig.defaults                   # esp32, 4MB, 8MB PSRAM, 1000Hz FreeRTOS tick, bootloader rollback, FATFS LFN heap
├── AI_MEMORY.md / CLAUDE.md             # Persistent engineering memory
├── project idea.md / more advance.md    # Original specifications
│
├── main/
│   ├── CMakeLists.txt                   # Registers all src files & include dirs across app, core, drivers, bsp/esp32_wrover
│   └── main.c                           # Dual-core FreeRTOS entry (Core 1: 500Hz flight + 100Hz nav; Core 0: 100Hz comms)
│
├── app/
│   ├── include/main_brain.h             # pan_brain_bsp_t & pan_brain_system_t
│   └── src/main_brain.c                 # Unified init + 500Hz/100Hz step loops + cloud cmd handler + LED state selector
│
├── core/
│   ├── include/
│   │   ├── ahrs_fusion.h                # Mahony 9-DOF quaternion AHRS & baro/ToF alt fusion
│   │   ├── config_manager.h             # /sdcard/config.json loader/writer/JSON patcher
│   │   ├── crypto_auth.h                # SHA-256, HMAC-SHA256, +-10s UTC anti-replay, 32-nonce cache
│   │   ├── dsp_filters.h                # 2nd-order Butterworth LPF, RPM harmonic notches, 64-pt Radix-2 FFT peak notch
│   │   ├── ekf_nav.h                    # 15-state Error-State/Nav EKF + PMW3901 optical flow auto-switching
│   │   ├── failsafe_engine.h            # ROLE_DRONE vs ROLE_VEHICLE failsafes, RTH, >75 deg crash kill, deep-sleep WOM
│   │   ├── flight_pid.h                 # Cascaded Angle P -> Rate PID (D-on-meas), Alt/Nav PID, Haversine helpers
│   │   ├── link_router.h                # Priority router: CRSF (1) -> Wi-Fi UDP (2) -> LTE MQTT (3) -> SMS (4)
│   │   ├── mission_engine.h             # Keep-In/Keep-Out ray-casting 3D geofence + wind/Wh RTH + Douglas-Peucker breadcrumbs
│   │   ├── offline_queue.h              # 2048-pkt PSRAM ring buffer + /sdcard/offline_log.bin spill & drip replay
│   │   ├── ota_manager.h                # /sdcard/fw_update.bin chunk staging, SHA-256 verify while disarmed, hal_flash_t + rollback
│   │   └── telemetry_proto.h            # 36-byte packed LE pan_telem_pkt_t + CRC-16-CCITT + JSON formatter
│   └── src/ (matching .c files)
│
├── drivers/
│   ├── include/
│   │   ├── camera_ctrl.h                # 2-axis gimbal stabilizer & RunCam UART shutter/record trigger
│   │   ├── env_sensors.h                # BMP390 (NVM trim comp), QMC5883L, VL53L1X, INA226 + ADC battery fallback
│   │   ├── gps_ublox.h                  # Non-blocking UBX-NAV-PVT & NMEA GGA/RMC parser (>2000ms auto-fallback to SIM7000G GNSS)
│   │   ├── imu_universal.h              # Vtable auto-probe: ICM-42688-P (SPI/I2C), MPU6050 (I2C), BMI270 (I2C), SITL virtual + WOM
│   │   ├── led_status.h                 # Non-blocking GPIO 12 onboard status LED state machine (8 patterns)
│   │   ├── modem_sim7000g.h             # Non-blocking SIM7000G AT FSM (PWRKEY 1100ms, GNSS, Cat-M/NB-IoT, MQTT, CCLK UTC, SMS)
│   │   ├── motor_esc.h                  # Quad-X mixer, arming interlock, slew-rate limiter, eRPM feedback, prop-strike alert
│   │   ├── optflow_pmw3901.h            # PMW3901 SPI motion burst & gyro-derotated velocity
│   │   └── rc_radio.h                   # CRSF (0xC8 + CRC8 0xD5) & SBUS (0x0F) 16-ch parser + 500ms failsafe
│   └── src/ (matching .c files)
│
├── bsp/
│   ├── include/hal_interfaces.h
│   ├── esp32_wrover/
│   │   ├── include/board_lilygo_sim7000g.h
│   │   └── src/
│   │       ├── bsp_gpio_sys.c           # Sys tick, PSRAM alloc, EXT0(GPIO34)/EXT1 deep sleep, JTAG GPIO 12..15 pin guard
│   │       ├── bsp_uart.c               # UART1 SIM7000G (27/26) & UART2 Ext GPS/CRSF (32/33)
│   │       ├── bsp_i2c.c                # ESP-IDF v6.1 driver/i2c_master.h
│   │       ├── bsp_spi.c                # Shared SPI2_HOST HSPI (guarded when JTAG attached)
│   │       ├── bsp_mcpwm.c              # ESP-IDF v6.1 driver/mcpwm_prelude.h (400Hz PWM / OneShot125 ESCs)
│   │       ├── bsp_sdcard.c             # SDSPI FATFS (/sdcard), esp_adc/adc_oneshot.h + cali, esp_ota_ops.h hal_flash_t
│   │       └── bsp_wireless_cfg.c       # Wi-Fi SoftAP ("PAN_BRAIN_CFG"), esp_http_server, UDP 14550 TX / 14551 RX
│   └── pc_sitl/
│       ├── CMakeLists.txt
│       ├── include/sitl_physics.h
│       └── src/ (sitl_physics.c, bsp_pc_sitl.c, sitl_main.c)
│
└── cloud/
    ├── docker-compose.yml               # eclipse-mosquitto:2, timescale/timescaledb:latest-pg15, backend, frontend
    ├── mosquitto/mosquitto.conf
    ├── db/init.sql                      # telemetry_frames hypertable, command_audit, geofences, mission_waypoints
    ├── backend/                         # Node.js TypeScript: 36B+JSON decoder, HMAC signer, MQTT+UDP bridge, built-in SITL sim
    └── frontend/                        # React 18 + TypeScript + Tailwind + Leaflet Tactical HUD & Command Console
```

---

## 3. LilyGO T-SIM7000G Hardware Pinout & Critical Errata

Defined in [`bsp/esp32_wrover/include/board_lilygo_sim7000g.h`](bsp/esp32_wrover/include/board_lilygo_sim7000g.h):

| Subsystem | Signal / Pin Name | ESP32 GPIO | Hardware Notes |
| :--- | :--- | :--- | :--- |
| **PSRAM Reserved** | `PIN_PSRAM_RESERVED_0/1` | **16, 17** | **DO NOT USE.** Wired internally to 8MB PSRAM on ESP32-WROVER-E (`bsp_gpio_sys.c` rejects mode/write calls). |
| **SPI Flash** | Internal Flash | **6..11** | **DO NOT USE.** Internal SPI Flash bus. |
| **Onboard Blue LED** | `PIN_ONBOARD_LED` | **12** | **Active-Low (`PIN_ONBOARD_LED_ACTIVE_LOW = true`).** Moved `PIN_GIMBAL_PITCH` to `-1` to avoid clash. |
| **SIM7000G Modem** | `TX / RX / PWRKEY / DTR` | **27 / 26 / 4 / 25** | UART1 @ 115200 baud. Note: PWRKEY pulses LOW 1100 ms, waits 3000 ms non-blockingly. `PIN_SIM_STATUS = 36`. |
| **MicroSD (HSPI)** | `MOSI / MISO / SCLK / CS` | **15 / 2 / 14 / 13** | Shared `SPI2_HOST` @ 10 MHz (`bsp_spi_ensure_bus_init()`). |
| **ADC Inputs** | `PIN_BAT_ADC / PIN_SOLAR_ADC` | **35 / 36** | Input-only ADC1 channels (`BSP_ADC_CH_BAT=0`, `BSP_ADC_CH_SOLAR=1`). Voltage divider 1:2. |
| **I2C0 Sensor Bus** | `SDA / SCL` | **21 / 22** | 400 kHz master (`driver/i2c_master.h`). Shared by IMU, BMP390, QMC5883L, VL53L1X, INA226. |
| **IMU Wake-on-Motion** | `PIN_IMU_INT_WOM` | **34** | Input-only RTC_IO (no internal pull-up; `gpio_mode_input` skips pull-up on GPIO >= 34). Deep-sleep EXT0 wakeup on pin 34 (`sys_enter_deep_sleep`). |
| **Ext GPS / CRSF** | `RX / TX` | **32 / 33** | UART2 @ 115200 (NMEA/UBX auto-detect) or 420000 (CRSF). |
| **Brushless ESCs** | `M1 / M2 / M3 / M4` | **18 / 19 / 23 / 5** | MCPWM Group 0 @ 400 Hz (1000..2000 us). Quad-X: M1 FR-CCW, M2 RL-CCW, M3 FL-CW, M4 RR-CW. |

### Onboard Status LED Patterns (`drivers/src/led_status.c` on GPIO 12, active-low)
| Pattern Enum | Timing (ms) | Meaning |
| :--- | :--- | :--- |
| `LED_PAT_OFF_SLEEP` | Continuous OFF | Deep sleep / Vehicle standby |
| `LED_PAT_BOOT_INIT` | 100 ON / 100 OFF / 100 ON / 700 OFF | Boot double-blip |
| `LED_PAT_SEARCHING_NET_GPS` | 500 ON / 500 OFF | 1 Hz blink — waiting for SIM7000G cellular or GPS lock |
| `LED_PAT_DISARMED_ONLINE` | 80 ON / 1920 OFF | 2s heartbeat — online & ready (disarmed) |
| `LED_PAT_ARMED_FLIGHT` | 420 ON / 80 OFF | Long pulse — armed / active flight / active vehicle |
| `LED_PAT_FAILSAFE_RTH` | 100 ON / 100 OFF | 5 Hz fast blink — Smart-RTH or Autoland active |
| `LED_PAT_OTA_UPDATING` | 3x 70 ON/OFF + 650 OFF | Triple burst — SD/flash OTA staging |
| `LED_PAT_CRITICAL_ERROR` | 50 ON / 50 OFF | 10 Hz strobe — Emergency kill / Geofence / Prop strike / IMU fault |

### JTAG & ESP-Prog Hardware Warning (GPIO 12 `MTDI` Strapping!)
- **GPIO 12..15 overlap hardware JTAG** (`MTDI=12`, `MTCK=13`, `MTMS=14`, `MTDO=15`), colliding with `PIN_ONBOARD_LED` (GPIO 12) and MicroSD HSPI (`13/14/15`).
- **Firmware JTAG Guard**: `bsp_gpio_sys.c` (`bsp_is_jtag_pin_guarded()`) and `bsp_spi.c` (`bsp_spi_ensure_bus_init()`) check `esp_cpu_dbgr_is_attached() || BSP_JTAG_FORCE_ENABLE`. When a JTAG debugger is attached, GPIO 12..15 pin MUX and HSPI initialization are automatically preserved so OpenOCD does not disconnect.
- **CRITICAL HARDWARE STRAPPING ERRATUM**: On ESP32 (`ESP32-D0WD-V3`), **GPIO 12 (`MTDI`) latches `VDD_SDIO` flash voltage at reset** (`1` = 1.8V, `0` = 3.3V). If an ESP-Prog JTAG `TDI` wire pulls `GPIO 12` HIGH (or if ESP-Prog is connected without USB data power), the 3.3V internal SPI flash browns out during `esptool` flashing (`WARNING: Failed to communicate with the flash chip`). Always disconnect the `GPIO 12 (TDI)` JTAG jumper while resetting/flashing over USB-C (`COM7`), or power ESP-Prog properly with `TDI` low/hi-Z during reset.
- **ESP-Prog Windows USB Note**: ESP-Prog uses an FTDI `FT2232HL` (`VID_0403&PID_6010`). It requires a **data-capable** Micro-USB cable (not charge-only). In Windows (via Zadig), **Dual RS232-HS (Interface 0)** must use **WinUSB** for OpenOCD JTAG debugging (hence no COM port for Interface 0!), while **Interface 1** provides the auxiliary UART COM port if FTDI VCP is loaded. `.vscode/launch.json` targets `localhost:3333` with `C:/Espressif/tools/xtensa-esp-elf-gdb/17.1_20260402/xtensa-esp-elf-gdb/bin/xtensa-esp32-elf-gdb.exe`.

---

## 4. Wire Protocols & Cryptographic Contracts

### A. 36-Byte Packed Binary Telemetry Packet (`core/include/telemetry_proto.h`)
All multi-byte fields are **Little-Endian**. `_Static_assert(sizeof(pan_telem_pkt_t) == 36)`.
Matched identically by `cloud/backend/src/decoder.ts`.

| Byte Offset | Type | Field | Scale / Meaning |
| :--- | :--- | :--- | :--- |
| `0` | `uint8` | `magic` | `0xA5` |
| `1` | `uint8` | `msg_type` | `0x01`=FastHUD, `0x02`=GPS/Nav, `0x03`=Health |
| `2..3` | `uint16` | `seq_id` | Rolling packet sequence |
| `4..7` | `uint32` | `utc_epoch` | UTC seconds (`AT+CCLK?` or GPS) |
| `8..11` | `int32` | `lat_e7` | Latitude * `1e7` |
| `12..15` | `int32` | `lon_e7` | Longitude * `1e7` |
| `16..17` | `int16` | `alt_cm` | Altitude AGL in cm |
| `18..19` | `int16` | `roll_cdeg` | Roll * `100` |
| `20..21` | `int16` | `pitch_cdeg` | Pitch * `100` |
| `22..23` | `uint16` | `yaw_cdeg` | Heading * `100` (`0..36000`) |
| `24..25` | `uint16` | `speed_cms` | Ground speed in cm/s |
| `26..27` | `uint16` | `vbat_mv` | Battery voltage in mV |
| `28..29` | `uint16` | `curr_ca` | Current in centi-amps (`10 mA` units) |
| `30` | `uint8` | `flight_mode` | `pan_flight_mode_t` (`0=DISARMED..10=EMERGENCY_KILL`) |
| `31` | `uint8` | `sats_rssi` | High nibble: sats (`0..15`); Low nibble: scaled CSQ (`0..15`) |
| `32` | `uint8` | `active_link` | `1`=CRSF, `2`=Wi-Fi UDP, `3`=LTE MQTT, `4`=SMS |
| `33` | `uint8` | `health_flags` | Bit0=ARMED, Bit1=GPS_LOCK, Bit2=OPTFLOW_OK, Bit3=GEOFENCE_HIT, Bit4=PROP_ANOMALY, Bit5=FAILSAFE_RTH, Bit6=VEHICLE_MODE, Bit7=WOM_ALERT |
| `34..35` | `uint16` | `crc16` | **CRC-16-CCITT** (`poly=0x1021`, `init=0xFFFF`) over bytes `[0..33]` |

### B. HMAC-SHA256 Signed Cloud Command Protocol (`core/src/crypto_auth.c` <-> `cloud/backend/src/crypto.ts`)
- MQTT Topic: `drone/<device_id>/cmd` (or Wi-Fi UDP port `14551`).
- JSON Envelope:
  ```json
  {"cmd":"GOTO_WAYPOINT","ts":1763210000,"nonce":1234,"lat":15.3647000,"lon":75.1240000,"alt":25.00,"mode":0,"hmac":"<hex16_or_hex64>"}
  ```
- **Canonical Signing String**:
  `"<cmd>|<ts>|<nonce>|<lat_e7>|<lon_e7>|<alt_cm>"`
  where `lat_e7 = llround(lat * 1e7)`, `lon_e7 = llround(lon * 1e7)`, `alt_cm = llround(alt * 100.0)`.
- Default shared secret: `"PAN_BRAIN_DEV_KEY_2026"` (`config_manager.c` / `HMAC_SECRET` env var).
- Validation: constant-time hex compare on 8 or 32 bytes HMAC-SHA256, `+-10s` UTC replay window (`CRYPTO_REPLAY_WINDOW_SEC`), 32-entry circular nonce deduplication (`CRYPTO_MAX_NONCE_HISTORY`).
- Supported Commands: `ARM`, `DISARM`, `RTH`, `GOTO_WAYPOINT`, `SET_MODE` (`0`=Drone, `1`=Vehicle), `VEHICLE_SLEEP`, `OTA_MANIFEST`.

---

## 5. ESP-IDF v6.1 Build Quirks & SDK Notes

- **Installed SDK**: ESP-IDF **v6.1** at `D:\esp\v6.1\esp-idf`; tools in `C:\Espressif\tools`; PowerShell activation script: `C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1`.
- **v6.1 Split Component Drivers**: Legacy `driver/i2c.h` and `driver/mcpwm.h` are not used. `main/CMakeLists.txt` requires `esp_driver_gpio`, `esp_driver_uart`, `esp_driver_i2c` (`driver/i2c_master.h`), `esp_driver_spi`, `esp_driver_mcpwm` (`driver/mcpwm_prelude.h`), `esp_driver_sdspi`, `esp_adc` (`esp_adc/adc_oneshot.h`), `esp_psram`, `app_update`, `esp_http_server`.
- **GCC 15 Strict Checks**: Keep string formatting free of `-Werror=format-truncation` (e.g., `bsp_wireless_cfg.c` uses bounded `memcpy` for `wifi_cfg.ap.ssid`/`password`).

---

## 6. How to Build, Run & Flash

### 1) ESP32 Firmware Build & Flash
```powershell
. "C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1"
Set-Location e:\Program\ESP32_Devlopment\drone
idf.py build
idf.py -p COM7 flash monitor   # Disconnect GPIO 12 / TDI JTAG wire while flashing!
```

### 2) PC SITL Simulator
No native Windows C compiler (`gcc`, `clang`, `cl`) was found on this workstation (`esp-clang` targets `riscv32-esp-unknown-elf` by default). On any machine with CMake + MinGW-w64 / MSVC / GCC:
```bash
cmake -S bsp/pc_sitl -B build_sitl
cmake --build build_sitl
./build_sitl/pan_sitl   # Streams 36B telemetry to UDP 127.0.0.1:14550
```

### 3) Cloud GCS Backend & Frontend
- **Full Docker stack** (Mosquitto + TimescaleDB + Backend + Frontend):
  `cd cloud && docker compose up --build`
- **Local Dev (Windows host with Node.js)**:
  - Terminal 1 (Backend HTTP `:8080`, WS `/ws/telemetry`, UDP `:14550/:14551`, built-in SITL simulator fallback):
    `cd cloud/backend && npm install && npm run dev`
  - Terminal 2 (Frontend Vite dev UI on `:5173`):
    `cd cloud/frontend && npm install && npm run dev`

---

## 7. Verified Verification Log & Current Hardware State (2026-11-15)

- **GitHub Repository**: `https://github.com/shrinathpol/Drone-Development.git` (`main` branch).
- **Current Connected Hardware (`COM7` Bench Setup)**:
  - **Connected / Populated**: `LilyGO T-SIM7000G` (`ESP32-D0WD-V3 rev v3.0`, 4MB SPI Flash, 8MB PSRAM), `SIM7000G` cellular modem + internal GNSS on UART1 (`TX=27, RX=26, PWRKEY=4, DTR=25`), `MicroSD Card` on HSPI/SPI2 (`/sdcard`), `Wi-Fi` SoftAP/UDP (`PAN_BRAIN_CFG`), `BLE`, and onboard `GPIO 12` active-low status LED.
  - **Not Connected Yet (Unpopulated External Sensors)**: External I2C IMU (`ICM-42688-P`/`MPU6050`/`BMI270`), Barometer (`BMP390`), Compass (`QMC5883L`), ToF (`VL53L1X`), Current/Power monitor (`INA226`), Optical Flow (`PMW3901`), Brushless ESCs/Motors, and CRSF/SBUS RX. `bsp_i2c.c` (`i2c_master_probe` + 16-entry NACK cache) and `bsp_mcpwm.c` (skip when pins `< 0`) safely bypass absent sensors without stalling.
- **ESP-IDF v6.1 Firmware Build (`idf.py build`)**: **PASSED (exit 0)** -> generated `build/pan_brain_firmware.bin` (`0x105220` bytes; `37%` free in `ota_0`) & `bootloader/bootloader.bin` (`0x6660` bytes).
- **Zero-MCU Header Isolation (`drivers/`, `core/`, `app/`)**: **PASSED** (`ISOLATION_OK` & `APP_ISOLATION_OK`).
- **Cloud Backend (`cloud/backend` `npm run build`)**: **PASSED (exit 0)** + Node smoke test verified 36-byte packed encode/decode (`LEN= 36`) and 64-char HMAC-SHA256 signing (`HMAC_LEN= 64`).
- **Cloud Frontend (`cloud/frontend` `tsc && vite build`)**: **PASSED (exit 0)** (1516 modules bundled into `dist/`).
- **`COM7` Hardware Flash & Live Serial Verification**:
  - Removing external JTAG `TDI` from `GPIO 12 (MTDI)` resolved the `VDD_SDIO` 1.8V strapping brownout; `idf.py -p COM7 flash` **PASSED (exit 0)** on MAC `34:94:54:f5:2e:54`.
  - Serial capture at `115200` baud confirmed: 8MB PSRAM initialized (`SPI SRAM memory test OK`), `/sdcard` mounted (`I (2583) bsp_sdcard_adc_ota: SD card mounted at /sdcard`), `pan_brain_init()` step logs (`[1/12] Storage mounted=1 | id=drone_brain_01...`), and live 1 Hz FreeRTOS `STATUS | mode=0 armed=0 led_pat=2 led_on=0 link=2 modem_st=4 ...` telemetry loop logs.
- **Follow-Up Boot & Modem Fixes Applied (`idf.py -p COM7 flash` rebuild in progress at push time)**:
  - `core/src/offline_queue.c`: Deferred `stat("/sdcard/offline_log.bin")` out of boot init (`q->sd_has_backlog = false`) to avoid ~60s FATFS root cluster scanning when `/sdcard/offline_log.bin` does not exist yet.
  - `drivers/src/modem_sim7000g.c`: Added `FSM` state transition logs (`transition_state`) and allowed `AT+CPIN?` errors (`SIM not inserted`) after 2 tries to advance `MODEM_STATE_INIT_CFG` toward `MODEM_STATE_ENABLE_GNSS` (`AT+CGNSPWR=1`).
  - `drivers/src/env_sensors.c`: Report raw positive `ADC1` battery voltage readings (`mv > 0`) for USB-only bench debugging while gating `power.valid` at `mv > 2500`.
  - `sdkconfig.defaults` / `sdkconfig`: Raised `CONFIG_ESP_MAIN_TASK_STACK_SIZE` from `3584` to `8192`.


