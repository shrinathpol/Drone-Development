# CLAUDE.md — Project Guidelines & AI Context

Read [`AI_MEMORY.md`](AI_MEMORY.md) before making changes to this repository.

## Non-Negotiable Rules
1. **Maintain Strict Layer Isolation**:
   - NEVER `#include` any `esp_*`, `driver/*`, `freertos/*`, `sdkconfig.h`, or `Arduino.h` header inside `core/`, `drivers/`, or `app/`.
   - All hardware access inside `drivers/` and `core/` MUST go through `bsp/include/hal_interfaces.h`.
   - Keep `include/` (headers) and `src/` (sources) strictly separated in `core/`, `drivers/`, `app/`, `bsp/esp32_wrover/`, and `bsp/pc_sitl/`.
2. **LilyGO T-SIM7000G Hardware Safety**:
   - NEVER use **GPIO 16 / 17** (wired to 8MB PSRAM) or **GPIO 6..11** (SPI Flash).
   - **GPIO 12** is the active-low onboard blue LED (`PIN_ONBOARD_LED`) AND ESP32 JTAG `MTDI` (`VDD_SDIO` flash voltage strap). Keep the JTAG pin guard (`bsp_is_jtag_pin_guarded()`) intact across `bsp_gpio_sys.c` and `bsp_spi.c`.
   - Motor outputs (`GPIO 18, 19, 23, 5`) must remain disarmed unless `motor_esc_t.armed` is explicitly enabled after preflight checks.
3. **Wire Format Parity**:
   - `core/include/telemetry_proto.h` (`pan_telem_pkt_t`, exact 36 bytes Little-Endian, CRC-16-CCITT `0x1021`/`0xFFFF`) must always match `cloud/backend/src/decoder.ts`.
   - `core/src/crypto_auth.c` (`"<cmd>|<ts>|<nonce>|<lat_e7>|<lon_e7>|<alt_cm>"`) must always match `cloud/backend/src/crypto.ts`.
4. **ESP-IDF SDK Path & Repo**:
   - Build with ESP-IDF v6.1 (`D:\esp\v6.1\esp-idf`) initialized via `C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1`.
   - Upstream repo: `https://github.com/shrinathpol/Drone-Development.git` (`main`).
5. **Current Hardware Scope (Bench Bring-Up)**:
   - Only `LilyGO T-SIM7000G` onboard modules (`ESP32`, `SIM7000G` modem + GPS, `SD Card`, `Wi-Fi`, `BLE`, `GPIO 12` LED) are currently attached; keep external I2C/SPI/PWM probes non-blocking (`bsp_i2c` NACK cache, `bsp_mcpwm` pin `< 0` skip).


