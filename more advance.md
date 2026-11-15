To push the **Pan-Brain** architecture from a DIY autopilot into an industrial, commercial-grade flight and telemetry controller, add these **8 advanced firmware, navigation, and security subsystems** into the `core/` and `drivers/` layers—while keeping the exact same zero-MCU-dependency rule so everything stays 100% portable.

---

## 1. 15-State Extended Kalman Filter (EKF) + Dead-Reckoning (GPS-Denied Flight)

Simple Mahony/Complementary filters work for attitude stabilization, but if your drone flies under a bridge, inside a warehouse, or gets hit by GPS jamming/multipath errors, a basic PID loop will drift away.

* **Upgrade (`core/ekf_nav.c`):** Replace the separate attitude and altitude filters with a unified **15-State Extended Kalman Filter** estimating:

$$\text{State Vector } \mathbf{x} = [\text{Roll, Pitch, Yaw}, V_N, V_E, V_D, P_N, P_E, P_D, b_{gx}, b_{gy}, b_{gz}, b_{ax}, b_{ay}, b_{az}]^T$$

* **Optical Flow + ToF Fusion (`drivers/optflow_pmw3901.c`):** Add a tiny **PMW3901 / PAA5100 Optical Flow sensor** (SPI/UART, weighs 2 grams) pointing downward alongside your VL53L1X laser rangefinder.
* When GPS HDOP degrades (`> 2.5`) or satellites drop below 6, the EKF automatically switches velocity/position fusion from GPS to **Optical Flow + IMU Dead-Reckoning**, allowing the drone to hold position indoors or return along its last buffered vector without GPS.

---

## 2. Dynamic FFT Gyro Notch Filtering & ESC RPM Telemetry (Bidirectional DShot)

Drone frames vibrate at specific frequencies depending on motor RPM (`100Hz – 400Hz`). A static low-pass filter adds phase lag, which reduces PID responsiveness and causes oscillations in wind.

* **Bidirectional DShot / ESC Telemetry (`drivers/motor_esc.c`):** Upgrade the PWM/DShot driver to read back **eRPM, ESC Temperature, Voltage, and Current** from BLHeli_32 / AM32 ESCs.
* **Harmonic Notch Filter (`core/dsp_filters.c`):** Use the exact motor RPM (or run a lightweight 64-point Radix-2 FFT on the gyro Z-axis using the ESP32-WROVER PSRAM) to dynamically slide 3 narrow notch filters directly over each motor's fundamental vibration frequency and its 2nd harmonic.
* **Motor Health / Prop-Strike Detection:** By comparing commanded throttle vs. actual RPM + IMU vibration spikes in real time, the brain can detect a cracked propeller or failing motor bearing mid-flight and alert the cloud dashboard immediately.

---

## 3. Binary Compact Telemetry Protocol + Authenticated Command Signing

Sending raw JSON strings (`{"lat":15.3647,"lon":75.1240...}`) over cellular NB-IoT / Cat-M1 wastes bandwidth, increases latency, and drains your data plan. More importantly, plain-text MQTT commands can be spoofed if someone guesses your topic.

* **Custom Binary Frame (`core/telemetry_proto.h`):** Pack all telemetry into a fixed **48-byte packed C struct** (or **Nanopb / MAVLink v2** frame) with a **CRC-16-CCITT** checksum:

```c
typedef struct __attribute__((packed)) {
    uint8_t  magic;          // 0xA5
    uint8_t  msg_type;       // 0x01 = Fast HUD, 0x02 = GPS/Nav, 0x03 = Health
    uint16_t seq_id;
    uint32_t utc_epoch;      // GSM/GPS synced epoch timestamp
    int32_t  lat_e7;         // Lat * 1e7 (4 bytes instead of 8-byte double, 1.1cm precision)
    int32_t  lon_e7;         // Lon * 1e7
    int16_t  alt_cm;         // Altitude in cm
    int16_t  roll_cdeg;      // Roll * 100
    int16_t  pitch_cdeg;     // Pitch * 100
    uint16_t yaw_cdeg;       // Yaw * 100
    uint16_t speed_cms;      // Ground speed cm/s
    uint16_t vbat_mv;        // Battery in millivolts
    uint16_t curr_ca;        // Current in centi-amps
    uint8_t  flight_mode;    // Armed/Loiter/RTH/Mission/Vehicle-Sleep
    uint8_t  sats_rssi;      // Upper 4 bits: Sats, Lower 4 bits: Cellular CSQ
    uint16_t crc16;
} pan_telem_pkt_t;           // Total: 36 bytes!

```

* **HMAC-SHA256 + Anti-Replay Command Security (`core/crypto_auth.c`):** Every command sent from your Google Cloud server (`ARM`, `DISARM`, `GOTO_WAYPOINT`) includes a 4-byte `utc_timestamp`, a `nonce`, and a truncated **HMAC-SHA256 signature**. If the timestamp is older than 10 seconds (replay attack) or the signature fails, the drone rejects the command.

---

## 4. Dual-Bank Atomic OTA Firmware Updates (BLE, Wi-Fi & Cellular LTE)

Since you already work with bootloader & OTA architectures, build a **transport-agnostic OTA Engine (`core/ota_manager.c`)** abstracted via `hal_flash_t` in `hal_interfaces.h`:

* **Delta / Chunked Resume over SIM7000G:** Cellular connections drop frequently. Instead of downloading a 1MB `.bin` in one shot, the cloud server slices the firmware into **4KB chunks**, stores them in `/sdcard/fw_update.bin`, verifies the **SHA-256 hash** of the complete file on the SD card first, and only flashes the internal MCU secondary slot (`ota_1`) when 100% verified and the drone is **DISARMED** on the ground.
* **Auto-Rollback Watchdog:** On first boot after an OTA update, the new firmware must pass a self-test (IMU WHO_AM_I check, Baro check, SIM7000G AT sync) within 15 seconds and set a `boot_verified` flag; otherwise, the hardware watchdog triggers a reset and the bootloader rolls back to the previous working slot.

---

## 5. 3D Polygonal Geofencing & Autonomous Mission Engine

Instead of just basic Return-to-Home, give the `core/mission_engine.c` full autonomous capability stored on the SD card:

* **Keep-In / Keep-Out Polygonal Geofences:** Store an array of GPS vertices (`lat_e7, lon_e7`) and a max altitude ceiling (`e.g., 120m AGL`). Run a **Ray-Casting Point-in-Polygon algorithm** at 10Hz. If the pilot or wind pushes the drone toward a restricted zone (like an airport or boundary wall), the brain overrides manual stick inputs and applies a "virtual rubber bumper" braking vector.
* **Smart Wind & Battery-Aware RTH ("Smart RTL"):**
* Traditional RTH triggers at a fixed voltage (e.g., `3.5V/cell`), which fails if the drone is 3 km away fighting a headwind.
* **Advanced Upgrade:** Continuously compute **ground-speed vs. airspeed/tilt angle** to estimate wind velocity, calculate the exact energy (`Watt-hours` / `mAh`) required to fly the distance back to `HOME_POS` against that wind vector plus a 20% safety reserve, and trigger RTH dynamically the moment remaining battery crosses that calculated return threshold.

* **Breadcrumb Path Backtracking:** Instead of flying a straight line home (which might crash into a building or tree you flew *around* on the way out), continuously record a simplified 3D breadcrumb trail in PSRAM (`Douglas-Peucker path simplification`) and retrace the exact safe corridor backward during RTH before descending.

---

## 6. Multi-Link Redundant Telemetry Router (MAVLink / Custom Mesh)

Right now you have RC Radio (CRSF) and Cellular (SIM7000G). Add a **Link Priority & Failover Router (`core/link_router.c`)** that bonds all communication links seamlessly:

1. **Link 1 (Sub-GHz CRSF / ELRS Telemetry - 5ms latency):** Primary real-time stick control & basic HUD back to the handheld radio.
2. **Link 2 (Wi-Fi UDP / ESP-NOW Long-Range Mode - 15ms latency):** Automatically active when within 300m of the ground station or phone for high-rate (50Hz) PID tuning and live blackbox streaming.
3. **Link 3 (SIM7000G 4G LTE / NB-IoT MQTT - 150ms latency):** Unlimited range BVLOS (Beyond Visual Line of Sight) tracking and waypoint control via Google Cloud.
4. **Link 4 (SIM7000G GSM SMS - Last Resort):** Emergency coordinate beacon and kill/find command when data bearers fail completely.

---

## 7. Hardware-in-the-Loop (HITL) / Software-in-the-Loop (SITL) PC Simulator

Because your `drivers/`, `core/`, and `app/` layers only depend on `hal_interfaces.h`, you get a superpower that standard Arduino/ESP-IDF projects don't have: **you can compile your exact drone firmware as a native `.exe` on your Windows PC using CMake + MinGW/MSVC!**

* Create a folder `bsp/pc_sitl/` that maps `hal_uart_t` to TCP/UDP sockets and feeds synthetic physics data (simulated gravity, motor thrust, wind, GPS drift, and simulated cellular drops) into your `imu_driver_t` and `sim7000g_dev_t`.
* Connect your local PC build directly to your **Google Cloud MQTT server and Web Dashboard** to test RTH failsafes, geofence breaches, and waypoint missions on the map before spinning a single real propeller.

---

## 8. Prompt Addendum for Antigravity (Copy-Paste to Upgrade the Spec)

Append this block to your **Antigravity Master Spec** so the coding agent builds these advanced modules into your architecture:

```markdown
## ADVANCED SUBSYSTEM REQUIREMENTS (PHASE 8+)
1. **Binary Telemetry & Security (`core/telemetry_proto.c`, `core/crypto_auth.c`):**
   - Implement a packed 36-byte binary telemetry struct (`pan_telem_pkt_t`) with CRC-16-CCITT instead of verbose JSON strings for cellular efficiency.
   - Implement HMAC-SHA256 + UTC Epoch anti-replay verification on all incoming cloud control commands (`ARM`, `DISARM`, `GOTO_WAYPOINT`, `RTH`).
2. **15-State EKF & Optical Flow Fallback (`core/ekf_nav.c`, `drivers/optflow_pmw3901.c`):**
   - Fuse 6-DOF IMU, 3-axis Mag, Barometer, ToF Laser, GPS, and PMW3901 Optical Flow so the drone maintains position hold (Dead-Reckoning) even if GPS lock is lost.
3. **Smart Wind-Aware RTH & Polygonal Geofencing (`core/mission_engine.c`):**
   - Implement Ray-Casting Point-in-Polygon geofence enforcement.
   - Implement Breadcrumb Backtracking RTH (retracing safe flight corridor) and dynamic battery energy-to-home estimation using INA226 mAh draw and groundspeed vs. pitch wind estimation.
4. **PC SITL Simulator BSP (`bsp/pc_sitl/`):**
   - Implement `hal_interfaces.h` for native Windows/Linux CMake compilation using UDP sockets and a 6-DOF quadcopter physics model so the entire `core/` flight controller and `cloud/` dashboard can be tested in simulation without hardware.

```

Which of these advanced modules would you like to include in our first code generation pass, or should we start writing the **Phase 1 HAL + SIM7000G + Binary Telemetry** code right now?
