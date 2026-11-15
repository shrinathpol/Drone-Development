#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "hal_interfaces.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OTA_STAGING_PATH "/sdcard/fw_update.bin"
#define OTA_SELF_TEST_WINDOW_MS 15000U

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_STAGING_CHUNKS,
    OTA_STATE_VERIFYING_SHA256,
    OTA_STATE_FLASHING_BANK,
    OTA_STATE_READY_REBOOT,
    OTA_STATE_FAILED
} ota_state_t;

typedef struct {
    hal_sys_t *sys;
    hal_storage_t *storage;
    hal_flash_t *flash;

    ota_state_t state;
    uint32_t expected_size;
    uint32_t written_bytes;
    char expected_sha256_hex[65];

    bool boot_pending_verify;
    bool boot_confirmed;
    uint32_t boot_ms;
} ota_manager_t;

void ota_manager_init(ota_manager_t *ota, hal_sys_t *sys, hal_storage_t *storage, hal_flash_t *flash);

int ota_manager_begin_manifest(ota_manager_t *ota, uint32_t total_size, const char *sha256_hex);

int ota_manager_write_chunk(ota_manager_t *ota, const uint8_t *chunk, size_t len);

int ota_manager_commit_if_disarmed(ota_manager_t *ota, bool armed);

/**
 * @brief Called on every boot cycle: validates IMU, Baro, and SIM7000G health within 15s,
 *        marking OTA slot valid or triggering hardware rollback.
 */
void ota_poll_boot_verification(ota_manager_t *ota, bool imu_ok, bool modem_synced);

#ifdef __cplusplus
}
#endif

#endif /* OTA_MANAGER_H */

