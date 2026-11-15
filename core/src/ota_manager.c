#include "ota_manager.h"
#include "crypto_auth.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ota_manager_init(ota_manager_t *ota, hal_sys_t *sys, hal_storage_t *storage, hal_flash_t *flash)
{
    if (ota == NULL) {
        return;
    }
    memset(ota, 0, sizeof(*ota));
    ota->sys = sys;
    ota->storage = storage;
    ota->flash = flash;
    ota->boot_ms = (sys != NULL) ? sys->millis() : 0U;

    if (flash != NULL && flash->ota_is_pending_verify != NULL) {
        ota->boot_pending_verify = flash->ota_is_pending_verify();
    }
}

int ota_manager_begin_manifest(ota_manager_t *ota, uint32_t total_size, const char *sha256_hex)
{
    if (ota == NULL || total_size == 0U || sha256_hex == NULL || strlen(sha256_hex) < 64U) {
        return HAL_ERR_INVALID_ARG;
    }
    if (ota->storage != NULL && ota->storage->delete_file != NULL) {
        ota->storage->delete_file(OTA_STAGING_PATH);
    }
    ota->expected_size = total_size;
    ota->written_bytes = 0U;
    snprintf(ota->expected_sha256_hex, sizeof(ota->expected_sha256_hex), "%s", sha256_hex);
    ota->state = OTA_STATE_STAGING_CHUNKS;
    return HAL_OK;
}

int ota_manager_write_chunk(ota_manager_t *ota, const uint8_t *chunk, size_t len)
{
    if (ota == NULL || ota->state != OTA_STATE_STAGING_CHUNKS || chunk == NULL || len == 0U) {
        return HAL_ERR_INVALID_ARG;
    }
    if (ota->storage == NULL || ota->storage->append_file == NULL) {
        return HAL_ERR_NOT_READY;
    }
    if (ota->storage->append_file(OTA_STAGING_PATH, chunk, len) != HAL_OK) {
        ota->state = OTA_STATE_FAILED;
        return HAL_ERR_IO;
    }
    ota->written_bytes += (uint32_t)len;
    return HAL_OK;
}

int ota_manager_commit_if_disarmed(ota_manager_t *ota, bool armed)
{
    if (ota == NULL || armed) {
        return HAL_ERR_NOT_READY;
    }
    if (ota->storage == NULL || ota->flash == NULL || ota->written_bytes != ota->expected_size) {
        ota->state = OTA_STATE_FAILED;
        return HAL_ERR_INVALID_ARG;
    }

    ota->state = OTA_STATE_VERIFYING_SHA256;
    uint8_t *img = NULL;
    if (ota->sys != NULL && ota->sys->alloc_psram != NULL) {
        img = (uint8_t *)ota->sys->alloc_psram(ota->expected_size);
    } else {
        img = (uint8_t *)malloc(ota->expected_size);
    }
    if (img == NULL) {
        ota->state = OTA_STATE_FAILED;
        return HAL_ERR_NO_MEM;
    }

    size_t offset = 0U;
    while (offset < ota->expected_size) {
        size_t chunk = ota->expected_size - offset;
        if (chunk > 4096U) chunk = 4096U;
        int got = ota->storage->read_file(OTA_STAGING_PATH, img + offset, chunk, offset);
        if (got <= 0) {
            break;
        }
        offset += (size_t)got;
    }

    if (offset != ota->expected_size) {
        if (ota->sys != NULL && ota->sys->free_psram != NULL) ota->sys->free_psram(img);
        else free(img);
        ota->state = OTA_STATE_FAILED;
        return HAL_ERR_IO;
    }

    uint8_t digest[CRYPTO_SHA256_LEN];
    crypto_sha256(img, ota->expected_size, digest);
    char actual_hex[65];
    for (size_t i = 0U; i < CRYPTO_SHA256_LEN; i++) {
        snprintf(&actual_hex[i * 2U], 3U, "%02x", digest[i]);
    }
    for (size_t i = 0U; i < 64U; i++) {
        if ((char)tolower((unsigned char)ota->expected_sha256_hex[i]) != actual_hex[i]) {
            if (ota->sys != NULL && ota->sys->free_psram != NULL) ota->sys->free_psram(img);
            else free(img);
            ota->state = OTA_STATE_FAILED;
            return HAL_ERR_INVALID_ARG;
        }
    }

    ota->state = OTA_STATE_FLASHING_BANK;
    int rc = ota->flash->ota_begin(ota->expected_size);
    if (rc == HAL_OK) {
        rc = ota->flash->ota_write(img, ota->expected_size);
    }
    if (rc == HAL_OK) {
        rc = ota->flash->ota_end();
    }
    if (rc == HAL_OK) {
        rc = ota->flash->ota_set_boot_next();
    }

    if (ota->sys != NULL && ota->sys->free_psram != NULL) ota->sys->free_psram(img);
    else free(img);

    if (rc != HAL_OK) {
        ota->flash->ota_abort();
        ota->state = OTA_STATE_FAILED;
        return rc;
    }

    ota->state = OTA_STATE_READY_REBOOT;
    if (ota->sys != NULL && ota->sys->reboot != NULL) {
        ota->sys->reboot();
    }
    return HAL_OK;
}

void ota_poll_boot_verification(ota_manager_t *ota, bool imu_ok, bool modem_synced)
{
    if (ota == NULL || !ota->boot_pending_verify || ota->boot_confirmed || ota->flash == NULL) {
        return;
    }
    uint32_t now = (ota->sys != NULL) ? ota->sys->millis() : 0U;
    if (imu_ok && modem_synced) {
        if (ota->flash->ota_mark_app_valid() == HAL_OK) {
            ota->boot_confirmed = true;
            ota->boot_pending_verify = false;
        }
    } else if ((now - ota->boot_ms) > OTA_SELF_TEST_WINDOW_MS) {
        /* Self-test window expired -> trigger bootloader rollback to previous good slot */
        ota->flash->ota_trigger_rollback();
    }
}

