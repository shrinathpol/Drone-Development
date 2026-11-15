#include "offline_queue.h"

#include <stdlib.h>
#include <string.h>

int offline_queue_init(offline_queue_t *q, hal_sys_t *sys, hal_storage_t *storage)
{
    if (q == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(q, 0, sizeof(*q));
    q->sys = sys;
    q->storage = storage;
    q->capacity = OFFLINE_QUEUE_PSRAM_CAPACITY;

    size_t bytes = q->capacity * sizeof(pan_telem_pkt_t);
    if (sys->alloc_psram != NULL) {
        q->ring = (pan_telem_pkt_t *)sys->alloc_psram(bytes);
    } else {
        q->ring = (pan_telem_pkt_t *)malloc(bytes);
    }

    if (q->ring == NULL) {
        /* Fallback small internal RAM ring if PSRAM unavailable */
        q->capacity = 128U;
        q->ring = (pan_telem_pkt_t *)malloc(q->capacity * sizeof(pan_telem_pkt_t));
        if (q->ring == NULL) {
            return HAL_ERR_NO_MEM;
        }
    }

    /* Defer checking /sdcard/offline_log.bin to background runtime rather than stalling
     * pan_brain_init() on a full FATFS LFN cluster scan when the file does not exist yet. */
    q->sd_has_backlog = false;
    return HAL_OK;
}

void offline_queue_push(offline_queue_t *q, const pan_telem_pkt_t *pkt, bool lte_online)
{
    if (q == NULL || q->ring == NULL || pkt == NULL) {
        return;
    }

    /* If buffer is nearly full and offline, flush oldest batch to SD card */
    if (!lte_online && q->count >= (q->capacity - 16U)) {
        offline_queue_flush_to_sd(q, 64U);
    }

    if (q->count == q->capacity) {
        /* Overwrite oldest packet */
        q->tail = (q->tail + 1U) % q->capacity;
        q->count--;
        q->dropped_count++;
    }

    q->ring[q->head] = *pkt;
    q->head = (q->head + 1U) % q->capacity;
    q->count++;
}

void offline_queue_flush_to_sd(offline_queue_t *q, size_t max_batch)
{
    if (q == NULL || q->storage == NULL || q->storage->append_file == NULL || q->count == 0U) {
        return;
    }

    pan_telem_pkt_t batch[32];
    size_t to_write = (q->count < max_batch) ? q->count : max_batch;
    if (to_write > 32U) {
        to_write = 32U;
    }

    for (size_t i = 0U; i < to_write; i++) {
        batch[i] = q->ring[q->tail];
        q->tail = (q->tail + 1U) % q->capacity;
        q->count--;
    }

    if (q->storage->append_file(OFFLINE_SD_LOG_PATH, batch, to_write * sizeof(pan_telem_pkt_t)) == HAL_OK) {
        q->sd_flushed_count += (uint32_t)to_write;
        q->sd_has_backlog = true;
    }
}

bool offline_queue_pop(offline_queue_t *q, pan_telem_pkt_t *out_pkt)
{
    if (q == NULL || out_pkt == NULL) {
        return false;
    }

    if (q->count > 0U && q->ring != NULL) {
        *out_pkt = q->ring[q->tail];
        q->tail = (q->tail + 1U) % q->capacity;
        q->count--;
        return true;
    }

    if (q->sd_has_backlog && q->storage != NULL && q->storage->read_file != NULL) {
        uint8_t raw[sizeof(pan_telem_pkt_t)];
        int n = q->storage->read_file(OFFLINE_SD_LOG_PATH, raw, sizeof(raw), q->sd_read_offset);
        if (n == (int)sizeof(pan_telem_pkt_t)) {
            q->sd_read_offset += sizeof(pan_telem_pkt_t);
            if (telemetry_verify_pkt(raw, sizeof(raw), out_pkt)) {
                return true;
            }
        } else {
            /* Finished replaying SD backlog */
            q->sd_has_backlog = false;
            q->sd_read_offset = 0U;
            if (q->storage->delete_file != NULL) {
                q->storage->delete_file(OFFLINE_SD_LOG_PATH);
            }
        }
    }

    return false;
}

size_t offline_queue_count(const offline_queue_t *q)
{
    return (q != NULL) ? q->count : 0U;
}
