#ifndef OFFLINE_QUEUE_H
#define OFFLINE_QUEUE_H

#include "hal_interfaces.h"
#include "telemetry_proto.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OFFLINE_QUEUE_PSRAM_CAPACITY 2048U
#define OFFLINE_SD_LOG_PATH          "/sdcard/offline_log.bin"

typedef struct {
    hal_sys_t *sys;
    hal_storage_t *storage;
    pan_telem_pkt_t *ring;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
    size_t sd_read_offset;
    uint32_t dropped_count;
    uint32_t sd_flushed_count;
    bool sd_has_backlog;
} offline_queue_t;

int offline_queue_init(offline_queue_t *q, hal_sys_t *sys, hal_storage_t *storage);
void offline_queue_push(offline_queue_t *q, const pan_telem_pkt_t *pkt, bool lte_online);
bool offline_queue_pop(offline_queue_t *q, pan_telem_pkt_t *out_pkt);
void offline_queue_flush_to_sd(offline_queue_t *q, size_t max_batch);
size_t offline_queue_count(const offline_queue_t *q);

#ifdef __cplusplus
}
#endif

#endif /* OFFLINE_QUEUE_H */
