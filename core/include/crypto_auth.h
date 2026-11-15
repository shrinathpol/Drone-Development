#ifndef CRYPTO_AUTH_H
#define CRYPTO_AUTH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRYPTO_SHA256_LEN        32U
#define CRYPTO_MAX_NONCE_HISTORY 32U
#define CRYPTO_REPLAY_WINDOW_SEC 10U

typedef enum {
    CMD_NONE = 0,
    CMD_ARM,
    CMD_DISARM,
    CMD_RTH,
    CMD_GOTO_WAYPOINT,
    CMD_SET_MODE,
    CMD_VEHICLE_SLEEP,
    CMD_OTA_MANIFEST
} pan_cloud_cmd_type_t;

typedef struct {
    pan_cloud_cmd_type_t type;
    uint32_t utc_timestamp;
    uint32_t nonce;
    double target_lat;
    double target_lon;
    float target_alt_m;
    uint8_t mode_arg;
    char ota_sha256_hex[65];
    uint32_t ota_size_bytes;
} pan_cloud_cmd_t;

typedef struct {
    uint8_t secret_key[64];
    size_t secret_key_len;
    uint32_t seen_nonces[CRYPTO_MAX_NONCE_HISTORY];
    size_t nonce_head;
    bool allow_unsynced_clock;
} crypto_auth_ctx_t;

void crypto_sha256(const uint8_t *data, size_t len, uint8_t out_hash[CRYPTO_SHA256_LEN]);

void crypto_hmac_sha256(const uint8_t *key,
                        size_t key_len,
                        const uint8_t *msg,
                        size_t msg_len,
                        uint8_t out_mac[CRYPTO_SHA256_LEN]);

void crypto_auth_init(crypto_auth_ctx_t *ctx, const char *secret_key);

/**
 * @brief Parse and verify an authenticated JSON command payload:
 *        {"cmd":"GOTO_WAYPOINT","ts":1763210000,"nonce":1234,"lat":15.36,"lon":75.12,"alt":35,"hmac":"<hex16_or_hex64>"}
 *
 * Canonical signing string: "<cmd>|<ts>|<nonce>|<lat_e7>|<lon_e7>|<alt_cm>"
 * Validates UTC anti-replay window (+-10s), rolling nonce deduplication, and HMAC-SHA256.
 */
bool crypto_verify_and_parse_cmd(crypto_auth_ctx_t *ctx,
                                 const char *json_payload,
                                 size_t payload_len,
                                 uint32_t current_utc_epoch,
                                 pan_cloud_cmd_t *out_cmd);

#ifdef __cplusplus
}
#endif

#endif /* CRYPTO_AUTH_H */
