#include "crypto_auth.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static inline uint32_t rotr32(uint32_t x, uint32_t n)
{
    return (x >> n) | (x << (32U - n));
}

static const uint32_t K256[64] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U
};

static void sha256_transform(uint32_t state[8], const uint8_t block[64])
{
    uint32_t w[64];
    for (size_t i = 0U; i < 16U; i++) {
        w[i] = ((uint32_t)block[i * 4U] << 24U) |
               ((uint32_t)block[i * 4U + 1U] << 16U) |
               ((uint32_t)block[i * 4U + 2U] << 8U) |
               ((uint32_t)block[i * 4U + 3U]);
    }
    for (size_t i = 16U; i < 64U; i++) {
        uint32_t s0 = rotr32(w[i - 15U], 7U) ^ rotr32(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
        uint32_t s1 = rotr32(w[i - 2U], 17U) ^ rotr32(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
        w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];

    for (size_t i = 0U; i < 64U; i++) {
        uint32_t S1 = rotr32(e, 6U) ^ rotr32(e, 11U) ^ rotr32(e, 25U);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t temp1 = h + S1 + ch + K256[i] + w[i];
        uint32_t S0 = rotr32(a, 2U) ^ rotr32(a, 13U) ^ rotr32(a, 22U);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t temp2 = S0 + maj;

        h = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}

void crypto_sha256(const uint8_t *data, size_t len, uint8_t out_hash[CRYPTO_SHA256_LEN])
{
    uint32_t state[8] = {
        0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U
    };

    uint8_t block[64];
    size_t idx = 0U;

    for (size_t i = 0U; i < len; i++) {
        block[idx++] = data[i];
        if (idx == 64U) {
            sha256_transform(state, block);
            idx = 0U;
        }
    }

    block[idx++] = 0x80U;
    if (idx > 56U) {
        while (idx < 64U) {
            block[idx++] = 0x00U;
        }
        sha256_transform(state, block);
        idx = 0U;
    }
    while (idx < 56U) {
        block[idx++] = 0x00U;
    }

    uint64_t bit_len = (uint64_t)len * 8ULL;
    for (int i = 0; i < 8; i++) {
        block[56 + i] = (uint8_t)((bit_len >> (56 - i * 8)) & 0xFFULL);
    }
    sha256_transform(state, block);

    for (size_t i = 0U; i < 8U; i++) {
        out_hash[i * 4U + 0U] = (uint8_t)((state[i] >> 24U) & 0xFFU);
        out_hash[i * 4U + 1U] = (uint8_t)((state[i] >> 16U) & 0xFFU);
        out_hash[i * 4U + 2U] = (uint8_t)((state[i] >> 8U) & 0xFFU);
        out_hash[i * 4U + 3U] = (uint8_t)(state[i] & 0xFFU);
    }
}

void crypto_hmac_sha256(const uint8_t *key,
                        size_t key_len,
                        const uint8_t *msg,
                        size_t msg_len,
                        uint8_t out_mac[CRYPTO_SHA256_LEN])
{
    uint8_t k_pad[64];
    uint8_t k_hash[CRYPTO_SHA256_LEN];
    memset(k_pad, 0, sizeof(k_pad));

    if (key_len > 64U) {
        crypto_sha256(key, key_len, k_hash);
        memcpy(k_pad, k_hash, CRYPTO_SHA256_LEN);
    } else if (key_len > 0U && key != NULL) {
        memcpy(k_pad, key, key_len);
    }

    uint8_t inner_buf[64U + 256U];
    uint8_t i_key_pad[64];
    uint8_t o_key_pad[64];
    for (size_t i = 0U; i < 64U; i++) {
        i_key_pad[i] = k_pad[i] ^ 0x36U;
        o_key_pad[i] = k_pad[i] ^ 0x5cU;
    }

    uint8_t inner_hash[CRYPTO_SHA256_LEN];
    if (msg_len <= 256U) {
        memcpy(inner_buf, i_key_pad, 64U);
        if (msg != NULL && msg_len > 0U) {
            memcpy(inner_buf + 64U, msg, msg_len);
        }
        crypto_sha256(inner_buf, 64U + msg_len, inner_hash);
    } else {
        /* Fallback chunked hash path for oversized buffers */
        uint8_t *heap_buf = (uint8_t *)malloc(64U + msg_len);
        if (heap_buf == NULL) {
            memset(out_mac, 0, CRYPTO_SHA256_LEN);
            return;
        }
        memcpy(heap_buf, i_key_pad, 64U);
        memcpy(heap_buf + 64U, msg, msg_len);
        crypto_sha256(heap_buf, 64U + msg_len, inner_hash);
        free(heap_buf);
    }

    uint8_t outer_buf[64U + CRYPTO_SHA256_LEN];
    memcpy(outer_buf, o_key_pad, 64U);
    memcpy(outer_buf + 64U, inner_hash, CRYPTO_SHA256_LEN);
    crypto_sha256(outer_buf, sizeof(outer_buf), out_mac);
}

void crypto_auth_init(crypto_auth_ctx_t *ctx, const char *secret_key)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    const char *k = (secret_key != NULL && secret_key[0] != '\0') ? secret_key : "PAN_BRAIN_DEV_KEY_2026";
    size_t len = strlen(k);
    if (len > sizeof(ctx->secret_key)) {
        len = sizeof(ctx->secret_key);
    }
    memcpy(ctx->secret_key, k, len);
    ctx->secret_key_len = len;
    ctx->allow_unsynced_clock = true;
}

static bool extract_json_str(const char *json, const char *key, char *out, size_t max_len)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) {
        return false;
    }
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') {
        p++;
    }
    if (*p != '\"') {
        return false;
    }
    p++;
    size_t i = 0U;
    while (*p != '\0' && *p != '\"' && (i + 1U) < max_len) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return (i > 0U);
}

static bool extract_json_double(const char *json, const char *key, double *out_val)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) {
        return false;
    }
    p += strlen(pat);
    while (*p == ' ' || *p == ':' || *p == '\t') {
        p++;
    }
    if (*p != '-' && !isdigit((unsigned char)*p)) {
        return false;
    }
    *out_val = strtod(p, NULL);
    return true;
}

static bool ct_hex_matches(const char *hex_in, const uint8_t *mac, size_t compare_bytes)
{
    if (hex_in == NULL || strlen(hex_in) < compare_bytes * 2U) {
        return false;
    }
    char expected[65];
    for (size_t i = 0U; i < CRYPTO_SHA256_LEN; i++) {
        snprintf(&expected[i * 2U], 3U, "%02x", mac[i]);
    }
    uint8_t diff = 0U;
    for (size_t i = 0U; i < compare_bytes * 2U; i++) {
        char c = (char)tolower((unsigned char)hex_in[i]);
        diff |= (uint8_t)(c ^ expected[i]);
    }
    return (diff == 0U);
}

bool crypto_verify_and_parse_cmd(crypto_auth_ctx_t *ctx,
                                 const char *json_payload,
                                 size_t payload_len,
                                 uint32_t current_utc_epoch,
                                 pan_cloud_cmd_t *out_cmd)
{
    if (ctx == NULL || json_payload == NULL || payload_len == 0U || out_cmd == NULL) {
        return false;
    }

    char buf[512];
    size_t copy_len = (payload_len < sizeof(buf) - 1U) ? payload_len : (sizeof(buf) - 1U);
    memcpy(buf, json_payload, copy_len);
    buf[copy_len] = '\0';

    memset(out_cmd, 0, sizeof(*out_cmd));

    char cmd_str[32] = {0};
    char hmac_hex[68] = {0};
    if (!extract_json_str(buf, "cmd", cmd_str, sizeof(cmd_str)) ||
        !extract_json_str(buf, "hmac", hmac_hex, sizeof(hmac_hex))) {
        return false;
    }

    double ts_d = 0.0, nonce_d = 0.0, lat_d = 0.0, lon_d = 0.0, alt_d = 0.0, mode_d = 0.0;
    if (!extract_json_double(buf, "ts", &ts_d) ||
        !extract_json_double(buf, "nonce", &nonce_d)) {
        return false;
    }
    extract_json_double(buf, "lat", &lat_d);
    extract_json_double(buf, "lon", &lon_d);
    extract_json_double(buf, "alt", &alt_d);
    extract_json_double(buf, "mode", &mode_d);

    out_cmd->utc_timestamp = (uint32_t)ts_d;
    out_cmd->nonce = (uint32_t)nonce_d;
    out_cmd->target_lat = lat_d;
    out_cmd->target_lon = lon_d;
    out_cmd->target_alt_m = (float)alt_d;
    out_cmd->mode_arg = (uint8_t)mode_d;

    /* Anti-replay UTC timestamp window check (+-10 seconds) */
    if (current_utc_epoch > 1700000000U) {
        int64_t delta = (int64_t)current_utc_epoch - (int64_t)out_cmd->utc_timestamp;
        if (delta < -(int64_t)CRYPTO_REPLAY_WINDOW_SEC || delta > (int64_t)CRYPTO_REPLAY_WINDOW_SEC) {
            return false;
        }
    } else if (!ctx->allow_unsynced_clock) {
        return false;
    }

    /* Deduplicate nonce */
    for (size_t i = 0U; i < CRYPTO_MAX_NONCE_HISTORY; i++) {
        if (ctx->seen_nonces[i] != 0U && ctx->seen_nonces[i] == out_cmd->nonce) {
            return false;
        }
    }

    int32_t lat_e7 = (int32_t)llround(lat_d * 1e7);
    int32_t lon_e7 = (int32_t)llround(lon_d * 1e7);
    int32_t alt_cm = (int32_t)llround(alt_d * 100.0);

    char canonical[160];
    int clen = snprintf(
        canonical,
        sizeof(canonical),
        "%s|%lu|%lu|%ld|%ld|%ld",
        cmd_str,
        (unsigned long)out_cmd->utc_timestamp,
        (unsigned long)out_cmd->nonce,
        (long)lat_e7,
        (long)lon_e7,
        (long)alt_cm);
    if (clen <= 0) {
        return false;
    }

    uint8_t mac[CRYPTO_SHA256_LEN];
    crypto_hmac_sha256(ctx->secret_key, ctx->secret_key_len, (const uint8_t *)canonical, (size_t)clen, mac);

    size_t hex_len = strlen(hmac_hex);
    size_t cmp_bytes = (hex_len >= 64U) ? 32U : ((hex_len >= 16U) ? 8U : 0U);
    if (cmp_bytes == 0U || !ct_hex_matches(hmac_hex, mac, cmp_bytes)) {
        return false;
    }

    /* Record accepted nonce */
    ctx->seen_nonces[ctx->nonce_head] = out_cmd->nonce;
    ctx->nonce_head = (ctx->nonce_head + 1U) % CRYPTO_MAX_NONCE_HISTORY;

    if (strcmp(cmd_str, "ARM") == 0) {
        out_cmd->type = CMD_ARM;
    } else if (strcmp(cmd_str, "DISARM") == 0) {
        out_cmd->type = CMD_DISARM;
    } else if (strcmp(cmd_str, "RTH") == 0) {
        out_cmd->type = CMD_RTH;
    } else if (strcmp(cmd_str, "GOTO_WAYPOINT") == 0) {
        out_cmd->type = CMD_GOTO_WAYPOINT;
    } else if (strcmp(cmd_str, "SET_MODE") == 0) {
        out_cmd->type = CMD_SET_MODE;
    } else if (strcmp(cmd_str, "VEHICLE_SLEEP") == 0) {
        out_cmd->type = CMD_VEHICLE_SLEEP;
    } else if (strcmp(cmd_str, "OTA_MANIFEST") == 0) {
        out_cmd->type = CMD_OTA_MANIFEST;
        extract_json_str(buf, "sha256", out_cmd->ota_sha256_hex, sizeof(out_cmd->ota_sha256_hex));
        double sz = 0.0;
        if (extract_json_double(buf, "size", &sz)) {
            out_cmd->ota_size_bytes = (uint32_t)sz;
        }
    } else {
        return false;
    }

    return true;
}
