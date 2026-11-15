#include "board_lilygo_sim7000g.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "bsp_wireless";

#define CFG_BUF_SIZE 2048
#define UDP_TELEM_PORT 14550
#define UDP_CMD_PORT   14551

typedef struct {
    bool running;
    bool netif_inited;
    esp_netif_t *ap_netif;
    httpd_handle_t httpd;
    int udp_sock;
    char pending_cfg_json[CFG_BUF_SIZE];
    bool has_pending_cfg;
    char status_json[1024];
} esp32_wireless_ctx_t;

static esp32_wireless_ctx_t s_wireless_ctx = {
    .running = false,
    .netif_inited = false,
    .ap_netif = NULL,
    .httpd = NULL,
    .udp_sock = -1,
    .has_pending_cfg = false,
    .status_json = "{\"status\":\"booting\"}"
};

static const char *CFG_HTML_PAGE =
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>PAN-BRAIN Config</title>"
    "<style>body{background:#0b1120;color:#e2e8f0;font-family:monospace;padding:20px}"
    "textarea{width:100%;height:260px;background:#1e293b;color:#38bdf8;border:1px solid #334155;padding:10px}"
    "button{background:#0ea5e9;color:#082f49;border:0;padding:10px 18px;font-weight:bold;cursor:pointer;margin-top:10px}"
    "</style></head><body><h2>PAN-BRAIN Field Config & Status</h2>"
    "<pre id='st'>Loading...</pre>"
    "<textarea id='cfg'>{\"role\":\"DRONE\",\"apn\":\"iot.com\",\"mqtt_host\":\"broker.hivemq.com\",\"mqtt_port\":1883}</textarea><br>"
    "<button onclick='save()'>Save & Apply Config</button><script>"
    "fetch('/api/status').then(r=>r.json()).then(d=>{document.getElementById('st').textContent=JSON.stringify(d,null,2);});"
    "function save(){fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},"
    "body:document.getElementById('cfg').value}).then(r=>r.text()).then(alert);}"
    "</script></body></html>";

static esp_err_t handle_root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, CFG_HTML_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_status_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, s_wireless_ctx.status_json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_config_post(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= CFG_BUF_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid config size");
        return ESP_FAIL;
    }

    int received = 0;
    while (received < req->content_len) {
        int r = httpd_req_recv(req, s_wireless_ctx.pending_cfg_json + received, req->content_len - received);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }
        received += r;
    }
    s_wireless_ctx.pending_cfg_json[received] = '\0';
    s_wireless_ctx.has_pending_cfg = true;

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static int bsp_wireless_start(void *ctx, const char *ssid, const char *pass)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    if (w == NULL || ssid == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (w->running) {
        return HAL_OK;
    }

    if (!w->netif_inited) {
        esp_netif_init();
        esp_err_t ev_err = esp_event_loop_create_default();
        if (ev_err != ESP_OK && ev_err != ESP_ERR_INVALID_STATE) {
            return HAL_ERR_BUS;
        }
        w->ap_netif = esp_netif_create_default_wifi_ap();
        wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
        if (esp_wifi_init(&init_cfg) != ESP_OK) {
            return HAL_ERR_BUS;
        }
        w->netif_inited = true;
    }

    wifi_config_t wifi_cfg;
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));
    size_t slen = strlen(ssid);
    if (slen > sizeof(wifi_cfg.ap.ssid)) {
        slen = sizeof(wifi_cfg.ap.ssid);
    }
    memcpy(wifi_cfg.ap.ssid, ssid, slen);
    wifi_cfg.ap.ssid_len = (uint8_t)slen;
    wifi_cfg.ap.channel = 6;
    wifi_cfg.ap.max_connection = 4;

    if (pass != NULL && strlen(pass) >= 8U) {
        size_t plen = strlen(pass);
        if (plen >= sizeof(wifi_cfg.ap.password)) {
            plen = sizeof(wifi_cfg.ap.password) - 1U;
        }
        memcpy(wifi_cfg.ap.password, pass, plen);
        wifi_cfg.ap.password[plen] = '\0';
        wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK ||
        esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        return HAL_ERR_BUS;
    }

    httpd_config_t httpd_cfg = HTTPD_DEFAULT_CONFIG();
    httpd_cfg.server_port = 80;
    if (httpd_start(&w->httpd, &httpd_cfg) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = handle_root_get };
        httpd_uri_t st = { .uri = "/api/status", .method = HTTP_GET, .handler = handle_status_get };
        httpd_uri_t cfg = { .uri = "/api/config", .method = HTTP_POST, .handler = handle_config_post };
        httpd_register_uri_handler(w->httpd, &root);
        httpd_register_uri_handler(w->httpd, &st);
        httpd_register_uri_handler(w->httpd, &cfg);
    }

    w->udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (w->udp_sock >= 0) {
        int broadcast = 1;
        setsockopt(w->udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
        fcntl(w->udp_sock, F_SETFL, O_NONBLOCK);

        struct sockaddr_in bind_addr;
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(UDP_CMD_PORT);
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind(w->udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
    }

    w->running = true;
    ESP_LOGI(TAG, "SoftAP '%s' started on 192.168.4.1", ssid);
    return HAL_OK;
}

static int bsp_wireless_stop(void *ctx)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    if (w == NULL || !w->running) {
        return HAL_OK;
    }
    if (w->udp_sock >= 0) {
        close(w->udp_sock);
        w->udp_sock = -1;
    }
    if (w->httpd != NULL) {
        httpd_stop(w->httpd);
        w->httpd = NULL;
    }
    esp_wifi_stop();
    w->running = false;
    return HAL_OK;
}

static int bsp_wireless_send_udp(void *ctx, const uint8_t *data, size_t len)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    if (w == NULL || !w->running || w->udp_sock < 0 || data == NULL || len == 0U) {
        return HAL_ERR_NOT_READY;
    }

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(UDP_TELEM_PORT);
    dst.sin_addr.s_addr = inet_addr("192.168.4.255");

    int sent = sendto(w->udp_sock, data, len, 0, (struct sockaddr *)&dst, sizeof(dst));
    return (sent >= 0) ? HAL_OK : HAL_ERR_IO;
}

static int bsp_wireless_poll_cmd(void *ctx, uint8_t *buf, size_t max_len)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    if (w == NULL || !w->running || w->udp_sock < 0 || buf == NULL || max_len == 0U) {
        return 0;
    }
    int n = recvfrom(w->udp_sock, buf, max_len, 0, NULL, NULL);
    return (n > 0) ? n : 0;
}

static bool bsp_wireless_has_cfg(void *ctx)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    return (w != NULL) ? w->has_pending_cfg : false;
}

static int bsp_wireless_take_cfg(void *ctx, char *out_json, size_t max_len)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    if (w == NULL || !w->has_pending_cfg || out_json == NULL || max_len == 0U) {
        return HAL_ERR_NOT_READY;
    }
    snprintf(out_json, max_len, "%s", w->pending_cfg_json);
    w->has_pending_cfg = false;
    return HAL_OK;
}

static void bsp_wireless_update_status(void *ctx, const char *status_json)
{
    esp32_wireless_ctx_t *w = (esp32_wireless_ctx_t *)ctx;
    if (w != NULL && status_json != NULL) {
        snprintf(w->status_json, sizeof(w->status_json), "%s", status_json);
    }
}

hal_wireless_t bsp_wireless = {
    .ctx = &s_wireless_ctx,
    .start_ap_cfg = bsp_wireless_start,
    .stop_ap_cfg = bsp_wireless_stop,
    .send_udp_telem = bsp_wireless_send_udp,
    .poll_rx_cmd = bsp_wireless_poll_cmd,
    .has_pending_cfg_json = bsp_wireless_has_cfg,
    .take_pending_cfg_json = bsp_wireless_take_cfg,
    .update_status_json = bsp_wireless_update_status
};
