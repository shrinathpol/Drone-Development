#include "hal_interfaces.h"
#include "main_brain.h"
#include "sitl_physics.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static sitl_physics_t g_phys;
static double g_origin_lat = 15.364700;
static double g_origin_lon = 75.124000;

/* ---------------- Virtual System & GPIO ---------------- */
static uint32_t sitl_millis(void) { return (uint32_t)(g_phys.sim_time_us / 1000ULL); }
static uint64_t sitl_micros(void) { return g_phys.sim_time_us; }
static void sitl_delay_ms(uint32_t ms) { g_phys.sim_time_us += (uint64_t)ms * 1000ULL; }
static void sitl_deep_sleep(uint32_t mask) { printf("[SITL] Enter Deep Sleep (wake=0x%x)\n", mask); }
static void *sitl_alloc(size_t b) { return malloc(b); }
static void sitl_free(void *p) { free(p); }
static void sitl_log(uint8_t lvl, const char *tag, const char *msg) {
    (void)lvl;
    printf("[SITL/%s] %s\n", tag ? tag : "pan", msg ? msg : "");
}

hal_sys_t sitl_sys = {
    .millis = sitl_millis,
    .micros = sitl_micros,
    .delay_ms = sitl_delay_ms,
    .enter_deep_sleep = sitl_deep_sleep,
    .alloc_psram = sitl_alloc,
    .free_psram = sitl_free,
    .log_msg = sitl_log,
    .reboot = NULL
};

static int sitl_gpio_mode_out(uint8_t p) { (void)p; return HAL_OK; }
static int sitl_gpio_mode_in(uint8_t p, bool pu) { (void)p; (void)pu; return HAL_OK; }
static void sitl_gpio_write(uint8_t p, bool l) { (void)p; (void)l; }
static bool sitl_gpio_read(uint8_t p) { (void)p; return false; }

hal_gpio_t sitl_gpio = {
    .mode_output = sitl_gpio_mode_out,
    .mode_input = sitl_gpio_mode_in,
    .write = sitl_gpio_write,
    .read = sitl_gpio_read
};

/* ---------------- Virtual SIM7000G AT Responder UART ---------------- */
typedef struct {
    char rx_ring[2048];
    size_t rx_len;
    char tx_acc[256];
    size_t tx_idx;
    bool awaiting_pub_payload;
    bool awaiting_sms_body;
} sitl_modem_uart_t;

static sitl_modem_uart_t s_sim_uart;

static void push_modem_rx(sitl_modem_uart_t *u, const char *s)
{
    size_t slen = strlen(s);
    if (u->rx_len + slen < sizeof(u->rx_ring)) {
        memcpy(u->rx_ring + u->rx_len, s, slen);
        u->rx_len += slen;
    }
}

static int sitl_uart_init(void *ctx, uint32_t baud) { (void)ctx; (void)baud; return HAL_OK; }
static int sitl_uart_baud(void *ctx, uint32_t baud) { (void)ctx; (void)baud; return HAL_OK; }

static int sitl_uart_write(void *ctx, const uint8_t *data, size_t len)
{
    sitl_modem_uart_t *u = (sitl_modem_uart_t *)ctx;
    if (u == NULL) return (int)len;

    if (u->awaiting_pub_payload) {
        u->awaiting_pub_payload = false;
        push_modem_rx(u, "OK\r\n");
        return (int)len;
    }
    if (u->awaiting_sms_body) {
        for (size_t i = 0U; i < len; i++) {
            if (data[i] == 0x1AU) {
                u->awaiting_sms_body = false;
                push_modem_rx(u, "+CMGS: 1\r\nOK\r\n");
            }
        }
        return (int)len;
    }

    for (size_t i = 0U; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            u->tx_acc[u->tx_idx] = '\0';
            if (strstr(u->tx_acc, "AT+CPIN?") != NULL) {
                push_modem_rx(u, "+CPIN: READY\r\nOK\r\n");
            } else if (strstr(u->tx_acc, "AT+CGREG?") != NULL) {
                push_modem_rx(u, "+CGREG: 0,1\r\nOK\r\n");
            } else if (strstr(u->tx_acc, "AT+CNACT?") != NULL) {
                push_modem_rx(u, "+CNACT: 0,1,\"10.42.0.18\"\r\nOK\r\n");
            } else if (strstr(u->tx_acc, "AT+CSQ") != NULL) {
                push_modem_rx(u, "+CSQ: 24,0\r\nOK\r\n");
            } else if (strstr(u->tx_acc, "AT+CCLK?") != NULL) {
                push_modem_rx(u, "+CCLK: \"26/11/15,14:00:00+22\"\r\nOK\r\n");
            } else if (strstr(u->tx_acc, "AT+CGNSINF") != NULL) {
                double lat = g_origin_lat + (g_phys.pn_m / 6371000.0) * 57.2957795;
                double lon = g_origin_lon + (g_phys.pe_m / (6371000.0 * cos(g_origin_lat * 0.01745329))) * 57.2957795;
                char resp[200];
                snprintf(resp, sizeof(resp),
                         "+CGNSINF: 1,1,20261115140000.000,%.6f,%.6f,%.2f,%.2f,%.1f,1,,0.9,1.2,1.0,,12,11\r\nOK\r\n",
                         lat, lon, -g_phys.pd_m, sqrtf(g_phys.vn_ms * g_phys.vn_ms + g_phys.ve_ms * g_phys.ve_ms) * 3.6f,
                         g_phys.yaw_rad * 57.2957795f);
                push_modem_rx(u, resp);
            } else if (strstr(u->tx_acc, "AT+SMPUB=") != NULL) {
                u->awaiting_pub_payload = true;
                push_modem_rx(u, ">");
            } else if (strstr(u->tx_acc, "AT+CMGS=") != NULL) {
                u->awaiting_sms_body = true;
                push_modem_rx(u, ">");
            } else {
                push_modem_rx(u, "OK\r\n");
            }
            u->tx_idx = 0U;
        } else if (c != '\r' && u->tx_idx < sizeof(u->tx_acc) - 1U) {
            u->tx_acc[u->tx_idx++] = c;
        }
    }
    return (int)len;
}

static int sitl_uart_read(void *ctx, uint8_t *buf, size_t max_len)
{
    sitl_modem_uart_t *u = (sitl_modem_uart_t *)ctx;
    if (u == NULL || u->rx_len == 0U || max_len == 0U) return 0;
    size_t n = (u->rx_len < max_len) ? u->rx_len : max_len;
    memcpy(buf, u->rx_ring, n);
    if (n < u->rx_len) {
        memmove(u->rx_ring, u->rx_ring + n, u->rx_len - n);
    }
    u->rx_len -= n;
    return (int)n;
}

static void sitl_uart_flush(void *ctx)
{
    sitl_modem_uart_t *u = (sitl_modem_uart_t *)ctx;
    if (u != NULL) u->rx_len = 0U;
}

hal_uart_t sitl_uart_sim = {
    .ctx = &s_sim_uart,
    .init = sitl_uart_init,
    .set_baudrate = sitl_uart_baud,
    .write = sitl_uart_write,
    .read = sitl_uart_read,
    .flush_rx = sitl_uart_flush
};

/* ---------------- Virtual PWM ESC Feedback ---------------- */
static int sitl_pwm_init(void *ctx, uint32_t hz) { (void)ctx; (void)hz; return HAL_OK; }
static int sitl_pwm_set(void *ctx, uint8_t ch, uint16_t us) {
    (void)ctx;
    if (ch < 4U) g_phys.motor_pwm_us[ch] = us;
    return HAL_OK;
}
static int sitl_pwm_erpm(void *ctx, uint8_t ch, uint32_t *out) {
    (void)ctx;
    if (ch >= 4U || out == NULL) return HAL_ERR_INVALID_ARG;
    float duty = (g_phys.motor_pwm_us[ch] > 1000U) ? ((float)(g_phys.motor_pwm_us[ch] - 1000U) / 1000.0f) : 0.0f;
    *out = (uint32_t)(duty * 11000.0f * 7.0f);
    return HAL_OK;
}

hal_pwm_t sitl_pwm_esc = {
    .ctx = NULL,
    .init = sitl_pwm_init,
    .set_pulse_us = sitl_pwm_set,
    .read_erpm = sitl_pwm_erpm
};

/* ---------------- Virtual ADC, Local File Storage & UDP Bridge ---------------- */
static int sitl_adc_init(uint8_t ch) { (void)ch; return HAL_OK; }
static uint32_t sitl_adc_read(uint8_t ch) { return (ch == 0U) ? (uint32_t)g_phys.vbat_mv : 5100U; }
hal_adc_t sitl_adc = { .init = sitl_adc_init, .read_mv = sitl_adc_read };

static const char *map_sitl_path(const char *p)
{
    if (strncmp(p, "/sdcard/", 8) == 0) return p + 8;
    return p;
}
static int sitl_st_mount(void) { return HAL_OK; }
static int sitl_st_append(const char *p, const void *d, size_t l) {
    FILE *f = fopen(map_sitl_path(p), "ab");
    if (!f) return HAL_ERR_IO;
    size_t w = fwrite(d, 1, l, f); fclose(f);
    return (w == l) ? HAL_OK : HAL_ERR_IO;
}
static int sitl_st_write(const char *p, const void *d, size_t l) {
    FILE *f = fopen(map_sitl_path(p), "wb");
    if (!f) return HAL_ERR_IO;
    size_t w = fwrite(d, 1, l, f); fclose(f);
    return (w == l) ? HAL_OK : HAL_ERR_IO;
}
static int sitl_st_read(const char *p, void *b, size_t m, size_t off) {
    FILE *f = fopen(map_sitl_path(p), "rb");
    if (!f) return HAL_ERR_IO;
    if (off > 0) fseek(f, (long)off, SEEK_SET);
    size_t r = fread(b, 1, m, f); fclose(f);
    return (int)r;
}
static int sitl_st_del(const char *p) { return (remove(map_sitl_path(p)) == 0) ? HAL_OK : HAL_ERR_IO; }
static int64_t sitl_st_sz(const char *p) {
    struct stat st;
    return (stat(map_sitl_path(p), &st) == 0) ? (int64_t)st.st_size : -1;
}
hal_storage_t sitl_storage = {
    .mount = sitl_st_mount,
    .append_file = sitl_st_append,
    .read_file = sitl_st_read,
    .delete_file = sitl_st_del,
    .write_file = sitl_st_write,
    .file_size = sitl_st_sz
};

/* ---------------- Virtual IMU VTable ---------------- */
static int sitl_imu_init(imu_driver_t *dev) {
    dev->chip_type = IMU_CHIP_SITL_VIRTUAL;
    return HAL_OK;
}
static int sitl_imu_read(imu_driver_t *dev, imu_sample_t *out) {
    (void)dev;
    memcpy(out->gyro_rad_s, g_phys.gyro_rad_s, sizeof(out->gyro_rad_s));
    memcpy(out->accel_m_s2, g_phys.accel_body_m_s2, sizeof(out->accel_m_s2));
    out->temp_c = 28.0f;
    out->timestamp_us = (uint32_t)g_phys.sim_time_us;
    out->valid = true;
    return HAL_OK;
}
static int sitl_imu_wom(imu_driver_t *dev, uint16_t th) {
    (void)dev; (void)th;
    return HAL_OK;
}
const imu_vtable_t sitl_imu_vtable = {
    .name = "SITL-6DOF-VirtualIMU",
    .probe_and_init = sitl_imu_init,
    .read_sample = sitl_imu_read,
    .enable_wom = sitl_imu_wom
};

/* ---------------- UDP Socket Link to Cloud Backend / GCS ---------------- */
static int s_udp_sock = -1;
static int sitl_wifi_start(void *ctx, const char *s, const char *p) {
    (void)ctx; (void)s; (void)p;
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    s_udp_sock = (int)socket(AF_INET, SOCK_DGRAM, 0);
    return HAL_OK;
}
static int sitl_wifi_stop(void *ctx) { (void)ctx; return HAL_OK; }
static int sitl_wifi_send_udp(void *ctx, const uint8_t *d, size_t l) {
    (void)ctx;
    if (s_udp_sock < 0) return HAL_ERR_NOT_READY;
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(14550);
    dst.sin_addr.s_addr = inet_addr("127.0.0.1");
    sendto(s_udp_sock, (const char *)d, (int)l, 0, (struct sockaddr *)&dst, sizeof(dst));
    return HAL_OK;
}
static int sitl_wifi_poll_cmd(void *ctx, uint8_t *b, size_t m) { (void)ctx; (void)b; (void)m; return 0; }

hal_wireless_t sitl_wireless = {
    .ctx = NULL,
    .start_ap_cfg = sitl_wifi_start,
    .stop_ap_cfg = sitl_wifi_stop,
    .send_udp_telem = sitl_wifi_send_udp,
    .poll_rx_cmd = sitl_wifi_poll_cmd,
    .has_pending_cfg_json = NULL,
    .take_pending_cfg_json = NULL,
    .update_status_json = NULL
};

sitl_physics_t *sitl_get_physics(void) { return &g_phys; }
void sitl_sync_brain_sensors(pan_brain_system_t *brain)
{
    brain->env.sim_override = true;
    brain->env.baro.altitude_m = -g_phys.pd_m;
    brain->env.baro.valid = true;
    brain->env.tof.range_m = -g_phys.pd_m;
    brain->env.tof.range_mm = (uint16_t)(-g_phys.pd_m * 1000.0f);
    brain->env.tof.valid = (-g_phys.pd_m <= 3.8f);
    brain->env.mag.mag_ut[0] = cosf(g_phys.yaw_rad) * 45.0f;
    brain->env.mag.mag_ut[1] = -sinf(g_phys.yaw_rad) * 45.0f;
    brain->env.mag.mag_ut[2] = 20.0f;
    brain->env.mag.valid = true;
    brain->env.power.vbat_mv = (uint16_t)g_phys.vbat_mv;
    brain->env.power.current_ma = (int32_t)(g_phys.current_a * 1000.0f);
    brain->env.power.power_w = (g_phys.vbat_mv * 0.001f) * g_phys.current_a;
    brain->env.power.valid = true;
}

