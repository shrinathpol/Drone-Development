#include "led_status.h"

#include <string.h>

typedef struct {
    bool on;
    uint16_t dur_ms;
} led_step_t;

static const led_step_t PAT_OFF[] = { { false, 1000 } };
static const led_step_t PAT_BOOT[] = { { true, 100 }, { false, 100 }, { true, 100 }, { false, 700 } };
static const led_step_t PAT_SEARCH[] = { { true, 500 }, { false, 500 } };
static const led_step_t PAT_ONLINE[] = { { true, 80 }, { false, 1920 } };
static const led_step_t PAT_ARMED[] = { { true, 420 }, { false, 80 } };
static const led_step_t PAT_RTH[] = { { true, 100 }, { false, 100 } };
static const led_step_t PAT_OTA[] = {
    { true, 70 }, { false, 70 }, { true, 70 }, { false, 70 }, { true, 70 }, { false, 650 }
};
static const led_step_t PAT_ERROR[] = { { true, 50 }, { false, 50 } };

static void get_pattern_table(led_pattern_t pat, const led_step_t **out_tbl, size_t *out_len)
{
    switch (pat) {
    case LED_PAT_OFF_SLEEP:
        *out_tbl = PAT_OFF; *out_len = sizeof(PAT_OFF) / sizeof(PAT_OFF[0]);
        break;
    case LED_PAT_BOOT_INIT:
        *out_tbl = PAT_BOOT; *out_len = sizeof(PAT_BOOT) / sizeof(PAT_BOOT[0]);
        break;
    case LED_PAT_SEARCHING_NET_GPS:
        *out_tbl = PAT_SEARCH; *out_len = sizeof(PAT_SEARCH) / sizeof(PAT_SEARCH[0]);
        break;
    case LED_PAT_DISARMED_ONLINE:
        *out_tbl = PAT_ONLINE; *out_len = sizeof(PAT_ONLINE) / sizeof(PAT_ONLINE[0]);
        break;
    case LED_PAT_ARMED_FLIGHT:
        *out_tbl = PAT_ARMED; *out_len = sizeof(PAT_ARMED) / sizeof(PAT_ARMED[0]);
        break;
    case LED_PAT_FAILSAFE_RTH:
        *out_tbl = PAT_RTH; *out_len = sizeof(PAT_RTH) / sizeof(PAT_RTH[0]);
        break;
    case LED_PAT_OTA_UPDATING:
        *out_tbl = PAT_OTA; *out_len = sizeof(PAT_OTA) / sizeof(PAT_OTA[0]);
        break;
    case LED_PAT_CRITICAL_ERROR:
    default:
        *out_tbl = PAT_ERROR; *out_len = sizeof(PAT_ERROR) / sizeof(PAT_ERROR[0]);
        break;
    }
}

static void apply_led_level(led_status_t *led, bool on)
{
    led->current_on = on;
    if (led->gpio != NULL && led->gpio->write != NULL) {
        bool hw_level = led->active_low ? (!on) : on;
        led->gpio->write(led->pin, hw_level);
    }
}

int led_status_init(led_status_t *led,
                    hal_gpio_t *gpio,
                    hal_sys_t *sys,
                    uint8_t pin,
                    bool active_low)
{
    if (led == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(led, 0, sizeof(*led));
    led->gpio = gpio;
    led->sys = sys;
    led->pin = pin;
    led->active_low = active_low;
    led->pattern = LED_PAT_BOOT_INIT;
    led->step_start_ms = sys->millis();

    if (gpio != NULL && gpio->mode_output != NULL) {
        gpio->mode_output(pin);
    }
    apply_led_level(led, true);
    return HAL_OK;
}

void led_status_set_pattern(led_status_t *led, led_pattern_t pattern)
{
    if (led == NULL || led->pattern == pattern) {
        return;
    }
    led->pattern = pattern;
    led->step_idx = 0U;
    led->step_start_ms = (led->sys != NULL) ? led->sys->millis() : 0U;

    const led_step_t *tbl = NULL;
    size_t len = 0U;
    get_pattern_table(pattern, &tbl, &len);
    if (len > 0U) {
        apply_led_level(led, tbl[0].on);
    }
}

void led_status_step_poll(led_status_t *led)
{
    if (led == NULL || led->sys == NULL) {
        return;
    }
    const led_step_t *tbl = NULL;
    size_t len = 0U;
    get_pattern_table(led->pattern, &tbl, &len);
    if (len == 0U) {
        return;
    }

    uint32_t now = led->sys->millis();
    if ((now - led->step_start_ms) >= tbl[led->step_idx].dur_ms) {
        led->step_idx = (uint8_t)((led->step_idx + 1U) % len);
        led->step_start_ms = now;
        apply_led_level(led, tbl[led->step_idx].on);
    }
}

