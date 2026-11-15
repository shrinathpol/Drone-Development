#ifndef LED_STATUS_H
#define LED_STATUS_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_PAT_OFF_SLEEP = 0,
    LED_PAT_BOOT_INIT,
    LED_PAT_SEARCHING_NET_GPS,
    LED_PAT_DISARMED_ONLINE,
    LED_PAT_ARMED_FLIGHT,
    LED_PAT_FAILSAFE_RTH,
    LED_PAT_OTA_UPDATING,
    LED_PAT_CRITICAL_ERROR
} led_pattern_t;

typedef struct {
    hal_gpio_t *gpio;
    hal_sys_t *sys;
    uint8_t pin;
    bool active_low;
    led_pattern_t pattern;
    uint8_t step_idx;
    uint32_t step_start_ms;
    bool current_on;
} led_status_t;

int led_status_init(led_status_t *led,
                    hal_gpio_t *gpio,
                    hal_sys_t *sys,
                    uint8_t pin,
                    bool active_low);

void led_status_set_pattern(led_status_t *led, led_pattern_t pattern);

void led_status_step_poll(led_status_t *led);

#ifdef __cplusplus
}
#endif

#endif /* LED_STATUS_H */

