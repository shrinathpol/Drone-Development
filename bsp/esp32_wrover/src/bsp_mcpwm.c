#include "board_lilygo_sim7000g.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char *TAG = "bsp_mcpwm";

#define PWM_RESOLUTION_HZ 1000000U /* 1 us per tick */
#define MAX_PWM_CHANS     4U

typedef struct {
    int group_id;
    uint8_t channel_count;
    int pins[MAX_PWM_CHANS];
    bool initialized;
    uint32_t period_us;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t opers[2];
    mcpwm_cmpr_handle_t cmprs[MAX_PWM_CHANS];
    mcpwm_gen_handle_t gens[MAX_PWM_CHANS];
} esp32_pwm_ctx_t;

static esp32_pwm_ctx_t s_esc_pwm_ctx = {
    .group_id = 0,
    .channel_count = 4,
    .pins = { PIN_ESC_M1, PIN_ESC_M2, PIN_ESC_M3, PIN_ESC_M4 },
    .initialized = false,
    .period_us = 2500U
};

static esp32_pwm_ctx_t s_gimbal_pwm_ctx = {
    .group_id = 1,
    .channel_count = 2,
    .pins = { PIN_GIMBAL_PITCH, PIN_GIMBAL_ROLL, -1, -1 },
    .initialized = false,
    .period_us = 20000U
};

static int bsp_mcpwm_init(void *ctx, uint32_t freq_hz)
{
    esp32_pwm_ctx_t *p = (esp32_pwm_ctx_t *)ctx;
    if (p == NULL || freq_hz == 0U || freq_hz > 1000U) {
        return HAL_ERR_INVALID_ARG;
    }
    if (p->initialized) {
        return HAL_OK;
    }

    bool any_pin = false;
    for (uint8_t ch = 0; ch < p->channel_count; ch++) {
        if (p->pins[ch] >= 0) {
            any_pin = true;
            break;
        }
    }
    if (!any_pin) {
        p->period_us = PWM_RESOLUTION_HZ / freq_hz;
        p->initialized = true;
        ESP_LOGI(TAG, "MCPWM group %d skipped (no pins assigned)", p->group_id);
        return HAL_OK;
    }

    p->period_us = PWM_RESOLUTION_HZ / freq_hz;
    if (p->period_us < 2000U) {
        p->period_us = 2000U;
    }

    mcpwm_timer_config_t timer_cfg = {
        .group_id = p->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = p->period_us,
        .intr_priority = 0
    };
    if (mcpwm_new_timer(&timer_cfg, &p->timer) != ESP_OK) {
        return HAL_ERR_BUS;
    }

    uint8_t op_needed = (p->channel_count + 1U) / 2U;
    for (uint8_t op = 0; op < op_needed; op++) {
        mcpwm_operator_config_t oper_cfg = {
            .group_id = p->group_id
        };
        if (mcpwm_new_operator(&oper_cfg, &p->opers[op]) != ESP_OK) {
            return HAL_ERR_BUS;
        }
        if (mcpwm_operator_connect_timer(p->opers[op], p->timer) != ESP_OK) {
            return HAL_ERR_BUS;
        }
    }

    for (uint8_t ch = 0; ch < p->channel_count; ch++) {
        if (p->pins[ch] < 0) {
            p->cmprs[ch] = NULL;
            p->gens[ch] = NULL;
            continue;
        }

        mcpwm_oper_handle_t oper = p->opers[ch / 2U];
        mcpwm_comparator_config_t cmp_cfg = {
            .flags.update_cmp_on_tez = true
        };
        if (mcpwm_new_comparator(oper, &cmp_cfg, &p->cmprs[ch]) != ESP_OK) {
            return HAL_ERR_BUS;
        }

        mcpwm_generator_config_t gen_cfg = {
            .gen_gpio_num = p->pins[ch]
        };
        if (mcpwm_new_generator(oper, &gen_cfg, &p->gens[ch]) != ESP_OK) {
            return HAL_ERR_BUS;
        }

        mcpwm_comparator_set_compare_value(p->cmprs[ch], 1000U);

        mcpwm_generator_set_action_on_timer_event(
            p->gens[ch],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(
            p->gens[ch],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, p->cmprs[ch], MCPWM_GEN_ACTION_LOW));
    }

    if (mcpwm_timer_enable(p->timer) != ESP_OK) {
        return HAL_ERR_BUS;
    }
    if (mcpwm_timer_start_stop(p->timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
        return HAL_ERR_BUS;
    }

    p->initialized = true;
    return HAL_OK;
}

static int bsp_mcpwm_set_pulse_us(void *ctx, uint8_t channel, uint16_t pulse_us)
{
    esp32_pwm_ctx_t *p = (esp32_pwm_ctx_t *)ctx;
    if (p == NULL || !p->initialized || channel >= p->channel_count) {
        return HAL_ERR_INVALID_ARG;
    }
    if (p->cmprs[channel] == NULL) {
        return HAL_OK; /* Channel unassigned (NC) */
    }
    if (pulse_us < 800U) {
        pulse_us = 800U;
    }
    if (pulse_us > 2200U) {
        pulse_us = 2200U;
    }
    return (mcpwm_comparator_set_compare_value(p->cmprs[channel], pulse_us) == ESP_OK) ? HAL_OK : HAL_ERR_BUS;
}

static int bsp_mcpwm_read_erpm(void *ctx, uint8_t channel, uint32_t *erpm_out)
{
    /* Standard analog PWM/OneShot125 via MCPWM does not have bidirectional DShot line turnaround.
     * Return HAL_ERR_NOT_READY so motor_esc falls back to estimated eRPM / FFT peak detection. */
    (void)ctx;
    (void)channel;
    if (erpm_out != NULL) {
        *erpm_out = 0U;
    }
    return HAL_ERR_NOT_READY;
}

hal_pwm_t bsp_pwm_esc = {
    .ctx = &s_esc_pwm_ctx,
    .init = bsp_mcpwm_init,
    .set_pulse_us = bsp_mcpwm_set_pulse_us,
    .read_erpm = bsp_mcpwm_read_erpm
};

hal_pwm_t bsp_pwm_gimbal = {
    .ctx = &s_gimbal_pwm_ctx,
    .init = bsp_mcpwm_init,
    .set_pulse_us = bsp_mcpwm_set_pulse_us,
    .read_erpm = NULL
};
