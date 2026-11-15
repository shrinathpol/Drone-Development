#include "motor_esc.h"

#include <math.h>
#include <string.h>

static float clampf_m(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int motor_esc_init(motor_esc_t *esc, hal_pwm_t *pwm, hal_sys_t *sys)
{
    if (esc == NULL || pwm == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(esc, 0, sizeof(*esc));
    esc->pwm = pwm;
    esc->sys = sys;
    esc->armed = false;
    esc->max_slew_us_per_ms = 2.5f; /* Full 1000->2000 sweep in ~400ms max transient rate */
    esc->motor_pole_pairs = 7U;     /* 14-pole brushless motor typical */
    esc->last_update_ms = sys->millis();

    if (pwm->init != NULL && pwm->init(pwm->ctx, 400U) != HAL_OK) {
        return HAL_ERR_BUS;
    }

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
        esc->cmd_us[i] = MOTOR_PWM_MIN_US;
        esc->cur_us[i] = MOTOR_PWM_MIN_US;
        pwm->set_pulse_us(pwm->ctx, i, MOTOR_PWM_MIN_US);
    }
    return HAL_OK;
}

int motor_esc_arm(motor_esc_t *esc, float throttle_norm, bool sensors_healthy)
{
    if (esc == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (!sensors_healthy || throttle_norm > 0.05f) {
        return HAL_ERR_NOT_READY;
    }
    esc->armed = true;
    esc->prop_strike_alert = false;
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
        esc->cmd_us[i] = MOTOR_PWM_IDLE_US;
        esc->cur_us[i] = MOTOR_PWM_IDLE_US;
        esc->pwm->set_pulse_us(esc->pwm->ctx, i, MOTOR_PWM_IDLE_US);
    }
    return HAL_OK;
}

void motor_esc_disarm(motor_esc_t *esc)
{
    if (esc == NULL) {
        return;
    }
    esc->armed = false;
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
        esc->cmd_us[i] = MOTOR_PWM_MIN_US;
        esc->cur_us[i] = MOTOR_PWM_MIN_US;
        esc->rpm[i] = 0U;
        if (esc->pwm != NULL && esc->pwm->set_pulse_us != NULL) {
            esc->pwm->set_pulse_us(esc->pwm->ctx, i, MOTOR_PWM_MIN_US);
        }
    }
    esc->dominant_freq_hz = 0.0f;
}

void motor_esc_mix_and_write(motor_esc_t *esc,
                             float throttle,
                             float roll,
                             float pitch,
                             float yaw,
                             float vbat_mv,
                             float gyro_rms_rad_s)
{
    if (esc == NULL || esc->pwm == NULL) {
        return;
    }

    uint32_t now = esc->sys->millis();
    uint32_t dt_ms = (now > esc->last_update_ms) ? (now - esc->last_update_ms) : 2U;
    if (dt_ms > 50U) {
        dt_ms = 50U;
    }
    esc->last_update_ms = now;

    if (!esc->armed) {
        motor_esc_disarm(esc);
        return;
    }

    float t = clampf_m(throttle, 0.0f, 1.0f);
    /* Quad-X Mixing Matrix:
     * M1 (GPIO 18): Front-Right CCW -> T - P - R - Y
     * M2 (GPIO 19): Rear-Left   CCW -> T + P + R - Y
     * M3 (GPIO 23): Front-Left  CW  -> T - P + R + Y
     * M4 (GPIO 5):  Rear-Right  CW  -> T + P - R + Y
     */
    float mix[MOTOR_COUNT];
    mix[0] = t - pitch - roll - yaw;
    mix[1] = t + pitch + roll - yaw;
    mix[2] = t - pitch + roll + yaw;
    mix[3] = t + pitch - roll + yaw;

    /* Desaturation / Airmode preserving differential torque authority */
    float max_m = mix[0];
    float min_m = mix[0];
    for (uint8_t i = 1U; i < MOTOR_COUNT; i++) {
        if (mix[i] > max_m) max_m = mix[i];
        if (mix[i] < min_m) min_m = mix[i];
    }
    if (max_m > 1.0f) {
        float shift = max_m - 1.0f;
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
            mix[i] -= shift;
        }
    } else if (min_m < 0.0f) {
        float shift = -min_m;
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
            mix[i] += shift;
        }
    }

    float max_slew = esc->max_slew_us_per_ms * (float)dt_ms;
    float rpm_sum = 0.0f;
    uint32_t rpm_min = 999999U;
    uint32_t rpm_max = 0U;

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
        float norm = clampf_m(mix[i], 0.0f, 1.0f);
        uint16_t target = (uint16_t)((float)MOTOR_PWM_IDLE_US +
                                     norm * (float)(MOTOR_PWM_MAX_US - MOTOR_PWM_IDLE_US));
        esc->cmd_us[i] = target;

        float diff = (float)target - (float)esc->cur_us[i];
        if (diff > max_slew) diff = max_slew;
        if (diff < -max_slew) diff = -max_slew;
        esc->cur_us[i] = (uint16_t)((float)esc->cur_us[i] + diff);

        esc->pwm->set_pulse_us(esc->pwm->ctx, i, esc->cur_us[i]);

        uint32_t erpm = 0U;
        if (esc->pwm->read_erpm != NULL &&
            esc->pwm->read_erpm(esc->pwm->ctx, i, &erpm) == HAL_OK && erpm > 0U) {
            esc->rpm[i] = erpm / (esc->motor_pole_pairs > 0U ? esc->motor_pole_pairs : 1U);
        } else {
            /* Physics-based KV*V estimate when analog MCPWM ESC has no return wire */
            float volts = (vbat_mv > 6000.0f) ? (vbat_mv * 0.001f) : 11.1f;
            float duty = (float)(esc->cur_us[i] - MOTOR_PWM_MIN_US) / 1000.0f;
            esc->rpm[i] = (uint32_t)(duty * volts * 950.0f);
        }
        rpm_sum += (float)esc->rpm[i];
        if (esc->rpm[i] < rpm_min) rpm_min = esc->rpm[i];
        if (esc->rpm[i] > rpm_max) rpm_max = esc->rpm[i];
    }

    float avg_rpm = rpm_sum / (float)MOTOR_COUNT;
    esc->dominant_freq_hz = clampf_m(avg_rpm / 60.0f, 60.0f, 450.0f);

    /* Prop-strike / bearing anomaly: extreme vibration + asymmetric RPM drop at flight throttle */
    if (t > 0.25f && gyro_rms_rad_s > 6.5f && (rpm_max > rpm_min + 2500U)) {
        esc->prop_strike_alert = true;
    }
}
