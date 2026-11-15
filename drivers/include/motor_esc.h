#ifndef MOTOR_ESC_H
#define MOTOR_ESC_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_COUNT       4U
#define MOTOR_PWM_MIN_US  1000U
#define MOTOR_PWM_IDLE_US 1080U
#define MOTOR_PWM_MAX_US  2000U

typedef struct {
    hal_pwm_t *pwm;
    hal_sys_t *sys;
    bool armed;
    uint16_t cmd_us[MOTOR_COUNT];
    uint16_t cur_us[MOTOR_COUNT];
    uint32_t rpm[MOTOR_COUNT];
    float dominant_freq_hz;
    bool prop_strike_alert;
    uint32_t last_update_ms;
    float max_slew_us_per_ms;
    uint8_t motor_pole_pairs;
} motor_esc_t;

int motor_esc_init(motor_esc_t *esc, hal_pwm_t *pwm, hal_sys_t *sys);
int motor_esc_arm(motor_esc_t *esc, float throttle_norm, bool sensors_healthy);
void motor_esc_disarm(motor_esc_t *esc);

/**
 * @brief Apply Quad-X mixing (throttle [0..1], roll/pitch/yaw [-1..1]) with slew-rate limit & airmode clamp.
 */
void motor_esc_mix_and_write(motor_esc_t *esc,
                             float throttle,
                             float roll,
                             float pitch,
                             float yaw,
                             float vbat_mv,
                             float gyro_rms_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ESC_H */
