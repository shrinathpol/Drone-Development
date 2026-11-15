#ifndef CAMERA_CTRL_H
#define CAMERA_CTRL_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    hal_pwm_t *gimbal_pwm;
    hal_uart_t *runcam_uart;
    float pitch_trim_deg;
    bool recording;
} camera_ctrl_t;

int camera_ctrl_init(camera_ctrl_t *cam, hal_pwm_t *gimbal_pwm, hal_uart_t *runcam_uart);
void camera_ctrl_update_gimbal(camera_ctrl_t *cam, float drone_roll_deg, float drone_pitch_deg, float target_pitch_deg);
void camera_ctrl_trigger_shutter(camera_ctrl_t *cam);
void camera_ctrl_toggle_recording(camera_ctrl_t *cam);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_CTRL_H */
