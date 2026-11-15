#include "camera_ctrl.h"

#include <string.h>

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((crc & 0x80U) != 0U) {
            crc = (uint8_t)((crc << 1U) ^ 0xD5U);
        } else {
            crc = (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static uint16_t deg_to_servo_us(float deg)
{
    if (deg < -45.0f) deg = -45.0f;
    if (deg > 45.0f) deg = 45.0f;
    return (uint16_t)(1500.0f + (deg / 45.0f) * 500.0f);
}

int camera_ctrl_init(camera_ctrl_t *cam, hal_pwm_t *gimbal_pwm, hal_uart_t *runcam_uart)
{
    if (cam == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(cam, 0, sizeof(*cam));
    cam->gimbal_pwm = gimbal_pwm;
    cam->runcam_uart = runcam_uart;

    if (gimbal_pwm != NULL && gimbal_pwm->init != NULL) {
        gimbal_pwm->init(gimbal_pwm->ctx, 50U);
        gimbal_pwm->set_pulse_us(gimbal_pwm->ctx, 0U, 1500U);
        gimbal_pwm->set_pulse_us(gimbal_pwm->ctx, 1U, 1500U);
    }
    return HAL_OK;
}

void camera_ctrl_update_gimbal(camera_ctrl_t *cam, float drone_roll_deg, float drone_pitch_deg, float target_pitch_deg)
{
    if (cam == NULL || cam->gimbal_pwm == NULL || cam->gimbal_pwm->set_pulse_us == NULL) {
        return;
    }
    cam->pitch_trim_deg = target_pitch_deg;
    uint16_t pitch_us = deg_to_servo_us(target_pitch_deg - drone_pitch_deg);
    uint16_t roll_us = deg_to_servo_us(-drone_roll_deg);

    cam->gimbal_pwm->set_pulse_us(cam->gimbal_pwm->ctx, 0U, pitch_us);
    cam->gimbal_pwm->set_pulse_us(cam->gimbal_pwm->ctx, 1U, roll_us);
}

static void send_runcam_ctrl(camera_ctrl_t *cam, uint8_t action)
{
    if (cam->runcam_uart == NULL || cam->runcam_uart->write == NULL) {
        return;
    }
    /* RunCam Device Protocol: [0xCC, cmd=0x01 (RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL), action, crc8] */
    uint8_t pkt[4] = {0xCCU, 0x01U, action, 0x00U};
    uint8_t crc = 0U;
    for (size_t i = 0U; i < 3U; i++) {
        crc = crc8_dvb_s2(crc, pkt[i]);
    }
    pkt[3] = crc;
    cam->runcam_uart->write(cam->runcam_uart->ctx, pkt, sizeof(pkt));
}

void camera_ctrl_trigger_shutter(camera_ctrl_t *cam)
{
    if (cam != NULL) {
        send_runcam_ctrl(cam, 0x00U); /* Simulate WiFi/Shutter button */
    }
}

void camera_ctrl_toggle_recording(camera_ctrl_t *cam)
{
    if (cam != NULL) {
        cam->recording = !cam->recording;
        send_runcam_ctrl(cam, cam->recording ? 0x03U : 0x04U);
    }
}
