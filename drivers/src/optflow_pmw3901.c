#include "optflow_pmw3901.h"

#include <string.h>

#define PMW3901_REG_PRODUCT_ID 0x00U
#define PMW3901_REG_MOTION_BURST 0x16U
#define PMW3901_PRODUCT_ID_VAL 0x49U
#define PMW3901_FOV_RAD_SCALE  0.0021f /* Radians per pixel count (42 deg FOV / 30 px scalar) */

int optflow_pmw3901_init(optflow_pmw3901_t *dev, hal_spi_t *spi, hal_sys_t *sys)
{
    if (dev == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(dev, 0, sizeof(*dev));
    dev->spi = spi;
    dev->sys = sys;

    if (spi == NULL || spi->init == NULL || spi->init(spi->ctx, 2000000U) != HAL_OK) {
        return HAL_ERR_NOT_READY;
    }

    uint8_t prod_id = 0U;
    if (spi->transfer_reg(spi->ctx, PMW3901_REG_PRODUCT_ID, NULL, &prod_id, 1U) != HAL_OK ||
        prod_id != PMW3901_PRODUCT_ID_VAL) {
        return HAL_ERR_NOT_READY;
    }

    dev->initialized = true;
    dev->last_poll_us = (uint32_t)sys->micros();
    return HAL_OK;
}

int optflow_pmw3901_poll(optflow_pmw3901_t *dev,
                         float range_agl_m,
                         float gyro_roll_rads,
                         float gyro_pitch_rads,
                         optflow_sample_t *out)
{
    if (dev == NULL || out == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (dev->sim_override) {
        *out = dev->last;
        return dev->last.valid ? HAL_OK : HAL_ERR_NOT_READY;
    }
    if (!dev->initialized || dev->spi == NULL) {
        out->valid = false;
        return HAL_ERR_NOT_READY;
    }

    uint32_t now_us = (uint32_t)dev->sys->micros();
    float dt = (now_us > dev->last_poll_us) ? ((float)(now_us - dev->last_poll_us) * 1e-6f) : 0.01f;
    if (dt < 0.001f || dt > 0.25f) {
        dt = 0.01f;
    }
    dev->last_poll_us = now_us;

    uint8_t burst[12];
    memset(burst, 0, sizeof(burst));
    if (dev->spi->transfer_reg(dev->spi->ctx, PMW3901_REG_MOTION_BURST, NULL, burst, sizeof(burst)) != HAL_OK) {
        out->valid = false;
        return HAL_ERR_BUS;
    }

    bool motion = (burst[0] & 0x80U) != 0U;
    int16_t dx = (int16_t)((burst[3] << 8U) | burst[2]);
    int16_t dy = (int16_t)((burst[5] << 8U) | burst[4]);
    uint8_t squal = burst[6];

    out->delta_x = dx;
    out->delta_y = dy;
    out->surface_quality = squal;

    /* Gate on optical surface quality (> 35) and valid ToF height (< 4.0m) */
    if (squal >= 35U && range_agl_m > 0.05f && range_agl_m <= 4.0f) {
        float flow_rad_x = motion ? ((float)dx * PMW3901_FOV_RAD_SCALE) : 0.0f;
        float flow_rad_y = motion ? ((float)dy * PMW3901_FOV_RAD_SCALE) : 0.0f;
        float rate_x = (flow_rad_x / dt) - gyro_pitch_rads;
        float rate_y = (flow_rad_y / dt) + gyro_roll_rads;

        out->vel_body_x_ms = rate_x * range_agl_m;
        out->vel_body_y_ms = rate_y * range_agl_m;
        out->valid = true;
    } else {
        out->vel_body_x_ms = 0.0f;
        out->vel_body_y_ms = 0.0f;
        out->valid = false;
    }

    dev->last = *out;
    return HAL_OK;
}
