#ifndef OPTFLOW_PMW3901_H
#define OPTFLOW_PMW3901_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    uint8_t surface_quality;
    float vel_body_x_ms;
    float vel_body_y_ms;
    bool valid;
} optflow_sample_t;

typedef struct {
    hal_spi_t *spi;
    hal_sys_t *sys;
    bool initialized;
    bool sim_override;
    uint32_t last_poll_us;
    optflow_sample_t last;
} optflow_pmw3901_t;

int optflow_pmw3901_init(optflow_pmw3901_t *dev, hal_spi_t *spi, hal_sys_t *sys);

/**
 * @brief Poll PMW3901 motion burst and compute gyro-compensated ground velocity in m/s.
 */
int optflow_pmw3901_poll(optflow_pmw3901_t *dev,
                         float range_agl_m,
                         float gyro_roll_rads,
                         float gyro_pitch_rads,
                         optflow_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* OPTFLOW_PMW3901_H */
