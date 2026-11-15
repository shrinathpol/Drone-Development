#ifndef AHRS_FUSION_H
#define AHRS_FUSION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q[4]; /* Quaternion w, x, y, z */
    float integral_fb[3];
    float kp;
    float ki;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float accel_earth_z_mss;
    float alt_agl_m;
    float climb_rate_ms;
    bool initialized;
} ahrs_state_t;

void ahrs_init(ahrs_state_t *ahrs, float kp, float ki);

void ahrs_update(ahrs_state_t *ahrs,
                 const float gyro_rad_s[3],
                 const float accel_m_s2[3],
                 const float mag_ut[3],
                 bool mag_valid,
                 float baro_alt_m,
                 bool baro_valid,
                 float tof_range_m,
                 bool tof_valid,
                 float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_FUSION_H */

