#ifndef EKF_NAV_H
#define EKF_NAV_H

#include "ahrs_fusion.h"
#include "gps_ublox.h"
#include "optflow_pmw3901.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 15-State Extended Kalman Filter state layout:
 *   x[0..2]  : Roll, Pitch, Yaw (rad)
 *   x[3..5]  : Vn, Ve, Vd (m/s, NED)
 *   x[6..8]  : Pn, Pe, Pd (m, NED relative to origin)
 *   x[9..11] : Gyro bias bgx, bgy, bgz (rad/s)
 *   x[12..14]: Accel bias bax, bay, baz (m/s^2)
 */
#define EKF_STATE_DIM 15U

typedef struct {
    float x[EKF_STATE_DIM];
    float p_diag[EKF_STATE_DIM];

    double origin_lat;
    double origin_lon;
    bool origin_set;

    bool using_optflow_dead_reckon;
    double est_lat;
    double est_lon;
    float est_alt_agl_m;
    float ground_speed_ms;
} ekf_nav_t;

void ekf_nav_init(ekf_nav_t *ekf);

void ekf_nav_set_origin(ekf_nav_t *ekf, double lat, double lon);

void ekf_nav_predict(ekf_nav_t *ekf,
                     const float gyro_rad_s[3],
                     const float accel_m_s2[3],
                     const ahrs_state_t *ahrs,
                     float dt_s);

void ekf_nav_update_measurements(ekf_nav_t *ekf,
                                 const gps_nav_data_t *gps,
                                 const optflow_sample_t *optflow,
                                 float baro_alt_m,
                                 bool baro_valid,
                                 float tof_range_m,
                                 bool tof_valid,
                                 const ahrs_state_t *ahrs);

#ifdef __cplusplus
}
#endif

#endif /* EKF_NAV_H */

