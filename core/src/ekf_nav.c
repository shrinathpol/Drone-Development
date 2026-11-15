#include "ekf_nav.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD_F 0.017453292519943295f
#define RAD_TO_DEG_F 57.29577951308232f
#define EARTH_R_M    6371000.0
#define GRAVITY_MSS  9.80665f

static float wrap_pi(float a)
{
    while (a > 3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

static void scalar_update(ekf_nav_t *ekf, size_t idx, float innov, float r_var)
{
    if (idx >= EKF_STATE_DIM || r_var <= 1e-6f) {
        return;
    }
    float s = ekf->p_diag[idx] + r_var;
    float k = ekf->p_diag[idx] / s;
    ekf->x[idx] += k * innov;
    ekf->p_diag[idx] = (1.0f - k) * ekf->p_diag[idx];
    if (ekf->p_diag[idx] < 1e-5f) {
        ekf->p_diag[idx] = 1e-5f;
    }
}

void ekf_nav_init(ekf_nav_t *ekf)
{
    if (ekf == NULL) {
        return;
    }
    memset(ekf, 0, sizeof(*ekf));
    for (size_t i = 0U; i < 3U; i++) {
        ekf->p_diag[i] = 0.05f;
        ekf->p_diag[i + 3U] = 1.0f;
        ekf->p_diag[i + 6U] = 4.0f;
        ekf->p_diag[i + 9U] = 0.001f;
        ekf->p_diag[i + 12U] = 0.02f;
    }
}

void ekf_nav_set_origin(ekf_nav_t *ekf, double lat, double lon)
{
    if (ekf == NULL) {
        return;
    }
    ekf->origin_lat = lat;
    ekf->origin_lon = lon;
    ekf->origin_set = true;
    ekf->est_lat = lat;
    ekf->est_lon = lon;
}

void ekf_nav_predict(ekf_nav_t *ekf,
                     const float gyro_rad_s[3],
                     const float accel_m_s2[3],
                     const ahrs_state_t *ahrs,
                     float dt_s)
{
    if (ekf == NULL || dt_s <= 0.0f || dt_s > 0.1f) {
        return;
    }

    /* Attitude propagation with bias subtraction */
    float gx = gyro_rad_s[0] - ekf->x[9];
    float gy = gyro_rad_s[1] - ekf->x[10];
    float gz = gyro_rad_s[2] - ekf->x[11];

    ekf->x[0] = wrap_pi(ekf->x[0] + gx * dt_s);
    ekf->x[1] = wrap_pi(ekf->x[1] + gy * dt_s);
    ekf->x[2] = wrap_pi(ekf->x[2] + gz * dt_s);

    /* Gently anchor attitude covariance to Mahony 9-DOF quaternion output */
    if (ahrs != NULL) {
        float ar = ahrs->roll_deg * DEG_TO_RAD_F;
        float ap = ahrs->pitch_deg * DEG_TO_RAD_F;
        float ay = ahrs->yaw_deg * DEG_TO_RAD_F;
        scalar_update(ekf, 0U, wrap_pi(ar - ekf->x[0]), 0.02f);
        scalar_update(ekf, 1U, wrap_pi(ap - ekf->x[1]), 0.02f);
        scalar_update(ekf, 2U, wrap_pi(ay - ekf->x[2]), 0.05f);
    }

    float cr = cosf(ekf->x[0]), sr = sinf(ekf->x[0]);
    float cp = cosf(ekf->x[1]), sp = sinf(ekf->x[1]);
    float cy = cosf(ekf->x[2]), sy = sinf(ekf->x[2]);

    float ax = accel_m_s2[0] - ekf->x[12];
    float ay = accel_m_s2[1] - ekf->x[13];
    float az = accel_m_s2[2] - ekf->x[14];

    /* Body-to-NED linear acceleration */
    float r11 = cp * cy;
    float r12 = sr * sp * cy - cr * sy;
    float r13 = cr * sp * cy + sr * sy;
    float r21 = cp * sy;
    float r22 = sr * sp * sy + cr * cy;
    float r23 = cr * sp * sy - sr * cy;
    float r31 = -sp;
    float r32 = sr * cp;
    float r33 = cr * cp;

    float a_n = r11 * ax + r12 * ay + r13 * az;
    float a_e = r21 * ax + r22 * ay + r23 * az;
    float a_d = -(r31 * ax + r32 * ay + r33 * az - GRAVITY_MSS);

    ekf->x[3] += a_n * dt_s;
    ekf->x[4] += a_e * dt_s;
    ekf->x[5] += a_d * dt_s;

    /* Mild velocity damping against unmodeled drag when dead-reckoning */
    ekf->x[3] *= (1.0f - 0.08f * dt_s);
    ekf->x[4] *= (1.0f - 0.08f * dt_s);

    ekf->x[6] += ekf->x[3] * dt_s;
    ekf->x[7] += ekf->x[4] * dt_s;
    ekf->x[8] += ekf->x[5] * dt_s;

    /* Process noise growth */
    for (size_t i = 0U; i < 3U; i++) {
        ekf->p_diag[i] += 0.0005f * dt_s;
        ekf->p_diag[i + 3U] += 0.25f * dt_s;
        ekf->p_diag[i + 6U] += 0.05f * dt_s;
        ekf->p_diag[i + 9U] += 1e-6f * dt_s;
        ekf->p_diag[i + 12U] += 1e-4f * dt_s;
    }

    /* Compute global lat/lon estimate from NED meters */
    if (ekf->origin_set) {
        double cos_lat = cos(ekf->origin_lat * (double)DEG_TO_RAD_F);
        if (cos_lat < 0.1) cos_lat = 0.1;
        ekf->est_lat = ekf->origin_lat + ((double)ekf->x[6] / EARTH_R_M) * (double)RAD_TO_DEG_F;
        ekf->est_lon = ekf->origin_lon + ((double)ekf->x[7] / (EARTH_R_M * cos_lat)) * (double)RAD_TO_DEG_F;
    }
    ekf->est_alt_agl_m = -ekf->x[8];
    ekf->ground_speed_ms = sqrtf(ekf->x[3] * ekf->x[3] + ekf->x[4] * ekf->x[4]);
}

void ekf_nav_update_measurements(ekf_nav_t *ekf,
                                 const gps_nav_data_t *gps,
                                 const optflow_sample_t *optflow,
                                 float baro_alt_m,
                                 bool baro_valid,
                                 float tof_range_m,
                                 bool tof_valid,
                                 const ahrs_state_t *ahrs)
{
    if (ekf == NULL) {
        return;
    }

    bool gps_good = (gps != NULL && gps->fix_valid && gps->satellites >= 6U && gps->hdop <= 2.5f);

    if (gps_good) {
        ekf->using_optflow_dead_reckon = false;
        if (!ekf->origin_set) {
            ekf_nav_set_origin(ekf, gps->latitude, gps->longitude);
        }
        double cos_lat = cos(ekf->origin_lat * (double)DEG_TO_RAD_F);
        if (cos_lat < 0.1) cos_lat = 0.1;
        float meas_pn = (float)((gps->latitude - ekf->origin_lat) * (double)DEG_TO_RAD_F * EARTH_R_M);
        float meas_pe = (float)((gps->longitude - ekf->origin_lon) * (double)DEG_TO_RAD_F * EARTH_R_M * cos_lat);

        float pos_var = (gps->hdop * gps->hdop) * 1.5f;
        if (pos_var < 0.8f) pos_var = 0.8f;

        scalar_update(ekf, 6U, meas_pn - ekf->x[6], pos_var);
        scalar_update(ekf, 7U, meas_pe - ekf->x[7], pos_var);
        scalar_update(ekf, 3U, gps->vel_n_ms - ekf->x[3], 0.35f);
        scalar_update(ekf, 4U, gps->vel_e_ms - ekf->x[4], 0.35f);
    } else if (optflow != NULL && optflow->valid) {
        /* GPS denied or jammed: switch velocity fusion to PMW3901 Optical Flow dead-reckoning */
        ekf->using_optflow_dead_reckon = true;
        float cy = cosf(ekf->x[2]);
        float sy = sinf(ekf->x[2]);
        float flow_vn = cy * optflow->vel_body_x_ms - sy * optflow->vel_body_y_ms;
        float flow_ve = sy * optflow->vel_body_x_ms + cy * optflow->vel_body_y_ms;

        scalar_update(ekf, 3U, flow_vn - ekf->x[3], 0.08f);
        scalar_update(ekf, 4U, flow_ve - ekf->x[4], 0.08f);
    } else {
        ekf->using_optflow_dead_reckon = false;
    }

    /* Fuse Vertical Altitude (ToF preferred near ground, otherwise Baro/AHRS) */
    if (tof_valid && tof_range_m > 0.02f && tof_range_m < 3.8f) {
        scalar_update(ekf, 8U, (-tof_range_m) - ekf->x[8], 0.04f);
    } else if (baro_valid) {
        scalar_update(ekf, 8U, (-baro_alt_m) - ekf->x[8], 0.35f);
    } else if (ahrs != NULL) {
        scalar_update(ekf, 8U, (-ahrs->alt_agl_m) - ekf->x[8], 0.50f);
    }
}

