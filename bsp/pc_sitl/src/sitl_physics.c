#include "sitl_physics.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD_D 0.017453292519943295
#define RAD_TO_DEG_D 57.29577951308232
#define EARTH_R_M    6371000.0
#define GRAVITY_MSS  9.80665f

void sitl_physics_init(sitl_physics_t *p, double home_lat, double home_lon)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->lat = home_lat;
    p->lon = home_lon;
    p->vbat_mv = 12450.0f; /* Full 3S LiPo */
    p->mass_kg = 1.25f;
    p->wind_n_ms = 1.5f;
    p->wind_e_ms = -0.8f;
    for (int i = 0; i < 4; i++) {
        p->motor_pwm_us[i] = 1000U;
    }
    p->accel_body_m_s2[2] = GRAVITY_MSS;
}

void sitl_physics_step(sitl_physics_t *p, float dt_s)
{
    if (p == NULL || dt_s <= 0.0f) {
        return;
    }

    float t_norm[4];
    float total_thrust = 0.0f;
    float total_curr = 0.8f; /* Idle avionics draw */

    for (int i = 0; i < 4; i++) {
        float duty = (p->motor_pwm_us[i] > 1000U) ? ((float)(p->motor_pwm_us[i] - 1000U) / 1000.0f) : 0.0f;
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;
        t_norm[i] = duty;
        total_thrust += duty * 6.4f; /* ~25.6N max total thrust for 1.25kg quad (hover ~0.48) */
        total_curr += duty * 8.5f;
    }

    p->current_a = total_curr;
    p->vbat_mv -= total_curr * dt_s * 0.015f;
    if (p->vbat_mv < 9000.0f) p->vbat_mv = 9000.0f;

    /* Quad-X torques matching motor_esc.c */
    float roll_torque = (-t_norm[0] + t_norm[1] + t_norm[2] - t_norm[3]) * 18.0f;
    float pitch_torque = (-t_norm[0] + t_norm[1] - t_norm[2] + t_norm[3]) * 18.0f;
    float yaw_torque = (-t_norm[0] - t_norm[1] + t_norm[2] + t_norm[3]) * 9.0f;

    /* Ground contact clamp */
    if (p->pd_m >= -0.001f && total_thrust < p->mass_kg * GRAVITY_MSS) {
        p->gyro_rad_s[0] = 0.0f;
        p->gyro_rad_s[1] = 0.0f;
        p->gyro_rad_s[2] = 0.0f;
        p->roll_rad *= 0.8f;
        p->pitch_rad *= 0.8f;
        p->vn_ms = 0.0f;
        p->ve_ms = 0.0f;
        p->vd_ms = 0.0f;
        p->pd_m = 0.0f;
        p->accel_body_m_s2[0] = 0.0f;
        p->accel_body_m_s2[1] = 0.0f;
        p->accel_body_m_s2[2] = GRAVITY_MSS;
        p->sim_time_us += (uint64_t)(dt_s * 1e6f);
        return;
    }

    /* Rotational damping + integration */
    p->gyro_rad_s[0] += (roll_torque - p->gyro_rad_s[0] * 4.5f) * dt_s;
    p->gyro_rad_s[1] += (pitch_torque - p->gyro_rad_s[1] * 4.5f) * dt_s;
    p->gyro_rad_s[2] += (yaw_torque - p->gyro_rad_s[2] * 3.0f) * dt_s;

    p->roll_rad += p->gyro_rad_s[0] * dt_s;
    p->pitch_rad += p->gyro_rad_s[1] * dt_s;
    p->yaw_rad += p->gyro_rad_s[2] * dt_s;

    /* Thrust vector in NED */
    float sp = sinf(p->pitch_rad), cp = cosf(p->pitch_rad);
    float sr = sinf(p->roll_rad), cr = cosf(p->roll_rad);
    float sy = sinf(p->yaw_rad), cy = cosf(p->yaw_rad);

    float spec_thrust = total_thrust / p->mass_kg;
    float accel_n = spec_thrust * (-sp * cy + sr * sy) - (p->vn_ms - p->wind_n_ms) * 0.35f;
    float accel_e = spec_thrust * (-sp * sy - sr * cy) + (sr * 0.0f) - (p->ve_ms - p->wind_e_ms) * 0.35f;
    float accel_up = spec_thrust * (cp * cr) - GRAVITY_MSS - p->vd_ms * (-0.25f);

    p->vn_ms += accel_n * dt_s;
    p->ve_ms += accel_e * dt_s;
    p->vd_ms -= accel_up * dt_s;

    p->pn_m += p->vn_ms * dt_s;
    p->pe_m += p->ve_ms * dt_s;
    p->pd_m += p->vd_ms * dt_s;

    if (p->pd_m > 0.0f) {
        p->pd_m = 0.0f;
        if (p->vd_ms > 0.0f) p->vd_ms = 0.0f;
    }

    /* Body accelerometer senses specific force (thrust along +Z_body + reaction to gravity) */
    p->accel_body_m_s2[0] = -GRAVITY_MSS * sp;
    p->accel_body_m_s2[1] = GRAVITY_MSS * sr * cp;
    p->accel_body_m_s2[2] = spec_thrust * cp * cr;
    if (p->pd_m >= 0.0f && p->accel_body_m_s2[2] < GRAVITY_MSS) {
        p->accel_body_m_s2[2] = GRAVITY_MSS;
    }

    p->sim_time_us += (uint64_t)(dt_s * 1e6f);
}

