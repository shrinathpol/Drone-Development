#include "flight_pid.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD_F 0.017453292519943295f
#define RAD_TO_DEG_F 57.29577951308232f
#define EARTH_R_M    6371000.0

static float clampf_p(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float i_max, float out_max, float d_lpf_alpha)
{
    if (pid == NULL) {
        return;
    }
    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
    pid->d_lpf_alpha = clampf_p(d_lpf_alpha, 0.05f, 1.0f);
}

void pid_reset(pid_ctrl_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filtered = 0.0f;
    pid->initialized = false;
}

float pid_step(pid_ctrl_t *pid, float setpoint, float measurement, float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f || dt_s > 0.2f) {
        return 0.0f;
    }

    float err = setpoint - measurement;
    float p_term = pid->kp * err;

    pid->integral = clampf_p(pid->integral + pid->ki * err * dt_s, -pid->i_max, pid->i_max);

    float d_raw = 0.0f;
    if (pid->initialized) {
        /* Derivative on measurement eliminates setpoint-kick spikes */
        d_raw = -(measurement - pid->prev_meas) / dt_s;
    } else {
        pid->initialized = true;
    }
    pid->prev_meas = measurement;

    pid->d_filtered += pid->d_lpf_alpha * (d_raw - pid->d_filtered);
    float d_term = pid->kd * pid->d_filtered;

    return clampf_p(p_term + pid->integral + d_term, -pid->out_max, pid->out_max);
}

void flight_pid_init_defaults(flight_pid_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    pid_init(&ctx->rate_roll, 0.0022f, 0.0015f, 0.00008f, 0.15f, 0.45f, 0.25f);
    pid_init(&ctx->rate_pitch, 0.0022f, 0.0015f, 0.00008f, 0.15f, 0.45f, 0.25f);
    pid_init(&ctx->rate_yaw, 0.0040f, 0.0020f, 0.00002f, 0.20f, 0.40f, 0.20f);

    ctx->angle_kp_roll = 4.5f;
    ctx->angle_kp_pitch = 4.5f;
    ctx->max_tilt_deg = 30.0f;
    ctx->max_rate_rp_dps = 220.0f;
    ctx->max_rate_yaw_dps = 150.0f;

    pid_init(&ctx->alt_pid, 1.2f, 0.05f, 0.0f, 1.0f, 3.0f, 0.3f);
    pid_init(&ctx->climb_pid, 0.14f, 0.04f, 0.01f, 0.20f, 0.35f, 0.25f);
    ctx->hover_throttle = 0.48f;

    pid_init(&ctx->pos_n_pid, 0.65f, 0.05f, 0.18f, 2.0f, 6.0f, 0.3f);
    pid_init(&ctx->pos_e_pid, 0.65f, 0.05f, 0.18f, 2.0f, 6.0f, 0.3f);
    ctx->max_horiz_speed_ms = 8.0f;
}

void flight_pid_reset_all(flight_pid_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    pid_reset(&ctx->rate_roll);
    pid_reset(&ctx->rate_pitch);
    pid_reset(&ctx->rate_yaw);
    pid_reset(&ctx->alt_pid);
    pid_reset(&ctx->climb_pid);
    pid_reset(&ctx->pos_n_pid);
    pid_reset(&ctx->pos_e_pid);
}

void flight_pid_attitude_step(flight_pid_ctx_t *ctx,
                              float target_roll_deg,
                              float target_pitch_deg,
                              float target_yaw_rate_dps,
                              float cur_roll_deg,
                              float cur_pitch_deg,
                              const float gyro_rad_s[3],
                              float dt_s,
                              float *out_roll,
                              float *out_pitch,
                              float *out_yaw)
{
    float tr = clampf_p(target_roll_deg, -ctx->max_tilt_deg, ctx->max_tilt_deg);
    float tp = clampf_p(target_pitch_deg, -ctx->max_tilt_deg, ctx->max_tilt_deg);

    float sp_roll_dps = clampf_p((tr - cur_roll_deg) * ctx->angle_kp_roll,
                                 -ctx->max_rate_rp_dps, ctx->max_rate_rp_dps);
    float sp_pitch_dps = clampf_p((tp - cur_pitch_deg) * ctx->angle_kp_pitch,
                                  -ctx->max_rate_rp_dps, ctx->max_rate_rp_dps);
    float sp_yaw_dps = clampf_p(target_yaw_rate_dps, -ctx->max_rate_yaw_dps, ctx->max_rate_yaw_dps);

    float gx_dps = gyro_rad_s[0] * RAD_TO_DEG_F;
    float gy_dps = gyro_rad_s[1] * RAD_TO_DEG_F;
    float gz_dps = gyro_rad_s[2] * RAD_TO_DEG_F;

    *out_roll = pid_step(&ctx->rate_roll, sp_roll_dps, gx_dps, dt_s);
    *out_pitch = pid_step(&ctx->rate_pitch, sp_pitch_dps, gy_dps, dt_s);
    *out_yaw = pid_step(&ctx->rate_yaw, sp_yaw_dps, gz_dps, dt_s);
}

float flight_pid_altitude_step(flight_pid_ctx_t *ctx,
                               float target_alt_m,
                               float cur_alt_m,
                               float cur_climb_ms,
                               float dt_s)
{
    float target_climb = pid_step(&ctx->alt_pid, target_alt_m, cur_alt_m, dt_s);
    float thr_delta = pid_step(&ctx->climb_pid, target_climb, cur_climb_ms, dt_s);
    return clampf_p(ctx->hover_throttle + thr_delta, 0.12f, 0.90f);
}

void flight_pid_nav_to_ned(flight_pid_ctx_t *ctx,
                           float target_pn_m,
                           float target_pe_m,
                           float cur_pn_m,
                           float cur_pe_m,
                           float cur_vn_ms,
                           float cur_ve_ms,
                           float cur_yaw_deg,
                           float dt_s,
                           float *out_target_roll_deg,
                           float *out_target_pitch_deg)
{
    /* Outer-loop position -> desired NED velocity */
    float des_vn = clampf_p((target_pn_m - cur_pn_m) * 0.7f, -ctx->max_horiz_speed_ms, ctx->max_horiz_speed_ms);
    float des_ve = clampf_p((target_pe_m - cur_pe_m) * 0.7f, -ctx->max_horiz_speed_ms, ctx->max_horiz_speed_ms);

    /* Velocity error -> desired NED acceleration / tilt degrees */
    float accel_n = pid_step(&ctx->pos_n_pid, des_vn, cur_vn_ms, dt_s);
    float accel_e = pid_step(&ctx->pos_e_pid, des_ve, cur_ve_ms, dt_s);

    float yaw_rad = cur_yaw_deg * DEG_TO_RAD_F;
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    /* Rotate NED acceleration to body forward/right tilt angles */
    float accel_fwd = accel_n * cy + accel_e * sy;
    float accel_right = -accel_n * sy + accel_e * cy;

    *out_target_pitch_deg = clampf_p(-accel_fwd * 4.2f, -ctx->max_tilt_deg, ctx->max_tilt_deg);
    *out_target_roll_deg = clampf_p(accel_right * 4.2f, -ctx->max_tilt_deg, ctx->max_tilt_deg);
}

double nav_haversine_distance_m(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = (lat2 - lat1) * (double)DEG_TO_RAD_F;
    double dlon = (lon2 - lon1) * (double)DEG_TO_RAD_F;
    double a = sin(dlat * 0.5) * sin(dlat * 0.5) +
               cos(lat1 * (double)DEG_TO_RAD_F) * cos(lat2 * (double)DEG_TO_RAD_F) *
               sin(dlon * 0.5) * sin(dlon * 0.5);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_R_M * c;
}

double nav_bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double phi1 = lat1 * (double)DEG_TO_RAD_F;
    double phi2 = lat2 * (double)DEG_TO_RAD_F;
    double dlam = (lon2 - lon1) * (double)DEG_TO_RAD_F;
    double y = sin(dlam) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlam);
    double brng = atan2(y, x) * (double)RAD_TO_DEG_F;
    if (brng < 0.0) brng += 360.0;
    return brng;
}

void nav_latlon_to_ned_offset(double origin_lat, double origin_lon,
                              double target_lat, double target_lon,
                              float *out_pn_m, float *out_pe_m)
{
    double cos_lat = cos(origin_lat * (double)DEG_TO_RAD_F);
    if (cos_lat < 0.1) cos_lat = 0.1;
    *out_pn_m = (float)((target_lat - origin_lat) * (double)DEG_TO_RAD_F * EARTH_R_M);
    *out_pe_m = (float)((target_lon - origin_lon) * (double)DEG_TO_RAD_F * EARTH_R_M * cos_lat);
}

