#ifndef FLIGHT_PID_H
#define FLIGHT_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float d_lpf_alpha;

    float integral;
    float prev_meas;
    float d_filtered;
    bool initialized;
} pid_ctrl_t;

typedef struct {
    /* Inner Angular Rate PIDs (Roll, Pitch, Yaw) */
    pid_ctrl_t rate_roll;
    pid_ctrl_t rate_pitch;
    pid_ctrl_t rate_yaw;

    /* Outer Attitude Angle P gains */
    float angle_kp_roll;
    float angle_kp_pitch;
    float max_tilt_deg;
    float max_rate_rp_dps;
    float max_rate_yaw_dps;

    /* Vertical Altitude & Climb Rate PIDs */
    pid_ctrl_t alt_pid;
    pid_ctrl_t climb_pid;
    float hover_throttle;

    /* Horizontal GPS/EKF Position Hold & Waypoint Navigation */
    pid_ctrl_t pos_n_pid;
    pid_ctrl_t pos_e_pid;
    float max_horiz_speed_ms;
} flight_pid_ctx_t;

typedef struct {
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float throttle_cmd;
} mixer_cmd_t;

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float i_max, float out_max, float d_lpf_alpha);
void pid_reset(pid_ctrl_t *pid);
float pid_step(pid_ctrl_t *pid, float setpoint, float measurement, float dt_s);

void flight_pid_init_defaults(flight_pid_ctx_t *ctx);
void flight_pid_reset_all(flight_pid_ctx_t *ctx);

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
                              float *out_yaw);

float flight_pid_altitude_step(flight_pid_ctx_t *ctx,
                               float target_alt_m,
                               float cur_alt_m,
                               float cur_climb_ms,
                               float dt_s);

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
                           float *out_target_pitch_deg);

double nav_haversine_distance_m(double lat1, double lon1, double lat2, double lon2);
double nav_bearing_deg(double lat1, double lon1, double lat2, double lon2);
void nav_latlon_to_ned_offset(double origin_lat, double origin_lon,
                              double target_lat, double target_lon,
                              float *out_pn_m, float *out_pe_m);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_PID_H */

