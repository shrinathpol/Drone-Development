#ifndef SITL_PHYSICS_H
#define SITL_PHYSICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double lat;
    double lon;
    float pn_m;
    float pe_m;
    float pd_m; /* Negative altitude AGL */
    float vn_ms;
    float ve_ms;
    float vd_ms;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_rad_s[3];
    float accel_body_m_s2[3];

    uint16_t motor_pwm_us[4];
    float wind_n_ms;
    float wind_e_ms;
    float vbat_mv;
    float current_a;
    float mass_kg;

    uint64_t sim_time_us;
} sitl_physics_t;

void sitl_physics_init(sitl_physics_t *p, double home_lat, double home_lon);
void sitl_physics_step(sitl_physics_t *p, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* SITL_PHYSICS_H */

