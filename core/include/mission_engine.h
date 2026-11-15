#ifndef MISSION_ENGINE_H
#define MISSION_ENGINE_H

#include "hal_interfaces.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MISSION_MAX_GEOFENCE_PTS 16U
#define MISSION_MAX_WAYPOINTS    32U
#define MISSION_MAX_BREADCRUMBS  256U

typedef struct {
    double lat;
    double lon;
    float alt_m;
} geo_point_t;

typedef struct {
    geo_point_t keep_in_poly[MISSION_MAX_GEOFENCE_PTS];
    size_t keep_in_count;
    geo_point_t keep_out_poly[MISSION_MAX_GEOFENCE_PTS];
    size_t keep_out_count;
    float max_alt_agl_m;
    float fence_margin_m;
} geofence_cfg_t;

typedef struct {
    hal_sys_t *sys;
    geofence_cfg_t fence;

    bool home_set;
    geo_point_t home;

    geo_point_t waypoints[MISSION_MAX_WAYPOINTS];
    size_t waypoint_count;
    size_t active_wp_idx;
    bool mission_active;

    /* Breadcrumb trail in PSRAM or RAM */
    geo_point_t *breadcrumbs;
    size_t breadcrumb_cap;
    size_t breadcrumb_count;
    bool backtracking_rth;
    size_t backtrack_idx;

    /* Wind & Smart Battery RTH estimation */
    float wind_n_ms;
    float wind_e_ms;
    float wh_needed_home;
    float wh_remaining_est;
    bool smart_rth_triggered;

    /* Geofence rubber-bumper braking output */
    bool geofence_breach;
    float bumper_vn_ms;
    float bumper_ve_ms;
    float clamped_alt_target_m;
} mission_engine_t;

int mission_engine_init(mission_engine_t *m, hal_sys_t *sys);
void mission_engine_set_home(mission_engine_t *m, double lat, double lon, float alt_m);
void mission_engine_set_default_geofence(mission_engine_t *m, double center_lat, double center_lon, float radius_m);

bool mission_point_in_polygon(const geo_point_t *poly, size_t n, double lat, double lon);

void mission_record_breadcrumb(mission_engine_t *m, double lat, double lon, float alt_agl_m);
void mission_simplify_breadcrumbs_dp(mission_engine_t *m, float epsilon_m);

void mission_begin_rth_backtrack(mission_engine_t *m);
bool mission_get_active_target(mission_engine_t *m,
                               double cur_lat,
                               double cur_lon,
                               float cur_alt_m,
                               geo_point_t *out_target);

void mission_evaluate_geofence_10hz(mission_engine_t *m,
                                    double cur_lat,
                                    double cur_lon,
                                    float cur_alt_agl_m,
                                    float cur_vn_ms,
                                    float cur_ve_ms);

void mission_update_wind_and_smart_rth(mission_engine_t *m,
                                       double cur_lat,
                                       double cur_lon,
                                       float roll_deg,
                                       float pitch_deg,
                                       float yaw_deg,
                                       float vn_ms,
                                       float ve_ms,
                                       float power_w,
                                       float battery_cap_wh,
                                       float wh_consumed,
                                       float vbat_mv);

void mission_add_waypoint(mission_engine_t *m, double lat, double lon, float alt_m, bool clear_existing);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_ENGINE_H */

