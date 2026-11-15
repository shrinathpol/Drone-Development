#include "mission_engine.h"
#include "flight_pid.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEG_TO_RAD_D 0.017453292519943295
#define EARTH_R_M    6371000.0

static geo_point_t s_fallback_breadcrumbs[64];

int mission_engine_init(mission_engine_t *m, hal_sys_t *sys)
{
    if (m == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(m, 0, sizeof(*m));
    m->sys = sys;
    m->fence.max_alt_agl_m = 120.0f;
    m->fence.fence_margin_m = 8.0f;
    m->breadcrumb_cap = MISSION_MAX_BREADCRUMBS;

    size_t bytes = m->breadcrumb_cap * sizeof(geo_point_t);
    if (sys != NULL && sys->alloc_psram != NULL) {
        m->breadcrumbs = (geo_point_t *)sys->alloc_psram(bytes);
    } else {
        m->breadcrumbs = (geo_point_t *)malloc(bytes);
    }
    if (m->breadcrumbs == NULL) {
        m->breadcrumbs = s_fallback_breadcrumbs;
        m->breadcrumb_cap = 64U;
    }
    return HAL_OK;
}

void mission_engine_set_home(mission_engine_t *m, double lat, double lon, float alt_m)
{
    if (m == NULL) {
        return;
    }
    m->home.lat = lat;
    m->home.lon = lon;
    m->home.alt_m = alt_m;
    m->home_set = true;

    if (m->fence.keep_in_count == 0U) {
        mission_engine_set_default_geofence(m, lat, lon, 500.0f);
    }
}

void mission_engine_set_default_geofence(mission_engine_t *m, double center_lat, double center_lon, float radius_m)
{
    if (m == NULL) {
        return;
    }
    double dlat = ((double)radius_m / EARTH_R_M) / DEG_TO_RAD_D;
    double cos_lat = cos(center_lat * DEG_TO_RAD_D);
    if (cos_lat < 0.1) cos_lat = 0.1;
    double dlon = ((double)radius_m / (EARTH_R_M * cos_lat)) / DEG_TO_RAD_D;

    /* 8-point octagonal Keep-In polygon */
    m->fence.keep_in_count = 8U;
    for (size_t i = 0U; i < 8U; i++) {
        double ang = (double)i * (6.28318530718 / 8.0);
        m->fence.keep_in_poly[i].lat = center_lat + dlat * sin(ang);
        m->fence.keep_in_poly[i].lon = center_lon + dlon * cos(ang);
        m->fence.keep_in_poly[i].alt_m = m->fence.max_alt_agl_m;
    }
}

/* Ray-Casting Point-in-Polygon (x=lon, y=lat) */
bool mission_point_in_polygon(const geo_point_t *poly, size_t n, double lat, double lon)
{
    if (poly == NULL || n < 3U) {
        return true;
    }
    bool inside = false;
    for (size_t i = 0U, j = n - 1U; i < n; j = i++) {
        double yi = poly[i].lat, xi = poly[i].lon;
        double yj = poly[j].lat, xj = poly[j].lon;
        bool intersect = ((yi > lat) != (yj > lat)) &&
                         (lon < (xj - xi) * (lat - yi) / ((yj - yi) + 1e-12) + xi);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

void mission_record_breadcrumb(mission_engine_t *m, double lat, double lon, float alt_agl_m)
{
    if (m == NULL || m->breadcrumbs == NULL || (lat == 0.0 && lon == 0.0)) {
        return;
    }
    if (m->breadcrumb_count > 0U) {
        const geo_point_t *last = &m->breadcrumbs[m->breadcrumb_count - 1U];
        double dist = nav_haversine_distance_m(last->lat, last->lon, lat, lon);
        if (dist < 8.0 && fabsf(last->alt_m - alt_agl_m) < 4.0f) {
            return;
        }
    }

    if (m->breadcrumb_count >= m->breadcrumb_cap) {
        mission_simplify_breadcrumbs_dp(m, 4.0f);
        if (m->breadcrumb_count >= m->breadcrumb_cap) {
            /* Drop second oldest to keep HOME at index 0 */
            memmove(&m->breadcrumbs[1], &m->breadcrumbs[2], (m->breadcrumb_cap - 2U) * sizeof(geo_point_t));
            m->breadcrumb_count--;
        }
    }

    m->breadcrumbs[m->breadcrumb_count].lat = lat;
    m->breadcrumbs[m->breadcrumb_count].lon = lon;
    m->breadcrumbs[m->breadcrumb_count].alt_m = alt_agl_m;
    m->breadcrumb_count++;
}

static float point_to_segment_dist_m(const geo_point_t *p, const geo_point_t *a, const geo_point_t *b)
{
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f, px = 0.0f, py = 0.0f;
    nav_latlon_to_ned_offset(a->lat, a->lon, b->lat, b->lon, &bx, &by);
    nav_latlon_to_ned_offset(a->lat, a->lon, p->lat, p->lon, &px, &py);
    float dx = bx - ax, dy = by - ay;
    float len2 = dx * dx + dy * dy;
    if (len2 < 1e-4f) {
        return sqrtf(px * px + py * py);
    }
    float t = (px * dx + py * dy) / len2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float cx = t * dx - px;
    float cy = t * dy - py;
    return sqrtf(cx * cx + cy * cy);
}

void mission_simplify_breadcrumbs_dp(mission_engine_t *m, float epsilon_m)
{
    if (m == NULL || m->breadcrumbs == NULL || m->breadcrumb_count < 4U) {
        return;
    }

    bool keep[MISSION_MAX_BREADCRUMBS];
    memset(keep, 0, sizeof(keep));
    size_t n = m->breadcrumb_count;
    keep[0] = true;
    keep[n - 1U] = true;

    /* Iterative Douglas-Peucker stack (no recursion) */
    size_t stack_l[64];
    size_t stack_r[64];
    int sp = 0;
    stack_l[sp] = 0U;
    stack_r[sp] = n - 1U;
    sp++;

    while (sp > 0) {
        sp--;
        size_t l = stack_l[sp];
        size_t r = stack_r[sp];
        if (r <= l + 1U) {
            continue;
        }
        float max_d = 0.0f;
        size_t best_idx = l;
        for (size_t i = l + 1U; i < r; i++) {
            float d = point_to_segment_dist_m(&m->breadcrumbs[i], &m->breadcrumbs[l], &m->breadcrumbs[r]);
            if (d > max_d) {
                max_d = d;
                best_idx = i;
            }
        }
        if (max_d > epsilon_m) {
            keep[best_idx] = true;
            if (sp < 62) {
                stack_l[sp] = l; stack_r[sp] = best_idx; sp++;
                stack_l[sp] = best_idx; stack_r[sp] = r; sp++;
            }
        }
    }

    size_t out_n = 0U;
    for (size_t i = 0U; i < n; i++) {
        if (keep[i]) {
            m->breadcrumbs[out_n++] = m->breadcrumbs[i];
        }
    }
    m->breadcrumb_count = out_n;
}

void mission_begin_rth_backtrack(mission_engine_t *m)
{
    if (m == NULL) {
        return;
    }
    mission_simplify_breadcrumbs_dp(m, 3.0f);
    m->backtracking_rth = true;
    m->backtrack_idx = (m->breadcrumb_count > 0U) ? (m->breadcrumb_count - 1U) : 0U;
}

bool mission_get_active_target(mission_engine_t *m,
                               double cur_lat,
                               double cur_lon,
                               float cur_alt_m,
                               geo_point_t *out_target)
{
    (void)cur_alt_m;
    if (m == NULL || out_target == NULL) {
        return false;
    }

    if (m->backtracking_rth) {
        if (m->breadcrumb_count > 0U && m->backtrack_idx < m->breadcrumb_count) {
            geo_point_t wp = m->breadcrumbs[m->backtrack_idx];
            double d = nav_haversine_distance_m(cur_lat, cur_lon, wp.lat, wp.lon);
            if (d < 4.5 && m->backtrack_idx > 0U) {
                m->backtrack_idx--;
                wp = m->breadcrumbs[m->backtrack_idx];
            }
            *out_target = wp;
            return true;
        }
        if (m->home_set) {
            *out_target = m->home;
            return true;
        }
        return false;
    }

    if (m->mission_active && m->waypoint_count > 0U && m->active_wp_idx < m->waypoint_count) {
        geo_point_t wp = m->waypoints[m->active_wp_idx];
        double d = nav_haversine_distance_m(cur_lat, cur_lon, wp.lat, wp.lon);
        if (d < 4.0) {
            if (m->active_wp_idx + 1U < m->waypoint_count) {
                m->active_wp_idx++;
                wp = m->waypoints[m->active_wp_idx];
            } else {
                m->mission_active = false;
            }
        }
        *out_target = wp;
        return true;
    }

    return false;
}

void mission_evaluate_geofence_10hz(mission_engine_t *m,
                                    double cur_lat,
                                    double cur_lon,
                                    float cur_alt_agl_m,
                                    float cur_vn_ms,
                                    float cur_ve_ms)
{
    if (m == NULL) {
        return;
    }
    m->geofence_breach = false;
    m->bumper_vn_ms = 0.0f;
    m->bumper_ve_ms = 0.0f;
    m->clamped_alt_target_m = m->fence.max_alt_agl_m;

    if (cur_alt_agl_m >= m->fence.max_alt_agl_m) {
        m->geofence_breach = true;
    }

    bool outside_keep_in = (m->fence.keep_in_count >= 3U) &&
                           !mission_point_in_polygon(m->fence.keep_in_poly, m->fence.keep_in_count, cur_lat, cur_lon);
    bool inside_keep_out = (m->fence.keep_out_count >= 3U) &&
                           mission_point_in_polygon(m->fence.keep_out_poly, m->fence.keep_out_count, cur_lat, cur_lon);

    if (outside_keep_in || inside_keep_out) {
        m->geofence_breach = true;
        if (m->home_set) {
            /* Virtual rubber-bumper vector pulling back toward safe Home interior */
            float pn = 0.0f, pe = 0.0f;
            nav_latlon_to_ned_offset(cur_lat, cur_lon, m->home.lat, m->home.lon, &pn, &pe);
            float dist = sqrtf(pn * pn + pe * pe);
            if (dist > 0.5f) {
                m->bumper_vn_ms = (pn / dist) * 4.5f - cur_vn_ms * 0.4f;
                m->bumper_ve_ms = (pe / dist) * 4.5f - cur_ve_ms * 0.4f;
            }
        }
    }
}

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
                                       float vbat_mv)
{
    if (m == NULL || !m->home_set) {
        return;
    }

    /* Airspeed tilt-vector model: pitch/roll tilt produces predictable airspeed */
    float tilt_fwd_deg = -pitch_deg;
    float tilt_right_deg = roll_deg;
    float air_fwd = tilt_fwd_deg * 0.45f;
    float air_right = tilt_right_deg * 0.45f;

    float yaw_rad = yaw_deg * (float)DEG_TO_RAD_D;
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);
    float air_vn = air_fwd * cy - air_right * sy;
    float air_ve = air_fwd * sy + air_right * cy;

    m->wind_n_ms = 0.98f * m->wind_n_ms + 0.02f * (vn_ms - air_vn);
    m->wind_e_ms = 0.98f * m->wind_e_ms + 0.02f * (ve_ms - air_ve);

    double dist_home = nav_haversine_distance_m(cur_lat, cur_lon, m->home.lat, m->home.lon);
    double brng_rad = nav_bearing_deg(cur_lat, cur_lon, m->home.lat, m->home.lon) * DEG_TO_RAD_D;

    float unit_n = (float)cos(brng_rad);
    float unit_e = (float)sin(brng_rad);
    float tailwind = m->wind_n_ms * unit_n + m->wind_e_ms * unit_e;

    float cruise_air_ms = 8.5f;
    float eff_return_spd = cruise_air_ms + tailwind;
    if (eff_return_spd < 2.5f) {
        eff_return_spd = 2.5f;
    }

    float time_home_h = ((float)dist_home / eff_return_spd) / 3600.0f;
    float avg_power = (power_w > 30.0f) ? power_w : 140.0f;
    /* Add 20% safety reserve + 0.8 Wh for controlled ToF landing */
    m->wh_needed_home = (avg_power * time_home_h) * 1.20f + 0.8f;

    float est_rem_by_wh = (battery_cap_wh > wh_consumed) ? (battery_cap_wh - wh_consumed) : 0.0f;
    m->wh_remaining_est = est_rem_by_wh;

    bool low_volt = (vbat_mv > 3000.0f && vbat_mv < 10500.0f); /* 3S < 3.5V/cell */
    if (dist_home > 15.0 && (est_rem_by_wh <= m->wh_needed_home || low_volt)) {
        m->smart_rth_triggered = true;
    }
}

void mission_add_waypoint(mission_engine_t *m, double lat, double lon, float alt_m, bool clear_existing)
{
    if (m == NULL) {
        return;
    }
    if (clear_existing) {
        m->waypoint_count = 0U;
        m->active_wp_idx = 0U;
    }
    if (m->waypoint_count < MISSION_MAX_WAYPOINTS) {
        m->waypoints[m->waypoint_count].lat = lat;
        m->waypoints[m->waypoint_count].lon = lon;
        m->waypoints[m->waypoint_count].alt_m = (alt_m > 1.5f) ? alt_m : 15.0f;
        m->waypoint_count++;
        m->mission_active = true;
        m->backtracking_rth = false;
    }
}

