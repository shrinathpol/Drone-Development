#ifndef ENV_SENSORS_H
#define ENV_SENSORS_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pressure_pa;
    float temp_c;
    float altitude_m;
    float baseline_pa;
    bool valid;
} baro_data_t;

typedef struct {
    float mag_ut[3];
    float hard_iron[3];
    float soft_iron[3];
    bool valid;
} mag_data_t;

typedef struct {
    float range_m;
    uint16_t range_mm;
    bool valid;
} tof_data_t;

typedef struct {
    uint16_t vbat_mv;
    int32_t current_ma;
    float power_w;
    float mah_consumed;
    float wh_consumed;
    bool valid;
} power_data_t;

typedef struct {
    hal_i2c_t *i2c;
    hal_adc_t *adc;
    hal_sys_t *sys;

    bool bmp390_ok;
    bool qmc5883l_ok;
    bool vl53l1x_ok;
    bool ina226_ok;
    uint8_t bmp390_addr;

    /* BMP390 calibration parameters */
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin;

    baro_data_t baro;
    mag_data_t mag;
    tof_data_t tof;
    power_data_t power;

    uint32_t last_power_ms;
    float shunt_ohms;

    /* Optional virtual physics override for SITL */
    bool sim_override;
} env_sensors_t;

int env_sensors_init(env_sensors_t *env, hal_i2c_t *i2c, hal_adc_t *adc, hal_sys_t *sys);
void env_sensors_poll_100hz(env_sensors_t *env);
void env_sensors_poll_20hz(env_sensors_t *env);

#ifdef __cplusplus
}
#endif

#endif /* ENV_SENSORS_H */
