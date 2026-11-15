#include "env_sensors.h"

#include <math.h>
#include <string.h>

#define BMP390_ADDR_PRI 0x77U
#define BMP390_ADDR_SEC 0x76U
#define BMP390_CHIP_ID  0x60U

#define QMC5883L_ADDR   0x0DU
#define VL53L1X_ADDR    0x29U
#define INA226_ADDR     0x40U

static float pressure_to_alt_m(float pressure_pa, float sea_level_pa)
{
    if (pressure_pa <= 1000.0f || sea_level_pa <= 1000.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.19029495f));
}

static bool init_bmp390(env_sensors_t *env)
{
    if (env->i2c == NULL) {
        return false;
    }
    const uint8_t addrs[2] = {BMP390_ADDR_PRI, BMP390_ADDR_SEC};
    for (size_t i = 0U; i < 2U; i++) {
        uint8_t chip = 0U;
        if (env->i2c->read_reg(env->i2c->ctx, addrs[i], 0x00U, &chip, 1U) == HAL_OK && chip == BMP390_CHIP_ID) {
            env->bmp390_addr = addrs[i];
            uint8_t nvm[21];
            if (env->i2c->read_reg(env->i2c->ctx, addrs[i], 0x31U, nvm, sizeof(nvm)) == HAL_OK) {
                uint16_t t1 = (uint16_t)(nvm[0] | (nvm[1] << 8U));
                uint16_t t2 = (uint16_t)(nvm[2] | (nvm[3] << 8U));
                int8_t t3 = (int8_t)nvm[4];
                int16_t p1 = (int16_t)(nvm[5] | (nvm[6] << 8U));
                int16_t p2 = (int16_t)(nvm[7] | (nvm[8] << 8U));
                int8_t p3 = (int8_t)nvm[9];
                int8_t p4 = (int8_t)nvm[10];
                uint16_t p5 = (uint16_t)(nvm[11] | (nvm[12] << 8U));
                uint16_t p6 = (uint16_t)(nvm[13] | (nvm[14] << 8U));
                int8_t p7 = (int8_t)nvm[15];
                int8_t p8 = (int8_t)nvm[16];
                int16_t p9 = (int16_t)(nvm[17] | (nvm[18] << 8U));
                int8_t p10 = (int8_t)nvm[19];
                int8_t p11 = (int8_t)nvm[20];

                env->par_t1 = (float)t1 / 0.00390625f;
                env->par_t2 = (float)t2 / 1073741824.0f;
                env->par_t3 = (float)t3 / 281474976710656.0f;
                env->par_p1 = ((float)p1 - 16384.0f) / 1048576.0f;
                env->par_p2 = ((float)p2 - 16384.0f) / 536870912.0f;
                env->par_p3 = (float)p3 / 4294967296.0f;
                env->par_p4 = (float)p4 / 137438953472.0f;
                env->par_p5 = (float)p5 / 0.125f;
                env->par_p6 = (float)p6 / 64.0f;
                env->par_p7 = (float)p7 / 256.0f;
                env->par_p8 = (float)p8 / 32768.0f;
                env->par_p9 = (float)p9 / 281474976710656.0f;
                env->par_p10 = (float)p10 / 281474976710656.0f;
                env->par_p11 = (float)p11 / 36893488147419103232.0f;
            }
            /* Normal mode, temp + press enabled, OSR x4, IIR coeff 3 */
            uint8_t osr = 0x0AU;
            uint8_t odr = 0x02U;
            uint8_t iir = 0x02U;
            uint8_t pwr = 0x33U;
            env->i2c->write_reg(env->i2c->ctx, addrs[i], 0x1CU, &osr, 1U);
            env->i2c->write_reg(env->i2c->ctx, addrs[i], 0x1DU, &odr, 1U);
            env->i2c->write_reg(env->i2c->ctx, addrs[i], 0x1FU, &iir, 1U);
            env->i2c->write_reg(env->i2c->ctx, addrs[i], 0x1BU, &pwr, 1U);
            return true;
        }
    }
    return false;
}

static void poll_bmp390(env_sensors_t *env)
{
    uint8_t raw[6];
    if (env->i2c->read_reg(env->i2c->ctx, env->bmp390_addr, 0x04U, raw, sizeof(raw)) != HAL_OK) {
        return;
    }
    uint32_t uncomp_p = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8U) | ((uint32_t)raw[2] << 16U);
    uint32_t uncomp_t = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8U) | ((uint32_t)raw[5] << 16U);

    float dt = (float)uncomp_t - env->par_t1;
    env->t_lin = dt * env->par_t2 + (dt * dt) * env->par_t3;

    float pd1 = env->par_p6 * env->t_lin;
    float pd2 = env->par_p7 * (env->t_lin * env->t_lin);
    float pd3 = env->par_p8 * (env->t_lin * env->t_lin * env->t_lin);
    float po1 = env->par_p5 + pd1 + pd2 + pd3;

    float pd4 = env->par_p2 * env->t_lin;
    float pd5 = env->par_p3 * (env->t_lin * env->t_lin);
    float pd6 = env->par_p4 * (env->t_lin * env->t_lin * env->t_lin);
    float po2 = (float)uncomp_p * (env->par_p1 + pd4 + pd5 + pd6);

    float p_sq = (float)uncomp_p * (float)uncomp_p;
    float po3 = p_sq * (env->par_p9 + env->par_p10 * env->t_lin) + (p_sq * (float)uncomp_p) * env->par_p11;

    float comp_p = po1 + po2 + po3;
    if (comp_p > 30000.0f && comp_p < 120000.0f) {
        env->baro.pressure_pa = comp_p;
        env->baro.temp_c = env->t_lin;
        if (env->baro.baseline_pa < 30000.0f) {
            env->baro.baseline_pa = comp_p;
        }
        env->baro.altitude_m = pressure_to_alt_m(comp_p, env->baro.baseline_pa);
        env->baro.valid = true;
    }
}

static bool init_qmc5883l(env_sensors_t *env)
{
    if (env->i2c == NULL) {
        return false;
    }
    uint8_t chip = 0U;
    if (env->i2c->read_reg(env->i2c->ctx, QMC5883L_ADDR, 0x0DU, &chip, 1U) != HAL_OK || chip != 0xFFU) {
        return false;
    }
    uint8_t set_reset = 0x01U;
    uint8_t ctrl1 = 0x1DU; /* Continuous, 200Hz, +-8G, OSR 512 */
    env->i2c->write_reg(env->i2c->ctx, QMC5883L_ADDR, 0x0BU, &set_reset, 1U);
    env->i2c->write_reg(env->i2c->ctx, QMC5883L_ADDR, 0x09U, &ctrl1, 1U);
    return true;
}

static void poll_qmc5883l(env_sensors_t *env)
{
    uint8_t raw[6];
    if (env->i2c->read_reg(env->i2c->ctx, QMC5883L_ADDR, 0x00U, raw, sizeof(raw)) != HAL_OK) {
        return;
    }
    int16_t mx = (int16_t)((raw[1] << 8U) | raw[0]);
    int16_t my = (int16_t)((raw[3] << 8U) | raw[2]);
    int16_t mz = (int16_t)((raw[5] << 8U) | raw[4]);

    /* 3000 LSB/Gauss = 0.0333 uT/LSB (1 Gauss = 100 uT) */
    const float scale = 100.0f / 3000.0f;
    env->mag.mag_ut[0] = ((float)mx * scale - env->mag.hard_iron[0]) * env->mag.soft_iron[0];
    env->mag.mag_ut[1] = ((float)my * scale - env->mag.hard_iron[1]) * env->mag.soft_iron[1];
    env->mag.mag_ut[2] = ((float)mz * scale - env->mag.hard_iron[2]) * env->mag.soft_iron[2];
    env->mag.valid = true;
}

static bool init_vl53l1x(env_sensors_t *env)
{
    if (env->i2c == NULL) {
        return false;
    }
    /* Probe VL53L1X model ID register 0x010F (single-byte 0x0F check fallback on bridge) */
    uint8_t id = 0U;
    if (env->i2c->read_reg(env->i2c->ctx, VL53L1X_ADDR, 0x00U, &id, 1U) != HAL_OK) {
        return false;
    }
    /* Start continuous ranging command 0x87 = 0x40 */
    uint8_t start = 0x40U;
    env->i2c->write_reg(env->i2c->ctx, VL53L1X_ADDR, 0x87U, &start, 1U);
    return true;
}

static void poll_vl53l1x(env_sensors_t *env)
{
    uint8_t buf[2];
    if (env->i2c->read_reg(env->i2c->ctx, VL53L1X_ADDR, 0x96U, buf, 2U) == HAL_OK) {
        uint16_t mm = (uint16_t)((buf[0] << 8U) | buf[1]);
        if (mm >= 20U && mm <= 4200U) {
            env->tof.range_mm = mm;
            env->tof.range_m = (float)mm * 0.001f;
            env->tof.valid = true;
        } else {
            env->tof.valid = false;
        }
    }
}

static bool init_ina226(env_sensors_t *env)
{
    if (env->i2c == NULL) {
        return false;
    }
    uint8_t id[2];
    if (env->i2c->read_reg(env->i2c->ctx, INA226_ADDR, 0xFEU, id, 2U) != HAL_OK) {
        return false;
    }
    uint16_t mfg = (uint16_t)((id[0] << 8U) | id[1]);
    if (mfg != 0x5449U) { /* 'TI' */
        return false;
    }
    uint8_t cfg[2] = {0x45U, 0x27U}; /* 16 avg, 1.1ms, continuous shunt+bus */
    env->i2c->write_reg(env->i2c->ctx, INA226_ADDR, 0x00U, cfg, 2U);
    return true;
}

static void poll_ina226_or_adc(env_sensors_t *env)
{
    uint32_t now = (env->sys != NULL) ? env->sys->millis() : 0U;
    float dt_h = (env->last_power_ms > 0U && now > env->last_power_ms)
                     ? ((float)(now - env->last_power_ms) / 3600000.0f)
                     : 0.0f;
    env->last_power_ms = now;

    if (env->ina226_ok) {
        uint8_t vbuf[2], sbuf[2];
        if (env->i2c->read_reg(env->i2c->ctx, INA226_ADDR, 0x02U, vbuf, 2U) == HAL_OK &&
            env->i2c->read_reg(env->i2c->ctx, INA226_ADDR, 0x01U, sbuf, 2U) == HAL_OK) {
            uint16_t raw_v = (uint16_t)((vbuf[0] << 8U) | vbuf[1]);
            int16_t raw_s = (int16_t)((sbuf[0] << 8U) | sbuf[1]);
            env->power.vbat_mv = (uint16_t)((float)raw_v * 1.25f);
            float shunt_v = (float)raw_s * 0.0000025f; /* 2.5 uV/LSB */
            float amps = shunt_v / env->shunt_ohms;
            if (amps < 0.0f) {
                amps = 0.0f;
            }
            env->power.current_ma = (int32_t)(amps * 1000.0f);
            env->power.power_w = ((float)env->power.vbat_mv * 0.001f) * amps;
            env->power.mah_consumed += (float)env->power.current_ma * dt_h;
            env->power.wh_consumed += env->power.power_w * dt_h;
            env->power.valid = true;
            return;
        }
    }

    if (env->adc != NULL && env->adc->read_mv != NULL) {
        uint32_t mv = env->adc->read_mv(0U);
        if (mv > 0U) {
            env->power.vbat_mv = (uint16_t)mv;
            env->power.valid = (mv > 2500U);
        }
    }
}

int env_sensors_init(env_sensors_t *env, hal_i2c_t *i2c, hal_adc_t *adc, hal_sys_t *sys)
{
    if (env == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    memset(env, 0, sizeof(*env));
    env->i2c = i2c;
    env->adc = adc;
    env->sys = sys;
    env->shunt_ohms = 0.002f; /* 2 mOhm standard drone shunt */
    env->baro.baseline_pa = 101325.0f;
    env->mag.soft_iron[0] = 1.0f;
    env->mag.soft_iron[1] = 1.0f;
    env->mag.soft_iron[2] = 1.0f;

    if (i2c != NULL && i2c->init != NULL) {
        i2c->init(i2c->ctx, 400000U);
        env->bmp390_ok = init_bmp390(env);
        env->qmc5883l_ok = init_qmc5883l(env);
        env->vl53l1x_ok = init_vl53l1x(env);
        env->ina226_ok = init_ina226(env);
    }

    if (adc != NULL && adc->init != NULL) {
        adc->init(0U);
        adc->init(1U);
    }

    return HAL_OK;
}

void env_sensors_poll_100hz(env_sensors_t *env)
{
    if (env == NULL || env->sim_override) {
        return;
    }
    if (env->bmp390_ok) {
        poll_bmp390(env);
    }
    if (env->qmc5883l_ok) {
        poll_qmc5883l(env);
    }
}

void env_sensors_poll_20hz(env_sensors_t *env)
{
    if (env == NULL || env->sim_override) {
        return;
    }
    if (env->vl53l1x_ok) {
        poll_vl53l1x(env);
    }
    poll_ina226_or_adc(env);
}
