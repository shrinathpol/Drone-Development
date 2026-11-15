#include "imu_universal.h"

#include <string.h>

#define DEG_TO_RAD_F 0.017453292519943295f
#define GRAVITY_MSS  9.80665f

/* ---------------- ICM-42688-P Backend ---------------- */
#define ICM42688_REG_WHO_AM_I    0x75U
#define ICM42688_WHO_AM_I_VAL    0x47U
#define ICM42688_REG_PWR_MGMT0   0x4EU
#define ICM42688_REG_GYRO_CFG0   0x4FU
#define ICM42688_REG_ACCEL_CFG0  0x50U
#define ICM42688_REG_ACCEL_DATA  0x1FU
#define ICM42688_REG_BANK_SEL    0x76U
#define ICM42688_REG_INT_SRC1    0x66U
#define ICM42688_REG_SMD_CONFIG  0x57U
#define ICM42688_REG_WOM_X_TH    0x4AU /* Bank 4 */

static int icm_write(imu_driver_t *dev, uint8_t reg, uint8_t val)
{
    if (dev->use_spi && dev->spi != NULL) {
        return dev->spi->transfer_reg(dev->spi->ctx, (uint8_t)(reg & 0x7FU), &val, NULL, 1U);
    }
    return dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, reg, &val, 1U);
}

static int icm_read(imu_driver_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    if (dev->use_spi && dev->spi != NULL) {
        return dev->spi->transfer_reg(dev->spi->ctx, (uint8_t)(reg | 0x80U), NULL, buf, len);
    }
    return dev->i2c->read_reg(dev->i2c->ctx, dev->i2c_addr, reg, buf, len);
}

static int icm42688_init(imu_driver_t *dev)
{
    uint8_t who = 0U;
    if (icm_read(dev, ICM42688_REG_WHO_AM_I, &who, 1U) != HAL_OK || who != ICM42688_WHO_AM_I_VAL) {
        return HAL_ERR_NOT_READY;
    }
    /* Enable Accel & Gyro in Low-Noise mode (0x0F), 1kHz ODR, +-2000dps, +-16g */
    if (icm_write(dev, ICM42688_REG_PWR_MGMT0, 0x0FU) != HAL_OK) {
        return HAL_ERR_BUS;
    }
    icm_write(dev, ICM42688_REG_GYRO_CFG0, 0x06U);
    icm_write(dev, ICM42688_REG_ACCEL_CFG0, 0x06U);
    dev->chip_type = IMU_CHIP_ICM42688P;
    return HAL_OK;
}

static int icm42688_read(imu_driver_t *dev, imu_sample_t *out)
{
    uint8_t raw[12];
    if (icm_read(dev, ICM42688_REG_ACCEL_DATA, raw, sizeof(raw)) != HAL_OK) {
        out->valid = false;
        return HAL_ERR_BUS;
    }

    int16_t ax = (int16_t)((raw[0] << 8U) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8U) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8U) | raw[5]);
    int16_t gx = (int16_t)((raw[6] << 8U) | raw[7]);
    int16_t gy = (int16_t)((raw[8] << 8U) | raw[9]);
    int16_t gz = (int16_t)((raw[10] << 8U) | raw[11]);

    /* +-16g -> 2048 LSB/g ; +-2000 dps -> 16.4 LSB/(deg/s) */
    const float acc_scale = GRAVITY_MSS / 2048.0f;
    const float gyr_scale = DEG_TO_RAD_F / 16.4f;

    out->accel_m_s2[0] = (float)ax * acc_scale - dev->accel_bias[0];
    out->accel_m_s2[1] = (float)ay * acc_scale - dev->accel_bias[1];
    out->accel_m_s2[2] = (float)az * acc_scale - dev->accel_bias[2];
    out->gyro_rad_s[0] = (float)gx * gyr_scale - dev->gyro_bias[0];
    out->gyro_rad_s[1] = (float)gy * gyr_scale - dev->gyro_bias[1];
    out->gyro_rad_s[2] = (float)gz * gyr_scale - dev->gyro_bias[2];
    out->temp_c = 25.0f;
    out->timestamp_us = (uint32_t)dev->sys->micros();
    out->valid = true;
    return HAL_OK;
}

static int icm42688_enable_wom(imu_driver_t *dev, uint16_t threshold_mg)
{
    uint8_t th = (uint8_t)((threshold_mg * 256U) / 1000U);
    if (th == 0U) {
        th = 10U;
    }
    /* Accel Low-Power Wake-on-Motion configuration */
    icm_write(dev, ICM42688_REG_PWR_MGMT0, 0x02U);
    icm_write(dev, ICM42688_REG_BANK_SEL, 0x04U);
    icm_write(dev, ICM42688_REG_WOM_X_TH, th);
    icm_write(dev, ICM42688_REG_WOM_X_TH + 1U, th);
    icm_write(dev, ICM42688_REG_WOM_X_TH + 2U, th);
    icm_write(dev, ICM42688_REG_BANK_SEL, 0x00U);
    icm_write(dev, ICM42688_REG_INT_SRC1, 0x0EU);
    icm_write(dev, ICM42688_REG_SMD_CONFIG, 0x05U);
    return HAL_OK;
}

static const imu_vtable_t s_icm42688_vtable = {
    .name = "ICM-42688-P",
    .probe_and_init = icm42688_init,
    .read_sample = icm42688_read,
    .enable_wom = icm42688_enable_wom
};

/* ---------------- MPU6050 I2C Backend ---------------- */
#define MPU6050_REG_WHO_AM_I    0x75U
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_PWR_MGMT_2  0x6CU
#define MPU6050_REG_GYRO_CFG    0x1BU
#define MPU6050_REG_ACCEL_CFG   0x1CU
#define MPU6050_REG_ACCEL_XOUT  0x3BU
#define MPU6050_REG_MOT_THR     0x1FU
#define MPU6050_REG_MOT_DUR     0x20U
#define MPU6050_REG_INT_ENABLE  0x38U

static int mpu6050_init(imu_driver_t *dev)
{
    if (dev->use_spi || dev->i2c == NULL) {
        return HAL_ERR_NOT_READY;
    }
    uint8_t who = 0U;
    if (dev->i2c->read_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_WHO_AM_I, &who, 1U) != HAL_OK) {
        return HAL_ERR_NOT_READY;
    }
    if (who != 0x68U && who != 0x69U && who != 0x70U) {
        return HAL_ERR_NOT_READY;
    }
    uint8_t wake = 0x01U;
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_PWR_MGMT_1, &wake, 1U);
    uint8_t gcfg = 0x18U; /* +-2000 dps */
    uint8_t acfg = 0x18U; /* +-16 g */
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_GYRO_CFG, &gcfg, 1U);
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_ACCEL_CFG, &acfg, 1U);
    dev->chip_type = IMU_CHIP_MPU6050;
    return HAL_OK;
}

static int mpu6050_read(imu_driver_t *dev, imu_sample_t *out)
{
    uint8_t raw[14];
    if (dev->i2c->read_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_ACCEL_XOUT, raw, sizeof(raw)) != HAL_OK) {
        out->valid = false;
        return HAL_ERR_BUS;
    }

    int16_t ax = (int16_t)((raw[0] << 8U) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8U) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8U) | raw[5]);
    int16_t tp = (int16_t)((raw[6] << 8U) | raw[7]);
    int16_t gx = (int16_t)((raw[8] << 8U) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8U) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8U) | raw[13]);

    const float acc_scale = GRAVITY_MSS / 2048.0f;
    const float gyr_scale = DEG_TO_RAD_F / 16.4f;

    out->accel_m_s2[0] = (float)ax * acc_scale - dev->accel_bias[0];
    out->accel_m_s2[1] = (float)ay * acc_scale - dev->accel_bias[1];
    out->accel_m_s2[2] = (float)az * acc_scale - dev->accel_bias[2];
    out->gyro_rad_s[0] = (float)gx * gyr_scale - dev->gyro_bias[0];
    out->gyro_rad_s[1] = (float)gy * gyr_scale - dev->gyro_bias[1];
    out->gyro_rad_s[2] = (float)gz * gyr_scale - dev->gyro_bias[2];
    out->temp_c = (float)tp / 340.0f + 36.53f;
    out->timestamp_us = (uint32_t)dev->sys->micros();
    out->valid = true;
    return HAL_OK;
}

static int mpu6050_enable_wom(imu_driver_t *dev, uint16_t threshold_mg)
{
    uint8_t thr = (uint8_t)(threshold_mg / 2U);
    if (thr == 0U) {
        thr = 20U;
    }
    uint8_t dur = 1U;
    uint8_t ie = 0x40U;
    uint8_t p1 = 0x20U;
    uint8_t p2 = 0x47U; /* LP accel wake 1.25Hz, gyros standby */
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_MOT_THR, &thr, 1U);
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_MOT_DUR, &dur, 1U);
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_INT_ENABLE, &ie, 1U);
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_PWR_MGMT_1, &p1, 1U);
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, MPU6050_REG_PWR_MGMT_2, &p2, 1U);
    return HAL_OK;
}

static const imu_vtable_t s_mpu6050_vtable = {
    .name = "MPU6050",
    .probe_and_init = mpu6050_init,
    .read_sample = mpu6050_read,
    .enable_wom = mpu6050_enable_wom
};

/* ---------------- BMI270 I2C Backend ---------------- */
#define BMI270_REG_CHIP_ID  0x00U
#define BMI270_CHIP_ID_VAL  0x24U
#define BMI270_REG_DATA_8   0x0CU
#define BMI270_REG_PWR_CTRL 0x7DU

static int bmi270_init(imu_driver_t *dev)
{
    if (dev->use_spi || dev->i2c == NULL) {
        return HAL_ERR_NOT_READY;
    }
    uint8_t id = 0U;
    if (dev->i2c->read_reg(dev->i2c->ctx, dev->i2c_addr, BMI270_REG_CHIP_ID, &id, 1U) != HAL_OK ||
        id != BMI270_CHIP_ID_VAL) {
        return HAL_ERR_NOT_READY;
    }
    uint8_t pwr = 0x0EU;
    dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, BMI270_REG_PWR_CTRL, &pwr, 1U);
    dev->chip_type = IMU_CHIP_BMI270;
    return HAL_OK;
}

static int bmi270_read(imu_driver_t *dev, imu_sample_t *out)
{
    uint8_t raw[12];
    if (dev->i2c->read_reg(dev->i2c->ctx, dev->i2c_addr, BMI270_REG_DATA_8, raw, sizeof(raw)) != HAL_OK) {
        out->valid = false;
        return HAL_ERR_BUS;
    }

    int16_t ax = (int16_t)((raw[1] << 8U) | raw[0]);
    int16_t ay = (int16_t)((raw[3] << 8U) | raw[2]);
    int16_t az = (int16_t)((raw[5] << 8U) | raw[4]);
    int16_t gx = (int16_t)((raw[7] << 8U) | raw[6]);
    int16_t gy = (int16_t)((raw[9] << 8U) | raw[8]);
    int16_t gz = (int16_t)((raw[11] << 8U) | raw[10]);

    const float acc_scale = GRAVITY_MSS / 4096.0f;
    const float gyr_scale = DEG_TO_RAD_F / 16.4f;

    out->accel_m_s2[0] = (float)ax * acc_scale - dev->accel_bias[0];
    out->accel_m_s2[1] = (float)ay * acc_scale - dev->accel_bias[1];
    out->accel_m_s2[2] = (float)az * acc_scale - dev->accel_bias[2];
    out->gyro_rad_s[0] = (float)gx * gyr_scale - dev->gyro_bias[0];
    out->gyro_rad_s[1] = (float)gy * gyr_scale - dev->gyro_bias[1];
    out->gyro_rad_s[2] = (float)gz * gyr_scale - dev->gyro_bias[2];
    out->temp_c = 25.0f;
    out->timestamp_us = (uint32_t)dev->sys->micros();
    out->valid = true;
    return HAL_OK;
}

static int bmi270_enable_wom(imu_driver_t *dev, uint16_t threshold_mg)
{
    (void)threshold_mg;
    uint8_t pwr = 0x04U; /* Accel only low-power */
    return dev->i2c->write_reg(dev->i2c->ctx, dev->i2c_addr, BMI270_REG_PWR_CTRL, &pwr, 1U);
}

static const imu_vtable_t s_bmi270_vtable = {
    .name = "BMI270",
    .probe_and_init = bmi270_init,
    .read_sample = bmi270_read,
    .enable_wom = bmi270_enable_wom
};

int imu_universal_init(imu_driver_t *dev,
                       hal_i2c_t *i2c,
                       hal_spi_t *spi,
                       hal_gpio_t *gpio,
                       hal_sys_t *sys,
                       uint8_t int_pin)
{
    if (dev == NULL || sys == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    /* Preserve custom SITL virtual backend if registered prior to init */
    const imu_vtable_t *saved_vt = dev->vtable;
    void *saved_ctx = dev->backend_ctx;

    memset(dev, 0, sizeof(*dev));
    dev->i2c = i2c;
    dev->spi = spi;
    dev->gpio = gpio;
    dev->sys = sys;
    dev->int_pin = int_pin;

    if (gpio != NULL && int_pin < 40U) {
        gpio->mode_input(int_pin, false);
    }

    if (saved_vt != NULL) {
        dev->vtable = saved_vt;
        dev->backend_ctx = saved_ctx;
        if (dev->vtable->probe_and_init(dev) == HAL_OK) {
            dev->healthy = true;
            return HAL_OK;
        }
    }

    /* Try SPI ICM-42688-P first if SPI bus is wired */
    if (spi != NULL && spi->init != NULL && spi->init(spi->ctx, 8000000U) == HAL_OK) {
        dev->use_spi = true;
        if (icm42688_init(dev) == HAL_OK) {
            dev->vtable = &s_icm42688_vtable;
            dev->healthy = true;
            return HAL_OK;
        }
    }

    /* Probe I2C addresses 0x68 and 0x69 across supported IMUs */
    if (i2c != NULL && i2c->init != NULL) {
        i2c->init(i2c->ctx, 400000U);
        dev->use_spi = false;
        const uint8_t addrs[2] = {0x68U, 0x69U};
        const imu_vtable_t *cands[3] = {&s_icm42688_vtable, &s_mpu6050_vtable, &s_bmi270_vtable};

        for (size_t a = 0U; a < 2U; a++) {
            dev->i2c_addr = addrs[a];
            for (size_t c = 0U; c < 3U; c++) {
                if (cands[c]->probe_and_init(dev) == HAL_OK) {
                    dev->vtable = cands[c];
                    dev->healthy = true;
                    return HAL_OK;
                }
            }
        }
    }

    dev->healthy = false;
    return HAL_ERR_NOT_READY;
}

void imu_universal_register_virtual(imu_driver_t *dev, const imu_vtable_t *vtable, void *ctx)
{
    if (dev != NULL) {
        dev->vtable = vtable;
        dev->backend_ctx = ctx;
        dev->chip_type = IMU_CHIP_SITL_VIRTUAL;
    }
}

int imu_universal_read(imu_driver_t *dev, imu_sample_t *out)
{
    if (dev == NULL || out == NULL || dev->vtable == NULL || dev->vtable->read_sample == NULL) {
        return HAL_ERR_NOT_READY;
    }
    int rc = dev->vtable->read_sample(dev, out);
    if (rc == HAL_OK) {
        dev->last_sample = *out;
        dev->healthy = true;
    }
    return rc;
}

int imu_universal_enable_wom(imu_driver_t *dev, uint16_t threshold_mg)
{
    if (dev == NULL || dev->vtable == NULL || dev->vtable->enable_wom == NULL) {
        return HAL_ERR_NOT_READY;
    }
    return dev->vtable->enable_wom(dev, threshold_mg);
}

void imu_universal_set_bias(imu_driver_t *dev, const float gyro_bias[3], const float accel_bias[3])
{
    if (dev == NULL) {
        return;
    }
    if (gyro_bias != NULL) {
        memcpy(dev->gyro_bias, gyro_bias, sizeof(dev->gyro_bias));
    }
    if (accel_bias != NULL) {
        memcpy(dev->accel_bias, accel_bias, sizeof(dev->accel_bias));
    }
}
