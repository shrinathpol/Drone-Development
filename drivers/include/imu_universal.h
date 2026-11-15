#ifndef IMU_UNIVERSAL_H
#define IMU_UNIVERSAL_H

#include "hal_interfaces.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_CHIP_NONE = 0,
    IMU_CHIP_ICM42688P,
    IMU_CHIP_MPU6050,
    IMU_CHIP_BMI270,
    IMU_CHIP_SITL_VIRTUAL
} imu_chip_type_t;

typedef struct {
    float gyro_rad_s[3];   /* X (roll), Y (pitch), Z (yaw) rad/s */
    float accel_m_s2[3];   /* X (fwd), Y (right), Z (down/up) m/s^2 */
    float temp_c;
    uint32_t timestamp_us;
    bool valid;
} imu_sample_t;

typedef struct imu_driver imu_driver_t;

typedef struct {
    const char *name;
    int (*probe_and_init)(imu_driver_t *dev);
    int (*read_sample)(imu_driver_t *dev, imu_sample_t *out);
    int (*enable_wom)(imu_driver_t *dev, uint16_t threshold_mg);
} imu_vtable_t;

struct imu_driver {
    hal_i2c_t *i2c;
    hal_spi_t *spi;
    hal_gpio_t *gpio;
    hal_sys_t *sys;
    uint8_t i2c_addr;
    uint8_t int_pin;
    bool use_spi;

    imu_chip_type_t chip_type;
    const imu_vtable_t *vtable;
    void *backend_ctx;

    float gyro_bias[3];
    float accel_bias[3];
    imu_sample_t last_sample;
    bool healthy;
};

int imu_universal_init(imu_driver_t *dev,
                       hal_i2c_t *i2c,
                       hal_spi_t *spi,
                       hal_gpio_t *gpio,
                       hal_sys_t *sys,
                       uint8_t int_pin);

void imu_universal_register_virtual(imu_driver_t *dev, const imu_vtable_t *vtable, void *ctx);

int imu_universal_read(imu_driver_t *dev, imu_sample_t *out);

int imu_universal_enable_wom(imu_driver_t *dev, uint16_t threshold_mg);

void imu_universal_set_bias(imu_driver_t *dev, const float gyro_bias[3], const float accel_bias[3]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_UNIVERSAL_H */
