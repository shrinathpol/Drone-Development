#include "ahrs_fusion.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG_F 57.29577951308232f
#define GRAVITY_MSS 9.80665f

static float inv_sqrt(float x) {
  return (x > 1e-12f) ? (1.0f / sqrtf(x)) : 0.0f;
}

void ahrs_init(ahrs_state_t *ahrs, float kp, float ki) {
  if (ahrs == NULL) {
    return;
  }
  memset(ahrs, 0, sizeof(*ahrs));
  ahrs->q[0] = 1.0f;
  ahrs->kp = (kp > 0.0f) ? kp : 1.5f;
  ahrs->ki = (ki >= 0.0f) ? ki : 0.05f;
  ahrs->initialized = true;
}

void ahrs_update(ahrs_state_t *ahrs, const float gyro_rad_s[3],
                 const float accel_m_s2[3], const float mag_ut[3],
                 bool mag_valid, float baro_alt_m, bool baro_valid,
                 float tof_range_m, bool tof_valid, float dt_s) {
  if (ahrs == NULL || !ahrs->initialized || dt_s <= 0.0f || dt_s > 0.1f) {
    return;
  }

  float gx = gyro_rad_s[0];
  float gy = gyro_rad_s[1];
  float gz = gyro_rad_s[2];
  float ax = accel_m_s2[0];
  float ay = accel_m_s2[1];
  float az = accel_m_s2[2];

  float q0 = ahrs->q[0], q1 = ahrs->q[1], q2 = ahrs->q[2], q3 = ahrs->q[3];
  float ex = 0.0f, ey = 0.0f, ez = 0.0f;

  float a_norm2 = ax * ax + ay * ay + az * az;
  if (a_norm2 > 1.0f) {
    float recip_a = inv_sqrt(a_norm2);
    ax *= recip_a;
    ay *= recip_a;
    az *= recip_a;

    /* Estimated gravity direction from quaternion */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex += (ay * vz - az * vy);
    ey += (az * vx - ax * vz);
    ez += (ax * vy - ay * vx);
  }

  if (mag_valid && mag_ut != NULL) {
    float mx = mag_ut[0], my = mag_ut[1], mz = mag_ut[2];
    float m_norm2 = mx * mx + my * my + mz * mz;
    if (m_norm2 > 1e-6f) {
      float recip_m = inv_sqrt(m_norm2);
      mx *= recip_m;
      my *= recip_m;
      mz *= recip_m;

      float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
      float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
      float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

      float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) +
                         mz * (q1q3 + q0q2));
      float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) +
                         mz * (q2q3 - q0q1));
      float bx = sqrtf(hx * hx + hy * hy);
      float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
                         mz * (0.5f - q1q1 - q2q2));

      float wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
      float wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
      float wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

      ex += (my * wz - mz * wy);
      ey += (mz * wx - mx * wz);
      ez += (mx * wy - my * wx);
    }
  }

  if (ahrs->ki > 0.0f) {
    ahrs->integral_fb[0] += ahrs->ki * ex * dt_s;
    ahrs->integral_fb[1] += ahrs->ki * ey * dt_s;
    ahrs->integral_fb[2] += ahrs->ki * ez * dt_s;
    gx += ahrs->integral_fb[0];
    gy += ahrs->integral_fb[1];
    gz += ahrs->integral_fb[2];
  }
  gx += ahrs->kp * ex;
  gy += ahrs->kp * ey;
  gz += ahrs->kp * ez;

  float half_dt = 0.5f * dt_s;
  float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
  float dq1 = (q0 * gx + q2 * gz - q3 * gy) * half_dt;
  float dq2 = (q0 * gy - q1 * gz + q3 * gx) * half_dt;
  float dq3 = (q0 * gz + q1 * gy - q2 * gx) * half_dt;

  q0 += dq0;
  q1 += dq1;
  q2 += dq2;
  q3 += dq3;

  float recip_q = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (recip_q > 0.0f) {
    q0 *= recip_q;
    q1 *= recip_q;
    q2 *= recip_q;
    q3 *= recip_q;
  } else {
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
  }
  ahrs->q[0] = q0;
  ahrs->q[1] = q1;
  ahrs->q[2] = q2;
  ahrs->q[3] = q3;

  /* Extract Euler angles (ZYX) */
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  ahrs->roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG_F;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp >= 1.0f)
    ahrs->pitch_deg = 90.0f;
  else if (sinp <= -1.0f)
    ahrs->pitch_deg = -90.0f;
  else
    ahrs->pitch_deg = asinf(sinp) * RAD_TO_DEG_F;

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  float yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG_F;
  if (yaw < 0.0f)
    yaw += 360.0f;
  ahrs->yaw_deg = yaw;

  /* Earth-frame vertical acceleration (Up positive) */
  float ez_acc = 2.0f * (q1 * q3 - q0 * q2) * accel_m_s2[0] +
                 2.0f * (q0 * q1 + q2 * q3) * accel_m_s2[1] +
                 (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * accel_m_s2[2];
  ahrs->accel_earth_z_mss = ez_acc - GRAVITY_MSS;

  /* Complementary Altitude + Climb-Rate Estimator (IMU + Baro + ToF) */
  ahrs->climb_rate_ms += ahrs->accel_earth_z_mss * dt_s;
  ahrs->alt_agl_m += ahrs->climb_rate_ms * dt_s;

  if (tof_valid && tof_range_m >= 0.02f && tof_range_m <= 3.8f &&
      fabsf(ahrs->roll_deg) < 25.0f && fabsf(ahrs->pitch_deg) < 25.0f) {
    float cos_tilt = cosf(ahrs->roll_deg / RAD_TO_DEG_F) *
                     cosf(ahrs->pitch_deg / RAD_TO_DEG_F);
    float tof_alt = tof_range_m * cos_tilt;
    float err = tof_alt - ahrs->alt_agl_m;
    ahrs->alt_agl_m += 4.5f * err * dt_s;
    ahrs->climb_rate_ms += 2.0f * err * dt_s;
  } else if (baro_valid) {
    float err = baro_alt_m - ahrs->alt_agl_m;
    ahrs->alt_agl_m += 1.8f * err * dt_s;
    ahrs->climb_rate_ms += 0.8f * err * dt_s;
  }
}
