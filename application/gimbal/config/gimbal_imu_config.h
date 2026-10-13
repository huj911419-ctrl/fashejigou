#ifndef GIMBAL_IMU_CONFIG_H
#define GIMBAL_IMU_CONFIG_H

#include "../../bench_features.h"

/* Compatibility name; edit the central bench_features.h switches. */
#define GIMBAL_IMU_ENABLED BENCH_ENABLE_IMU
#define GIMBAL_IMU_PERIOD_MS 2U

/* User confirmed that this control board rotates with the gimbal.
 * Confirm chip axes against BODY X forward, Y left, Z up on the real mount
 * before changing MOUNTING_CONFIRMED to 1. Identity currently reports sensor
 * axes; valid_for_control stays false. No motor feedback is switched here. */
#define GIMBAL_IMU_ON_GIMBAL 1
#define GIMBAL_IMU_MOUNTING_CONFIRMED 0

#define GIMBAL_ATTITUDE_CONFIG                  \
{                                              \
    .axis_map = {1, 2, 3},                      \
    .gravity_m_s2 = 9.80665f,                   \
    .accel_gate_m_s2 = 1.0f,                    \
    .correction_gain_s = 2.0f,                  \
    .stationary_gyro_rad_s = 0.05f,             \
    .stationary_gyro_span = 0.01f,              \
    .stationary_accel_span = 0.2f,              \
    .max_gyro_rad_s = 35.0f,                    \
    .calibration_samples = 1000U,               \
    .max_step_ms = 10U,                         \
    .stale_ms = 20U                            \
}

#endif
