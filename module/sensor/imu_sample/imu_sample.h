#ifndef IMU_SAMPLE_H
#define IMU_SAMPLE_H

#include <stdint.h>

/** One complete sample in the sensor's right-handed XYZ frame, in SI units.
 * Acceleration is specific force (a stationary upward axis reads +g).
 * timestamp_ms is the acquisition-start time from the caller's monotonic
 * uint32_t millisecond clock. No packed layout or wire-format assumptions. */
typedef struct
{
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float temperature_c;
    uint32_t timestamp_ms;
} ImuSample_s;

#endif
