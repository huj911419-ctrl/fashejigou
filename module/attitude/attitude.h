#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "imu_sample.h"
#include <stdbool.h>

typedef struct
{
    int8_t axis_map[3]; /* Body XYZ = signed sensor axes +/-1, +/-2, +/-3.
                        * Must be a right-handed permutation (determinant +1). */
    float gravity_m_s2;
    float accel_gate_m_s2;          /* Allowed |norm(accel)-g| for gravity correction. */
    float correction_gain_s;        /* Proportional complementary correction, 1/s. */
    float stationary_gyro_rad_s;    /* Maximum axis magnitude during calibration. */
    float stationary_gyro_span;     /* Maximum axis range across a calibration window, rad/s. */
    float stationary_accel_span;    /* Maximum axis range across a calibration window, m/s^2. */
    float max_gyro_rad_s;
    uint32_t calibration_samples;
    uint32_t max_step_ms;
    uint32_t stale_ms;
} AttitudeConfig_s;

typedef enum
{
    ATTITUDE_CALIBRATING,
    ATTITUDE_TRACKING,
    ATTITUDE_LOST,
    ATTITUDE_BAD_CONFIG
} AttitudeState_e;

enum
{
    ATTITUDE_FAULT_DATA = 1U,
    ATTITUDE_FAULT_TIMING = 2U,
    ATTITUDE_FAULT_SOURCE = 4U,
    ATTITUDE_FAULT_CONFIG = 8U
};

typedef struct
{
    float quaternion[4]; /* Hamilton w,x,y,z: body -> world; world Z points up. */
    float gyro_rad_s[3]; /* Body rates after stationary bias subtraction. */
    float accel_m_s2[3]; /* Body-frame specific force, NOT linear acceleration. */
    float gyro_bias_rad_s[3];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;       /* ZYX Euler heading in [-pi,pi]. Initial heading is zero. */
    double yaw_unwrapped_rad;
    float temperature_c;
    uint32_t sample_ms;
    uint32_t calibration_count;
    uint32_t fault;
    AttitudeState_e state;
    bool valid;
    bool accel_correction_used;
    bool heading_valid;  /* False near Euler pitch singularity. */
    bool yaw_continuous; /* False after crossing singularity; reset to re-reference. */
} AttitudeOutput_s;

typedef struct
{
    AttitudeConfig_s config;
    AttitudeOutput_s output;
    float mean_gyro[3];
    float mean_accel[3];
    float min_gyro[3];
    float max_gyro[3];
    float min_accel[3];
    float max_accel[3];
    uint32_t last_sample_ms;
    bool have_sample;
} Attitude_s;

/** Pure C, one task owns each instance; no allocation/global state/IO.
 * Six-axis proportional complementary quaternion filter with a separate
 * startup stationary gyro calibration. It has NO absolute heading reference.
 * Init also explicitly resets a latched loss and all angle references.
 * Identity axis_map means SENSOR coordinates until mounting is verified. */
bool AttitudeInit(Attitude_s *attitude, const AttitudeConfig_s *config);
/** sample and now_ms share one uint32_t monotonic millisecond clock.
 * calibration_allowed must be false while actuators are enabled. The caller
 * must also physically keep the unit still: IMU-only stillness tests cannot
 * distinguish slow constant rotation from bias. Moving windows reset progress.
 * Invalid data/timing during tracking latches LOST until explicit Init.
 * Returns true only for a newly accepted, valid attitude output. */
bool AttitudeUpdate(Attitude_s *attitude, const ImuSample_s *sample,
                    uint32_t now_ms, bool calibration_allowed);
/** Call even when no sample arrives. Does not extrapolate an old sample. */
void AttitudeCheckFresh(Attitude_s *attitude, uint32_t now_ms);
/** Source failure immediately invalidates the output. History is diagnostic. */
void AttitudeInvalidate(Attitude_s *attitude, uint32_t fault);

#endif
