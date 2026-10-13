#ifndef YAW_RATE_H
#define YAW_RATE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float max_rate_rad_s;
    float acceleration_rad_s2;
    uint32_t max_step_ms;
} YawRateConfig_s;

/** Diagnostic reasons for the most recent control decision; bits may combine.
 * Zero means the rate controller authorized output, NOT a motor enable ACK.
 * These are observations, not new stop conditions or commands. After the
 * originating problem clears, a held ON request reports REARM_REQUIRED. */
enum
{
    YAW_INHIBIT_CONFIG = 1U,
    YAW_INHIBIT_COMMAND = 2U,
    YAW_INHIBIT_ACTUATOR = 4U,
    YAW_INHIBIT_TIMING = 8U,
    YAW_INHIBIT_RATE = 16U,
    YAW_INHIBIT_SWITCH_OFF = 32U,
    YAW_INHIBIT_REARM_REQUIRED = 64U,
    YAW_INHIBIT_START_NOT_ZERO = 128U
};

typedef struct
{
    YawRateConfig_s config;
    float target_rad_s;
    float reference_rad_s;
    uint32_t last_step_ms;
    uint32_t inhibit;
    bool initialized;
    bool valid_config;
    bool off_seen;
    bool previous_arm;
    bool enabled;
} YawRate_s;

/** HAL/RTOS independent. One control task owns the instance. No angle wrapping
 * or position hold: the output is a motor angular rate in rad/s.
 * Invalid configuration fails closed and returns false.
 */
bool YawRateInit(YawRate_s *yaw, const YawRateConfig_s *config);
/** valid means a fresh, healthy input; ready means a healthy motor/transport.
 * Arming requires observing OFF while ready, then an ON edge at zero rate.
 * rate_rad_s is a finite MOTOR-side angular rate within +/- max_rate_rad_s;
 * input mapping/deadband belongs to the command producer. Out-of-range input,
 * unavailable motor or timing gap requires rearming (no silent clamping).
 * Normal rate changes ramp; disable clears the reference immediately.
 */
void YawRateStep(YawRate_s *yaw, uint32_t now_ms, bool valid, bool ready,
                 bool arm, float rate_rad_s);
/** Force disarm and forget OFF authorization after any asynchronous fault. */
void YawRateDisarm(YawRate_s *yaw);

#endif
