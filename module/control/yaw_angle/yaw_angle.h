#ifndef YAW_ANGLE_H
#define YAW_ANGLE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float kp_s;                     /* Angle error rad -> rate rad/s, 1/s. */
    float ki_s2;                    /* Integral gain, 1/s^2; zero disables I. */
    float integral_limit_rad_s;
    float max_rate_rad_s;
    float acceleration_rad_s2;
    uint32_t max_step_ms;           /* 1..100. */
    uint32_t feedback_timeout_ms;   /* max_step_ms..1000. */
    uint32_t command_timeout_ms;    /* max_step_ms..1000. */
} YawAngleConfig_s;

typedef struct
{
    double target_rad;
    double measured_rad;           /* Same continuous/unwrapped coordinate as target. */
    uint32_t feedback_ms;
    uint32_t command_ms;
    uint32_t reference_id;         /* Change when zero/frame/unwrap reference changes. */
    bool feedback_valid;
    bool enable;                   /* Caller supplies external authorization. */
} YawAngleInput_s;

typedef enum
{
    YAW_ANGLE_OFF,
    YAW_ANGLE_TRACKING,
    YAW_ANGLE_FAULT
} YawAngleState_e;

enum
{
    YAW_ANGLE_ERROR_CONFIG = 1U,
    YAW_ANGLE_ERROR_DATA = 2U,
    YAW_ANGLE_ERROR_FEEDBACK = 4U,
    YAW_ANGLE_ERROR_COMMAND = 8U,
    YAW_ANGLE_ERROR_TIMING = 16U,
    YAW_ANGLE_ERROR_REFERENCE = 32U
};

typedef struct
{
    double error_rad;
    float integral_rad_s;
    float rate_rad_s;
    uint32_t fault;
    YawAngleState_e state;
    bool limited;
} YawAngleOutput_s;

typedef struct
{
    YawAngleConfig_s config;
    YawAngleOutput_s output;
    uint32_t last_step_ms;
    uint32_t reference_id;
    bool have_step;
    bool have_reference;
    bool configured;
} YawAngle_s;

/** 可移植角度外环：角度误差 -> PI -> 限速/加速度限制 -> 角速度。
 * Single owner, no IO/heap/global state. Init copies config on a stopped system.
 * Positive output must increase measured_rad after caller's actuator mapping.
 * NO automatic shortest-path wrapping: continuous targets preserve requested
 * multi-turn motion. Caller manages target capture, modes, axes and gear ratio.
 * This core is prepared/tested but not connected to the bench motor path. */
bool YawAngleInit(YawAngle_s *controller, const YawAngleConfig_s *config);
/** All timestamps use one wrapping ms clock. At most 100 ms between active steps.
 * First call has zero elapsed time. Disabling clears I/output/runtime faults;
 * configuration faults require Init. Runtime faults latch until a disabled call.
 * A reference change while active invalidates old targets and latches a fault.
 * Return true only when TRACKING; it does not grant motor enable permission.
 * Disabled/fault output is immediately zero, overriding the normal slew limit. */
bool YawAngleStep(YawAngle_s *controller, const YawAngleInput_s *input, uint32_t now_ms);

#endif
