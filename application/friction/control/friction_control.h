#ifndef FRICTION_CONTROL_H
#define FRICTION_CONTROL_H

#include <stdint.h>
#include "loop_stats.h"

#define FRICTION_MOTOR_COUNT 3U
#define FRICTION_ALL_MOTORS 0x07U

typedef enum
{
    FRICTION_FAULT_NONE = 0,
    FRICTION_FAULT_CONFIG = 1U << 0,
    FRICTION_FAULT_COMMAND = 1U << 1,
    FRICTION_FAULT_OFFLINE = 1U << 2,
    FRICTION_FAULT_TEMPERATURE = 1U << 3,
    FRICTION_FAULT_TIMING = 1U << 4,
    FRICTION_FAULT_DRIVER = 1U << 5,
} Friction_Fault_e;

/* One atomic command. Submit periodically; mask=0 means zero current/coast. */
typedef struct
{
    float target_rpm[FRICTION_MOTOR_COUNT];
    uint8_t enable_mask;
} Friction_Command_s;

typedef struct
{
    float max_rpm;
    float ramp_rpm_s;
    uint32_t command_timeout_ms;
    uint32_t feedback_timeout_ms;
    uint32_t max_step_ms;
    uint8_t max_temperature_c;
} Friction_Control_Config_s;

typedef struct
{
    float rpm;
    int16_t current_raw;
    uint8_t temperature_c;
    uint8_t valid;
    uint32_t last_rx_ms;
} Friction_Feedback_s;

typedef struct
{
    float requested_rpm[FRICTION_MOTOR_COUNT];
    float reference_rpm[FRICTION_MOTOR_COUNT];
    Friction_Feedback_s feedback[FRICTION_MOTOR_COUNT];
    uint32_t fault;
    uint32_t active_fault; /* This step's causes; fault also retains old causes. */
    uint32_t command_age_ms;
    uint32_t step_ms;
    LoopStats_s timing;
    uint8_t online_mask;
    uint8_t enabled_mask;
    uint8_t requested_mask;
    uint8_t rearm_required;
} Friction_Status_s;

typedef struct
{
    Friction_Status_s status;
    uint32_t last_step_ms;
    uint8_t require_disable;
} Friction_Control_s;

/** HAL-free state and ramp logic, owned by the motor task. */
void FrictionControlInit(Friction_Control_s *control, uint32_t now_ms);
void FrictionControlStep(Friction_Control_s *control,
                         const Friction_Control_Config_s *config,
                         const Friction_Command_s *command, uint32_t command_ms,
                         const Friction_Feedback_s feedback[FRICTION_MOTOR_COUNT],
                         uint32_t now_ms, uint32_t external_fault);

#endif
