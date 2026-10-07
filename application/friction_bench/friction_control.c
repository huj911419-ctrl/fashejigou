#include "friction_control.h"
#include <math.h>
#include <string.h>

void FrictionControlInit(Friction_Control_s *control, uint32_t now_ms)
{
    memset(control, 0, sizeof(*control));
    control->last_step_ms = now_ms;
    /* An explicit disabled command must be observed before first enable. */
    control->require_disable = 1;
}

static uint8_t ConfigValid(const Friction_Control_Config_s *config)
{
    return isfinite(config->max_rpm) && config->max_rpm > 0.0f &&
           isfinite(config->ramp_rpm_s) && config->ramp_rpm_s > 0.0f &&
           config->command_timeout_ms > 0 && config->command_timeout_ms < 0x80000000U &&
           config->feedback_timeout_ms > 0 && config->feedback_timeout_ms < 0x80000000U &&
           config->max_step_ms > 0 && config->max_step_ms < 1000U &&
           config->max_temperature_c > 0;
}

void FrictionControlStep(Friction_Control_s *control,
                         const Friction_Control_Config_s *config,
                         const Friction_Command_s *command, uint32_t command_ms,
                         const Friction_Feedback_s feedback[FRICTION_MOTOR_COUNT],
                         uint32_t now_ms, uint32_t external_fault)
{
    Friction_Status_s *status = &control->status;
    LoopStatsObserve(&status->timing, now_ms, config->max_step_ms);
    uint32_t fault = external_fault;
    uint8_t mask = command->enable_mask;
    status->step_ms = now_ms - control->last_step_ms;
    control->last_step_ms = now_ms;
    status->command_age_ms = now_ms - command_ms;
    status->online_mask = 0;
    memcpy(status->feedback, feedback, sizeof(status->feedback));
    memcpy(status->requested_rpm, command->target_rpm, sizeof(status->requested_rpm));

    if (!ConfigValid(config))
        fault |= FRICTION_FAULT_CONFIG;
    if (mask & ~FRICTION_ALL_MOTORS)
        fault |= FRICTION_FAULT_COMMAND;

    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (feedback[i].valid && now_ms - feedback[i].last_rx_ms <= config->feedback_timeout_ms &&
            isfinite(feedback[i].rpm))
            status->online_mask |= 1U << i;
        if (mask & (1U << i))
        {
            if (!isfinite(command->target_rpm[i]) || fabsf(command->target_rpm[i]) > config->max_rpm)
                fault |= FRICTION_FAULT_COMMAND;
            if (!(status->online_mask & (1U << i)))
                fault |= FRICTION_FAULT_OFFLINE;
            if (feedback[i].temperature_c >= config->max_temperature_c)
                fault |= FRICTION_FAULT_TEMPERATURE;
        }
    }
    if (mask)
    {
        if (status->command_age_ms > config->command_timeout_ms)
            fault |= FRICTION_FAULT_COMMAND;
        if (status->step_ms == 0 || status->step_ms > config->max_step_ms)
            fault |= FRICTION_FAULT_TIMING;
    }

    if (fault)
    {
        status->fault |= fault;
        control->require_disable = 1;
    }
    else if (!mask && status->command_age_ms <= config->command_timeout_ms)
    {
        /* Recovery never resumes an old enable request. */
        status->fault = 0;
        control->require_disable = 0;
    }

    status->enabled_mask = control->require_disable ? 0 : mask;
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (!(status->enabled_mask & (1U << i)))
        {
            status->reference_rpm[i] = 0.0f;
            continue;
        }
        float step = config->ramp_rpm_s * ((float)status->step_ms * 0.001f);
        float delta = command->target_rpm[i] - status->reference_rpm[i];
        status->reference_rpm[i] += fmaxf(-step, fminf(step, delta));
    }
}
