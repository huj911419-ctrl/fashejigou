#include "gimbal_arbiter.h"
#include <math.h>
#include <string.h>

bool GimbalArbiterInit(GimbalArbiter_s *arbiter, const ControlAuthorityConfig_s *config,
                       float max_rate_rad_s)
{
    if (arbiter == NULL)
    {
        return false;
    }
    memset(arbiter, 0, sizeof(*arbiter));
    bool authority_valid = ControlAuthorityInit(&arbiter->authority, config);
    arbiter->max_rate_rad_s = max_rate_rad_s;
    arbiter->configured = authority_valid && isfinite(max_rate_rad_s) && max_rate_rad_s > 0.0f;
    return arbiter->configured;
}

bool GimbalArbiterSelect(GimbalArbiter_s *arbiter, ControlSource_t source)
{
    if (arbiter == NULL || !arbiter->configured)
    {
        return false;
    }
    uint32_t previous_epoch = arbiter->authority.status.epoch;
    if (!ControlAuthoritySelect(&arbiter->authority, source))
    {
        return false;
    }
    if (previous_epoch != arbiter->authority.status.epoch)
    {
        memset(&arbiter->candidate, 0, sizeof(arbiter->candidate));
    }
    return true;
}

bool GimbalArbiterSubmit(GimbalArbiter_s *arbiter, ControlSource_t source, uint32_t epoch,
                         const GimbalCommand_s *command)
{
    if (arbiter == NULL || !arbiter->configured || command == NULL)
    {
        return false;
    }
    ControlAuthorityInput_s input =
    {
        .source = source,
        .epoch = epoch,
        .received_ms = command->received_ms,
        .sequence = command->sequence,
        .valid = command->valid && isfinite(command->rate_rad_s) &&
                 fabsf(command->rate_rad_s) <= arbiter->max_rate_rad_s,
        .arm = command->arm,
        .neutral = command->rate_rad_s == 0.0f,
        .stop_requested = command->stop_requested
    };
    if (!ControlAuthoritySubmit(&arbiter->authority, &input))
    {
        return false;
    }
    arbiter->candidate = *command;
    arbiter->candidate.valid = input.valid;
    return true;
}

void GimbalArbiterStop(GimbalArbiter_s *arbiter)
{
    if (arbiter != NULL)
    {
        ControlAuthorityStop(&arbiter->authority);
    }
}

GimbalCommand_s GimbalArbiterStep(GimbalArbiter_s *arbiter, uint32_t now_ms)
{
    GimbalCommand_s output = {.stop_requested = true};
    if (arbiter == NULL || !arbiter->configured)
    {
        return output;
    }
    ControlAuthorityStep(&arbiter->authority, now_ms);
    output = arbiter->candidate;
    output.valid = output.valid && arbiter->authority.status.online;
    if (!arbiter->authority.status.authorized && output.arm)
    {
        output.stop_requested = true;
    }
    return output;
}
