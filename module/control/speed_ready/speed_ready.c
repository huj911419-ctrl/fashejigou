#include "speed_ready.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint8_t ConfigValid(const SpeedReady_Config_s *config)
{
    if (config == NULL)
    {
        return 0;
    }

    return isfinite(config->minimum_target) && config->minimum_target > 0.0f &&
           isfinite(config->reference_tolerance) && config->reference_tolerance >= 0.0f &&
           isfinite(config->enter_absolute_error) && config->enter_absolute_error >= 0.0f &&
           isfinite(config->exit_absolute_error) &&
           config->exit_absolute_error >= config->enter_absolute_error &&
           isfinite(config->enter_relative_error) && config->enter_relative_error >= 0.0f &&
           isfinite(config->exit_relative_error) && config->exit_relative_error < 1.0f &&
           config->exit_relative_error >= config->enter_relative_error &&
           config->dwell_ms > 0U && config->dwell_ms < 0x80000000U &&
           config->maximum_sample_gap_ms > 0U &&
           config->maximum_sample_gap_ms < 0x80000000U;
}

static void ClearStability(SpeedReady_s *detector)
{
    detector->status.ready = 0;
    detector->status.stable_ms = 0;
    detector->tracking_stability = 0;
}

uint8_t SpeedReadyInit(SpeedReady_s *detector, const SpeedReady_Config_s *config)
{
    if (detector == NULL)
    {
        return 0;
    }

    memset(detector, 0, sizeof(*detector));
    if (!ConfigValid(config))
    {
        return 0;
    }

    detector->config = *config;
    detector->configured = 1;
    return 1;
}

void SpeedReadyReset(SpeedReady_s *detector)
{
    if (detector == NULL)
    {
        return;
    }

    ClearStability(detector);
    detector->have_sample = 0;
}

SpeedReady_Status_s SpeedReadyUpdate(SpeedReady_s *detector,
                                    const SpeedReady_Input_s *input,
                                    uint32_t now_ms)
{
    const SpeedReady_Status_s not_ready = {0};
    if (detector == NULL)
    {
        return not_ready;
    }

    if (!detector->configured || input == NULL || !input->enabled ||
        !input->feedback_valid || !isfinite(input->target) ||
        !isfinite(input->reference) || !isfinite(input->measured))
    {
        SpeedReadyReset(detector);
        return detector->status;
    }

    const SpeedReady_Config_s *config = &detector->config;
    uint32_t elapsed_ms = now_ms - detector->previous_ms;
    if (!detector->have_sample || input->target != detector->previous_target ||
        elapsed_ms == 0U || elapsed_ms > config->maximum_sample_gap_ms)
    {
        ClearStability(detector);
    }
    detector->previous_ms = now_ms;
    detector->previous_target = input->target;
    detector->have_sample = 1;

    /* A well-tracked ramp is not equivalent to reaching the final target.
     * Also reject zero/reversed feedback even if an absolute tolerance is wide. */
    if (fabsf(input->target) < config->minimum_target ||
        fabsf(input->reference - input->target) > config->reference_tolerance ||
        (input->target > 0.0f && input->measured <= 0.0f) ||
        (input->target < 0.0f && input->measured >= 0.0f))
    {
        ClearStability(detector);
        return detector->status;
    }

    float absolute_tolerance = detector->status.ready ? config->exit_absolute_error :
                                                        config->enter_absolute_error;
    float relative_tolerance = detector->status.ready ? config->exit_relative_error :
                                                        config->enter_relative_error;
    float tolerance = fmaxf(absolute_tolerance, relative_tolerance * fabsf(input->target));
    if (fabsf(input->target - input->measured) > tolerance)
    {
        ClearStability(detector);
        return detector->status;
    }

    if (!detector->tracking_stability)
    {
        detector->stable_since_ms = now_ms;
        detector->tracking_stability = 1;
    }

    if (!detector->status.ready)
    {
        uint32_t stable_ms = now_ms - detector->stable_since_ms;
        detector->status.stable_ms = stable_ms < config->dwell_ms ? stable_ms : config->dwell_ms;
        detector->status.ready = stable_ms >= config->dwell_ms;
    }
    return detector->status;
}
