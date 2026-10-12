#include "friction_log_stats.h"
#include <math.h>
#include <string.h>

void FrictionLogStatsReset(FrictionLogStats_s *stats)
{
    memset(stats, 0, sizeof(*stats));
}

bool FrictionLogStatsMatches(const FrictionLogStats_s *stats, const Friction_Telemetry_s *sample)
{
    if (stats->frames == 0U)
    {
        return true;
    }
    if (stats->revision != sample->tuning_revision || stats->requested_mask != sample->status.requested_mask ||
        stats->enabled_mask != sample->status.enabled_mask || stats->fault != sample->status.fault)
    {
        return false;
    }
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (stats->target[i] != sample->status.requested_rpm[i])
        {
            return false;
        }
    }
    return true;
}

void FrictionLogStatsObserve(FrictionLogStats_s *stats, const Friction_Telemetry_s *sample,
                             uint32_t now_ms, uint8_t usable_mask, const SpeedReady_Config_s *ready)
{
    if (stats->frames == 0U)
    {
        stats->start_ms = now_ms;
        stats->revision = sample->tuning_revision;
        stats->requested_mask = sample->status.requested_mask;
        stats->enabled_mask = sample->status.enabled_mask;
        stats->fault = sample->status.fault;
        memcpy(stats->target, sample->status.requested_rpm, sizeof(stats->target));
    }
    else
    {
        uint32_t gap = now_ms - stats->end_ms;
        if (gap > stats->maximum_gap_ms)
        {
            stats->maximum_gap_ms = gap;
        }
        if (sample->sample_id == stats->last_sample_id)
        {
            usable_mask = 0U;
        }
    }
    stats->last_sample_id = sample->sample_id;
    stats->end_ms = now_ms;
    ++stats->frames;
    usable_mask &= sample->status.enabled_mask;
    if (sample->status.fault)
    {
        usable_mask = 0U;
    }
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (!(usable_mask & (1U << i)))
        {
            continue;
        }
        FrictionLogWheelStats_s *wheel = &stats->wheel[i];
        float actual = sample->status.feedback[i].rpm;
        float tracking = sample->status.reference_rpm[i] - actual;
        float final = stats->target[i] - actual;
        if (wheel->valid_samples == 0U)
        {
            wheel->actual_min = wheel->actual_max = actual;
        }
        ++wheel->valid_samples;
        wheel->actual_min = fminf(wheel->actual_min, actual);
        wheel->actual_max = fmaxf(wheel->actual_max, actual);
        wheel->tracking_abs_max = fmaxf(wheel->tracking_abs_max, fabsf(tracking));
        wheel->output_abs_max = fmaxf(wheel->output_abs_max, fabsf(sample->output[i]));
        if (sample->status.feedback[i].temperature_c > wheel->temperature_max)
        {
            wheel->temperature_max = sample->status.feedback[i].temperature_c;
        }
        if (sample->tuning.motor[i].current_limit > 0.0f &&
            fabsf(sample->output[i]) >= sample->tuning.motor[i].current_limit - 1.0f)
        {
            ++wheel->saturated_samples;
        }
        if (fabsf(sample->status.reference_rpm[i] - stats->target[i]) > ready->reference_tolerance)
        {
            continue;
        }
        if (wheel->steady_samples == 0U)
        {
            wheel->final_min = wheel->final_max = final;
        }
        ++wheel->steady_samples;
        wheel->final_min = fminf(wheel->final_min, final);
        wheel->final_max = fmaxf(wheel->final_max, final);
        wheel->final_sum += final;
        wheel->final_square_sum += (double)final * final;
        float tolerance = fmaxf(ready->enter_absolute_error, ready->enter_relative_error * fabsf(stats->target[i]));
        if (fabsf(final) <= tolerance)
        {
            ++wheel->in_band_samples;
        }
    }
}
