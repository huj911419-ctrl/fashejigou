#include "friction_log_stats.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    const SpeedReady_Config_s ready = {.reference_tolerance = 1.0f,
        .enter_absolute_error = 10.0f, .enter_relative_error = 0.02f};
    FrictionLogStats_s stats;
    FrictionLogStatsReset(&stats);
    Friction_Telemetry_s sample = {0};
    sample.status.enabled_mask = sample.status.requested_mask = 1U;
    sample.status.requested_rpm[0] = 1000.0f;
    sample.status.reference_rpm[0] = 500.0f;
    sample.status.feedback[0].rpm = 490.0f;
    sample.status.feedback[0].temperature_c = 25U;
    sample.tuning.motor[0].current_limit = sample.output[0] = 14000.0f;
    sample.sample_id = 1U;
    FrictionLogStatsObserve(&stats, &sample, UINT32_MAX - 10U, 1U, &ready);
    assert(stats.wheel[0].valid_samples == 1U && stats.wheel[0].steady_samples == 0U);
    assert(stats.wheel[0].saturated_samples == 1U);
    sample.status.reference_rpm[0] = 1000.0f;
    sample.status.feedback[0].rpm = 990.0f;
    sample.output[0] = 1000.0f;
    ++sample.sample_id;
    FrictionLogStatsObserve(&stats, &sample, 9U, 1U, &ready);
    assert(stats.maximum_gap_ms == 20U && stats.wheel[0].steady_samples == 1U);
    sample.status.feedback[0].rpm = 970.0f;
    ++sample.sample_id;
    FrictionLogStatsObserve(&stats, &sample, 29U, 1U, &ready);
    assert(stats.wheel[0].steady_samples == 2U && stats.wheel[0].in_band_samples == 1U);
    assert(stats.wheel[0].actual_min == 490.0f && stats.wheel[0].actual_max == 990.0f);
    assert(stats.wheel[0].final_sum == 40.0 && stats.wheel[0].final_square_sum == 1000.0);
    FrictionLogStatsObserve(&stats, &sample, 49U, 1U, &ready);
    assert(stats.wheel[0].valid_samples == 3U); /* Repeated motor sample is not counted twice. */
    ++sample.sample_id;
    FrictionLogStatsObserve(&stats, &sample, 69U, 0U, &ready);
    assert(stats.wheel[0].valid_samples == 3U); /* Invalid/stale observations excluded by caller. */
    assert(FrictionLogStatsMatches(&stats, &sample));
    ++sample.tuning_revision;
    assert(!FrictionLogStatsMatches(&stats, &sample));
    --sample.tuning_revision;
    sample.status.requested_rpm[0] = 1100.0f;
    assert(!FrictionLogStatsMatches(&stats, &sample));
    sample.status.requested_rpm[0] = 1000.0f;
    sample.status.enabled_mask = 0U;
    assert(!FrictionLogStatsMatches(&stats, &sample));
    sample.status.enabled_mask = 1U;
    sample.status.fault = FRICTION_FAULT_COMMAND;
    assert(!FrictionLogStatsMatches(&stats, &sample));
    FrictionLogStatsReset(&stats);
    ++sample.sample_id;
    FrictionLogStatsObserve(&stats, &sample, 100U, 1U, &ready);
    assert(stats.frames == 1U && stats.wheel[0].valid_samples == 0U);
    assert(isfinite(stats.wheel[0].final_sum));
    puts("PASS: ramp/steady separation, band counts, observed extrema/RMS, saturation, filtering, duplicate samples, revision boundaries and wrapping time");
    return 0;
}
