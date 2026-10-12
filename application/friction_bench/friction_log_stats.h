#ifndef FRICTION_LOG_STATS_H
#define FRICTION_LOG_STATS_H

#include "friction.h"

typedef struct
{
    uint32_t valid_samples;
    uint32_t steady_samples;
    uint32_t in_band_samples;
    uint32_t saturated_samples;
    float actual_min;
    float actual_max;
    float tracking_abs_max;
    float final_min;
    float final_max;
    float output_abs_max;
    double final_sum;
    double final_square_sum;
    uint8_t temperature_max;
} FrictionLogWheelStats_s;

typedef struct
{
    FrictionLogWheelStats_s wheel[FRICTION_MOTOR_COUNT];
    float target[FRICTION_MOTOR_COUNT];
    uint32_t revision;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t frames;
    uint32_t maximum_gap_ms;
    uint32_t last_sample_id;
    uint32_t fault;
    uint8_t requested_mask;
    uint8_t enabled_mask;
} FrictionLogStats_s;

/** Pure C, one diagnostic-task owner, decimated observations only.
 * A window must not mix target/mask/fault/parameter revisions. */
bool FrictionLogStatsMatches(const FrictionLogStats_s *stats, const Friction_Telemetry_s *sample);
void FrictionLogStatsReset(FrictionLogStats_s *stats);
/** usable_mask is prepared by the caller after fresh/valid/finite/not-clipped
 * checks; enabled/fault and duplicate-sample checks are also enforced here.
 * Steady statistics require reference within ready.reference_tolerance of target.
 * Counts describe observations, not time occupancy or full-rate peak bounds. */
void FrictionLogStatsObserve(FrictionLogStats_s *stats, const Friction_Telemetry_s *sample,
                             uint32_t now_ms, uint8_t usable_mask, const SpeedReady_Config_s *ready);

#endif
