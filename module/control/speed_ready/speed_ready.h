#ifndef SPEED_READY_H
#define SPEED_READY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** All speed fields use the same application-selected unit (e.g. rpm).
 * Relative tolerances are fractions, not percentages. Exit bounds must be at
 * least as wide as entry bounds. Times must be positive and below 2^31 ms. */
typedef struct
{
    float minimum_target;
    float reference_tolerance;
    float enter_absolute_error;
    float enter_relative_error;
    float exit_absolute_error;
    float exit_relative_error;
    uint32_t dwell_ms;
    uint32_t maximum_sample_gap_ms;
} SpeedReady_Config_s;

typedef struct
{
    float target;
    float reference;
    float measured;
    uint8_t enabled;
    uint8_t feedback_valid;
} SpeedReady_Input_s;

typedef struct
{
    uint32_t stable_ms;
    uint8_t ready;
} SpeedReady_Status_s;

/** Private tracking fields are exposed only for static allocation. One owner;
 * no HAL, OS, heap, device ID, or implicit clock. Use Init before Update. */
typedef struct
{
    SpeedReady_Config_s config;
    SpeedReady_Status_s status;
    float previous_target;
    uint32_t previous_ms;
    uint32_t stable_since_ms;
    uint8_t configured;
    uint8_t have_sample;
    uint8_t tracking_stability;
} SpeedReady_s;

/** Return 0 for invalid configuration; such a detector never reports ready. */
uint8_t SpeedReadyInit(SpeedReady_s *detector, const SpeedReady_Config_s *config);
/** Clear readiness/history, preserve configuration. Call after controller tuning. */
void SpeedReadyReset(SpeedReady_s *detector);
/** Evaluate one observation, using a monotonically increasing uint32 ms clock.
 * Unsigned clock wrap is supported. Missing/stale feedback, disable, target
 * changes, nonfinite data, duplicate time or excessive sample gaps invalidate
 * readiness. Caller determines feedback freshness. Dwell starts at the first
 * eligible sample, not before it. No output is sent to an actuator. */
SpeedReady_Status_s SpeedReadyUpdate(SpeedReady_s *detector,
                                    const SpeedReady_Input_s *input,
                                    uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
