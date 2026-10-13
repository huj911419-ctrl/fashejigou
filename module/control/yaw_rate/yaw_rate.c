#include "yaw_rate.h"
#include <math.h>
#include <string.h>

bool YawRateInit(YawRate_s *yaw, const YawRateConfig_s *config)
{
    memset(yaw, 0, sizeof(*yaw));
    yaw->inhibit = YAW_INHIBIT_CONFIG;
    if (config == NULL)
    {
        return false;
    }
    yaw->config = *config;
    yaw->valid_config = isfinite(config->max_rate_rad_s) && config->max_rate_rad_s > 0.0f &&
                        isfinite(config->acceleration_rad_s2) && config->acceleration_rad_s2 > 0.0f &&
                        config->max_step_ms > 0U && config->max_step_ms <= 100U;
    yaw->inhibit = yaw->valid_config ? YAW_INHIBIT_REARM_REQUIRED : YAW_INHIBIT_CONFIG;
    return yaw->valid_config;
}

void YawRateDisarm(YawRate_s *yaw)
{
    yaw->enabled = false;
    yaw->off_seen = false;
    yaw->target_rad_s = 0.0f;
    yaw->reference_rad_s = 0.0f;
    yaw->inhibit = YAW_INHIBIT_REARM_REQUIRED;
}

void YawRateStep(YawRate_s *yaw, uint32_t now_ms, bool valid, bool ready,
                 bool arm, float rate_rad_s)
{
    uint32_t elapsed = now_ms - yaw->last_step_ms;
    bool timing_ok = !yaw->initialized || (elapsed > 0U && elapsed <= yaw->config.max_step_ms);
    bool rising = arm && !yaw->previous_arm;
    yaw->last_step_ms = now_ms;
    yaw->previous_arm = arm;
    bool first = !yaw->initialized;
    yaw->initialized = true;
    bool rate_ok = isfinite(rate_rad_s) && fabsf(rate_rad_s) <= yaw->config.max_rate_rad_s;
    uint32_t inhibit = (yaw->valid_config ? 0U : YAW_INHIBIT_CONFIG) |
                       (valid ? 0U : YAW_INHIBIT_COMMAND) |
                       (ready ? 0U : YAW_INHIBIT_ACTUATOR) |
                       (timing_ok ? 0U : YAW_INHIBIT_TIMING) |
                       (rate_ok ? 0U : YAW_INHIBIT_RATE);
    if (inhibit != 0U)
    {
        YawRateDisarm(yaw);
        yaw->inhibit = inhibit;
        return;
    }
    if (!arm)
    {
        YawRateDisarm(yaw);
        yaw->off_seen = true;
        yaw->inhibit = YAW_INHIBIT_SWITCH_OFF;
        return;
    }
    if (rising)
    {
        inhibit = (yaw->off_seen ? 0U : YAW_INHIBIT_REARM_REQUIRED) |
                  (rate_rad_s == 0.0f ? 0U : YAW_INHIBIT_START_NOT_ZERO);
        yaw->enabled = yaw->off_seen && rate_rad_s == 0.0f;
        yaw->off_seen = false;
    }
    if (!yaw->enabled)
    {
        yaw->inhibit = inhibit != 0U ? inhibit : YAW_INHIBIT_REARM_REQUIRED;
        return;
    }
    yaw->inhibit = 0U;
    yaw->target_rad_s = rate_rad_s;
    /* Convert to seconds first: multiplying a finite acceleration by raw
     * milliseconds can overflow even though the final step is representable. */
    float max_delta = first ? 0.0f : yaw->config.acceleration_rad_s2 * ((float)elapsed * 0.001f);
    float delta = yaw->target_rad_s - yaw->reference_rad_s;
    yaw->reference_rad_s += fminf(max_delta, fmaxf(-max_delta, delta));
}
