#include "yaw_angle.h"
#include <math.h>
#include <string.h>

static bool ConfigValid(const YawAngleConfig_s *config)
{
    return config != NULL &&
           isfinite(config->kp_s) && config->kp_s > 0.0f &&
           isfinite(config->ki_s2) && config->ki_s2 >= 0.0f &&
           isfinite(config->integral_limit_rad_s) && config->integral_limit_rad_s >= 0.0f &&
           isfinite(config->max_rate_rad_s) && config->max_rate_rad_s > 0.0f &&
           config->integral_limit_rad_s <= config->max_rate_rad_s &&
           isfinite(config->acceleration_rad_s2) && config->acceleration_rad_s2 > 0.0f &&
           config->max_step_ms >= 1U && config->max_step_ms <= 100U &&
           config->feedback_timeout_ms >= config->max_step_ms && config->feedback_timeout_ms <= 1000U &&
           config->command_timeout_ms >= config->max_step_ms && config->command_timeout_ms <= 1000U;
}

static bool Fault(YawAngle_s *controller, uint32_t reason)
{
    controller->output.fault |= reason;
    controller->output.state = YAW_ANGLE_FAULT;
    controller->output.rate_rad_s = 0.0f;
    controller->output.integral_rad_s = 0.0f;
    controller->output.limited = false;
    return false;
}

bool YawAngleInit(YawAngle_s *controller, const YawAngleConfig_s *config)
{
    if (controller == NULL)
    {
        return false;
    }
    memset(controller, 0, sizeof(*controller));
    if (!ConfigValid(config))
    {
        return Fault(controller, YAW_ANGLE_ERROR_CONFIG);
    }
    controller->config = *config;
    controller->configured = true;
    return true;
}

static double Clamp(double value, double minimum, double maximum)
{
    return fmin(maximum, fmax(minimum, value));
}

static double LimitRate(const YawAngle_s *controller, double requested, double seconds)
{
    double limit = controller->config.max_rate_rad_s;
    double rate = Clamp(requested, -limit, limit);
    double maximum_delta = (double)controller->config.acceleration_rad_s2 * seconds;
    return Clamp(rate, (double)controller->output.rate_rad_s - maximum_delta,
                 (double)controller->output.rate_rad_s + maximum_delta);
}

bool YawAngleStep(YawAngle_s *controller, const YawAngleInput_s *input, uint32_t now_ms)
{
    if (controller == NULL)
    {
        return false;
    }
    if (!controller->configured)
    {
        return Fault(controller, YAW_ANGLE_ERROR_CONFIG);
    }
    if (input == NULL)
    {
        return Fault(controller, YAW_ANGLE_ERROR_DATA);
    }
    uint32_t elapsed = controller->have_step ? now_ms - controller->last_step_ms : 0U;
    bool timing_ok = !controller->have_step || (elapsed > 0U && elapsed <= controller->config.max_step_ms);
    controller->have_step = true;
    controller->last_step_ms = now_ms;
    if (!input->enable)
    {
        memset(&controller->output, 0, sizeof(controller->output));
        controller->have_reference = false;
        return false;
    }
    if (controller->output.fault != 0U)
    {
        return false;
    }
    if (!timing_ok)
    {
        return Fault(controller, YAW_ANGLE_ERROR_TIMING);
    }
    if (!input->feedback_valid || now_ms - input->feedback_ms > controller->config.feedback_timeout_ms)
    {
        return Fault(controller, YAW_ANGLE_ERROR_FEEDBACK);
    }
    if (now_ms - input->command_ms > controller->config.command_timeout_ms)
    {
        return Fault(controller, YAW_ANGLE_ERROR_COMMAND);
    }
    if (controller->have_reference && controller->reference_id != input->reference_id)
    {
        return Fault(controller, YAW_ANGLE_ERROR_REFERENCE);
    }
    double error = input->target_rad - input->measured_rad;
    if (!isfinite(input->target_rad) || !isfinite(input->measured_rad) || !isfinite(error))
    {
        return Fault(controller, YAW_ANGLE_ERROR_DATA);
    }
    controller->reference_id = input->reference_id;
    controller->have_reference = true;
    double seconds = (double)elapsed * 0.001;
    double p = (double)controller->config.kp_s * error;
    double candidate_i = controller->config.ki_s2 == 0.0f ? 0.0 :
                         controller->output.integral_rad_s + (double)controller->config.ki_s2 * error * seconds;
    if (!isfinite(p) || !isfinite(candidate_i))
    {
        return Fault(controller, YAW_ANGLE_ERROR_DATA);
    }
    candidate_i = Clamp(candidate_i, -controller->config.integral_limit_rad_s,
                        controller->config.integral_limit_rad_s);
    double requested = p + candidate_i;
    if (!isfinite(requested))
    {
        return Fault(controller, YAW_ANGLE_ERROR_DATA);
    }
    double rate = LimitRate(controller, requested, seconds);
    /* 条件积分：限速或限加速度时，不继续积累推动饱和方向的积分。 */
    bool pushes_limit = (error > 0.0 && requested > rate) || (error < 0.0 && requested < rate);
    if (pushes_limit)
    {
        candidate_i = controller->output.integral_rad_s;
        requested = p + candidate_i;
        rate = LimitRate(controller, requested, seconds);
    }
    controller->output = (YawAngleOutput_s)
    {
        .error_rad = error,
        .integral_rad_s = (float)candidate_i,
        .rate_rad_s = (float)rate,
        .state = YAW_ANGLE_TRACKING,
        .limited = rate != requested
    };
    return true;
}
