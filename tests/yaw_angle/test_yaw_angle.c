#include "yaw_angle.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static const YawAngleConfig_s defaults =
{
    .kp_s = 2.0f, .ki_s2 = 0.0f, .integral_limit_rad_s = 0.5f,
    .max_rate_rad_s = 1.0f, .acceleration_rad_s2 = 4.0f,
    .max_step_ms = 10U, .feedback_timeout_ms = 20U, .command_timeout_ms = 100U
};

static YawAngleInput_s Input(uint32_t now, double target, double measured)
{
    return (YawAngleInput_s){.target_rad = target, .measured_rad = measured,
        .feedback_ms = now, .command_ms = now, .reference_id = 1U,
        .feedback_valid = true, .enable = true};
}

static void TestConfigAndLimits(void)
{
    YawAngle_s controller;
    assert(!YawAngleInit(NULL, &defaults));
    assert(!YawAngleInit(&controller, NULL));
    assert(controller.output.fault == YAW_ANGLE_ERROR_CONFIG);
    YawAngleInput_s input = Input(0U, 1.0, 0.0);
    input.enable = false;
    assert(!YawAngleStep(&controller, &input, 0U));
    assert(controller.output.fault == YAW_ANGLE_ERROR_CONFIG);
    YawAngleConfig_s bad = defaults;
    bad.acceleration_rad_s2 = NAN;
    assert(!YawAngleInit(&controller, &bad));
    bad = defaults;
    bad.feedback_timeout_ms = 1U;
    assert(!YawAngleInit(&controller, &bad));
    assert(YawAngleInit(&controller, &defaults));
    float previous = 0.0f;
    for (uint32_t now = 0U; now <= 1000U; now += 2U)
    {
        input = Input(now, 100.0, 0.0);
        assert(YawAngleStep(&controller, &input, now));
        float rate = controller.output.rate_rad_s;
        assert(isfinite(rate) && rate <= 1.0f && rate >= 0.0f);
        assert(rate - previous <= 0.008001f);
        previous = rate;
    }
    assert(previous == 1.0f && controller.output.limited);
    input = Input(1002U, -100.0, 0.0);
    assert(YawAngleStep(&controller, &input, 1002U));
    assert(fabsf(controller.output.rate_rad_s - 0.992f) < 0.00001f);
    input.enable = false;
    assert(!YawAngleStep(&controller, &input, 1004U));
    assert(controller.output.rate_rad_s == 0.0f && controller.output.state == YAW_ANGLE_OFF);
}

static void TestFaultsAndClock(void)
{
    YawAngle_s controller;
    assert(YawAngleInit(&controller, &defaults));
    YawAngleInput_s input = Input(UINT32_MAX - 1U, 1.0, 0.0);
    assert(YawAngleStep(&controller, &input, UINT32_MAX - 1U));
    input = Input(0U, 1.0, 0.0);
    assert(YawAngleStep(&controller, &input, 0U));
    assert(controller.output.rate_rad_s > 0.0f);
    input.feedback_ms = UINT32_MAX - 30U;
    assert(!YawAngleStep(&controller, &input, 2U));
    assert(controller.output.fault == YAW_ANGLE_ERROR_FEEDBACK && controller.output.rate_rad_s == 0.0f);
    input = Input(4U, 1.0, 0.0);
    assert(!YawAngleStep(&controller, &input, 4U)); /* Fresh feedback alone cannot restart. */
    input.enable = false;
    assert(!YawAngleStep(&controller, &input, 6U));
    input = Input(8U, 1.0, 0.0);
    assert(YawAngleStep(&controller, &input, 8U));
    ++input.reference_id;
    assert(!YawAngleStep(&controller, &input, 10U));
    assert(controller.output.fault == YAW_ANGLE_ERROR_REFERENCE);

    assert(YawAngleInit(&controller, &defaults));
    input = Input(1000U, 1.0, 0.0);
    input.command_ms = 899U;
    assert(!YawAngleStep(&controller, &input, 1000U));
    assert(controller.output.fault == YAW_ANGLE_ERROR_COMMAND);
    assert(YawAngleInit(&controller, &defaults));
    input = Input(1000U, 1.0, 0.0);
    assert(YawAngleStep(&controller, &input, 1000U));
    input = Input(1011U, 1.0, 0.0);
    assert(!YawAngleStep(&controller, &input, 1011U));
    assert(controller.output.fault == YAW_ANGLE_ERROR_TIMING);
    assert(YawAngleInit(&controller, &defaults));
    assert(YawAngleStep(&controller, &input, 1011U));
    assert(!YawAngleStep(&controller, &input, 1011U)); /* Same tick is not a valid active step. */
    assert(YawAngleInit(&controller, &defaults));
    input = Input(1000U, NAN, 0.0);
    assert(!YawAngleStep(&controller, &input, 1000U));
    assert(controller.output.fault == YAW_ANGLE_ERROR_DATA && isfinite(controller.output.rate_rad_s));
    assert(YawAngleInit(&controller, &defaults));
    input = Input(1000U, DBL_MAX, -DBL_MAX);
    assert(!YawAngleStep(&controller, &input, 1000U));
    assert(YawAngleInit(&controller, &defaults));
    input = Input(1000U, 1.0, 0.0);
    input.feedback_ms = 1001U;
    assert(!YawAngleStep(&controller, &input, 1000U)); /* Future timestamp is rejected. */
}

static void TestIntegralAndFrames(void)
{
    YawAngle_s controller;
    YawAngleConfig_s config = defaults;
    config.ki_s2 = 1.0f;
    assert(YawAngleInit(&controller, &config));
    for (uint32_t now = 0U; now <= 10000U; now += 2U)
    {
        YawAngleInput_s input = Input(now, 100.0, 0.0);
        assert(YawAngleStep(&controller, &input, now));
        assert(controller.output.integral_rad_s == 0.0f); /* No windup against limits. */
    }
    assert(YawAngleInit(&controller, &config));
    for (uint32_t now = 0U; now <= 1000U; now += 2U)
    {
        YawAngleInput_s input = Input(now, 0.1, 0.0);
        assert(YawAngleStep(&controller, &input, now));
    }
    assert(controller.output.integral_rad_s > 0.05f && controller.output.integral_rad_s < 0.11f);
    YawAngle_s second;
    assert(YawAngleInit(&second, &defaults));
    YawAngleInput_s input = Input(1002U, -3.2, 3.2);
    assert(YawAngleStep(&second, &input, 1002U));
    input = Input(1004U, -3.2, 3.2);
    assert(YawAngleStep(&second, &input, 1004U));
    assert(second.output.rate_rad_s < 0.0f && second.output.error_rad == -6.4);
    assert(controller.output.integral_rad_s > 0.05f); /* Instances independent; no implicit shortest path. */
}

static void TestIllustrativePlant(void)
{
    /* Synthetic first-order velocity response, NOT a measured J4310 model. */
    YawAngle_s controller;
    assert(YawAngleInit(&controller, &defaults));
    double position = 0.0;
    double velocity = 0.0;
    const double target = 7.0; /* More than one revolution: never wrap target. */
    for (uint32_t now = 0U; now <= 14000U; now += 2U)
    {
        YawAngleInput_s input = Input(now, target, position);
        assert(YawAngleStep(&controller, &input, now));
        velocity += ((double)controller.output.rate_rad_s - velocity) * (0.002 / 0.04);
        position += velocity * 0.002;
        if (now == 11000U)
        {
            position -= 0.2; /* Demonstrate recovery from an artificial disturbance. */
        }
        assert(isfinite(position) && fabs(controller.output.rate_rad_s) <= 1.0);
    }
    assert(fabs(position - target) < 0.002);
}

int main(void)
{
    TestConfigAndLimits();
    TestFaultsAndClock();
    TestIntegralAndFrames();
    TestIllustrativePlant();
    puts("PASS: angle-to-rate core, limits, anti-windup, stale commands/feedback, reference resets, faults and synthetic plant");
    return 0;
}
