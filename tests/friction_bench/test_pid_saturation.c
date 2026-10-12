#include "controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static float step_seconds = 0.001f;

float DWT_GetDeltaT(uint32_t *last)
{
    ++*last;
    return step_seconds;
}

static PIDInstance Controller(float kp, float ki, float limit, float integral_limit)
{
    PIDInstance pid;
    PID_Init_Config_s config = {
        .Kp = kp,
        .Ki = ki,
        .MaxOut = limit,
        .IntegralLimit = integral_limit,
        .Improve = PID_Continuous | PID_Integral_Limit | PID_ConditionalIntegration
    };
    PIDInit(&pid, &config);
    return pid;
}

static void Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static void TestSaturationBothDirections(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float sign = (float)direction;
        PIDInstance pid = Controller(1.0f, 100.0f, 10.0f, 8.0f);
        for (unsigned i = 0U; i < 2000U; ++i)
        {
            Near(PIDCalculate(&pid, 0.0f, sign * 20.0f), sign * 10.0f);
            Near(pid.Iout, 0.0f); /* Saturation must not charge even the first step. */
        }

        /* I has the opposite sign to the saturated output. Changing I toward
         * zero would STILL increase saturation; testing Err*I misses this. */
        pid.Iout = -sign * 3.0f;
        Near(PIDCalculate(&pid, 0.0f, sign * 20.0f), sign * 10.0f);
        Near(pid.Iout, -sign * 3.0f);

        /* Rejecting an increment must not subsequently clamp its stale
         * candidate into I and increase I despite output saturation. */
        pid.Iout = sign * 7.9f;
        Near(PIDCalculate(&pid, 0.0f, sign * 20.0f), sign * 10.0f);
        Near(pid.Iout, sign * 7.9f);
    }
}

static void TestRecoveryAndIntegralLimit(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float sign = (float)direction;
        PIDInstance pid = Controller(1.0f, 100.0f, 10.0f, 20.0f);
        pid.Iout = sign * 15.0f;
        Near(PIDCalculate(&pid, sign, 0.0f), sign * 10.0f);
        Near(pid.Iout, sign * 14.9f); /* Permit unwinding while still saturated. */

        pid = Controller(0.0f, 100.0f, 10.0f, 2.0f);
        pid.Iout = sign * 1.95f;
        Near(PIDCalculate(&pid, 0.0f, sign), sign * 2.0f);
        Near(pid.Iout, sign * 2.0f);
        Near(PIDCalculate(&pid, 0.0f, 0.0f), sign * 2.0f);
        /* A reduced integral limit bounds stored I, even at zero error. */
        pid.IntegralLimit = 1.0f;
        Near(PIDCalculate(&pid, 0.0f, 0.0f), sign);

        pid = Controller(1.0f, 0.0f, 10.0f, 10.0f);
        Near(PIDCalculate(&pid, 0.0f, sign * 20.0f), sign * 10.0f);
        Near(pid.Iout, 0.0f);
    }
}

static void TestCandidateIncludesDerivative(void)
{
    PIDInstance pid = Controller(0.0f, 100.0f, 10.0f, 10.0f);
    pid.Kd = 0.02f;
    Near(PIDCalculate(&pid, 0.0f, 1.0f), 10.0f);
    Near(pid.Iout, 0.0f);
    Near(PIDCalculate(&pid, 0.0f, 1.0f), 0.1f);
    Near(pid.Iout, 0.1f);

    pid = Controller(1.0f, 100.0f, 10.0f, 10.0f);
    pid.Iout = 8.95f;
    PIDCalculate(&pid, 0.0f, 1.0f); /* Existing output below limit, candidate above. */
    Near(pid.Iout, 8.95f);
}

static void TestIndependentInstancesAndTime(void)
{
    PIDInstance first = Controller(1.0f, 2.0f, 100.0f, 100.0f);
    PIDInstance second = Controller(1.0f, 2.0f, 100.0f, 100.0f);
    for (unsigned i = 0U; i < 100U; ++i)
    {
        PIDCalculate(&first, 0.0f, 1.0f);
        PIDCalculate(&second, 0.0f, -2.0f);
    }
    Near(first.Iout, 0.2f);
    Near(second.Iout, -0.4f);
    step_seconds = 0.005f;
    PIDCalculate(&first, 0.0f, 1.0f);
    Near(first.Iout, 0.21f);
    Near(second.Iout, -0.4f);
    step_seconds = 0.001f;
}

static void TestLegacyOptOut(void)
{
    PIDInstance pid = Controller(1.0f, 100.0f, 10.0f, 8.0f);
    pid.Improve = PID_Continuous | PID_Integral_Limit;
    Near(PIDCalculate(&pid, 0.0f, 20.0f), 10.0f);
    Near(pid.Iout, 2.0f); /* Legacy behavior intentionally remains available. */
    Near(PIDCalculate(&pid, 0.0f, 20.0f), 10.0f);
    Near(pid.Iout, 2.0f);
    pid.Improve = PID_Integral_Limit;
    Near(PIDCalculate(&pid, 1.0f, 1.0f), 0.0f);
}

int main(void)
{
    TestSaturationBothDirections();
    TestRecoveryAndIntegralLimit();
    TestCandidateIncludesDerivative();
    TestIndependentInstancesAndTime();
    TestLegacyOptOut();
    puts("PASS: conditional integration, signed saturation/recovery, I bounds, D contribution, timing and legacy opt-out");
    return 0;
}
