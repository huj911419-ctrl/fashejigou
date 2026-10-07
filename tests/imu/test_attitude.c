#include "attitude.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846f
#define GRAVITY 9.80665f

static const AttitudeConfig_s config =
{
    .axis_map = {1, 2, 3}, .gravity_m_s2 = GRAVITY, .accel_gate_m_s2 = 1.0f,
    .correction_gain_s = 2.0f, .stationary_gyro_rad_s = 0.05f,
    .stationary_gyro_span = 0.01f, .stationary_accel_span = 0.2f,
    .max_gyro_rad_s = 35.0f, .calibration_samples = 32U, .max_step_ms = 10U, .stale_ms = 20U
};

static void Gravity(ImuSample_s *sample, float roll, float pitch)
{
    sample->accel_m_s2[0] = -GRAVITY * sinf(pitch);
    sample->accel_m_s2[1] = GRAVITY * sinf(roll) * cosf(pitch);
    sample->accel_m_s2[2] = GRAVITY * cosf(roll) * cosf(pitch);
}

static bool Step(Attitude_s *attitude, ImuSample_s *sample, bool allowed)
{
    sample->timestamp_ms += 2U;
    return AttitudeUpdate(attitude, sample, sample->timestamp_ms, allowed);
}

static void Calibrate(Attitude_s *attitude, ImuSample_s *sample)
{
    for (unsigned i = 0U; i < config.calibration_samples; ++i)
    {
        assert(Step(attitude, sample, true) == (i + 1U == config.calibration_samples));
    }
    assert(attitude->output.valid && attitude->output.state == ATTITUDE_TRACKING);
}

static void AssertQuaternion(const AttitudeOutput_s *output)
{
    float norm = 0.0f;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        assert(isfinite(output->quaternion[i]));
        norm += output->quaternion[i] * output->quaternion[i];
    }
    assert(fabsf(norm - 1.0f) < 0.00001f);
}

static void TestInitialization(void)
{
    Attitude_s attitude;
    assert(!AttitudeInit(&attitude, NULL));
    AttitudeConfig_s bad = config;
    bad.axis_map[0] = -1; /* Reflection, not a physical rotation. */
    assert(!AttitudeInit(&attitude, &bad));
    bad = config;
    bad.axis_map[1] = 1;
    assert(!AttitudeInit(&attitude, &bad));
    bad = config;
    bad.correction_gain_s = NAN;
    assert(!AttitudeInit(&attitude, &bad));
    assert(!attitude.output.valid && attitude.output.fault == ATTITUDE_FAULT_CONFIG);

    /* Every right-handed signed axis permutation, no left-handed mapping. */
    unsigned accepted = 0U;
    for (int x = -3; x <= 3; ++x)
    {
        for (int y = -3; y <= 3; ++y)
        {
            for (int z = -3; z <= 3; ++z)
            {
                bad = config;
                bad.axis_map[0] = (int8_t)x;
                bad.axis_map[1] = (int8_t)y;
                bad.axis_map[2] = (int8_t)z;
                accepted += AttitudeInit(&attitude, &bad);
            }
        }
    }
    assert(accepted == 24U);
    const float rolls[] = {0.0f, 0.5f, -0.5f, PI};
    for (unsigned i = 0U; i < sizeof(rolls) / sizeof(rolls[0]); ++i)
    {
        ImuSample_s sample = {.temperature_c = 23.0f};
        float pitch = i == 3U ? 0.0f : 0.25f;
        Gravity(&sample, rolls[i], pitch);
        assert(AttitudeInit(&attitude, &config));
        Calibrate(&attitude, &sample);
        AssertQuaternion(&attitude.output);
        assert(fabsf(remainderf(attitude.output.roll_rad - rolls[i], 2.0f * PI)) < 0.0001f);
        assert(fabsf(attitude.output.pitch_rad - pitch) < 0.0001f);
        assert(fabsf(attitude.output.yaw_rad) < 0.0001f);
    }
}

static void TestCalibration(void)
{
    Attitude_s attitude;
    ImuSample_s sample = {.temperature_c = 23.0f, .gyro_rad_s = {0.01f, -0.02f, 0.005f}};
    Gravity(&sample, 0.0f, 0.0f);
    assert(AttitudeInit(&attitude, &config));
    for (unsigned i = 0U; i < 50U; ++i)
    {
        assert(!Step(&attitude, &sample, false));
    }
    assert(attitude.output.calibration_count == 0U);
    Calibrate(&attitude, &sample);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        assert(fabsf(attitude.output.gyro_bias_rad_s[i] - sample.gyro_rad_s[i]) < 0.000001f);
        assert(fabsf(attitude.output.gyro_rad_s[i]) < 0.000001f);
    }
    for (unsigned i = 0U; i < 1000U; ++i)
    {
        assert(Step(&attitude, &sample, false));
    }
    assert(fabs(attitude.output.yaw_unwrapped_rad) < 0.00001);

    assert(AttitudeInit(&attitude, &config));
    sample.gyro_rad_s[0] = 0.2f;
    for (unsigned i = 0U; i < 50U; ++i)
    {
        assert(!Step(&attitude, &sample, true));
    }
    assert(attitude.output.calibration_count == 0U);
    sample.gyro_rad_s[0] = 0.0f;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        (void)Step(&attitude, &sample, true);
    }
    sample.gyro_rad_s[0] = 0.02f; /* Still below magnitude limit but window changed. */
    assert(!Step(&attitude, &sample, true) && attitude.output.calibration_count == 1U);
    sample.timestamp_ms += 50U;
    assert(!Step(&attitude, &sample, true) && attitude.output.calibration_count == 1U);
}

static void TestRotationAndWrap(void)
{
    Attitude_s attitude;
    ImuSample_s sample = {.temperature_c = 23.0f, .timestamp_ms = UINT32_MAX - 50U};
    Gravity(&sample, 0.0f, 0.0f);
    assert(AttitudeInit(&attitude, &config));
    Calibrate(&attitude, &sample); /* Calibration also crosses the uint32 clock wrap. */
    sample.gyro_rad_s[2] = 1.0f;
    for (unsigned i = 0U; i < 10000U; ++i)
    {
        assert(Step(&attitude, &sample, false));
        AssertQuaternion(&attitude.output);
    }
    assert(fabs(attitude.output.yaw_unwrapped_rad - 20.0) < 0.002);
    sample.gyro_rad_s[2] = -1.0f;
    for (unsigned i = 0U; i < 20000U; ++i)
    {
        assert(Step(&attitude, &sample, false));
    }
    assert(fabs(attitude.output.yaw_unwrapped_rad + 20.0) < 0.003);
    assert(attitude.output.yaw_continuous && attitude.output.heading_valid);

    /* Accelerometer cannot observe Z heading: this also verifies that gravity
     * correction does not incorrectly drive a rotating yaw back to zero. */
    AttitudeConfig_s mapped = config;
    mapped.axis_map[0] = 2;
    mapped.axis_map[1] = -1;
    assert(AttitudeInit(&attitude, &mapped));
    memset(sample.gyro_rad_s, 0, sizeof(sample.gyro_rad_s));
    Calibrate(&attitude, &sample);
    sample.gyro_rad_s[0] = 0.5f;
    assert(Step(&attitude, &sample, false));
    assert(attitude.output.gyro_rad_s[1] == -0.5f);
}

static void TestCorrectionAndRejection(void)
{
    Attitude_s attitude;
    ImuSample_s sample = {.temperature_c = 23.0f};
    Gravity(&sample, 0.0f, 0.0f);
    assert(AttitudeInit(&attitude, &config));
    Calibrate(&attitude, &sample);
    Gravity(&sample, 0.4f, -0.2f);
    for (unsigned i = 0U; i < 3000U; ++i)
    {
        assert(Step(&attitude, &sample, false));
    }
    assert(fabsf(attitude.output.roll_rad - 0.4f) < 0.001f);
    assert(fabsf(attitude.output.pitch_rad + 0.2f) < 0.001f);
    assert(attitude.output.accel_correction_used);
    float before[4];
    memcpy(before, attitude.output.quaternion, sizeof(before));
    sample.accel_m_s2[0] = 40.0f;
    assert(Step(&attitude, &sample, false));
    assert(!attitude.output.accel_correction_used);
    assert(fabsf(attitude.output.quaternion[0] - before[0]) < 0.00001f);
    memset(sample.accel_m_s2, 0, sizeof(sample.accel_m_s2)); /* Freefall: gyro only. */
    assert(Step(&attitude, &sample, false) && !attitude.output.accel_correction_used);

    assert(AttitudeInit(&attitude, &config));
    Gravity(&sample, 0.0f, PI * 0.5f);
    Calibrate(&attitude, &sample);
    AssertQuaternion(&attitude.output);
    assert(!attitude.output.heading_valid && !attitude.output.yaw_continuous);
}

static void TestFaults(void)
{
    for (unsigned fault = 0U; fault < 6U; ++fault)
    {
        Attitude_s attitude;
        ImuSample_s sample = {.temperature_c = 23.0f};
        Gravity(&sample, 0.0f, 0.0f);
        assert(AttitudeInit(&attitude, &config));
        Calibrate(&attitude, &sample);
        assert(!AttitudeUpdate(&attitude, &sample, sample.timestamp_ms, false));
        assert(attitude.output.valid); /* A duplicate is not an integration step. */
        if (fault == 0U)
        {
            sample.gyro_rad_s[0] = NAN;
            assert(!Step(&attitude, &sample, false));
        }
        else if (fault == 1U)
        {
            sample.accel_m_s2[1] = FLT_MAX;
            assert(!Step(&attitude, &sample, false));
        }
        else if (fault == 2U)
        {
            sample.timestamp_ms += 10U;
            assert(!Step(&attitude, &sample, false));
        }
        else if (fault == 3U)
        {
            AttitudeCheckFresh(&attitude, sample.timestamp_ms + 21U);
        }
        else if (fault == 4U)
        {
            assert(!AttitudeUpdate(&attitude, &sample, sample.timestamp_ms - 1U, false));
        }
        else
        {
            AttitudeInvalidate(&attitude, ATTITUDE_FAULT_SOURCE);
        }
        assert(!attitude.output.valid && attitude.output.state == ATTITUDE_LOST);
        sample.gyro_rad_s[0] = 0.0f;
        Gravity(&sample, 0.0f, 0.0f);
        assert(!Step(&attitude, &sample, true)); /* No implicit reference reset. */
        AssertQuaternion(&attitude.output); /* Last finite value is diagnostic only. */
    }
}

static void TestBodyFrameIntegration(void)
{
    Attitude_s attitude;
    AttitudeConfig_s gyro_only = config;
    gyro_only.correction_gain_s = 0.0f;
    ImuSample_s sample = {.temperature_c = 23.0f};
    Gravity(&sample, PI * 0.5f, 0.0f);
    assert(AttitudeInit(&attitude, &gyro_only));
    Calibrate(&attitude, &sample);
    sample.gyro_rad_s[1] = PI * 0.5f;
    for (unsigned i = 0U; i < 500U; ++i)
    {
        assert(Step(&attitude, &sample, false));
    }
    /* Start at 90 degrees roll, then rotate 90 degrees about BODY Y:
     * independently known orientation is quaternion (1,1,1,1)/2. */
    for (unsigned i = 0U; i < 4U; ++i)
    {
        assert(fabsf(attitude.output.quaternion[i] - 0.5f) < 0.0001f);
    }
    assert(!attitude.output.accel_correction_used);

    /* Skip across the near-vertical band in one legal step. The quaternion
     * remains useful but continuous heading must be invalidated. */
    Gravity(&sample, 0.0f, 1.45f);
    sample.gyro_rad_s[1] = 0.0f;
    assert(AttitudeInit(&attitude, &gyro_only));
    Calibrate(&attitude, &sample);
    assert(attitude.output.yaw_continuous);
    sample.gyro_rad_s[1] = 30.0f;
    sample.timestamp_ms += 10U;
    assert(AttitudeUpdate(&attitude, &sample, sample.timestamp_ms, false));
    assert(attitude.output.valid && !attitude.output.yaw_continuous);
}

int main(void)
{
    TestInitialization();
    TestCalibration();
    TestRotationAndWrap();
    TestCorrectionAndRejection();
    TestFaults();
    TestBodyFrameIntegration();
    puts("PASS: stationary bias/tilt, 24 mappings, analytical rotation, yaw/clock wrap, correction gating and latched loss");
    return 0;
}
