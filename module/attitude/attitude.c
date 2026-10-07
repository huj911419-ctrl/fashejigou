#include "attitude.h"
#include <math.h>
#include <string.h>

#define ATTITUDE_PI 3.14159265358979323846f

static bool InRange(float value, float low, float high)
{
    return isfinite(value) && value >= low && value <= high;
}

static bool MappingValid(const int8_t map[3])
{
    int determinant = 1;
    unsigned axes[3];
    for (unsigned i = 0U; i < 3U; ++i)
    {
        int value = map[i];
        if (value == 0 || value < -3 || value > 3)
        {
            return false;
        }
        axes[i] = (unsigned)(value < 0 ? -value : value);
        determinant *= value < 0 ? -1 : 1;
        for (unsigned j = 0U; j < i; ++j)
        {
            if (axes[j] == axes[i])
            {
                return false;
            }
            if (axes[j] > axes[i])
            {
                determinant = -determinant;
            }
        }
    }
    return determinant == 1;
}

bool AttitudeInit(Attitude_s *attitude, const AttitudeConfig_s *config)
{
    if (attitude == NULL)
    {
        return false;
    }
    memset(attitude, 0, sizeof(*attitude));
    attitude->output.quaternion[0] = 1.0f;
    if (config == NULL || !MappingValid(config->axis_map) ||
        !InRange(config->gravity_m_s2, 8.0f, 11.0f) ||
        !InRange(config->accel_gate_m_s2, 0.01f, 3.0f) ||
        !InRange(config->correction_gain_s, 0.0f, 20.0f) ||
        !InRange(config->stationary_gyro_rad_s, 0.001f, 0.1f) ||
        !InRange(config->stationary_gyro_span, 0.0001f, 0.05f) ||
        !InRange(config->stationary_accel_span, 0.001f, 1.0f) ||
        !InRange(config->max_gyro_rad_s, 1.0f, 40.0f) ||
        config->calibration_samples < 32U || config->calibration_samples > 10000U ||
        config->max_step_ms == 0U || config->max_step_ms > 20U ||
        config->stale_ms < config->max_step_ms || config->stale_ms > 1000U)
    {
        attitude->output.state = ATTITUDE_BAD_CONFIG;
        attitude->output.fault = ATTITUDE_FAULT_CONFIG;
        return false;
    }
    attitude->config = *config;
    attitude->output.state = ATTITUDE_CALIBRATING;
    return true;
}

void AttitudeInvalidate(Attitude_s *attitude, uint32_t fault)
{
    attitude->output.valid = false;
    attitude->output.heading_valid = false;
    attitude->output.yaw_continuous = false;
    attitude->output.accel_correction_used = false;
    attitude->output.fault |= fault;
    if (attitude->output.state != ATTITUDE_BAD_CONFIG)
    {
        attitude->output.state = ATTITUDE_LOST;
    }
}

void AttitudeCheckFresh(Attitude_s *attitude, uint32_t now_ms)
{
    if (attitude->have_sample && now_ms - attitude->last_sample_ms > attitude->config.stale_ms)
    {
        if (attitude->output.state == ATTITUDE_TRACKING)
        {
            AttitudeInvalidate(attitude, ATTITUDE_FAULT_TIMING);
        }
        else if (attitude->output.state == ATTITUDE_CALIBRATING)
        {
            attitude->output.calibration_count = 0U;
        }
    }
}

static float Norm3(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

static void UpdateAngles(AttitudeOutput_s *output, bool first)
{
    const float *q = output->quaternion;
    float sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    float previous_yaw = output->yaw_rad;
    output->roll_rad = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                              1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    output->pitch_rad = asinf(fminf(1.0f, fmaxf(-1.0f, sin_pitch)));
    output->yaw_rad = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                             1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    output->heading_valid = fabsf(cosf(output->pitch_rad)) > 0.1f;
    if (first)
    {
        output->yaw_unwrapped_rad = output->yaw_rad;
        output->yaw_continuous = output->heading_valid;
    }
    else if (!output->heading_valid)
    {
        output->yaw_continuous = false;
    }
    else if (output->yaw_continuous)
    {
        float delta = output->yaw_rad - previous_yaw;
        if (delta > ATTITUDE_PI)
        {
            delta -= 2.0f * ATTITUDE_PI;
        }
        else if (delta < -ATTITUDE_PI)
        {
            delta += 2.0f * ATTITUDE_PI;
        }
        /* A sampled trajectory can jump across the pitch singularity without
         * landing in its exclusion band. Do not unwrap a large ambiguous jump. */
        if (fabsf(delta) >= ATTITUDE_PI * 0.5f)
        {
            output->yaw_continuous = false;
        }
        else
        {
            output->yaw_unwrapped_rad += (double)delta;
        }
    }
}

static void AlignFromGravity(Attitude_s *attitude)
{
    float *a = attitude->mean_accel;
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    float sr = sinf(roll * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float *q = attitude->output.quaternion;
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp; /* ZYX Euler yaw = 0. */
    memcpy(attitude->output.gyro_bias_rad_s, attitude->mean_gyro, sizeof(attitude->mean_gyro));
    attitude->output.state = ATTITUDE_TRACKING;
    attitude->output.valid = true;
    UpdateAngles(&attitude->output, true);
}

static bool Calibrate(Attitude_s *attitude, const float gyro[3], const float accel[3], bool allowed)
{
    const AttitudeConfig_s *config = &attitude->config;
    bool stationary = allowed && fabsf(Norm3(accel) - config->gravity_m_s2) <= config->accel_gate_m_s2;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        stationary = stationary && fabsf(gyro[i]) <= config->stationary_gyro_rad_s;
    }
    if (!stationary)
    {
        attitude->output.calibration_count = 0U;
        return false;
    }
    uint32_t count = attitude->output.calibration_count;
    if (count > 0U)
    {
        for (unsigned i = 0U; i < 3U; ++i)
        {
            attitude->min_gyro[i] = fminf(attitude->min_gyro[i], gyro[i]);
            attitude->max_gyro[i] = fmaxf(attitude->max_gyro[i], gyro[i]);
            attitude->min_accel[i] = fminf(attitude->min_accel[i], accel[i]);
            attitude->max_accel[i] = fmaxf(attitude->max_accel[i], accel[i]);
            if (attitude->max_gyro[i] - attitude->min_gyro[i] > config->stationary_gyro_span ||
                attitude->max_accel[i] - attitude->min_accel[i] > config->stationary_accel_span)
            {
                count = 0U;
                break;
            }
        }
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (count == 0U)
        {
            attitude->mean_gyro[i] = attitude->min_gyro[i] = attitude->max_gyro[i] = gyro[i];
            attitude->mean_accel[i] = attitude->min_accel[i] = attitude->max_accel[i] = accel[i];
        }
        else
        {
            attitude->mean_gyro[i] += (gyro[i] - attitude->mean_gyro[i]) / (float)(count + 1U);
            attitude->mean_accel[i] += (accel[i] - attitude->mean_accel[i]) / (float)(count + 1U);
        }
    }
    attitude->output.calibration_count = count + 1U;
    if (attitude->output.calibration_count >= config->calibration_samples)
    {
        AlignFromGravity(attitude);
        return true;
    }
    return false;
}

static bool Integrate(Attitude_s *attitude, const float gyro[3], const float accel[3], float dt)
{
    AttitudeOutput_s *output = &attitude->output;
    const float *q = output->quaternion;
    float omega[3];
    for (unsigned i = 0U; i < 3U; ++i)
    {
        omega[i] = gyro[i] - output->gyro_bias_rad_s[i];
    }
    float norm = Norm3(accel);
    output->accel_correction_used = attitude->config.correction_gain_s > 0.0f &&
                                    fabsf(norm - attitude->config.gravity_m_s2) <=
                                    attitude->config.accel_gate_m_s2;
    if (output->accel_correction_used)
    {
        const float up[3] = {2.0f * (q[1] * q[3] - q[0] * q[2]),
                            2.0f * (q[0] * q[1] + q[2] * q[3]),
                            1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])};
        float gain = attitude->config.correction_gain_s / norm;
        omega[0] += gain * (accel[1] * up[2] - accel[2] * up[1]);
        omega[1] += gain * (accel[2] * up[0] - accel[0] * up[2]);
        omega[2] += gain * (accel[0] * up[1] - accel[1] * up[0]);
    }
    /* Quaternion exponential gives exact constant-rate integration for this
     * step; gravity correction is evaluated once at the start of the step. */
    float rate = Norm3(omega);
    float half_angle = 0.5f * rate * dt;
    float scale = rate > 0.000001f ? sinf(half_angle) / rate : 0.5f * dt;
    float d[4] = {cosf(half_angle), omega[0] * scale, omega[1] * scale, omega[2] * scale};
    float next[4] =
    {
        q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
        q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
        q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
        q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0]
    };
    float qnorm = sqrtf(next[0] * next[0] + next[1] * next[1] + next[2] * next[2] + next[3] * next[3]);
    if (!isfinite(qnorm) || qnorm < 0.5f)
    {
        AttitudeInvalidate(attitude, ATTITUDE_FAULT_DATA);
        return false;
    }
    for (unsigned i = 0U; i < 4U; ++i)
    {
        output->quaternion[i] = next[i] / qnorm;
    }
    UpdateAngles(output, false);
    return true;
}

bool AttitudeUpdate(Attitude_s *attitude, const ImuSample_s *sample,
                    uint32_t now_ms, bool calibration_allowed)
{
    if (attitude->output.state == ATTITUDE_LOST || attitude->output.state == ATTITUDE_BAD_CONFIG)
    {
        return false;
    }
    if (sample == NULL || !InRange(sample->temperature_c, -50.0f, 150.0f))
    {
        AttitudeInvalidate(attitude, ATTITUDE_FAULT_DATA);
        return false;
    }
    if (now_ms - sample->timestamp_ms > attitude->config.stale_ms)
    {
        AttitudeInvalidate(attitude, ATTITUDE_FAULT_TIMING);
        return false;
    }
    float gyro[3];
    float accel[3];
    for (unsigned i = 0U; i < 3U; ++i)
    {
        int map = attitude->config.axis_map[i];
        unsigned source = (unsigned)(map < 0 ? -map : map) - 1U;
        float sign = map < 0 ? -1.0f : 1.0f;
        gyro[i] = sign * sample->gyro_rad_s[source];
        accel[i] = sign * sample->accel_m_s2[source];
        if (!InRange(gyro[i], -attitude->config.max_gyro_rad_s, attitude->config.max_gyro_rad_s) ||
            !InRange(accel[i], -250.0f, 250.0f))
        {
            AttitudeInvalidate(attitude, ATTITUDE_FAULT_DATA);
            return false;
        }
    }
    uint32_t elapsed = sample->timestamp_ms - attitude->last_sample_ms;
    if (attitude->have_sample && elapsed == 0U)
    {
        AttitudeCheckFresh(attitude, now_ms);
        return false; /* Never integrate a repeated sample. */
    }
    if (attitude->have_sample && elapsed > attitude->config.max_step_ms)
    {
        if (attitude->output.state == ATTITUDE_TRACKING)
        {
            AttitudeInvalidate(attitude, ATTITUDE_FAULT_TIMING);
            return false;
        }
        attitude->output.calibration_count = 0U;
    }
    attitude->have_sample = true;
    attitude->last_sample_ms = sample->timestamp_ms;
    attitude->output.sample_ms = sample->timestamp_ms;
    attitude->output.temperature_c = sample->temperature_c;
    memcpy(attitude->output.accel_m_s2, accel, sizeof(accel));
    bool accepted = attitude->output.state == ATTITUDE_CALIBRATING ?
                    Calibrate(attitude, gyro, accel, calibration_allowed) :
                    Integrate(attitude, gyro, accel, (float)elapsed * 0.001f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        attitude->output.gyro_rad_s[i] = gyro[i] - attitude->output.gyro_bias_rad_s[i];
    }
    return accepted;
}
