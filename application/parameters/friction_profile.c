#include "friction_profile.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4U && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "Parameter profiles require IEEE754 binary32");

static void Put16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static uint16_t Get16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | (uint16_t)bytes[1] << 8);
}

static void Put32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i)
    {
        bytes[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint32_t Get32(const uint8_t *bytes)
{
    uint32_t value = 0U;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        value |= (uint32_t)bytes[i] << (8U * i);
    }
    return value;
}

static uint32_t Checksum(const uint8_t *bytes, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return ~crc;
}

bool FrictionProfileEncode(const FrictionProfile_s *profile, uint8_t bytes[FRICTION_PROFILE_SIZE])
{
    if (profile == NULL || bytes == NULL)
    {
        return false;
    }
    for (unsigned i = 0U; i < FRICTION_PROFILE_FIELDS; ++i)
    {
        if (!isfinite(profile->values[i]))
        {
            return false;
        }
    }
    memset(bytes, 0, FRICTION_PROFILE_SIZE);
    memcpy(bytes, "FRP1", 4U);
    Put16(bytes + 4U, FRICTION_PROFILE_SCHEMA);
    Put16(bytes + 6U, FRICTION_PROFILE_VERSION);
    Put16(bytes + 8U, FRICTION_PROFILE_UNITS);
    Put16(bytes + 10U, FRICTION_PROFILE_FIELDS);
    for (unsigned i = 0U; i < FRICTION_PROFILE_FIELDS; ++i)
    {
        uint32_t bits;
        memcpy(&bits, &profile->values[i], sizeof(bits));
        Put32(bytes + 16U + i * 4U, bits);
    }
    Put32(bytes + FRICTION_PROFILE_CRC_OFFSET, Checksum(bytes, FRICTION_PROFILE_CRC_OFFSET));
    return true;
}

bool FrictionProfileDecode(FrictionProfile_s *profile, const uint8_t *bytes, size_t length)
{
    if (profile == NULL || bytes == NULL || length != FRICTION_PROFILE_SIZE ||
        memcmp(bytes, "FRP1", 4U) != 0 || Get16(bytes + 4U) != FRICTION_PROFILE_SCHEMA ||
        Get16(bytes + 6U) != FRICTION_PROFILE_VERSION || Get16(bytes + 8U) != FRICTION_PROFILE_UNITS ||
        Get16(bytes + 10U) != FRICTION_PROFILE_FIELDS || Get32(bytes + 12U) != 0U ||
        Get32(bytes + FRICTION_PROFILE_CRC_OFFSET) != Checksum(bytes, FRICTION_PROFILE_CRC_OFFSET))
    {
        return false;
    }
    FrictionProfile_s decoded;
    for (unsigned i = 0U; i < FRICTION_PROFILE_FIELDS; ++i)
    {
        uint32_t bits = Get32(bytes + 16U + i * 4U);
        memcpy(&decoded.values[i], &bits, sizeof(bits));
        if (!isfinite(decoded.values[i]))
        {
            return false;
        }
    }
    *profile = decoded;
    return true;
}

static bool InRange(float value, float minimum, float maximum)
{
    return isfinite(value) && isfinite(maximum) && maximum >= minimum &&
           value >= minimum && value <= maximum;
}

bool FrictionProfileValidate(const FrictionProfile_s *profile, const FrictionProfileLimits_s *limits)
{
    if (profile == NULL || limits == NULL || !InRange(profile->values[0], 1.0f, limits->ramp_max))
    {
        return false;
    }
    for (unsigned wheel = 0U; wheel < 3U; ++wheel)
    {
        const float *fields = &profile->values[1U + wheel * 4U];
        if (!InRange(fields[0], 0.0f, limits->kp_max) || !InRange(fields[1], 0.0f, limits->ki_max) ||
            !InRange(fields[2], 0.0f, limits->kd_max) || !InRange(fields[3], 1.0f, limits->current_max[wheel]))
        {
            return false;
        }
    }
    return true;
}

uint32_t FrictionProfileChecksum(const FrictionProfile_s *profile)
{
    uint8_t bytes[FRICTION_PROFILE_SIZE];
    return FrictionProfileEncode(profile, bytes) ? Get32(bytes + FRICTION_PROFILE_CRC_OFFSET) : 0U;
}
