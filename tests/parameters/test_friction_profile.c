#include "friction_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    FrictionProfile_s profile = {.values = {300, 1, 2, 0, 1500, 1, 2, 0, 1500, 1, 2, 0, 1500}};
    FrictionProfileLimits_s limits = {100, 1000, 1, 2000, {1500, 1500, 1500}};
    uint8_t bytes[FRICTION_PROFILE_SIZE];
    assert(FrictionProfileValidate(&profile, &limits));
    assert(FrictionProfileEncode(&profile, bytes));
    assert(FrictionProfileChecksum(&profile) == 0x77ac2132U); /* Independent Python golden vector. */
    const uint8_t header[16] = {'F', 'R', 'P', '1', 1, 0, 1, 0, 1, 0, 13, 0, 0, 0, 0, 0};
    assert(memcmp(bytes, header, sizeof(header)) == 0);
    FrictionProfile_s decoded = {0};
    assert(FrictionProfileDecode(&decoded, bytes, sizeof(bytes)));
    assert(memcmp(profile.values, decoded.values, sizeof(profile.values)) == 0);
    for (unsigned i = 0U; i < sizeof(bytes); ++i)
    {
        uint8_t damaged[FRICTION_PROFILE_SIZE];
        memcpy(damaged, bytes, sizeof(bytes));
        damaged[i] ^= 1U;
        assert(!FrictionProfileDecode(&decoded, damaged, sizeof(damaged)));
        assert(memcmp(profile.values, decoded.values, sizeof(profile.values)) == 0);
    }
    assert(!FrictionProfileDecode(&decoded, bytes, sizeof(bytes) - 1U));
    for (unsigned i = 0U; i < FRICTION_PROFILE_FIELDS; ++i)
    {
        float saved = profile.values[i];
        profile.values[i] = NAN;
        assert(!FrictionProfileEncode(&profile, bytes));
        assert(!FrictionProfileValidate(&profile, &limits));
        profile.values[i] = saved;
    }
    profile.values[4] = 1501;
    assert(!FrictionProfileValidate(&profile, &limits));
    profile.values[4] = 1500;
    profile.values[0] = 0;
    assert(!FrictionProfileValidate(&profile, &limits));
    profile.values[0] = 300;
    limits.ki_max = INFINITY;
    assert(!FrictionProfileValidate(&profile, &limits));
    puts("PASS: canonical profile golden CRC, round trip, every-byte corruption, finite/range checks");
    return 0;
}
