#ifndef FRICTION_PROFILE_H
#define FRICTION_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum
{
    FRICTION_PROFILE_SCHEMA = 1U,
    FRICTION_PROFILE_VERSION = 1U,
    FRICTION_PROFILE_UNITS = 1U,
    FRICTION_PROFILE_FIELDS = 13U,
    FRICTION_PROFILE_SIZE = 72U,
    FRICTION_PROFILE_CRC_OFFSET = 68U
};

typedef struct
{
    float values[FRICTION_PROFILE_FIELDS];
} FrictionProfile_s;

typedef struct
{
    float kp_max;
    float ki_max;
    float kd_max;
    float ramp_max;
    float current_max[3];
} FrictionProfileLimits_s;

/** Pure value codec, no transport, hardware, allocation or shared state.
 * Field order: ramp RPM/s, then F1/F2/F3 (Kp, Ki, Kd, current limit).
 * PID error is motor degrees/s, I/D time is seconds, current is C620 raw.
 * No commands, enable bits, motor IDs or controller dynamic state are stored.
 * Wire: FRP1, schema/version/units/count (u16 LE), zero reserved u32,
 * 13 binary32 LE fields, CRC32/ISO-HDLC over the preceding 68 bytes.
 * Encode/Decode require finite values; range checks use Validate separately.
 * Decode failure leaves the destination unchanged. Caller owns all buffers. */
bool FrictionProfileEncode(const FrictionProfile_s *profile, uint8_t bytes[FRICTION_PROFILE_SIZE]);
bool FrictionProfileDecode(FrictionProfile_s *profile, const uint8_t *bytes, size_t length);
bool FrictionProfileValidate(const FrictionProfile_s *profile, const FrictionProfileLimits_s *limits);
uint32_t FrictionProfileChecksum(const FrictionProfile_s *profile);

#endif
