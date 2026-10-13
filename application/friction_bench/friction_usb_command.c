#include "../bench_features.h"
#include "friction_usb_command.h"
#include "friction.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint16_t sequence;

static uint8_t IsSeparator(char byte)
{
    return byte == ' ' || byte == '\t';
}

/* Every argument must start after a separator, not at a sign or decimal
 * point left over from the previous conversion (for example 1+500+0+0). */
static const char *ArgumentStart(const char *cursor)
{
    if (cursor == NULL || !IsSeparator(*cursor))
    {
        return NULL;
    }
    while (IsSeparator(*cursor))
    {
        ++cursor;
    }
    return cursor;
}

static const char *ParseFloat(const char *cursor, float *value)
{
    char *end;
    cursor = ArgumentStart(cursor);
    if (cursor == NULL)
    {
        return NULL;
    }
    errno = 0;
    *value = strtof(cursor, &end);
    if (end == cursor || errno == ERANGE || !isfinite(*value) ||
        (*end != '\0' && !IsSeparator(*end)))
    {
        return NULL;
    }
    return end;
}

static const char *ParseUnsigned(const char *cursor, unsigned *value)
{
    cursor = ArgumentStart(cursor);
    if (cursor == NULL || *cursor < '0' || *cursor > '9')
    {
        return NULL;
    }
    /* strtoul accepts a leading minus and wraps in unsigned long. Parse a
     * bounded decimal explicitly so host and 32-bit MCU accept the same IDs. */
    uint32_t parsed = 0U;
    while (*cursor >= '0' && *cursor <= '9')
    {
        uint32_t digit = (uint32_t)(*cursor - '0');
        if (parsed > (UINT16_MAX - digit) / 10U)
        {
            return NULL;
        }
        parsed = parsed * 10U + digit;
        ++cursor;
    }
    if (*cursor != '\0' && !IsSeparator(*cursor))
    {
        return NULL;
    }
    *value = (unsigned)parsed;
    return cursor;
}

static uint8_t EndOfLine(const char *cursor)
{
    while (cursor != NULL && (*cursor == ' ' || *cursor == '\t'))
    {
        ++cursor;
    }
    return cursor != NULL && *cursor == '\0';
}

static FrictionUsbResult_e SubmitResult(Friction_Submit_Result_e result)
{
    switch (result)
    {
        case FRICTION_SUBMIT_ACCEPTED: return FRICTION_USB_QUEUED;
        case FRICTION_SUBMIT_WRONG_SOURCE: return FRICTION_USB_WRONG_SOURCE;
        case FRICTION_SUBMIT_BUSY: return FRICTION_USB_BUSY;
        case FRICTION_SUBMIT_STALE: return FRICTION_USB_STALE;
        case FRICTION_SUBMIT_REARM_REQUIRED: return FRICTION_USB_REARM_REQUIRED;
        default: return FRICTION_USB_UNAVAILABLE;
    }
}

static FrictionUsbResult_e QueueTuning(const Friction_Tuning_s *tuning, uint16_t base)
{
    sequence = sequence == UINT16_MAX ? 1U : (uint16_t)(sequence + 1U);
    switch (FrictionQueueTuning(tuning, sequence, base))
    {
        case FRICTION_TUNE_QUEUED: return FRICTION_USB_QUEUED;
        case FRICTION_TUNE_RANGE: return FRICTION_USB_RANGE;
        case FRICTION_TUNE_BUSY: return FRICTION_USB_BUSY;
        default: return FRICTION_USB_UNAVAILABLE;
    }
}

FrictionUsbResult_e FrictionUsbExecute(const char *line, uint32_t now_ms)
{
    if (!BENCH_ENABLE_USB || line == NULL)
    {
        return FRICTION_USB_UNAVAILABLE;
    }
    if (strcmp(line, "off") == 0)
    {
        FrictionStopFromUsb();
        return FRICTION_USB_QUEUED;
    }
    if (strncmp(line, "set", 3U) == 0 && (line[3] == ' ' || line[3] == '\t'))
    {
        unsigned mask;
        Friction_Command_s command;
        const char *cursor = ParseUnsigned(line + 3U, &mask);
        if (cursor == NULL || mask > FRICTION_ALL_MOTORS)
        {
            return FRICTION_USB_SYNTAX;
        }
        command.enable_mask = (uint8_t)mask;
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            cursor = ParseFloat(cursor, &command.target_rpm[i]);
            if (cursor == NULL)
            {
                return FRICTION_USB_SYNTAX;
            }
        }
        if (!EndOfLine(cursor))
        {
            return FRICTION_USB_SYNTAX;
        }
        return SubmitResult(FrictionSubmitFrom(FRICTION_SOURCE_USB, &command, now_ms));
    }

    Friction_Telemetry_s telemetry;
    Friction_Tuning_s tuning;
    if (strncmp(line, "ramp", 4U) == 0 && (line[4] == ' ' || line[4] == '\t'))
    {
        float value;
        const char *cursor = ParseFloat(line + 4U, &value);
        if (cursor == NULL || !EndOfLine(cursor))
        {
            return FRICTION_USB_SYNTAX;
        }
        FrictionGetTelemetry(&telemetry);
        tuning = telemetry.tuning;
        tuning.ramp_rpm_s = value;
        return QueueTuning(&tuning, telemetry.applied_sequence);
    }
    if (strncmp(line, "pid", 3U) == 0 && (line[3] == ' ' || line[3] == '\t'))
    {
        unsigned wheel;
        const char *cursor = ParseUnsigned(line + 3U, &wheel);
        if (cursor == NULL || wheel < 1U || wheel > FRICTION_MOTOR_COUNT)
        {
            return FRICTION_USB_SYNTAX;
        }
        FrictionGetTelemetry(&telemetry);
        tuning = telemetry.tuning;
        Friction_PID_Tuning_s *pid = &tuning.motor[wheel - 1U];
        cursor = ParseFloat(cursor, &pid->kp);
        cursor = ParseFloat(cursor, &pid->ki);
        cursor = ParseFloat(cursor, &pid->kd);
        cursor = ParseFloat(cursor, &pid->current_limit);
        if (!EndOfLine(cursor))
        {
            return FRICTION_USB_SYNTAX;
        }
        return QueueTuning(&tuning, telemetry.applied_sequence);
    }
    return FRICTION_USB_SYNTAX;
}

uint8_t FrictionSubmitFromUsbCommand(const char *line, uint32_t now_ms)
{
    return FrictionUsbExecute(line, now_ms) == FRICTION_USB_QUEUED;
}

const char *FrictionUsbResultName(FrictionUsbResult_e result)
{
    static const char *const names[] = {
        "QUEUED", "SYNTAX", "RANGE", "WRONG_SOURCE", "REARM_REQUIRED", "BUSY", "STALE", "UNAVAILABLE"
    };
    return (unsigned)result < sizeof(names) / sizeof(names[0]) ? names[result] : "UNKNOWN";
}
