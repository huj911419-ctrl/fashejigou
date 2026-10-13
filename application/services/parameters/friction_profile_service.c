#include "friction_profile_service.h"
#include "../../bench_features.h"
#include "../../friction/friction.h"
#include <math.h>
#include <string.h>

static Vofa_Result_e TuneResult(Friction_Tune_Result_e result)
{
    switch (result)
    {
    case FRICTION_TUNE_QUEUED:
        return VOFA_QUEUED;
    case FRICTION_TUNE_RANGE:
        return VOFA_RANGE;
    case FRICTION_TUNE_BUSY:
        return VOFA_BUSY;
    default:
        return VOFA_UNAVAILABLE;
    }
}

static bool Integer(float value, unsigned maximum)
{
    return isfinite(value) && value >= 0.0f && value <= (float)maximum && value == floorf(value);
}

static FrictionProfile_s FromTuning(const Friction_Tuning_s *tuning)
{
    FrictionProfile_s profile = {.values = {tuning->ramp_rpm_s}};
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        unsigned base = 1U + i * 4U;
        profile.values[base] = tuning->motor[i].kp;
        profile.values[base + 1U] = tuning->motor[i].ki;
        profile.values[base + 2U] = tuning->motor[i].kd;
        profile.values[base + 3U] = tuning->motor[i].current_limit;
    }
    return profile;
}

static Friction_Tuning_s ToTuning(const FrictionProfile_s *profile)
{
    Friction_Tuning_s tuning = {.ramp_rpm_s = profile->values[0]};
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        unsigned base = 1U + i * 4U;
        tuning.motor[i] = (Friction_PID_Tuning_s){profile->values[base], profile->values[base + 1U],
            profile->values[base + 2U], profile->values[base + 3U]};
    }
    return tuning;
}

void FrictionProfileServiceInit(FrictionProfileService_s *service)
{
    memset(service, 0, sizeof(*service));
}

bool FrictionProfileServiceBusy(const FrictionProfileService_s *service)
{
    return BENCH_ENABLE_PARAMETERS &&
        (service->state == PROFILE_STAGING || service->state == PROFILE_QUEUED);
}

void FrictionProfileServiceInvalidate(FrictionProfileService_s *service)
{
    if (service->state == PROFILE_STAGING)
    {
        service->state = PROFILE_FAILED;
    }
    service->snapshot_valid = false;
}

void FrictionProfileServicePoll(FrictionProfileService_s *service, uint32_t now_ms)
{
    service->now_ms = now_ms;
    if (service->snapshot_valid && now_ms - service->snapshot_ms >= PROFILE_LEASE_MS)
    {
        service->snapshot_valid = false;
    }
    if (service->state == PROFILE_STAGING && now_ms - service->started_ms >= PROFILE_LEASE_MS)
    {
        service->state = PROFILE_FAILED;
    }
    if (service->state == PROFILE_QUEUED)
    {
        Friction_Telemetry_s telemetry;
        FrictionGetTelemetry(&telemetry);
        if (telemetry.tuning_revision != service->base_revision)
        {
            bool applied = telemetry.tuning_revision == service->base_revision + 1U &&
                telemetry.applied_sequence == service->commit_sequence;
            service->state = applied ? PROFILE_APPLIED : PROFILE_FAILED;
        }
    }
}

Vofa_Result_e FrictionProfileServiceGet(FrictionProfileService_s *service, uint16_t id, float *value)
{
    bool field = id >= PROFILE_GET_FIELD_BASE && id < PROFILE_GET_FIELD_BASE + FRICTION_PROFILE_FIELDS;
    if (!field && (id < PROFILE_GET_SCHEMA || id > PROFILE_GET_REV_HIGH))
    {
        return VOFA_UNKNOWN;
    }
    if (!BENCH_ENABLE_PARAMETERS)
    {
        return VOFA_UNAVAILABLE;
    }
    if (id == PROFILE_GET_STATE || id == PROFILE_GET_STORAGE)
    {
        *value = id == PROFILE_GET_STATE ? (float)service->state : 0.0f;
        return VOFA_OK;
    }
    if (id == PROFILE_GET_REV_LOW || id == PROFILE_GET_REV_HIGH)
    {
        Friction_Telemetry_s telemetry;
        FrictionGetTelemetry(&telemetry);
        *value = (float)(id == PROFILE_GET_REV_LOW ? telemetry.tuning_revision & 0xffffU :
                        telemetry.tuning_revision >> 16);
        return FrictionTuningAvailable() ? VOFA_OK : VOFA_UNAVAILABLE;
    }
    if (!service->snapshot_valid)
    {
        return VOFA_UNAVAILABLE;
    }
    if (field)
    {
        *value = service->snapshot.values[id - PROFILE_GET_FIELD_BASE];
        return VOFA_OK;
    }
    uint32_t crc = FrictionProfileChecksum(&service->snapshot);
    float current_limits[FRICTION_MOTOR_COUNT];
    FrictionGetCurrentLimits(current_limits);
    switch (id)
    {
    case PROFILE_GET_SCHEMA: *value = FRICTION_PROFILE_SCHEMA; break;
    case PROFILE_GET_VERSION: *value = FRICTION_PROFILE_VERSION; break;
    case PROFILE_GET_UNITS: *value = FRICTION_PROFILE_UNITS; break;
    case PROFILE_GET_COUNT: *value = FRICTION_PROFILE_FIELDS; break;
    case PROFILE_GET_CRC_LOW: *value = (float)(crc & 0xffffU); break;
    case PROFILE_GET_CRC_HIGH: *value = (float)(crc >> 16); break;
    case PROFILE_GET_SNAPSHOT_REV_LOW: *value = (float)(service->snapshot_revision & 0xffffU); break;
    case PROFILE_GET_SNAPSHOT_REV_HIGH: *value = (float)(service->snapshot_revision >> 16); break;
    case PROFILE_GET_ACTIVE_MASK: *value = BENCH_FRICTION_MASK; break;
    default: *value = current_limits[id - PROFILE_GET_CURRENT_F1]; break;
    }
    return VOFA_OK;
}

static Vofa_Result_e Begin(FrictionProfileService_s *service, uint16_t sequence,
                           const float *args, size_t count)
{
    if (count != 6U || args[0] != FRICTION_PROFILE_SCHEMA || args[1] != FRICTION_PROFILE_VERSION ||
        args[2] != FRICTION_PROFILE_UNITS || args[3] != FRICTION_PROFILE_FIELDS ||
        !Integer(args[4], UINT16_MAX) || !Integer(args[5], UINT16_MAX) || sequence == 0U)
    {
        return VOFA_RANGE;
    }
    if (FrictionProfileServiceBusy(service) || sequence == service->token)
    {
        return VOFA_BUSY;
    }
    Friction_Tune_Result_e result = FrictionProfileAdmission();
    if (result != FRICTION_TUNE_QUEUED)
    {
        return TuneResult(result);
    }
    Friction_Telemetry_s telemetry;
    FrictionGetTelemetry(&telemetry);
    service->base_revision = telemetry.tuning_revision;
    service->expected_crc = (uint32_t)args[4] | (uint32_t)args[5] << 16;
    service->token = sequence;
    service->received_mask = 0U;
    service->started_ms = service->now_ms;
    service->state = PROFILE_STAGING;
    memset(&service->staged, 0, sizeof(service->staged));
    return VOFA_OK;
}

Vofa_Result_e FrictionProfileServiceCall(FrictionProfileService_s *service, uint16_t sequence,
                                        uint16_t operation, const float *args, size_t count)
{
    if (operation < PROFILE_OP_SNAPSHOT || operation > PROFILE_OP_ABORT)
    {
        return VOFA_UNKNOWN;
    }
    if (!BENCH_ENABLE_PARAMETERS)
    {
        return VOFA_UNAVAILABLE;
    }
    if (operation == PROFILE_OP_SNAPSHOT)
    {
        if (count != 0U)
        {
            return VOFA_RANGE;
        }
        if (!FrictionTuningAvailable())
        {
            return VOFA_UNAVAILABLE;
        }
        Friction_Telemetry_s telemetry;
        FrictionGetTelemetry(&telemetry);
        service->snapshot = FromTuning(&telemetry.tuning);
        service->snapshot_revision = telemetry.tuning_revision;
        service->snapshot_ms = service->now_ms;
        service->snapshot_valid = true;
        return VOFA_OK;
    }
    if (operation == PROFILE_OP_BEGIN)
    {
        return Begin(service, sequence, args, count);
    }
    size_t expected_count = operation == PROFILE_OP_FIELD ? 3U : 1U;
    if (count != expected_count || !Integer(args[0], UINT16_MAX) || args[0] != service->token)
    {
        return VOFA_RANGE;
    }
    if (service->state != PROFILE_STAGING)
    {
        return VOFA_BUSY;
    }
    if (operation == PROFILE_OP_ABORT)
    {
        service->state = PROFILE_IDLE;
        return VOFA_OK;
    }
    if (operation == PROFILE_OP_FIELD)
    {
        if (!Integer(args[1], FRICTION_PROFILE_FIELDS - 1U) || !isfinite(args[2]))
        {
            return VOFA_RANGE;
        }
        unsigned index = (unsigned)args[1];
        service->staged.values[index] = args[2];
        service->received_mask |= (uint16_t)(1U << index);
        return VOFA_OK;
    }
    if (service->received_mask != (1U << FRICTION_PROFILE_FIELDS) - 1U ||
        FrictionProfileChecksum(&service->staged) != service->expected_crc)
    {
        return VOFA_RANGE; /* Still staging: correct fields or abort within the original lease. */
    }
    Friction_Tuning_s tuning = ToTuning(&service->staged);
    Vofa_Result_e result = TuneResult(FrictionQueueProfile(&tuning, sequence, service->base_revision));
    if (result == VOFA_QUEUED)
    {
        service->commit_sequence = sequence;
        service->state = PROFILE_QUEUED;
    }
    return result;
}
