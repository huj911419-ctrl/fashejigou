#include "friction_profile_service.h"
#include "friction.h"
#include "bench_features.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    Friction_Telemetry_s telemetry;
    Friction_Tuning_s queued;
    Friction_Tune_Result_e admission;
    Friction_Tune_Result_e queue_result;
    uint32_t queued_base;
    uint16_t queued_sequence;
    unsigned queue_calls;
    unsigned admission_calls;
    uint8_t available;
    float limits[FRICTION_MOTOR_COUNT];
} FakeFriction_s;

static FakeFriction_s fake;

void FrictionGetTelemetry(Friction_Telemetry_s *telemetry)
{
    *telemetry = fake.telemetry;
}

Friction_Tune_Result_e FrictionProfileAdmission(void)
{
    ++fake.admission_calls;
    return fake.admission;
}

uint8_t FrictionTuningAvailable(void)
{
    return fake.available;
}

void FrictionGetCurrentLimits(float limits[FRICTION_MOTOR_COUNT])
{
    memcpy(limits, fake.limits, sizeof(fake.limits));
}

Friction_Tune_Result_e FrictionQueueProfile(const Friction_Tuning_s *tuning,
                                          uint16_t sequence, uint32_t base_revision)
{
    ++fake.queue_calls;
    fake.queued = *tuning;
    fake.queued_base = base_revision;
    fake.queued_sequence = sequence;
    return fake.queue_result;
}

static FrictionProfile_s Fixture(void)
{
    FrictionProfile_s profile = {.values = {300.0f}};
    for (unsigned wheel = 0U; wheel < FRICTION_MOTOR_COUNT; ++wheel)
    {
        unsigned base = 1U + wheel * 4U;
        profile.values[base] = 0.5f + (float)wheel;
        profile.values[base + 1U] = 0.125f + (float)wheel;
        profile.values[base + 2U] = 0.0625f + (float)wheel;
        profile.values[base + 3U] = 500.0f + (float)wheel * 100.0f;
    }
    return profile;
}

static void Publish(const FrictionProfile_s *profile)
{
    fake.telemetry.tuning.ramp_rpm_s = profile->values[0];
    for (unsigned wheel = 0U; wheel < FRICTION_MOTOR_COUNT; ++wheel)
    {
        unsigned base = 1U + wheel * 4U;
        fake.telemetry.tuning.motor[wheel] = (Friction_PID_Tuning_s){
            profile->values[base], profile->values[base + 1U],
            profile->values[base + 2U], profile->values[base + 3U]};
    }
}

static void Reset(FrictionProfileService_s *service)
{
    memset(&fake, 0, sizeof(fake));
    fake.admission = FRICTION_TUNE_QUEUED;
    fake.queue_result = FRICTION_TUNE_QUEUED;
    fake.available = 1U;
    fake.limits[0] = 1500.0f;
    fake.limits[1] = 1600.0f;
    fake.limits[2] = 1700.0f;
    fake.telemetry.tuning_revision = UINT32_C(0x12345678);
    FrictionProfile_s profile = Fixture();
    Publish(&profile);
    FrictionProfileServiceInit(service);
    FrictionProfileServicePoll(service, 100U);
}

static void Metadata(const FrictionProfile_s *profile, float args[6])
{
    uint32_t crc = FrictionProfileChecksum(profile);
    args[0] = FRICTION_PROFILE_SCHEMA;
    args[1] = FRICTION_PROFILE_VERSION;
    args[2] = FRICTION_PROFILE_UNITS;
    args[3] = FRICTION_PROFILE_FIELDS;
    args[4] = (float)(crc & UINT16_MAX);
    args[5] = (float)(crc >> 16);
}

static Vofa_Result_e Begin(FrictionProfileService_s *service, const FrictionProfile_s *profile,
                            uint16_t token)
{
    float args[6];
    Metadata(profile, args);
    return FrictionProfileServiceCall(service, token, PROFILE_OP_BEGIN, args, 6U);
}

static Vofa_Result_e Field(FrictionProfileService_s *service, uint16_t token,
                            unsigned index, float value)
{
    float args[] = {(float)token, (float)index, value};
    return FrictionProfileServiceCall(service, (uint16_t)(1000U + index), PROFILE_OP_FIELD, args, 3U);
}

static Vofa_Result_e Commit(FrictionProfileService_s *service, uint16_t token, uint16_t sequence)
{
    float args[] = {(float)token};
    return FrictionProfileServiceCall(service, sequence, PROFILE_OP_COMMIT, args, 1U);
}

static void StageAll(FrictionProfileService_s *service, const FrictionProfile_s *profile,
                      uint16_t token)
{
    assert(Begin(service, profile, token) == VOFA_OK);
    for (unsigned index = 0U; index < FRICTION_PROFILE_FIELDS; ++index)
    {
        assert(Field(service, token, index, profile->values[index]) == VOFA_OK);
    }
}

static float Get(FrictionProfileService_s *service, uint16_t id)
{
    float value = -123.0f;
    assert(FrictionProfileServiceGet(service, id, &value) == VOFA_OK);
    return value;
}

static void CheckQueuedTuning(const FrictionProfile_s *profile)
{
    assert(fake.queued.ramp_rpm_s == profile->values[0]);
    for (unsigned wheel = 0U; wheel < FRICTION_MOTOR_COUNT; ++wheel)
    {
        unsigned base = 1U + wheel * 4U;
        assert(fake.queued.motor[wheel].kp == profile->values[base]);
        assert(fake.queued.motor[wheel].ki == profile->values[base + 1U]);
        assert(fake.queued.motor[wheel].kd == profile->values[base + 2U]);
        assert(fake.queued.motor[wheel].current_limit == profile->values[base + 3U]);
    }
}

static void TestSnapshotFrozenAndExpiry(void)
{
    FrictionProfileService_s service;
    FrictionProfile_s original = Fixture();
    float value = 0.0f;
    Reset(&service);
    assert(!FrictionProfileServiceBusy(&service));
    assert(Get(&service, PROFILE_GET_STATE) == PROFILE_IDLE);
    assert(Get(&service, PROFILE_GET_STORAGE) == 0.0f); /* No physical storage adapter. */
    assert(FrictionProfileServiceGet(&service, PROFILE_GET_FIELD_BASE, &value) == VOFA_UNAVAILABLE);
    assert(FrictionProfileServiceGet(&service, 99U, &value) == VOFA_UNKNOWN);
    assert(FrictionProfileServiceGet(&service, PROFILE_GET_FIELD_BASE + FRICTION_PROFILE_FIELDS,
                                      &value) == VOFA_UNKNOWN);
    assert(FrictionProfileServiceCall(&service, 1U, 99U, NULL, 0U) == VOFA_UNKNOWN);
    assert(FrictionProfileServiceCall(&service, 1U, PROFILE_OP_SNAPSHOT, &value, 1U) == VOFA_RANGE);
    fake.available = 0U;
    assert(FrictionProfileServiceCall(&service, 1U, PROFILE_OP_SNAPSHOT, NULL, 0U) == VOFA_UNAVAILABLE);
    assert(FrictionProfileServiceGet(&service, PROFILE_GET_REV_LOW, &value) == VOFA_UNAVAILABLE);
    fake.available = 1U;
    assert(FrictionProfileServiceCall(&service, 1U, PROFILE_OP_SNAPSHOT, NULL, 0U) == VOFA_OK);
    assert(Get(&service, PROFILE_GET_SCHEMA) == FRICTION_PROFILE_SCHEMA);
    assert(Get(&service, PROFILE_GET_VERSION) == FRICTION_PROFILE_VERSION);
    assert(Get(&service, PROFILE_GET_UNITS) == FRICTION_PROFILE_UNITS);
    assert(Get(&service, PROFILE_GET_COUNT) == FRICTION_PROFILE_FIELDS);
    assert(Get(&service, PROFILE_GET_ACTIVE_MASK) == BENCH_FRICTION_MASK);
    assert(Get(&service, PROFILE_GET_CURRENT_F1) == 1500.0f);
    assert(Get(&service, PROFILE_GET_CURRENT_F2) == 1600.0f);
    assert(Get(&service, PROFILE_GET_CURRENT_F3) == 1700.0f);
    assert(Get(&service, PROFILE_GET_SNAPSHOT_REV_LOW) == 0x5678U);
    assert(Get(&service, PROFILE_GET_SNAPSHOT_REV_HIGH) == 0x1234U);
    uint32_t expected_crc = FrictionProfileChecksum(&original);
    assert(Get(&service, PROFILE_GET_CRC_LOW) == (float)(expected_crc & UINT16_MAX));
    assert(Get(&service, PROFILE_GET_CRC_HIGH) == (float)(expected_crc >> 16));

    FrictionProfile_s changed = original;
    for (unsigned index = 0U; index < FRICTION_PROFILE_FIELDS; ++index)
    {
        changed.values[index] += 1.0f;
    }
    Publish(&changed);
    fake.telemetry.tuning_revision = UINT32_C(0xABCDEF01);
    for (unsigned index = 0U; index < FRICTION_PROFILE_FIELDS; ++index)
    {
        assert(Get(&service, (uint16_t)(PROFILE_GET_FIELD_BASE + index)) == original.values[index]);
    }
    assert(Get(&service, PROFILE_GET_REV_LOW) == 0xEF01U);
    assert(Get(&service, PROFILE_GET_REV_HIGH) == 0xABCDU);
    assert(Get(&service, PROFILE_GET_SNAPSHOT_REV_LOW) == 0x5678U);
    FrictionProfileServicePoll(&service, 100U + PROFILE_LEASE_MS - 1U);
    assert(Get(&service, PROFILE_GET_FIELD_BASE) == original.values[0]);
    FrictionProfileServicePoll(&service, 100U + PROFILE_LEASE_MS);
    assert(FrictionProfileServiceGet(&service, PROFILE_GET_FIELD_BASE, &value) == VOFA_UNAVAILABLE);
    assert(FrictionProfileServiceGet(&service, PROFILE_GET_CRC_LOW, &value) == VOFA_UNAVAILABLE);
    assert(FrictionProfileServiceCall(&service, 2U, PROFILE_OP_SNAPSHOT, NULL, 0U) == VOFA_OK);
    assert(Get(&service, PROFILE_GET_FIELD_BASE) == changed.values[0]);
    assert(fake.queue_calls == 0U && fake.admission_calls == 0U);
}

static void TestCompleteStagingAndCommit(void)
{
    FrictionProfileService_s service;
    FrictionProfile_s profile = Fixture();
    Reset(&service);
    assert(Begin(&service, &profile, 100U) == VOFA_OK);
    assert(FrictionProfileServiceBusy(&service));
    assert(service.state == PROFILE_STAGING);
    assert(Begin(&service, &profile, 200U) == VOFA_BUSY);
    assert(Commit(&service, 100U, 400U) == VOFA_RANGE);
    assert(fake.queue_calls == 0U);
    /* Out of order and duplicate fields must not conceal a missing field. */
    for (unsigned index = FRICTION_PROFILE_FIELDS - 1U; index > 0U; --index)
    {
        assert(Field(&service, 100U, index, profile.values[index]) == VOFA_OK);
    }
    assert(Field(&service, 100U, 1U, profile.values[1]) == VOFA_OK);
    assert(Commit(&service, 100U, 401U) == VOFA_RANGE);
    assert(fake.queue_calls == 0U);
    assert(Field(&service, 100U, 0U, profile.values[0] + 1.0f) == VOFA_OK);
    assert(Commit(&service, 100U, 402U) == VOFA_RANGE); /* Complete but CRC differs. */
    assert(fake.queue_calls == 0U && service.state == PROFILE_STAGING);
    assert(Field(&service, 100U, 0U, profile.values[0]) == VOFA_OK);
    assert(Commit(&service, 100U, 403U) == VOFA_QUEUED);
    assert(fake.queue_calls == 1U && fake.queued_sequence == 403U);
    assert(fake.queued_base == UINT32_C(0x12345678));
    CheckQueuedTuning(&profile);
    assert(service.state == PROFILE_QUEUED && FrictionProfileServiceBusy(&service));
    assert(Commit(&service, 100U, 404U) == VOFA_BUSY);
    assert(Field(&service, 100U, 0U, 999.0f) == VOFA_BUSY);
    assert(fake.queue_calls == 1U);
    fake.telemetry.applied_sequence = 403U;
    FrictionProfileServicePoll(&service, 101U);
    assert(service.state == PROFILE_QUEUED); /* Sequence alone is not application. */
    ++fake.telemetry.tuning_revision;
    FrictionProfileServicePoll(&service, 102U);
    assert(service.state == PROFILE_APPLIED && !FrictionProfileServiceBusy(&service));
    assert(Begin(&service, &profile, 100U) == VOFA_BUSY); /* Old token cannot restart. */
    assert(Begin(&service, &profile, 101U) == VOFA_OK);
}

static void TestMalformedMetadataAndTokens(void)
{
    FrictionProfileService_s service;
    FrictionProfile_s profile = Fixture();
    float args[6];
    Reset(&service);
    for (unsigned field = 0U; field < 4U; ++field)
    {
        Metadata(&profile, args);
        args[field] += 1.0f;
        assert(FrictionProfileServiceCall(&service, 50U, PROFILE_OP_BEGIN, args, 6U) == VOFA_RANGE);
    }
    const float invalid_words[] = {-1.0f, 0.5f, 65536.0f, NAN, INFINITY};
    for (size_t index = 0U; index < sizeof(invalid_words) / sizeof(invalid_words[0]); ++index)
    {
        for (unsigned field = 4U; field < 6U; ++field)
        {
            Metadata(&profile, args);
            args[field] = invalid_words[index];
            assert(FrictionProfileServiceCall(&service, 50U, PROFILE_OP_BEGIN, args, 6U) == VOFA_RANGE);
        }
    }
    Metadata(&profile, args);
    assert(FrictionProfileServiceCall(&service, 50U, PROFILE_OP_BEGIN, args, 5U) == VOFA_RANGE);
    assert(FrictionProfileServiceCall(&service, 0U, PROFILE_OP_BEGIN, args, 6U) == VOFA_RANGE);
    assert(fake.admission_calls == 0U && service.state == PROFILE_IDLE);
    assert(Begin(&service, &profile, 50U) == VOFA_OK);
    float field_args[] = {51.0f, 0.0f, 300.0f};
    assert(FrictionProfileServiceCall(&service, 60U, PROFILE_OP_FIELD, field_args, 3U) == VOFA_RANGE);
    assert(Commit(&service, 51U, 61U) == VOFA_RANGE);
    assert(FrictionProfileServiceCall(&service, 62U, PROFILE_OP_ABORT, field_args, 1U) == VOFA_RANGE);
    field_args[0] = 50.0f;
    const float invalid_indices[] = {-1.0f, 0.5f, (float)FRICTION_PROFILE_FIELDS, NAN};
    for (size_t index = 0U; index < sizeof(invalid_indices) / sizeof(invalid_indices[0]); ++index)
    {
        field_args[1] = invalid_indices[index];
        assert(FrictionProfileServiceCall(&service, 63U, PROFILE_OP_FIELD, field_args, 3U) == VOFA_RANGE);
    }
    field_args[1] = 0.0f;
    field_args[2] = NAN;
    assert(FrictionProfileServiceCall(&service, 64U, PROFILE_OP_FIELD, field_args, 3U) == VOFA_RANGE);
    field_args[2] = INFINITY;
    assert(FrictionProfileServiceCall(&service, 64U, PROFILE_OP_FIELD, field_args, 3U) == VOFA_RANGE);
    assert(FrictionProfileServiceCall(&service, 64U, PROFILE_OP_FIELD, field_args, 2U) == VOFA_RANGE);
    assert(service.received_mask == 0U && fake.queue_calls == 0U);
}

static void TestAdmissionAndQueueRejections(void)
{
    FrictionProfileService_s service;
    FrictionProfile_s profile = Fixture();
    const Friction_Tune_Result_e rejected[] = {
        FRICTION_TUNE_BUSY, FRICTION_TUNE_RANGE, FRICTION_TUNE_UNAVAILABLE};
    const Vofa_Result_e mapped[] = {VOFA_BUSY, VOFA_RANGE, VOFA_UNAVAILABLE};
    for (unsigned index = 0U; index < 3U; ++index)
    {
        Reset(&service);
        fake.admission = rejected[index];
        assert(Begin(&service, &profile, 10U) == mapped[index]);
        assert(service.state == PROFILE_IDLE && !FrictionProfileServiceBusy(&service));
        assert(fake.queue_calls == 0U);

        Reset(&service);
        StageAll(&service, &profile, 10U);
        fake.queue_result = rejected[index];
        assert(Commit(&service, 10U, 200U) == mapped[index]);
        assert(service.state == PROFILE_STAGING && service.started_ms == 100U);
        fake.queue_result = FRICTION_TUNE_QUEUED;
        assert(Commit(&service, 10U, 201U) == VOFA_QUEUED);
        assert(fake.queue_calls == 2U && fake.queued_sequence == 201U);
    }
}

static void TestExpiryWrapAbortAndInvalidate(void)
{
    FrictionProfileService_s service;
    FrictionProfile_s profile = Fixture();
    uint32_t start = UINT32_MAX - 10000U;
    Reset(&service);
    FrictionProfileServicePoll(&service, start);
    StageAll(&service, &profile, 20U);
    assert(FrictionProfileServiceCall(&service, 21U, PROFILE_OP_SNAPSHOT, NULL, 0U) == VOFA_OK);
    FrictionProfileServicePoll(&service, start + PROFILE_LEASE_MS - 1U);
    assert(service.state == PROFILE_STAGING && service.snapshot_valid);
    assert(Get(&service, PROFILE_GET_FIELD_BASE) == profile.values[0]);
    assert(Field(&service, 20U, 0U, profile.values[0]) == VOFA_OK);
    FrictionProfileServicePoll(&service, start + PROFILE_LEASE_MS);
    assert(service.state == PROFILE_FAILED && !service.snapshot_valid);
    assert(!FrictionProfileServiceBusy(&service));
    assert(Commit(&service, 20U, 30U) == VOFA_BUSY);
    assert(fake.queue_calls == 0U);
    assert(Begin(&service, &profile, 20U) == VOFA_BUSY);
    assert(Begin(&service, &profile, 21U) == VOFA_OK);
    float token[] = {21.0f};
    assert(FrictionProfileServiceCall(&service, 31U, PROFILE_OP_ABORT, token, 1U) == VOFA_OK);
    assert(service.state == PROFILE_IDLE && !FrictionProfileServiceBusy(&service));
    assert(Commit(&service, 21U, 32U) == VOFA_BUSY);

    StageAll(&service, &profile, 22U);
    assert(FrictionProfileServiceCall(&service, 33U, PROFILE_OP_SNAPSHOT, NULL, 0U) == VOFA_OK);
    FrictionProfileServiceInvalidate(&service);
    assert(service.state == PROFILE_FAILED && !service.snapshot_valid);
    assert(Commit(&service, 22U, 34U) == VOFA_BUSY);
    StageAll(&service, &profile, 23U);
    assert(Commit(&service, 23U, 35U) == VOFA_QUEUED);
    FrictionProfileServiceInvalidate(&service);
    assert(service.state == PROFILE_QUEUED); /* RX fault cannot retract accepted work. */
    ++fake.telemetry.tuning_revision;
    fake.telemetry.applied_sequence = 35U;
    FrictionProfileServicePoll(&service, start + PROFILE_LEASE_MS + 1U);
    assert(service.state == PROFILE_APPLIED);
}

static void TestAppliedRevisionConflictsAndWrap(void)
{
    FrictionProfileService_s service;
    FrictionProfile_s profile = Fixture();
    Reset(&service);
    StageAll(&service, &profile, 1U);
    assert(Commit(&service, 1U, 90U) == VOFA_QUEUED);
    ++fake.telemetry.tuning_revision;
    fake.telemetry.applied_sequence = 91U;
    FrictionProfileServicePoll(&service, 101U);
    assert(service.state == PROFILE_FAILED); /* Somebody else applied parameters. */

    Reset(&service);
    StageAll(&service, &profile, 2U);
    assert(Commit(&service, 2U, 90U) == VOFA_QUEUED);
    fake.telemetry.tuning_revision += 2U;
    fake.telemetry.applied_sequence = 90U;
    FrictionProfileServicePoll(&service, 101U);
    assert(service.state == PROFILE_FAILED); /* Same 16-bit ACK does not prove lineage. */

    Reset(&service);
    fake.telemetry.tuning_revision = UINT32_MAX;
    StageAll(&service, &profile, 3U);
    assert(Commit(&service, 3U, UINT16_MAX) == VOFA_QUEUED);
    fake.telemetry.tuning_revision = 0U;
    fake.telemetry.applied_sequence = UINT16_MAX;
    FrictionProfileServicePoll(&service, 101U);
    assert(service.state == PROFILE_APPLIED);
}

int main(void)
{
    TestSnapshotFrozenAndExpiry();
    TestCompleteStagingAndCommit();
    TestMalformedMetadataAndTokens();
    TestAdmissionAndQueueRejections();
    TestExpiryWrapAbortAndInvalidate();
    TestAppliedRevisionConflictsAndWrap();
    puts("friction_profile_service: all tests passed");
    return 0;
}
