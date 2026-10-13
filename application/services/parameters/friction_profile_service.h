#ifndef FRICTION_PROFILE_SERVICE_H
#define FRICTION_PROFILE_SERVICE_H

#include "friction_profile.h"
#include "vofa.h"

/* Application protocol IDs: all integer words fit exactly in binary32. */
enum
{
    PROFILE_OP_SNAPSHOT = 10,
    PROFILE_OP_BEGIN = 11,
    PROFILE_OP_FIELD = 12,
    PROFILE_OP_COMMIT = 13,
    PROFILE_OP_ABORT = 14,
    PROFILE_GET_SCHEMA = 100,
    PROFILE_GET_VERSION,
    PROFILE_GET_UNITS,
    PROFILE_GET_COUNT,
    PROFILE_GET_CRC_LOW,
    PROFILE_GET_CRC_HIGH,
    PROFILE_GET_SNAPSHOT_REV_LOW,
    PROFILE_GET_SNAPSHOT_REV_HIGH,
    PROFILE_GET_STATE,
    PROFILE_GET_STORAGE,
    PROFILE_GET_ACTIVE_MASK,
    PROFILE_GET_CURRENT_F1,
    PROFILE_GET_CURRENT_F2,
    PROFILE_GET_CURRENT_F3,
    PROFILE_GET_REV_LOW,
    PROFILE_GET_REV_HIGH,
    PROFILE_GET_FIELD_BASE = 200,
    PROFILE_LEASE_MS = 30000
};

typedef enum
{
    PROFILE_IDLE,
    PROFILE_STAGING,
    PROFILE_QUEUED,
    PROFILE_APPLIED,
    PROFILE_FAILED
} FrictionProfileState_e;

typedef struct
{
    FrictionProfile_s snapshot;
    FrictionProfile_s staged;
    uint32_t now_ms;
    uint32_t snapshot_ms;
    uint32_t started_ms;
    uint32_t snapshot_revision;
    uint32_t base_revision;
    uint32_t expected_crc;
    uint16_t token;
    uint16_t commit_sequence;
    uint16_t received_mask;
    bool snapshot_valid;
    FrictionProfileState_e state;
} FrictionProfileService_s;

/** One diagnostic-task owner per instance. Poll before parsing each RX batch.
 * BEGIN's command sequence is the transaction token; FIELD/COMMIT/ABORT carry
 * that token, each with a fresh command sequence. Exactly one request in flight.
 * Staging and frozen export each expire after 30 s (reads do not extend leases).
 * QUEUED is not APPLIED. This service never writes nonvolatile storage. */
void FrictionProfileServiceInit(FrictionProfileService_s *service);
void FrictionProfileServicePoll(FrictionProfileService_s *service, uint32_t now_ms);
bool FrictionProfileServiceBusy(const FrictionProfileService_s *service);
void FrictionProfileServiceInvalidate(FrictionProfileService_s *service);
Vofa_Result_e FrictionProfileServiceGet(FrictionProfileService_s *service, uint16_t id, float *value);
Vofa_Result_e FrictionProfileServiceCall(FrictionProfileService_s *service, uint16_t sequence,
                                        uint16_t operation, const float *args, size_t count);

#endif
