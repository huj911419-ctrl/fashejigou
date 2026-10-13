#ifndef FRICTION_REMOTE_BENCH_H
#define FRICTION_REMOTE_BENCH_H

#include "friction_remote.h"
#include "friction_remote_port.h"
#include "../friction.h"

typedef struct
{
    FrictionRemoteStatus_s remote;
    FrictionRcPortStats_s port;
    uint32_t sample_ms;
    uint32_t accepted_frames;
    uint32_t gear_changed_ms; /* Original receive time of latest selection change. */
    uint32_t gear_changes;
    uint8_t previous_gear;    /* Latest transition's old selection, 0=STOP. */
    Friction_Submit_Result_e last_submit;
    bool initialized;
} FrictionRemoteBenchStatus_s;

/** Friction-only composition. Init before tasks; Poll from sole motor task,
 * before FrictionTask. Never drives motors directly. Disabled switch is inert.
 * RX initialization never changes authority; active remote is the boot default.
 * Remote never steals an existing USB owner. USB off releases other owners:
 * fresh physical S1 STOP can reclaim NONE, but cannot reclaim USB.
 * GetStatus is a coherent task-context observer; it grants no control authority. */
void FrictionRemoteBenchInit(void);
void FrictionRemoteBenchPoll(uint32_t now_ms);
void FrictionRemoteBenchGetStatus(FrictionRemoteBenchStatus_s *status);

#endif
