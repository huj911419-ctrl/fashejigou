#ifndef GIMBAL_BENCH_H
#define GIMBAL_BENCH_H

#include "gimbal_control.h"
#include "gimbal_remote.h"
#include "control_authority.h"

/* The bench composes input and actuator snapshots; the actuator core itself
 * does not depend on a particular command source or its diagnostic fields. */
typedef struct
{
    GimbalControlStatus_s axis;
    GimbalRemoteStatus_s remote;
    ControlAuthorityStatus_s authority; /* Permission only; axis.enabled includes actuator checks. */
} GimbalStatus_s;

/** Initialize selected resources once before scheduler (bench_features.h).
 * Remote and motor can be selected separately. No remote means no valid run
 * source; the motor may perform its existing handshake but cannot auto-enable.
 * Both disabled returns success without starting either peripheral service. */
bool GimbalInit(uint32_t now_ms);
/** Single task owner, every GIMBAL_CONTROL_PERIOD_MS. Samples time after RX. */
void GimbalTask(void);
/** Coherent copied diagnostic snapshot; task context. Check axis.sample_ms age. */
void GimbalGetStatus(GimbalStatus_s *status);

#endif
