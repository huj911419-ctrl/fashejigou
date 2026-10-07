#ifndef GIMBAL_ARBITER_H
#define GIMBAL_ARBITER_H

#include "control_authority.h"
#include "gimbal_command.h"

typedef struct
{
    ControlAuthority_s authority;
    GimbalCommand_s candidate;
    float max_rate_rad_s;
    bool configured;
} GimbalArbiter_s;

/** Portable command adapter: no transport, actuator, HAL, RTOS or allocation.
 * One composition-task owner serializes all calls; concurrent producers must
 * use synchronized mailboxes BEFORE this adapter. Rates are motor-side rad/s.
 * Config is copied. The authority lease and actuator lease both remain active. */
bool GimbalArbiterInit(GimbalArbiter_s *arbiter, const ControlAuthorityConfig_s *config,
                       float max_rate_rad_s);
/** Explicit handoff. A changed source clears cached payload and advances epoch.
 * Producers must capture the epoch when activated; never re-tag queued old
 * commands with the current epoch. No source is selected automatically. */
bool GimbalArbiterSelect(GimbalArbiter_s *arbiter, ControlSource_t source);
/** Source/epoch/ordering checked before copying the payload. Invalid numeric
 * input from the selected source revokes permission; wrong owners cannot take
 * control. received_ms is original RX time, not queue-drain/polling time. */
bool GimbalArbiterSubmit(GimbalArbiter_s *arbiter, ControlSource_t source, uint32_t epoch,
                         const GimbalCommand_s *command);
/** Explicit global stop, independent of selected source. Sticky until Step. */
void GimbalArbiterStop(GimbalArbiter_s *arbiter);
/** Returns a copied request for GimbalControlStep. Rejected RUN is invalid or
 * carries stop_requested, never a fabricated valid OFF that could unlock yaw.
 * status.authorized only permits this request; actuator faults still override. */
GimbalCommand_s GimbalArbiterStep(GimbalArbiter_s *arbiter, uint32_t now_ms);

#endif
